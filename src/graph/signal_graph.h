#pragma once

#include "graph/input_stage.h"
#include "graph/safety/output_stage.h"
#include "delay/delay_engine.h"
#include "graph/node_bus.h"
#include "circuit/block_state.h"
#include "circuit/rail.h"
#include "parts/clock.h"
#include "parts/oscillator.h"
#include "parts/hum.h"
#include "parts/contact_mic.h"
#include "parts/preamp.h"
#include "parts/filter.h"
#include "parts/mixer.h"
#include "bends/amount_bus.h"
#include "bends/mod_matrix.h"
#include "bends/live_wire.h"
#include <algorithm>

namespace bends {

struct CapacitiveBridge {
    float val{0.0f};

    __attribute__((always_inline)) inline float Tick(float target, float alpha_charge, float alpha_decay) {
        if (__builtin_expect(val == 0.0f && target == 0.0f, 1)) return 0.0f;
        float diff = target - val;
        if (diff > 0.0f) {
            val += diff * alpha_charge;
        } else {
            val += diff * alpha_decay;
            if (val < 0.0002f) val = 0.0f;
        }
        return val;
    }

    void Reset() { val = 0.0f; }
};

struct RenderProfile {
    uint32_t clear{0};
    uint32_t circuit{0};
    uint32_t prep{0};
    uint32_t inj{0};
    uint32_t clock{0};
    uint32_t osc{0};
    uint32_t preamp{0};
    uint32_t filter{0};
    uint32_t delay{0};
    uint32_t mixer{0};
    uint32_t output{0};
    uint32_t total{0};
};

class SignalGraph {
public:
    RenderProfile last_profile;

    void Init(float sample_rate, Stereo* sdram_delay_buf) {
        _sample_rate = sample_rate;
        _input.Init(sample_rate);
        _output.Init(sample_rate);
        _delay.Init(sample_rate, sdram_delay_buf);

        _clock.Init(sample_rate);
        _osc.Init(sample_rate);
        _hum.Init(sample_rate);
        _contact_mic.Init(sample_rate);
        _preamp.Init(sample_rate);
        _filter.Init(sample_rate);
        _live_wire.Init(sample_rate);

        _cap_osc_amp.Reset();
        _cap_osc_flt.Reset();
        _cap_flt_amp.Reset();
        _cap_fbk_amp.Reset();
        _cap_out_amp.Reset();
        _dly_in_hp = {0.0f, 0.0f};
        _flt_osc_dc = 0.0f;
        _audio_envelope = 0.0f;
        _in_env = 0.0f;
        _out_env = 0.0f;
        _dly_env = 0.0f;
        _cur_flt_cutoff = 0.0f;
        _cur_dly_clk_fm = 0.0f;
        _prev_tap_count_p10 = 0;
        _prev_midi_trigger_cnt = 0;
    }

    void Reset() {
        _input.Init(_sample_rate);
        _output.Reset();
        _delay.Clear();
        _preamp.Reset();
        _filter.Reset();
        _live_wire.Reset();
        _cap_osc_amp.Reset();
        _cap_osc_flt.Reset();
        _cap_flt_amp.Reset();
        _cap_fbk_amp.Reset();
        _cap_out_amp.Reset();
        _dly_in_hp = {0.0f, 0.0f};
        _flt_osc_dc = 0.0f;
        _cur_flt_cutoff = 0.0f;
        _cur_dly_clk_fm = 0.0f;
        _in_env = 0.0f;
        _out_env = 0.0f;
        _dly_env = 0.0f;
        _prev_tap_count_p10 = 0;
        _prev_midi_trigger_cnt = 0;
    }

    inline void ServiceIncrementalClear(size_t chunk_size = 1024) {
        _delay.ServiceIncrementalClear(chunk_size);
    }

    void PrepareBlock(BlockState& state, const AmountBus& amounts, const ControlFrame& frame) {
        // Submodule block-rate preparation
        float overclock = amounts.Get(Slot::ClkOverclock);
        if (state.topology.clock_siren) {
            overclock = std::max(overclock, 0.40f);
        }

        // MIDI pitch tracking
        float f_midi = frame.midi_osc_freq;
        bool has_midi_pitch = (f_midi > 0.0f);

        // Fader tuning offsets (0.50 center = unity)
        float osc_tune_mult = FastPow2((state.s36_osc - 0.50f) * 4.0f);
        float dly_tune_mult = FastPow2((state.s34_dly - 0.50f) * 2.0f);
        float flt_tune_mult = FastPow2((state.s31_flt - 0.50f) * 4.0f);

        // Delay clock (S33 CLK / external MIDI clock)
        float brake_factor = std::clamp(state.delay_regs.clock.rate * std::abs(state.delay_regs.speed.speed), 0.05f, 2.0f);
        float clk_rate_in = frame.midi_clk_rate;
        if (clk_rate_in >= 0.0f) {
            _clock.PrepareBlockHz(clk_rate_in * (1.0f + overclock * 0.40f) * brake_factor);
        } else {
            _clock.PrepareBlock(state.s33_clk, state.pull[NodeClk], overclock);
            if (brake_factor < 0.999f) {
                _clock.PrepareBlockHz(_clock.Rate() * brake_factor);
            }
        }
        state.delay_regs.clock.rate *= (1.0f + overclock * 0.50f);

        if (state.topology.lissajous_cloud) {
            state.delay_regs.feedback.crossfeed = std::max(state.delay_regs.feedback.crossfeed, 0.70f);
            state.delay_regs.time.stereo_split = std::max(state.delay_regs.time.stereo_split, 0.75f);
            state.delay_regs.feedback.gain = std::max(state.delay_regs.feedback.gain, 0.88f);
        }

        // Karplus-Strong string tuning when DLY/FBK bend is active
        float override_delay_samples = 0.0f;
        bool dly_fbk_bend_active = state.touch_dly_fbk || state.topology.karplus_resonator;

        if (has_midi_pitch && f_midi > 20.0f && dly_fbk_bend_active) {
            float target_f = std::clamp(f_midi * dly_tune_mult, 20.0f, 2000.0f);
            override_delay_samples = _sample_rate / target_f;
            state.delay_regs.feedback.gain = std::max(state.delay_regs.feedback.gain, 0.985f + frame.midi_aftertouch * 0.12f);
            state.delay_regs.feedback.damp = std::clamp(state.s31_flt * 0.85f, 0.10f, 0.90f);
        } else if (state.topology.karplus_resonator) {
            state.delay_regs.feedback.gain = std::max(state.delay_regs.feedback.gain, 0.985f);
            state.delay_regs.feedback.damp = std::clamp(state.s31_flt * 0.85f, 0.10f, 0.90f);
        }

        // Oscillator tuning
        bool osc_plays_midi = has_midi_pitch && state.touch_osc;
        if (osc_plays_midi) {
            _osc.PrepareBlockFreq(f_midi * osc_tune_mult);
        } else {
            // Proximity antenna sway
            float osc_fader = state.s36_osc;
            if (state.pressures[NodeOsc] <= 0.001f && frame.proximities[NodeOsc] > 0.001f) {
                osc_fader = std::clamp(osc_fader + frame.proximities[NodeOsc] * 0.005f, 0.0f, 1.0f);
            }
            _osc.PrepareBlock(osc_fader);
        }

        // Resonant filter tuning
        FilterMods fmods{};
        fmods.resonance = amounts.Get(Slot::FltRes);
        fmods.howl = state.retention[NodeFlt] * 0.5f;

        // Rungler triad coupling (CLK + OSC + FLT in Rnd mode)
        if (state.topology.rungler_triad && frame.osc_mode == Oscillator::ModeRnd) {
            fmods.resonance = std::max(fmods.resonance, 0.95f);
            float rungler_clk = _osc.RndHeld() * 0.55f;
            state.delay_regs.clock.rate *= std::clamp(1.0f + rungler_clk, 0.30f, 2.50f);
        }

        // Resonant ping boost under chord topologies
        bool is_drum_resonant = state.topology.ping_drum_triad || state.topology.voice_quartet ||
                                state.topology.clk_amp_flt ||
                                (state.g_osc_flt > 0.01f && state.g_osc_amp > 0.01f) ||
                                (state.g_flt_amp > 0.01f && state.g_osc_amp > 0.01f) ||
                                (state.g_flt_amp > 0.01f && state.g_osc_flt > 0.01f) ||
                                (state.g_clk_flt > 0.01f && state.g_flt_amp > 0.01f);
        if (is_drum_resonant) {
            float flt_coupling = std::max(state.g_osc_flt, state.g_flt_amp);
            if (flt_coupling > 0.005f) {
                float blend = std::clamp(flt_coupling * 1.5f, 0.0f, 1.0f);
                float target_res = 0.76f;
                if (target_res > fmods.resonance) {
                    fmods.resonance += (target_res - fmods.resonance) * blend;
                }
            }
        }

        float override_flt_cutoff = 0.0f;
        if (state.touch_flt && has_midi_pitch) {
            override_flt_cutoff = f_midi * flt_tune_mult;
            fmods.resonance = std::max(fmods.resonance, 0.92f);
        }

        // Proximity cutoff modulation
        float eff_flt_knob = state.s31_flt;
        if (state.pressures[NodeFlt] <= 0.001f && frame.proximities[NodeFlt] > 0.001f) {
            eff_flt_knob = std::clamp(eff_flt_knob + frame.proximities[NodeFlt] * 0.04f, 0.0f, 1.0f);
        }

        float flt_touch_res = std::max(std::max(state.pressures[NodeFlt], state.g_flt_amp * 0.65f), state.g_osc_flt * 0.75f);
        _filter.PrepareBlock(eff_flt_knob, state.pull[NodeFlt], flt_touch_res,
                             state.s30_amp, state.vcc, state.starve, state.s33_clk, fmods, override_flt_cutoff);

        // Resonant filter impulse trigger
        bool fresh_strike = (frame.midi_trigger_cnt != _prev_midi_trigger_cnt);
        if (fresh_strike) {
            _prev_midi_trigger_cnt = frame.midi_trigger_cnt;
        }

        bool is_pong_active = false;
        if (amounts.Get(Slot::FltPing) > 0.001f) {
            is_pong_active = true;
        } else if (state.topology.karplus_resonator || dly_fbk_bend_active) {
            is_pong_active = true;
        }

        if (fresh_strike && frame.midi_gate > 0.0f && is_pong_active) {
            float vel_ping = std::clamp(frame.midi_velocity * 1.35f, 0.35f, 1.35f);
            _filter.TriggerPing(vel_ping);
        }

        float eff_dly_knob = state.s34_dly;
        float amt_dly_head_mod = amounts.Get(Slot::DlyHeadMod);
        bool instant_dly = false;
        float base_samp = std::max(tuning::kMinDelaySamples, _delay.BaseSamples());

        // Proportional span scaling for Karplus comb filter (< 50ms):
        // Preserves musical pitch interval (semitones) and prevents clipping at minimum delay
        float karplus_span_scale = 1.0f;
        if (base_samp < tuning::kKarplusMaxSamples) {
            karplus_span_scale = std::clamp((base_samp - tuning::kMinDelaySamples) * (1.0f / tuning::kKarplusMaxSamples), 0.04f, 1.0f);
        }
        _cached_clk_vibrato_span = 260.0f * karplus_span_scale;

        if (amt_dly_head_mod > 0.015f) {

            if (frame.osc_mode == Oscillator::ModeRnd) {
                _prev_dly_hop_active = true;
                // S&H delay sequencer: random steps trigger jumps to delay taps
                uint8_t cur_step = _osc.RndStep();
                if (cur_step != _prev_rnd_step) {
                    _prev_rnd_step = cur_step;
                    static constexpr float kTapFractions[16] = {
                        0.0625f, 0.25f, 0.50f, 0.125f, 0.75f, 0.375f, 1.00f, 0.1875f,
                        0.50f, 0.875f, 0.3125f, 0.625f, 0.25f, 0.6875f, 0.4375f, 0.8125f
                    };
                    float tap_frac = kTapFractions[cur_step & 0x0F];
                    float tap_offset = std::clamp(base_samp * (0.25f + tap_frac * 1.50f * amt_dly_head_mod),
                                                  tuning::kMinDelaySamples,
                                                  static_cast<float>(tuning::kDelayFrames - 4800));
                    state.delay_regs.time.RequestJump(tap_offset, 64);
                }
            } else {
                if (_prev_dly_hop_active) {
                    _prev_dly_hop_active = false;
                    _prev_rnd_step = 255;
                    state.delay_regs.time.RequestJump(base_samp, 128);
                }
                if (frame.osc_mode == Oscillator::ModeSaw) {
                    // Saw mode: Unidirectional forward granular scrub across delay memory
                    float saw_phase = _osc.Phase();
                    float depth = 0.35f * amt_dly_head_mod;
                    eff_dly_knob = std::clamp(state.s34_dly + (saw_phase - 0.5f) * depth, 0.005f, 1.0f);
                    instant_dly = true;
                } else if (frame.osc_mode == Oscillator::ModeSin) {
                    // Sin mode: Smooth bidirectional back-and-forth sweep (like turning DLY knob with an LFO)
                    float sin_wave = FastSin(_osc.Phase() * kTwoPi);
                    float depth = 0.25f * amt_dly_head_mod;
                    eff_dly_knob = std::clamp(state.s34_dly + sin_wave * depth, 0.005f, 1.0f);
                    instant_dly = true;
                }
            }
        } else if (_prev_dly_hop_active) {
            _prev_dly_hop_active = false;
            _prev_rnd_step = 255;
            float base_samp = std::max(tuning::kMinDelaySamples, _delay.BaseSamples());
            state.delay_regs.time.RequestJump(base_samp, 128);
        }

        _delay.PrepareBlock(eff_dly_knob, state.s33_clk, state.delay_regs, 0.0f, override_delay_samples, instant_dly);

        _input.PrepareBlock(state.s30_amp);

        _preamp.PrepareBlock(state.s30_amp, state.pull[NodeAmp], state.vcc, state.starve,
                             state.pressures[NodeAmp], 0.0f, amounts.Get(Slot::PreampBias));

        _output.PrepareBlock(state.vcc,
                             amounts.Get(Slot::MasterCrash),
                             amounts.Get(Slot::MasterMute),
                             state.touch_out_gnd);

        MixerMods mmods{};
        mmods.filter_isolate = amounts.Get(Slot::MixFilterIsol);
        mmods.wet_only = amounts.Get(Slot::MixWetOnly);
        mmods.drive = amounts.Get(Slot::MixDrive);
        mmods.amp_direct = amounts.Get(Slot::MixAmpDirect);
        mmods.dropout = amounts.Get(Slot::MixDropout);
        _mixer.PrepareBlock(state.s32_mix, mmods);

        if (frame.tap_count_p10 != _prev_tap_count_p10) {
            _prev_tap_count_p10 = frame.tap_count_p10;
            _contact_mic.TriggerTap(frame.tap_vel_p10);
        }
        _contact_mic.SetArcingMotion(state.dynamic_motion, state.digital_motion_ratio);
        float raw_flt_ping = amounts.Raw(Slot::FltPing);
        if (raw_flt_ping > 0.01f && !_prev_flt_ping_active) {
            _prev_flt_ping_active = true;
            _filter.TriggerPing(std::clamp(raw_flt_ping, 0.25f, 1.20f));
        } else if (raw_flt_ping <= 0.005f) {
            _prev_flt_ping_active = false;
        }
    }

    __attribute__((always_inline)) inline void Render(const float* in_l, const float* in_r,
                                                      float* out_l, float* out_r,
                                                      size_t size,
                                                      BlockState& state,
                                                      const AmountBus& amounts,
                                                      const ModMatrix& matrix,
                                                      const ControlFrame& frame,
                                                      Rail& rail,
                                                      uint32_t cyc_start = 0) {
        (void)cyc_start;
        _input.DetectMono(in_l, in_r, size);

        float osc_vcc_sag = mappings::OscVccSag(state.vcc);
        float base_fbk_gain = state.delay_regs.feedback.gain;

        float g_osc_amp_target = amounts.Get(Slot::AmpOscVoice);
        if (frame.midi_gate > 0.0f && state.touch_osc) {
            g_osc_amp_target = std::max(g_osc_amp_target, frame.midi_gate + frame.midi_aftertouch * 0.25f);
        }
        float g_osc_flt_target = state.g_osc_flt;
        float firmness = std::clamp(std::max(g_osc_amp_target, g_osc_flt_target), 0.1f, 1.0f);
        float attack_tau = (65.0f + 1200.0f * (state.starve * state.starve)) / (0.4f + 0.6f * firmness);
        float decay_tau  = 850.0f + 22000.0f * (state.starve * state.starve);
        float alpha_charge = 1.0f / attack_tau;
        float alpha_decay  = 1.0f / decay_tau;

        // Block-rate invariant precomputations (direct array queries avoid 35 stack variables)
        ClockMods cmods_base{};
        cmods_base.hold = amounts.Get(Slot::ClkHold);
        cmods_base.rate_mod = 0.0f;
        cmods_base.swing = amounts.Get(Slot::ClkSwing);

        OscillatorMods omods_base{};
        omods_base.sub_octave = amounts.Get(Slot::OscSubOctave);
        omods_base.shape_warp = amounts.Get(Slot::OscShapeWarp);
        omods_base.chime = amounts.Get(Slot::ClkChime);
        omods_base.rungler_lock = amounts.Get(Slot::OscRunglerLock);
        omods_base.sync = amounts.Get(Slot::OscSync);

        PreampMods pmods_base{};
        pmods_base.drive = amounts.Get(Slot::PreampDrive) + (state.topology.intermod_loop ? 0.85f : 0.0f);
        pmods_base.bias = amounts.Get(Slot::PreampBias) + (state.motorboat_pulse > 0.001f ? state.motorboat_pulse * 0.16f : 0.0f);
        pmods_base.osc_voice = amounts.Get(Slot::AmpOscVoice);
        pmods_base.flt_loop = amounts.Get(Slot::AmpFltLoop);
        pmods_base.fbk_loop = amounts.Get(Slot::AmpFbkLoop);

        MixerMods mmods{};
        mmods.filter_isolate = amounts.Get(Slot::MixFilterIsol);
        mmods.wet_only = amounts.Get(Slot::MixWetOnly);
        mmods.drive = amounts.Get(Slot::MixDrive);
        mmods.amp_direct = amounts.Get(Slot::MixAmpDirect);
        mmods.dropout = amounts.Get(Slot::MixDropout);
        mmods.tremolo = amounts.Get(Slot::MixTremolo);

        float pump_depth = 0.0f;
        if (state.topology.voltage_pump) {
            pump_depth = 0.35f;
        } else if (omods_base.shape_warp > 0.02f) {
            pump_depth = omods_base.shape_warp * 0.22f;
        }
        const float eff_pump_scale = pump_depth * (0.35f + 0.65f * state.starve);

        const float mic_boost = 1.0f + amounts.Get(Slot::MicBoost) * 3.5f;
        const float pre_mute = amounts.Get(Slot::PreampMute);
        const float pre_mute_factor = (pre_mute > 0.02f) ? std::max(0.0f, 1.0f - pre_mute * 1.25f) : 1.0f;

        float clk_amp_bleed_boost = 0.0f;
        if (state.g_clk_amp > 0.008f) clk_amp_bleed_boost = state.g_clk_amp * 1.2f;
        if (state.topology.clk_amp_flt || state.topology.voice_quartet) {
            clk_amp_bleed_boost = std::max(clk_amp_bleed_boost, std::min(state.g_clk_amp, state.g_flt_amp) * 0.90f);
        }

        float clk_flt_bleed_boost = 0.0f;
        if (state.g_clk_flt > 0.008f) clk_flt_bleed_boost = state.g_clk_flt * 1.2f;
        if (state.topology.clk_amp_flt || state.topology.voice_quartet) {
            clk_flt_bleed_boost = std::max(clk_flt_bleed_boost, std::min(state.g_clk_flt, state.g_flt_amp) * 0.90f);
        }

        const bool mic_fbk_active = (amounts.Get(Slot::MicFeedback) > 0.001f);
        const float mic_fbk_scale = state.topology.karplus_resonator ? 1.25f : (amounts.Get(Slot::MicToFbk) * 0.85f);
        const float osc_out_gain = state.g_osc_out * 0.50f;

        const bool live_wire_active = (state.alone_node >= 0) || !_live_wire.IsIdle() ||
                                      (state.topology.active_node_count <= 2 && (state.g_in_clk > 0.005f || state.g_in_dly > 0.005f));
        const bool contact_mic_active = mic_fbk_active || _contact_mic.IsActive(state.pressure_in, state.rub_speed_in);
        const bool hum_active = (amounts.Get(Slot::HumVoice) > 0.001f) || (amounts.Get(Slot::Microsounds) > 0.001f) ||
                                (frame.proximities[NodeGnd] > 0.10f) || (_hum.SmoothHum() >= 0.0002f);
        const float flt_amp_scale = 0.40f * state.topology.norm_scale;
        const float fbk_amp_scale = 0.55f * state.topology.norm_scale;
        const float out_amp_scale = 0.65f * state.topology.norm_scale;
        const float amp_to_dly    = state.g_amp_dly * 1.20f;

        // Pre-resolve Kirchhoff AC cross-injections between connected nodes
        struct PreparedInj {
            uint8_t dst_idx; // 0: Osc, 1: Amp, 2: Flt, 3: Dly, 4: Mix
            uint8_t src_node;
            float gain;
        };
        PreparedInj prep_inj[8];
        uint8_t num_prep_inj = 0;
        if (state.num_cross_pairs > 0) {
            auto map_dst = [](uint8_t node) -> int8_t {
                if (node == NodeOsc) return 0;
                if (node == NodeAmp) return 1;
                if (node == NodeFlt) return 2;
                if (node == NodeDly) return 3;
                if (node == NodeMix) return 4;
                return -1;
            };
            for (uint8_t k = 0; k < state.num_cross_pairs; ++k) {
                const auto& p = state.cross_pairs[k];
                int8_t da = map_dst(p.a);
                if (da >= 0 && p.ga > 0.0001f) prep_inj[num_prep_inj++] = { static_cast<uint8_t>(da), p.b, p.ga };
                int8_t db = map_dst(p.b);
                if (db >= 0 && p.gb > 0.0001f) prep_inj[num_prep_inj++] = { static_cast<uint8_t>(db), p.a, p.gb };
            }
        }

        // Block-rate mod matrix presence checks (bypasses 576 evals/block when idle)
        const bool has_any_mod        = matrix.Count() > 0;
        const bool has_mod_clk_rate   = has_any_mod && matrix.HasSlot(kModClkRate);
        const bool has_mod_osc_fm     = has_any_mod && matrix.HasSlot(kModOscFm);
        const bool has_mod_amp_trem   = has_any_mod && matrix.HasSlot(kModAmpTremolo);
        const bool has_mod_pre_drive  = has_any_mod && matrix.HasSlot(kModPreampDrive);
        const bool has_mod_flt_cutoff = has_any_mod && matrix.HasSlot(kModFltCutoff);
        const bool has_mod_flt_clk    = has_any_mod && matrix.HasSlot(kModFltClockStep);
        const bool has_mod_dly_phase  = has_any_mod && matrix.HasSlot(kModDlyPhaseMod);
        const bool has_mod_dly_clk_fm = has_any_mod && matrix.HasSlot(kModDlyClockFm);
        const bool has_mod_dly_recirc = has_any_mod && matrix.HasSlot(kModDlyRecircFm);
        const bool has_mod_dly_burst  = has_any_mod && matrix.HasSlot(kModDlyClockBurst);
        const bool has_mod_mix_trem   = has_any_mod && matrix.HasSlot(kModMixTremolo);
        if (has_any_mod) {
            _bus.UpdateSigArray();
        }
        const uint16_t used_sig_mask = matrix.UsedSignalsMask();
        const bool uses_in_audio  = (used_sig_mask & (1u << static_cast<size_t>(Sig::InAudio))) != 0;
        const bool uses_in_env    = (used_sig_mask & (1u << static_cast<size_t>(Sig::InEnv))) != 0;
        const bool uses_preamp    = (used_sig_mask & (1u << static_cast<size_t>(Sig::PreampOut))) != 0;
        const bool uses_flt_out   = (used_sig_mask & (1u << static_cast<size_t>(Sig::FltOut))) != 0;
        const bool uses_dly_out   = (used_sig_mask & (1u << static_cast<size_t>(Sig::DlyOut))) != 0;
        const bool uses_dly_env   = (used_sig_mask & (1u << static_cast<size_t>(Sig::DlyEnv))) != 0;
        const bool uses_out_audio = (used_sig_mask & ((1u << static_cast<size_t>(Sig::OutAudio)) | (1u << static_cast<size_t>(Sig::MasterOut)))) != 0;
        const bool uses_out_env   = (used_sig_mask & (1u << static_cast<size_t>(Sig::OutEnv))) != 0;
        const bool uses_hum       = (used_sig_mask & (1u << static_cast<size_t>(Sig::Hum))) != 0;
        const bool uses_mic       = (used_sig_mask & (1u << static_cast<size_t>(Sig::ContactMic))) != 0;
        const float amt_clk_rate     = amounts.Get(Slot::ClkRate);
        const float amt_flt_cutoff   = amounts.Get(Slot::FltCutoff);
        const float amt_flt_clk      = amounts.Get(Slot::FltClockStep);
        const float amt_dly_phase    = amounts.Get(Slot::DlyPhaseMod);
        const float amt_dly_recirc   = amounts.Get(Slot::DlyRecircFm);
        const float amt_dly_burst    = amounts.Get(Slot::DlyClockBurst);
        const float amt_osc_fm       = amounts.Get(Slot::OscFm);
        const float amt_amp_trem     = amounts.Get(Slot::AmpTremolo);
        const float amt_pre_drive    = amounts.Get(Slot::PreampDrive);
        const float amt_dly_clk_fm   = amounts.Get(Slot::DlyClockFm);
        const float amt_mix_trem     = amounts.Get(Slot::MixTremolo);
        const float amt_flt_loop     = amounts.Get(Slot::AmpFltLoop);
        const float amt_fbk_loop     = amounts.Get(Slot::AmpFbkLoop);
        const float amt_hum_voice    = amounts.Get(Slot::HumVoice);
        const float amt_microsounds  = amounts.Get(Slot::Microsounds);
        const float amt_mic_fbk      = amounts.Get(Slot::MicFeedback);
        const float shunt_both       = state.retention[NodeMix] * state.retention[NodeOut];
        const float eff_pre_retention = pre_mute_factor * state.retention[NodeAmp];
        const float osc_retention = state.retention[NodeOsc];
        const bool scale_osc_retention = (osc_retention < 0.999f);
        const float clk_retention = state.retention[NodeClk];
        const bool scale_clk_retention = (clk_retention < 0.999f);

        FilterMods flt_mods_base{};
        flt_mods_base.resonance = amounts.Get(Slot::FltRes) + (state.motorboat_pulse > 0.001f ? state.motorboat_pulse * 0.08f : 0.0f);
        flt_mods_base.wavefold = amounts.Get(Slot::FltWavefold);

        float cached_sample_vcc  = rail.EffectiveVcc();
        float cached_sample_starve = daisysp::fclamp(1.0f - cached_sample_vcc, 0.0f, 1.0f);
        float cached_clk_fm      = amt_clk_rate;
        float cur_flt_cutoff     = _cur_flt_cutoff;
        float inc_flt_cutoff     = (amt_flt_cutoff - cur_flt_cutoff) * (1.0f / static_cast<float>(size));
        float cached_flt_cutoff_eff = cur_flt_cutoff;
        float cached_flt_clk     = amt_flt_clk;
        float cached_dly_phase   = amt_dly_phase;
        float cached_dly_recirc  = amt_dly_recirc;
        float cached_dly_burst   = amt_dly_burst;
        float cached_osc_fm      = amt_osc_fm;
        float cached_amp_trem    = amt_amp_trem;
        float cached_pre_drive   = amt_pre_drive;
        float cached_dly_clk_fm  = 0.0f;
        float cached_mix_trem    = amt_mix_trem;

        float cur_dly_clk_fm     = _cur_dly_clk_fm;
        float inc_dly_clk_fm     = (amt_dly_clk_fm - cur_dly_clk_fm) * (1.0f / static_cast<float>(size));

        float block_peak = 0.0f;
        float cached_quad_osc = 0.0f;
        float cached_intermod = 0.0f;

        float node_signals[kSolverNodes] = {0.0f};
        node_signals[NodeClk] = _bus.clk.l;
        node_signals[NodeOsc] = _bus.osc.l;
        node_signals[NodeFlt] = _bus.flt.l;
        node_signals[NodeDly] = _bus.dly.l;
        node_signals[NodeFbk] = _bus.fbk.l;
        node_signals[NodeAmp] = _bus.amp.l;
        node_signals[NodeMix] = _bus.mix.l;
        node_signals[NodeIn]  = _bus.in_sig.l;
        node_signals[NodeOut] = _bus.out_sig.l;

        float cached_inj_osc = 0.0f, cached_inj_amp = 0.0f, cached_inj_flt = 0.0f, cached_inj_dly = 0.0f, cached_inj_mix = 0.0f;
        float cached_cap_osc_amp = 0.0f, cached_cap_osc_flt = 0.0f, cached_cap_flt_amp = 0.0f, cached_cap_fbk_amp = 0.0f, cached_cap_out_amp = 0.0f;
        Stereo cached_flt_loop{0.0f, 0.0f}, cached_fbk_loop{0.0f, 0.0f}, cached_out_loop{0.0f, 0.0f}, cached_loop_sum{0.0f, 0.0f};

        uint32_t c_clk = 0, c_osc = 0, c_pre = 0, c_flt = 0, c_dly = 0, c_mix = 0, c_out = 0;

        for (size_t n = 0; n < size; ++n) {
            float boot_gain = 1.0f;
            Stereo in_ext = _input.ProcessSample(in_l ? in_l[n] : 0.0f,
                                                 in_r ? in_r[n] : 0.0f,
                                                 state.retention[NodeIn],
                                                 boot_gain);

            // Capacitive bridge tracking and AC cross-injections (12 kHz decimation)
            if ((n & 3) == 0) {
                cached_cap_osc_amp = _cap_osc_amp.Tick(g_osc_amp_target, alpha_charge * 4.0f, alpha_decay * 4.0f);
                cached_cap_osc_flt = _cap_osc_flt.Tick(g_osc_flt_target, alpha_charge * 4.0f, alpha_decay * 4.0f);
                cached_cap_flt_amp = _cap_flt_amp.Tick(amt_flt_loop, alpha_charge * 4.0f, alpha_decay * 4.0f);
                cached_cap_fbk_amp = _cap_fbk_amp.Tick(amt_fbk_loop, alpha_charge * 4.0f, alpha_decay * 4.0f);
                cached_cap_out_amp = _cap_out_amp.Tick(state.g_out_amp, alpha_charge * 4.0f, alpha_decay * 4.0f);

                if (num_prep_inj > 0) {
                    float inj[5]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
                    for (uint8_t k = 0; k < num_prep_inj; ++k) {
                        inj[prep_inj[k].dst_idx] += prep_inj[k].gain * node_signals[prep_inj[k].src_node];
                    }
                    cached_inj_osc = std::clamp(inj[0], -1.0f, 1.0f);
                    cached_inj_amp = std::clamp(inj[1], -1.0f, 1.0f);
                    cached_inj_flt = std::clamp(inj[2], -1.0f, 1.0f);
                    cached_inj_dly = std::clamp(inj[3], -1.0f, 1.0f);
                    cached_inj_mix = std::clamp(inj[4], -1.0f, 1.0f);
                } else {
                    cached_inj_osc = 0.0f;
                    cached_inj_amp = 0.0f;
                    cached_inj_flt = 0.0f;
                    cached_inj_dly = 0.0f;
                    cached_inj_mix = 0.0f;
                }
            }
            float cap_g_osc_amp = cached_cap_osc_amp;
            float cap_g_osc_flt = cached_cap_osc_flt;
            float cap_g_flt_amp = cached_cap_flt_amp;
            float cap_g_fbk_amp = cached_cap_fbk_amp;
            float cap_g_out_amp = cached_cap_out_amp;

            float inj_osc = cached_inj_osc;
            float inj_amp = cached_inj_amp;
            float inj_flt = cached_inj_flt;
            float inj_dly = cached_inj_dly;
            float inj_mix = cached_inj_mix;

            bool profile_sample = __builtin_expect(n == 0, 0);
            uint32_t t_sub = profile_sample ? DWT->CYCCNT : 0;

            cur_flt_cutoff += inc_flt_cutoff;
            cur_dly_clk_fm += inc_dly_clk_fm;

            // Dynamic audio-rate power rail breathing (evaluated at 24 kHz for efficiency):
            if ((n & 1) == 0) {
                float out_peak = 0.5f * (std::abs(_bus.out_sig.l) + std::abs(_bus.out_sig.r));
                float fbk_peak = 0.5f * (std::abs(_bus.fbk.l) + std::abs(_bus.fbk.r));
                float loop_peak = std::max(out_peak, fbk_peak);
                float audio_ripple = -loop_peak * (0.06f + 0.35f * state.starve);
                float pump_ripple = __builtin_expect(eff_pump_scale > 0.001f, 0) ? (_bus.osc.l * eff_pump_scale) : 0.0f;
                cached_sample_vcc = rail.Tick(audio_ripple + pump_ripple);
                cached_sample_starve = daisysp::fclamp(1.0f - cached_sample_vcc, 0.0f, 1.0f);
            }
            float sample_vcc = cached_sample_vcc;
            float sample_starve = cached_sample_starve;

            // Clock & ModMatrix (12 kHz interleaved eval)
            if (__builtin_expect(has_any_mod, 0)) {
                switch (n & 3) {
                case 0:
                    osc_vcc_sag = mappings::OscVccSag(sample_vcc);
                    if (has_mod_osc_fm)     cached_osc_fm     = matrix.Eval(kModOscFm, amt_osc_fm, _bus.sig_array);
                    if (has_mod_clk_rate)   cached_clk_fm     = matrix.Eval(kModClkRate, amt_clk_rate, _bus.sig_array);
                    if (has_mod_flt_clk)    cached_flt_clk    = matrix.Eval(kModFltClockStep, amt_flt_clk, _bus.sig_array);
                    break;
                case 1:
                    if (has_mod_dly_clk_fm) cached_dly_clk_fm = matrix.Eval(kModDlyClockFm, 0.0f, _bus.sig_array);
                    if (has_mod_dly_phase)  cached_dly_phase  = matrix.Eval(kModDlyPhaseMod, amt_dly_phase, _bus.sig_array);
                    if (has_mod_dly_recirc) cached_dly_recirc = matrix.Eval(kModDlyRecircFm, amt_dly_recirc, _bus.sig_array);
                    break;
                case 2:
                    if (has_mod_dly_burst)  cached_dly_burst  = matrix.Eval(kModDlyClockBurst, amt_dly_burst, _bus.sig_array);
                    if (has_mod_amp_trem)   cached_amp_trem   = matrix.Eval(kModAmpTremolo, amt_amp_trem, _bus.sig_array);
                    break;
                case 3:
                    if (has_mod_pre_drive)  cached_pre_drive  = matrix.Eval(kModPreampDrive, amt_pre_drive, _bus.sig_array);
                    if (has_mod_mix_trem)   cached_mix_trem   = matrix.Eval(kModMixTremolo, amt_mix_trem, _bus.sig_array);
                    break;
                }
            } else if ((n & 3) == 0) {
                osc_vcc_sag = mappings::OscVccSag(sample_vcc);
            }
            float clk_fm = cached_clk_fm;
            if (state.topology.rungler_triad && frame.osc_mode == Oscillator::ModeRnd) {
                clk_fm += _osc.RndHeld() * 0.85f;
            }
            float s_clk = 0.0f;
            _clock.Tick(sample_vcc, clk_fm, cmods_base, state.osc_mode, _bus.osc.l, _bus.osc_ramp, s_clk);
            if (__builtin_expect(scale_clk_retention, 0)) s_clk *= clk_retention;
            if (profile_sample) { c_clk = DWT->CYCCNT - t_sub; t_sub = DWT->CYCCNT; }

            // Oscillator
            float s_osc = 0.0f, osc_ramp = 0.0f, osc_reset = 0.0f, osc_sub = 0.0f;
            float osc_fm = cached_osc_fm;
            float osc_in = inj_osc + osc_fm;
            if (state.topology.rungler_triad && frame.osc_mode == Oscillator::ModeRnd) {
                osc_in += FastSoftSat(_bus.flt.l * 1.6f) * 0.40f;
            }
            bool clk_square = (_clock.Phase() < 0.5f);
            _osc.Tick(state.osc_mode, sample_vcc, osc_vcc_sag, osc_in, omods_base,
                      s_osc, osc_ramp, osc_reset, osc_sub, _clock.Beat(), clk_square, node_signals[NodeIn]);
            if (__builtin_expect(scale_osc_retention, 0)) s_osc *= osc_retention;
            if (profile_sample) { c_osc = DWT->CYCCNT - t_sub; }

            if (has_any_mod) {
                _bus.sig_array[static_cast<size_t>(Sig::Osc)] = s_osc;
                _bus.sig_array[static_cast<size_t>(Sig::OscRamp)] = osc_ramp;
                _bus.sig_array[static_cast<size_t>(Sig::OscReset)] = osc_reset;
                _bus.sig_array[static_cast<size_t>(Sig::OscSub)] = osc_sub;
            }

            if (profile_sample) { t_sub = DWT->CYCCNT; }

            // Hum
            float s_hum = 0.0f;
            if (__builtin_expect(hum_active, 0)) {
                _hum.Tick(sample_vcc, sample_starve, amt_hum_voice, frame.proximities[NodeGnd], 0.0f, amt_microsounds, s_hum);
                s_hum *= state.retention[NodeGnd];
            }

            // Contact mic
            float s_mic = 0.0f;
            if (__builtin_expect(contact_mic_active, 0)) {
                _contact_mic.Process(state.pressure_in, state.rub_speed_in, s_mic);
                float fbk_howl = mic_fbk_active ? (FastSoftSat(_bus.out_sig.l * 1.5f) * amt_mic_fbk) : 0.0f;
                s_mic = (s_mic * mic_boost + fbk_howl) * state.retention[NodeIn];
            }
            TouchAffordance affordance = __builtin_expect(live_wire_active, 0)
                                            ? _live_wire.Process(state, s_osc, sample_vcc, sample_starve)
                                            : TouchAffordance{};
            float in_amp_gain = 1.0f + state.g_in_amp * 2.5f * state.topology.norm_scale;
            Stereo in_combined = (in_ext + Stereo(s_mic)) * in_amp_gain + Stereo(affordance.pre);
            node_signals[NodeIn] = in_combined.l;

            // Preamp stage
            PreampMods pmods = pmods_base;
            pmods.drive = cached_pre_drive;
            pmods.tremolo = cached_amp_trem;
            pmods.ring_mod = cap_g_osc_amp;

            float clk_bleed_gain = cached_amp_trem;
            if (clk_amp_bleed_boost > 0.008f) {
                clk_bleed_gain = std::clamp(clk_bleed_gain + clk_amp_bleed_boost, 0.0f, 1.0f);
            }
            float clk_to_amp_bleed = clk_bleed_gain * s_clk * 0.85f;
            float common_inj = inj_amp * 0.25f + clk_to_amp_bleed;
            float osc_scale = cap_g_osc_amp * 0.10f;
            float osc_inject_l = s_osc * osc_scale + common_inj;
            float s_osc_r = s_osc;
            if (frame.osc_mode == Oscillator::ModeRnd && omods_base.shape_warp < 0.5f) {
                float osc_v_scale = (sample_vcc > 0.05f) ? (0.75f + 0.25f * sample_vcc) : (sample_vcc * 15.0f);
                float core_rnd = _osc.RndHeld();
                float rnd_diff = (_osc.RndHeldR() - core_rnd) * 0.20f;
                osc_inject_l = (core_rnd - rnd_diff) * (osc_v_scale * osc_scale) + common_inj;
                s_osc_r = (core_rnd + rnd_diff) * osc_v_scale;
            }
            float osc_inject_r = s_osc_r * osc_scale + common_inj;
            _bus.osc = Stereo(s_osc, s_osc_r);
            _bus.osc_ramp = osc_ramp;
            if ((n & 3) == 0) {
                // Non-linear feedback equilibrium: loop gain compresses as power rail voltage sags under heavy load
                float vcc_loop_comp = 0.68f + 0.32f * sample_vcc;
                cached_flt_loop = (cap_g_flt_amp > 0.0001f) ? (_bus.flt * (1.12f * cap_g_flt_amp * flt_amp_scale * vcc_loop_comp)) : Stereo(0.0f, 0.0f);
                if (state.topology.intermod_loop) {
                    cached_intermod = FastSoftSat(_bus.fbk.l * 2.2f);
                    cached_fbk_loop = (_bus.fbk * 0.90f + Stereo(cached_intermod * 0.45f)) * (std::max(cap_g_fbk_amp, 0.40f) * 0.55f * vcc_loop_comp);
                } else {
                    cached_fbk_loop = (cap_g_fbk_amp > 0.0001f) ? (_bus.fbk * (1.18f * cap_g_fbk_amp * fbk_amp_scale * vcc_loop_comp)) : Stereo(0.0f, 0.0f);
                }
                cached_out_loop = (cap_g_out_amp > 0.0001f) ? (_bus.out_sig * (1.15f * cap_g_out_amp * out_amp_scale * vcc_loop_comp)) : Stereo(0.0f, 0.0f);
                cached_loop_sum = cached_flt_loop + cached_fbk_loop + cached_out_loop;
            }

            if (state.topology.intermod_loop) {
                pmods.bias += cached_intermod * 0.35f;
            }

            // Preamp input mute (AMP-GND spillover kill)
            in_combined *= pre_mute_factor;

            Stereo in_pre = in_combined + cached_loop_sum;
            Stereo amp_out;
            _preamp.Process(in_pre.l, in_pre.r, 0.0f, pmods,
                            osc_inject_l, osc_inject_r, s_hum, sample_vcc, sample_starve, state.retention[NodeAmp],
                            amp_out.l, amp_out.r);
            // AMP <-> GND hard ground shunt: silences the entire preamp stage
            amp_out *= eff_pre_retention;
            if (profile_sample) { c_pre = DWT->CYCCNT - t_sub; t_sub = DWT->CYCCNT; }

            // Filter stage
            FilterMods flt_mods = flt_mods_base;
            if ((n & 1) == 0) {
                cached_flt_cutoff_eff = has_mod_flt_cutoff
                                           ? matrix.Eval(kModFltCutoff, cur_flt_cutoff, _bus.sig_array)
                                           : cur_flt_cutoff;
            }
            flt_mods.cutoff_ramp = cached_flt_cutoff_eff;
            flt_mods.clock_step = cached_flt_clk;
            if (state.topology.rungler_triad && frame.osc_mode == Oscillator::ModeRnd) {
                flt_mods.cutoff_ramp += s_clk * 0.85f;
            }

            float ping_duck = 1.0f - 0.25f * _filter.PingEnv();
            float flt_in_l = amp_out.l * ping_duck + inj_flt * 0.35f;
            float flt_in_r = amp_out.r * ping_duck + inj_flt * 0.35f;

            // Clock pulse injection and acoustic ping into Filter
            float clk_flt_gain = flt_mods.clock_step;
            if (clk_flt_bleed_boost > 0.008f) {
                clk_flt_gain = std::clamp(clk_flt_gain + clk_flt_bleed_boost, 0.0f, 1.0f);
            }
            if (clk_flt_gain > 0.01f) {
                float clk_punch = s_clk * clk_flt_gain * 0.20f;
                flt_in_l += clk_punch;
                flt_in_r += clk_punch;
            }

            // Continuous direct oscillator voice into filter (slewed analog capacitive bridge)
            if (cap_g_osc_flt > 0.0002f) {
                float raw_osc = (frame.osc_mode == Oscillator::ModeRnd && omods_base.shape_warp < 0.5f)
                                    ? _osc.RndHeld()
                                    : s_osc;
                float osc_voice = raw_osc * cap_g_osc_flt * 0.12f;
                _flt_osc_dc += 0.0024f * (osc_voice - _flt_osc_dc);
                float osc_voice_ac = osc_voice - _flt_osc_dc;
                flt_in_l += osc_voice_ac;
                flt_in_r += osc_voice_ac;
            }

            Stereo flt_out;
            _filter.Process(flt_in_l, flt_in_r,
                            0.0f, flt_mods, sample_vcc, sample_starve,
                            state.retention[NodeFlt], flt_out.l, flt_out.r);
            if (profile_sample) { c_flt = DWT->CYCCNT - t_sub; t_sub = DWT->CYCCNT; }

            // Delay stage (Memory Machine)
            float phase_mod = cached_dly_phase * 320.0f;

            // OSC -> CLK: Full 48 kHz continuous Doppler pitch vibrato & analog chorus (anti-aliased, zero zipper)
            if (cur_dly_clk_fm > 0.005f) {
                if (state.osc_mode == Oscillator::ModeSin) {
                    float sin_wave = FastSin(_osc.Phase() * kTwoPi);
                    phase_mod += sin_wave * (cur_dly_clk_fm * _cached_clk_vibrato_span);
                } else if (state.osc_mode == Oscillator::ModeSaw) {
                    float saw_phase = _osc.Phase();
                    float grain_ramp = saw_phase;
                    if (__builtin_expect(saw_phase > 0.94f, 0)) {
                        float edge = (1.0f - saw_phase) * (1.0f / 0.06f);
                        grain_ramp = 0.94f + (saw_phase - 0.94f) * edge;
                    }
                    float saw_wave = (grain_ramp - 0.47f) * 2.0f;
                    phase_mod += saw_wave * (cur_dly_clk_fm * _cached_clk_vibrato_span);
                }
            }

            float dly_clk_fm = cached_dly_clk_fm;

            // OSC -> CLK random stepped melodies in ModeRnd
            if (state.osc_mode == Oscillator::ModeRnd && cur_dly_clk_fm > 0.01f) {
                dly_clk_fm += _osc.RndHeld() * 0.85f * cur_dly_clk_fm;
            }

            // Lissajous quadrature cloud (OSC + CLK + DLY)
            if (state.topology.lissajous_cloud) {
                if ((n & 1) == 0) {
                    float quad_phase = _osc.Phase() + 0.25f;
                    if (quad_phase >= 1.0f) quad_phase -= 1.0f;
                    cached_quad_osc = FastSin(quad_phase * kTwoPi);
                }
                dly_clk_fm = dly_clk_fm * 0.65f + cached_quad_osc * 0.35f;
                phase_mod += cached_quad_osc * 36.0f;
            }

            state.delay_regs.time.mod_offset = phase_mod;
            state.delay_regs.clock.fm = dly_clk_fm;
            state.delay_regs.feedback.recirc_fm = cached_dly_recirc;
            float burst = cached_dly_burst;
            float fbk_gain = base_fbk_gain + burst * 0.45f;

            // Clock-synced feedback siren (VCC + CLK + FBK)
            if (state.topology.clock_siren) {
                float siren_surge = s_clk * 0.45f;
                fbk_gain = std::min(1.18f, fbk_gain + siren_surge);
                state.delay_regs.clock.fm += siren_surge * 0.60f;
            }

            // Karplus-Strong string resonator (FLT + DLY + FBK)
            if (state.topology.karplus_resonator) {
                fbk_gain = std::max(fbk_gain, 0.985f);
            }

            state.delay_regs.feedback.gain = fbk_gain;

            // Direct acoustic mic injection into feedback loop (IN-FBK)
            Stereo mic_to_fbk = Stereo(s_mic * mic_fbk_scale);

            Stereo dly_raw = flt_out * 1.0f + mic_to_fbk + Stereo(inj_dly * 0.20f + affordance.dly);
            if (__builtin_expect(amp_to_dly > 0.005f, 0)) {
                dly_raw += amp_out * amp_to_dly;
            }
            // 8 Hz pre-delay highpass filter: blocks DC offsets
            if ((n & 1) == 0) {
                _dly_in_hp.l += 0.0021f * (dly_raw.l - _dly_in_hp.l);
                _dly_in_hp.r += 0.0021f * (dly_raw.r - _dly_in_hp.r);
            }
            dly_raw.l -= _dly_in_hp.l;
            dly_raw.r -= _dly_in_hp.r;

            Stereo dly_in = {
                std::clamp(dly_raw.l, -2.5f, 2.5f),
                std::clamp(dly_raw.r, -2.5f, 2.5f)
            };
            Stereo dly_out = _delay.Process(dly_in, state.delay_regs, s_osc, _in_env);
            Stereo dry_sampled = _mixer.ClockMorphActive()
                                     ? _delay.ProcessDry(flt_out, state.delay_regs)
                                     : flt_out;
            if (profile_sample) { c_dly = DWT->CYCCNT - t_sub; t_sub = DWT->CYCCNT; }

            // Mixer stage
            Stereo mix_out;
            float flt_strike = _filter.PingEnv();
            mmods.tremolo = cached_mix_trem;
            float tap_out_osc_l = s_osc * osc_out_gain;
            float tap_out_osc_r = (state.osc_mode == Oscillator::ModeRnd ? s_osc_r : s_osc) * osc_out_gain;
            _mixer.Process(amp_out.l, amp_out.r, flt_out.l, flt_out.r, dly_out.l, dly_out.r,
                           dry_sampled.l, dry_sampled.r,
                           flt_strike, inj_mix + affordance.osc, inj_mix + affordance.osc,
                           tap_out_osc_l, tap_out_osc_r, mmods, shunt_both,
                           mix_out.l, mix_out.r);
            if (profile_sample) { c_mix = DWT->CYCCNT - t_sub; t_sub = DWT->CYCCNT; }

            // Output stage: Limiter, DC block, and filter
            Stereo final_sig = _output.Process(mix_out,
                                                sample_vcc,
                                                boot_gain);

            out_l[n] = final_sig.l;
            out_r[n] = final_sig.r;
            if (profile_sample) { c_out = DWT->CYCCNT - t_sub; }

            // Node bus history update (signals for next-sample feedback)
            _bus.flt = flt_out;
            _bus.fbk = dly_out;
            _bus.out_sig = final_sig;
            _bus.in_sig = in_combined;

            if (__builtin_expect(num_prep_inj > 0, 0) && (n & 1) == 0) {
                node_signals[NodeClk] = s_clk;
                node_signals[NodeOsc] = s_osc;
                node_signals[NodeFlt] = flt_out.l;
                node_signals[NodeDly] = dly_out.l;
                node_signals[NodeFbk] = dly_out.l;
                node_signals[NodeAmp] = amp_out.l;
                node_signals[NodeMix] = mix_out.l;
                node_signals[NodeIn]  = in_combined.l;
                node_signals[NodeOut] = final_sig.l;
            }

            // Audio envelope followers for dynamic modulation (12 kHz decimation)
            if ((n & 3) == 0) {
                float in_mag = std::abs(in_combined.l + in_combined.r) * 0.5f;
                _in_env += (in_mag > _in_env ? 0.30f : 0.008f) * (in_mag - _in_env);
                _bus.in_env = _in_env;

                float out_mag = std::abs(final_sig.l + final_sig.r) * 0.5f;
                _out_env += (out_mag > _out_env ? 0.20f : 0.008f) * (out_mag - _out_env);
                _bus.out_env = _out_env;

                float dly_mag = std::abs(dly_out.l + dly_out.r) * 0.5f;
                _dly_env += (dly_mag > _dly_env ? 0.22f : 0.008f) * (dly_mag - _dly_env);
                _bus.dly_env = _dly_env;
            }

            if (has_any_mod && (n & 1) == 0) {
                _bus.sig_array[static_cast<size_t>(Sig::ClkPulse)] = s_clk;
                if (uses_in_audio)  _bus.sig_array[static_cast<size_t>(Sig::InAudio)]   = in_combined.l;
                if (uses_in_env)    _bus.sig_array[static_cast<size_t>(Sig::InEnv)]     = _in_env;
                if (uses_preamp)    _bus.sig_array[static_cast<size_t>(Sig::PreampOut)] = amp_out.l;
                if (uses_flt_out)   _bus.sig_array[static_cast<size_t>(Sig::FltOut)]    = flt_out.l;
                if (uses_dly_out)   _bus.sig_array[static_cast<size_t>(Sig::DlyOut)]    = dly_out.l;
                if (uses_dly_env)   _bus.sig_array[static_cast<size_t>(Sig::DlyEnv)]    = _dly_env;
                if (uses_out_audio) {
                    _bus.sig_array[static_cast<size_t>(Sig::OutAudio)]  = final_sig.l;
                    _bus.sig_array[static_cast<size_t>(Sig::MasterOut)] = final_sig.l;
                }
                if (uses_out_env)   _bus.sig_array[static_cast<size_t>(Sig::OutEnv)]    = _out_env;
                if (uses_hum)       _bus.sig_array[static_cast<size_t>(Sig::Hum)]       = s_hum;
                if (uses_mic)       _bus.sig_array[static_cast<size_t>(Sig::ContactMic)] = s_mic;
            }

            float p = std::max(final_sig.MaxAbs(), std::max(_bus.flt.MaxAbs(), _bus.fbk.MaxAbs()));
            if (p > block_peak) block_peak = p;
        }

        _cur_flt_cutoff = cur_flt_cutoff;
        _cur_dly_clk_fm = cur_dly_clk_fm;

        last_profile.inj = 0;
        last_profile.clock = c_clk * size;
        last_profile.osc = c_osc * size;
        last_profile.preamp = c_pre * size;
        last_profile.filter = c_flt * size;
        last_profile.delay = c_dly * size;
        last_profile.mixer = c_mix * size;
        last_profile.output = c_out * size;


        _audio_envelope += 0.35f * (block_peak - _audio_envelope);
        _audio_envelope = Sanitize(_audio_envelope, 0.0f);
    }

    float AudioEnvelope() const { return _audio_envelope; }
    bool ClkBeat() const { return _clock.BlockBeat(); }
    float ClkRate() const { return _clock.Rate(); }
    float OscBaseFreq() const { return _osc.BaseFreq(); }
    float OscPhase() const { return _osc.Phase(); }
    uint8_t OscRndStep() const { return _osc.RndStep(); }

    DelayEngine& GetDelay() { return _delay; }
    Filter& GetFilter() { return _filter; }
    ContactMic& GetContactMic() { return _contact_mic; }

private:
    float _sample_rate{48000.0f};

    InputStage    _input;
    OutputStage   _output;
    DelayEngine   _delay;
    Clock         _clock;
    Oscillator    _osc;
    Hum           _hum;
    ContactMic    _contact_mic;
    Preamp        _preamp;
    Filter        _filter;
    Mixer         _mixer;
    LiveWireAffordance _live_wire;

    NodeBus       _bus;
    CapacitiveBridge _cap_osc_amp;
    CapacitiveBridge _cap_osc_flt;
    CapacitiveBridge _cap_flt_amp;
    CapacitiveBridge _cap_fbk_amp;
    CapacitiveBridge _cap_out_amp;
    Stereo        _dly_in_hp{0.0f, 0.0f};
    float         _flt_osc_dc{0.0f};
    float         _cur_flt_cutoff{0.0f};
    float         _cur_dly_clk_fm{0.0f};

    float         _audio_envelope{0.0f};
    float         _in_env{0.0f};
    float         _out_env{0.0f};
    float         _dly_env{0.0f};
    uint16_t      _prev_tap_count_p10{0};
    uint16_t      _prev_midi_trigger_cnt{0};
    bool          _prev_flt_ping_active{false};
    uint8_t       _prev_rnd_step{255};
    bool          _prev_dly_hop_active{false};
    float         _cached_clk_vibrato_span{260.0f};
};

} // namespace bends
