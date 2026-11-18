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
#include <cstring>

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

        for (size_t i = 0; i < kMaxBlockSize; ++i) {
            _prev_buf_flt[i] = {0.0f, 0.0f};
            _prev_buf_fbk[i] = {0.0f, 0.0f};
            _prev_buf_out[i] = {0.0f, 0.0f};
            _prev_buf_amp[i] = {0.0f, 0.0f};
            _prev_buf_mix[i] = {0.0f, 0.0f};
            _prev_buf_in[i]  = {0.0f, 0.0f};
            _prev_buf_osc[i] = 0.0f;
            _prev_buf_osc_ramp[i] = 0.0f;
            _prev_buf_clk[i] = 0.0f;
        }
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

        for (size_t i = 0; i < kMaxBlockSize; ++i) {
            _prev_buf_flt[i] = {0.0f, 0.0f};
            _prev_buf_fbk[i] = {0.0f, 0.0f};
            _prev_buf_out[i] = {0.0f, 0.0f};
            _prev_buf_amp[i] = {0.0f, 0.0f};
            _prev_buf_mix[i] = {0.0f, 0.0f};
            _prev_buf_in[i]  = {0.0f, 0.0f};
            _prev_buf_osc[i] = 0.0f;
            _prev_buf_osc_ramp[i] = 0.0f;
            _prev_buf_clk[i] = 0.0f;
        }
    }

    inline void ServiceIncrementalClear(size_t chunk_size = 1024) {
        _delay.ServiceIncrementalClear(chunk_size);
    }

    __attribute__((noinline)) void PrepareBlock(BlockState& state, const AmountBus& amounts, const ControlFrame& frame) {
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
        float brake_factor = std::clamp(state.delay_regs.clock.rate * std::abs(state.delay_regs.speed.speed), 0.00002f, 2.0f);
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

    struct RenderContext {
        float osc_vcc_sag{1.0f};
        float base_fbk_gain{0.0f};
        float g_osc_amp_target{0.0f};
        float g_osc_flt_target{0.0f};
        float alpha_charge{0.01f};
        float alpha_decay{0.001f};

        ClockMods cmods_base{};
        OscillatorMods omods_base{};
        PreampMods pmods_base{};
        MixerMods mmods{};
        FilterMods flt_mods_base{};

        float eff_pump_scale{0.0f};
        float mic_boost{1.0f};
        float pre_mute{0.0f};
        float pre_mute_factor{1.0f};
        float clk_amp_bleed_boost{0.0f};
        float clk_flt_bleed_boost{0.0f};
        bool mic_fbk_active{false};
        float mic_fbk_scale{0.0f};
        float osc_out_gain{0.0f};
        bool live_wire_active{false};
        bool contact_mic_active{false};
        bool hum_active{false};
        float flt_amp_scale{0.40f};
        float fbk_amp_scale{0.55f};
        float out_amp_scale{0.65f};
        float amp_to_dly{0.0f};

        struct PreparedInj {
            uint8_t dst_idx; // 0: Osc, 1: Amp, 2: Flt, 3: Dly, 4: Mix
            uint8_t src_node;
            float gain;
        };
        PreparedInj prep_inj[8];
        uint8_t num_prep_inj{0};

        bool has_any_mod{false};
        bool has_mod_clk_rate{false};
        bool has_mod_osc_fm{false};
        bool has_mod_amp_trem{false};
        bool has_mod_pre_drive{false};
        bool has_mod_flt_cutoff{false};
        bool has_mod_flt_clk{false};
        bool has_mod_dly_phase{false};
        bool has_mod_dly_clk_fm{false};
        bool has_mod_dly_recirc{false};
        bool has_mod_dly_burst{false};
        bool has_mod_mix_trem{false};

        bool uses_in_audio{false};
        bool uses_in_env{false};
        bool uses_preamp{false};
        bool uses_flt_out{false};
        bool uses_dly_out{false};
        bool uses_dly_env{false};
        bool uses_out_audio{false};
        bool uses_out_env{false};
        bool uses_hum{false};
        bool uses_mic{false};

        float amt_clk_rate{0.0f};
        float amt_flt_cutoff{0.0f};
        float amt_flt_clk{0.0f};
        float amt_dly_phase{0.0f};
        float amt_dly_recirc{0.0f};
        float amt_dly_burst{0.0f};
        float amt_osc_fm{0.0f};
        float amt_amp_trem{0.0f};
        float amt_pre_drive{0.0f};
        float amt_dly_clk_fm{0.0f};
        float amt_mix_trem{0.0f};
        float amt_flt_loop{0.0f};
        float amt_fbk_loop{0.0f};
        float amt_hum_voice{0.0f};
        float amt_microsounds{0.0f};
        float amt_mic_fbk{0.0f};

        float shunt_both{1.0f};
        float eff_pre_retention{1.0f};
        float osc_retention{1.0f};
        bool  scale_osc_retention{false};
        float clk_retention{1.0f};
        bool  scale_clk_retention{false};

        float inc_flt_cutoff{0.0f};
        float inc_dly_clk_fm{0.0f};
    };

    __attribute__((noinline)) void PrecomputeRenderContext(
        BlockState& state,
        const AmountBus& amounts,
        const ModMatrix& matrix,
        const ControlFrame& frame,
        size_t size,
        RenderContext& ctx) {
        ctx.osc_vcc_sag = mappings::OscVccSag(state.vcc);
        ctx.base_fbk_gain = state.delay_regs.feedback.gain;

        float g_osc_amp_target = amounts.Get(Slot::AmpOscVoice);
        if (frame.midi_gate > 0.0f && state.touch_osc) {
            g_osc_amp_target = std::max(g_osc_amp_target, frame.midi_gate + frame.midi_aftertouch * 0.25f);
        }
        ctx.g_osc_amp_target = g_osc_amp_target;
        ctx.g_osc_flt_target = state.g_osc_flt;

        float firmness = std::clamp(std::max(g_osc_amp_target, ctx.g_osc_flt_target), 0.1f, 1.0f);
        float attack_tau = (65.0f + 1200.0f * (state.starve * state.starve)) / (0.4f + 0.6f * firmness);
        float decay_tau  = 850.0f + 22000.0f * (state.starve * state.starve);
        ctx.alpha_charge = 1.0f / attack_tau;
        ctx.alpha_decay  = 1.0f / decay_tau;

        // Block-rate invariant precomputations
        ctx.cmods_base.hold = amounts.Get(Slot::ClkHold);
        ctx.cmods_base.rate_mod = 0.0f;
        ctx.cmods_base.swing = amounts.Get(Slot::ClkSwing);

        ctx.omods_base.sub_octave = amounts.Get(Slot::OscSubOctave);
        ctx.omods_base.shape_warp = amounts.Get(Slot::OscShapeWarp);
        ctx.omods_base.rungler_lock = amounts.Get(Slot::OscRunglerLock);
        ctx.omods_base.sync = amounts.Get(Slot::OscSync);

        ctx.pmods_base.drive = amounts.Get(Slot::PreampDrive) + (state.topology.intermod_loop ? 0.85f : 0.0f);
        ctx.pmods_base.bias = amounts.Get(Slot::PreampBias) + (state.motorboat_pulse > 0.001f ? state.motorboat_pulse * 0.16f : 0.0f);
        ctx.pmods_base.osc_voice = amounts.Get(Slot::AmpOscVoice);
        ctx.pmods_base.flt_loop = amounts.Get(Slot::AmpFltLoop);
        ctx.pmods_base.fbk_loop = amounts.Get(Slot::AmpFbkLoop);

        ctx.mmods.filter_isolate = amounts.Get(Slot::MixFilterIsol);
        ctx.mmods.wet_only = amounts.Get(Slot::MixWetOnly);
        ctx.mmods.drive = amounts.Get(Slot::MixDrive);
        ctx.mmods.amp_direct = amounts.Get(Slot::MixAmpDirect);
        ctx.mmods.dropout = amounts.Get(Slot::MixDropout);
        ctx.mmods.tremolo = amounts.Get(Slot::MixTremolo);

        float pump_depth = 0.0f;
        if (state.topology.voltage_pump) {
            pump_depth = 0.35f;
        } else if (ctx.omods_base.shape_warp > 0.02f) {
            pump_depth = ctx.omods_base.shape_warp * 0.22f;
        }
        ctx.eff_pump_scale = pump_depth * (0.35f + 0.65f * state.starve);

        ctx.mic_boost = 1.0f + amounts.Get(Slot::MicBoost) * 3.5f;
        ctx.pre_mute = amounts.Get(Slot::PreampMute);
        ctx.pre_mute_factor = (ctx.pre_mute > 0.02f) ? std::max(0.0f, 1.0f - ctx.pre_mute * 1.25f) : 1.0f;

        ctx.clk_amp_bleed_boost = 0.0f;
        if (state.g_clk_amp > 0.008f) ctx.clk_amp_bleed_boost = state.g_clk_amp * 1.2f;

        ctx.clk_flt_bleed_boost = 0.0f;
        if (state.g_clk_flt > 0.008f) ctx.clk_flt_bleed_boost = state.g_clk_flt * 1.2f;

        if (state.topology.clk_amp_flt || state.topology.voice_quartet) {
            float triad_coupling = std::min(state.g_clk_flt, state.g_flt_amp);
            float boost = std::max(triad_coupling, 0.75f) * 1.35f;
            ctx.clk_amp_bleed_boost = std::max(ctx.clk_amp_bleed_boost, boost);
            ctx.clk_flt_bleed_boost = std::max(ctx.clk_flt_bleed_boost, boost);
        }

        ctx.mic_fbk_active = (amounts.Get(Slot::MicFeedback) > 0.001f);
        ctx.mic_fbk_scale = state.topology.karplus_resonator ? 1.25f : (amounts.Get(Slot::MicToFbk) * 0.85f);
        ctx.osc_out_gain = state.g_osc_out * 0.50f;

        ctx.live_wire_active = (state.alone_node >= 0) || !_live_wire.IsIdle() ||
                               (state.topology.active_node_count <= 2 && (state.g_in_clk > 0.005f || state.g_in_dly > 0.005f));
        ctx.contact_mic_active = ctx.mic_fbk_active || _contact_mic.IsActive(state.pressure_in, state.rub_speed_in);
        ctx.hum_active = (amounts.Get(Slot::HumVoice) > 0.001f) || (amounts.Get(Slot::Microsounds) > 0.001f) ||
                         (frame.proximities[NodeGnd] > 0.10f) || (_hum.SmoothHum() >= 0.0002f);
        ctx.flt_amp_scale = 0.40f * state.topology.norm_scale;
        ctx.fbk_amp_scale = 0.55f * state.topology.norm_scale;
        ctx.out_amp_scale = 0.65f * state.topology.norm_scale;
        ctx.amp_to_dly    = state.g_amp_dly * 1.20f;

        // Pre-resolve Kirchhoff AC cross-injections between connected nodes
        ctx.num_prep_inj = 0;
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
                if (da >= 0 && p.ga > 0.0001f) ctx.prep_inj[ctx.num_prep_inj++] = { static_cast<uint8_t>(da), p.b, p.ga };
                int8_t db = map_dst(p.b);
                if (db >= 0 && p.gb > 0.0001f) ctx.prep_inj[ctx.num_prep_inj++] = { static_cast<uint8_t>(db), p.a, p.gb };
            }
        }

        // Block-rate mod matrix presence checks
        ctx.has_any_mod        = matrix.Count() > 0;
        ctx.has_mod_clk_rate   = ctx.has_any_mod && matrix.HasSlot(kModClkRate);
        ctx.has_mod_osc_fm     = ctx.has_any_mod && matrix.HasSlot(kModOscFm);
        ctx.has_mod_amp_trem   = ctx.has_any_mod && matrix.HasSlot(kModAmpTremolo);
        ctx.has_mod_pre_drive  = ctx.has_any_mod && matrix.HasSlot(kModPreampDrive);
        ctx.has_mod_flt_cutoff = ctx.has_any_mod && matrix.HasSlot(kModFltCutoff);
        ctx.has_mod_flt_clk    = ctx.has_any_mod && matrix.HasSlot(kModFltClockStep);
        ctx.has_mod_dly_phase  = ctx.has_any_mod && matrix.HasSlot(kModDlyPhaseMod);
        ctx.has_mod_dly_clk_fm = ctx.has_any_mod && matrix.HasSlot(kModDlyClockFm);
        ctx.has_mod_dly_recirc = ctx.has_any_mod && matrix.HasSlot(kModDlyRecircFm);
        ctx.has_mod_dly_burst  = ctx.has_any_mod && matrix.HasSlot(kModDlyClockBurst);
        ctx.has_mod_mix_trem   = ctx.has_any_mod && matrix.HasSlot(kModMixTremolo);
        if (ctx.has_any_mod) {
            _bus.UpdateSigArray();
        }
        const uint16_t used_sig_mask = matrix.UsedSignalsMask();
        ctx.uses_in_audio  = (used_sig_mask & (1u << static_cast<size_t>(Sig::InAudio))) != 0;
        ctx.uses_in_env    = (used_sig_mask & (1u << static_cast<size_t>(Sig::InEnv))) != 0;
        ctx.uses_preamp    = (used_sig_mask & (1u << static_cast<size_t>(Sig::PreampOut))) != 0;
        ctx.uses_flt_out   = (used_sig_mask & (1u << static_cast<size_t>(Sig::FltOut))) != 0;
        ctx.uses_dly_out   = (used_sig_mask & (1u << static_cast<size_t>(Sig::DlyOut))) != 0;
        ctx.uses_dly_env   = (used_sig_mask & (1u << static_cast<size_t>(Sig::DlyEnv))) != 0;
        ctx.uses_out_audio = (used_sig_mask & ((1u << static_cast<size_t>(Sig::OutAudio)) | (1u << static_cast<size_t>(Sig::MasterOut)))) != 0;
        ctx.uses_out_env   = (used_sig_mask & (1u << static_cast<size_t>(Sig::OutEnv))) != 0;
        ctx.uses_hum       = (used_sig_mask & (1u << static_cast<size_t>(Sig::Hum))) != 0;
        ctx.uses_mic       = (used_sig_mask & (1u << static_cast<size_t>(Sig::ContactMic))) != 0;

        ctx.amt_clk_rate     = amounts.Get(Slot::ClkRate);
        ctx.amt_flt_cutoff   = amounts.Get(Slot::FltCutoff);
        ctx.amt_flt_clk      = amounts.Get(Slot::FltClockStep);
        ctx.amt_dly_phase    = amounts.Get(Slot::DlyPhaseMod);
        ctx.amt_dly_recirc   = amounts.Get(Slot::DlyRecircFm);
        ctx.amt_dly_burst    = amounts.Get(Slot::DlyClockBurst);
        ctx.amt_osc_fm       = amounts.Get(Slot::OscFm);
        ctx.amt_amp_trem     = amounts.Get(Slot::AmpTremolo);
        ctx.amt_pre_drive    = amounts.Get(Slot::PreampDrive);
        ctx.amt_dly_clk_fm   = amounts.Get(Slot::DlyClockFm);
        ctx.amt_mix_trem     = amounts.Get(Slot::MixTremolo);
        ctx.amt_flt_loop     = amounts.Get(Slot::AmpFltLoop);
        ctx.amt_fbk_loop     = amounts.Get(Slot::AmpFbkLoop);
        ctx.amt_hum_voice    = amounts.Get(Slot::HumVoice);
        ctx.amt_microsounds  = amounts.Get(Slot::Microsounds);
        ctx.amt_mic_fbk      = amounts.Get(Slot::MicFeedback);

        ctx.shunt_both        = state.retention[NodeMix] * state.retention[NodeOut];
        ctx.eff_pre_retention = ctx.pre_mute_factor * state.retention[NodeAmp];
        ctx.osc_retention     = state.retention[NodeOsc];
        ctx.scale_osc_retention = (ctx.osc_retention < 0.999f);
        ctx.clk_retention     = state.retention[NodeClk];
        ctx.scale_clk_retention = (ctx.clk_retention < 0.999f);

        ctx.flt_mods_base.resonance = amounts.Get(Slot::FltRes) + (state.motorboat_pulse > 0.001f ? state.motorboat_pulse * 0.08f : 0.0f);
        ctx.flt_mods_base.wavefold = amounts.Get(Slot::FltWavefold);

        ctx.inc_flt_cutoff = (ctx.amt_flt_cutoff - _cur_flt_cutoff) * (1.0f / static_cast<float>(size));
        ctx.inc_dly_clk_fm = (ctx.amt_dly_clk_fm - _cur_dly_clk_fm) * (1.0f / static_cast<float>(size));
    }

    __attribute__((noinline)) void FinalizeBlockTelemetry(
        float cur_flt_cutoff, float cur_dly_clk_fm,
        float block_peak) {
        _cur_flt_cutoff = cur_flt_cutoff;
        _cur_dly_clk_fm = cur_dly_clk_fm;
        last_profile.inj = 0;
        _audio_envelope += 0.35f * (block_peak - _audio_envelope);
        _audio_envelope = Sanitize(_audio_envelope, 0.0f);
    }

    __attribute__((noinline)) void Render(const float* in_l, const float* in_r,
                                          float* out_l, float* out_r,
                                          size_t size,
                                          BlockState& state,
                                          const AmountBus& amounts,
                                          const ModMatrix& matrix,
                                          const ControlFrame& frame,
                                          Rail& rail,
                                          uint32_t cyc_start = 0) {
        (void)cyc_start;
        if (size == 0) return;
        if (size > kMaxBlockSize) size = kMaxBlockSize;

        _input.DetectMono(in_l, in_r, size);

        RenderContext ctx{};
        PrecomputeRenderContext(state, amounts, matrix, frame, size, ctx);

        float cur_flt_cutoff     = _cur_flt_cutoff;
        float cur_dly_clk_fm     = _cur_dly_clk_fm;
        float cached_clk_fm      = ctx.amt_clk_rate;
        float cached_osc_fm      = ctx.amt_osc_fm;
        float cached_amp_trem    = ctx.amt_amp_trem;
        float cached_pre_drive   = ctx.amt_pre_drive;
        float cached_flt_clk     = ctx.amt_flt_clk;
        float cached_flt_cutoff_eff = cur_flt_cutoff;
        float cached_dly_phase   = ctx.amt_dly_phase;
        float cached_dly_recirc  = ctx.amt_dly_recirc;
        float cached_dly_burst   = ctx.amt_dly_burst;
        float cached_dly_clk_fm  = 0.0f;
        float cached_mix_trem    = ctx.amt_mix_trem;
        float cached_sample_vcc  = rail.EffectiveVcc();
        float cached_sample_starve = std::clamp(1.0f - cached_sample_vcc, 0.0f, 1.0f);
        float osc_vcc_sag        = ctx.osc_vcc_sag;

        float cached_cap_osc_amp = 0.0f, cached_cap_osc_flt = 0.0f, cached_cap_flt_amp = 0.0f, cached_cap_fbk_amp = 0.0f, cached_cap_out_amp = 0.0f;
        float cached_inj_osc = 0.0f, cached_inj_amp = 0.0f, cached_inj_flt = 0.0f, cached_inj_dly = 0.0f, cached_inj_mix = 0.0f;
        Stereo cached_flt_loop{0.0f, 0.0f}, cached_fbk_loop{0.0f, 0.0f}, cached_out_loop{0.0f, 0.0f}, cached_loop_sum{0.0f, 0.0f};
        float cached_intermod = 0.0f;
        float cached_quad_osc = 0.0f;
        float boot_gain = 1.0f;

        // =====================================================================
        // STAGE 1: Inputs, Decimated Bridges, Dynamic Rail, Hum, Mic, Affordance
        // =====================================================================
        for (size_t n = 0; n < size; ++n) {
            Stereo in_ext = _input.ProcessSample(in_l ? in_l[n] : 0.0f,
                                                 in_r ? in_r[n] : 0.0f,
                                                 state.retention[NodeIn],
                                                 boot_gain);

            if ((n & 3) == 0) {
                cached_cap_osc_amp = _cap_osc_amp.Tick(ctx.g_osc_amp_target, ctx.alpha_charge * 4.0f, ctx.alpha_decay * 4.0f);
                cached_cap_osc_flt = _cap_osc_flt.Tick(ctx.g_osc_flt_target, ctx.alpha_charge * 4.0f, ctx.alpha_decay * 4.0f);
                cached_cap_flt_amp = _cap_flt_amp.Tick(ctx.amt_flt_loop, ctx.alpha_charge * 4.0f, ctx.alpha_decay * 4.0f);
                cached_cap_fbk_amp = _cap_fbk_amp.Tick(ctx.amt_fbk_loop, ctx.alpha_charge * 4.0f, ctx.alpha_decay * 4.0f);
                cached_cap_out_amp = _cap_out_amp.Tick(state.g_out_amp, ctx.alpha_charge * 4.0f, ctx.alpha_decay * 4.0f);

                if (ctx.num_prep_inj > 0) {
                    float inj[5]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
                    for (uint8_t k = 0; k < ctx.num_prep_inj; ++k) {
                        float sig_val = 0.0f;
                        switch (ctx.prep_inj[k].src_node) {
                            case NodeClk: sig_val = _prev_buf_clk[n]; break;
                            case NodeOsc: sig_val = _prev_buf_osc[n]; break;
                            case NodeFlt: sig_val = _prev_buf_flt[n].l; break;
                            case NodeDly: sig_val = _prev_buf_fbk[n].l; break;
                            case NodeFbk: sig_val = _prev_buf_fbk[n].l; break;
                            case NodeAmp: sig_val = _prev_buf_amp[n].l; break;
                            case NodeMix: sig_val = _prev_buf_mix[n].l; break;
                            case NodeIn:  sig_val = _prev_buf_in[n].l; break;
                            case NodeOut: sig_val = _prev_buf_out[n].l; break;
                            default: break;
                        }
                        inj[ctx.prep_inj[k].dst_idx] += ctx.prep_inj[k].gain * sig_val;
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
            _buf_cap_osc_amp[n] = cached_cap_osc_amp;
            _buf_cap_osc_flt[n] = cached_cap_osc_flt;
            _buf_inj_osc[n] = cached_inj_osc;
            _buf_inj_amp[n] = cached_inj_amp;
            _buf_inj_flt[n] = cached_inj_flt;
            _buf_inj_dly[n] = cached_inj_dly;
            _buf_inj_mix[n] = cached_inj_mix;

            if ((n & 1) == 0) {
                float out_peak = 0.5f * (std::abs(_prev_buf_out[n].l) + std::abs(_prev_buf_out[n].r));
                float fbk_peak = 0.5f * (std::abs(_prev_buf_fbk[n].l) + std::abs(_prev_buf_fbk[n].r));
                float loop_peak = std::max(out_peak, fbk_peak);
                float audio_ripple = -loop_peak * (0.06f + 0.35f * state.starve);
                float pump_ripple = __builtin_expect(ctx.eff_pump_scale > 0.001f, 0) ? (_prev_buf_osc[n] * ctx.eff_pump_scale) : 0.0f;
                cached_sample_vcc = rail.Tick(audio_ripple + pump_ripple);
                cached_sample_starve = std::clamp(1.0f - cached_sample_vcc, 0.0f, 1.0f);
            }
            _buf_rail_vcc[n] = cached_sample_vcc;
            _buf_rail_starve[n] = cached_sample_starve;

            float s_hum = 0.0f;
            if (__builtin_expect(ctx.hum_active, 0)) {
                _hum.Tick(cached_sample_vcc, cached_sample_starve, ctx.amt_hum_voice, frame.proximities[NodeGnd], 0.0f, ctx.amt_microsounds, s_hum);
                s_hum *= state.retention[NodeGnd];
            }
            _buf_hum[n] = s_hum;

            float s_mic = 0.0f;
            if (__builtin_expect(ctx.contact_mic_active, 0)) {
                _contact_mic.Process(state.pressure_in, state.rub_speed_in, s_mic);
                float fbk_howl = ctx.mic_fbk_active ? (FastSoftSat(_prev_buf_out[n].l * 1.5f) * ctx.amt_mic_fbk) : 0.0f;
                s_mic = (s_mic * ctx.mic_boost + fbk_howl) * state.retention[NodeIn];
            }
            _buf_mic[n] = s_mic;

            TouchAffordance affordance = __builtin_expect(ctx.live_wire_active, 0)
                                            ? _live_wire.Process(state, _prev_buf_osc[n], cached_sample_vcc, cached_sample_starve)
                                            : TouchAffordance{};
            _buf_afford_osc[n] = affordance.osc;
            _buf_afford_dly[n] = affordance.dly;

            float in_amp_gain = 1.0f + state.g_in_amp * 2.5f * state.topology.norm_scale;
            Stereo in_combined = (in_ext + Stereo(s_mic)) * in_amp_gain + Stereo(affordance.pre);
            in_combined *= ctx.pre_mute_factor;
            _buf_in_combined[n] = in_combined;

            // Non-linear feedback equilibrium: loop gain compresses as power rail voltage sags under heavy load
            float vcc_loop_comp = 0.68f + 0.32f * cached_sample_vcc;
            if (state.topology.intermod_loop) {
                cached_intermod = FastSoftSat(_prev_buf_fbk[n].l * 2.2f);
                cached_fbk_loop = (_prev_buf_fbk[n] * 0.90f + Stereo(cached_intermod * 0.45f)) * (std::max(cached_cap_fbk_amp, 0.40f) * 0.55f * vcc_loop_comp);
            } else {
                cached_fbk_loop = (cached_cap_fbk_amp > 0.0001f) ? (_prev_buf_fbk[n] * (1.18f * cached_cap_fbk_amp * ctx.fbk_amp_scale * vcc_loop_comp)) : Stereo(0.0f, 0.0f);
            }
            cached_out_loop = (cached_cap_out_amp > 0.0001f) ? (_prev_buf_out[n] * (1.15f * cached_cap_out_amp * ctx.out_amp_scale * vcc_loop_comp)) : Stereo(0.0f, 0.0f);
            cached_loop_sum = cached_fbk_loop + cached_out_loop;
            _buf_loop_sum[n] = cached_loop_sum;
            _buf_intermod[n] = cached_intermod;
        }

        // =====================================================================
        // STAGE 2: Clock Generation
        // =====================================================================
        uint32_t t_stage = DWT->CYCCNT;
        for (size_t n = 0; n < size; ++n) {
            if (__builtin_expect(ctx.has_any_mod, 0)) {
                if ((n & 3) == 0 && ctx.has_mod_clk_rate) {
                    cached_clk_fm = matrix.Eval(kModClkRate, ctx.amt_clk_rate, _bus.sig_array);
                }
            }
            float clk_fm = cached_clk_fm;
            if (state.topology.rungler_triad && frame.osc_mode == Oscillator::ModeRnd) {
                clk_fm += _osc.RndHeld() * 0.85f;
            }
            float s_clk = 0.0f;
            _clock.Tick(_buf_rail_vcc[n], clk_fm, ctx.cmods_base, state.osc_mode, _prev_buf_osc[n], _prev_buf_osc_ramp[n], s_clk);
            if (__builtin_expect(ctx.scale_clk_retention, 0)) s_clk *= ctx.clk_retention;
            _buf_clk[n] = s_clk;
            _buf_clk_sq[n] = (_clock.Phase() < 0.5f);
            _buf_clk_beat[n] = _clock.Beat();
        }
        last_profile.clock = DWT->CYCCNT - t_stage;

        // =====================================================================
        // STAGE 3: Oscillator Generation
        // =====================================================================
        t_stage = DWT->CYCCNT;
        for (size_t n = 0; n < size; ++n) {
            if ((n & 3) == 0) {
                osc_vcc_sag = mappings::OscVccSag(_buf_rail_vcc[n]);
                if (__builtin_expect(ctx.has_any_mod, 0)) {
                    if (ctx.has_mod_osc_fm) cached_osc_fm = matrix.Eval(kModOscFm, ctx.amt_osc_fm, _bus.sig_array);
                }
            }
            float s_osc = 0.0f, osc_ramp = 0.0f, osc_reset = 0.0f, osc_sub = 0.0f;
            float osc_in = _buf_inj_osc[n] + cached_osc_fm;
            if (state.topology.rungler_triad && frame.osc_mode == Oscillator::ModeRnd) {
                osc_in += FastSoftSat(_prev_buf_flt[n].l * 1.6f) * 0.40f;
            }
            _osc.Tick(state.osc_mode, _buf_rail_vcc[n], osc_vcc_sag, osc_in, ctx.omods_base,
                      s_osc, osc_ramp, osc_reset, osc_sub, _buf_clk_beat[n], _buf_clk_sq[n], _buf_in_combined[n].l);
            if (__builtin_expect(ctx.scale_osc_retention, 0)) s_osc *= ctx.osc_retention;

            float s_osc_r = s_osc;
            if (frame.osc_mode == Oscillator::ModeRnd && ctx.omods_base.shape_warp < 0.5f) {
                float osc_v_scale = (_buf_rail_vcc[n] > 0.05f) ? (0.75f + 0.25f * _buf_rail_vcc[n]) : (_buf_rail_vcc[n] * 15.0f);
                float core_rnd = _osc.RndHeld();
                float rnd_diff = (_osc.RndHeldR() - core_rnd) * 0.20f;
                s_osc_r = (core_rnd + rnd_diff) * osc_v_scale;
            }

            _buf_osc[n] = s_osc;
            _buf_osc_r[n] = s_osc_r;
            _buf_osc_ramp[n] = osc_ramp;
            _buf_osc_phase[n] = _osc.Phase();
        }
        _bus.osc = Stereo(_buf_osc[size - 1], _buf_osc_r[size - 1]);
        _bus.osc_ramp = _buf_osc_ramp[size - 1];
        if (ctx.has_any_mod) {
            _bus.sig_array[static_cast<size_t>(Sig::Osc)] = _buf_osc[size - 1];
            _bus.sig_array[static_cast<size_t>(Sig::OscRamp)] = _buf_osc_ramp[size - 1];
        }
        last_profile.osc = DWT->CYCCNT - t_stage;

        // =====================================================================
        // STAGE 4 & 5: Preamp & Resonant Filter (1-Sample Analog Feedback Core)
        // =====================================================================
        t_stage = DWT->CYCCNT;
        PreampMods pmods = ctx.pmods_base;
        FilterMods flt_mods = ctx.flt_mods_base;
        for (size_t n = 0; n < size; ++n) {
            cur_flt_cutoff += ctx.inc_flt_cutoff;
            if (__builtin_expect(ctx.has_any_mod, 0)) {
                if ((n & 3) == 0 && ctx.has_mod_flt_clk) cached_flt_clk = matrix.Eval(kModFltClockStep, ctx.amt_flt_clk, _bus.sig_array);
                if ((n & 3) == 2 && ctx.has_mod_amp_trem) cached_amp_trem = matrix.Eval(kModAmpTremolo, ctx.amt_amp_trem, _bus.sig_array);
                if ((n & 3) == 3 && ctx.has_mod_pre_drive) cached_pre_drive = matrix.Eval(kModPreampDrive, ctx.amt_pre_drive, _bus.sig_array);
            }
            if ((n & 1) == 0) {
                cached_flt_cutoff_eff = ctx.has_mod_flt_cutoff
                                           ? matrix.Eval(kModFltCutoff, cur_flt_cutoff, _bus.sig_array)
                                           : cur_flt_cutoff;
            }

            pmods.drive = cached_pre_drive;
            pmods.tremolo = cached_amp_trem;
            pmods.ring_mod = _buf_cap_osc_amp[n];

            float clk_bleed_gain = cached_amp_trem;
            if (ctx.clk_amp_bleed_boost > 0.008f) {
                clk_bleed_gain = std::clamp(clk_bleed_gain + ctx.clk_amp_bleed_boost, 0.0f, 1.0f);
            }
            float clk_to_amp_bleed = clk_bleed_gain * _buf_clk[n] * 0.85f;
            float common_inj = _buf_inj_amp[n] * 0.25f + clk_to_amp_bleed;
            float osc_scale = _buf_cap_osc_amp[n] * 0.10f;
            float osc_inject_l = _buf_osc[n] * osc_scale + common_inj;
            float osc_inject_r = _buf_osc_r[n] * osc_scale + common_inj;

            if (state.topology.intermod_loop) {
                pmods.bias = ctx.pmods_base.bias + _buf_intermod[n] * 0.35f;
            }

            // 1-sample instantaneous feedback from filter output into preamp
            float vcc_loop_comp = 0.68f + 0.32f * _buf_rail_vcc[n];
            Stereo flt_loop = (cached_cap_flt_amp > 0.0001f)
                                  ? (_bus.flt * (1.10f * cached_cap_flt_amp * ctx.flt_amp_scale * vcc_loop_comp))
                                  : Stereo(0.0f, 0.0f);

            Stereo in_pre = _buf_in_combined[n] + flt_loop + _buf_loop_sum[n];
            Stereo amp_out;
            _preamp.Process(in_pre.l, in_pre.r, 0.0f, pmods,
                            osc_inject_l, osc_inject_r, _buf_hum[n], _buf_rail_vcc[n], _buf_rail_starve[n],
                            state.retention[NodeAmp], amp_out.l, amp_out.r);
            amp_out *= ctx.eff_pre_retention;
            _buf_amp[n] = amp_out;

            // Filter Stage (immediate 0-sample feed from preamp)
            flt_mods.cutoff_ramp = cached_flt_cutoff_eff;
            flt_mods.clock_step = cached_flt_clk;
            if (state.topology.rungler_triad && frame.osc_mode == Oscillator::ModeRnd) {
                flt_mods.cutoff_ramp += _buf_clk[n] * 0.85f;
            }

            float ping_duck = 1.0f - 0.25f * _filter.PingEnv();
            float flt_in_l = amp_out.l * ping_duck + _buf_inj_flt[n] * 0.35f;
            float flt_in_r = amp_out.r * ping_duck + _buf_inj_flt[n] * 0.35f;

            float clk_flt_gain = flt_mods.clock_step;
            if (ctx.clk_flt_bleed_boost > 0.008f) {
                clk_flt_gain = std::clamp(clk_flt_gain + ctx.clk_flt_bleed_boost, 0.0f, 1.0f);
            }
            if (_buf_clk_beat[n] && (ctx.clk_flt_bleed_boost > 0.01f || state.topology.clk_amp_flt)) {
                _filter.TriggerPing(std::clamp(ctx.clk_flt_bleed_boost * 1.25f, 0.65f, 1.25f));
            }
            if (clk_flt_gain > 0.01f) {
                float clk_punch = _buf_clk[n] * clk_flt_gain * 0.20f;
                flt_in_l += clk_punch;
                flt_in_r += clk_punch;
                flt_mods.cutoff_ramp += _buf_clk[n] * clk_flt_gain * 1.6f;
            }

            if (_buf_cap_osc_flt[n] > 0.0002f) {
                float raw_osc = (frame.osc_mode == Oscillator::ModeRnd && ctx.omods_base.shape_warp < 0.5f)
                                    ? _osc.RndHeld()
                                    : _buf_osc[n];
                float osc_voice = raw_osc * _buf_cap_osc_flt[n] * 0.12f;
                _flt_osc_dc += 0.0024f * (osc_voice - _flt_osc_dc);
                float osc_voice_ac = osc_voice - _flt_osc_dc;
                flt_in_l += osc_voice_ac;
                flt_in_r += osc_voice_ac;
            }

            Stereo flt_out;
            _filter.Process(flt_in_l, flt_in_r,
                            0.0f, flt_mods, _buf_rail_vcc[n], _buf_rail_starve[n],
                            state.retention[NodeFlt], flt_out.l, flt_out.r);
            _buf_flt[n] = flt_out;
            _bus.flt = flt_out;
        }

        uint32_t dt_amp_flt = DWT->CYCCNT - t_stage;
        last_profile.preamp = dt_amp_flt / 2;
        last_profile.filter = dt_amp_flt - last_profile.preamp;

        _bus.amp = _buf_amp[size - 1];
        if (ctx.has_any_mod) {
            if (ctx.uses_preamp)  _bus.sig_array[static_cast<size_t>(Sig::PreampOut)] = _buf_amp[size - 1].l;
            if (ctx.uses_flt_out) _bus.sig_array[static_cast<size_t>(Sig::FltOut)]    = _buf_flt[size - 1].l;
        }

        // =====================================================================
        // STAGE 6: Delay (Memory Machine)
        // =====================================================================
        t_stage = DWT->CYCCNT;
        for (size_t n = 0; n < size; ++n) {
            cur_dly_clk_fm += ctx.inc_dly_clk_fm;
            if (__builtin_expect(ctx.has_any_mod, 0)) {
                switch (n & 3) {
                case 1:
                    if (ctx.has_mod_dly_clk_fm) cached_dly_clk_fm = matrix.Eval(kModDlyClockFm, 0.0f, _bus.sig_array);
                    if (ctx.has_mod_dly_phase)  cached_dly_phase  = matrix.Eval(kModDlyPhaseMod, ctx.amt_dly_phase, _bus.sig_array);
                    if (ctx.has_mod_dly_recirc) cached_dly_recirc = matrix.Eval(kModDlyRecircFm, ctx.amt_dly_recirc, _bus.sig_array);
                    break;
                case 2:
                    if (ctx.has_mod_dly_burst)  cached_dly_burst  = matrix.Eval(kModDlyClockBurst, ctx.amt_dly_burst, _bus.sig_array);
                    break;
                default: break;
                }
            }

            float phase_mod = cached_dly_phase * 320.0f;
            if (cur_dly_clk_fm > 0.005f) {
                if (state.osc_mode == Oscillator::ModeSin) {
                    float sin_wave = FastSin(_buf_osc_phase[n] * kTwoPi);
                    phase_mod += sin_wave * (cur_dly_clk_fm * _cached_clk_vibrato_span);
                } else if (state.osc_mode == Oscillator::ModeSaw) {
                    float saw_phase = _buf_osc_phase[n];
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
            if (state.osc_mode == Oscillator::ModeRnd && cur_dly_clk_fm > 0.01f) {
                dly_clk_fm += _osc.RndHeld() * 0.85f * cur_dly_clk_fm;
            }

            if (state.topology.lissajous_cloud) {
                if ((n & 1) == 0) {
                    float quad_phase = _buf_osc_phase[n] + 0.25f;
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
            float fbk_gain = ctx.base_fbk_gain + burst * 0.45f;

            if (state.topology.clock_siren) {
                float siren_surge = _buf_clk[n] * 0.45f;
                fbk_gain = std::min(1.18f, fbk_gain + siren_surge);
                state.delay_regs.clock.fm += siren_surge * 0.60f;
            }
            if (state.topology.karplus_resonator) {
                fbk_gain = std::max(fbk_gain, 0.985f);
            }
            state.delay_regs.feedback.gain = fbk_gain;

            Stereo mic_to_fbk = Stereo(_buf_mic[n] * ctx.mic_fbk_scale);
            Stereo dly_raw = _buf_flt[n] * 1.0f + mic_to_fbk + Stereo(_buf_inj_dly[n] * 0.20f + _buf_afford_dly[n]);
            if (__builtin_expect(ctx.amp_to_dly > 0.005f, 0)) {
                dly_raw += _buf_amp[n] * ctx.amp_to_dly;
            }

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
            Stereo dly_out = _delay.Process(dly_in, state.delay_regs, _buf_osc[n], _in_env);
            _buf_dly[n] = dly_out;
            _buf_dry[n] = _mixer.ClockMorphActive()
                              ? _delay.ProcessDry(_buf_flt[n], state.delay_regs)
                              : _buf_flt[n];
        }
        _bus.fbk = _buf_dly[size - 1];
        if (ctx.has_any_mod && ctx.uses_dly_out) {
            _bus.sig_array[static_cast<size_t>(Sig::DlyOut)] = _buf_dly[size - 1].l;
        }
        last_profile.delay = DWT->CYCCNT - t_stage;

        // =====================================================================
        // STAGE 7: Mixer, Output Safety Limiter, Bus & History
        // =====================================================================
        t_stage = DWT->CYCCNT;
        float block_peak = 0.0f;
        MixerMods mmods = ctx.mmods;
        float flt_strike = _filter.PingEnv();

        for (size_t n = 0; n < size; ++n) {
            if (__builtin_expect(ctx.has_any_mod, 0)) {
                if ((n & 3) == 3 && ctx.has_mod_mix_trem) cached_mix_trem = matrix.Eval(kModMixTremolo, ctx.amt_mix_trem, _bus.sig_array);
            }
            mmods.tremolo = cached_mix_trem;
            float tap_out_osc_l = _buf_osc[n] * ctx.osc_out_gain;
            float tap_out_osc_r = (state.osc_mode == Oscillator::ModeRnd ? _buf_osc_r[n] : _buf_osc[n]) * ctx.osc_out_gain;

            Stereo mix_out;
            _mixer.Process(_buf_amp[n].l, _buf_amp[n].r, _buf_flt[n].l, _buf_flt[n].r,
                           _buf_dly[n].l, _buf_dly[n].r, _buf_dry[n].l, _buf_dry[n].r,
                           flt_strike, _buf_inj_mix[n] + _buf_afford_osc[n], _buf_inj_mix[n] + _buf_afford_osc[n],
                           tap_out_osc_l, tap_out_osc_r, mmods, ctx.shunt_both,
                           mix_out.l, mix_out.r);
            _buf_mix[n] = mix_out;

            Stereo final_sig = _output.Process(mix_out, _buf_rail_vcc[n], boot_gain);
            out_l[n] = final_sig.l;
            out_r[n] = final_sig.r;

            _prev_buf_flt[n] = _buf_flt[n];
            _prev_buf_fbk[n] = _buf_dly[n];
            _prev_buf_out[n] = final_sig;
            _prev_buf_osc[n] = _buf_osc[n];
            _prev_buf_osc_ramp[n] = _buf_osc_ramp[n];
            _prev_buf_clk[n] = _buf_clk[n];
            _prev_buf_amp[n] = _buf_amp[n];
            _prev_buf_mix[n] = mix_out;
            _prev_buf_in[n]  = _buf_in_combined[n];

            // Envelope followers (decimated)
            if ((n & 3) == 0) {
                float in_mag = std::abs(_buf_in_combined[n].l + _buf_in_combined[n].r) * 0.5f;
                _in_env += (in_mag > _in_env ? 0.30f : 0.008f) * (in_mag - _in_env);
                float out_mag = std::abs(final_sig.l + final_sig.r) * 0.5f;
                _out_env += (out_mag > _out_env ? 0.20f : 0.008f) * (out_mag - _out_env);
                float dly_mag = std::abs(_buf_dly[n].l + _buf_dly[n].r) * 0.5f;
                _dly_env += (dly_mag > _dly_env ? 0.22f : 0.008f) * (dly_mag - _dly_env);
            }

            float p = std::max(final_sig.MaxAbs(), std::max(_buf_flt[n].MaxAbs(), _buf_dly[n].MaxAbs()));
            if (p > block_peak) block_peak = p;
        }

        uint32_t dt_mix_out = DWT->CYCCNT - t_stage;
        last_profile.mixer = dt_mix_out / 2;
        last_profile.output = dt_mix_out - last_profile.mixer;

        _bus.out_sig = Stereo(out_l[size - 1], out_r[size - 1]);
        _bus.in_sig  = _buf_in_combined[size - 1];
        _bus.in_env  = _in_env;
        _bus.out_env = _out_env;
        _bus.dly_env = _dly_env;

        if (ctx.has_any_mod) {
            _bus.sig_array[static_cast<size_t>(Sig::ClkPulse)] = _buf_clk[size - 1];
            if (ctx.uses_in_audio)  _bus.sig_array[static_cast<size_t>(Sig::InAudio)]   = _buf_in_combined[size - 1].l;
            if (ctx.uses_out_audio) {
                _bus.sig_array[static_cast<size_t>(Sig::OutAudio)]  = out_l[size - 1];
                _bus.sig_array[static_cast<size_t>(Sig::MasterOut)] = out_l[size - 1];
            }
            if (ctx.uses_in_env)    _bus.sig_array[static_cast<size_t>(Sig::InEnv)]     = _in_env;
            if (ctx.uses_out_env)   _bus.sig_array[static_cast<size_t>(Sig::OutEnv)]    = _out_env;
            if (ctx.uses_dly_env)   _bus.sig_array[static_cast<size_t>(Sig::DlyEnv)]    = _dly_env;
            if (ctx.uses_hum)       _bus.sig_array[static_cast<size_t>(Sig::Hum)]       = _buf_hum[size - 1];
            if (ctx.uses_mic)       _bus.sig_array[static_cast<size_t>(Sig::ContactMic)] = _buf_mic[size - 1];
        }

        FinalizeBlockTelemetry(cur_flt_cutoff, cur_dly_clk_fm, block_peak);
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
    static constexpr size_t kMaxBlockSize = 64;

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

    // Working buffers for stage-by-stage block processing
    alignas(16) Stereo _buf_in_combined[kMaxBlockSize];
    alignas(16) Stereo _buf_amp[kMaxBlockSize];
    alignas(16) Stereo _buf_flt[kMaxBlockSize];
    alignas(16) Stereo _buf_dly[kMaxBlockSize];
    alignas(16) Stereo _buf_dry[kMaxBlockSize];
    alignas(16) Stereo _buf_mix[kMaxBlockSize];
    alignas(16) Stereo _buf_loop_sum[kMaxBlockSize];

    alignas(16) float  _buf_clk[kMaxBlockSize];
    alignas(16) bool   _buf_clk_sq[kMaxBlockSize];
    alignas(16) bool   _buf_clk_beat[kMaxBlockSize];
    alignas(16) float  _buf_osc[kMaxBlockSize];
    alignas(16) float  _buf_osc_r[kMaxBlockSize];
    alignas(16) float  _buf_osc_ramp[kMaxBlockSize];
    alignas(16) float  _buf_osc_phase[kMaxBlockSize];
    alignas(16) float  _buf_hum[kMaxBlockSize];
    alignas(16) float  _buf_mic[kMaxBlockSize];
    alignas(16) float  _buf_rail_vcc[kMaxBlockSize];
    alignas(16) float  _buf_rail_starve[kMaxBlockSize];
    alignas(16) float  _buf_afford_osc[kMaxBlockSize];
    alignas(16) float  _buf_afford_dly[kMaxBlockSize];
    alignas(16) float  _buf_cap_osc_amp[kMaxBlockSize];
    alignas(16) float  _buf_cap_osc_flt[kMaxBlockSize];
    alignas(16) float  _buf_inj_osc[kMaxBlockSize];
    alignas(16) float  _buf_inj_amp[kMaxBlockSize];
    alignas(16) float  _buf_inj_flt[kMaxBlockSize];
    alignas(16) float  _buf_inj_dly[kMaxBlockSize];
    alignas(16) float  _buf_inj_mix[kMaxBlockSize];
    alignas(16) float  _buf_intermod[kMaxBlockSize];

    // Feedback history from previous block (for inter-stage feedback into Preamp and AC cross-injection)
    alignas(16) Stereo _prev_buf_flt[kMaxBlockSize];
    alignas(16) Stereo _prev_buf_fbk[kMaxBlockSize];
    alignas(16) Stereo _prev_buf_out[kMaxBlockSize];
    alignas(16) Stereo _prev_buf_amp[kMaxBlockSize];
    alignas(16) Stereo _prev_buf_mix[kMaxBlockSize];
    alignas(16) Stereo _prev_buf_in[kMaxBlockSize];
    alignas(16) float  _prev_buf_osc[kMaxBlockSize];
    alignas(16) float  _prev_buf_osc_ramp[kMaxBlockSize];
    alignas(16) float  _prev_buf_clk[kMaxBlockSize];
};

} // namespace bends
