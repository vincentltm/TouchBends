#pragma once

#include "dsp/fast_math.h"
#include "bends/mod_pack.h"
#include "circuit/health.h"
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace bends {

class Oscillator {
public:
    enum Mode : uint8_t {
        ModeSin = 0,
        ModeSaw = 1,
        ModeRnd = 2
    };

    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        _inv_sample_rate = (sample_rate > 0.0f) ? (1.0f / sample_rate) : (1.0f / 48000.0f);
        Reset();
    }

    void Reset() {
        _phase = 0.0f;
        _saw_ramp = 0.0f;
        _saw_reset_pulse = 0.0f;
        _lfsr = 0xACE1u;
        _rungler_sr = 0x55AAu;
        _rnd_held = 0.0f;
        _rnd_held_r = 0.0f;
        _sub_state = false;
        _base_freq = 110.0f;
        _curr_freq = 110.0f;
        _cached_target_freq = 110.0f;
        _slewed_sh = 0.0f;
        _slewed_sh_r = 0.0f;
        _chime_idx = 0;
        _drift_phase = 0.0f;
        _drift_smooth = 0.0f;
        _snap_val = 0.0f;
        _snap_env = 0.0f;
        _smooth_noise_lp1 = 0.0f;
        _smooth_noise_lp2 = 0.0f;
        _noise_step = 0;
        _cached_noise_alpha = 0.05f;
        _cached_noise_makeup = 3.16f;
        _rnd_step = 0;
        _rnd_step_r = 0;
        _prev_sync = false;
        _prng = 0x7A3B19E4u;
    }

    inline void PrepareBlock(float knob_osc) {
        float u = std::clamp(knob_osc, 0.0f, 1.0f);
        float octaves = u * (22.57f - 10.02f * u);
        _base_freq = std::clamp(0.25f * FastPow2(octaves), 0.1f, 1600.0f);
        _base_freq = Sanitize(_base_freq, 110.0f);
    }

    inline void PrepareBlockFreq(float freq_hz) {
        _base_freq = std::clamp(freq_hz, 0.1f, 4000.0f);
        _base_freq = Sanitize(_base_freq, 110.0f);
    }

    inline void PrepareBlock(float knob_osc, float /*node_pull*/) {
        PrepareBlock(knob_osc);
    }

    float BaseFreq() const { return _base_freq; }
    float Phase() const { return _phase; }
    float RndHeld() const { return _slewed_sh; }
    float RndHeldR() const { return _slewed_sh_r; }
    uint8_t RndStep() const { return _rnd_step; }
    uint8_t RndStepR() const { return _rnd_step_r; }

    inline void ResetPhase() {
        _phase = 0.0f;
        _sub_state = false;
        _saw_reset_pulse = 1.0f;
        _snap_env = 1.0f;
        _snap_val = -0.16f;
    }

    __attribute__((always_inline)) inline void Tick(uint8_t mode, float v_rail_eff, float clk_vcc_sag,
                     float fm_input, const OscillatorMods& mods,
                     float& out_osc, float& out_ramp, float& out_reset_pulse, float& out_sub,
                     bool clk_beat, bool clk_square = false, float ext_audio = 0.0f) {
        float dt = _inv_sample_rate;

        v_rail_eff = std::clamp(v_rail_eff, 0.0f, 1.0f);
        clk_vcc_sag = std::clamp(clk_vcc_sag, 0.1f, 1.0f);

        // Thermal drift (~0.08%, ~1.4 cents)
        if ((_drift_cnt++ & 15) == 0) {
            _drift_phase += (0.38f * 16.0f) * dt;
            if (_drift_phase >= 1.0f) _drift_phase -= 1.0f;

            _prng ^= _prng << 13; _prng ^= _prng >> 17; _prng ^= _prng << 5;
            _cached_thermal_noise = static_cast<float>(static_cast<int32_t>(_prng)) * (1.0f / 2147483648.0f);
            _drift_smooth += 0.0048f * (_cached_thermal_noise - _drift_smooth);
            _cached_analog_drift = 1.0f + 0.0007f * FastSin(_drift_phase * kTwoPi) + 0.0005f * _drift_smooth;
            _cached_target_freq = std::clamp(_base_freq * clk_vcc_sag * _cached_analog_drift, 0.1f, 4000.0f);
        }
        float thermal_noise = _cached_thermal_noise;

        // 1-pole pitch slew (~3.5ms time constant)
        _curr_freq += 0.006f * (_cached_target_freq - _curr_freq);
        float freq = _curr_freq;

        if (mods.chime > 0.01f) {
            if (clk_beat) {
                _chime_idx = (_chime_idx + 1) % 6;
            }
            static constexpr float kPenta[6] = { 1.0f, 1.125f, 1.25f, 1.50f, 1.6667f, 2.0f };
            float ch = std::min(1.0f, mods.chime);
            freq *= (1.0f - ch) + kPenta[_chime_idx] * ch;
        }

        float total_fm = std::clamp(fm_input * 1.50f, -4.0f, 4.0f);
        _step_tick = (_step_tick + 1) & 3;
        if (_step_tick == 0) {
            _cached_fm_mult = (std::abs(total_fm) > 0.001f) ? FastPow2(total_fm * 0.90f) : 1.0f;
        }
        if (std::abs(total_fm) > 0.001f) {
            freq = std::clamp(freq * _cached_fm_mult, 0.1f, 2400.0f);
        }

        bool cycle_reset = false;
        _saw_reset_pulse = 0.0f;
        float sig = 0.0f;

        // RST <-> OSC bend: Contact bit-tickle mutation in ModeRnd, hard sync punch in Saw/Sin
        bool sync_onset = (mods.sync > 0.05f) && !_prev_sync;
        _prev_sync = (mods.sync > 0.05f);
        if (sync_onset) {
            if (mode == ModeRnd) {
                _rungler_sr ^= static_cast<uint16_t>(1u << (_prng & 7u));
            } else {
                _phase = 0.0f;
                cycle_reset = true;
                _saw_reset_pulse = 1.0f;
            }
        }

        // In ModeRnd with Rungler Lock engaged (CLK <-> OSC), synchronize the stepped melody to clock downbeats
        if (mode == ModeRnd && mods.rungler_lock > 0.05f && clk_beat) {
            _phase = 1.0f;
        }

        _phase += freq * dt;
        if (_phase >= 1.0f) {
            _phase -= 1.0f;
            cycle_reset = true;
            _saw_reset_pulse = 1.0f;

            // Relaxation flyback reset pulse:
            float snap_jitter = 1.0f + 0.18f * thermal_noise;
            _snap_val = -0.16f * snap_jitter;
            _snap_env = 1.0f;

            if (mode == ModeRnd) {
                // Shift register rungler:
                bool bit7 = (_rungler_sr >> 7) & 1u;
                bool bit4 = (_rungler_sr >> 4) & 1u;
                bool osc_bit = _sub_state;
                bool clk_bit = clk_square;
                bool in_bit = (std::abs(ext_audio) > 0.035f);

                bool new_bit;
                if (mods.rungler_lock > 0.05f) {
                    // Loop lock: bit 7 recirculates to bit 0
                    bool mutate = false;
                    if (mods.rungler_lock < 0.75f) {
                        _lfsr = (_lfsr >> 1) ^ (-(_lfsr & 1u) & 0xB400u);
                        mutate = ((_lfsr & 0x1Fu) < static_cast<uint32_t>((0.75f - mods.rungler_lock) * 12.0f));
                    }
                    new_bit = mutate ? !bit7 : bit7;
                    if (in_bit && (_lfsr & 0x07u) == 0) {
                        new_bit = !new_bit;
                    }
                } else {
                    // XOR comparison of oscillator and clock against shift register taps
                    _lfsr = (_lfsr >> 1) ^ (-(_lfsr & 1u) & 0xB400u);
                    bool chaotic_bit = (_lfsr & 1u);
                    bool xor_clocks = osc_bit ^ clk_bit;
                    new_bit = bit7 ^ bit4 ^ xor_clocks ^ chaotic_bit ^ in_bit;
                }

                _rungler_sr = static_cast<uint16_t>((_rungler_sr << 1) | (new_bit ? 1u : 0u));

                // 4-bit ladder DAC decoding with resistor tolerance variation (+/- 3%):
                // Gives each step of the melodic loop a distinct stepped pitch interval
                static constexpr float kLadderDac[16] = {
                    -1.000f, -0.868f, -0.732f, -0.595f,
                    -0.470f, -0.330f, -0.205f, -0.065f,
                     0.068f,  0.195f,  0.335f,  0.465f,
                     0.602f,  0.728f,  0.865f,  1.000f
                };
                uint8_t dac_l = ((_rungler_sr >> 4) & 1u) |
                               (((_rungler_sr >> 5) & 1u) << 1) |
                               (((_rungler_sr >> 6) & 1u) << 2) |
                               (((_rungler_sr >> 7) & 1u) << 3);
                _rnd_step = dac_l & 0x0F;
                _rnd_held = kLadderDac[_rnd_step];

                // Right channel ladder DAC: taps bits 0..3 for stereo variation
                uint8_t dac_r = ((_rungler_sr >> 0) & 1u) |
                               (((_rungler_sr >> 1) & 1u) << 1) |
                               (((_rungler_sr >> 2) & 1u) << 2) |
                               (((_rungler_sr >> 3) & 1u) << 3);
                _rnd_step_r = dac_r & 0x0F;
                _rnd_held_r = kLadderDac[_rnd_step_r];
            } else {
                _lfsr = (_lfsr >> 1) ^ (-(_lfsr & 1u) & 0xB400u);
                _rnd_held = (static_cast<float>(_lfsr % 2001) - 1000.0f) * 0.001f;
                _rnd_held_r = _rnd_held;
                _slewed_sh = _rnd_held;
                _slewed_sh_r = _rnd_held_r;
            }
        } else if (_phase < 0.0f) {
            _phase = 0.0f;
        }

        // Decay capacitive flyback snap (~3 samples duration)
        if (_snap_env > 0.001f) {
            _snap_env *= 0.44f;
        } else {
            _snap_env = 0.0f;
        }

        float osc_v_scale = (v_rail_eff > 0.05f) ? (0.75f + 0.25f * v_rail_eff) : (v_rail_eff * 15.0f);

        if (mode == ModeSaw) {
            float p = _phase;
            // Exponential RC capacitor charging curve:
            // Charging current decreases as capacitor voltage rises toward comparator threshold
            float p_rc = p * (1.14f - 0.14f * p);
            if (v_rail_eff < 0.85f) {
                float starve_curve = (0.85f - v_rail_eff) * 1.50f;
                p_rc = p_rc * (1.0f + starve_curve * (1.0f - p_rc));
            }

            float warp = (mods.shape_warp > 0.01f) ? std::min(1.0f, mods.shape_warp) : 0.0f;

            // VCC <-> OSC Bend: Sawtooth slope inversion
            float eff_ramp = (1.0f - warp) * p_rc + warp * (1.0f - p_rc);
            _saw_ramp = eff_ramp;

            float raw_saw = (2.0f * p_rc) - 1.0f;
            float phase_inc = freq * dt;
            if (p < phase_inc) {
                float t = p / phase_inc;
                raw_saw -= (t + t - t * t - 1.0f);
            } else if (p > 1.0f - phase_inc) {
                float t = (p - 1.0f) / phase_inc;
                raw_saw -= (t * t + t + t + 1.0f);
            }

            // Invert audio saw direction and flyback snap as it morphs into reverse saw
            float morph_dir = 1.0f - 2.0f * warp;
            raw_saw = raw_saw * morph_dir + (_snap_val * _snap_env) * morph_dir;

            sig = raw_saw;
            if (v_rail_eff < health::kBroken) {
                float s_depth = (health::kBroken - v_rail_eff) * (1.0f / health::kBroken);
                sig = FastSoftSat(sig * (1.0f + s_depth * 1.35f));
            }
            sig *= osc_v_scale;
        } else if (mode == ModeSin) {
            float pm = total_fm * 0.35f;
            float s_raw = FastSin((_phase + pm) * kTwoPi);

            // Differential BJT sine shaper harmonic modeling
            float s_even = s_raw * s_raw;
            float s_odd  = s_raw * s_even;
            float s_analog = s_raw + 0.038f * s_even - 0.065f * s_odd;

            // Zero-crossing crossover pinch around 0V:
            if (std::abs(s_analog) < 0.07f) {
                s_analog *= (0.88f + 1.70f * std::abs(s_analog));
            }
            // Center DC offset back to zero and normalize gain
            float s = (s_analog - 0.019f) * 1.04f;

            // VCC <-> OSC Bend: Overdrive sine into saturation
            if (mods.shape_warp > 0.01f) {
                float warp = std::min(1.0f, mods.shape_warp);
                float drive = 1.0f + 14.0f * warp;
                float sqr = FastTanh(s * drive) * 0.95f;
                s = s * (1.0f - warp) + sqr * warp;
            }
            if (v_rail_eff < health::kBroken) {
                float s_depth = (health::kBroken - v_rail_eff) * (1.0f / health::kBroken);
                s = FastSoftSat(s * (1.0f + s_depth * 1.5f));
            }
            sig = s * osc_v_scale;
            _saw_ramp = 0.0f;
        } else {
            // ModeRnd: Shift register DAC output with continuous noise
            _slewed_sh   += 0.22f * (_rnd_held - _slewed_sh);
            _slewed_sh_r += 0.22f * (_rnd_held_r - _slewed_sh_r);
            float s = _slewed_sh;

            // Generate full-rate continuous white noise
            _prng ^= _prng << 13; _prng ^= _prng >> 17; _prng ^= _prng << 5;
            float white = static_cast<float>(static_cast<int32_t>(_prng)) * (1.0f / 2147483648.0f);

            // 2-pole continuous analog lowpass tracking oscillator frequency
            if ((_noise_step++ & 7) == 0) {
                float fc = std::clamp(freq * 2.2f, 0.4f, 16000.0f);
                _cached_noise_alpha = std::clamp(fc * (kTwoPi * dt), 0.0001f, 0.85f);
                _cached_noise_makeup = 1.0f / FastSqrt(std::max(0.0002f, _cached_noise_alpha * 2.0f));
            }
            float alpha = _cached_noise_alpha;
            _smooth_noise_lp1 += alpha * (white - _smooth_noise_lp1);
            _smooth_noise_lp2 += alpha * (_smooth_noise_lp1 - _smooth_noise_lp2);

            // Equal-loudness makeup gain across varying filter bandwidth
            float makeup = _cached_noise_makeup;
            float smooth_noise = daisysp::fclamp(_smooth_noise_lp2 * makeup * 0.80f, -1.0f, 1.0f);

            // VCC <-> OSC Bend: Morph hard stepped S&H / DAC ladder into smooth continuous noise
            if (mods.shape_warp > 0.01f) {
                float warp = std::min(1.0f, mods.shape_warp);
                s = s * (1.0f - warp) + smooth_noise * warp;
                _saw_ramp = (smooth_noise * 0.5f + 0.5f) * warp;
            } else {
                _saw_ramp = 0.0f;
            }

            if (v_rail_eff < health::kBroken) {
                float s_depth = (health::kBroken - v_rail_eff) * (1.0f / health::kBroken);
                s = FastSoftSat(s * (1.0f + s_depth * 1.5f));
            }
            sig = s * osc_v_scale;
        }

        if (cycle_reset) {
            _sub_state = !_sub_state;
        }
        float sub_sig = (_sub_state ? 1.0f : -1.0f) * osc_v_scale;

        if (mods.sub_octave > 0.01f) {
            float sub_amt = std::min(1.0f, mods.sub_octave);
            sig = (1.0f - 0.35f * sub_amt) * sig + (0.50f * sub_amt) * sub_sig;
        }

        out_osc = sig;
        out_ramp = _saw_ramp;
        out_reset_pulse = cycle_reset ? 1.0f : 0.0f;
        out_sub = sub_sig;
    }

private:
    float _sample_rate{48000.0f};
    float _inv_sample_rate{1.0f / 48000.0f};
    float _phase{0.0f};
    float _saw_ramp{0.0f};
    float _saw_reset_pulse{0.0f};
    uint16_t _lfsr{0xACE1u};
    uint16_t _rungler_sr{0x55AAu};
    float   _rnd_held{0.0f};
    float   _rnd_held_r{0.0f};
    bool    _sub_state{false};
    float   _base_freq{110.0f};
    float   _curr_freq{110.0f};
    float   _cached_target_freq{110.0f};
    float   _slewed_sh{0.0f};
    float   _slewed_sh_r{0.0f};
    uint8_t _chime_idx{0};
    uint8_t _step_tick{0};
    float   _cached_fm_mult{1.0f};
    float   _drift_phase{0.0f};
    float   _drift_smooth{0.0f};
    uint8_t _drift_cnt{0};
    float   _cached_analog_drift{1.0f};
    float   _cached_thermal_noise{0.0f};
    float   _snap_val{0.0f};
    float   _snap_env{0.0f};
    float   _smooth_noise_lp1{0.0f};
    float   _smooth_noise_lp2{0.0f};
    uint8_t _rnd_step{0};
    uint8_t _rnd_step_r{0};
    bool    _prev_sync{false};
    uint8_t _noise_step{0};
    float   _cached_noise_alpha{0.05f};
    float   _cached_noise_makeup{3.16f};
    uint32_t _prng{0x7A3B19E4u};
};

} // namespace bends
