#pragma once

#include "dsp/fast_math.h"
#include "dsp/filters.h"
#include "bends/mod_pack.h"
#include "circuit/health.h"
#include <cmath>
#include <algorithm>

namespace bends {

class Filter {
public:
    struct PingExciter {
        bool     active{false};
        uint16_t sample_idx{0};
        uint16_t total_samples{24};
        float    amplitude{0.0f};
        float    phase{0.0f};
        float    phase_inc{0.0f};

        void Trigger(float vel) {
            vel = std::clamp(vel, 0.15f, 1.20f);
            active = true;
            sample_idx = 0;
            total_samples = 20; // ~0.42 ms exciter pulse
            phase_inc = kPi / 20.0f;
            phase = 0.0f;
            amplitude = vel * 0.85f;
        }

        __attribute__((always_inline)) inline float Tick() {
            if (!active) return 0.0f;
            sample_idx++;
            if (sample_idx >= total_samples) {
                active = false;
                return 0.0f;
            }
            phase += phase_inc;
            return FastSin(phase) * amplitude;
        }

        void Reset() {
            active = false;
            sample_idx = 0;
            phase = 0.0f;
            amplitude = 0.0f;
        }
    };

    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        _inv_sr = kPi / sample_rate;
        Reset();
    }

    void Reset() {
        _s1_l = 0.0f; _s2_l = 0.0f;
        _s1_r = 0.0f; _s2_r = 0.0f;
        _ping_env = 0.0f;
        _pitch_dive = 0.0f;
        _cur_base_f = 18000.0f;
        _target_base_f = 18000.0f;
        _base_f_inc = 0.0f;
        _cur_low_mix = 1.0f;
        _target_low_mix = 1.0f;
        _low_mix_inc = 0.0f;
        _cur_high_mix = 0.0f;
        _target_high_mix = 0.0f;
        _high_mix_inc = 0.0f;
        _cur_dry_mix = 0.0f;
        _target_dry_mix = 0.0f;
        _dry_mix_inc = 0.0f;
        _cached_r = 0.707f;
        _cached_min_R = 0.055f;
        _dc_out_l.Reset();
        _dc_out_r.Reset();
        _exciter.Reset();
        _post_gain = 1.0f; _target_post_gain = 1.0f;
        _post_drive = 1.0f; _target_post_drive = 1.0f;
        _smooth_wavefold = 0.0f;
        _flt_decim = 0;
        _cached_g_l = 0.1f; _cached_g_r = 0.1f;
        _cached_R_l = 0.707f; _cached_R_r = 0.707f;
        _cached_h_l = 0.5f; _cached_h_r = 0.5f;
        _cached_r2_g_l = 1.4f; _cached_r2_g_r = 1.4f;
    }

    inline float PingEnv() const { return _ping_env; }

    void TriggerPing(float strength) {
        _exciter.Trigger(strength);
        _ping_env = std::clamp(strength, 0.40f, 1.20f);
        _pitch_dive = 1.0f;
    }

    inline void InjectSawPing(float amount) {
        float safe = daisysp::fclamp(amount, 0.0f, 0.50f);
        _s1_l = daisysp::fclamp(_s1_l + safe, -6.0f, 6.0f);
        _s1_r = daisysp::fclamp(_s1_r + safe, -6.0f, 6.0f);
        _ping_env = std::max(_ping_env, std::min(1.0f, safe * 4.0f));
    }

    inline void PrepareBlock(float knob_flt, float node_pull, float touch_flt, float knob_amp = 0.5f,
                             float v_rail_eff = 1.0f, float starve = 0.0f, float knob_clk = 1.0f,
                             const FilterMods& mods = {}, float override_cutoff_hz = 0.0f) {
        float flt_dc = std::clamp(knob_flt + node_pull * 0.20f, 0.0f, 1.0f);

        // Continuous Single-Stage Stereo SVF with Tone Crossfade:
        // Knob CCW (< 0.46): Resonant Low-Pass sweeping 30 Hz -> 18,000 Hz, output low
        // Deadzone (0.46 - 0.54): Transparent flat pass across 12:00 detent, crossfading low -> dry -> high
        // Knob CW (> 0.54): Resonant High-Pass sweeping 20 Hz -> 3,600 Hz, output high
        float target_f;
        float target_low, target_high;
        float t_res;

        if (override_cutoff_hz > 0.0f) {
            target_f = std::clamp(override_cutoff_hz, 25.0f, 18000.0f);
            target_low = 1.0f;
            target_high = 0.0f;
            t_res = 1.0f;
        } else if (flt_dc <= 0.46f) {
            float t = flt_dc / 0.46f;
            target_f = 30.0f * FastPow2(t * 9.23f); // 30 Hz to ~18,000 Hz
            target_low = 1.0f;
            target_high = 0.0f;
            // Taper resonance at high frequencies
            if (target_f > 4500.0f) {
                float hf_taper = std::max(0.18f, 1.0f - (target_f - 4500.0f) / 7500.0f);
                t_res = hf_taper * hf_taper;
            } else {
                t_res = 1.0f;
            }
        } else if (flt_dc >= 0.54f) {
            float t = (flt_dc - 0.54f) / 0.46f;
            target_f = 20.0f * FastPow2(t * 7.49f); // 20 Hz to ~3,600 Hz
            target_low = 0.0f;
            target_high = 0.88f; // Gentle acoustic trim so bright HP doesn't overpower warm LP
            // High-pass resonance scaling:
            t_res = 0.85f;
            if (target_f < 80.0f) {
                t_res *= std::max(0.0f, (target_f - 20.0f) / 60.0f);
            } else if (target_f > 1200.0f) {
                t_res *= std::max(0.35f, 1.0f - 0.65f * ((target_f - 1200.0f) / 2400.0f));
            }
        } else {
            // Deadzone [0.46, 0.54]:
            t_res = 0.0f; // Neutral Butterworth (R = 0.707) across center detent
            if (flt_dc < 0.49f) {
                target_f = 18000.0f;
                float blend = (flt_dc - 0.46f) / 0.03f;
                target_low = 1.0f - blend;
                target_high = 0.0f;
            } else if (flt_dc <= 0.51f) {
                // Pure dry bypass at 12:00 detent
                target_f = 18000.0f;
                target_low = 0.0f;
                target_high = 0.0f;
                // Decay state to 0 so when crossing to HP it starts fresh and click-free
                _s1_l *= 0.80f; _s2_l *= 0.80f;
                _s1_r *= 0.80f; _s2_r *= 0.80f;
            } else {
                target_f = 20.0f;
                float blend = (flt_dc - 0.51f) / 0.03f;
                target_low = 0.0f;
                target_high = blend * 0.88f;
            }
        }

        _target_base_f = Sanitize(target_f, 18000.0f);
        _target_low_mix = target_low;
        _target_high_mix = target_high;
        _target_dry_mix = std::max(0.0f, 1.0f - target_low - target_high);

        // 64-sample linear parameter ramp
        _base_f_inc = (_target_base_f - _cur_base_f) * (1.0f / 64.0f);
        _low_mix_inc = (_target_low_mix - _cur_low_mix) * (1.0f / 64.0f);
        _high_mix_inc = (_target_high_mix - _cur_high_mix) * (1.0f / 64.0f);
        _dry_mix_inc = (_target_dry_mix - _cur_dry_mix) * (1.0f / 64.0f);

        _cached_v_rail_eff = v_rail_eff;
        _cached_starve = starve;

        // Voltage sag: cutoff scales down from 1.0x to 0.25x under starvation
        float v_norm = daisysp::fclamp(v_rail_eff, 0.0f, 1.0f);
        _flt_v_scale = 0.25f + 0.75f * (v_norm * (1.5f - 0.5f * v_norm));

        // Clock filter tracking: bandwidth scales down to 0.55x with lower clock
        float clk_scale = 0.55f + 0.45f * (knob_clk * (1.5f - 0.5f * knob_clk));
        _flt_v_scale *= clk_scale;

        float flt_headroom = (v_rail_eff > 0.45f) ? (1.50f + 0.30f * ((v_rail_eff - 0.45f) / 0.55f))
                                                  : (1.50f * (v_rail_eff / 0.45f));

        float h = flt_headroom * (1.0f - 0.40f * starve);
        _cached_h_pos = std::max(0.001f, h * (1.0f - 0.20f * starve));
        _cached_h_neg = std::max(0.001f, h * (1.0f + 0.08f * starve));
        _cached_inv_h_pos = 1.0f / _cached_h_pos;
        _cached_inv_h_neg = 1.0f / _cached_h_neg;
        _cached_starve_t = std::min(1.0f, starve * 20.0f);

        if (starve > 0.40f) {
            _cached_deadzone = 0.05f * (starve - 0.40f) * 1.6667f;
            _cached_inv_deadzone = (_cached_deadzone > 0.001f) ? (1.0f / _cached_deadzone) : 0.0f;
            _cached_has_deadzone = (_cached_deadzone > 0.001f);
        } else {
            _cached_deadzone = 0.0f;
            _cached_inv_deadzone = 0.0f;
            _cached_has_deadzone = false;
        }

        // Active resonance calculation:
        float res = daisysp::fclamp(touch_flt + mods.resonance, 0.0f, 1.0f) * t_res;
        float inv_res = 1.0f - res;
        bool is_hp = (flt_dc >= 0.52f);
        float r_min = is_hp ? ((mods.howl > 0.01f) ? 0.14f : 0.18f)
                            : ((mods.howl > 0.01f) ? 0.022f : 0.065f);
        float target_r = r_min + (0.707f - r_min) * (inv_res * inv_res);
        _cached_r += 0.20f * (target_r - _cached_r);
        _cached_min_R = is_hp ? ((mods.howl > 0.01f) ? 0.13f : 0.17f)
                              : ((mods.howl > 0.01f) ? 0.018f : 0.055f);

        // Rail starve damping adjustment
        _cached_starve_damping = (v_rail_eff < 0.25f) ? (0.25f - v_rail_eff) * 0.10f : 0.0f;
        _cached_drive = 1.0f + starve * 0.45f;

        // Post-filter soft saturation / level control (controlled by S30 AMP)
        float amp_u = std::clamp(knob_amp, 0.0f, 1.0f);
        if (amp_u <= 0.50f) {
            float t = amp_u / 0.50f;
            _target_post_gain = 0.50f + 0.50f * (t * t);
            _target_post_drive = 1.0f;
        } else {
            float t = (amp_u - 0.50f) / 0.50f;
            _target_post_gain = 1.0f;
            _target_post_drive = 1.0f + 1.25f * t;
        }
        _post_gain += 0.20f * (_target_post_gain - _post_gain);
        _post_drive += 0.20f * (_target_post_drive - _post_drive);
    }

    __attribute__((noinline)) void Process(float in_l, float in_r,
                        float touch_flt,
                        const FilterMods& mods,
                        float v_rail_eff, float starve,
                        float shunt_gnd,
                        float& out_l, float& out_r) {
        // Transient pitch decay (~28ms)
        if (_pitch_dive > 0.001f) {
            _pitch_dive *= 0.9968f; // ~28ms decay @ 48kHz
        } else {
            _pitch_dive = 0.0f;
        }

        // Ping envelope decay (~260ms)
        float decay_rate = 0.99962f;
        if (_ping_env > 0.0001f) {
            _ping_env *= decay_rate;
        } else {
            _ping_env = 0.0f;
        }

        // Excitation pulse
        if (__builtin_expect(_exciter.active, 0)) {
            constexpr float exc_scale = 0.85f;
            float exc = _exciter.Tick() * exc_scale;
            in_l += exc;
            in_r += exc;
        }

        // Linear parameter interpolation
        _cur_base_f += _base_f_inc;
        _cur_low_mix += _low_mix_inc;
        _cur_high_mix += _high_mix_inc;
        _cur_dry_mix += _dry_mix_inc;

        // Center detent bypass when filter is fully open / dry
        if (__builtin_expect(_cur_dry_mix >= 0.999f && mods.wavefold <= 0.01f && _ping_env <= 0.0001f, 0)) {
            _s1_l *= 0.90f; _s2_l *= 0.90f;
            _s1_r *= 0.90f; _s2_r *= 0.90f;
            float flt_l = _dc_out_l.Process(in_l);
            float flt_r = _dc_out_r.Process(in_r);
            float out_sig_l = Saturate(flt_l) * shunt_gnd;
            float out_sig_r = Saturate(flt_r) * shunt_gnd;
            if (_post_drive > 1.01f) {
                out_sig_l = FastSoftSat(out_sig_l * (_post_drive * 0.65f));
                out_sig_r = FastSoftSat(out_sig_r * (_post_drive * 0.65f));
            } else if (_post_gain < 0.99f) {
                out_sig_l *= _post_gain;
                out_sig_r *= _post_gain;
            }
            out_l = std::clamp(out_sig_l, -1.5f, 1.5f);
            out_r = std::clamp(out_sig_r, -1.5f, 1.5f);
            return;
        }

        // 12 kHz decimation for SVF coefficient pre-warping and denominator division
        _flt_decim = (_flt_decim + 1) & 3;
        if (__builtin_expect(_flt_decim == 0, 0)) {
            float total_cutoff_mod = std::clamp(mods.cutoff_ramp + mods.clock_step * 1.5f, -4.0f, 4.0f);
            float drum_punch = 1.0f + _pitch_dive * 1.85f;
            float base = _cur_base_f * _flt_v_scale * drum_punch;
            if (__builtin_expect(std::abs(total_cutoff_mod) > 0.003f, 0)) {
                base *= FastPow2(total_cutoff_mod);
            }

            float detune_spread = touch_flt * 0.035f;
            float freq_l = std::clamp(base * (1.0f - detune_spread), 20.0f, 18000.0f);
            float g_l = std::clamp(FastTan(freq_l * _inv_sr), 0.001f, 2.5f);
            float g_r = g_l;
            if (__builtin_expect(detune_spread > 0.0001f, 0)) {
                float freq_r = std::clamp(base * (1.0f + detune_spread), 20.0f, 18000.0f);
                g_r = std::clamp(FastTan(freq_r * _inv_sr), 0.001f, 2.5f);
            }
            _cached_g_l = g_l;
            _cached_g_r = g_r;

            // Damping & resonance
            float r_ping = 0.040f;
            float ping_fade = 1.0f - _ping_env;
            float r_ping_eff = r_ping + (0.707f - r_ping) * ping_fade;

            float R = std::min(r_ping_eff, _cached_r);
            R = std::max(_cached_min_R, R);

            float hf_loss_l = g_l * g_l * 0.22f;
            float hf_loss_r = g_r * g_r * 0.22f;

            float band_sq_l = _s1_l * _s1_l * 0.05f;
            float band_sq_r = _s1_r * _s1_r * 0.05f;
            float diode_l = std::min(0.35f, band_sq_l);
            float diode_r = std::min(0.35f, band_sq_r);

            float R_l = R + hf_loss_l + diode_l + _cached_starve_damping;
            float R_r = R + hf_loss_r + diode_r + _cached_starve_damping;

            R_l = 0.707f + (R_l - 0.707f) * shunt_gnd;
            R_r = 0.707f + (R_r - 0.707f) * shunt_gnd;

            if (mods.howl > 0.08f && v_rail_eff < health::kTired) {
                R_l = std::max(0.040f, R_l - mods.howl * 0.30f);
                R_r = std::max(0.040f, R_r - mods.howl * 0.30f);
            }
            _cached_R_l = R_l;
            _cached_R_r = R_r;

            _cached_h_l = 1.0f / (1.0f + 2.0f * R_l * g_l + g_l * g_l);
            _cached_r2_g_l = 2.0f * R_l + g_l;

            if (__builtin_expect(detune_spread > 0.0001f, 0)) {
                _cached_h_r = 1.0f / (1.0f + 2.0f * R_r * g_r + g_r * g_r);
                _cached_r2_g_r = 2.0f * R_r + g_r;
            } else {
                _cached_h_r = _cached_h_l;
                _cached_r2_g_r = _cached_r2_g_l;
            }
        }

        float g_l = _cached_g_l;
        float g_r = _cached_g_r;
        float h_l = _cached_h_l;
        float h_r = _cached_h_r;
        float r2_g_l = _cached_r2_g_l;
        float r2_g_r = _cached_r2_g_r;

        if (mods.howl > 0.08f && v_rail_eff < health::kTired) {
            in_l += starve * mods.howl * 0.15f;
            in_r += starve * mods.howl * 0.15f;
        }

        // Input saturation under starvation
        if (__builtin_expect(starve > 0.01f, 0)) {
            in_l = (in_l + starve * 0.20f * in_l * std::abs(in_l)) * _cached_drive;
            in_r = (in_r + starve * 0.20f * in_r * std::abs(in_r)) * _cached_drive;
        }

        // Sine wavefolder
        _smooth_wavefold += 0.04f * (mods.wavefold - _smooth_wavefold);
        if (__builtin_expect(_smooth_wavefold > 0.005f, 0)) {
            float fold_gain = 1.0f + _smooth_wavefold * 2.8f;
            float folded_l = FastSin(in_l * fold_gain * 1.5707963f);
            float folded_r = FastSin(in_r * fold_gain * 1.5707963f);
            in_l = in_l * (1.0f - _smooth_wavefold) + folded_l * _smooth_wavefold;
            in_r = in_r * (1.0f - _smooth_wavefold) + folded_r * _smooth_wavefold;
        }

        // Sub-audible thermal Johnson seed (-100 dBFS):
        if (mods.howl > 0.08f && _cached_r < 0.035f && std::abs(_s1_l) < 1.0e-5f) {
            _s1_l += 1.0e-5f;
            _s1_r += 1.0e-5f;
        }

        // Stereo SVF with non-linear integrator saturation:
        // Gives the filter warm, fat, singing, gooey analog character, especially when starved!
        float band_sat_scale = 0.85f + 0.65f * starve;
        float high_l = (in_l - r2_g_l * _s1_l - _s2_l) * h_l;
        float v1_l = g_l * high_l;
        float band_l = v1_l + _s1_l;
        float band_sat_l = FastSoftSat(band_l * band_sat_scale);
        float v2_l = g_l * band_sat_l;
        float low_l = v2_l + _s2_l;
        _s1_l = LimitState(2.0f * v1_l + _s1_l);
        _s2_l = LimitState(2.0f * v2_l + _s2_l);

        float high_r = (in_r - r2_g_r * _s1_r - _s2_r) * h_r;
        float v1_r = g_r * high_r;
        float band_r = v1_r + _s1_r;
        float band_sat_r = FastSoftSat(band_r * band_sat_scale);
        float v2_r = g_r * band_sat_r;
        float low_r = v2_r + _s2_r;
        _s1_r = LimitState(2.0f * v1_r + _s1_r);
        _s2_r = LimitState(2.0f * v2_r + _s2_r);

        // Continuous tone selection: Low-Pass <-> Dry Bypass <-> High-Pass
        float flt_l = _cur_low_mix * low_l + _cur_high_mix * high_l + _cur_dry_mix * in_l;
        float flt_r = _cur_low_mix * low_r + _cur_high_mix * high_r + _cur_dry_mix * in_r;

        // AC-couple filter output to block DC offsets
        flt_l = _dc_out_l.Process(flt_l);
        flt_r = _dc_out_r.Process(flt_r);

        float out_sig_l = Saturate(flt_l);
        float out_sig_r = Saturate(flt_r);

        out_sig_l *= shunt_gnd;
        out_sig_r *= shunt_gnd;

        // Post-filter soft saturation (controlled by S30 AMP)
        if (_post_drive > 1.01f) {
            out_sig_l = FastSoftSat(out_sig_l * _post_drive);
            out_sig_r = FastSoftSat(out_sig_r * _post_drive);
        } else if (_post_gain < 0.99f) {
            out_sig_l *= _post_gain;
            out_sig_r *= _post_gain;
        }

        out_l = std::clamp(out_sig_l, -1.5f, 1.5f);
        out_r = std::clamp(out_sig_r, -1.5f, 1.5f);
    }

private:
    __attribute__((always_inline)) static inline float LimitState(float x) {
        return std::clamp(x, -6.0f, 6.0f);
    }

    __attribute__((always_inline)) inline float Saturate(float x) const {
        float mag = std::abs(x);
        if (_cached_starve < 0.001f) {
            if (mag <= 1.05f) return x;
            constexpr float kSpan = 0.35f;
            constexpr float kInvSpan = 1.0f / kSpan;
            float s = 1.05f + kSpan * FastSoftSat((mag - 1.05f) * kInvSpan);
            return (x >= 0.0f) ? s : -s;
        }

        float inv_h = (x >= 0.0f) ? _cached_inv_h_pos : _cached_inv_h_neg;
        float h = (x >= 0.0f) ? _cached_h_pos : _cached_h_neg;
        float norm = mag * inv_h;
        float sat;
        if (__builtin_expect(norm <= 0.15f, 1)) {
            sat = x;
        } else {
            float s = h * FastSoftSat(norm);
            sat = (x >= 0.0f) ? s : -s;
        }

        if (__builtin_expect(_cached_has_deadzone && std::abs(sat) < _cached_deadzone, 0)) {
            float sgn = (sat >= 0.0f) ? 1.0f : -1.0f;
            sat = sgn * (sat * sat) * _cached_inv_deadzone;
        }
        return x + _cached_starve_t * (sat - x);
    }

    float _sample_rate{48000.0f};
    float _inv_sr{kPi / 48000.0f};
    float _s1_l{0.0f}; float _s2_l{0.0f};
    float _s1_r{0.0f}; float _s2_r{0.0f};
    float _ping_env{0.0f};
    float _pitch_dive{0.0f};
    float _flt_v_scale{1.0f};

    float _cur_base_f{18000.0f};
    float _target_base_f{18000.0f};
    float _base_f_inc{0.0f};

    float _cur_low_mix{1.0f};
    float _target_low_mix{1.0f};
    float _low_mix_inc{0.0f};

    float _cur_high_mix{0.0f};
    float _target_high_mix{0.0f};
    float _high_mix_inc{0.0f};

    float _cur_dry_mix{0.0f};
    float _target_dry_mix{0.0f};
    float _dry_mix_inc{0.0f};

    float _cached_v_rail_eff{1.0f};
    float _cached_starve{0.0f};
    float _cached_h_pos{1.5f};
    float _cached_h_neg{1.5f};
    float _cached_inv_h_pos{1.0f / 1.5f};
    float _cached_inv_h_neg{1.0f / 1.5f};
    float _cached_deadzone{0.0f};
    float _cached_inv_deadzone{0.0f};
    bool  _cached_has_deadzone{false};
    float _cached_starve_t{0.0f};
    float _cached_r{0.707f};
    float _cached_min_R{0.055f};
    float _cached_starve_damping{0.0f};
    float _cached_drive{1.0f};

    DcBlocker _dc_out_l;
    DcBlocker _dc_out_r;

    float _post_gain{1.0f};
    float _target_post_gain{1.0f};
    float _post_drive{1.0f};
    float _target_post_drive{1.0f};

    PingExciter _exciter;
    uint8_t _flt_decim{0};
    float _cached_g_l{0.1f}, _cached_g_r{0.1f};
    float _cached_R_l{0.707f}, _cached_R_r{0.707f};
    float _cached_h_l{0.5f}, _cached_h_r{0.5f};
    float _cached_r2_g_l{1.4f}, _cached_r2_g_r{1.4f};
    float _smooth_wavefold{0.0f};
};

} // namespace bends
