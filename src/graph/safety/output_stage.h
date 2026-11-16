#pragma once

#include "dsp/stereo.h"
#include "dsp/filters.h"
#include "tuning/tuning.h"
#include <algorithm>
#include <cstdint>

namespace bends {

class OutputStage {
public:
    static constexpr float kCeiling = tuning::kMasterCeiling; // -1.0 dBFS true peak ceiling (safe for live & headphones)
    static constexpr float kKnee    = tuning::kMasterKnee;    // -3.5 dBFS soft-knee start

    void Init(float sample_rate) {
        _sample_rate = (sample_rate > 1000.0f) ? sample_rate : 48000.0f;
        _bp_harsh.SetBandpass(4800.0f, 1.2f, _sample_rate);
        _prev_vcc = -1.0f;
        _cached_lp_alpha = 0.92f;
        Reset();
    }

    void Reset() {
        _sub_hp_l = 0.0f;
        _sub_hp_r = 0.0f;
        _bp_harsh.Reset();
        _lp_warm_state_l = 0.0f;
        _lp_warm_state_r = 0.0f;
        _harsh_env = 0.0f;
        _lim_gain = 1.0f;
        _crashing = false;
        _crash_len = 64;
        _crash_idx = 0;
        _hist_pos = 0;
        std::memset(_hist, 0, sizeof(_hist));
        _sputter_timer = 0;
        _sputter_on = true;
        _sputter_gate = 1.0f;
        _mute_smooth = 1.0f;
        _vcc_headroom = 1.0f;
        _prev_vcc = -1.0f;
        _crash_active = false;
        _crash_blend = 0.0f;
        _crash_inv_blend = 1.0f;
        _target_mute_gain = 1.0f;
        _step_tick = false;
    }

    inline void PrepareBlock(float vcc,
                             float master_crash_amt = 0.0f,
                             float master_mute_amt = 0.0f,
                             bool touch_out_gnd = false) {
        // Track op-amp supply voltage sag smoothly
        _vcc_headroom = std::clamp(0.70f + 0.30f * vcc, 0.65f, 1.0f);
        _eff_ceil = kCeiling * _vcc_headroom;
        _eff_knee = kKnee * _vcc_headroom;
        _eff_span = std::max(0.01f, _eff_ceil - _eff_knee);
        _eff_inv_span = 1.0f / _eff_span;

        // Precompute master crash blend
        _crash_active = (master_crash_amt > 0.20f);
        if (_crash_active) {
            _crash_blend = std::clamp((master_crash_amt - 0.20f) * 1.6f, 0.0f, 0.90f);
            _crash_inv_blend = 1.0f - _crash_blend;
            if (!_crashing) {
                _crashing = true;
                uint16_t clen = static_cast<uint16_t>(32u + static_cast<uint16_t>((1.0f - std::min(1.0f, master_crash_amt)) * 128.0f));
                _crash_len = std::clamp(clen, static_cast<uint16_t>(16), static_cast<uint16_t>(256));
                _crash_idx = 0;
            }
        } else {
            _crashing = false;
            _crash_blend = 0.0f;
            _crash_inv_blend = 1.0f;
        }

        // Precompute master mute gain: OUT <-> GND is a hard master ground shunt (silence)
        if (master_mute_amt > 0.015f || touch_out_gnd) {
            _target_mute_gain = 0.0f;
        } else {
            _target_mute_gain = 1.0f;
        }

        // Op-amp bandwidth roll-off under starve: 19.5 kHz nominal down to ~5.5 kHz at minimum rail
        if (std::abs(vcc - _prev_vcc) > 0.005f) {
            _prev_vcc = vcc;
            float fc = daisysp::fclamp(19500.0f * (0.35f + 0.65f * vcc), 5500.0f, 19500.0f);
            _cached_lp_alpha = daisysp::fclamp(0.25f + 0.67f * (fc / 19500.0f), 0.35f, 0.92f);
        }
    }

    __attribute__((always_inline)) inline Stereo Process(Stereo sig,
                                                         float vcc,
                                                         float boot_gain) {
        // Subsonic highpass (22 Hz)
        _sub_decim = !_sub_decim;
        if (_sub_decim) {
            _sub_hp_l += 0.00576f * (sig.l - _sub_hp_l);
            _sub_hp_r += 0.00576f * (sig.r - _sub_hp_r);
        }
        sig.l -= _sub_hp_l;
        sig.r -= _sub_hp_r;

        // Digital fracture / crash effect
        if (_crash_active) {
            uint8_t idx = static_cast<uint8_t>(_hist_pos - _crash_len + _crash_idx);
            float cl = _hist[0][idx];
            float cr = _hist[1][idx];
            if (++_crash_idx >= _crash_len) _crash_idx = 0;

            int16_t ql = static_cast<int16_t>(daisysp::fclamp(cl, -1.0f, 1.0f) * 32767.0f);
            int16_t qr = static_cast<int16_t>(daisysp::fclamp(cr, -1.0f, 1.0f) * 32767.0f);
            ql ^= (ql >> 3);
            qr ^= (qr >> 3);
            Stereo bent = {
                static_cast<float>(ql) * (1.0f / 32767.0f),
                static_cast<float>(qr) * (1.0f / 32767.0f)
            };

            sig = sig * _crash_inv_blend + bent * _crash_blend;
        } else if ((_hist_tick = !_hist_tick)) {
            _hist[0][_hist_pos] = sig.l;
            _hist[1][_hist_pos] = sig.r;
            _hist_pos++;
        }

        // High-frequency harshness de-esser
        _step_tick = (_step_tick + 1) & 3;
        if (_step_tick == 0) {
            _cached_h_mono = _bp_harsh.Process(0.5f * (sig.l + sig.r));
            float harsh_peak = std::abs(_cached_h_mono);

            float att_h = (harsh_peak > _harsh_env) ? 0.15f : 0.008f;
            _harsh_env += att_h * (harsh_peak - _harsh_env);

            if (_harsh_env > 0.32f) {
                float excess = _harsh_env - 0.32f;
                _cached_deharsh_amt = std::clamp(excess * 2.0f, 0.0f, 0.65f);
            } else {
                _cached_deharsh_amt = 0.0f;
            }
        }

        if (_cached_deharsh_amt > 0.0f) {
            sig.l -= _cached_deharsh_amt * _cached_h_mono;
            sig.r -= _cached_deharsh_amt * _cached_h_mono;
        }

        // Safety limiter
        float max_p = std::max(std::abs(sig.l), std::abs(sig.r));
        if (_step_tick == 0) {
            _cached_target_g = (max_p > _eff_knee) ? (_eff_knee / max_p) : 1.0f;
        }
        float target_g = _cached_target_g;

        if (target_g < _lim_gain) {
            _lim_gain += 0.08f * (target_g - _lim_gain);
        } else {
            float rel_rate = (_lim_gain < 0.65f) ? 0.00012f : 0.00020f;
            _lim_gain += rel_rate * (1.0f - _lim_gain);
        }

        if (__builtin_expect(_lim_gain < 0.999f, 0)) {
            sig.l *= _lim_gain;
            sig.r *= _lim_gain;
            max_p *= _lim_gain;
        }

        // Soft-knee analog saturation
        if (__builtin_expect(max_p > _eff_knee, 0)) {
            sig.l = SoftSaturate(sig.l, _eff_knee, _eff_ceil, _eff_span, _eff_inv_span);
            sig.r = SoftSaturate(sig.r, _eff_knee, _eff_ceil, _eff_span, _eff_inv_span);
        }

        // Ultrasonic reconstruction filter
        _lp_warm_state_l += _cached_lp_alpha * (sig.l - _lp_warm_state_l);
        _lp_warm_state_r += _cached_lp_alpha * (sig.r - _lp_warm_state_r);
        sig.l = _lp_warm_state_l;
        sig.r = _lp_warm_state_r;

        // Output mute / gate
        if (__builtin_expect(_mute_smooth < 0.999f || _target_mute_gain < 0.999f, 0)) {
            _mute_smooth += 0.08f * (_target_mute_gain - _mute_smooth);
            sig *= _mute_smooth;
        }

        // Power rail brownout sputter and shutdown
        if (__builtin_expect(vcc < 0.045f, 0)) {
            if (vcc <= 0.008f) {
                _sputter_gate = 0.0f;
                _sputter_on = false;
            } else {
                float t = (vcc - 0.008f) * (1.0f / 0.037f);
                if (_sputter_timer == 0) {
                    _sputter_rng = _sputter_rng * 1664525u + 1013904223u;
                    float r = static_cast<float>(_sputter_rng & 0xFFFF) * (1.0f / 65536.0f);
                    if (_sputter_on) {
                        _sputter_on = false;
                        _sputter_timer = static_cast<uint16_t>(48.0f + (1.0f - t) * (350.0f + r * 1400.0f));
                    } else {
                        _sputter_on = true;
                        _sputter_timer = static_cast<uint16_t>(32.0f + t * (250.0f + r * 800.0f));
                    }
                } else {
                    _sputter_timer--;
                }
                float target_gate = _sputter_on ? (0.30f + 0.70f * t) : 0.0f;
                _sputter_gate += 0.05f * (target_gate - _sputter_gate);
            }
            sig *= _sputter_gate;
        } else {
            _sputter_gate = 1.0f;
            _sputter_on = true;
            _sputter_timer = 0;
        }

        // Boot fade and DAC clamp
        if (__builtin_expect(boot_gain != 1.0f, 0)) {
            sig *= boot_gain;
        }
        sig.l = std::clamp(sig.l, -0.95f, 0.95f);
        sig.r = std::clamp(sig.r, -0.95f, 0.95f);

        return sig;
    }

private:
    __attribute__((always_inline)) static inline float SoftSaturate(float x, float knee, float ceiling, float span, float inv_span) {
        float mag = std::abs(x);
        if (__builtin_expect(mag <= knee, 1)) return x;
        float norm = (mag - knee) * inv_span;
        float sat = knee + span * FastSoftSat(norm);
        sat = std::min(sat, ceiling);
        return (x >= 0.0f) ? sat : -sat;
    }

    float     _sample_rate{48000.0f};

    float     _sub_hp_l{0.0f};
    float     _sub_hp_r{0.0f};
    Biquad    _bp_harsh;
    float     _lp_warm_state_l{0.0f};
    float     _lp_warm_state_r{0.0f};
    float     _cached_lp_alpha{0.92f};

    float     _harsh_env{0.0f};
    float     _lim_gain{1.0f};
    float     _cached_target_g{1.0f};

    float     _hist[2][256]{};
    uint8_t   _hist_pos{0};
    bool      _crashing{false};
    uint16_t  _crash_len{64};
    uint16_t  _crash_idx{0};

    uint16_t  _sputter_timer{0};
    bool      _sputter_on{true};
    float     _sputter_gate{1.0f};
    uint32_t  _sputter_rng{0xACE1u};

    float     _mute_smooth{1.0f};
    float     _target_mute_gain{1.0f};
    bool      _crash_active{false};
    float     _crash_blend{0.0f};
    float     _crash_inv_blend{1.0f};
    float     _vcc_headroom{1.0f};
    float     _eff_ceil{kCeiling};
    float     _eff_knee{kKnee};
    float     _eff_span{kCeiling - kKnee};
    float     _eff_inv_span{1.0f / (kCeiling - kKnee)};
    float     _prev_vcc{-1.0f};
    float     _cached_h_mono{0.0f};
    float     _cached_deharsh_amt{0.0f};
    uint8_t   _step_tick{0};
    bool      _hist_tick{false};
    bool      _sub_decim{false};
};

} // namespace bends
