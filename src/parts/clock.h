#pragma once

#include "dsp/fast_math.h"
#include "bends/mod_pack.h"
#include <cmath>
#include <algorithm>

namespace bends {

class Clock {
public:
    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        _inv_sample_rate = (sample_rate > 0.0f) ? (1.0f / sample_rate) : (1.0f / 48000.0f);
        Reset();
    }

    void Reset() {
        _phase = 0.0f;
        _rate = 1.0f;
        _base_rate = 1.0f;
        _beat = false;
        _block_beat = false;
        _trig_env = 0.0f;
        _held = false;
        _swing_step = false;
    }

    inline void PrepareBlock(float knob_clk, float node_pull, float overclock = 0.0f) {
        _block_beat = false;
        float clk_dc = std::clamp(knob_clk + node_pull * 0.35f + overclock * 0.25f, 0.0f, 1.0f);
        if (clk_dc <= 0.50f) {
            float t = clk_dc * 2.0f;
            _base_rate = 0.50f * FastPow2(t * 4.58f); // 0.50 Hz (30 BPM) to 12.0 Hz (720 BPM)
        } else {
            float t = (clk_dc - 0.50f) * 2.0f;
            _base_rate = 12.0f * FastPow2(t * 7.70f); // 12.0 Hz to 2470 Hz
        }
        _base_rate = Sanitize(_base_rate * (1.0f + overclock * 0.50f), 1.0f);
        _clk_sag_depth = (_base_rate > 35.0f) ? 0.45f : 0.80f;
    }

    inline void PrepareBlockHz(float rate_hz) {
        _block_beat = false;
        _base_rate = Sanitize(std::clamp(rate_hz, 0.05f, 12000.0f), 1.0f);
        _clk_sag_depth = (_base_rate > 35.0f) ? 0.45f : 0.80f;
    }

    inline void ResetPhase() {
        _phase = 0.0f;
        _beat = true;
        _block_beat = true;
    }

    __attribute__((always_inline)) inline void Tick(float v_rail_eff, float fm_input, const ClockMods& mods,
                     uint8_t osc_mode, float osc_sig, float osc_ramp,
                     float& out_clk) {
        // Tempo clock runs independent of rail voltage sag
        (void)v_rail_eff;
        _step_tick = (_step_tick + 1) & 3;
        if (_step_tick == 0) {
            float r = _base_rate;
            if (__builtin_expect(std::abs(fm_input) > 0.001f, 0)) {
                r *= FastPow2(fm_input * 0.35f);
            }
            if (__builtin_expect(mods.rate_mod > 0.001f, 0)) {
                r *= std::clamp(FastPow2(mods.rate_mod * osc_sig * 0.50f), 0.5f, 2.0f);
            }
            _cached_eff_rate = std::clamp(r, 0.05f, 12000.0f);
            _cached_phase_inc = _cached_eff_rate * _inv_sample_rate;
            _cached_clk_level = (v_rail_eff > 0.05f) ? (0.75f + 0.25f * v_rail_eff) : (v_rail_eff * 15.0f);
        }
        float rate = _cached_eff_rate;
        _rate = rate;
        _beat = false;

        if (mods.hold > 0.5f) {
            // Held in reset: counter frozen at zero, no beats
            _phase = 0.0f;
            _held = true;
        } else {
            if (_held) {
                // Released: restart on the downbeat
                _held = false;
                _beat = true;
                _block_beat = true;
                _trig_env = 1.0f;
            }
            _phase += _cached_phase_inc;

            float phase_threshold = 1.0f;
            if (mods.swing > 0.01f) {
                // Swing groove: warps threshold between even and odd beats
                float swing_depth = std::clamp(mods.swing * 0.42f, 0.0f, 0.40f);
                float osc_micro = osc_sig * 0.08f * mods.swing;
                float eff_swing = std::clamp(swing_depth + osc_micro, 0.0f, 0.45f);

                phase_threshold = _swing_step ? (1.0f - eff_swing) : (1.0f + eff_swing);
            }

            if (_phase >= phase_threshold) {
                _phase -= phase_threshold;
                if (_phase < 0.0f) _phase = 0.0f;
                _beat = true;
                _block_beat = true;
                _trig_env = 1.0f;
                _swing_step = !_swing_step;
            }
        }

        // Trigger pulse output (~30ms decay)
        if (__builtin_expect(_trig_env > 0.0005f, 0)) {
            _trig_env *= 0.9968f;
            if (_trig_env < 0.0005f) {
                _trig_env = 0.0f;
                out_clk = 0.0f;
            } else {
                float click = _trig_env * _trig_env * _trig_env;
                float thump = 4.0f * _trig_env * (1.0f - _trig_env) * _trig_env;
                float punchy_pulse = click * 0.85f + thump * 0.40f;
                out_clk = punchy_pulse * _cached_clk_level;
            }
        } else {
            out_clk = 0.0f;
        }
    }

    float Rate() const { return _rate; }
    bool Beat() const { return _beat; }
    bool BlockBeat() const { return _block_beat; }
    float Phase() const { return _phase; }

private:
    float _sample_rate{48000.0f};
    float _inv_sample_rate{1.0f / 48000.0f};
    float _phase{0.0f};
    float _rate{1.0f};
    float _base_rate{1.0f};
    bool  _beat{false};
    bool  _block_beat{false};
    bool  _held{false};
    bool  _swing_step{false};
    float _trig_env{0.0f};
    float _clk_sag_depth{0.60f};
    uint8_t _step_tick{0};
    float _cached_fm_mult{1.0f};
    float _cached_rate_mod_mult{1.0f};
    float _cached_eff_rate{1.0f};
    float _cached_phase_inc{1.0f / 48000.0f};
    float _cached_clk_level{1.0f};
};

} // namespace bends
