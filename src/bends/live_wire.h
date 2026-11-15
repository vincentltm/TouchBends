#pragma once

#include "dsp/fast_math.h"
#include "circuit/nodes.h"
#include "circuit/block_state.h"
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace bends {

struct TouchAffordance {
    float pre{0.0f};  // Preamp stage (GND/AMP ground hum into in_combined)
    float osc{0.0f};  // Master mix (OSC ghost radio bleed into tap_mix)
    float dly{0.0f};  // Delay stage
};

/**
 * Physical touch affordances:
 * - GND & AMP: 55 Hz + 110 Hz mains induction hum into preamp input.
 * - OSC: Bandpassed bleed (750 Hz - 1800 Hz) scaled by drive slider.
 * - RST: Stepping pulses.
 * - CLK: Clock frequency whine.
 * - DLY: DRAM packet noise.
 * Mutes when two pads are bridged to form a circuit bend.
 */
class LiveWireAffordance {
public:
    void Init(float sample_rate) {
        _sample_rate = (sample_rate > 1000.0f) ? sample_rate : 48000.0f;
        _inv_sr = 1.0f / _sample_rate;
        Reset();
    }

    void Reset() {
        _active_node = -1;
        _prev_alone_node = -1;
        _hum_phase = 0.0f;
        _ghost_hp = 0.0f;
        _ghost_lp = 0.0f;
        _env = 0.0f;
        _smooth_pre = 0.0f;
        _smooth_osc = 0.0f;
        _smooth_dly = 0.0f;
        _mem_burst_phase = 0.0f;
        _mem_bp_s1 = 0.0f;
        _mem_bp_s2 = 0.0f;
        _clk_whine_phase = 0.0f;
        _clk_sub_phase = 0.0f;
        _rst_phase = 0.0f;
        _mem_rng = 0x51A7B394u;
    }

    __attribute__((always_inline)) inline TouchAffordance Process(const BlockState& state, float live_osc,
                                                                 float /*vcc*/, float /*starve*/) {
        int8_t alone_node = state.alone_node;
        if (__builtin_expect(alone_node < 0 && _active_node < 0 && _env <= 0.0001f && _smooth_pre <= 0.0001f &&
                             (state.topology.active_node_count > 2 || (state.g_in_clk <= 0.005f && state.g_in_dly <= 0.005f)), 1)) {
            return { 0.0f, 0.0f, 0.0f };
        }

        // Detect solo touch transitions
        float pressure = state.alone_pressure;
        if (alone_node != _prev_alone_node) {
            _active_node = (alone_node >= 0 && pressure > 0.008f) ? alone_node : -1;
            _prev_alone_node = alone_node;
        }

        // Smooth solo touch envelope (soft 25ms rise, soft 35ms fade - never a harsh gate)
        float target_env = (_active_node >= 0) ? std::min(1.0f, pressure * 1.15f) : 0.0f;
        if (target_env > _env) {
            _env += (_inv_sr * 40.0f) * (target_env - _env); // ~25ms attack
        } else {
            _env += (_inv_sr * 30.0f) * (target_env - _env); // ~35ms release
            if (_env < 0.0003f) {
                _env = 0.0f;
                _active_node = -1;
            }
        }

        bool has_bridge_in = (state.topology.active_node_count <= 2) && (state.g_in_clk > 0.005f || state.g_in_dly > 0.005f);
        if (_env <= 0.0001f && !has_bridge_in) {
            if (_smooth_pre > 0.0001f || _smooth_osc > 0.0001f || _smooth_dly > 0.0001f) {
                _smooth_pre *= 0.85f;
                _smooth_osc *= 0.85f;
                _smooth_dly *= 0.85f;
                return { _smooth_pre, _smooth_osc, _smooth_dly };
            }
            return { 0.0f, 0.0f, 0.0f };
        }

        TouchAffordance out{};
        float p = _env;

        if (_active_node == NodeGnd || _active_node == NodeAmp) {
            // Ground & Preamp: mains hum into preamp input
            _hum_phase += 55.0f * _inv_sr;
            if (_hum_phase >= 1.0f) _hum_phase -= 1.0f;
            float hum = FastSin(_hum_phase * kTwoPi) + 0.20f * FastSin(_hum_phase * (kTwoPi * 2.0f));
            out.pre = hum * (0.0022f * p);
        } else if (_active_node == NodeOsc) {
            // OSC: Bandpassed bleed (750 Hz to 1800 Hz)
            _ghost_hp += 0.095f * (live_osc - _ghost_hp);
            float hp = live_osc - _ghost_hp;
            _ghost_lp += 0.200f * (hp - _ghost_lp);
            float drive_vol = 0.15f + 0.85f * state.s30_amp;
            out.osc = _ghost_lp * (0.0035f * p * drive_vol);
        } else if (_active_node == NodeRst) {
            // RST: Stepping pulses into master mix tap
            _rst_phase += 190.0f * _inv_sr;
            if (_rst_phase >= 1.0f) _rst_phase -= 1.0f;
            float click = (_rst_phase < 0.08f) ? (1.0f - _rst_phase / 0.08f) : 0.0f;
            _mem_rng ^= _mem_rng << 13; _mem_rng ^= _mem_rng >> 17; _mem_rng ^= _mem_rng << 5;
            float sign = (_mem_rng & 1) ? 1.0f : -1.0f;
            out.osc = click * sign * (0.0030f * p);
        }

        // Clock whine:
        // Solo touch: mix tap; Bridged with IN: preamp input
        bool need_clk = (_active_node == NodeClk || state.g_in_clk > 0.005f);
        if (need_clk) {
            float clk_knob = state.s33_clk;
            float clk_freq = 1600.0f * (1.0f + 2.6f * clk_knob * clk_knob);
            _clk_whine_phase += clk_freq * _inv_sr;
            if (_clk_whine_phase >= 1.0f) _clk_whine_phase -= 1.0f;
            float sin1 = FastSin(_clk_whine_phase * kTwoPi);
            float sin2 = FastSin(_clk_whine_phase * (kTwoPi * 2.0f));
            _clk_sub_phase += (clk_freq * 0.5f) * _inv_sr;
            if (_clk_sub_phase >= 1.0f) _clk_sub_phase -= 1.0f;
            float sub_tone = (_clk_sub_phase < 0.5f) ? 0.35f : -0.35f;
            float whine = sin1 * 0.65f + sin2 * 0.25f + sub_tone * (state.starve * 0.60f);

            if (_active_node == NodeClk) {
                out.osc += whine * (0.0018f * p);
            }
            if (state.g_in_clk > 0.005f) {
                out.pre += whine * (0.085f * state.g_in_clk);
            }
        }

        // DRAM refresh noise:
        // Solo touch: mix tap; Bridged with IN: preamp input
        bool need_dly = (_active_node == NodeDly || state.g_in_dly > 0.005f);
        if (need_dly) {
            float stride_rate = 80.0f;
            if (state.s34_dly < 0.25f) stride_rate = 260.0f;
            else if (state.s34_dly < 0.50f) stride_rate = 180.0f;
            else if (state.s34_dly < 0.75f) stride_rate = 120.0f;
            _mem_burst_phase += stride_rate * _inv_sr;
            if (_mem_burst_phase >= 1.0f) _mem_burst_phase -= 1.0f;

            _mem_rng ^= _mem_rng << 13; _mem_rng ^= _mem_rng >> 17; _mem_rng ^= _mem_rng << 5;
            float raw_noise = static_cast<float>(static_cast<int16_t>(_mem_rng & 0xFFFF)) * (1.0f / 32768.0f);
            float burst_env = (_mem_burst_phase < 0.25f) ? (1.0f - _mem_burst_phase / 0.25f) : 0.0f;
            float raw_packet = raw_noise * burst_env;

            float bp_alpha_lp = 0.22f + 0.35f * state.s31_flt;
            float bp_alpha_hp = 0.05f + 0.07f * state.s31_flt;
            _mem_bp_s1 += bp_alpha_lp * (raw_packet - _mem_bp_s1);
            _mem_bp_s2 += bp_alpha_hp * (_mem_bp_s1 - _mem_bp_s2);
            float packet = _mem_bp_s1 - _mem_bp_s2;

            if (_active_node == NodeDly) {
                out.osc += packet * (0.0030f * p);
            }
            if (state.g_in_dly > 0.005f) {
                out.pre += packet * (0.095f * state.g_in_dly);
            }
        }

        _smooth_pre = out.pre;
        _smooth_osc = out.osc;
        _smooth_dly = out.dly;

        return out;
    }

    bool IsIdle() const {
        return (_active_node < 0 && _env <= 0.0001f && _smooth_pre <= 0.0001f &&
                _smooth_osc <= 0.0001f && _smooth_dly <= 0.0001f);
    }

private:
    float    _sample_rate{48000.0f};
    float    _inv_sr{1.0f / 48000.0f};
    int8_t   _active_node{-1};
    int8_t   _prev_alone_node{-1};
    float    _hum_phase{0.0f};
    float    _ghost_hp{0.0f};
    float    _ghost_lp{0.0f};
    float    _env{0.0f};

    float    _smooth_pre{0.0f};
    float    _smooth_osc{0.0f};
    float    _smooth_dly{0.0f};

    float    _mem_burst_phase{0.0f};
    float    _mem_bp_s1{0.0f};
    float    _mem_bp_s2{0.0f};
    float    _clk_whine_phase{0.0f};
    float    _clk_sub_phase{0.0f};
    float    _rst_phase{0.0f};
    uint32_t _mem_rng{0x51A7B394u};
};

} // namespace bends
