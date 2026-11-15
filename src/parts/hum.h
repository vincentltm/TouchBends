#pragma once

#include "dsp/fast_math.h"
#include "circuit/health.h"
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace bends {

class Hum {
public:
    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        _phase_inc = (sample_rate > 0.0f) ? (50.0f * kTwoPi / sample_rate) : (50.0f * kTwoPi / 48000.0f);
        Reset();
    }

    void Reset() {
        _smooth_gnd_hum = 0.0f;
        _phase = 0.0f;
        _crackle_rng = 13579u;
        _crackle_filter = 0.0f;
        _ms_timer = 0;
        _ms_env = 0.0f;
        _ms_decay = 0.95f;
        _ms_phase = 0.0f;
        _ms_freq = 0.0f;
        _ms_pitch_sweep = 0.0f;
        _ms_type = 0;
    }

    __attribute__((always_inline)) inline void Tick(float v_rail_eff, float starve, float standalone_p_gnd, float gnd_prox,
                     float touched_nodes_sum, float amt_microsounds, float& out_hum) {
        float eff_prox = std::max(0.0f, (gnd_prox - 0.10f) / 0.90f);

        float gnd_touch_hum = standalone_p_gnd * 0.045f;
        float hover_hum = (eff_prox * eff_prox) * 0.015f;

        float hum_target = gnd_touch_hum + hover_hum;
        _smooth_gnd_hum += (hum_target - _smooth_gnd_hum) * 0.008f;

        // Microsound burst generation on VCC-GND bridge
        float ms_activity = amt_microsounds;
        float ms_audio = 0.0f;

        if (ms_activity > 0.01f) {
            if (_ms_timer > 0) {
                _ms_timer--;
            } else {
                _crackle_rng = _crackle_rng * 1664525u + 1013904223u;
                uint32_t max_interval = static_cast<uint32_t>(28800.0f - ms_activity * 25000.0f);
                if (max_interval < 1200) max_interval = 1200;
                _ms_timer = 400 + (_crackle_rng % max_interval);

                uint32_t roll = (_crackle_rng >> 16) & 0x7FFF;
                if (ms_activity < 0.40f) {
                    _ms_type = (roll & 1) ? 0 : 3; // Clicks & noise pops
                } else if (ms_activity < 0.75f) {
                    _ms_type = (roll & 1) ? 1 : 0; // Chirps & clicks
                } else {
                    _ms_type = (roll & 1) ? 1 : 2; // Chirps & high squeals
                }

                _ms_env = 0.045f * ms_activity;
                _ms_phase = 0.0f;

                if (_ms_type == 0) {
                    _ms_decay = 0.80f; // Soft pop
                } else if (_ms_type == 1) {
                    // Downward chirp
                    _ms_decay = 0.985f;
                    _ms_freq = 0.18f + 0.12f * (roll * (1.0f / 32768.0f));
                    _ms_pitch_sweep = -0.0005f - 0.0008f * ms_activity;
                } else if (_ms_type == 2) {
                    // Upward high squeal
                    _ms_decay = 0.992f;
                    _ms_freq = 0.25f + 0.15f * (roll * (1.0f / 32768.0f));
                    _ms_pitch_sweep = 0.0001f + 0.0002f * ms_activity;
                } else {
                    // Short noise burst / digital hash pop
                    _ms_decay = 0.94f;
                }
            }

            if (_ms_env > 0.0005f) {
                if (_ms_type == 0) {
                    ms_audio = _ms_env * 0.70f;
                    _ms_env *= _ms_decay;
                } else if (_ms_type == 1 || _ms_type == 2) {
                    _ms_phase += _ms_freq;
                    if (_ms_phase >= kTwoPi) _ms_phase -= kTwoPi;
                    _ms_freq += _ms_pitch_sweep;
                    if (_ms_freq < 0.005f) _ms_freq = 0.005f;

                    float s = FastSin(_ms_phase);
                    if (ms_activity > 0.50f) {
                        int16_t q = static_cast<int16_t>(s * 16384.0f);
                        uint8_t mask = static_cast<uint8_t>((ms_activity - 0.50f) * 60.0f);
                        q ^= (mask << 5);
                        s = static_cast<float>(q) * (1.0f / 16384.0f);
                    }
                    ms_audio = s * _ms_env;
                    _ms_env *= _ms_decay;
                } else {
                    _crackle_rng = _crackle_rng * 1664525u + 1013904223u;
                    float n = static_cast<float>(static_cast<int32_t>(_crackle_rng)) * 4.6566129e-10f;
                    ms_audio = n * _ms_env;
                    _ms_env *= _ms_decay;
                }
            }
        }

        if (_smooth_gnd_hum < 0.0002f && ms_activity <= 0.01f) {
            out_hum = 0.0f;
            return;
        }

        float buzz = 0.0f;
        if (_smooth_gnd_hum >= 0.0002f) {
            _phase += _phase_inc;
            if (_phase >= kTwoPi) _phase -= kTwoPi;

            float sin1 = FastSin(_phase);
            float sin2 = FastSin(_phase * 2.0f);
            float harmonic_brightness = 0.20f + 0.40f * (standalone_p_gnd + eff_prox);
            buzz = sin1 * 0.75f + sin2 * 0.25f * harmonic_brightness;
        }

        // Electrostatic crackle generator
        _crackle_rng = _crackle_rng * 1664525u + 1013904223u;
        float white = static_cast<float>(static_cast<int32_t>(_crackle_rng)) * 4.6566129e-10f;
        float crackle = (white > 0.96f) ? (white - 0.96f) * 4.0f : ((white < -0.97f) ? (white + 0.97f) * 3.0f : 0.0f);
        _crackle_filter += (crackle - _crackle_filter) * 0.20f;

        float starve_buzz = buzz + (buzz * buzz - 0.5f) * starve * 0.50f;
        float gnd_hum_audio = (starve_buzz * 0.90f + _crackle_filter * 0.10f) * _smooth_gnd_hum + ms_audio;

        float body_antenna_mult = 1.0f + std::min(0.5f, touched_nodes_sum * 0.5f);
        out_hum = gnd_hum_audio * body_antenna_mult;
    }

    float SmoothHum() const { return _smooth_gnd_hum; }

private:
    float _sample_rate{48000.0f};
    float _phase_inc{50.0f * kTwoPi / 48000.0f};
    float _smooth_gnd_hum{0.0f};
    float _phase{0.0f};
    uint32_t _crackle_rng{13579u};
    float _crackle_filter{0.0f};

    // Microsound state
    uint32_t _ms_timer{0};
    float    _ms_env{0.0f};
    float    _ms_decay{0.95f};
    float    _ms_phase{0.0f};
    float    _ms_freq{0.0f};
    float    _ms_pitch_sweep{0.0f};
    uint8_t  _ms_type{0};
};

} // namespace bends
