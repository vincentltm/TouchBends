#pragma once

#include "circuit/nodes.h"
#include "circuit/health.h"
#include "dsp/fast_math.h"
#include <cmath>
#include <algorithm>

namespace bends {

class Rail {
public:
    void Init(float sample_rate) {
        (void)sample_rate;
        Reset();
    }

    void Reset() {
        _fader_vcc = 1.0f;
        _cap_voltage = 1.0f;
        _effective_vcc = 1.0f;
        _cached_ripple_scale = 0.0f;
        _cached_min_v = 0.50f;
        _motorboat_phase = 0.0f;
        _motorboat_pulse = 0.0f;
    }

    // Call at control rate (~750 Hz)
    void ProcessControl(float fader_pos, float external_load, float network_vcc_pull, float direct_gnd_short = 0.0f) {
        float u = std::clamp(Sanitize(fader_pos, 1.0f), 0.0f, 1.0f);
        // Gradual continuous multi-stage voltage taper across full slider travel (exact 0.0V at u=0)
        _fader_vcc = (u <= 0.015f) ? 0.0f : (u * (0.75f + 0.25f * u));
        external_load = Sanitize(external_load, 0.0f);
        network_vcc_pull = Sanitize(network_vcc_pull, 0.0f);
        direct_gnd_short = std::clamp(Sanitize(direct_gnd_short, 0.0f), 0.0f, 1.0f);
        _cap_voltage = daisysp::fclamp(Sanitize(_cap_voltage, 1.0f), 0.0f, 1.0f);

        // Musical internal battery impedance: produces dynamic sag on heavy feedback load
        float starve_factor = 1.0f - _fader_vcc;
        float r_internal = 0.015f + 0.075f * (starve_factor * starve_factor);

        // Target voltage accounts for fader, current load, and network DC pull
        float pull_effect = (network_vcc_pull < 0.0f) ? (network_vcc_pull * 0.35f) : (network_vcc_pull * 0.20f);
        float sag = std::min(0.20f * _fader_vcc, r_internal * external_load);
        float v_target = _fader_vcc - sag + pull_effect;

        // Direct VCC <-> GND battery short pulls voltage directly down to 0V
        if (direct_gnd_short > 0.08f) {
            float drain = std::clamp((direct_gnd_short - 0.08f) * 1.25f, 0.0f, 1.0f);
            v_target *= (1.0f - drain);
            v_target = std::clamp(v_target, 0.0f, 1.0f);
            _motorboat_pulse *= 0.80f;
        } else if (_fader_vcc <= 0.015f) {
            // Slider at bottom: complete clean power-down to 0V
            v_target = 0.0f;
            _motorboat_pulse = 0.0f;
        } else {
            // Soft min_floor allows dynamic touch-pressure rail sag in starved mode
            float min_floor = 0.010f;
            v_target = std::clamp(v_target, min_floor, 1.0f);

            // Low-voltage supply motorboating (relaxation oscillation under load):
            if (_fader_vcc < 0.25f && external_load > 0.08f) {
                float starve_intensity = (0.25f - _fader_vcc) * 4.0f; // 0.0 to 1.0
                float mb_freq = 3.0f + 5.5f * std::clamp(external_load * 0.5f, 0.0f, 1.0f);
                _motorboat_phase += mb_freq * (1.0f / 750.0f);
                if (_motorboat_phase >= 1.0f) _motorboat_phase -= 1.0f;

                // Cosine relaxation pulse
                float pulse = 0.5f * (1.0f - FastCos(_motorboat_phase * kTwoPi));
                float dip = pulse * pulse * starve_intensity * 0.35f * _cap_voltage;
                _motorboat_pulse = pulse * starve_intensity;
                v_target = std::max(min_floor, v_target - dip);
            } else {
                _motorboat_pulse *= 0.85f;
                if (_motorboat_pulse < 0.001f) _motorboat_pulse = 0.0f;
                _motorboat_phase = 0.0f;
            }
        }

        // Reservoir capacitor: fast discharge on load spike, smooth recharge
        float alpha = (_fader_vcc <= 0.015f) ? 0.65f : ((v_target < _cap_voltage) ? 0.20f : 0.038f);
        _cap_voltage += alpha * (v_target - _cap_voltage);
        _effective_vcc = _cap_voltage;
        _cached_ripple_scale = (1.0f - _cap_voltage) * 0.45f;
        _cached_min_v = _cap_voltage * 0.50f;
    }

    // Call per sample (48 kHz) with AC ripple from NodeVcc
    __attribute__((always_inline)) inline float Tick(float ac_ripple) {
        if (__builtin_expect(_cap_voltage <= 0.005f, 0)) {
            _effective_vcc = 0.0f;
            return 0.0f;
        }
        float ripple_v = _cap_voltage + ac_ripple * _cached_ripple_scale;
        _effective_vcc = daisysp::fclamp(ripple_v, _cached_min_v, 1.0f);
        return _effective_vcc;
    }

    float EffectiveVcc() const   { return _effective_vcc; }
    float CapVoltage() const     { return _cap_voltage; }
    float FaderVcc() const       { return _fader_vcc; }
    float MotorboatPulse() const { return _motorboat_pulse; }

private:
    float _fader_vcc{1.0f};
    float _cap_voltage{1.0f};
    float _effective_vcc{1.0f};
    float _cached_ripple_scale{0.0f};
    float _cached_min_v{0.50f};
    float _motorboat_phase{0.0f};
    float _motorboat_pulse{0.0f};
};

} // namespace bends
