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
        _is_conducting = true;
        _motorboat_pulse = 0.0f;
    }

    // Call at control rate (~750 Hz)
    void ProcessControl(float fader_pos, float external_load, float network_vcc_pull, float direct_gnd_short = 0.0f) {
        float u = std::clamp(Sanitize(fader_pos, 1.0f), 0.0f, 1.0f);
        // Continuous non-linear voltage taper (0V at u=0, 1.0V at u=1)
        _fader_vcc = (u <= 0.015f) ? 0.0f : (u * (0.75f + 0.25f * u));
        external_load = Sanitize(external_load, 0.0f);
        network_vcc_pull = Sanitize(network_vcc_pull, 0.0f);
        direct_gnd_short = std::clamp(Sanitize(direct_gnd_short, 0.0f), 0.0f, 1.0f);
        _cap_voltage = daisysp::fclamp(Sanitize(_cap_voltage, 1.0f), 0.0f, 1.0f);

        if (_fader_vcc <= 0.015f) {
            // Power off: immediate decay to 0V
            _cap_voltage *= 0.50f;
            if (_cap_voltage < 0.005f) _cap_voltage = 0.0f;
            _effective_vcc = _cap_voltage;
            _motorboat_pulse = 0.0f;
            _is_conducting = false;
            return;
        }

        // Physical internal battery impedance: rises non-linearly as battery starves
        float starve_factor = 1.0f - _fader_vcc;
        float r_internal = 0.025f + 0.55f * (starve_factor * starve_factor);
        constexpr float c_rail = 0.220f; // Bulk reservoir capacitor (~220uF equivalent)
        constexpr float dt = 1.0f / 750.0f;

        // Dynamic Kirchhoff network DC pull effect on the battery
        float pull_effect = (network_vcc_pull < 0.0f) ? (network_vcc_pull * 0.35f) : (network_vcc_pull * 0.20f);
        float v_source = std::clamp(_fader_vcc + pull_effect, 0.005f, 1.0f);

        // Hysteretic amplifier latch-up thresholds (physical transistor cut-off & recovery)
        constexpr float v_cutoff = 0.065f;
        constexpr float v_turn_on = 0.145f;

        if (_is_conducting) {
            if (_cap_voltage < v_cutoff) {
                _is_conducting = false;
            }
        } else {
            if (_cap_voltage > v_turn_on) {
                _is_conducting = true;
            }
        }

        // Active dynamic current draw from the rail:
        // Amplifiers draw heavy current when conducting; when cut off, only quiescent leakage remains
        float i_load = _is_conducting ? (external_load * 0.95f + 0.015f * _cap_voltage) : (0.010f * _cap_voltage);

        // Direct ground short (finger bridge directly between VCC and GND)
        if (direct_gnd_short > 0.05f) {
            i_load += direct_gnd_short * 3.5f * _cap_voltage;
        }

        // Differential equation: C * (dv/dt) = I_charge - I_load
        float i_charge = (v_source - _cap_voltage) / r_internal;
        float dv = ((i_charge - i_load) / c_rail) * dt;

        _cap_voltage = std::clamp(_cap_voltage + dv, 0.005f, 1.0f);
        _effective_vcc = _cap_voltage;

        // Motorboat pulse: tracks dynamic relaxation voltage deflection when starved
        if (_fader_vcc < 0.28f && external_load > 0.06f) {
            float starve_intensity = (0.28f - _fader_vcc) * 3.57f;
            float dip = std::clamp((v_turn_on - _cap_voltage) / (v_turn_on - v_cutoff + 0.001f), 0.0f, 1.0f);
            _motorboat_pulse += 0.25f * (dip * starve_intensity - _motorboat_pulse);
        } else {
            _motorboat_pulse *= 0.82f;
            if (_motorboat_pulse < 0.001f) _motorboat_pulse = 0.0f;
        }

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
    bool  IsConducting() const   { return _is_conducting; }

private:
    float _fader_vcc{1.0f};
    float _cap_voltage{1.0f};
    float _effective_vcc{1.0f};
    float _cached_ripple_scale{0.0f};
    float _cached_min_v{0.50f};
    bool  _is_conducting{true};
    float _motorboat_pulse{0.0f};
};

} // namespace bends
