#pragma once

#include "dsp/fast_math.h"
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace bends {

class ContactMic {
public:
    struct Resonator {
        float s1{0.0f}, s2{0.0f};
        float g{0.0f}, R{0.0f}, h{0.0f};
        float r2_g{0.0f}, norm_gain{0.0f};

        void Init(float g_in, float h_in, float r2_g_in, float norm_in) {
            g = g_in;
            h = h_in;
            r2_g = r2_g_in;
            norm_gain = norm_in;
            s1 = 0.0f;
            s2 = 0.0f;
        }

        __attribute__((always_inline)) inline float Process(float in) {
            float in_safe = daisysp::fclamp(in, -2.5f, 2.5f);
            float high = (in_safe - r2_g * s1 - s2) * h;
            float v1 = g * high;
            float band = v1 + s1;
            s1 = daisysp::fclamp(2.0f * v1 + s1, -2.5f, 2.5f);
            float v2 = g * band;
            s2 = daisysp::fclamp(2.0f * v2 + s2, -2.5f, 2.5f);
            return band * norm_gain; // Peak gain strictly normalized to 1.0 (0dB) at resonance
        }

        void Reset() { s1 = 0.0f; s2 = 0.0f; }
    };

    struct TapVoice {
        bool     active{false};
        uint16_t sample_idx{0};
        uint16_t total_samples{0};
        float    vel{0.0f};
        float    pitch_phase{0.0f};
        float    pitch_freq{0.0f};
        float    pitch_decay{0.0f};
        float    env{0.0f};
        float    env_decay{0.0f};
        float    click_env{0.0f};
        float    inv_sr{1.0f / 48000.0f};

        void Trigger(float velocity, float sample_rate) {
            vel = std::clamp(velocity, 0.15f, 1.0f);
            active = true;
            sample_idx = 0;
            inv_sr = 1.0f / sample_rate;
            // Total tap envelope ~50ms
            total_samples = static_cast<uint16_t>(sample_rate * 0.050f);

            // Body resonance: starts ~135-165Hz and drops to ~65Hz
            pitch_freq = 135.0f + vel * 30.0f;
            pitch_decay = 0.9991f;
            pitch_phase = 0.0f;

            env = 1.0f;
            env_decay = 0.9987f; // ~45ms decay to 0.01
            click_env = 1.0f;    // Fast contact click ~3.5ms
        }

        __attribute__((always_inline)) inline float Process(float white) {
            if (!active) return 0.0f;
            sample_idx++;
            if (sample_idx >= total_samples) {
                active = false;
                return 0.0f;
            }

            // Chassis body displacement: dropping pitch sine
            pitch_phase += (pitch_freq * inv_sr);
            if (pitch_phase >= 1.0f) pitch_phase -= 1.0f;
            pitch_freq = std::max(60.0f, pitch_freq * pitch_decay);

            float body = FastSin(pitch_phase * kTwoPi);
            env *= env_decay;
            click_env *= 0.9950f; // fast click decay ~3.5ms

            // Impact signal: body resonance + click transient
            return (body * 0.75f + white * click_env * 0.40f) * env * vel;
        }

        void Reset() {
            active = false;
            sample_idx = 0;
            total_samples = 0;
            vel = 0.0f;
            pitch_phase = 0.0f;
            env = 0.0f;
            click_env = 0.0f;
        }
    };

    void Init(float sample_rate) {
        if (sample_rate < 1000.0f) sample_rate = 48000.0f;
        _sample_rate = sample_rate;

        // Enclosure resonance (135 Hz, Q=2.2) + high mode (720 Hz, Q=3.0)
        _res_chassis.Init(0.008836f, 0.995925f, 0.463382f, 0.454545f);
        _res_ceramic.Init(0.047159f, 0.982373f, 0.380492f, 0.333333f);

        Reset();
    }

    void Reset() {
        _res_chassis.Reset();
        _res_ceramic.Reset();

        _hp_state = 0.0f;
        _lp_damp = 0.0f;

        _tap.Reset();

        _pink_b0 = 0.0f; _pink_b1 = 0.0f; _pink_b2 = 0.0f;
        _lp_pulp = 0.0f;
        _smooth_rub = 0.0f;

        _arcing_target = 0.0f;
        _arcing_motion = 0.0f;
        _digital_target = 0.0f;
        _digital_ratio = 0.0f;
        _arc_bp_s1 = 0.0f;
        _arc_bp_s2 = 0.0f;

        _rng = 0x93A11234u;
        _res_active_cnt = 0;
        _step_cnt = 0;
        _cached_chas = 0.0f;
        _cached_ceram = 0.0f;
    }

    void TriggerTap(float velocity) {
        _tap.Trigger(velocity, _sample_rate);
    }

    void SetArcingMotion(float motion, float digital_ratio = 0.0f) {
        _arcing_target = std::clamp(motion, 0.0f, 1.0f);
        _digital_target = std::clamp(digital_ratio, 0.0f, 1.0f);
        _arcing_motion += (_arcing_target - _arcing_motion) * 0.35f;
        _digital_ratio += (_digital_target - _digital_ratio) * 0.35f;
    }

    __attribute__((always_inline)) inline bool IsActive(float p10, float v_rub10) const {
        return (v_rub10 > 0.002f) || (p10 > 0.02f) || _tap.active || (_arcing_motion > 0.002f) || (std::abs(_lp_damp) >= 1e-4f);
    }

    __attribute__((always_inline)) inline void Process(float p10, float v_rub10, float& out_mic) {
        // Quick idle bypass when no physical rub/tap excitation, touch pressure, or arcing motion is present
        bool has_rub = (v_rub10 > 0.002f);
        bool has_touch = (p10 > 0.02f && v_rub10 > 0.002f);
        bool has_tap = _tap.active;
        bool has_arcing = (_arcing_motion > 0.002f);
        if (!has_rub && !has_touch && !has_tap && !has_arcing && std::abs(_lp_damp) < 1e-4f) {
            out_mic = 0.0f;
            return;
        }

        // Shared PRNG for acoustic textures
        _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
        float white = static_cast<float>(static_cast<int32_t>(_rng)) * (1.0f / 2147483648.0f);

        // Friction model for pad P10
        float scratch_sig = 0.0f;
        if (v_rub10 > 0.002f && p10 > 0.02f) {
            _smooth_rub += (v_rub10 - _smooth_rub) * 0.040f;
            float eff = std::clamp(_smooth_rub, 0.0f, 1.0f);

            // Pink noise shaping (Kellet 3-pole)
            _pink_b0 = 0.99765f * _pink_b0 + white * 0.0555179f;
            _pink_b1 = 0.96300f * _pink_b1 + white * 0.1538520f;
            _pink_b2 = 0.57000f * _pink_b2 + white * 0.5329522f;
            float pink = (_pink_b0 + _pink_b1 + _pink_b2 + white * 0.5362f) * 0.22f;

            // Slip-stick burst generation
            bool asperity = ((_rng & 0x03FF) < static_cast<uint32_t>(eff * 1200.0f));
            float micro = asperity ? (white * (0.25f + 0.35f * eff)) : 0.0f;
            float raw_fric = pink * 0.45f + micro * 0.55f;

            // Filter cutoff tracks rubbing velocity (450Hz to 1100Hz)
            float cutoff = 450.0f + eff * 650.0f;
            float g = std::clamp(cutoff * (kPi / _sample_rate), 0.015f, 0.20f);
            _lp_pulp += g * (raw_fric - _lp_pulp);

            // Dynamic gain scaling
            float cl_eff = std::clamp(eff, 0.0f, 1.0f);
            float sqrt_eff = cl_eff * (1.5f - 0.5f * cl_eff);
            float gain = (sqrt_eff * 0.75f + cl_eff * 0.45f) * (0.35f + 0.65f * p10);
            scratch_sig = _lp_pulp * gain * 1.25f;
        } else {
            _smooth_rub *= 0.75f;
            if (_smooth_rub < 0.001f) _smooth_rub = 0.0f;
            _pink_b0 *= 0.85f; _pink_b1 *= 0.85f; _pink_b2 *= 0.85f; _lp_pulp *= 0.80f;
        }

        // Contact noise simulation:
        float arcing_sig = 0.0f;
        if (has_arcing) {
            float arc_eff = std::min(1.0f, (_arcing_motion - 0.002f) * 8.0f);
            bool is_digital = (_digital_ratio > 0.35f);

            bool spark;
            float raw_spark;
            if (is_digital) {
                spark = ((_rng & 0x01FFu) < static_cast<uint32_t>(arc_eff * 1400.0f));
                int32_t step = ((_rng >> 12) & 0x07) - 3; // -3..+3 discrete logic steps
                raw_spark = spark ? (static_cast<float>(step) * 0.25f * (0.35f + 0.65f * arc_eff)) : 0.0f;
            } else {
                spark = ((_rng & 0x07FFu) < static_cast<uint32_t>(arc_eff * 900.0f));
                raw_spark = spark ? (white * (0.25f + 0.40f * arc_eff)) : 0.0f;
            }

            // Adaptive bandpass filter
            float lp_alpha = 0.35f + 0.30f * _digital_ratio;
            float hp_alpha = 0.045f + 0.075f * _digital_ratio;
            _arc_bp_s1 += lp_alpha * (raw_spark - _arc_bp_s1);
            _arc_bp_s2 += hp_alpha * (_arc_bp_s1 - _arc_bp_s2);
            float bp = _arc_bp_s1 - _arc_bp_s2;

            float gain = is_digital ? 0.085f : 0.12f;
            arcing_sig = bp * (arc_eff * gain);
        } else {
            _arc_bp_s1 *= 0.80f;
            _arc_bp_s2 *= 0.80f;
        }

        // Tap Impulse Voice
        float tap_sig = _tap.Process(white);
        float tactile = tap_sig * 0.85f + scratch_sig + arcing_sig;

        if (std::abs(tactile) > 1e-5f) {
            _res_active_cnt = 480;
        }

        if (_res_active_cnt > 0) {
            _res_active_cnt--;

            // Highpass filter (~20Hz DC blocker)
            constexpr float hp_g = 0.0026f;
            _hp_state += hp_g * (tactile - _hp_state);
            float sig = tactile - _hp_state;

            // Enclosure resonators (135Hz, 720Hz)
            if ((_step_cnt++ & 3) == 0) {
                _cached_chas = _res_chassis.Process(sig);
                _cached_ceram = _res_ceramic.Process(sig);
            }
            float chas = _cached_chas;
            float ceram = _cached_ceram;

            // Blend direct signal with resonators
            float disc_out = sig * 0.45f + chas * 0.40f + ceram * 0.25f;

            // High frequency surface damping
            _lp_damp += 0.70f * (disc_out - _lp_damp);

            // Safe acoustic output level for preamp warming and touch response
            float sat = SaturatePiezo(_lp_damp * 0.75f);
            out_mic = daisysp::fclamp(sat, -0.45f, 0.45f);
            _lp_damp = daisysp::fclamp(_lp_damp, -2.0f, 2.0f);
            _hp_state = daisysp::fclamp(_hp_state, -2.0f, 2.0f);
        } else {
            out_mic = 0.0f;
        }
    }

private:
    __attribute__((always_inline)) inline float SaturatePiezo(float x) const {
        return FastSoftSat(x * 0.75f);
    }

    float _sample_rate{48000.0f};

    Resonator _res_chassis;
    Resonator _res_ceramic;

    float _hp_state{0.0f};
    float _lp_damp{0.0f};

    TapVoice _tap;

    float _pink_b0{0.0f}, _pink_b1{0.0f}, _pink_b2{0.0f};
    float _lp_pulp{0.0f};
    float _smooth_rub{0.0f};
    float _arcing_target{0.0f};
    float _arcing_motion{0.0f};
    float _digital_target{0.0f};
    float _digital_ratio{0.0f};
    float _arc_bp_s1{0.0f};
    float _arc_bp_s2{0.0f};
    uint32_t _rng{0x93A11234u};

    uint16_t _res_active_cnt{0};
    uint8_t  _step_cnt{0};
    float    _cached_chas{0.0f};
    float    _cached_ceram{0.0f};
};

} // namespace bends
