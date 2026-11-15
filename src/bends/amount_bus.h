#pragma once

#include "bends/slots.h"
#include "dsp/fast_math.h"
#include <algorithm>
#include <cmath>

namespace bends {

class AmountBus {
public:
    static constexpr size_t kNumSlots = static_cast<size_t>(Slot::Count);

    void Init(float control_rate_hz = 750.0f) {
        _control_rate_hz = control_rate_hz;
        float dt_sec = 1.0f / control_rate_hz;

        for (size_t i = 0; i < kNumSlots; ++i) {
            Slot s = static_cast<Slot>(i);
            float att_sec = std::max(0.0005f, SlotAttackMs(s) * 0.001f);
            float rel_sec = std::max(0.0005f, SlotReleaseMs(s) * 0.001f);

            // 1-pole filter alpha: alpha = 1 - exp(-dt / tau)
            _alpha_att[i] = std::clamp(1.0f - FastExp(-dt_sec / att_sec), 0.01f, 1.0f);
            _alpha_rel[i] = std::clamp(1.0f - FastExp(-dt_sec / rel_sec), 0.001f, 1.0f);

            _raw[i] = 0.0f;
            _smooth[i] = 0.0f;
        }
    }

    void ResetRaw() {
        for (size_t i = 0; i < kNumSlots; ++i) {
            _raw[i] = 0.0f;
        }
    }

    void Apply(Slot slot, float eff_amount) {
        size_t idx = static_cast<size_t>(slot);
        if (idx == 0 || idx >= kNumSlots) return;

        CombineRule rule = SlotCombineRule(slot);
        if (rule == CombineRule::Sum) {
            _raw[idx] += eff_amount;
        } else {
            _raw[idx] = std::max(_raw[idx], eff_amount);
        }
    }

    void UpdateBlock() {
        _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
        float wander = static_cast<float>(static_cast<int16_t>(_rng & 0xFFFF)) * (1.0f / 32768.0f);
        _slow_wander += 0.005f * (wander - _slow_wander);

        for (size_t i = 1; i < kNumSlots; ++i) {
            float target = _raw[i];
            float current = _smooth[i];
            float diff = target - current;

            if (diff > 0.0f) {
                _smooth[i] += diff * _alpha_att[i];
            } else {
                _smooth[i] += diff * _alpha_rel[i];
                if (std::abs(_smooth[i]) < 0.0002f) {
                    _smooth[i] = 0.0f;
                }
            }

            // Tactile tremor drift: simulates subtle involuntary pressure variations
            // of human fingertip contact (+/-1.2% at ~1-2 Hz)
            if (_smooth[i] > 0.04f && std::abs(diff) < 0.02f) {
                float breathing = _smooth[i] * (1.0f + _slow_wander * 0.012f);
                _smooth[i] = std::clamp(breathing, 0.0f, 1.5f);
            }
        }
    }

    inline float Get(Slot slot) const {
        return _smooth[static_cast<size_t>(slot)];
    }

    inline float Raw(Slot slot) const {
        return _raw[static_cast<size_t>(slot)];
    }

    inline float operator[](Slot slot) const {
        return Get(slot);
    }

private:
    float _control_rate_hz{750.0f};
    float _raw[kNumSlots]{0.0f};
    float _smooth[kNumSlots]{0.0f};
    float _alpha_att[kNumSlots]{1.0f};
    float _alpha_rel[kNumSlots]{1.0f};
    float _slow_wander{0.0f};
    uint32_t _rng{0x45A19CD1u};
};

} // namespace bends
