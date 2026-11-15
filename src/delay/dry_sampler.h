#pragma once

#include "dsp/stereo.h"
#include "delay/registers.h"
#include <algorithm>
#include <cstdint>

namespace bends {

class DrySampler {
public:
    void Init() {
        Reset();
    }

    void Reset() {
        _hold = {0.0f, 0.0f};
        _phase = 0.0f;
        _smooth_strobe = 1.0f;
        _can_bypass = true;
        _alias_grit = 0.0f;
    }

    inline void Prepare(float clock_ratio, const DataRegisters& data) {
        _can_bypass = (clock_ratio >= 0.99f && _smooth_strobe >= 0.999f &&
                       data.strobe_l >= 0.999f && data.strobe_r >= 0.999f);
        _alias_grit = (clock_ratio < 0.95f) ? (1.0f - clock_ratio) * 0.50f : 0.0f;
    }

    __attribute__((always_inline)) inline Stereo Process(Stereo dry,
                                                         float clock_rate,
                                                         const DataRegisters& data,
                                                         bool /*is_chop*/ = false) {
        // Fast bypass at nominal clock rate without active strobe gating
        if (__builtin_expect(_can_bypass, 1)) {
            _hold = dry;
            return dry;
        }

        // Slewed strobe gating
        constexpr float kStrobeSlew = 0.08f;
        float target_strobe = std::min(data.strobe_l, data.strobe_r);
        if (__builtin_expect(_smooth_strobe < 0.999f || target_strobe < 0.999f, 0)) {
            _smooth_strobe += kStrobeSlew * (target_strobe - _smooth_strobe);
        }

        // Clock sample-and-hold downsampling
        _phase += clock_rate;
        if (_phase >= 1.0f) {
            _phase -= 1.0f;
            _hold = dry;
        }

        // Blend filtered signal with S&H hold
        Stereo out = dry * (1.0f - _alias_grit) + _hold * _alias_grit;
        return { out.l * _smooth_strobe, out.r * _smooth_strobe };
    }

private:
    Stereo _hold{0.0f, 0.0f};
    float  _phase{0.0f};
    float  _smooth_strobe{1.0f};
    float  _alias_grit{0.0f};
    bool   _can_bypass{true};
};

} // namespace bends
