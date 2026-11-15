#pragma once

#include "dsp/fast_math.h"
#include <cmath>
#include <algorithm>

namespace bends {

// 1-pole DC blocker (highpass filter ~3 Hz at 48 kHz)
struct DcBlocker {
    float x1{0.0f};
    float y1{0.0f};

    inline void Reset() {
        x1 = 0.0f;
        y1 = 0.0f;
    }

    __attribute__((always_inline)) inline float Process(float in) {
        float out = in - x1 + 0.9996f * y1;
        x1 = in;
        y1 = daisysp::fclamp(out, -4.0f, 4.0f);
        return out;
    }
};

// Simple 1-pole lowpass filter
struct OnePole {
    float state{0.0f};

    inline void Reset(float initial = 0.0f) {
        state = initial;
    }

    __attribute__((always_inline)) inline float Process(float in, float alpha) {
        state += alpha * (in - state);
        return state;
    }
};

// Parameter smoother for control-rate slewing (zipper noise elimination)
struct ParameterSmoother {
    float target{0.0f};
    float current{0.0f};
    float coeff{0.008f};

    inline void Init(float initial = 0.0f, float slew = 0.008f) {
        target = initial;
        current = target;
        coeff = slew;
    }

    inline void SetTarget(float t) {
        target = t;
    }

    __attribute__((always_inline)) inline float Tick() {
        current += coeff * (target - current);
        return current;
    }

    inline float Value() const {
        return current;
    }
};

// Direct Form II Transposed Biquad Filter (2-pole)
struct Biquad {
    float b0{1.0f}, b1{0.0f}, b2{0.0f};
    float a1{0.0f}, a2{0.0f};
    float s1{0.0f}, s2{0.0f};

    inline void Reset() {
        s1 = 0.0f;
        s2 = 0.0f;
    }

    __attribute__((always_inline)) inline float Process(float in) {
        float out = b0 * in + s1;
        s1 = b1 * in - a1 * out + s2;
        s2 = b2 * in - a2 * out;
        return out;
    }

    void SetLowpass(float f0, float Q, float fs = 48000.0f) {
        float w0 = 2.0f * 3.141592653589793f * std::clamp(f0, 20.0f, fs * 0.45f) / fs;
        float cos_w = FastCos(w0);
        float sin_w = FastSin(w0);
        float alpha = sin_w / (2.0f * std::max(0.1f, Q));
        float a0_inv = 1.0f / (1.0f + alpha);
        b0 = (1.0f - cos_w) * 0.5f * a0_inv;
        b1 = (1.0f - cos_w) * a0_inv;
        b2 = (1.0f - cos_w) * 0.5f * a0_inv;
        a1 = (-2.0f * cos_w) * a0_inv;
        a2 = (1.0f - alpha) * a0_inv;
    }

    void SetHighpass(float f0, float Q, float fs = 48000.0f) {
        float w0 = 2.0f * 3.141592653589793f * std::clamp(f0, 10.0f, fs * 0.45f) / fs;
        float cos_w = FastCos(w0);
        float sin_w = FastSin(w0);
        float alpha = sin_w / (2.0f * std::max(0.1f, Q));
        float a0_inv = 1.0f / (1.0f + alpha);
        b0 = (1.0f + cos_w) * 0.5f * a0_inv;
        b1 = -(1.0f + cos_w) * a0_inv;
        b2 = (1.0f + cos_w) * 0.5f * a0_inv;
        a1 = (-2.0f * cos_w) * a0_inv;
        a2 = (1.0f - alpha) * a0_inv;
    }

    void SetBandpass(float f0, float Q, float fs = 48000.0f) {
        float w0 = 2.0f * 3.141592653589793f * std::clamp(f0, 20.0f, fs * 0.45f) / fs;
        float cos_w = FastCos(w0);
        float sin_w = FastSin(w0);
        float alpha = sin_w / (2.0f * std::max(0.1f, Q));
        float a0_inv = 1.0f / (1.0f + alpha);
        b0 = (sin_w * 0.5f) * a0_inv;
        b1 = 0.0f;
        b2 = -(sin_w * 0.5f) * a0_inv;
        a1 = (-2.0f * cos_w) * a0_inv;
        a2 = (1.0f - alpha) * a0_inv;
    }
};

} // namespace bends
