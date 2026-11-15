#pragma once

#include <cmath>
#include <algorithm>
#include <cstdint>
#include "daisysp.h"

namespace bends {

static constexpr float kPi    = 3.14159265358979323846f;
static constexpr float kTwoPi = 6.28318530717958647692f;

// On Cortex-M7, hardware Flush-to-Zero (FTZ) and Default-NaN (DN) modes are
// enabled in FPSCR during initialization (see src/main.cpp). Subnormal numbers
// flush to zero in hardware with zero CPU cycle penalty.
__attribute__((always_inline)) inline float FlushDenormal(float val) {
    return val;
}

// Single-cycle FPU square root for Cortex-M7 (0 soft-float libm calls)
__attribute__((always_inline)) inline float FastSqrt(float x) {
    return __builtin_sqrtf(std::max(0.0f, x));
}

// Single-cycle FPU float sanitizer for Cortex-M7 (0 soft-float libm calls, 0 FPU-core register moves)
// Returns fallback if x is NaN or +/-Inf
__attribute__((always_inline)) inline float Sanitize(float x, float fallback = 0.0f) {
    if (x >= -3.402823466e+38F && x <= 3.402823466e+38F) return x;
    return fallback;
}

// Bounded audio signal sanitizer: catches NaNs/Infs and clamps to [-bound, bound]
__attribute__((always_inline)) inline float SanitizeAudio(float x, float bound = 4.0f) {
    if (x >= -bound && x <= bound) return x;
    if (x > bound) return bound;
    if (x < -bound) return -bound;
    return 0.0f;
}

__attribute__((always_inline)) inline float FastTanh(float x) {
    float cx = std::clamp(x, -3.0f, 3.0f);
    float x2 = cx * cx;
    return cx * (27.0f + x2) / (27.0f + 9.0f * x2);
}

// Ultra-fast division-free cubic soft saturator on [-1.5, 1.5] (branchless, 0 libm calls)
__attribute__((always_inline)) inline float FastSoftSat(float x) {
    float cx = std::clamp(x, -1.5f, 1.5f);
    return cx * (1.0f - 0.14814815f * cx * cx);
}

__attribute__((always_inline)) inline float FastPow2(float x) {
    if (__builtin_expect(!(x >= -30.0f), 0)) return (x != x) ? 1.0f : 0.0f;
    float clamped = std::min(x, 30.0f);
    int i = static_cast<int>(clamped);
    float f = clamped - static_cast<float>(i);
    if (f < 0.0f) {
        f += 1.0f;
        i -= 1;
    }
    // Minimax polynomial for 2^f on [0, 1]
    float p = 1.0f + f * (0.693019f + f * (0.241404f + f * 0.052032f));
    union {
        uint32_t u;
        float f;
    } pun;
    pun.u = static_cast<uint32_t>((i + 127) << 23);
    return p * pun.f;
}

// Ultra-fast e^x approximation using FastPow2
__attribute__((always_inline)) inline float FastExp(float x) {
    return FastPow2(x * 1.44269504f);
}

// Fast parabolic sine approximation on [-pi, pi] (~6 cycles)
__attribute__((always_inline)) inline float FastSin(float rad) {
    float x = rad * (1.0f / kTwoPi);
    x -= static_cast<int>(x + (x >= 0.0f ? 0.5f : -0.5f));
    x *= kTwoPi;

    constexpr float B = 4.0f / kPi;
    constexpr float C = -4.0f / (kPi * kPi);
    float y = B * x + C * x * std::abs(x);
    constexpr float P = 0.225f;
    return P * (y * std::abs(y) - y) + y;
}

__attribute__((always_inline)) inline float FastCos(float rad) {
    return FastSin(rad + (0.5f * kPi));
}

// Fast tan(x) polynomial approximation for filter pre-warping on [0, 1.25]
__attribute__((always_inline)) inline float FastTan(float x) {
    x = daisysp::fclamp(x, 0.0f, 1.25f);
    float x2 = x * x;
    return x * (1.0f + x2 * (0.3333333f + x2 * (0.1333333f + x2 * 0.0539683f)));
}

// Fast DAC quantization with hardware float-to-int rounding (no libc std::round)
__attribute__((always_inline)) inline float QuantizeDac(float in, float levels, float inv, float dnl) {
    float scaled = in * levels;
    float rounded = static_cast<float>(static_cast<int32_t>(scaled + (scaled >= 0.0f ? 0.5f : -0.5f)));
    float q = rounded * inv;
    if (rounded != 0.0f) {
        float sgn = (in >= 0.0f) ? 1.0f : -1.0f;
        q += sgn * dnl * inv * 0.25f;
    }
    return daisysp::fclamp(q, -1.0f, 1.0f);
}

// 4-point Hermite (cubic) interpolation polynomial.
// Note: Circular delay lines in external SDRAM utilize 2-point linear interpolation
// to minimize FMC burst latency cycles, while Hermite is available for internal wavetable reads.
__attribute__((always_inline)) inline float FastHermite4(float ym1, float y0, float y1, float y2, float t) {
    float c0 = y0;
    float c1 = 0.5f * (y1 - ym1);
    float c2 = ym1 - 2.5f * y0 + 2.0f * y1 - 0.5f * y2;
    float c3 = 0.5f * (y2 - ym1) + 1.5f * (y0 - y1);
    return ((c3 * t + c2) * t + c1) * t + c0;
}

} // namespace bends
