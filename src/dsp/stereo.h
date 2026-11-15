#pragma once

#include <algorithm>
#include "dsp/fast_math.h"

namespace bends {

struct Stereo {
    float l{0.0f};
    float r{0.0f};

    constexpr Stereo() = default;
    constexpr Stereo(float mono) : l(mono), r(mono) {}
    constexpr Stereo(float left, float right) : l(left), r(right) {}

    inline Stereo operator+(const Stereo& o) const { return {l + o.l, r + o.r}; }
    inline Stereo operator-(const Stereo& o) const { return {l - o.l, r - o.r}; }
    inline Stereo operator*(float s) const { return {l * s, r * s}; }
    inline Stereo operator*(const Stereo& o) const { return {l * o.l, r * o.r}; }

    inline Stereo& operator+=(const Stereo& o) {
        l += o.l;
        r += o.r;
        return *this;
    }

    inline Stereo& operator*=(float s) {
        l *= s;
        r *= s;
        return *this;
    }

    inline float MaxAbs() const {
        return std::max(std::abs(l), std::abs(r));
    }

    inline Stereo SoftSat() const {
        return {FastSoftSat(l), FastSoftSat(r)};
    }

    inline Stereo Sanitize(float fallback = 0.0f) const {
        return {bends::Sanitize(l, fallback), bends::Sanitize(r, fallback)};
    }
};

inline Stereo operator*(float s, const Stereo& st) {
    return {s * st.l, s * st.r};
}

} // namespace bends
