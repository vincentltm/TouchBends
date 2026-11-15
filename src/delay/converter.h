#pragma once

#include "dsp/stereo.h"
#include "dsp/filters.h"
#include "parts/lofi.h"
#include "delay/registers.h"
#include <algorithm>

namespace bends {

class Converter {
public:
    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        _lofi_l.Init();
        _lofi_r.Init();
    }

    void Reset() {
        _lofi_l.Reset();
        _lofi_r.Reset();
        _smooth_strobe_l = 1.0f;
        _smooth_strobe_r = 1.0f;
    }

    inline void Prepare(float clock_ratio, const DataRegisters& data) {
        _lofi_l.Prepare(clock_ratio, _sample_rate);
        _lofi_r.Prepare(clock_ratio, _sample_rate);
        _can_bypass = (clock_ratio >= 0.99f && data.filter_bypass <= 0.01f &&
                       _smooth_strobe_l >= 0.999f && _smooth_strobe_r >= 0.999f &&
                       data.strobe_l >= 0.999f && data.strobe_r >= 0.999f &&
                       data.stuck_mask == 0 && data.resolution >= 16);
    }

    __attribute__((always_inline)) static inline Stereo ProcessDataBus(Stereo in, const DataRegisters& data, float clock_speed = 1.0f) {
        Stereo out = in;
        if (__builtin_expect(data.stuck_mask != 0, 0)) {
            out.l = ApplyBitFracture(out.l, data.stuck_mask);
            out.r = ApplyBitFracture(out.r, data.stuck_mask);
        }
        if (__builtin_expect(data.resolution < 16, 0)) {
            out.l = ApplyBitDepth(out.l, data.resolution);
            out.r = ApplyBitDepth(out.r, data.resolution);
        }
        if (clock_speed < 0.95f) {
            out.l = ApplyR2RCompand(out.l);
            out.r = ApplyR2RCompand(out.r);
        }
        return out;
    }

    __attribute__((always_inline)) inline Stereo Process(Stereo in,
                                                         float clock_speed,
                                                         const DataRegisters& data) {
        if (__builtin_expect(_can_bypass, 1)) {
            return in;
        }

        // 1. Downsampler & reconstruction filter
        Stereo out = {
            _lofi_l.Process(in.l, clock_speed, data.filter_bypass),
            _lofi_r.Process(in.r, clock_speed, data.filter_bypass)
        };

        // 2. Data Bus: Strobe gating with soft RC decay to eliminate digital edge clicks
        constexpr float kStrobeSlew = 0.08f;
        if (__builtin_expect(_smooth_strobe_l < 0.999f || data.strobe_l < 0.999f, 0)) {
            _smooth_strobe_l += kStrobeSlew * (data.strobe_l - _smooth_strobe_l);
            out.l *= _smooth_strobe_l;
        }
        if (__builtin_expect(_smooth_strobe_r < 0.999f || data.strobe_r < 0.999f, 0)) {
            _smooth_strobe_r += kStrobeSlew * (data.strobe_r - _smooth_strobe_r);
            out.r *= _smooth_strobe_r;
        }

        return ProcessDataBus(out, data, clock_speed);
    }

private:
    __attribute__((always_inline)) static inline float ApplyR2RCompand(float x) {
        float abs_x = std::abs(x);
        // Active on low-level signals (< -12 dBFS)
        if (abs_x < 1e-4f || abs_x > 0.25f) return x;
        float sign = (x >= 0.0f) ? 1.0f : -1.0f;
        constexpr float kSteps = 2048.0f;
        constexpr float kInvSteps = 1.0f / kSteps;
        float scaled = abs_x * kSteps;
        float step_q = static_cast<float>(static_cast<int32_t>(scaled + 0.5f)) * kInvSteps;
        float warm = 0.85f * abs_x + 0.15f * step_q;
        return sign * warm;
    }
    static inline float ApplyBitFracture(float x, uint32_t mask) {
        float abs_x = std::abs(x);
        if (abs_x < 0.005f) return x; // Silence gate
        float sign = (x >= 0.0f) ? 1.0f : -1.0f;
        uint16_t mag = static_cast<uint16_t>(std::clamp(abs_x, 0.0f, 1.0f) * 32767.0f);
        // Scramble bits proportional to active signal level
        uint16_t dynamic_mask = static_cast<uint16_t>(mask) & (mag | (mag >> 1) | (mag >> 2));
        mag ^= dynamic_mask;
        return sign * (static_cast<float>(mag) * (1.0f / 32767.0f)) * 0.75f; // Headroom trim
    }

    static inline float ApplyBitDepth(float x, uint8_t bits) {
        static constexpr float kHalfSteps[17] = {
            0.0f, 0.0f, 0.0f, 0.0f,
            8.0f, 16.0f, 32.0f, 64.0f,
            128.0f, 256.0f, 512.0f, 1024.0f,
            2048.0f, 4096.0f, 8192.0f, 16384.0f, 32768.0f
        };
        static constexpr float kInvHalfSteps[17] = {
            0.0f, 0.0f, 0.0f, 0.0f,
            1.0f / 8.0f, 1.0f / 16.0f, 1.0f / 32.0f, 1.0f / 64.0f,
            1.0f / 128.0f, 1.0f / 256.0f, 1.0f / 512.0f, 1.0f / 1024.0f,
            1.0f / 2048.0f, 1.0f / 4096.0f, 1.0f / 8192.0f, 1.0f / 16384.0f, 1.0f / 32768.0f
        };
        uint8_t b = std::clamp<uint8_t>(bits, 4, 16);
        float half_steps = kHalfSteps[b];
        float inv_half_steps = kInvHalfSteps[b];
        float scaled = std::clamp(x, -1.0f, 1.0f) * half_steps;
        float rounded = __builtin_roundf(scaled);
        float quant = rounded * inv_half_steps;
        // Soft floor on signals rounding to zero
        if (__builtin_expect(rounded == 0.0f && std::abs(x) > 1e-5f, 0)) {
            return x * 0.50f;
        }
        return quant;
    }

    float     _sample_rate{48000.0f};
    float     _smooth_strobe_l{1.0f};
    float     _smooth_strobe_r{1.0f};
    Lofi      _lofi_l;
    Lofi      _lofi_r;
    bool      _can_bypass{true};
};

} // namespace bends
