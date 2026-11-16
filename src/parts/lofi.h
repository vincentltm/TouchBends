#pragma once

#include "dsp/fast_math.h"
#include <algorithm>

namespace bends {

// Variable-rate downsampler with ADM slew limiting and reconstruction filtering
struct Lofi {
    float hold_sample{0.0f};
    float clock_phase{0.0f};
    float integrator{0.0f};
    float lp1{0.0f};
    float lp2{0.0f};

    float clock_speed{1.0f};
    float max_slew{0.40f};
    float alpha{0.45f};
    float lofi_amt{0.0f};
    float whine_scale{0.0f};
    float detent_blend{-1.0f};
    bool  active{false};

    void Init() {
        Reset();
    }

    void Reset() {
        hold_sample = 0.0f;
        clock_phase = 0.0f;
        integrator = 0.0f;
        lp1 = 0.0f;
        lp2 = 0.0f;
        clock_speed = 1.0f;
        max_slew = 0.40f;
        alpha = 0.45f;
        lofi_amt = 0.0f;
        whine_scale = 0.0f;
        detent_blend = -1.0f;
        active = true;
    }

    inline void Prepare(float clock_ratio, float sample_rate) {
        float target = std::clamp(clock_ratio, 0.04f, 1.0f);
        if (std::abs(target - clock_speed) < 0.003f && active) return;
        clock_speed = target;
        active = true;

        if (clock_speed >= 0.96f) {
            max_slew = 10.0f;
            alpha = 0.95f;
            lofi_amt = 0.0f;
            whine_scale = 0.0f;
            detent_blend = (clock_speed < 0.99f) ? std::clamp((0.99f - clock_speed) * 20.0f, 0.0f, 1.0f) : -1.0f;
        } else {
            // Underclock intensity: 0.0 at 0.92x, ramping to 1.0 at 0.20x
            lofi_amt = std::clamp((0.92f - clock_speed) * 1.3888889f, 0.0f, 1.0f);
            whine_scale = (lofi_amt > 0.005f) ? (lofi_amt * lofi_amt * 0.0030f) : 0.0f;
            detent_blend = (clock_speed > 0.94f) ? std::clamp((0.99f - clock_speed) * 20.0f, 0.0f, 1.0f) : -1.0f;

            // ADM slew rate limiting
            max_slew = 0.040f + 0.36f * (clock_speed * clock_speed);

            // Reconstruction filter cutoff tracking clock
            float inv_sr = (sample_rate > 0.0f) ? (1.0f / sample_rate) : (1.0f / 48000.0f);
            float cutoff = std::clamp(5200.0f * clock_speed + 2800.0f, 2800.0f, 18000.0f);
            float w = cutoff * (kTwoPi * inv_sr);
            alpha = std::clamp(w / (1.0f + w), 0.035f, 0.95f);
        }
    }

    __attribute__((always_inline)) inline float Process(float in) {
        return Process(in, clock_speed, 0.0f);
    }

    __attribute__((always_inline)) inline float Process(float in, float dynamic_speed, float filter_bypass = 0.0f) {
        if (!active) {
            integrator = in; lp1 = in; lp2 = in; hold_sample = in;
            return in;
        }

        // Fast pure bypass at nominal clock rate (1.0x center detent)
        if (__builtin_expect(dynamic_speed >= 0.99f && filter_bypass <= 0.01f, 1)) {
            integrator = in; lp1 = in; lp2 = in; hold_sample = in;
            return in;
        }

        // Sample-and-hold at clock phase
        clock_phase += dynamic_speed;
        if (clock_phase >= 1.0f) {
            clock_phase -= 1.0f;
            if (__builtin_expect(clock_phase >= 1.0f, 0)) {
                clock_phase -= 1.0f;
            }
            hold_sample = in;
        }

        // ADM slew rate limiting
        float diff = hold_sample - integrator;
        float step = std::clamp(diff, -max_slew, max_slew);
        integrator += step;

        // Reconstruction low-pass
        float out;
        if (__builtin_expect(filter_bypass >= 0.99f, 0)) {
            lp1 = integrator;
            lp2 = integrator;
            out = integrator;
        } else {
            lp1 += alpha * (integrator - lp1);
            lp2 += alpha * (lp1 - lp2);
            float eff_grit = (filter_bypass > 0.01f) ? filter_bypass : (lofi_amt * 0.18f);
            out = lp2 + eff_grit * (integrator - lp2);
        }

        // Clock heterodyne whine
        if (whine_scale > 0.0f) {
            float abs_in = std::abs(in);
            if (abs_in > 0.005f) {
                float tri = (clock_phase < 0.5f) ? (4.0f * clock_phase - 1.0f) : (3.0f - 4.0f * clock_phase);
                out += tri * (whine_scale * std::min(abs_in * 2.0f, 1.0f));
            }
        }

        // Crossfade to bypass near detent (0.94x .. 0.99x)
        if (__builtin_expect(detent_blend >= 0.0f && filter_bypass <= 0.01f, 0)) {
            return in + detent_blend * (out - in);
        }
        return out;
    }
};

} // namespace bends
