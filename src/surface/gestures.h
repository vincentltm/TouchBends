#pragma once

#include <cmath>
#include <algorithm>
#include <cstdint>
#include "dsp/fast_math.h"

namespace bends {

// Tracks physical tap impulses and continuous rub/scratch velocities on capacitive pads
struct PadGestureTracker {
    enum State : uint8_t {
        Idle,
        Onset,
        Held
    };

    State   state{Idle};
    uint8_t rest_scans{10};
    uint8_t onset_scans{0};
    uint8_t lockout_scans{0};
    float   onset_peak_dp{0.0f};
    float   onset_max_p{0.0f};
    float   rub_speed{0.0f};
    float   prev_p{0.0f};

    void Init() {
        state = Idle;
        rest_scans = 10;
        onset_scans = 0;
        lockout_scans = 0;
        onset_peak_dp = 0.0f;
        onset_max_p = 0.0f;
        rub_speed = 0.0f;
        prev_p = 0.0f;
    }

    void Update(float p, float& out_tap_vel) {
        out_tap_vel = 0.0f;
        float dp = p - prev_p;

        if (p < 0.02f) {
            if (rest_scans < 255) rest_scans++;
            state = Idle;
            rub_speed = 0.0f;
            onset_scans = 0;
            lockout_scans = 0;
            prev_p = p;
            return;
        }

        switch (state) {
        case Idle: {
            if (rest_scans >= 3) {
                state = Onset;
                onset_scans = 1;
                onset_peak_dp = (dp > 0.0f) ? dp : 0.0f;
                onset_max_p = p;
                rub_speed = 0.0f;
            } else {
                state = Held;
                lockout_scans = 6;
                rub_speed = 0.0f;
            }
            rest_scans = 0;
            break;
        }

        case Onset: {
            rest_scans = 0;
            onset_scans++;
            if (dp > onset_peak_dp) {
                onset_peak_dp = dp;
            }
            if (p > onset_max_p) {
                onset_max_p = p;
            }

            if (onset_max_p > 0.06f && onset_peak_dp > 0.025f) {
                float vel = onset_peak_dp * 3.2f + (onset_max_p - 0.06f) * 1.6f;
                out_tap_vel = std::clamp(vel, 0.20f, 1.0f);
                state = Held;
                lockout_scans = 5; // ~15ms settle lockout so landing vibration does not false-scratch
                rub_speed = 0.0f;
            } else if (onset_scans >= 6) {
                // Soft touchdown: settle cleanly into Held
                state = Held;
                lockout_scans = 2;
                rub_speed = 0.0f;
            }
            break;
        }

        case Held: {
            rest_scans = 0;
            if (lockout_scans > 0) {
                lockout_scans--;
                rub_speed = 0.0f;
            } else {
                float abs_dp = std::abs(dp);
                // Deadband of 0.0035f filters out sensor capacitance jitter (silence when still)
                if (abs_dp > 0.0035f) {
                    float excess = abs_dp - 0.0035f;
                    // Responsive curve: square root gives presence to feather scratches
                    float motion = std::clamp(excess * 18.0f + FastSqrt(excess) * 0.50f, 0.0f, 1.0f);
                    rub_speed += (motion - rub_speed) * 0.35f;
                } else {
                    rub_speed *= 0.75f;
                    if (rub_speed < 0.002f) rub_speed = 0.0f;
                }
            }
            break;
        }
        }

        prev_p = p;
    }
};

} // namespace bends
