#pragma once

#include <algorithm>
#include <cmath>
#include "tuning/tuning.h"
#include "dsp/fast_math.h"

namespace bends {
namespace mappings {

// Log-audio feedback taper:
// s <= 0.60: 0.0 to 0.58
// 0.60 < s <= 0.88: 0.58 to 0.98
// s > 0.88: 0.98 to 1.15 (self-oscillation range)
inline float FeedbackTaper(float s_fbk) {
    s_fbk = std::clamp(s_fbk, 0.0f, 1.0f);
    if (s_fbk <= 0.60f) {
        float t = s_fbk * (1.0f / 0.60f);
        return 0.58f * (t * t);
    } else if (s_fbk <= 0.88f) {
        float t = (s_fbk - 0.60f) * (1.0f / 0.28f);
        return 0.58f + 0.40f * (t * (2.0f - t));
    } else {
        float t = (s_fbk - 0.88f) * (1.0f / 0.12f);
        return 0.98f + 0.17f * t;
    }
}

// Bipolar Feedback Knob (S35):
// Center [0.48, 0.52]: 0% feedback (single echo / slapback)
// Turn CCW (< 0.48): Standard dual/mono delay feedback (0.0 to 1.12), 0 crossfeed
// Turn CW (> 0.52): Stereo Ping-Pong delay feedback (0.0 to 1.12), crossfeed scales 0.0 to 1.0
inline void BipolarFeedback(float s_fbk, float& out_gain, float& out_crossfeed) {
    s_fbk = std::clamp(s_fbk, 0.0f, 1.0f);
    if (s_fbk < 0.48f) {
        float norm = (0.48f - s_fbk) * (1.0f / 0.48f); // 0.0 at noon -> 1.0 at CCW
        out_gain = FeedbackTaper(norm);
        out_crossfeed = 0.0f;
    } else if (s_fbk > 0.52f) {
        float norm = (s_fbk - 0.52f) * (1.0f / 0.48f); // 0.0 at noon -> 1.0 at CW
        out_gain = FeedbackTaper(norm);
        out_crossfeed = std::clamp(0.40f + 0.60f * norm, 0.0f, 1.0f);
    } else {
        out_gain = 0.0f;
        out_crossfeed = 0.0f;
    }
}

// Bipolar tone damping for delay feedback loop:
// <= 0.50 (CCW): Low-pass rolls down from 20 kHz to 350 Hz, high-pass stays open (0.0).
// Center (0.50): Flat full-bandwidth transparent repeats (lp = 0.98, hp = 0.0).
// > 0.50 (CW): Low-pass stays open (0.98), high-pass sweeps up from 10 Hz to 2.5 kHz (hp up to 0.28).
inline void CalculateDelayTone(float flt_tone_norm, float vcc, float& out_lp, float& out_hp) {
    flt_tone_norm = daisysp::fclamp(flt_tone_norm, 0.0f, 1.0f);
    // DAC reconstruction filter droop under starved rail:
    float v_scale = 0.60f + 0.40f * daisysp::fclamp(vcc * (1.5f - 0.5f * vcc), 0.0f, 1.0f);
    if (flt_tone_norm <= 0.50f) {
        float t = flt_tone_norm * 2.0f; // 0.0 to 1.0
        out_lp = daisysp::fclamp((0.04f + 0.94f * (t * t)) * v_scale, 0.04f, 0.98f);
        out_hp = 0.0f; // High-pass disabled below center detent
    } else {
        float t = (flt_tone_norm - 0.50f) * 2.0f; // 0.0 to 1.0
        out_lp = 0.98f * v_scale;
        out_hp = daisysp::fclamp(0.28f * (t * t), 0.0f, 0.35f);
    }
}

// Physical modeling string damping derived from tone control
inline float StringDamp(float flt_tone_norm, float vcc) {
    float lp, hp;
    CalculateDelayTone(flt_tone_norm, vcc, lp, hp);
    return lp;
}

// VCC pitch sag curve for oscillator
inline float OscVccSag(float vcc) {
    float sag = daisysp::fclamp(vcc * (0.35f + 0.65f * vcc), 0.05f, 1.0f);
    if (vcc < tuning::kBroken) {
        float rel = vcc / tuning::kBroken;
        sag *= (0.25f + 0.75f * rel);
    }
    return sag;
}

// Base delay time calculation (Zone 1: Karplus string, Zone 2: Echo / Space / Looper)
inline float DelaySamples(float knob_dly, float sample_rate = 48000.0f) {
    knob_dly = std::clamp(knob_dly, 0.0f, 1.0f);
    float base_time;
    if (knob_dly < 0.20f) {
        float k = knob_dly / 0.20f;
        base_time = (14.0f / sample_rate) * FastPow2(k * 6.421434f);
    } else {
        float w = (knob_dly - 0.20f) / 0.80f;
        base_time = 0.025f * FastPow2(w * 9.643856f);
    }
    base_time = std::clamp(base_time, 14.0f / sample_rate, 20.0f);
    return std::clamp(base_time * sample_rate, tuning::kMinDelaySamples, static_cast<float>(tuning::kDelayFrames - 2));
}

} // namespace mappings
} // namespace bends
