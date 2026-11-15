#pragma once

#include <algorithm>
#include "dsp/stereo.h"
#include "dsp/filters.h"
#include "delay/registers.h"
#include "tuning/tuning.h"

namespace bends {

class FeedbackPath {
public:
    void Init() {
        Reset();
    }

    void Reset() {
        _wet_out = {0.0f, 0.0f};
        _damp = {0.0f, 0.0f};
        _hp = {0.0f, 0.0f};
        _mud_hp = {0.0f, 0.0f};
        _erode = {0.0f, 0.0f};
        _smooth_gain = 0.0f;
        _smooth_cross = 0.0f;
        _smooth_alpha = 0.98f;
        _smooth_alpha_r = 0.98f;
        _smooth_alpha_hp = 0.0f;
        _target_gain = 0.0f;
        _target_cross = 0.0f;
        _target_alpha = 0.98f;
        _target_alpha_r = 0.98f;
        _target_alpha_hp = 0.0f;
        _inc_gain = 0.0f;
        _inc_cross = 0.0f;
        _inc_alpha = 0.0f;
        _inc_alpha_r = 0.0f;
        _inc_alpha_hp = 0.0f;
        _larsen_howl = 0.0f;
        _infinite_hold = false;
        _larsen_active = false;
        _larsen_bp_l.Reset();
        _larsen_bp_r.Reset();
        _larsen_bp_l.SetBandpass(2400.0f, 6.0f);
        _larsen_bp_r.SetBandpass(2750.0f, 5.5f);
    }

    void PrepareBlock(const FeedbackRegisters& reg, float delay_knob_u = 0.5f, float clock_speed = 1.0f) {
        _smooth_gain = _target_gain;
        _smooth_cross = _target_cross;
        _smooth_alpha = _target_alpha;
        _smooth_alpha_r = _target_alpha_r;
        _smooth_alpha_hp = _target_alpha_hp;

        float drain_mult = (reg.drain > 0.02f) ? std::max(0.0f, 1.0f - reg.drain * 0.95f) : 1.0f;
        _eff_gain = std::clamp(reg.gain, 0.0f, tuning::kFeedbackCeiling) * drain_mult;
        if (reg.infinite_hold) {
            _eff_gain = 1.0f;
        }
        _infinite_hold = reg.infinite_hold;
        _larsen_howl = reg.larsen_howl;
        // Classic ping-pong crossfeed (0.0 = pure stereo dual delay, 1.0 = full ping-pong channel swap)
        float cross = std::clamp(reg.crossfeed, 0.0f, 1.0f);
        _cross_dir = 1.0f - cross;
        _cross_inv = cross;

        // Anti-aliasing HF damping when overclocked (> 1.0x):
        // Prevents recirculating repeats from accumulating supersonic alias reflection squeal
        float max_alpha = (clock_speed > 1.01f) ? std::clamp(0.98f / (clock_speed * 0.95f), 0.50f, 0.98f) : 0.98f;
        _alpha = std::clamp(reg.damp, 0.04f, max_alpha);
        // Symmetrical damping by default; subtle reflection absorption asymmetry only during active ping-pong bounce
        _alpha_r = (cross > 0.01f) ? std::clamp(reg.damp * 0.94f, 0.04f, max_alpha) : _alpha;

        // Short delay damping compensation (< 0.20):
        if (delay_knob_u < 0.20f) {
            float string_norm = 1.0f - (delay_knob_u * 5.0f); // 1.0 at min delay (highest pitch) -> 0.0 at 0.20
            float min_string_alpha = 0.92f + 0.06f * string_norm; // up to 0.98f
            _alpha = std::max(_alpha, min_string_alpha);
            _alpha_r = std::max(_alpha_r, min_string_alpha * 0.98f);
            if (_eff_gain > 0.60f) {
                _eff_gain = std::min(tuning::kFeedbackCeiling, _eff_gain * (1.0f + 0.08f * string_norm));
            }
        }
        _alpha_hp = std::clamp(reg.tone_hp, 0.0f, 0.35f);

        _loop_drive = 1.0f + reg.runaway * 0.50f;
        _recirc_fm = reg.recirc_fm;
        _erode_amt = reg.erode;

        _target_gain = _eff_gain;
        _target_cross = _cross_inv;
        _target_alpha = _alpha;
        _target_alpha_r = _alpha_r;
        _target_alpha_hp = _alpha_hp;

        constexpr float kRampStep = 1.0f / 64.0f;
        _inc_gain = (_target_gain - _smooth_gain) * kRampStep;
        _inc_cross = (_target_cross - _smooth_cross) * kRampStep;
        _inc_alpha = (_target_alpha - _smooth_alpha) * kRampStep;
        _inc_alpha_r = (_target_alpha_r - _smooth_alpha_r) * kRampStep;
        _inc_alpha_hp = (_target_alpha_hp - _smooth_alpha_hp) * kRampStep;
    }

    __attribute__((always_inline)) inline Stereo Process(Stereo wet,
                                                         float osc_sig,
                                                         const FeedbackRegisters& /*reg*/ = {},
                                                         float delay_knob_u = 0.5f,
                                                         float in_env = 0.0f) {
        // Per-sample parameter interpolation
        _smooth_cross += _inc_cross;
        _smooth_alpha += _inc_alpha;
        _smooth_alpha_r += _inc_alpha_r;
        _smooth_alpha_hp += _inc_alpha_hp;

        // 1. Asymmetric Bipolar Tone Filter Network (1-pole Low-Pass + 1-pole High-Pass)
        // Subtle L/R damping divergence mimics physical room reflection acoustic absorption
        _damp.l += _smooth_alpha * (wet.l - _damp.l);
        _damp.r += _smooth_alpha_r * (wet.r - _damp.r);

        Stereo toned = _damp;
        if (_smooth_alpha_hp > 0.0005f) {
            _hp.l += _smooth_alpha_hp * (_damp.l - _hp.l);
            _hp.r += _smooth_alpha_hp * (_damp.r - _hp.r);
            toned.l = _damp.l - _hp.l;
            toned.r = _damp.r - _hp.r;
        } else {
            _hp = {0.0f, 0.0f};
        }

        // Stereo crossfeed
        Stereo crossed = toned;
        if (__builtin_expect(_smooth_cross > 0.001f, 0)) {
            float c_inv = _smooth_cross;
            float c_dir = 1.0f - c_inv;
            crossed = {
                toned.l * c_dir + toned.r * c_inv,
                toned.r * c_dir + toned.l * c_inv
            };
        }

        // Output wet repeats
        _wet_out = crossed;

        // Linear ramped feedback gain
        _smooth_gain += _inc_gain;

        // Dynamic delay ducking under loud peaks (< 0.20 disabled for short delays)
        float tucking = (_infinite_hold || in_env <= 0.08f || delay_knob_u < 0.20f) ? 1.0f : (1.0f - std::min(0.06f, (in_env - 0.08f) * 0.15f));
        Stereo fb_sig = crossed * (_smooth_gain * tucking);

        // DC offset stabilization
        _mud_decim = !_mud_decim;
        if (_mud_decim) {
            _mud_hp.l += 0.0032f * (fb_sig.l - _mud_hp.l);
            _mud_hp.r += 0.0032f * (fb_sig.r - _mud_hp.r);
        }
        fb_sig.l -= _mud_hp.l * 0.08f;
        fb_sig.r -= _mud_hp.r * 0.08f;

        // 3. FLT <-> FBK Loop Erosion (high-pass whistling trails)
        if (_erode_amt > 0.01f) {
            float er = std::min(1.0f, _erode_amt);
            _erode.l += 0.04f * (fb_sig.l - _erode.l);
            _erode.r += 0.04f * (fb_sig.r - _erode.r);
            fb_sig.l = fb_sig.l * (1.0f - er) + (fb_sig.l - _erode.l) * (er * 1.05f);
            fb_sig.r = fb_sig.r * (1.0f - er) + (fb_sig.r - _erode.r) * (er * 1.05f);
        }

        // 4. OSC <-> FBK Recirculating AM Ring Mod / Stereo auto-pan
        if (std::abs(_recirc_fm) > 0.01f) {
            fb_sig.l *= (1.0f + osc_sig * 0.95f * _recirc_fm);
            fb_sig.r *= (1.0f - osc_sig * 0.95f * _recirc_fm);
        }

        // OUT <-> FBK feedback howl
        if (_larsen_howl > 0.01f) {
            float howl_l = _larsen_bp_l.Process(fb_sig.l) * (2.2f * _larsen_howl);
            float howl_r = _larsen_bp_r.Process(fb_sig.r) * (2.2f * _larsen_howl);
            fb_sig.l += howl_l;
            fb_sig.r += howl_r;
            _larsen_active = true;
        } else if (__builtin_expect(_larsen_active, 0)) {
            _larsen_bp_l.Reset();
            _larsen_bp_r.Reset();
            _larsen_active = false;
        }

        // 5. LoopGuard soft-clip ceiling protection
        if (_infinite_hold) {
            return {std::clamp(fb_sig.l, -1.0f, 1.0f), std::clamp(fb_sig.r, -1.0f, 1.0f)};
        }
        return (fb_sig * _loop_drive).SoftSat();
    }

    Stereo Damp() const { return _damp; }
    Stereo WetOut() const { return _wet_out; }
    float SmoothCross() const { return _smooth_cross; }
    float SmoothGain() const { return _smooth_gain; }

private:
    Stereo       _wet_out{0.0f, 0.0f};
    Stereo       _damp{0.0f, 0.0f};
    Stereo       _hp{0.0f, 0.0f};
    Stereo       _mud_hp{0.0f, 0.0f};
    Stereo       _erode{0.0f, 0.0f};
    Biquad       _larsen_bp_l;
    Biquad       _larsen_bp_r;

    // Block-rate hoisted coefficients
    float _eff_gain{0.0f};
    float _smooth_gain{0.0f};
    float _smooth_cross{0.0f};
    float _smooth_alpha{0.98f};
    float _smooth_alpha_r{0.98f};
    float _smooth_alpha_hp{0.0f};
    float _target_gain{0.0f};
    float _target_cross{0.0f};
    float _target_alpha{0.98f};
    float _target_alpha_r{0.98f};
    float _target_alpha_hp{0.0f};
    float _inc_gain{0.0f};
    float _inc_cross{0.0f};
    float _inc_alpha{0.0f};
    float _inc_alpha_r{0.0f};
    float _inc_alpha_hp{0.0f};
    float _alpha{0.98f};
    float _alpha_r{0.98f};
    float _alpha_hp{0.0f};
    float _cross_dir{1.0f};
    float _cross_inv{0.0f};
    float _loop_drive{1.0f};
    float _recirc_fm{0.0f};
    float _erode_amt{0.0f};
    float _larsen_howl{0.0f};
    bool  _infinite_hold{false};
    bool  _mud_decim{false};
    bool  _larsen_active{false};
};

} // namespace bends
