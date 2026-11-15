#pragma once

#include "dsp/fast_math.h"
#include "bends/mod_pack.h"
#include <algorithm>

#include <cmath>

namespace bends {

class Mixer {
public:
    void Reset() {
        _wet_g = 0.0f;
        _dry_g = 1.0f;
        _dry_machine_morph = 0.0f;
        _smooth_wet = 0.0f;
        _smooth_dry = 1.0f;
        _smooth_machine_morph = 0.0f;
        _drive_active = false;
        _drive_k = 1.0f;
        _drive_makeup = 1.0f;
        _dropout_phase = 0.0f;
        _smooth_opto_l = 1.0f;
        _smooth_opto_r = 1.0f;
    }

    inline void PrepareBlock(float mix_control, const MixerMods& mods) {
        float mix_wet = std::clamp(mix_control, 0.0f, 1.0f);
        mix_wet = std::max(mix_wet, std::min(1.0f, mods.wet_only));

        // Musical Dry-Through Mix Law:
        if (mix_wet <= 0.50f) {
            float t = mix_wet * 2.0f; // in [0, 1]
            _dry_g = 1.0f;
            _wet_g = t * (0.50f + 0.50f * t);
            _dry_machine_morph = t;
        } else {
            float w = (mix_wet - 0.50f) * 2.0f; // in [0, 1]
            _dry_g = 1.0f - w;
            _wet_g = 1.0f;
            _dry_machine_morph = 1.0f;
        }

        if (mods.wet_only > 0.01f) {
            float wo = std::min(1.0f, mods.wet_only);
            _wet_g = std::max(_wet_g, wo);
            _dry_g *= (1.0f - wo);
        }

        constexpr float kBlockDiv = 1.0f / 64.0f;
        _inc_dry = (_dry_g - _smooth_dry) * kBlockDiv;
        _inc_wet = (_wet_g - _smooth_wet) * kBlockDiv;
        _inc_morph = (_dry_machine_morph - _smooth_machine_morph) * kBlockDiv;

        _drive_active = (mods.drive > 0.01f);
        if (_drive_active) {
            float d = std::min(1.0f, mods.drive);
            _drive_k = 1.0f + 4.0f * d;
            _drive_makeup = 1.0f / (1.0f + 0.5f * d);
        }
    }

    inline bool ClockMorphActive() const {
        return (_dry_machine_morph > 0.001f || _smooth_machine_morph > 0.001f);
    }

    __attribute__((always_inline)) inline void Process(
        float raw_l, float raw_r,
        float flt_l, float flt_r,
        float wet_l, float wet_r,
        float dry_sampled_l, float dry_sampled_r,
        float ping_strike,
        float tap_mix_l, float tap_mix_r,
        float tap_out_l, float tap_out_r,
        const MixerMods& mods,
        float shunt_both,
        float& out_l, float& out_r) {
        // Linear parameter dezippering without pipeline stalls
        _smooth_dry           += _inc_dry;
        _smooth_wet           += _inc_wet;
        _smooth_machine_morph += _inc_morph;

        // Dry signal smoothly morphs from clean direct preamp into the unrepeated delay machine tap
        float morph_amt = _smooth_machine_morph;
        if (__builtin_expect(mods.amp_direct > 0.01f, 0)) {
            morph_amt *= (1.0f - std::min(1.0f, mods.amp_direct));
        }
        float dry_l = raw_l + morph_amt * (dry_sampled_l - raw_l);
        float dry_r = raw_r + morph_amt * (dry_sampled_r - raw_r);

        // Acoustic ping strike pulse blends in when Pad 04 is tapped
        if (__builtin_expect(ping_strike > 0.001f, 0)) {
            dry_l += ping_strike * (flt_l * 0.70f);
            dry_r += ping_strike * (flt_r * 0.70f);
        }

        float tap_l = 0.0f, tap_r = 0.0f;
        float tap_sum_l = tap_mix_l + tap_out_l;
        float tap_sum_r = tap_mix_r + tap_out_r;
        if (__builtin_expect(std::abs(tap_sum_l) > 0.001f, 0)) {
            tap_l = FastSoftSat(tap_sum_l) * 0.70f;
        }
        if (__builtin_expect(std::abs(tap_sum_r) > 0.001f, 0)) {
            tap_r = FastSoftSat(tap_sum_r) * 0.70f;
        }

        // Constant-headroom soft summing: prevents dry+wet sum at noon from choking the master limiter
        float raw_mix_l = (_smooth_dry * dry_l + _smooth_wet * wet_l + tap_l) * shunt_both;
        float raw_mix_r = (_smooth_dry * dry_r + _smooth_wet * wet_r + tap_r) * shunt_both;
        float mixed_l = raw_mix_l;
        float mixed_r = raw_mix_r;
        float abs_mix_l = std::abs(raw_mix_l);
        if (__builtin_expect(abs_mix_l > 0.82f, 0)) {
            float sat = 0.82f + 0.38f * FastSoftSat((abs_mix_l - 0.82f) * 2.63f);
            mixed_l = (raw_mix_l >= 0.0f) ? sat : -sat;
        }
        float abs_mix_r = std::abs(raw_mix_r);
        if (__builtin_expect(abs_mix_r > 0.82f, 0)) {
            float sat = 0.82f + 0.38f * FastSoftSat((abs_mix_r - 0.82f) * 2.63f);
            mixed_r = (raw_mix_r >= 0.0f) ? sat : -sat;
        }

        // FLT <-> MIX bend: master output filter isolator
        if (mods.filter_isolate > 0.015f) {
            float f_amt = std::min(1.0f, mods.filter_isolate);
            float iso_l = flt_l * shunt_both * 0.85f;
            float iso_r = flt_r * shunt_both * 0.85f;
            mixed_l += f_amt * (iso_l - mixed_l);
            mixed_r += f_amt * (iso_r - mixed_r);
        }

        // VCC <-> MIX: hot output stage
        if (_drive_active) {
            mixed_l = FastSoftSat(mixed_l * _drive_k) * _drive_makeup;
            mixed_r = FastSoftSat(mixed_r * _drive_k) * _drive_makeup;
        }

        // MIX <-> RST bend: delicate brownout dropouts & micro-gaps
        if (mods.dropout > 0.01f) {
            _dropout_phase += 0.0006f + mods.dropout * 0.0022f;
            if (_dropout_phase >= 1.0f) _dropout_phase -= 1.0f;
            float hole = (_dropout_phase > 0.88f) ? (1.0f - mods.dropout) : 1.0f;
            mixed_l *= hole;
            mixed_r *= hole;
        }

        // OSC <-> MIX bend: optical photocell tremolo & audio-rate AM sidebands
        if (__builtin_expect(std::abs(mods.tremolo) > 0.0005f || _smooth_opto_l < 0.999f || _smooth_opto_r < 0.999f, 0)) {
            float abs_t = std::min(1.0f, std::abs(mods.tremolo));
            float target_opto_l = 1.0f - 0.425f * (abs_t + mods.tremolo);
            float target_opto_r = 1.0f - 0.425f * (abs_t - mods.tremolo * 0.6f);
            _smooth_opto_l += 0.06f * (target_opto_l - _smooth_opto_l);
            _smooth_opto_r += 0.06f * (target_opto_r - _smooth_opto_r);
            mixed_l *= _smooth_opto_l;
            mixed_r *= _smooth_opto_r;
        }

        out_l = mixed_l;
        out_r = mixed_r;
    }

private:
    float _wet_g{0.0f};
    float _dry_g{1.0f};
    float _dry_machine_morph{0.0f};
    float _smooth_wet{0.0f};
    float _smooth_dry{1.0f};
    float _smooth_machine_morph{0.0f};
    float _inc_wet{0.0f};
    float _inc_dry{0.0f};
    float _inc_morph{0.0f};
    bool  _drive_active{false};
    float _drive_k{1.0f};
    float _drive_makeup{1.0f};
    float _dropout_phase{0.0f};
    float _smooth_opto_l{1.0f};
    float _smooth_opto_r{1.0f};
};

} // namespace bends
