#pragma once

#include "dsp/stereo.h"
#include "dsp/filters.h"
#include "dsp/fast_math.h"
#include <algorithm>
#include <cmath>

namespace bends {

class InputStage {
public:
    void Init(float sample_rate) {
        _sample_rate = (sample_rate > 1000.0f) ? sample_rate : 48000.0f;
        Reset();
        _boot_fade_samples = 0;
        _boot_fade_total = static_cast<uint32_t>(_sample_rate * 0.85f);
    }

    void Reset() {
        _dc_block_l.Reset();
        _dc_block_r.Reset();
        _rf_l = 0.0f;
        _rf_r = 0.0f;
    }

    inline void DetectMono(const float* /*in_l*/, const float* /*in_r*/, size_t /*size*/) {
        // Disabled: pass inputs directly through without threshold cutting or channel flipping
    }

    inline void PrepareBlock(float /*s30_amp*/) {
    }

    __attribute__((always_inline)) inline Stereo ProcessSample(float inl_raw, float inr_raw, float retention_in, float& out_boot_gain) {
        // Startup fade-in
        float boot_gain = 1.0f;
        if (__builtin_expect(_boot_fade_samples < _boot_fade_total, 0)) {
            float p = static_cast<float>(_boot_fade_samples) / static_cast<float>(_boot_fade_total);
            boot_gain = 0.5f * (1.0f - FastCos(p * kPi));
            _boot_fade_samples++;
        }
        out_boot_gain = boot_gain;

        float inl = inl_raw * boot_gain;
        float inr = inr_raw * boot_gain;

        // DC blocking (~3 Hz highpass)
        float in_l = _dc_block_l.Process(inl);
        float in_r = _dc_block_r.Process(inr);

        // Ultrasonic RF choke filter (~18.5 kHz)
        constexpr float kRfAlpha = 0.75f;
        _rf_l += kRfAlpha * (in_l - _rf_l);
        _rf_r += kRfAlpha * (in_r - _rf_r);

        return { _rf_l * retention_in, _rf_r * retention_in };
    }

    bool IsMono() const { return false; }

private:
    float     _sample_rate{48000.0f};
    DcBlocker _dc_block_l;
    DcBlocker _dc_block_r;

    float     _rf_l{0.0f};
    float     _rf_r{0.0f};

    uint32_t  _boot_fade_samples{0};
    uint32_t  _boot_fade_total{40800};
};

} // namespace bends
