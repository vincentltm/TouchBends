#include "knobs.h"

using namespace synthux;
using namespace daisy;

void Knobs::Init(DaisySeed& hw) {
    // S30..S35 are pots on A0..A5; S36, S37 are faders on A6, A7
    Pin pins[8] = {
        seed::A0, seed::A1, seed::A2, seed::A3,
        seed::A4, seed::A5, seed::A6, seed::A7
    };

    AdcChannelConfig cfg[8];
    for (int i = 0; i < 8; i++) {
        cfg[i].InitSingle(pins[i]);
    }
    hw.adc.Init(cfg, 8);

    // Polled in main loop at ~250 Hz (I2C + 1ms delay)
    constexpr float kKnobPollRate = 250.0f;
    for (int i = 0; i < 8; i++) {
        // Use 20ms slew filter for responsive feel without zipper noise
        _knobs[i].Init(hw.adc.GetPtr(i), kKnobPollRate, false, false, 0.020f);
        float v = _knobs[i].Process();
        _filtered[i] = v;
        _values[i] = v;
    }
}

void Knobs::Process() {
    for (int i = 0; i < 8; i++) {
        float raw = _knobs[i].Process();
        // Clean floor and ceiling snap
        if (raw < 0.005f) {
            raw = 0.0f;
        } else if (raw > 0.995f) {
            raw = 1.0f;
        }
        float diff = raw - _values[i];
        if (raw == 0.0f || raw == 1.0f) {
            _values[i] = raw;
        } else if (std::abs(diff) > 0.03f) {
            // Fast gesture: immediate responsive tracking
            _values[i] = raw;
        } else if (std::abs(diff) > 0.0008f) {
            // Smooth motion tracking: eliminates staircase quantization without ADC ripple
            _values[i] += 0.40f * diff;
        }
    }
}
