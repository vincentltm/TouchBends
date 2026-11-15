#pragma once

#include <cstdint>
#include <atomic>
#include <algorithm>

namespace bends {

struct ControlFrame {
    // 12 capacitive touch pad pressures and proximities
    float pressures[12]{0.0f};
    float proximities[12]{0.0f};

    // Knobs & Faders (PCB Silkscreen 1:1)
    float s30_amp{0.35f};  // S30 Pot: Preamp Drive (Pad 08 AMP)
    float s31_flt{0.65f};  // S31 Pot: Filter Cutoff (Pad 04 FLT)
    float s32_mix{0.50f};  // S32 Pot: Master Wet/Dry (Pad 00 MIX)
    float s33_clk{0.50f};  // S33 Pot: Master Delay Clock (Pad 02 CLK)
    float s34_dly{0.50f};  // S34 Pot: Delay Time (Pad 06 DLY)
    float s35_fbk{0.40f};  // S35 Pot: Delay Feedback (Pad 09 FBK)

    float s36_osc{0.25f};  // S36 Left Fader: Probe Oscillator Pitch (Pad 03 OSC)
    float s37_vcc{1.00f};  // S37 Right Fader: Power Rail / Starve (Pad 07 VCC)

    // Hardware Switches
    uint8_t osc_mode{1};   // 0 = Sin, 1 = Saw, 2 = Rnd
    bool    is_frozen{false};
    bool    is_latched{false};

    // Contact Mic Physical Gestures (P10 IN)
    float rub_speed_p10{0.0f};

    // Tap events: counter increments once per detected tap.
    uint16_t tap_count_p10{0};
    float    tap_vel_p10{0.0f};

    // MIDI Inbound Overrides
    float    midi_osc_freq{0.0f};  // 0 = inactive, >0 = MIDI Note target frequency in Hz
    float    midi_gate{0.0f};      // 0 = released, >0 = note on velocity
    float    midi_velocity{0.0f};  // 0.0 .. 1.0 strike velocity
    float    midi_aftertouch{0.0f};// 0.0 .. 1.0 continuous channel/poly aftertouch
    float    midi_mod_wheel{0.0f}; // 0.0 .. 1.0 CC 1 mod wheel
    float    midi_clk_rate{-1.0f}; // -1 = no tempo sync, >=0 = external tempo rate
    uint16_t midi_trigger_cnt{0};  // Increments on note strike
    bool     midi_legato{false};   // True when moving between notes without releasing

    uint32_t frame_index{0};
};

// Lock-free, wait-free SPSC triple buffer for Cortex-M.
// Guaranteed race-free index partition using atomic exchange.
class FrameBuffer {
public:
    FrameBuffer()
        : _write_idx(0),
          _read_idx(2),
          _ready_idx(1),
          _has_new(false) {}

    // Called only by main loop thread (producer)
    ControlFrame& BeginWrite() {
        return _frames[_write_idx];
    }

    void CommitWrite() {
        _write_idx = _ready_idx.exchange(_write_idx, std::memory_order_acq_rel);
        _has_new.store(true, std::memory_order_release);
    }

    // Called only by audio ISR (consumer)
    const ControlFrame& ReadLatest() {
        if (_has_new.exchange(false, std::memory_order_acq_rel)) {
            _read_idx = _ready_idx.exchange(_read_idx, std::memory_order_acq_rel);
        }
        return _frames[_read_idx];
    }

private:
    ControlFrame          _frames[3];
    uint32_t              _write_idx{0};
    uint32_t              _read_idx{2};
    std::atomic<uint32_t> _ready_idx{1};
    std::atomic<bool>     _has_new{false};
};

} // namespace bends
