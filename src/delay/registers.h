#pragma once

#include <cstdint>
#include <algorithm>
#include "tuning/tuning.h"

namespace bends {

struct ClockRegisters {
    float rate{1.0f};          // Variclock speed multiplier (1.0 = 48 kHz normal)
    float fm{0.0f};            // Audio-rate clock FM (-1.0 to 1.0)
    bool  run{true};           // Clock run gate (false = hard clock freeze)
};

struct WriteRegisters {
    bool  write_enable{true};  // Recording into memory (false when frozen)
    float write_gain{1.0f};    // Direct write level
};

struct SpeedRegisters {
    float speed{1.0f};         // Speed multiplier (+1.0 normal, -1.0 reverse, +2.0 octave, 0.0 stopped)
    float slew{0.08f};         // Slew rate toward target speed
};

struct TimeRegisters {
    float offset{tuning::kMinDelaySamples}; // Delay length in samples (from S34 knob)
    float mod_offset{0.0f};                 // Audio-rate head displacement (OSC/audio PM)
    float skew{0.0f};                       // Stereo head offset in samples (from DLY-FBK bend)
    float stereo_split{0.0f};               // Polyrhythmic stereo read head decoupling (0.0 to 1.0)
    bool  has_loop{false};                  // Loop window active
    bool  is_chop{false};                   // True when CLK-DLY touch chop stutter is active
    uint32_t loop_anchor{0};                // Loop window start index
    float loop_len{4800.0f};                // Loop window length in samples

    bool  bode_active{false};               // True when VCC-DLY Bode frequency shift is active
    float bode_shift{0.0f};                 // Frequency shift amount in Hz (-450 to +450 Hz)

    bool  jump_req{false};                  // Request jump crossfade
    float jump_target{tuning::kMinDelaySamples};
    uint16_t jump_fade{tuning::kXfadeSamples};
    float jump_loop_len{0.0f};

    void RequestJump(float target, uint16_t fade_samples = tuning::kXfadeSamples, float loop_len = 0.0f) {
        jump_req = true;
        jump_target = target;
        jump_fade = fade_samples;
        jump_loop_len = loop_len;
    }
};

struct DataRegisters {
    float    strobe_l{1.0f};   // Read-enable gain Left (1.0 = normal, 0.0 = muted/dropout)
    float    strobe_r{1.0f};   // Read-enable gain Right
    uint32_t stuck_mask{0};    // Stuck bit mask (data line short)
    uint8_t  resolution{16};   // Bit depth (16 = normal, down to 8 = lofi)
    float    filter_bypass{0.0f}; // 0.0 = reconstruction filter active, 1.0 = reconstruction filter bypassed
};

struct MemoryRegisters {
    float rot_depth{0.0f};     // Memory bit rot rate under low voltage
    bool  clear_req{false};    // Clear delay memory
};

struct FeedbackRegisters {
    float gain{0.0f};          // Feedback level (0.0 to 1.15)
    float damp{0.98f};         // High-frequency damping alpha (from S31 Tone <= 12:00)
    float tone_hp{0.0f};       // Low-frequency damping alpha (from S31 Tone > 12:00)
    float crossfeed{0.0f};     // Ping-pong stereo crossfeed (0.0 to 1.0)
    float recirc_fm{0.0f};     // Recirculating FM depth
    float erode{0.0f};         // High-pass loop filter (FLT-FBK)
    float runaway{0.0f};       // Feedback gain boost
    float drain{0.0f};         // Shunt feedback to ground (FBK-GND)
    bool  infinite_hold{false};// Exact unity gain infinite loop bed (FBK-MIX)
    float larsen_howl{0.0f};   // Acoustic resonant Larsen feedback (OUT-FBK)
};

struct DelayRegisters {
    ClockRegisters    clock;
    WriteRegisters    write;
    SpeedRegisters    speed;
    TimeRegisters     time;
    DataRegisters     data;
    MemoryRegisters   memory;
    FeedbackRegisters feedback;

    void Reset() {
        clock = ClockRegisters{};
        write = WriteRegisters{};
        speed = SpeedRegisters{};
        time = TimeRegisters{};
        data = DataRegisters{};
        memory = MemoryRegisters{};
        feedback = FeedbackRegisters{};
    }
};

} // namespace bends
