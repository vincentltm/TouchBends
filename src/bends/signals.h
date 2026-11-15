#pragma once

#include <cstdint>

namespace bends {

enum class Sig : uint8_t {
    One = 0,     // Constant 1.0 (static control-rate amount)
    Osc,         // Continuous oscillator waveform (-1.0 .. +1.0)
    OscRamp,     // Unipolar saw ramp (0.0 .. 1.0)
    OscReset,    // Single-sample flyback impulse (0.0 .. 1.0)
    OscSub,      // Sub-octave square wave (-1.0 .. +1.0)
    ClkPulse,    // Clock divider pulse train
    ClkBeat,     // Single-sample beat downbeat trigger
    InAudio,     // External input audio (mono sum)
    InEnv,       // Input envelope follower
    OutAudio,    // Output audio (mono sum)
    OutEnv,      // Output envelope follower
    PreampOut,   // Preamp output audio
    FltOut,      // SVF lowpass output audio
    DlyOut,      // Delay wet output audio
    DlyEnv,      // Delay wet envelope follower
    Hum,         // Mains hum
    ContactMic,  // Tactile contact mic audio
    MasterOut,   // Final master audio
    Count
};

} // namespace bends
