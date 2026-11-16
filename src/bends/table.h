#pragma once

#include "bends/bend.h"

namespace bends {

constexpr Bend kMasterBendTable[] = {
    // -------------------------------------------------------------------------
    // 1. Adjacent Seam Bends
    // -------------------------------------------------------------------------
    // MIX - FLT: Route filter directly to mix
    BEND_PAIR(Mix, Flt, Slot::MixFilterIsol,  Sig::One,      1.00f, When::Always, Flag::NoBleed),

    // CLK - RST: Reverse playback
    BEND_PAIR(Clk, Rst, Slot::DlyReverse,     Sig::One,      1.00f, When::Always, Flag::NoBleed),

    // CLK - DLY: Quantized beat chop / loop slice
    BEND_PAIR(Clk, Dly, Slot::DlyChop,        Sig::One,      1.00f, When::Always, Flag::NoBleed),

    // RST - FLT: Filter impulse ping on touch
    BEND_PAIR(Rst, Flt, Slot::FltPing,        Sig::One,      1.00f, When::Always, Flag::Onset),

    // RST - GND: Clear delay buffer
    BEND_PAIR(Rst, Gnd, Slot::None,           Sig::One,      1.00f, When::Always, Flag::Panic),

    // RST - DLY: Restart delay read position to write head
    BEND_PAIR(Rst, Dly, Slot::DlyRestart,     Sig::One,      1.00f, When::Always, Flag::None),

    // OSC - FLT: Audio-rate filter cutoff FM
    BEND_PAIR(Osc, Flt, Slot::FltCutoff,      Sig::Osc,      2.40f, When::Saw,    Flag::NoBleed),
    BEND_PAIR(Osc, Flt, Slot::FltCutoff,      Sig::Osc,      2.20f, When::Sin,    Flag::NoBleed),
    BEND_PAIR(Osc, Flt, Slot::FltCutoff,      Sig::Osc,      2.50f, When::Rnd,    Flag::NoBleed),

    // OSC - AMP: Inject oscillator into preamp input
    BEND_PAIR(Osc, Amp, Slot::AmpOscVoice,    Sig::One,      1.00f, When::Always, Flag::NoBleed),

    // FLT - GND: Pull filter cutoff and resonance down
    BEND_PAIR(Flt, Gnd, Slot::FltCutoff,      Sig::One,     -6.00f, When::Always, Flag::NoBleed),
    BEND_PAIR(Flt, Gnd, Slot::FltRes,         Sig::One,     -1.00f, When::Always, Flag::NoBleed),

    // FLT - AMP: Filter output feedback into preamp
    BEND_PAIR(Flt, Amp, Slot::AmpFltLoop,     Sig::One,      0.75f, When::Always, Flag::NoBleed),
    BEND_PAIR(Flt, Amp, Slot::FltRes,         Sig::One,      0.25f, When::Always, Flag::NoBleed),

    // DLY - GND: Buffer address skip / bit shuffle
    BEND_PAIR(Dly, Gnd, Slot::DlySkip,        Sig::One,      1.00f, When::Always, Flag::NoBleed),

    // DLY - VCC: Delay write scatter / address shift
    BEND_PAIR(Dly, Vcc, Slot::DlyScatter,     Sig::One,      1.00f, When::Always, Flag::None),

    // DLY - FBK: Delay feedback glitch and crossfeed modulation
    BEND_PAIR(Dly, Fbk, Slot::DlyFbkGlitch,   Sig::One,      1.00f, When::Always, Flag::NoBleed),

    // VCC - FBK: High feedback boost
    BEND_PAIR(Vcc, Fbk, Slot::DlyRunaway,     Sig::One,      1.00f, When::Always, Flag::None),

    // AMP - FBK: Delay feedback return into preamp
    BEND_PAIR(Amp, Fbk, Slot::AmpFbkLoop,     Sig::One,      1.00f, When::Always, Flag::NoBleed),

    // MIX - RST: Mix output dropouts
    BEND_PAIR(Mix, Rst, Slot::MixDropout,     Sig::One,      1.00f, When::Always, Flag::None),

    // AMP - GND: Preamp hard mute / physical ground shunt
    BEND_PAIR(Amp, Gnd, Slot::PreampMute,     Sig::One,      1.00f, When::Always, Flag::None),

    // FBK - GND: Feedback loop drain / choke
    BEND_PAIR(Fbk, Gnd, Slot::DlyFeedbackDrain, Sig::One,    1.00f, When::Always, Flag::NoBleed),

    // -------------------------------------------------------------------------
    // 2. Two-Point Bends
    // -------------------------------------------------------------------------
    // Clock / Speed
    BEND_PAIR(Clk, Gnd, Slot::DlyBrake,       Sig::One,      1.00f, When::Always, Flag::NoBleed),
    BEND_PAIR(Vcc, Clk, Slot::ClkOverclock,   Sig::One,      1.00f, When::Always, Flag::None),
    // OSC <-> CLK: Pitch vibrato, analog tape chorus, and clock FM
    BEND_PAIR(Osc, Clk, Slot::DlyClockFm,     Sig::One,      0.85f, When::Always, Flag::NoBleed),
    BEND_PAIR(Osc, Clk, Slot::OscRunglerLock, Sig::One,      1.00f, When::Rnd,    Flag::None),
    BEND_PAIR(Clk, Amp, Slot::AmpTremolo,     Sig::ClkPulse, 1.00f, When::Always, Flag::None),
    BEND_PAIR(Clk, Flt, Slot::FltClockStep,   Sig::ClkPulse, 0.30f, When::Always, Flag::None),
    BEND_PAIR(Clk, Fbk, Slot::DlyClockBurst,  Sig::ClkPulse, 0.85f, When::Always, Flag::None),

    // Delay Time / Addressing
    // OSC - DLY: Delay time modulation from oscillator
    BEND_PAIR(Osc, Dly, Slot::DlyHeadMod,     Sig::One,      1.75f, When::Always, Flag::NoBleed),
    // AMP - DLY: Preamp output modulates delay phase and drive
    BEND_PAIR(Amp, Dly, Slot::DlyPhaseMod,    Sig::PreampOut, 3.20f, When::Always, Flag::NoBleed),
    BEND_PAIR(Amp, Dly, Slot::PreampDrive,    Sig::One,       1.25f, When::Always, Flag::NoBleed),
    BEND_PAIR(Amp, Dly, Slot::DlyRunaway,     Sig::One,       0.85f, When::Always, Flag::NoBleed),
    BEND_PAIR(Rst, Fbk, Slot::DlyBufferJump,  Sig::One,      1.00f, When::Always, Flag::None),
    // DLY - MIX: Memory bus short / data hold
    BEND_PAIR(Dly, Mix, Slot::DlyDataShort,   Sig::One,      1.00f, When::Always, Flag::NoBleed),

    // VCC Overdrive & Wavefolding
    BEND_PAIR(Vcc, Amp, Slot::FltWavefold,    Sig::One,      1.00f, When::Always,  Flag::NoBleed),
    BEND_PAIR(Vcc, Amp, Slot::PreampDrive,    Sig::One,      0.60f, When::Always,  Flag::None),
    BEND_PAIR(Vcc, Amp, Slot::PreampBias,     Sig::One,      0.35f, When::Always,  Flag::None),
    BEND_PAIR(Vcc, Flt, Slot::FltCutoff,      Sig::One,      4.00f, When::Always,  Flag::NoBleed),
    BEND_PAIR(Vcc, Flt, Slot::FltRes,         Sig::One,      0.20f, When::Always,  Flag::None),
    BEND_PAIR(Osc, Vcc, Slot::OscShapeWarp,   Sig::One,      1.00f, When::Always,  Flag::None),

    // Glitches & Resets
    BEND_PAIR(Rst, Vcc, Slot::MasterCrash,    Sig::One,      1.00f, When::Always, Flag::NoBleed),
    BEND_PAIR(Osc, Gnd, Slot::OscSubOctave,   Sig::One,      1.00f, When::Always, Flag::NoBleed),
    BEND_PAIR(Osc, Rst, Slot::OscSync,        Sig::One,      1.00f, When::Always, Flag::None),
    BEND_PAIR(Vcc, Gnd, Slot::Microsounds,    Sig::One,      1.00f, When::Always, Flag::NoBleed),

    // Feedback Loop & Output Routing
    BEND_PAIR(Flt, Fbk, Slot::DlyFilterLoop,  Sig::One,      1.00f, When::Always, Flag::NoBleed),
    BEND_PAIR(Osc, Fbk, Slot::DlyRecircFm,    Sig::Osc,      0.95f, When::Always, Flag::None),
    // FBK - MIX: Infinite Loop Bed (locks feedback loop at exact unity gain without runaway distortion)
    BEND_PAIR(Fbk, Mix, Slot::DlyInfiniteHold, Sig::One,     1.00f, When::Always, Flag::NoBleed),
    BEND_PAIR(Mix, Gnd, Slot::MixWetOnly,     Sig::One,      1.00f, When::Always, Flag::NoBleed),
    BEND_PAIR(Vcc, Mix, Slot::MixDrive,       Sig::One,      1.00f, When::Always, Flag::None),
    BEND_PAIR(Amp, Mix, Slot::MixAmpDirect,   Sig::One,      1.00f, When::Always, Flag::NoBleed),
    BEND_PAIR(Osc, Mix, Slot::MixTremolo,     Sig::Osc,      1.00f, When::Always, Flag::NoBleed),
    BEND_PAIR(Out, Gnd, Slot::MasterMute,     Sig::One,      1.00f, When::Always, Flag::None),

    // Audio Probes & Dynamic Modulation
    BEND_PAIR(In,  Out, Slot::MicFeedback,    Sig::One,      1.00f, When::Always, Flag::NoBleed),

    // IN <-> CLK: Audio-rate delay clock FM (FM clock flutter & metallic sidebands)
    // ONLY active when actively bridging Pad 10 (IN) and Pad 02 (CLK)
    BEND_PAIR(In,  Clk, Slot::DlyClockFm,     Sig::InAudio,   0.60f, When::Always,  Flag::NoBleed),

    // OUT <-> CLK: Parasitic Output-to-Clock Pull / Elastic Clock Sag
    BEND_PAIR(Out, Clk, Slot::DlyClockFm,     Sig::OutEnv,   -2.20f, When::Always,  Flag::NoBleed),
    BEND_PAIR(Out, Clk, Slot::DlyClockFm,     Sig::OutAudio, -0.65f, When::Always,  Flag::NoBleed),

    // OSC <-> IN: Audio-rate tracking & cross FM into oscillator + voice blend
    BEND_PAIR(Osc, In,  Slot::OscFm,          Sig::InAudio,  2.50f, When::Always, Flag::None),
    BEND_PAIR(Osc, In,  Slot::OscFm,          Sig::InEnv,    2.00f, When::Always, Flag::None),
    BEND_PAIR(Osc, In,  Slot::AmpOscVoice,    Sig::One,      0.75f, When::Always, Flag::None),

    // OSC <-> OUT: Output audio chaos feedback loop into oscillator (direct audio tap handled in SignalGraph)
    BEND_PAIR(Osc, Out, Slot::OscFm,          Sig::OutAudio, 0.75f, When::Always, Flag::NoBleed),

    // Filter probing
    BEND_PAIR(In,  Flt, Slot::FltCutoff,      Sig::InAudio,  1.50f, When::Always, Flag::None),
    BEND_PAIR(In,  Flt, Slot::FltCutoff,      Sig::InEnv,    2.50f, When::Always, Flag::None),
    BEND_PAIR(In,  Flt, Slot::FltPing,        Sig::One,      1.00f, When::Always, Flag::Onset),
    BEND_PAIR(Out, Flt, Slot::FltCutoff,      Sig::OutAudio, 1.80f, When::Always, Flag::None),

    // DLY <-> FLT: Delay repeats envelope & audio modulate filter cutoff & resonance
    BEND_PAIR(Dly, Flt, Slot::FltCutoff,      Sig::DlyEnv,   3.20f, When::Always, Flag::NoBleed),
    BEND_PAIR(Dly, Flt, Slot::FltCutoff,      Sig::DlyOut,   1.20f, When::Always, Flag::NoBleed),
    BEND_PAIR(Dly, Flt, Slot::FltRes,         Sig::One,      0.40f, When::Always, Flag::NoBleed),

    // IN <-> AMP: High-gain input drive, dynamic touch sensitivity & preamp overdrive
    BEND_PAIR(In,  Amp, Slot::PreampDrive,    Sig::One,      1.50f, When::Always, Flag::NoBleed),
    BEND_PAIR(In,  Amp, Slot::PreampDrive,    Sig::InEnv,    1.80f, When::Always, Flag::NoBleed),
    BEND_PAIR(In,  Amp, Slot::MicBoost,       Sig::One,      1.25f, When::Always, Flag::NoBleed),

    BEND_PAIR(In,  Fbk, Slot::MicToFbk,       Sig::One,      1.00f, When::Always, Flag::NoBleed),

    // OUT <-> AMP: Recirculating output-to-preamp feedback loop, dynamic pumping overdrive & scream
    BEND_PAIR(Out, Amp, Slot::PreampDrive,    Sig::One,      1.65f, When::Always, Flag::NoBleed),
    BEND_PAIR(Out, Amp, Slot::PreampDrive,    Sig::OutEnv,   2.20f, When::Always, Flag::NoBleed),

    // OUT <-> FBK: Acoustic Resonant Larsen Howl & Feedback Squeal
    BEND_PAIR(Out, Fbk, Slot::DlyLarsenHowl,  Sig::One,      1.00f, When::Always, Flag::NoBleed),

    // -------------------------------------------------------------------------
    // 3. Solo Pad Touch
    // -------------------------------------------------------------------------
    BEND_SOLO(Gnd,      Slot::HumVoice,       Sig::One,      0.045f, When::Always, Flag::None),
    BEND_SOLO(Flt,      Slot::FltRes,         Sig::One,      0.95f,  When::Always, Flag::None),
};

constexpr size_t kNumMasterBends = sizeof(kMasterBendTable) / sizeof(kMasterBendTable[0]);

} // namespace bends
