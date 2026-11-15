#pragma once

#include "dsp/stereo.h"
#include "bends/signals.h"
#include "circuit/nodes.h"

namespace bends {

struct NodeBus {
    // Stereo audio history at physical circuit nodes (z^-1)
    Stereo clk{0.0f, 0.0f};
    Stereo osc{0.0f, 0.0f};
    Stereo flt{0.0f, 0.0f};
    Stereo gnd{0.0f, 0.0f};
    Stereo dly{0.0f, 0.0f};
    Stereo fbk{0.0f, 0.0f};
    Stereo amp{0.0f, 0.0f};
    Stereo mix{0.0f, 0.0f};
    Stereo in_sig{0.0f, 0.0f};
    Stereo out_sig{0.0f, 0.0f};

    // Scalar signals for modulation matrix
    float  osc_ramp{0.0f};
    float  osc_reset{0.0f};
    float  osc_sub{0.0f};
    float  clk_pulse{0.0f};
    float  clk_beat{0.0f};
    float  in_env{0.0f};
    float  out_env{0.0f};
    float  dly_env{0.0f};
    float  hum{0.0f};
    float  contact_mic{0.0f};

    // Array indexed by Sig enum
    float sig_array[static_cast<size_t>(Sig::Count)]{0.0f};

    __attribute__((always_inline)) inline void UpdateSigArray() {
        sig_array[static_cast<size_t>(Sig::One)]        = 1.0f;
        sig_array[static_cast<size_t>(Sig::Osc)]        = osc.l;
        sig_array[static_cast<size_t>(Sig::OscRamp)]    = osc_ramp;
        sig_array[static_cast<size_t>(Sig::OscReset)]   = osc_reset;
        sig_array[static_cast<size_t>(Sig::OscSub)]     = osc_sub;
        sig_array[static_cast<size_t>(Sig::ClkPulse)]   = clk_pulse;
        sig_array[static_cast<size_t>(Sig::ClkBeat)]    = clk_beat;
        sig_array[static_cast<size_t>(Sig::InAudio)]    = in_sig.l;
        sig_array[static_cast<size_t>(Sig::InEnv)]      = in_env;
        sig_array[static_cast<size_t>(Sig::OutAudio)]   = out_sig.l;
        sig_array[static_cast<size_t>(Sig::OutEnv)]     = out_env;
        sig_array[static_cast<size_t>(Sig::PreampOut)]  = amp.l;
        sig_array[static_cast<size_t>(Sig::FltOut)]     = flt.l;
        sig_array[static_cast<size_t>(Sig::DlyOut)]     = dly.l;
        sig_array[static_cast<size_t>(Sig::DlyEnv)]     = dly_env;
        sig_array[static_cast<size_t>(Sig::Hum)]        = hum;
        sig_array[static_cast<size_t>(Sig::ContactMic)] = contact_mic;
        sig_array[static_cast<size_t>(Sig::MasterOut)]  = out_sig.l;
    }
};

} // namespace bends
