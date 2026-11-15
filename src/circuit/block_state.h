#pragma once

#include "tuning/tuning.h"
#include "bends/amount_bus.h"
#include "bends/mod_matrix.h"
#include "delay/registers.h"
#include "circuit/nodes.h"
#include "circuit/topology.h"

namespace bends {

struct CrossPair {
    uint8_t a;
    uint8_t b;
    float   ga;
    float   gb;
};

struct BlockState {
    float   vcc{1.0f};
    float   starve{0.0f};

    float   retention[kSolverNodes]{1.0f};
    float   pull[kSolverNodes]{0.0f};

    uint8_t osc_mode{1};
    bool    is_frozen{false};
    bool    is_latched{false};

    // Multi-touch topology state
    MultiTouchTopology topology;

    // Precomputed sub-module parameters
    float   s30_amp{0.5f};
    float   s31_flt{0.5f};
    float   s32_mix{0.5f};
    float   s33_clk{0.5f};
    float   s34_dly{0.5f};
    float   s35_fbk{0.5f};
    float   s36_osc{0.25f};

    // Gestures & taps
    float   pressures[kPhysicalNodes]{0.0f};
    float   pressure_in{0.0f};
    float   rub_speed_in{0.0f};
    int8_t  alone_node{-1};
    float   alone_pressure{0.0f};

    // Active cross-injection pairs
    CrossPair cross_pairs[4];
    uint8_t   num_cross_pairs{0};

    // Specific direct bridge conductances
    float     g_osc_flt{0.0f};
    float     g_osc_amp{0.0f};
    float     g_flt_amp{0.0f};
    float     g_clk_amp{0.0f};
    float     g_clk_flt{0.0f};
    float     g_osc_clk{0.0f};
    float     g_in_clk{0.0f};
    float     g_in_dly{0.0f};
    float     g_in_amp{0.0f};
    float     g_out_amp{0.0f};
    float     g_osc_out{0.0f};
    float     g_amp_dly{0.0f};

    // Contextual touch routing flags for MIDI pitch
    bool      touch_osc{false};
    bool      touch_flt{false};
    bool      touch_clk{false};
    bool      touch_dly_fbk{false};
    bool      touch_out_gnd{false};

    // Delay registers for delay engine
    DelayRegisters delay_regs;

    // Physical modeling state
    float motorboat_pulse{0.0f};
    float dynamic_motion{0.0f};
    float digital_motion_ratio{0.0f};
};

} // namespace bends
