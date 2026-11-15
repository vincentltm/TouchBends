#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

namespace bends {

enum NodeId : uint8_t {
    NodeMix  = 0,  // P00: Master wet/dry - Upper-Left Ridge
    NodeRst  = 1,  // P01: Reset pull-up - Top Center Axis
    NodeClk  = 2,  // P02: Master clock (timing) - Upper-Right Ridge
    NodeOsc  = 3,  // P03: Probe oscillator - Far-Left Wing
    NodeFlt  = 4,  // P04: Low-pass filter - Mid-Center Left
    NodeGnd  = 5,  // P05: Ground plane (0V) - Bottom Center Axis
    NodeDly  = 6,  // P06: Delay read-head - Mid-Center Right
    NodeVcc  = 7,  // P07: Power rail - Far-Right Wing
    NodeAmp  = 8,  // P08: Preamp / Drive - Bottom-Left Corner
    NodeFbk  = 9,  // P09: Feedback loop - Bottom-Right Corner
    NodeIn   = 10, // P10: Ext In probe / Chassis contact mic Left
    NodeOut  = 11, // P11: Ext Out probe / Chassis contact mic Right
    NodeBody = 12, // Virtual hidden AC node (Hand/Body/Hum)
    kTotalNodes = 13
};

constexpr size_t kPhysicalNodes = 12; // P00 to P11
constexpr size_t kSolverNodes   = 13; // Includes virtual Body node

struct NodeSpec {
    float rest_dc;        // Normalized DC rest voltage (0.0 to 1.0)
    float z_dc;           // DC source impedance (conductance y = 1/z)
};

struct NodeTable {
    static constexpr std::array<NodeSpec, kSolverNodes> Specs = {{
        // Rest,   Z_dc
        { 0.50f,  1.20f }, // P00: MIX - Summing junction (moderate Z)
        { 1.00f,  2.50f }, // P01: RST - Weak pull-up logic line (high Z)
        { 0.50f,  1.50f }, // P02: CLK - Timing control core (high Z, easily pulled)
        { 0.50f,  0.35f }, // P03: OSC - Active oscillator driver (low Z source)
        { 0.50f,  1.60f }, // P04: FLT - Filter cutoff CV & core (high Z, easily pulled)
        { 0.00f,  0.02f }, // P05: GND - Ground plane 0V sink (ultra-low Z)
        { 0.50f,  1.80f }, // P06: DLY - Delay time CV / read head (high Z, easily pulled)
        { 1.00f,  0.05f }, // P07: VCC - Power rail source (very low Z)
        { 0.50f,  0.35f }, // P08: AMP - Preamp driver output (low Z source)
        { 0.50f,  1.40f }, // P09: FBK - Feedback loop return (high Z)
        { 0.50f,  0.30f }, // P10: IN  - Audio input probe / tactile source (low Z driver)
        { 0.50f,  0.30f }, // P11: OUT - Audio output probe test point (low Z driver)
        { 0.50f,  8.00f }  // P12: BODY- Stray skin/hand AC coupling (very high Z)
    }};

    static constexpr std::array<float, kSolverNodes> SourceConductance = {{
        1.0f / 1.20f,
        1.0f / 2.50f,
        1.0f / 1.50f,
        1.0f / 0.35f,
        1.0f / 1.60f,
        1.0f / 0.02f,
        1.0f / 1.80f,
        1.0f / 0.05f,
        1.0f / 0.35f,
        1.0f / 1.40f,
        1.0f / 0.30f,
        1.0f / 0.30f,
        1.0f / 8.00f
    }};

    struct Point2D {
        float x;
        float y;
    };

    // Normalized PCB 2D coordinates [0.0, 1.0] across Synthux Touch 2 surface
    static constexpr std::array<Point2D, kPhysicalNodes> PadPositions = {{
        { 0.28f, 0.15f }, // P00: MIX - Upper-Left Ridge
        { 0.50f, 0.10f }, // P01: RST - Top Center Axis
        { 0.72f, 0.15f }, // P02: CLK - Upper-Right Ridge
        { 0.08f, 0.42f }, // P03: OSC - Far-Left Wing
        { 0.34f, 0.42f }, // P04: FLT - Mid-Center Left
        { 0.50f, 0.65f }, // P05: GND - Bottom Center Axis
        { 0.66f, 0.42f }, // P06: DLY - Mid-Center Right
        { 0.92f, 0.42f }, // P07: VCC - Far-Right Wing
        { 0.22f, 0.82f }, // P08: AMP - Bottom-Left Corner
        { 0.78f, 0.82f }, // P09: FBK - Bottom-Right Corner
        { 0.06f, 0.88f }, // P10: IN  - Contact mic Left
        { 0.94f, 0.88f }, // P11: OUT - Contact mic Right
    }};

    // Physical inter-pad coupling matrix:
    // - Adjacent pads share a direct copper border (0.5mm gap): W = 1.000
    // - Non-adjacent pads scale inversely with 2D Euclidean distance across human body/chassis:
    //   W(a, b) = 1.0 / (1.0 + 1.25 * distance(a, b))
    static constexpr float PhysicalCoupling[kPhysicalNodes][kPhysicalNodes] = {
        { 0.000f, 1.000f, 0.645f, 0.704f, 1.000f, 0.594f, 0.632f, 0.535f, 0.543f, 0.489f, 0.512f, 0.448f }, // P00 MIX
        { 1.000f, 0.000f, 1.000f, 0.602f, 1.000f, 1.000f, 1.000f, 0.602f, 0.509f, 0.509f, 0.472f, 0.472f }, // P01 RST
        { 0.645f, 1.000f, 0.000f, 0.535f, 0.632f, 0.594f, 1.000f, 0.704f, 0.489f, 0.543f, 0.448f, 0.512f }, // P02 CLK
        { 0.704f, 0.602f, 0.535f, 0.000f, 1.000f, 0.626f, 0.580f, 0.488f, 1.000f, 0.498f, 0.635f, 0.451f }, // P03 OSC
        { 1.000f, 1.000f, 0.632f, 1.000f, 0.000f, 1.000f, 0.714f, 0.580f, 1.000f, 0.574f, 0.598f, 0.514f }, // P04 FLT
        { 0.594f, 1.000f, 0.594f, 0.626f, 1.000f, 0.000f, 1.000f, 0.626f, 1.000f, 1.000f, 0.617f, 0.617f }, // P05 GND
        { 0.632f, 1.000f, 1.000f, 0.580f, 0.714f, 1.000f, 0.000f, 1.000f, 0.574f, 1.000f, 0.514f, 0.598f }, // P06 DLY
        { 0.535f, 0.602f, 0.704f, 0.488f, 0.580f, 0.626f, 1.000f, 0.000f, 0.498f, 1.000f, 0.451f, 0.635f }, // P07 VCC
        { 0.543f, 0.509f, 0.489f, 1.000f, 1.000f, 1.000f, 0.574f, 0.498f, 0.000f, 1.000f, 0.824f, 0.525f }, // P08 AMP
        { 0.489f, 0.509f, 0.543f, 0.498f, 0.574f, 1.000f, 1.000f, 1.000f, 1.000f, 0.000f, 0.525f, 0.824f }, // P09 FBK
        { 0.512f, 0.472f, 0.448f, 0.635f, 0.598f, 0.617f, 0.514f, 0.451f, 0.824f, 0.525f, 0.000f, 0.476f }, // P10 IN
        { 0.448f, 0.472f, 0.512f, 0.451f, 0.514f, 0.617f, 0.598f, 0.635f, 0.525f, 0.824f, 0.476f, 0.000f }, // P11 OUT
    };

    // Physical pad adjacency on Synthux Touch 2 PCB (18 direct copper borders: coupling == 1.0)
    static constexpr bool IsAdjacent(size_t a, size_t b) {
        if (a >= kPhysicalNodes || b >= kPhysicalNodes) return false;
        return PhysicalCoupling[a][b] >= 0.999f;
    }

    static constexpr float PhysicalWeight(size_t a, size_t b) {
        if (a == b) return 0.0f;
        if (a >= kPhysicalNodes || b >= kPhysicalNodes) return 1.0f;
        return PhysicalCoupling[a][b];
    }
};

} // namespace bends
