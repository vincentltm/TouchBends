#pragma once

#include "circuit/nodes.h"
#include "circuit/contacts.h"
#include <cstdint>
#include <cmath>
#include <algorithm>

namespace bends {

// Bitmask constants for physical nodes (0..11)
constexpr uint16_t NodeMask(NodeId id) {
    return static_cast<uint16_t>(1u << id);
}

// Multi-finger functional co-occurrence masks
namespace TopologyMasks {
    constexpr uint16_t kPendulumSlice    = NodeMask(NodeClk) | NodeMask(NodeDly) | NodeMask(NodeRst);
    constexpr uint16_t kLissajousCloud   = NodeMask(NodeOsc) | NodeMask(NodeClk) | NodeMask(NodeDly);
    constexpr uint16_t kKarplusResonator = NodeMask(NodeFlt) | NodeMask(NodeDly) | NodeMask(NodeFbk);
    constexpr uint16_t kClockSiren       = NodeMask(NodeVcc) | NodeMask(NodeClk) | NodeMask(NodeFbk);
    constexpr uint16_t kIntermodLoop     = NodeMask(NodeAmp) | NodeMask(NodeFlt) | NodeMask(NodeFbk);
    constexpr uint16_t kVoltagePump      = NodeMask(NodeOsc) | NodeMask(NodeGnd) | NodeMask(NodeVcc);
    constexpr uint16_t kRunglerTriad    = NodeMask(NodeClk) | NodeMask(NodeOsc) | NodeMask(NodeFlt);
    constexpr uint16_t kPingDrumTriad    = NodeMask(NodeOsc) | NodeMask(NodeAmp) | NodeMask(NodeFlt);
    constexpr uint16_t kClkAmpFlt        = NodeMask(NodeClk) | NodeMask(NodeAmp) | NodeMask(NodeFlt);
    constexpr uint16_t kVoiceQuartet     = NodeMask(NodeClk) | NodeMask(NodeOsc) | NodeMask(NodeAmp) | NodeMask(NodeFlt);
}

struct MultiTouchTopology {
    uint16_t touch_mask{0};
    uint8_t  active_node_count{0};
    float    norm_scale{1.0f};
    float    cluster_load{0.0f};

    // Co-occurrence state flags (subset matching: true if mask is contained)
    bool pendulum_slice{false};
    bool lissajous_cloud{false};
    bool karplus_resonator{false};
    bool clock_siren{false};
    bool intermod_loop{false};
    bool voltage_pump{false};
    bool rungler_triad{false};
    bool ping_drum_triad{false};
    bool clk_amp_flt{false};
    bool voice_quartet{false};

    void Reset() {
        touch_mask = 0;
        active_node_count = 0;
        norm_scale = 1.0f;
        cluster_load = 0.0f;
        pendulum_slice = false;
        lissajous_cloud = false;
        karplus_resonator = false;
        clock_siren = false;
        intermod_loop = false;
        voltage_pump = false;
        rungler_triad = false;
        ping_drum_triad = false;
        clk_amp_flt = false;
        voice_quartet = false;
    }

    void Analyze(const Contacts& contacts) {
        touch_mask = 0;
        active_node_count = 0;

        uint16_t live_mask = 0;
        uint8_t  live_count = 0;
        uint16_t latched_mask = 0;
        uint8_t  latched_count = 0;

        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            float p_eff = contacts.Pressure(i);
            if (p_eff > 0.025f) {
                touch_mask |= static_cast<uint16_t>(1u << i);
                active_node_count++;
            }
            if (contacts.PressureLive(i) > 0.025f) {
                live_mask |= static_cast<uint16_t>(1u << i);
                live_count++;
            }
            if (contacts.LatchedPressure(i) > 0.025f) {
                latched_mask |= static_cast<uint16_t>(1u << i);
                latched_count++;
            }
        }

        // Conductance normalization: scales pairwise gain when 3+ pads are touched.
        // When latching is active, scale by live gesture or latched gesture independently
        // so playing a live 2-pad bend does not drop the gain of a latched 2-pad bend.
        static constexpr float kNormScale[13] = {
            1.0f, 1.0f, 1.0f,
            0.70710678f, // 3: 1/sqrt(2)
            0.57735027f, // 4: 1/sqrt(3)
            0.50000000f, // 5: 1/sqrt(4)
            0.44721360f, // 6: 1/sqrt(5)
            0.40824829f, // 7: 1/sqrt(6)
            0.37796447f, // 8: 1/sqrt(7)
            0.35355339f, // 9: 1/sqrt(8)
            0.33333333f, // 10: 1/sqrt(9)
            0.31622777f, // 11: 1/sqrt(10)
            0.30151134f  // 12: 1/sqrt(11)
        };
        uint8_t eff_norm_count = (contacts.HasLatchedSnapshot() && live_count > 0)
                                    ? std::max(live_count, latched_count)
                                    : active_node_count;
        norm_scale = kNormScale[std::min(eff_norm_count, static_cast<uint8_t>(12))];

        // Cluster galvanic rail load (draws down VCC on 4+ pads)
        if (active_node_count >= 4) {
            cluster_load = 0.045f * static_cast<float>(active_node_count - 3);
        } else {
            cluster_load = 0.0f;
        }

        // Chord topologies represent intentional multi-finger gestures.
        // A chord triggers if the player touches all pads together live, OR
        // if the chord was latched together as a unit snapshot. Disjoint pads
        // (e.g. latched OSC+AMP with live CLK+DLY) do NOT trigger phantom chords.
        auto check_chord = [&](uint16_t mask) -> bool {
            if ((live_mask & mask) == mask) return true;
            if (contacts.HasLatchedSnapshot() && (latched_mask & mask) == mask) return true;
            return false;
        };

        pendulum_slice    = check_chord(TopologyMasks::kPendulumSlice);
        karplus_resonator = check_chord(TopologyMasks::kKarplusResonator);
        clock_siren       = check_chord(TopologyMasks::kClockSiren);
        voltage_pump      = check_chord(TopologyMasks::kVoltagePump);
        intermod_loop     = check_chord(TopologyMasks::kIntermodLoop);
        voice_quartet     = check_chord(TopologyMasks::kVoiceQuartet);
        clk_amp_flt       = check_chord(TopologyMasks::kClkAmpFlt);
        ping_drum_triad   = check_chord(TopologyMasks::kPingDrumTriad);

        if (active_node_count <= 6) {
            lissajous_cloud   = check_chord(TopologyMasks::kLissajousCloud);
            rungler_triad    = check_chord(TopologyMasks::kRunglerTriad);
        } else {
            lissajous_cloud   = false;
            rungler_triad    = false;
        }
    }
};

} // namespace bends
