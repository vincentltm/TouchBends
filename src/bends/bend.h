#pragma once

#include "circuit/nodes.h"
#include "bends/slots.h"
#include <cstdint>

namespace bends {

enum class TriggerType : uint8_t {
    Pair,  // Conductance between node a and node b
    Solo   // Node a touched alone
};

namespace Flag {
    constexpr uint8_t None    = 0;
    constexpr uint8_t NoBleed = (1 << 0); // Disables generic nodal shunt
    constexpr uint8_t Panic   = (1 << 1); // Triggers buffer flush
    constexpr uint8_t Onset   = (1 << 2); // Triggers once on initial touch
}

namespace When {
    constexpr uint8_t Always  = 0;
    constexpr uint8_t Saw     = (1 << 0);
    constexpr uint8_t Sin     = (1 << 1);
    constexpr uint8_t Rnd     = (1 << 2);
    constexpr uint8_t ClkSlow = (1 << 3);
    constexpr uint8_t ClkFast = (1 << 4);
    constexpr uint8_t Latched = (1 << 5);
}

struct Bend {
    TriggerType type;
    NodeId      a;
    NodeId      b;
    Slot        slot;
    Sig         source;
    float       depth;
    uint8_t     when;
    uint8_t     flags;
};

#define BEND_PAIR(node_a, node_b, slot_val, sig_val, depth_val, when_val, flags_val) \
    { bends::TriggerType::Pair, bends::Node##node_a, bends::Node##node_b, slot_val, sig_val, depth_val, when_val, flags_val }

#define BEND_SOLO(node_a, slot_val, sig_val, depth_val, when_val, flags_val) \
    { bends::TriggerType::Solo, bends::Node##node_a, bends::Node##node_a, slot_val, sig_val, depth_val, when_val, flags_val }

} // namespace bends
