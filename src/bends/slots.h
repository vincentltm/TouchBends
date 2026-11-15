#pragma once

#include <cstdint>
#include <cstddef>
#include "bends/signals.h"

namespace bends {

enum class Slot : uint8_t {
    None = 0,
#define X(name, combine, att, rel, rate) name,
#include "slots.def"
#undef X
    Count
};

inline const char* SlotToString(Slot slot) {
    switch (slot) {
        case Slot::None: return "None";
#define X(name, combine, att, rel, rate) case Slot::name: return #name;
#include "slots.def"
#undef X
        default: return "Unknown";
    }
}

inline bool SlotIsAudioRate(Slot slot) {
    switch (slot) {
#define X(name, combine, att, rel, rate) \
        case Slot::name: return (#rate[0] == 'A');
#include "slots.def"
#undef X
        default: return false;
    }
}

inline float SlotAttackMs(Slot slot) {
    switch (slot) {
#define X(name, combine, att, rel, rate) case Slot::name: return static_cast<float>(att);
#include "slots.def"
#undef X
        default: return 5.0f;
    }
}

inline float SlotReleaseMs(Slot slot) {
    switch (slot) {
#define X(name, combine, att, rel, rate) case Slot::name: return static_cast<float>(rel);
#include "slots.def"
#undef X
        default: return 40.0f;
    }
}

enum class CombineRule : uint8_t {
    Sum,
    Max
};

inline CombineRule SlotCombineRule(Slot slot) {
    switch (slot) {
#define X(name, combine, att, rel, rate) case Slot::name: return CombineRule::combine;
#include "slots.def"
#undef X
        default: return CombineRule::Sum;
    }
}

} // namespace bends
