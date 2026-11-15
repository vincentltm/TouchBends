#pragma once

#include "bends/slots.h"
#include "bends/signals.h"
#include <cstdint>
#include <cstddef>
#include <cmath>

namespace bends {

enum ModSlotIndex : uint8_t {
    kModClkRate = 0,
    kModOscFm = 1,
    kModAmpTremolo = 2,
    kModFltCutoff = 3,
    kModFltClockStep = 4,
    kModDlyPhaseMod = 5,
    kModDlyClockFm = 6,
    kModDlyRecircFm = 7,
    kModDlyClockBurst = 8,
    kModPreampDrive = 9,
    kModMixTremolo = 10,
    kNumModSlots = 11
};

inline int8_t SlotToModIndex(Slot s) {
    static constexpr int8_t kLookup[] = {
        -1, // None = 0
#define X(name, combine, att, rel, rate) \
        (Slot::name == Slot::ClkRate ? kModClkRate : \
         Slot::name == Slot::OscFm ? kModOscFm : \
         Slot::name == Slot::AmpTremolo ? kModAmpTremolo : \
         Slot::name == Slot::FltCutoff ? kModFltCutoff : \
         Slot::name == Slot::FltClockStep ? kModFltClockStep : \
         Slot::name == Slot::DlyPhaseMod ? kModDlyPhaseMod : \
         Slot::name == Slot::DlyClockFm ? kModDlyClockFm : \
         Slot::name == Slot::DlyRecircFm ? kModDlyRecircFm : \
         Slot::name == Slot::DlyClockBurst ? kModDlyClockBurst : \
         Slot::name == Slot::PreampDrive ? kModPreampDrive : \
         Slot::name == Slot::MixTremolo ? kModMixTremolo : -1),
#include "slots.def"
#undef X
    };
    size_t idx = static_cast<size_t>(s);
    return (idx < sizeof(kLookup)) ? kLookup[idx] : -1;
}

struct ModSource {
    Sig   sig{Sig::One};
    float gain{0.0f};
};

class ModMatrix {
public:
    static constexpr size_t kMaxPerSlot = 5;

    void Reset() {
        for (size_t i = 0; i < kNumModSlots; ++i) {
            _counts[i] = 0;
        }
        _total_count = 0;
        _used_signals_mask = 0;
    }

    void Add(Slot slot, Sig sig, float gain) {
        if (std::abs(gain) < 0.001f) return;
        int8_t idx = SlotToModIndex(slot);
        if (idx < 0) return;
        uint8_t c = _counts[idx];
        if (c < kMaxPerSlot) {
            _entries[idx][c] = { sig, gain };
            _counts[idx] = c + 1;
            _total_count++;
            _used_signals_mask |= static_cast<uint16_t>(1u << static_cast<size_t>(sig));
        }
    }

    inline size_t Count() const { return _total_count; }
    inline bool HasSlot(ModSlotIndex slot_idx) const { return _counts[slot_idx] > 0; }
    inline bool HasSlot(Slot slot) const {
        int8_t idx = SlotToModIndex(slot);
        return (idx >= 0) ? (_counts[idx] > 0) : false;
    }
    inline uint16_t UsedSignalsMask() const { return _used_signals_mask; }

    // Evaluate active modulation routes for slot
    __attribute__((always_inline)) inline float Eval(ModSlotIndex slot_idx, float base_amount, const float* sig_bus) const {
        uint8_t c = _counts[slot_idx];
        if (c == 0) return base_amount;
        float val = base_amount;
        for (uint8_t i = 0; i < c; ++i) {
            val += _entries[slot_idx][i].gain * sig_bus[static_cast<size_t>(_entries[slot_idx][i].sig)];
        }
        return val;
    }

    // Overload for compatibility with Slot enum
    __attribute__((always_inline)) inline float Eval(Slot slot, float base_amount, const float* sig_bus) const {
        int8_t idx = SlotToModIndex(slot);
        if (idx < 0) return base_amount;
        return Eval(static_cast<ModSlotIndex>(idx), base_amount, sig_bus);
    }

private:
    ModSource _entries[kNumModSlots][kMaxPerSlot];
    uint8_t   _counts[kNumModSlots]{0};
    uint8_t   _total_count{0};
    uint16_t  _used_signals_mask{0};
};

} // namespace bends

