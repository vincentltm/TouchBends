#pragma once

#include "bends/bend.h"
#include "bends/table.h"
#include "bends/amount_bus.h"
#include "bends/mod_matrix.h"
#include "circuit/contacts.h"
#include "circuit/topology.h"
#include "tuning/tuning.h"
#include <cstdint>

namespace bends {

class BendEngine {
public:
    void Init(float control_rate_hz = 750.0f) {
        _amount_bus.Init(control_rate_hz);
        _mod_matrix.Reset();
        _flush_requested = false;
        for (size_t i = 0; i < kSolverNodes; ++i) {
            _routed_mask[i] = 0;
        }
        for (size_t i = 0; i < kNumMasterBends; ++i) {
            _was_engaged[i] = false;
        }
    }

    void Compile(const Contacts& contacts,
                 const MultiTouchTopology& topology,
                 uint8_t when_flags) {
        _amount_bus.ResetRaw();
        _mod_matrix.Reset();
        _flush_requested = false;
        for (size_t i = 0; i < kSolverNodes; ++i) {
            _routed_mask[i] = 0;
        }

        for (size_t i = 0; i < kNumMasterBends; ++i) {
            const auto& b = kMasterBendTable[i];

            bool when_ok = (b.when == When::Always) || ((b.when & when_flags) != 0);
            if (!when_ok) {
                _was_engaged[i] = false;
                continue;
            }

            float trigger_val = 0.0f;
            if (b.type == TriggerType::Pair) {
                trigger_val = contacts.Conductance(b.a, b.b);
            } else if (b.type == TriggerType::Solo) {
                if (contacts.IsAlone(b.a)) {
                    trigger_val = contacts.Pressure(b.a);
                }
            }

            bool engaged = (trigger_val > tuning::kBridgeThreshold);
            bool onset = engaged && !_was_engaged[i];
            _was_engaged[i] = engaged;

            if (!engaged) continue;

            // Route / NoBleed flag disables generic solver shunt
            if (b.flags & Flag::NoBleed) {
                _routed_mask[b.a] |= static_cast<uint16_t>(1u << b.b);
                _routed_mask[b.b] |= static_cast<uint16_t>(1u << b.a);
            }

            // Memory flush on dedicated RST-GND bridge
            if (b.flags & Flag::Panic) {
                bool direct_rst_gnd = (contacts.Pressure(NodeRst) > 0.15f && contacts.Pressure(NodeGnd) > 0.15f) ||
                                      (contacts.Conductance(NodeRst, NodeGnd) > 0.25f);
                if (direct_rst_gnd && topology.active_node_count <= 2) {
                    _flush_requested = true;
                }
            }

            // Multi-touch chord priority protection:
            if (topology.pendulum_slice) {
                // When holding CLK + DLY + RST (Pendulum Slice), suppress competing static reverse and restart
                if (b.slot == Slot::DlyReverse || b.slot == Slot::DlyRestart) {
                    continue;
                }
            }


            // If Onset flag is specified, only trigger on fresh touch
            if ((b.flags & Flag::Onset) && !onset) {
                continue;
            }

            if (b.slot == Slot::None) {
                continue;
            }

            // Scale effective gain by topology conductance normalization
            float eff_gain = trigger_val * b.depth * topology.norm_scale;

            if (b.source == Sig::One) {
                _amount_bus.Apply(b.slot, eff_gain);
            } else {
                _mod_matrix.Add(b.slot, b.source, eff_gain);
            }
        }

        _amount_bus.UpdateBlock();
    }

    const AmountBus& Amounts() const { return _amount_bus; }
    AmountBus& Amounts() { return _amount_bus; }
    const ModMatrix& Matrix() const { return _mod_matrix; }
    bool FlushRequested() const { return _flush_requested; }
    const uint16_t* RoutedMask() const { return _routed_mask; }
    bool IsRouted(uint8_t a, uint8_t b) const { return (_routed_mask[a] >> b) & 1u; }
    uint32_t ActiveBendsMask() const {
        uint32_t mask = 0;
        for (size_t i = 0; i < kNumMasterBends && i < 32; ++i) {
            if (_was_engaged[i]) mask |= (1u << i);
        }
        return mask;
    }

private:
    AmountBus _amount_bus;
    ModMatrix _mod_matrix;
    bool      _flush_requested{false};
    uint16_t  _routed_mask[kSolverNodes]{0};
    bool      _was_engaged[kNumMasterBends]{false};
};

} // namespace bends
