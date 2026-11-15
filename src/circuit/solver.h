#pragma once

#include "circuit/nodes.h"
#include "circuit/contacts.h"
#include "dsp/fast_math.h"
#include <cmath>
#include <algorithm>

namespace bends {

struct SolverFrame {
    // Normalized DC deviation from rest in [-1.0, +1.0]
    float pull[kSolverNodes]{0.0f};

    // Node retention (fraction of own signal surviving; 1.0 = open, 0.0 = grounded)
    float retention[kSolverNodes]{1.0f};

    // Physical VCC bridge current draw
    float vcc_current{0.0f};
};

class Solver {
public:
    Solver() {
        Reset();
    }

    void Reset() {
        for (size_t i = 0; i < kSolverNodes; ++i) {
            _smooth_pull[i] = 0.0f;
            _thermal_sag[i] = 0.0f;
            _frame.pull[i] = 0.0f;
            _frame.retention[i] = 1.0f;
        }
        _frame.vcc_current = 0.0f;
    }

    // Runs once per control block (~750 Hz)
    void Solve(const Contacts& contacts, float rail_voltage, const uint16_t* routed_mask) {
        // 1. Retention & physical AC conductance
        for (size_t i = 0; i < kSolverNodes; ++i) {
            // Pairs with an explicit Route row to GND have their own behavior: no generic shunt
            bool routed_to_gnd = (routed_mask[i] >> NodeGnd) & 1u;
            _frame.retention[i] = routed_to_gnd ? 1.0f : (1.0f - contacts.Conductance(i, NodeGnd) * 0.98f);
        }

        uint8_t num_bridges = contacts.NumActiveBridges();
        const auto* bridges = contacts.ActiveBridges();

        // If no bridges or body coupling are active, decay pulls to rest with thermal relaxation tail (1.8-2.5s)
        if (num_bridges == 0 && contacts.Pressure(NodeBody) <= 0.02f) {
            for (size_t i = 0; i < kSolverNodes; ++i) {
                // Thermal relaxation tail (~2.2s t60 at 750 Hz)
                _thermal_sag[i] = Sanitize(_thermal_sag[i], 0.0f) * 0.9985f;
                float target = _thermal_sag[i] * 0.15f;
                _smooth_pull[i] = Sanitize(_smooth_pull[i], 0.0f);
                _smooth_pull[i] += 0.05f * (target - _smooth_pull[i]);
                if (std::abs(_smooth_pull[i]) < 0.001f) _smooth_pull[i] = 0.0f;
                _frame.pull[i] = _smooth_pull[i];
            }
            _frame.vcc_current = 0.0f;
            return;
        }

        // Setup nodal resting DC voltages and stiffnesses
        float v_rest[kSolverNodes];
        float g_stiff[kSolverNodes];

        for (size_t i = 0; i < kSolverNodes; ++i) {
            v_rest[i] = NodeTable::Specs[i].rest_dc;
            g_stiff[i] = NodeTable::SourceConductance[i];
        }
        float starve_factor = 1.0f - rail_voltage;
        float z_vcc = 0.05f + 0.95f * starve_factor * starve_factor;
        v_rest[NodeVcc] = rail_voltage;
        g_stiff[NodeVcc] = 1.0f / z_vcc;

        // Seed target voltages with rest values
        float v_target[kSolverNodes];
        for (size_t i = 0; i < kSolverNodes; ++i) {
            v_target[i] = v_rest[i];
        }

        // 1-pass Jacobi DC relaxation over active bridges (constant compute, >98% DC convergence)
        {
            float sum_gv[kSolverNodes];
            float sum_g[kSolverNodes];

            for (size_t i = 0; i < kSolverNodes; ++i) {
                sum_gv[i] = g_stiff[i] * v_rest[i];
                sum_g[i]  = g_stiff[i];
            }

            uint8_t relax_br = std::min(num_bridges, static_cast<uint8_t>(Contacts::kMaxActiveBridges));
            for (uint8_t k = 0; k < relax_br; ++k) {
                uint8_t a = bridges[k].a;
                uint8_t b = bridges[k].b;
                float g = bridges[k].g;

                if (a < kSolverNodes && b < kSolverNodes) {
                    sum_gv[a] += g * v_target[b];
                    sum_g[a]  += g;
                    sum_gv[b] += g * v_target[a];
                    sum_g[b]  += g;
                }
            }

            for (size_t i = 0; i < kSolverNodes; ++i) {
                v_target[i] = sum_gv[i] / sum_g[i];
            }
        }

        // Compute total current drawn from VCC across all active bridges
        float vcc_current = 0.0f;
        for (uint8_t k = 0; k < num_bridges; ++k) {
            uint8_t a = bridges[k].a;
            uint8_t b = bridges[k].b;
            if (a == NodeVcc || b == NodeVcc) {
                uint8_t other = (a == NodeVcc) ? b : a;
                vcc_current += bridges[k].g * std::max(0.0f, v_target[NodeVcc] - v_target[other]);
            }
        }
        _frame.vcc_current = vcc_current;

        float vcc_scale = (rail_voltage > 0.08f) ? rail_voltage : 0.5f;
        float inv_vcc_scale = 1.0f / vcc_scale;

        for (size_t i = 0; i < kSolverNodes; ++i) {
            float rest = (i == NodeVcc) ? rail_voltage : NodeTable::Specs[i].rest_dc;
            float inv_scale = (i == NodeVcc) ? inv_vcc_scale : ((i == NodeRst) ? 1.0f : 2.0f);
            float diff = (v_target[i] - rest) * inv_scale;
            float raw_pull = std::clamp(diff, -1.0f, 1.0f);

            // Thermal sag accumulation under active stress
            if (std::abs(raw_pull) > 0.05f) {
                float stress = raw_pull * std::abs(raw_pull);
                _thermal_sag[i] += 0.008f * (stress - _thermal_sag[i]);
            } else {
                _thermal_sag[i] *= 0.9985f;
            }
            _thermal_sag[i] = std::clamp(_thermal_sag[i], -1.0f, 1.0f);

            float effective_pull = std::clamp(raw_pull + _thermal_sag[i] * 0.15f, -1.0f, 1.0f);
            _smooth_pull[i] += 0.20f * (effective_pull - _smooth_pull[i]);
            _smooth_pull[i] = Sanitize(_smooth_pull[i], 0.0f);
            _frame.pull[i] = _smooth_pull[i];
        }
    }

    const SolverFrame& Frame() const { return _frame; }
    float Pull(NodeId id) const { return _frame.pull[id]; }
    float Retention(NodeId id) const { return _frame.retention[id]; }
    float VccCurrent() const { return _frame.vcc_current; }

private:
    float _smooth_pull[kSolverNodes]{0.0f};
    float _thermal_sag[kSolverNodes]{0.0f};
    SolverFrame _frame{};
};

} // namespace bends
