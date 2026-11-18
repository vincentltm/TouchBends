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
                float target = -0.10f * _thermal_sag[i];
                float c = NodeTable::NodeCapacitance[i];
                float g = NodeTable::SourceConductance[i];
                float alpha = std::clamp(g / (g + c * 4.5f), 0.04f, 0.75f);
                _smooth_pull[i] = Sanitize(_smooth_pull[i], 0.0f);
                _smooth_pull[i] += alpha * (target - _smooth_pull[i]);
                if (std::abs(_smooth_pull[i]) < 0.0005f) _smooth_pull[i] = 0.0f;
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

        // 3-pass Gauss-Seidel relaxation with Successive Over-Relaxation (SOR)
        // Solves true multi-hop resistor ladder networks across connected pad clusters
        constexpr uint8_t kSolverPasses = 3;
        constexpr float   kOmega = 1.25f; // SOR acceleration parameter

        float sum_g[kSolverNodes];
        for (size_t i = 0; i < kSolverNodes; ++i) {
            sum_g[i] = g_stiff[i];
        }

        uint8_t relax_br = std::min(num_bridges, static_cast<uint8_t>(Contacts::kMaxActiveBridges));
        for (uint8_t k = 0; k < relax_br; ++k) {
            uint8_t a = bridges[k].a;
            uint8_t b = bridges[k].b;
            float g = bridges[k].g;
            if (a < kSolverNodes && b < kSolverNodes) {
                sum_g[a] += g;
                sum_g[b] += g;
            }
        }

        float inv_sum_g[kSolverNodes];
        for (size_t i = 0; i < kSolverNodes; ++i) {
            inv_sum_g[i] = (sum_g[i] > 0.0001f) ? (1.0f / sum_g[i]) : 1.0f;
        }

        for (uint8_t pass = 0; pass < kSolverPasses; ++pass) {
            float sum_gv[kSolverNodes];
            for (size_t i = 0; i < kSolverNodes; ++i) {
                sum_gv[i] = g_stiff[i] * v_rest[i];
            }

            for (uint8_t k = 0; k < relax_br; ++k) {
                uint8_t a = bridges[k].a;
                uint8_t b = bridges[k].b;
                float g = bridges[k].g;
                if (a < kSolverNodes && b < kSolverNodes) {
                    sum_gv[a] += g * v_target[b];
                    sum_gv[b] += g * v_target[a];
                }
            }

            for (size_t i = 0; i < kSolverNodes; ++i) {
                float v_new = sum_gv[i] * inv_sum_g[i];
                v_target[i] += kOmega * (v_new - v_target[i]);
            }
        }

        // Compute total current drawn from VCC and Joule heating dissipation across all active bridges
        float vcc_current = 0.0f;
        float joule_power[kSolverNodes]{0.0f};

        for (uint8_t k = 0; k < relax_br; ++k) {
            uint8_t a = bridges[k].a;
            uint8_t b = bridges[k].b;
            float g = bridges[k].g;
            if (a < kSolverNodes && b < kSolverNodes) {
                float v_diff = v_target[a] - v_target[b];
                float p_bridge = g * v_diff * v_diff;
                joule_power[a] += p_bridge;
                joule_power[b] += p_bridge;

                if (a == NodeVcc || b == NodeVcc) {
                    uint8_t other = (a == NodeVcc) ? b : a;
                    vcc_current += g * std::max(0.0f, v_target[NodeVcc] - v_target[other]);
                }
            }
        }
        _frame.vcc_current = vcc_current;

        float vcc_scale = (rail_voltage > 0.08f) ? rail_voltage : 0.5f;
        float inv_vcc_scale = 1.0f / vcc_scale;

        for (size_t i = 0; i < kSolverNodes; ++i) {
            float rest = (i == NodeVcc) ? rail_voltage : NodeTable::Specs[i].rest_dc;
            float inv_scale = (i == NodeVcc) ? inv_vcc_scale : ((i == NodeRst) ? 1.0f : 2.0f);
            float diff = (v_target[i] - rest) * inv_scale;

            // Soft diode saturation near rail boundaries (prevents harsh digital brick-wall clipping)
            float raw_pull;
            float abs_d = std::abs(diff);
            if (abs_d < 0.70f) {
                raw_pull = diff;
            } else {
                float excess = abs_d - 0.70f;
                float compressed = 0.70f + excess / (1.0f + excess * 1.5f);
                raw_pull = (diff > 0.0f) ? std::min(1.0f, compressed) : -std::min(1.0f, compressed);
            }

            // Starved CMOS subthreshold non-linearity:
            // Under starvation (< 0.70V), FET channel conduction operates in weak inversion.
            // Small finger pressure variations produce expressive, organic non-linear responses.
            if (rail_voltage < 0.70f) {
                float starve_depth = (0.70f - rail_voltage) * (1.0f / 0.70f);
                float sign = (raw_pull >= 0.0f) ? 1.0f : -1.0f;
                float mag = std::abs(raw_pull);
                float sub_mag = mag * (1.0f + starve_depth * (mag - 0.5f));
                raw_pull = sign * std::clamp(sub_mag, 0.0f, 1.0f);
            }

            // Physical Joule heating: contact power dissipation warms the virtual junction
            if (joule_power[i] > 0.002f) {
                _thermal_sag[i] += 0.012f * (joule_power[i] * 1.5f - _thermal_sag[i]);
            } else {
                _thermal_sag[i] *= 0.9985f; // Thermal relaxation tail (~2.2s t60 at 750 Hz)
            }
            _thermal_sag[i] = std::clamp(_thermal_sag[i], 0.0f, 1.0f);

            // Thermal sag shifts DC bias slightly (junction voltage temperature coefficient -2.1mV/C)
            float thermal_bias = -0.10f * _thermal_sag[i] * (raw_pull >= 0.0f ? 1.0f : -1.0f);
            float effective_pull = std::clamp(raw_pull + thermal_bias, -1.0f, 1.0f);

            // Dynamic RC Ballistics (tau = C_node / G_total)
            // Light feather touch = smooth spongy glide; hard press = instant punchy snap
            float c_node = NodeTable::NodeCapacitance[i];
            float alpha_i = std::clamp(sum_g[i] / (sum_g[i] + c_node * 4.5f), 0.04f, 0.75f);

            _smooth_pull[i] += alpha_i * (effective_pull - _smooth_pull[i]);
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
