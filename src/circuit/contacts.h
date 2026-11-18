#pragma once

#include "circuit/nodes.h"
#include "surface/frame.h"
#include "tuning/tuning.h"
#include <cmath>
#include <algorithm>

namespace bends {

struct ActiveBridge {
    uint8_t a;
    uint8_t b;
    float   g;
};

class Contacts {
public:
    static constexpr size_t kMaxActiveBridges = 8;

    Contacts() {
        Reset();
    }

    void Reset() {
        _has_latched_snapshot = false;
        _prev_is_latched = false;
        _was_touching_in_latch = false;
        _has_latched_bridges = false;
        _num_active_bridges = 0;
        _num_active_nodes = 0;
        _alone_node = -1;
        _total_current_load = 0.0f;
        _dynamic_motion = 0.0f;
        _digital_motion_ratio = 0.0f;
        _live_touched = 0;

        for (size_t i = 0; i < kSolverNodes; ++i) {
            _pressures[i] = 0.0f;
            _latched_pressures[i] = 0.0f;
            for (size_t j = 0; j < kSolverNodes; ++j) {
                _conductance[i][j] = 0.0f;
                _latched_conductance[i][j] = 0.0f;
            }
        }
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            _live_pressures[i] = 0.0f;
            _prev_pressures[i] = 0.0f;
            _sweat_drift[i] = 0.0f;
        }
        _crackle_seed = 0x9E3779B9u;
    }

    // Process touch surface contacts at block rate (750 Hz)
    void Process(const ControlFrame& frame, float bend_depth = 1.0f) {
        float sum_p = 0.0f;
        float sum_g = 0.0f;

        bool is_latched = frame.is_latched;

        // 0. Update bio-galvanic contact mechanics:
        // Fingertip perspiration accumulation under contact pressure, slow moisture evaporation on release
        float eff_live_p[kPhysicalNodes];
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            float raw = frame.pressures[i];
            if (raw > tuning::kTouchThreshold) {
                _sweat_drift[i] += 0.0015f * (0.24f * raw - _sweat_drift[i]);
            } else {
                _sweat_drift[i] *= 0.995f;
            }
            eff_live_p[i] = std::clamp(raw + _sweat_drift[i], 0.0f, 1.0f);
        }

        // 1. Calculate live physical bridges between pairs touched simultaneously
        float live_g[kPhysicalNodes][kPhysicalNodes] = {{0.0f}};
        uint8_t live_touched = 0;
        int8_t live_alone = -1;
        float sum_touched_p = 0.0f;
        float delta_p[kPhysicalNodes];

        uint8_t touched_idx[kPhysicalNodes];
        float total_motion = 0.0f;
        float digital_motion = 0.0f;
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            float pi = eff_live_p[i];
            delta_p[i] = std::abs(pi - _prev_pressures[i]);
            _prev_pressures[i] = pi;

            _live_pressures[i] = pi;
            if (pi > tuning::kTouchThreshold) {
                touched_idx[live_touched++] = static_cast<uint8_t>(i);
                live_alone = static_cast<int8_t>(i);
                sum_touched_p += pi;
                if (delta_p[i] > 0.003f) {
                    total_motion += delta_p[i];
                    if (i == NodeClk || i == NodeDly || i == NodeRst) {
                        digital_motion += delta_p[i];
                    }
                }
            }
        }
        _live_touched = live_touched;
        _dynamic_motion = total_motion;
        _digital_motion_ratio = (total_motion > 0.001f) ? (digital_motion / total_motion) : 0.0f;

        if (live_touched >= 2) {
            const float inv_sum_touched_p = (sum_touched_p > 0.001f) ? (1.0f / sum_touched_p) : 1.0f;
            for (uint8_t ti = 0; ti < live_touched; ++ti) {
                uint8_t i = touched_idx[ti];
                float pi = eff_live_p[i];
                for (uint8_t tj = ti + 1; tj < live_touched; ++tj) {
                    uint8_t j = touched_idx[tj];
                    float pj = eff_live_p[j];
                    float p_sum = pi + pj;
                    float num = 2.0f * pi * pj;
                    float p_direct = (std::abs(pi - pj) < 0.005f) ? pi : (num / p_sum);
                    float p_star = (sum_touched_p > p_sum) ? (num * inv_sum_touched_p) : p_direct;

                    // Adjacent copper seams share a direct local bridge (single coin or finger);
                    // Non-adjacent pairs span the player's body and obey star-mesh network current division:
                    float series_p = NodeTable::IsAdjacent(i, j) ? (0.70f * p_direct + 0.30f * p_star) : p_star;

                    // Fast galvanic series curve approximation (avoids 14-cycle VFP vsqrt in 66-pair loop)
                    float eff_series = (series_p > 0.005f) ? (series_p * (1.5f - 0.5f * series_p)) : series_p;
                    float g = eff_series * NodeTable::PhysicalWeight(i, j) * bend_depth;

                    live_g[i][j] = std::min(1.0f, g);
                    live_g[j][i] = live_g[i][j];
                }
            }
        }

        // Latch snapshot management
        if (is_latched && !_prev_is_latched) {
            ClearLatched();
            _has_latched_snapshot = false;
            _was_touching_in_latch = (live_touched > 0);
            if (live_touched > 0) {
                for (size_t i = 0; i < kPhysicalNodes; ++i) {
                    if (eff_live_p[i] > tuning::kTouchThreshold) {
                        _latched_pressures[i] = eff_live_p[i];
                    }
                    for (size_t j = i + 1; j < kPhysicalNodes; ++j) {
                        if ((i == NodeGnd && j == NodeVcc) || (i == NodeVcc && j == NodeGnd)) continue; // Never latch power rail short
                        if (live_g[i][j] > tuning::kBridgeThreshold) {
                            _latched_conductance[i][j] = live_g[i][j];
                            _latched_conductance[j][i] = live_g[i][j];
                        }
                    }
                }
            }
        } else if (is_latched) {
            if (!_has_latched_snapshot) {
                if (live_touched > 0) {
                    for (size_t i = 0; i < kPhysicalNodes; ++i) {
                        if (eff_live_p[i] > tuning::kTouchThreshold) {
                            _latched_pressures[i] = std::max(_latched_pressures[i], eff_live_p[i]);
                        }
                        for (size_t j = i + 1; j < kPhysicalNodes; ++j) {
                            if ((i == NodeGnd && j == NodeVcc) || (i == NodeVcc && j == NodeGnd)) continue; // Never latch power rail short
                            if (live_g[i][j] > tuning::kBridgeThreshold) {
                                float peak_g = std::max(_latched_conductance[i][j], live_g[i][j]);
                                _latched_conductance[i][j] = peak_g;
                                _latched_conductance[j][i] = peak_g;
                            }
                        }
                    }
                    _was_touching_in_latch = true;
                } else if (_was_touching_in_latch) {
                    // Lock captured snapshot on release
                    _has_latched_snapshot = true;
                    _was_touching_in_latch = false;
                }
            }
        } else {
            // Latch switch is UP / Center: momentary only
            if (_prev_is_latched) {
                ClearLatched();
            }
            _has_latched_snapshot = false;
            _was_touching_in_latch = false;
        }
        _prev_is_latched = is_latched;

        // 3. Assemble total conductance matrix and active bridge list
        _num_active_bridges = 0;
        if (!is_latched) {
            for (size_t i = 0; i < kPhysicalNodes; ++i) {
                _conductance[i][i] = 0.0f;
                for (size_t j = i + 1; j < kPhysicalNodes; ++j) {
                    float total_g = live_g[i][j];
                    _conductance[i][j] = total_g;
                    _conductance[j][i] = total_g;

                    if (total_g > tuning::kBridgeThreshold) {
                        if (_num_active_bridges < kMaxActiveBridges) {
                            _active_bridges[_num_active_bridges++] = {
                                static_cast<uint8_t>(i),
                                static_cast<uint8_t>(j),
                                total_g
                            };
                        }
                        sum_g += total_g;
                    }
                }
            }
        } else {
            for (size_t i = 0; i < kPhysicalNodes; ++i) {
                _conductance[i][i] = 0.0f;
                for (size_t j = i + 1; j < kPhysicalNodes; ++j) {
                    float total_g = live_g[i][j];
                    if (_has_latched_snapshot) {
                        float latched = _latched_conductance[i][j];
                        total_g = std::clamp(latched + live_g[i][j] * (1.0f - latched), 0.0f, 1.0f);
                    } else {
                        total_g = std::max(live_g[i][j], _latched_conductance[i][j]);
                    }
                    total_g = std::min(1.0f, total_g);
                    _conductance[i][j] = total_g;
                    _conductance[j][i] = total_g;

                    if (total_g > tuning::kBridgeThreshold) {
                        if (_num_active_bridges < kMaxActiveBridges) {
                            _active_bridges[_num_active_bridges++] = {
                                static_cast<uint8_t>(i),
                                static_cast<uint8_t>(j),
                                total_g
                            };
                        }
                        sum_g += total_g;
                    }
                }
            }
        }

        // 4. Update effective pad pressures
        int8_t latched_alone = -1;
        uint8_t active_nodes = 0;
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            float live_p = eff_live_p[i];
            float eff_p = live_p;
            if (is_latched && _has_latched_snapshot) {
                // Properly scales live touch on top of latched baseline
                float latched = _latched_pressures[i];
                eff_p = std::clamp(latched + live_p * (1.0f - latched), 0.0f, 1.0f);
            } else if (is_latched) {
                eff_p = std::max(live_p, _latched_pressures[i]);
            }
            _pressures[i] = eff_p;
            sum_p += eff_p;
            if (eff_p > tuning::kTouchThreshold) {
                active_nodes++;
                latched_alone = static_cast<int8_t>(i);
            }
        }
        _num_active_nodes = active_nodes;

        // An isolated single pad is active if:
        // 1. Exactly 1 pad is touched live
        // 2. OR a single pad was latched alone without bridges
        if (live_touched == 1) {
            _alone_node = live_alone;
        } else if (live_touched == 0 && is_latched && _num_active_bridges == 0 && active_nodes == 1) {
            _alone_node = latched_alone;
        } else {
            _alone_node = -1;
        }

        // 5. Capacitive Body Coupling for NodeBody (Node 12)
        float body_p = 0.0f;
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            float p = _pressures[i];
            if (p > tuning::kTouchThreshold) {
                float g_body = p * 0.15f * bend_depth;
                _conductance[i][NodeBody] = g_body;
                _conductance[NodeBody][i] = g_body;
                body_p += p;
            } else {
                _conductance[i][NodeBody] = 0.0f;
                _conductance[NodeBody][i] = 0.0f;
            }
        }
        // Subtle ambient air field coupling to virtual body node when hands hover near the board
        float air_body = 0.0f;
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            air_body += frame.proximities[i] * 0.035f;
        }
        if (air_body > 0.005f) {
            _crackle_seed ^= _crackle_seed << 13;
            _crackle_seed ^= _crackle_seed >> 17;
            _crackle_seed ^= _crackle_seed << 5;
            float ac_flutter = ((static_cast<float>(_crackle_seed & 0xFF) / 255.0f) * 0.12f - 0.06f) * air_body;
            air_body = std::max(0.0f, air_body + ac_flutter);
        }
        _pressures[NodeBody] = std::min(1.0f, body_p + air_body);

        // Current draw onto power supply: multi-touch draws real galvanic current through the starved supply
        _total_current_load = std::min(1.0f, sum_p * 0.16f + sum_g * 0.32f);
        _has_latched_bridges = _has_latched_snapshot;
    }

    void ClearLatched() {
        _has_latched_snapshot = false;
        _has_latched_bridges = false;
        for (size_t i = 0; i < kSolverNodes; ++i) {
            _latched_pressures[i] = 0.0f;
            for (size_t j = 0; j < kSolverNodes; ++j) {
                _latched_conductance[i][j] = 0.0f;
            }
        }
    }

    __attribute__((always_inline)) inline float Conductance(size_t i, size_t j) const {
        return _conductance[i][j];
    }

    __attribute__((always_inline)) inline float Pressure(size_t i) const {
        return _pressures[i];
    }

    float TotalCurrentLoad() const { return _total_current_load; }

    uint8_t NumActiveBridges() const { return _num_active_bridges; }
    const ActiveBridge* ActiveBridges() const { return _active_bridges; }
    uint8_t NumActiveNodes() const { return _num_active_nodes; }

    bool IsAlone(size_t i) const {
        return (_alone_node == static_cast<int8_t>(i));
    }
    int8_t AloneNode() const { return _alone_node; }

    bool HasLatched() const {
        return _has_latched_bridges;
    }

    float DynamicMotion() const { return _dynamic_motion; }
    float DigitalMotionRatio() const { return _digital_motion_ratio; }

    inline float PressureLive(size_t i) const {
        return (i < kPhysicalNodes && _live_pressures[i] > tuning::kTouchThreshold) ? _live_pressures[i] : 0.0f;
    }
    inline uint8_t LiveTouchedCount() const { return _live_touched; }
    inline bool HasLatchedSnapshot() const { return _has_latched_snapshot; }
    inline float LatchedPressure(size_t i) const { return (i < kPhysicalNodes) ? _latched_pressures[i] : 0.0f; }

private:
    bool _has_latched_bridges{false};
    float _pressures[kSolverNodes]{0.0f};
    float _latched_pressures[kSolverNodes]{0.0f};
    float _live_pressures[kPhysicalNodes]{0.0f};
    float _conductance[kSolverNodes][kSolverNodes]{{0.0f}};
    float _latched_conductance[kSolverNodes][kSolverNodes]{{0.0f}};

    ActiveBridge _active_bridges[kMaxActiveBridges];
    uint8_t      _num_active_bridges{0};
    uint8_t      _num_active_nodes{0};
    uint8_t      _live_touched{0};
    int8_t       _alone_node{-1};
    float        _total_current_load{0.0f};
    float        _dynamic_motion{0.0f};
    float        _digital_motion_ratio{0.0f};

    bool     _has_latched_snapshot{false};
    bool     _prev_is_latched{false};
    bool     _was_touching_in_latch{false};

    float    _prev_pressures[kPhysicalNodes]{0.0f};
    float    _sweat_drift[kPhysicalNodes]{0.0f};
    uint32_t _crackle_seed{0x9E3779B9u};
};

} // namespace bends
