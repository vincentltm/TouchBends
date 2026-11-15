#pragma once

#include "daisy_seed.h"
#include <functional>
#include <array>

namespace synthux {

/**
 * MPR121 12-pad capacitive touch driver (P00–P11) with continuous pressure sensing.
 * Connected via I2C1 (SCL: PB8, SDA: PB9, Address: 0x5A).
 */
class Pads {
public:
    struct PadConfig {
        float min_delta;  // Touch threshold
        float rel_delta;  // Release hysteresis threshold
        float max_delta;  // 1.0 pressure delta
        float gamma;      // Curve power
        float floor_p;    // Contact floor
    };

    // Unified physical calibration constants (0 to 620 ADC counts)
    // Pure Electronic Continuum: hover (proximity 0.0..1.0, pressure 0.0) -> touch contact (pressure 0.04..1.00)
    static constexpr float kUnifiedMinDelta = 22.0f;  // Touch trigger threshold (physical contact confirmed)
    static constexpr float kUnifiedRelDelta = 12.0f;  // Release hysteresis threshold
    static constexpr float kUnifiedMaxDelta = 620.0f; // Full firm pressure threshold
    static constexpr float kUnifiedFloorP   = 0.04f;  // Initial contact floor
    static constexpr float kUnifiedGamma    = 1.35f;  // Non-linear tactile skin pulp compression exponent

    static constexpr std::array<PadConfig, 12> kDefaultCalib = {{
        { kUnifiedMinDelta, kUnifiedRelDelta, kUnifiedMaxDelta, kUnifiedGamma, kUnifiedFloorP }, // P00: MIX
        { kUnifiedMinDelta, kUnifiedRelDelta, kUnifiedMaxDelta, kUnifiedGamma, kUnifiedFloorP }, // P01: RST
        { kUnifiedMinDelta, kUnifiedRelDelta, kUnifiedMaxDelta, kUnifiedGamma, kUnifiedFloorP }, // P02: CLK
        { kUnifiedMinDelta, kUnifiedRelDelta, kUnifiedMaxDelta, kUnifiedGamma, kUnifiedFloorP }, // P03: OSC
        { kUnifiedMinDelta, kUnifiedRelDelta, kUnifiedMaxDelta, kUnifiedGamma, kUnifiedFloorP }, // P04: FLT
        { kUnifiedMinDelta, kUnifiedRelDelta, kUnifiedMaxDelta, kUnifiedGamma, kUnifiedFloorP }, // P05: GND
        { kUnifiedMinDelta, kUnifiedRelDelta, kUnifiedMaxDelta, kUnifiedGamma, kUnifiedFloorP }, // P06: DLY
        { kUnifiedMinDelta, kUnifiedRelDelta, kUnifiedMaxDelta, kUnifiedGamma, kUnifiedFloorP }, // P07: VCC
        { kUnifiedMinDelta, kUnifiedRelDelta, kUnifiedMaxDelta, kUnifiedGamma, kUnifiedFloorP }, // P08: AMP
        { kUnifiedMinDelta, kUnifiedRelDelta, kUnifiedMaxDelta, kUnifiedGamma, kUnifiedFloorP }, // P09: FBK
        { kUnifiedMinDelta, kUnifiedRelDelta, kUnifiedMaxDelta, kUnifiedGamma, kUnifiedFloorP }, // P10: IN
        { kUnifiedMinDelta, kUnifiedRelDelta, kUnifiedMaxDelta, kUnifiedGamma, kUnifiedFloorP }  // P11: OUT
    }};

    Pads() : _state{0}, _oor_state{0}, _exponential{true} {
        for (size_t i = 0; i < 12; i++) {
            _pressure[i] = 0.0f;
            _proximity[i] = 0.0f;
            _debounce_cnt[i] = 0;
            _calib[i] = kDefaultCalib[i];
        }
    }

    void Init(daisy::DaisySeed& hw);
    bool Process();
    void Recalibrate();

    // Response curve: true = quadratic (natural finger pulp resistance), false = linear
    void SetCurveExponential(bool exp) { _exponential = exp; }
    bool IsCurveExponential() const { return _exponential; }

    // Pad calibration accessors
    void SetPadCalib(uint16_t pad, const PadConfig& cfg) { if (pad < 12) _calib[pad] = cfg; }
    const PadConfig& GetPadCalib(uint16_t pad) const { return _calib[pad < 12 ? pad : 0]; }
    void SetMaxDelta(uint16_t pad, float max_delta) { if (pad < 12) _calib[pad].max_delta = max_delta; }
    float MaxDelta(uint16_t pad) const { return (pad < 12) ? _calib[pad].max_delta : 0.0f; }

    // Event callbacks
    void SetOnTouch(std::function<void(uint16_t pad)> cb) { _on_touch = cb; }
    void SetOnRelease(std::function<void(uint16_t pad)> cb) { _on_release = cb; }

    // State queries
    bool IsTouched(uint16_t pad) const { return (_state & (1 << pad)) != 0; }
    bool HasTouch() const { return _state > 0; }
    uint16_t State() const { return _state; }
    uint16_t OutOfRange() const { return _oor_state; }

    // Continuous pressure sensing (0.0 .. 1.0)
    float Pressure(uint16_t pad) const { return (pad < 12) ? _pressure[pad] : 0.0f; }
    float operator[](size_t pad) const { return Pressure(pad); }
    const std::array<float, 12>& Pressures() const { return _pressure; }

    // Continuous non-linear proximity / air capacitance sensing (0.0 .. 1.0)
    float Proximity(uint16_t pad) const { return (pad < 12) ? _proximity[pad] : 0.0f; }
    const std::array<float, 12>& Proximities() const { return _proximity; }

    // Raw 10-bit ADC data and deltas for proximity / air tracking
    uint16_t Filtered(uint16_t pad) const { return (pad < 12) ? _filtered[pad] : 0; }
    uint16_t Baseline(uint16_t pad) const { return (pad < 12) ? (_has_sw_baseline ? _sw_baseline[pad] : _baseline[pad]) : 0; }
    int32_t RawDelta(uint16_t pad) const { return (pad < 12) ? _raw_delta[pad] : 0; }

    // Baseline locking (freeze baseline tracking to eliminate hover decay)
    void LockBaseline(bool lock);
    bool IsBaselineLocked() const { return _baseline_locked; }
    void CaptureSoftwareBaseline(size_t samples = 16);

private:
    bool WriteRegister(uint8_t reg, uint8_t val);
    uint8_t ReadRegister(uint8_t reg);
    bool ReadBurst(uint8_t start_reg, uint8_t* buffer, uint16_t size);
    void RecoverBus();
    void ConfigureMpr121Registers();

    static constexpr uint8_t kMpr121Addr = 0x5A;

    daisy::I2CHandle _i2c;
    daisy::I2CHandle::Config _i2c_conf{};
    uint8_t _consecutive_errors{0};
    uint32_t _last_poll_ms{0};
    uint32_t _last_recovery_ms{0};
    uint16_t _state;
    uint16_t _oor_state;
    bool _exponential;
    bool _baseline_locked{false};
    bool _has_sw_baseline{false};
    bool _has_saved_afe{false};
    uint8_t _all_zero_count{0};
    uint16_t _all_idle_count{0};
    uint32_t _last_nudge_ms{0};

    std::array<uint8_t, 12>  _saved_cdc{};
    uint8_t                  _saved_cdt{0};

    std::array<uint16_t, 12> _filtered{};
    std::array<uint16_t, 12> _baseline{};
    std::array<uint16_t, 12> _sw_baseline{};
    std::array<int32_t, 12>   _raw_delta{};
    std::array<PadConfig, 12> _calib{};
    std::array<float, 12>     _pressure{};
    std::array<float, 12>     _proximity{};
    std::array<uint8_t, 12>  _debounce_cnt{};

    std::function<void(uint16_t pad)> _on_touch;
    std::function<void(uint16_t pad)> _on_release;
};

} // namespace synthux
