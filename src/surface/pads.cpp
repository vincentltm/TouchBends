#include "pads.h"
#include <cmath>

using namespace synthux;
using namespace daisy;

bool Pads::WriteRegister(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return (_i2c.TransmitBlocking(kMpr121Addr, buf, 2, 2) == I2CHandle::Result::OK);
}

uint8_t Pads::ReadRegister(uint8_t reg) {
    uint8_t val = 0;
    if (_i2c.TransmitBlocking(kMpr121Addr, &reg, 1, 2) == I2CHandle::Result::OK) {
        _i2c.ReceiveBlocking(kMpr121Addr, &val, 1, 2);
    }
    return val;
}

void Pads::ConfigureMpr121Registers() {
    // Soft Reset
    if (!WriteRegister(0x80, 0x63)) {
        return;
    }
    daisy::System::Delay(3);

    // Stop Mode (ECR = 0x00) for register configuration
    if (!WriteRegister(0x5E, 0x00)) return;

    // Touch & release thresholds
    for (uint8_t i = 0; i < 12; i++) {
        uint8_t t_thresh = (i == 1) ? 16 : 12;
        uint8_t r_thresh = (i == 1) ? 10 : 7;
        WriteRegister(0x41 + i * 2, t_thresh);
        WriteRegister(0x42 + i * 2, r_thresh);
    }

    // Baseline filter configuration (NXP AN3891)
    WriteRegister(0x2B, 0x01); // MHDR
    WriteRegister(0x2C, 0x01); // NHDR
    WriteRegister(0x2D, 0x0E); // NCLR
    WriteRegister(0x2E, 0x00); // FDLR

    WriteRegister(0x2F, 0x01); // MHDF
    WriteRegister(0x30, 0x01); // NHDF
    WriteRegister(0x31, 0x10); // NCLF
    WriteRegister(0x32, 0x04); // FDLF

    WriteRegister(0x33, 0x00); // NHDT
    WriteRegister(0x34, 0x00); // NCLT
    WriteRegister(0x35, 0x00); // FDLT

    // Debounce
    WriteRegister(0x5B, 0x11);

    // AFE configuration
    WriteRegister(0x5C, 0x50);
    WriteRegister(0x5D, 0x61);

    // Auto-configuration registers
    WriteRegister(0x7D, 200); // USL
    WriteRegister(0x7E, 130); // LSL
    WriteRegister(0x7F, 180); // TL

    if (_has_saved_afe) {
        // Restore calibrated hardware charge currents and times so electrodes never have 0 charge
        for (uint8_t i = 0; i < 12; i++) {
            WriteRegister(0x5F + i, _saved_cdc[i]);
        }
        WriteRegister(0x6C, _saved_cdt);
        WriteRegister(0x7B, 0x00); // Auto-reconfiguration disabled
        WriteRegister(0x7C, 0x00);

        if (_baseline_locked && _has_sw_baseline) {
            for (uint8_t i = 0; i < 12; i++) {
                WriteRegister(0x1E + i, static_cast<uint8_t>(_sw_baseline[i] >> 2));
            }
            WriteRegister(0x5E, 0x0C); // CL = 00, 12 electrodes
        } else {
            WriteRegister(0x5E, 0x8C); // CL = 10, 12 electrodes
        }
    } else {
        // Initial bootup calibration: run auto-configuration engine to find optimal CDC/CDT
        WriteRegister(0x7B, 0x4B);
        WriteRegister(0x7C, 0x00);
        WriteRegister(0x5E, 0x8C); // CL = 10, 12 electrodes
    }
}

void Pads::RecoverBus() {
    uint32_t now = daisy::System::GetNow();
    if (now - _last_recovery_ms < 150) {
        return; // Rate limit recoveries to once per 150ms to keep main UI thread fast
    }
    _last_recovery_ms = now;

    // Reset I2C1 peripheral
    __HAL_RCC_I2C1_FORCE_RESET();
    daisy::System::DelayUs(10);
    __HAL_RCC_I2C1_RELEASE_RESET();
    __HAL_RCC_I2C1_CLK_ENABLE();

    // Bit-bang 9 clock cycles to release stuck slave on SDA
    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    for (int k = 0; k < 9; ++k) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        daisy::System::DelayUs(5);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        daisy::System::DelayUs(5);
    }
    // Generate I2C STOP condition
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    daisy::System::DelayUs(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    daisy::System::DelayUs(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    daisy::System::DelayUs(5);

    // Restore pins to I2C alternate function
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    // Re-initialize I2C peripheral
    _i2c.Init(_i2c_conf);

    // If MPR121 was knocked out of Run Mode or lost power, restore registers
    uint8_t ecr = ReadRegister(0x5E);
    if (ecr != 0x0C && ecr != 0x8C) {
        ConfigureMpr121Registers();
    }
    _consecutive_errors = 0;
}

bool Pads::ReadBurst(uint8_t start_reg, uint8_t* buffer, uint16_t size) {
    if (_i2c.TransmitBlocking(kMpr121Addr, &start_reg, 1, 8) != I2CHandle::Result::OK) {
        _consecutive_errors++;
        if (_consecutive_errors >= 6) {
            RecoverBus();
        }
        return false;
    }
    if (_i2c.ReceiveBlocking(kMpr121Addr, buffer, size, 12) != I2CHandle::Result::OK) {
        _consecutive_errors++;
        if (_consecutive_errors >= 6) {
            RecoverBus();
        }
        return false;
    }
    _consecutive_errors = 0;
    return true;
}

void Pads::Init(DaisySeed& hw) {
    // 1. Initialize I2C1 (PB8 = SCL, PB9 = SDA, 400 kHz Fast Mode)
    _i2c_conf.mode = I2CHandle::Config::Mode::I2C_MASTER;
    _i2c_conf.periph = I2CHandle::Config::Peripheral::I2C_1;
    _i2c_conf.speed = I2CHandle::Config::Speed::I2C_400KHZ;
    _i2c_conf.pin_config.scl = Pin(PORTB, 8);
    _i2c_conf.pin_config.sda = Pin(PORTB, 9);
    _i2c.Init(_i2c_conf);

    ConfigureMpr121Registers();
    System::Delay(80);
}

void Pads::Recalibrate() {
    // Put MPR121 in Stop Mode
    WriteRegister(0x5E, 0x00);
    System::Delay(5);

    // Re-enter Run Mode with Autoconfig enabled
    WriteRegister(0x5E, 0x8C);
    System::Delay(80);

    if (_baseline_locked) {
        CaptureSoftwareBaseline(16);
        WriteRegister(0x5E, 0x0C);
    }
}

void Pads::LockBaseline(bool lock) {
    if (lock) {
        if (!_has_sw_baseline) {
            CaptureSoftwareBaseline(16);
        }
        // Save calibrated CDC and CDT from startup autoconfiguration so recoveries never leave electrodes uncharged
        for (uint8_t i = 0; i < 12; i++) {
            _saved_cdc[i] = ReadRegister(0x5F + i);
        }
        _saved_cdt = ReadRegister(0x6C);
        _has_saved_afe = true;

        // Disable Auto-Reconfiguration (0x7B = 0x00) so touching pads cannot trigger ACF halts
        WriteRegister(0x7B, 0x00);
        // Put in Stop Mode to write baseline registers 0x1E..0x29
        WriteRegister(0x5E, 0x00);
        for (uint8_t i = 0; i < 12; i++) {
            WriteRegister(0x1E + i, static_cast<uint8_t>(_sw_baseline[i] >> 2));
        }
        // Write CL = 00 to ECR (0x5E) to disable hardware baseline tracking: (0b00 << 6) | 12 = 0x0C
        WriteRegister(0x5E, 0x0C);
        _baseline_locked = true;
    } else {
        WriteRegister(0x7B, 0x4B);
        // Write CL = 10 to ECR (0x5E) to resume normal baseline tracking: (0b10 << 6) | 12 = 0x8C
        WriteRegister(0x5E, 0x8C);
        _baseline_locked = false;
        _has_sw_baseline = false;
        _has_saved_afe = false;
    }
}

void Pads::CaptureSoftwareBaseline(size_t samples) {
    if (samples == 0) samples = 1;
    std::array<uint32_t, 12> sum{};
    for (size_t s = 0; s < samples; s++) {
        uint8_t raw[28];
        if (ReadBurst(0x00, raw, 28)) {
            for (size_t i = 0; i < 12; i++) {
                uint16_t filt = (static_cast<uint16_t>(raw[4 + i * 2 + 1] & 0x03) << 8) | raw[4 + i * 2];
                sum[i] += filt;
            }
        }
        System::Delay(2);
    }
    for (size_t i = 0; i < 12; i++) {
        _sw_baseline[i] = static_cast<uint16_t>((sum[i] + (samples / 2)) / samples);
        _baseline[i] = _sw_baseline[i];
    }
    _has_sw_baseline = true;
}

bool Pads::Process() {
    uint32_t now = daisy::System::GetNow();
    if (now - _last_poll_ms < 3) {
        return false; // Cadence to ~250-300 Hz: matches MPR121 scan rate and prevents I2C bus flooding
    }
    _last_poll_ms = now;

    // Read 28 bytes: Touch status (0-1), OOR (2-3), and 12 Filtered channels (4-27)
    uint8_t raw[28];
    if (!ReadBurst(0x00, raw, 28)) {
        for (size_t i = 0; i < 12; ++i) {
            _pressure[i] *= 0.5f;
            if (_pressure[i] < 0.01f) _pressure[i] = 0.0f;
        }
        _state = 0;
        return false;
    }

    uint16_t raw_state = (static_cast<uint16_t>(raw[1] & 0x0F) << 8) | raw[0];
    _oor_state = (static_cast<uint16_t>(raw[3] & 0x0F) << 8) | raw[2];

    // If MPR121 reset into Stop Mode from ESD/transients, all filtered ADC registers read 0.
    // Instantly wake it back into Run Mode so touch never dies:
    bool all_zero = true;
    for (size_t b = 4; b < 28; ++b) {
        if (raw[b] != 0) {
            all_zero = false;
            break;
        }
    }
    if (all_zero) {
        _all_zero_count++;
        // Decay pressures safely so missing sensor data never latches shorted nodes
        for (size_t i = 0; i < 12; ++i) {
            _pressure[i] *= 0.5f;
            if (_pressure[i] < 0.01f) _pressure[i] = 0.0f;
            _proximity[i] *= 0.5f;
            if (_proximity[i] < 0.01f) _proximity[i] = 0.0f;
        }
        _state = 0;
        if (_all_zero_count >= 3) {
            ConfigureMpr121Registers();
            _all_zero_count = 0;
        }
        return false;
    }
    _all_zero_count = 0;

    for (uint16_t i = 0; i < 12; i++) {
        uint16_t mask = 1 << i;

        // 10-bit Filtered Capacitance from register 0x04 + i*2
        uint16_t filt = (static_cast<uint16_t>(raw[4 + i * 2 + 1] & 0x03) << 8) | raw[4 + i * 2];

        // 10-bit Baseline Capacitance
        uint16_t base = _has_sw_baseline ? _sw_baseline[i] : _baseline[i];
        if (!_has_sw_baseline) {
            _baseline[i] = filt;
            base = filt;
        }

        // Store raw state
        _filtered[i] = filt;
        _baseline[i] = base;

        // Delta: positive when touched (capacitance increases, ADC voltage drops below baseline)
        int32_t delta = static_cast<int32_t>(base) - static_cast<int32_t>(filt);
        if (delta < 0) delta = 0;
        _raw_delta[i] = delta;

        const auto& cal = _calib[i];
        bool was_touched = (_state & mask) != 0;
        bool hw_touched = (raw_state & mask) != 0;
        // Touch detection with release hysteresis
        int32_t min_d = static_cast<int32_t>(cal.min_delta);
        int32_t rel_d = static_cast<int32_t>(cal.rel_delta);
        bool raw_touched = was_touched ? ((hw_touched && (delta >= rel_d)) || (delta >= rel_d + 4))
                                       : ((hw_touched && (delta >= min_d)) || (delta >= min_d + 4));
        bool state_changed = false;

        // Touch response with 2-scan release debounce
        if (raw_touched != was_touched) {
            if (raw_touched) {
                _state |= mask;
                state_changed = true;
                _debounce_cnt[i] = 0;
            } else {
                _debounce_cnt[i]++;
                if (_debounce_cnt[i] >= 2) {
                    _debounce_cnt[i] = 0;
                    _state &= ~mask;
                    state_changed = true;
                }
            }
        } else {
            _debounce_cnt[i] = 0;
        }

        // Proximity and pressure calculation
        float target_p = 0.0f;
        float target_prox = 0.0f;

        if (delta >= min_d) {
            target_prox = 1.0f;
            float eff_span = cal.max_delta - cal.min_delta;
            if (eff_span < 20.0f) eff_span = 20.0f;
            float touch_norm = static_cast<float>(delta - min_d) / eff_span;
            if (touch_norm > 1.0f) touch_norm = 1.0f;
            if (touch_norm < 0.0f) touch_norm = 0.0f;

            float curved = touch_norm * (0.42f * touch_norm + 0.58f);
            target_p = cal.floor_p + curved * (1.0f - cal.floor_p);
        } else if (delta > 2) {
            target_p = 0.0f;
            float active_d = static_cast<float>(delta - 2);
            float hover_span = static_cast<float>(min_d - 2);
            if (hover_span < 1.0f) hover_span = 1.0f;
            float norm_prox = active_d / hover_span;
            if (norm_prox > 1.0f) norm_prox = 1.0f;
            if (norm_prox < 0.0f) norm_prox = 0.0f;
            target_prox = norm_prox * norm_prox * (0.70f + 0.30f * norm_prox);
        }

        _proximity[i] += (target_prox - _proximity[i]) * 0.40f;
        if (_proximity[i] < 0.002f) _proximity[i] = 0.0f;

        if (state_changed && raw_touched) {
            _pressure[i] = target_p;
            if (_on_touch) _on_touch(i);
        } else {
            _pressure[i] += (target_p - _pressure[i]) * 0.40f;
            if (_pressure[i] < 0.001f) _pressure[i] = 0.0f;
        }

        if (state_changed && !raw_touched) {
            if (_on_release) _on_release(i);
        }
    }

    // Baseline tracking when all pads are idle
    if (_state == 0 && raw_state == 0) {
        if (_all_idle_count < 30) {
            _all_idle_count++;
        } else {
            if (now - _last_nudge_ms > 250) {
                _last_nudge_ms = now;
                for (size_t i = 0; i < 12; ++i) {
                    if (_has_sw_baseline && _raw_delta[i] >= 1 && _raw_delta[i] <= 5) {
                        _sw_baseline[i]--;
                    }
                }
            }
        }
    } else {
        _all_idle_count = 0;
    }
    return true;
}

