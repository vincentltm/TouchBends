#pragma once

#include "daisy_seed.h"
#include "surface/touch.h"
#include "surface/frame.h"
#include "surface/gestures.h"
#include "circuit/circuit.h"
#include "graph/signal_graph.h"
#if !RUN_PERF_TEST
#include "midi/midi_manager.h"
#endif
#include "dsp/itcm.h"
#include <cmath>

namespace bends {

class Instrument {
public:
    void Init(float sample_rate, Stereo* delay_buf, bool enable_usb_midi = true) {
        _sample_rate = sample_rate;
        _circuit.Init(sample_rate);
        _graph.Init(sample_rate, delay_buf);
#if !RUN_PERF_TEST
        if (enable_usb_midi) {
            _midi.Init();
        }
#else
        (void)enable_usb_midi;
#endif

        _tracker_p10.Init();

        _led_brightness = 0.0f;
        _led_pwm_phase = 0.0f;
        _led_flicker_rng = 0x471A9B25u;

        _cpu_avg = 0.0f;
        _cpu_peak = 0.0f;
    }

    // Called from main() while(1) loop (~1 kHz, runs I2C without blocking audio)
    void ProcessSurface(synthux::Touch& touch, bool new_scan) {
        (void)new_scan;
        _last_surface_tick = daisy::System::GetNow();
        ControlFrame& frame = _frame_buffer.BeginWrite();

        // Switches
        if (touch.switches().IsAUp()) {
            frame.osc_mode = 0;
        } else if (touch.switches().IsADown()) {
            frame.osc_mode = 2;
        } else {
            frame.osc_mode = 1;
        }

        frame.is_frozen = touch.switches().IsBUp();
        frame.is_latched = touch.switches().IsBDown();

        // Knobs & faders
        frame.s30_amp = touch.knobs().s30();
        frame.s31_flt = touch.knobs().s31();
        frame.s32_mix = touch.knobs().s32();
        frame.s33_clk = touch.knobs().s33();
        frame.s34_dly = touch.knobs().s34();
        frame.s35_fbk = touch.knobs().s35();
        frame.s36_osc = touch.knobs().s36();
        frame.s37_vcc = touch.knobs().s37();

        // Touch pads
        for (size_t i = 0; i < 12; ++i) {
            frame.pressures[i] = touch.pads().Pressure(i);
            frame.proximities[i] = touch.pads().Proximity(i);
        }

        // Tap gestures
        if (new_scan) {
            float v10 = 0.0f;
            _tracker_p10.Update(frame.pressures[NodeIn], v10);

            if (v10 > 0.0f) { _tap_count_p10++; _tap_vel_p10 = v10; }
        }

        frame.rub_speed_p10 = _tracker_p10.rub_speed;
        frame.tap_count_p10 = _tap_count_p10;
        frame.tap_vel_p10 = _tap_vel_p10;

#if !RUN_PERF_TEST
        _midi.Process(frame);
#endif

        frame.frame_index++;
        _frame_buffer.CommitWrite();
    }

    // Called from 750 Hz hardware audio DMA interrupt
    BENDS_ITCM void ProcessBlock(const float* in_l, const float* in_r,
                                 float* out_l, float* out_r,
                                 size_t size) {
        uint32_t cyc_start = DWT->CYCCNT;

        const ControlFrame& frame = _frame_buffer.ReadLatest();

        bool flush_req = _circuit.FlushRequested();
        if (flush_req && !_prev_flush) {
            _graph.Reset();
        }
        _prev_flush = flush_req;
        uint32_t t_m0 = DWT->CYCCNT;
        _graph.ServiceIncrementalClear();
        _graph.last_profile.clear = DWT->CYCCNT - t_m0;

        // Circuit solver & bend compilation (750 Hz block rate)
        t_m0 = DWT->CYCCNT;
        _circuit.Update(frame, _graph.AudioEnvelope(), _block_state, _graph.ClkBeat(), _graph.ClkRate(),
                        _graph.OscBaseFreq(), _graph.OscPhase(), _graph.OscRndStep());
        _graph.last_profile.circuit = DWT->CYCCNT - t_m0;

        // Audio DSP block render
        t_m0 = DWT->CYCCNT;
        _graph.PrepareBlock(_block_state, _circuit.Amounts(), frame);
        _graph.last_profile.prep = DWT->CYCCNT - t_m0;

        _graph.Render(in_l, in_r, out_l, out_r, size, _block_state, _circuit.Amounts(), _circuit.Matrix(), frame, _circuit.GetRail(), cyc_start);
        _graph.last_profile.total = DWT->CYCCNT - cyc_start;

        // Cycle budget metering (640,000 cycles for 64 samples @ 480 MHz)
        uint32_t cyc_end = DWT->CYCCNT;
        uint32_t elapsed = cyc_end - cyc_start;
        float block_cpu = static_cast<float>(elapsed) * (1.0f / 640000.0f);
        _cpu_avg += 0.05f * (block_cpu - _cpu_avg);
        if (block_cpu > _cpu_peak) {
            _cpu_peak = block_cpu;
        } else {
            _cpu_peak *= 0.999f;
        }

        // Buffer Underrun Detection: flags if audio block execution ever exceeds 640,000 cycles
        if (elapsed > 640000) {
            _underrun_count++;
            _max_overrun_cycles = std::max(_max_overrun_cycles, elapsed);
            _underrun_alarm_ticks = 1500; // Trigger visual alarm for 2 seconds (1500 blocks @ 750 Hz)
        }
    }

    // Flicker-free 120 Hz fixed-frequency PWM for Daisy Seed onboard LED
    void UpdateLedPwm(daisy::DaisySeed& hw) {
        // UI thread watchdog: if main() UI loop stops running for > 250ms, extinguish LED
        if (daisy::System::GetNow() - _last_surface_tick > 250) {
            hw.SetLed(false);
            return;
        }

        // Buffer Underrun Alarm: if an audio block exceeded 640,000 cycles, flash rapid 25 Hz warning strobe
        if (_underrun_alarm_ticks > 0) {
            _underrun_alarm_ticks--;
            bool alarm_strobe = ((_underrun_alarm_ticks & 15) < 8);
            hw.SetLed(alarm_strobe);
            return;
        }

        UpdateLedState();
        _led_pwm_phase += 120.0f / 750.0f;
        if (_led_pwm_phase >= 1.0f) _led_pwm_phase -= 1.0f;
        hw.SetLed(_led_brightness > _led_pwm_phase);
    }

    uint32_t UnderrunCount() const { return _underrun_count; }
    uint32_t MaxOverrunCycles() const { return _max_overrun_cycles; }
    float CpuPeak() const { return _cpu_peak; }
    float CpuAvg() const { return _cpu_avg; }

    void SetLedOverride(float b) { _led_override = b; }

    Circuit& GetCircuit() { return _circuit; }
    const Circuit& GetCircuit() const { return _circuit; }
    SignalGraph& GetGraph() { return _graph; }
    const SignalGraph& GetGraph() const { return _graph; }
    FrameBuffer& GetFrameBuffer() { return _frame_buffer; }
    const FrameBuffer& GetFrameBuffer() const { return _frame_buffer; }

private:
    void UpdateLedState() {
        if (_led_override >= 0.0f) {
            _led_brightness = _led_override;
            return;
        }

        float vcc = _block_state.vcc;
        float cap_v = _circuit.GetRail().CapVoltage();
        float starve = _block_state.starve;

        // User requirement: doesn't go off early, and goes darker before black
        if (vcc < 0.015f) {
            _led_brightness = 0.0f;
            return;
        }

        // Base brightness tracks rail voltage with natural quadratic curve
        float target_b = 0.015f + 0.84f * (vcc * vcc);

        // Smooth taper down into darkness below 0.08V right before black
        if (vcc < 0.08f) {
            float fade = (vcc - 0.015f) * (1.0f / (0.08f - 0.015f));
            target_b *= (fade * fade);
        }

        // Proportional touch load dip: dims up to 25% under heavy current, never blacking out at mid-rail
        float touch_load = _circuit.GetContacts().TotalCurrentLoad();
        float load_dip = std::min(0.25f, touch_load * (0.05f + starve * 0.15f));
        target_b *= (1.0f - load_dip);

        // Clock tempo beat flash
        if (_graph.ClkBeat()) {
            target_b = std::min(1.0f, target_b + 0.18f * vcc);
        }

        // Brownout flicker when rail drops below 0.22V
        if (cap_v < 0.22f && cap_v > 0.02f) {
            _led_flicker_rng = _led_flicker_rng * 1664525u + 1013904223u;
            if ((_led_flicker_rng >> 28) < 5) target_b *= 0.25f;
        }

        _led_brightness += 0.15f * (target_b - _led_brightness);
    }

    float _sample_rate{48000.0f};

    Circuit         _circuit;
    SignalGraph     _graph;
    BlockState      _block_state;
    FrameBuffer     _frame_buffer;

    PadGestureTracker _tracker_p10;

    uint16_t _tap_count_p10{0};
    float    _tap_vel_p10{0.0f};

    float    _cpu_avg{0.0f};
    float    _cpu_peak{0.0f};
    uint32_t _underrun_count{0};
    uint32_t _max_overrun_cycles{0};
    uint16_t _underrun_alarm_ticks{0};
    bool     _prev_flush{false};

    float _led_brightness{0.0f};
    float _led_pwm_phase{0.0f};
    float _led_override{-1.0f};
    uint32_t _led_flicker_rng{0x471A9B25u};

#if !RUN_PERF_TEST
    MidiManager _midi;
#endif
    uint32_t    _last_surface_tick{0};
};

} // namespace bends
