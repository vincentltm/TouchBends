#pragma once

#include "daisy_seed.h"
#include "app/instrument.h"
#include "stm32h7xx.h"
#include <cmath>
#include <algorithm>
#include <cstdio>

namespace bends {

using namespace daisy;

inline void RunOnDevicePerformanceCheck(DaisySeed& hw, Instrument& instrument, Stereo* g_sdram_delay_buffer) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    hw.StartLog(false);

    hw.usb_handle.SetReceiveCallback([](uint8_t* buff, uint32_t* len) {
        if (len && *len > 0) {
            for (uint32_t i = 0; i < *len; ++i) {
                if (buff[i] == 'b' || buff[i] == 'B' || buff[i] == 'd' || buff[i] == 'D') {
                    System::ResetToBootloader(System::BootloaderMode::STM);
                }
            }
        }
    }, UsbHandle::UsbPeriph::FS_INTERNAL);

    // Initial wait to let host enumerate USB CDC
    for (int i = 0; i < 30; ++i) {
        hw.SetLed(i % 2 == 0);
        System::Delay(100);
    }

    instrument.Init(hw.AudioSampleRate(), g_sdram_delay_buffer, false);

    // 4-Note Loud Chord audio stimulus (C3, E3, G3, B3 @ 1.4V peak)
    alignas(32) static float in_l[64] = {0.0f};
    alignas(32) static float in_r[64] = {0.0f};
    alignas(32) static float out_l[64] = {0.0f};
    alignas(32) static float out_r[64] = {0.0f};
    for (int i = 0; i < 64; ++i) {
        float t = static_cast<float>(i) * (1.0f / 48000.0f);
        float s = 0.35f * FastSin(2.0f * 3.14159265f * 130.81f * t)
                + 0.35f * FastSin(2.0f * 3.14159265f * 164.81f * t)
                + 0.35f * FastSin(2.0f * 3.14159265f * 196.00f * t)
                + 0.35f * FastSin(2.0f * 3.14159265f * 246.94f * t);
        in_l[i] = s;
        in_r[i] = s;
    }

    struct PerfResult {
        const char* name;
        uint32_t max_cyc;
        uint32_t avg_cyc;
        int max_pct;
        bool has_nan;
        bool has_click;
    };

    uint32_t worst_cyc = 0;
    const char* worst_name = "None";
    RenderProfile worst_prof{};
    uint32_t total_underruns_in_round = 0;
    uint32_t total_clicks_in_round = 0;

    auto RunTest = [&](const char* name, const ControlFrame& frame, int num_blocks = 20) __attribute__((noinline)) -> PerfResult {
        uint32_t cyc_max = 0;
        uint64_t cyc_sum = 0;
        bool has_nan = false;
        bool has_click = false;
        float prev_l = 0.0f;
        float prev_r = 0.0f;
        float max_delta = 0.0f;

        for (int b = 0; b < num_blocks; ++b) {
            ControlFrame& wf = instrument.GetFrameBuffer().BeginWrite();
            wf = frame;
            wf.frame_index = b;
            instrument.GetFrameBuffer().CommitWrite();

            uint32_t t0 = DWT->CYCCNT;
            instrument.ProcessBlock(in_l, in_r, out_l, out_r, 64);
            uint32_t dt = DWT->CYCCNT - t0;

            if (dt > cyc_max) cyc_max = dt;
            cyc_sum += dt;

            if (dt > worst_cyc) {
                worst_cyc = dt;
                worst_name = name;
                worst_prof = instrument.GetGraph().last_profile;
            }

            for (int s = 0; s < 64; ++s) {
                if (isnan(out_l[s]) || isnan(out_r[s]) || isinf(out_l[s]) || isinf(out_r[s])) {
                    has_nan = true;
                }
                if (b == 0 && s == 0) {
                    prev_l = out_l[0];
                    prev_r = out_r[0];
                    continue;
                }
                float dl = std::abs(out_l[s] - prev_l);
                float dr = std::abs(out_r[s] - prev_r);
                if (dl > max_delta) max_delta = dl;
                if (dr > max_delta) max_delta = dr;
                prev_l = out_l[s];
                prev_r = out_r[s];
            }
        }

        if (max_delta > 1.45f) {
            has_click = true;
            total_clicks_in_round++;
        }

        uint32_t cyc_avg = static_cast<uint32_t>(cyc_sum / num_blocks);
        int max_pct = static_cast<int>((cyc_max * 100) / 640000);
        if (max_pct >= 100) {
            total_underruns_in_round++;
        }

        const char* status = "PASS";
        if (max_pct >= 100) status = "OVERRUN!";
        else if (has_nan) status = "NAN!";
        else if (has_click) status = "CLICK!";

        hw.PrintLine("  %-42s | %6lu cyc (%2d%%) | %6lu cyc | %s",
                     name, cyc_max, max_pct, cyc_avg, status);
        System::Delay(2);

        return { name, cyc_max, cyc_avg, max_pct, has_nan, has_click };
    };

    auto RunTestSwept = [&](const char* name, const ControlFrame& frame, int num_blocks = 24) __attribute__((noinline)) -> PerfResult {
        uint32_t cyc_max = 0;
        uint64_t cyc_sum = 0;
        bool has_nan = false;
        bool has_click = false;
        float prev_l = 0.0f;
        float prev_r = 0.0f;
        float max_delta = 0.0f;

        for (int b = 0; b < num_blocks; ++b) {
            ControlFrame wf = frame;
            float t = static_cast<float>(b) / static_cast<float>(num_blocks - 1);
            // Continuous knob ramping to detect zipper noise and movement artifacts
            wf.s34_dly = 0.05f + 0.90f * t;
            wf.s31_flt = 0.10f + 0.80f * t;
            wf.s30_amp = 0.20f + 0.60f * (1.0f - t);
            wf.frame_index = b;

            ControlFrame& dst = instrument.GetFrameBuffer().BeginWrite();
            dst = wf;
            instrument.GetFrameBuffer().CommitWrite();

            uint32_t t0 = DWT->CYCCNT;
            instrument.ProcessBlock(in_l, in_r, out_l, out_r, 64);
            uint32_t dt = DWT->CYCCNT - t0;

            if (dt > cyc_max) cyc_max = dt;
            cyc_sum += dt;

            if (dt > worst_cyc) {
                worst_cyc = dt;
                worst_name = name;
                worst_prof = instrument.GetGraph().last_profile;
            }

            for (int s = 0; s < 64; ++s) {
                if (isnan(out_l[s]) || isnan(out_r[s]) || isinf(out_l[s]) || isinf(out_r[s])) {
                    has_nan = true;
                }
                if (b == 0 && s == 0) {
                    prev_l = out_l[0];
                    prev_r = out_r[0];
                    continue;
                }
                float dl = std::abs(out_l[s] - prev_l);
                float dr = std::abs(out_r[s] - prev_r);
                if (dl > max_delta) max_delta = dl;
                if (dr > max_delta) max_delta = dr;
                prev_l = out_l[s];
                prev_r = out_r[s];
            }
        }

        if (max_delta > 1.45f) {
            has_click = true;
            total_clicks_in_round++;
        }

        uint32_t cyc_avg = static_cast<uint32_t>(cyc_sum / num_blocks);
        int max_pct = static_cast<int>((cyc_max * 100) / 640000);
        if (max_pct >= 100) {
            total_underruns_in_round++;
        }

        const char* status = "PASS";
        if (max_pct >= 100) status = "OVERRUN!";
        else if (has_nan) status = "NAN!";
        else if (has_click) status = "CLICK!";

        hw.PrintLine("  %-42s | %6lu cyc (%2d%%) | %6lu cyc | %s",
                     name, cyc_max, max_pct, cyc_avg, status);
        System::Delay(2);

        return { name, cyc_max, cyc_avg, max_pct, has_nan, has_click };
    };

    ControlFrame fb{};
    fb.osc_mode = 1;
    fb.s30_amp = 0.5f;
    fb.s31_flt = 0.5f;
    fb.s32_mix = 0.5f;
    fb.s33_clk = 0.5f;
    fb.s34_dly = 0.5f;
    fb.s35_fbk = 0.5f;
    fb.s36_osc = 0.5f;
    fb.s37_vcc = 1.0f;
    for (int i = 0; i < 12; ++i) fb.pressures[i] = 0.0f;

    static const char* const kPadNames[12] = {
        "MIX", "RST", "CLK", "OSC", "FLT", "GND",
        "DLY", "VCC", "AMP", "FBK", "IN",  "OUT"
    };

    uint32_t round = 0;
    while (1) {
        round++;
        worst_cyc = 0;
        worst_name = "None";
        total_underruns_in_round = 0;
        total_clicks_in_round = 0;

        hw.PrintLine("\r\n=========================================================================");
        hw.PrintLine("  BENDS ON-DEVICE EXHAUSTIVE 66-PAIR PERFORMANCE TEST - ROUND #%lu", round);
        hw.PrintLine("=========================================================================");
        hw.PrintLine("  Scenario                                   |   Peak Cycles    |   Avg Cycles | Status");
        hw.PrintLine("  -------------------------------------------+------------------+--------------+-------");
        System::Delay(10);

        // ---------------------------------------------------------------------
        // 1. Baseline & Knobs / Faders
        // ---------------------------------------------------------------------
        {
            ControlFrame f = fb;
            RunTest("Baseline: 50% All Knobs Clean", f);
        }
        {
            ControlFrame f = fb; f.s32_mix = 0.00f;
            RunTest("Knob: Mix 0% Dry Only", f);
        }
        {
            ControlFrame f = fb; f.s32_mix = 1.00f;
            RunTest("Knob: Mix 100% Wet Only", f);
        }
        {
            ControlFrame f = fb; f.s30_amp = 1.0f; f.s35_fbk = 0.85f;
            RunTest("Knob: Cranked Amp & Fbk", f);
        }
        {
            ControlFrame f = fb; f.s33_clk = 0.00f; f.s34_dly = 1.0f;
            RunTest("Knob: Min Clock / Max Delay", f);
        }
        {
            ControlFrame f = fb; f.s33_clk = 1.00f; f.s34_dly = 0.005f;
            RunTest("Knob: Max Overclock / Karplus Delay", f);
        }

        // ---------------------------------------------------------------------
        // 2. Starvation Voltage Sweeps
        // ---------------------------------------------------------------------
        {
            ControlFrame f = fb; f.s37_vcc = 0.50f;
            RunTest("Starve: VCC=50%", f);
        }
        {
            ControlFrame f = fb; f.s37_vcc = 0.20f;
            RunTest("Starve: VCC=20%", f);
        }
        {
            ControlFrame f = fb; f.s37_vcc = 0.05f;
            RunTest("Starve: VCC=5% Deep Collapse", f);
        }

        // ---------------------------------------------------------------------
        // 3. Single-Pad Isolations (P00 through P11)
        // ---------------------------------------------------------------------
        for (int p = 0; p < 12; ++p) {
            ControlFrame f = fb;
            f.pressures[p] = 1.0f;
            char buf[48];
            snprintf(buf, sizeof(buf), "Solo Pad: %s", kPadNames[p]);
            RunTest(buf, f, 12);
        }

        // ---------------------------------------------------------------------
        // 4. Full Exhaustive 66-Pair Matrix Combinations (All 0 <= i < j < 12)
        // ---------------------------------------------------------------------
        char label[48];
        for (int i = 0; i < 12; ++i) {
            for (int j = i + 1; j < 12; ++j) {
                // Scenario A: Nominal
                {
                    ControlFrame f = fb;
                    f.pressures[i] = 1.0f;
                    f.pressures[j] = 1.0f;
                    snprintf(label, sizeof(label), "Pair %s-%s (Nominal)", kPadNames[i], kPadNames[j]);
                    RunTest(label, f, 16);
                }
                // Scenario B: Starved VCC = 0.20V
                {
                    ControlFrame f = fb;
                    f.pressures[i] = 1.0f;
                    f.pressures[j] = 1.0f;
                    f.s37_vcc = 0.20f;
                    snprintf(label, sizeof(label), "Pair %s-%s (Starved)", kPadNames[i], kPadNames[j]);
                    RunTest(label, f, 16);
                }
                // Scenario C: High Drive & Feedback
                {
                    ControlFrame f = fb;
                    f.pressures[i] = 1.0f;
                    f.pressures[j] = 1.0f;
                    f.s30_amp = 1.0f;
                    f.s35_fbk = 0.85f;
                    snprintf(label, sizeof(label), "Pair %s-%s (HighDrive)", kPadNames[i], kPadNames[j]);
                    RunTest(label, f, 16);
                }
                // Scenario D: Dynamic Swept Knobs (Delay, Filter, Amp moving across blocks)
                {
                    ControlFrame f = fb;
                    f.pressures[i] = 1.0f;
                    f.pressures[j] = 1.0f;
                    snprintf(label, sizeof(label), "Pair %s-%s (SweptKnobs)", kPadNames[i], kPadNames[j]);
                    RunTestSwept(label, f, 20);
                }
            }
        }

        // ---------------------------------------------------------------------
        // 5. Complex Multi-Touch Topologies (Triads, Quartets, Clusters)
        // ---------------------------------------------------------------------
        {
            ControlFrame f = fb;
            f.pressures[NodeOsc] = 1.0f; f.pressures[NodeClk] = 1.0f; f.pressures[NodeDly] = 1.0f;
            RunTest("Triad: Lissajous Cloud (OSC+CLK+DLY)", f);
        }
        {
            ControlFrame f = fb;
            f.pressures[NodeFlt] = 1.0f; f.pressures[NodeDly] = 1.0f; f.pressures[NodeFbk] = 1.0f;
            RunTest("Triad: Karplus Resonator (FLT+DLY+FBK)", f);
        }
        {
            ControlFrame f = fb;
            f.pressures[NodeVcc] = 1.0f; f.pressures[NodeClk] = 1.0f; f.pressures[NodeFbk] = 1.0f;
            RunTest("Triad: Clock Siren (VCC+CLK+FBK)", f);
        }
        {
            ControlFrame f = fb;
            f.pressures[NodeAmp] = 1.0f; f.pressures[NodeFlt] = 1.0f; f.pressures[NodeFbk] = 1.0f;
            RunTest("Triad: Intermod Loop (AMP+FLT+FBK)", f);
        }
        {
            ControlFrame f = fb;
            f.pressures[NodeOsc] = 1.0f; f.pressures[NodeGnd] = 1.0f; f.pressures[NodeVcc] = 1.0f;
            RunTest("Triad: Voltage Pump (OSC+GND+VCC)", f);
        }
        {
            ControlFrame f = fb; f.osc_mode = 2;
            f.pressures[NodeClk] = 1.0f; f.pressures[NodeOsc] = 1.0f; f.pressures[NodeFlt] = 1.0f;
            RunTest("Triad: Rungler (CLK+OSC+FLT Rnd)", f);
        }
        {
            ControlFrame f = fb;
            f.pressures[NodeOsc] = 1.0f; f.pressures[NodeAmp] = 1.0f; f.pressures[NodeFlt] = 1.0f;
            RunTest("Triad: Ping Drum (OSC+AMP+FLT)", f);
        }
        {
            ControlFrame f = fb;
            f.pressures[NodeClk] = 1.0f; f.pressures[NodeDly] = 1.0f; f.pressures[NodeRst] = 1.0f;
            RunTest("Triad: Pendulum Slice (CLK+DLY+RST)", f);
        }
        {
            ControlFrame f = fb;
            f.pressures[NodeClk] = 1.0f; f.pressures[NodeOsc] = 1.0f;
            f.pressures[NodeAmp] = 1.0f; f.pressures[NodeFlt] = 1.0f;
            RunTest("Quartet: Voice Quartet (CLK+OSC+AMP+FLT)", f);
        }
        {
            ControlFrame f = fb;
            f.pressures[NodeVcc] = 1.0f; f.pressures[NodeClk] = 1.0f;
            f.pressures[NodeAmp] = 1.0f; f.pressures[NodeFlt] = 1.0f;
            RunTest("Quartet: Siren Intermod (VCC+CLK+AMP+FLT)", f);
        }
        {
            ControlFrame f = fb;
            for (int p = 0; p < 6; ++p) f.pressures[p] = 1.0f;
            RunTest("Cluster: 6-Pad Hand Span (MIX-GND)", f);
        }
        {
            // Core Audio Synthesis Engine (CLK+OSC+FLT+DLY+AMP+FBK)
            ControlFrame f = fb;
            f.pressures[NodeClk] = 1.0f; f.pressures[NodeOsc] = 1.0f;
            f.pressures[NodeFlt] = 1.0f; f.pressures[NodeDly] = 1.0f;
            f.pressures[NodeAmp] = 1.0f; f.pressures[NodeFbk] = 1.0f;
            RunTest("Cluster: Sound Core (CLK+OSC+FLT+DLY+AMP+FBK)", f);
        }
        {
            // Bottom-Row Hand Span (DLY+VCC+AMP+FBK+IN+OUT)
            ControlFrame f = fb;
            f.pressures[NodeDly] = 1.0f; f.pressures[NodeVcc] = 1.0f;
            f.pressures[NodeAmp] = 1.0f; f.pressures[NodeFbk] = 1.0f;
            f.pressures[NodeIn]  = 1.0f; f.pressures[NodeOut] = 1.0f;
            RunTest("Cluster: Bottom-Row 6-Pad Hand Span", f);
        }
        {
            // High Feedback Stress (FBK+DLY+AMP+OUT+IN+VCC+FLT)
            ControlFrame f = fb;
            f.pressures[NodeFbk] = 1.0f; f.pressures[NodeDly] = 1.0f;
            f.pressures[NodeAmp] = 1.0f; f.pressures[NodeOut] = 1.0f;
            f.pressures[NodeIn]  = 1.0f; f.pressures[NodeVcc] = 1.0f;
            f.pressures[NodeFlt] = 1.0f;
            f.s35_fbk = 0.85f; f.s30_amp = 1.0f;
            RunTest("Cluster: High Feedback (7 pads)", f);
        }
        {
            // Starved Voltage Stress (OSC+FLT+AMP+DLY+FBK+VCC+GND @ 0.2V)
            ControlFrame f = fb;
            f.pressures[NodeOsc] = 1.0f; f.pressures[NodeFlt] = 1.0f;
            f.pressures[NodeAmp] = 1.0f; f.pressures[NodeDly] = 1.0f;
            f.pressures[NodeFbk] = 1.0f; f.pressures[NodeVcc] = 1.0f;
            f.pressures[NodeGnd] = 1.0f;
            f.s37_vcc = 0.20f;
            RunTest("Cluster: Starved Voltage (7 pads)", f);
        }
        {
            // Right-Side Palm (FLT+GND+DLY+VCC+AMP+FBK+IN+OUT)
            ControlFrame f = fb;
            f.pressures[NodeFlt] = 1.0f; f.pressures[NodeGnd] = 1.0f;
            f.pressures[NodeDly] = 1.0f; f.pressures[NodeVcc] = 1.0f;
            f.pressures[NodeAmp] = 1.0f; f.pressures[NodeFbk] = 1.0f;
            f.pressures[NodeIn]  = 1.0f; f.pressures[NodeOut] = 1.0f;
            RunTest("Cluster: Right-Side Palm (8 pads)", f);
        }
        {
            // Rungler Multi-Touch (ModeRnd with CLK+OSC+FLT+DLY+FBK+AMP+VCC)
            ControlFrame f = fb; f.osc_mode = 2;
            f.pressures[NodeClk] = 1.0f; f.pressures[NodeOsc] = 1.0f;
            f.pressures[NodeFlt] = 1.0f; f.pressures[NodeDly] = 1.0f;
            f.pressures[NodeFbk] = 1.0f; f.pressures[NodeAmp] = 1.0f;
            f.pressures[NodeVcc] = 1.0f;
            RunTest("Cluster: Rungler Multi-Touch (7 pads)", f);
        }
        {
            // 8-Pad Touch Cluster (Pads 0..7)
            ControlFrame f = fb;
            for (int p = 0; p < 8; ++p) f.pressures[p] = 1.0f;
            RunTest("Cluster: 8-Pad Touch Cluster", f);
        }
        {
            // 10-Pad Palm (all pads except GND and RST)
            ControlFrame f = fb;
            for (int p = 0; p < 12; ++p) {
                if (p != NodeGnd && p != NodeRst) f.pressures[p] = 1.0f;
            }
            RunTest("Cluster: 10-Pad Palm (No GND/RST)", f);
        }
        {
            // Swept 8-Pad Cluster (knob modulation while bridging 8 pads)
            ControlFrame f = fb;
            for (int p = 0; p < 8; ++p) f.pressures[p] = 1.0f;
            RunTestSwept("Cluster: Swept 8-Pad Cluster", f, 24);
        }
        {
            ControlFrame f = fb;
            for (int p = 0; p < 12; ++p) f.pressures[p] = 1.0f;
            RunTest("Cluster: 12-Pad Full Cluster", f);
        }
        {
            // Diagonal Cross-Span (MIX+RST+VCC+AMP+IN+FBK)
            ControlFrame f = fb;
            f.pressures[NodeMix] = 1.0f; f.pressures[NodeRst] = 1.0f;
            f.pressures[NodeVcc] = 1.0f; f.pressures[NodeAmp] = 1.0f;
            f.pressures[NodeIn]  = 1.0f; f.pressures[NodeFbk] = 1.0f;
            RunTest("Cluster: Diagonal Cross-Span (6 pads)", f);
        }
        {
            // High-Drive Saturated Stress (OSC+FLT+DLY+AMP+FBK+OUT with Max Knobs)
            ControlFrame f = fb;
            f.pressures[NodeOsc] = 1.0f; f.pressures[NodeFlt] = 1.0f;
            f.pressures[NodeDly] = 1.0f; f.pressures[NodeAmp] = 1.0f;
            f.pressures[NodeFbk] = 1.0f; f.pressures[NodeOut] = 1.0f;
            f.s30_amp = 1.0f; f.s35_fbk = 1.0f; f.s31_flt = 1.0f; f.s34_dly = 1.0f;
            RunTest("Cluster: Saturated Stress (Max Knobs)", f);
        }
        {
            // Underclocked Starve Multi-Touch (10 pads @ 0.15V VCC + S33 CLK = 0.0)
            ControlFrame f = fb;
            for (int p = 0; p < 12; ++p) {
                if (p != NodeGnd && p != NodeRst) f.pressures[p] = 1.0f;
            }
            f.s37_vcc = 0.15f; f.s33_clk = 0.0f;
            RunTest("Cluster: Underclocked Starve Chaos (10 pads)", f);
        }

        // Summary & Breakdown
        hw.PrintLine("  -------------------------------------------+------------------+--------------+-------");
        hw.PrintLine("  WORST-CASE SCENARIO: %s (%lu cyc, %d%%)", worst_name, worst_cyc, (int)(worst_cyc * 100 / 640000));
        hw.PrintLine("  MODULE BREAKDOWN FOR WORST-CASE:");
        hw.PrintLine("    clear:    %6lu (%2d%%)", worst_prof.clear, (int)(worst_prof.clear * 100 / 640000)); System::Delay(2);
        hw.PrintLine("    circuit:  %6lu (%2d%%)", worst_prof.circuit, (int)(worst_prof.circuit * 100 / 640000)); System::Delay(2);
        hw.PrintLine("    prep:     %6lu (%2d%%)", worst_prof.prep, (int)(worst_prof.prep * 100 / 640000)); System::Delay(2);
        hw.PrintLine("    clock:    %6lu (%2d%%)", worst_prof.clock, (int)(worst_prof.clock * 100 / 640000)); System::Delay(2);
        hw.PrintLine("    osc:      %6lu (%2d%%)", worst_prof.osc, (int)(worst_prof.osc * 100 / 640000)); System::Delay(2);
        hw.PrintLine("    preamp:   %6lu (%2d%%)", worst_prof.preamp, (int)(worst_prof.preamp * 100 / 640000)); System::Delay(2);
        hw.PrintLine("    filter:   %6lu (%2d%%)", worst_prof.filter, (int)(worst_prof.filter * 100 / 640000)); System::Delay(2);
        hw.PrintLine("    delay:    %6lu (%2d%%)", worst_prof.delay, (int)(worst_prof.delay * 100 / 640000)); System::Delay(2);
        hw.PrintLine("    mixer:    %6lu (%2d%%)", worst_prof.mixer, (int)(worst_prof.mixer * 100 / 640000)); System::Delay(2);
        hw.PrintLine("    output:   %6lu (%2d%%)", worst_prof.output, (int)(worst_prof.output * 100 / 640000)); System::Delay(2);
        hw.PrintLine("    TOTAL:    %6lu (%2d%%)", worst_prof.total, (int)(worst_prof.total * 100 / 640000)); System::Delay(2);

        hw.PrintLine("=========================================================================");
        hw.PrintLine("ROUND #%lu COMPLETE. Underruns: %lu. Clicks: %lu. Standing by 3s...\r\n",
                     round, total_underruns_in_round, total_clicks_in_round);
        for (int i = 0; i < 6; ++i) {
            hw.SetLed(i % 2 == 0);
            System::Delay(500);
        }
    }
}

} // namespace bends
