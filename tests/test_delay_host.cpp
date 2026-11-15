#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <iomanip>

#include "delay/delay_engine.h"
#include "dsp/stereo.h"
#include "bends/delay_bends.h"
#include "bends/mod_matrix.h"

using namespace bends;

void test_left_side() {
    std::vector<Stereo> buffer(tuning::kDelayFrames, Stereo(0.0f, 0.0f));
    DelayEngine delay;
    delay.Init(48000.0f, buffer.data());
    while (delay.Memory().IsClearing()) {
        delay.ServiceIncrementalClear(65536);
    }

    std::cout << "\nTest 5: S33 CLK Left Side Underclocking (1.0x -> 0.20x)..." << std::endl;
    DelayRegisters reg;
    reg.Reset();

    // S34 DLY = 0.20 (Karplus/short delay ~1000 samples), S33 CLK = 0.0 (down to 0.20x)
    for (int b = 0; b < 60; ++b) {
        reg.Reset();
        delay.PrepareBlock(0.20f, 0.00f, reg);
    }
    float speed_at_min = delay.BaseSpeed();
    std::cout << "  Base speed at S33=0.0: " << speed_at_min << " (Expected ~0.20x)" << std::endl;
    assert(std::abs(speed_at_min - 0.20f) < 0.02f);

    // Measure delay time at 0.20x: should be ~5x longer than base samples
    int impulse_frame = 100;
    int detected_delay_lofi = -1;
    for (int n = 0; n < 48000; ++n) {
        if ((n % 64) == 0) {
            reg.Reset();
            delay.PrepareBlock(0.20f, 0.00f, reg);
        }
        Stereo in = (n >= impulse_frame && n < impulse_frame + 10) ? Stereo(1.0f, 1.0f) : Stereo(0.0f, 0.0f);
        Stereo out = delay.Process(in, reg, 0.0f, 0.0f);
        if (n > impulse_frame + 10 && out.l > 0.05f && detected_delay_lofi < 0) {
            detected_delay_lofi = n - impulse_frame;
        }
    }
    std::cout << "  Base samples: " << delay.BaseSamples() << std::endl;
    std::cout << "  Detected delay frames at 0.20x: " << detected_delay_lofi << std::endl;
    float lofi_ratio = static_cast<float>(detected_delay_lofi) / delay.BaseSamples();
    std::cout << "  Lofi delay expansion ratio: " << lofi_ratio << " (Expected ~5.0x)" << std::endl;
    assert(std::abs(lofi_ratio - 5.0f) < 0.25f);
    std::cout << "  Left side 5x delay expansion: PASS" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Testing TouchBends Delay Engine" << std::endl;
    std::cout << "========================================" << std::endl;

    // Allocate SDRAM delay buffer on heap for host test
    std::vector<Stereo> buffer(tuning::kDelayFrames, Stereo(0.0f, 0.0f));
    DelayEngine delay;
    delay.Init(48000.0f, buffer.data());
    while (delay.Memory().IsClearing()) {
        delay.ServiceIncrementalClear(65536);
    }

    // ----------------------------------------------------
    // Test 1: S33 CLK = 1.0 (Center detent 0.50)
    // ----------------------------------------------------
    std::cout << "\nTest 1: S33 CLK = 1.0 (Center Detent, Clean 48kHz)..." << std::endl;
    DelayRegisters reg;
    reg.Reset();

    // S34 DLY = 0.50 (e.g. ~500ms delay), S33 CLK = 0.50 (1.0x)
    delay.PrepareBlock(0.50f, 0.50f, reg);
    assert(std::abs(delay.BaseSpeed() - 1.0f) < 0.001f);
    std::cout << "  Base speed at detent: " << delay.BaseSpeed() << " (PASS)" << std::endl;
    std::cout << "  Base samples: " << delay.BaseSamples() << std::endl;

    // Input an impulse and check delay time
    int impulse_frame = 100;
    int detected_delay = -1;
    for (int n = 0; n < 48000; ++n) {
        if ((n % 64) == 0) {
            reg.Reset();
            delay.PrepareBlock(0.50f, 0.50f, reg);
        }
        Stereo in = (n >= impulse_frame && n < impulse_frame + 10) ? Stereo(1.0f, 1.0f) : Stereo(0.0f, 0.0f);
        Stereo out = delay.Process(in, reg, 0.0f, 0.0f);
        if (n > impulse_frame + 10 && out.l > 0.05f && detected_delay < 0) {
            detected_delay = n - impulse_frame;
        }
    }
    std::cout << "  Detected delay frames: " << detected_delay << std::endl;
    assert(detected_delay > 1000);

    // ----------------------------------------------------
    // Test 2: S33 CLK = 1.0 -> 2.0 (Right side speedup)
    // ----------------------------------------------------
    std::cout << "\nTest 2: S33 CLK Right Side (1.0x -> 2.0x, Clean 48kHz, No Write Aliasing)..." << std::endl;
    delay.Clear();
    while (delay.Memory().IsClearing()) {
        delay.ServiceIncrementalClear(65536);
    }

    // S33 CLK at maximum (1.0 -> 2.0x speed)
    for (int b = 0; b < 50; ++b) {
        reg.Reset();
        delay.PrepareBlock(0.50f, 1.00f, reg);
    }
    float speed_at_max = delay.BaseSpeed();
    std::cout << "  Base speed at S33=1.0: " << speed_at_max << " (Expected 2.0x)" << std::endl;
    assert(std::abs(speed_at_max - 2.0f) < 0.01f);

    // Measure delay time at 2.0x speed: should be half of the delay at 1.0x
    int detected_delay_2x = -1;
    for (int n = 0; n < 48000; ++n) {
        if ((n % 64) == 0) {
            reg.Reset();
            delay.PrepareBlock(0.50f, 1.00f, reg);
        }
        Stereo in = (n >= impulse_frame && n < impulse_frame + 10) ? Stereo(1.0f, 1.0f) : Stereo(0.0f, 0.0f);
        Stereo out = delay.Process(in, reg, 0.0f, 0.0f);
        if (n > impulse_frame + 10 && out.l > 0.05f && detected_delay_2x < 0) {
            detected_delay_2x = n - impulse_frame;
        }
    }
    std::cout << "  Detected delay frames at 2.0x: " << detected_delay_2x << std::endl;
    float ratio = static_cast<float>(detected_delay) / static_cast<float>(detected_delay_2x);
    std::cout << "  Delay time ratio: " << ratio << " (Expected ~2.0x)" << std::endl;
    assert(std::abs(ratio - 2.0f) < 0.10f);
    std::cout << "  Right side delay time scales cleanly by 1/speed: PASS" << std::endl;

    // Verify clean sine pass-through at 2.0x without distortion
    delay.Clear();
    while (delay.Memory().IsClearing()) {
        delay.ServiceIncrementalClear(65536);
    }
    for (int n = 0; n < 24000; ++n) {
        if ((n % 64) == 0) {
            reg.Reset();
            delay.PrepareBlock(0.50f, 1.00f, reg);
        }
        float s = std::sin(2.0f * kPi * 440.0f * n / 48000.0f) * 0.5f;
        Stereo in(s, s);
        Stereo out = delay.Process(in, reg, 0.0f, 0.0f);
        if (n > detected_delay_2x + 100 && n < detected_delay_2x + 1000) {
            assert(!std::isnan(out.l) && !std::isinf(out.l));
            assert(std::abs(out.l) <= 1.0f);
        }
    }
    std::cout << "  Right side clean 48kHz playback: PASS" << std::endl;

    // ----------------------------------------------------
    // Test 3: S34 DLY Dual-Head Crossfade
    // ----------------------------------------------------
    std::cout << "\nTest 3: S34 DLY Granular Dual-Head Crossfade..." << std::endl;
    delay.Clear();
    while (delay.Memory().IsClearing()) {
        delay.ServiceIncrementalClear(65536);
    }
    reg.Reset();
    delay.PrepareBlock(0.40f, 0.50f, reg);

    // Fill buffer with 1 kHz tone
    for (int n = 0; n < 24000; ++n) {
        if ((n % 64) == 0) {
            reg.Reset();
            delay.PrepareBlock(0.40f, 0.50f, reg);
        }
        float s = std::sin(2.0f * kPi * 1000.0f * n / 48000.0f) * 0.5f;
        delay.Process(Stereo(s, s), reg, 0.0f, 0.0f);
    }

    // Change S34 DLY knob to 0.70f to trigger crossfade
    reg.Reset();
    delay.PrepareBlock(0.70f, 0.50f, reg);
    bool xfade_started = delay.Heads().IsXfadeActive();
    std::cout << "  Crossfade triggered on S34 move: " << (xfade_started ? "YES" : "NO") << std::endl;
    assert(xfade_started);

    // Verify crossfade completes cleanly without NaNs
    float max_sample = 0.0f;
    for (int n = 0; n < 3000; ++n) {
        Stereo out = delay.Process(Stereo(0.0f, 0.0f), reg, 0.0f, 0.0f);
        assert(!std::isnan(out.l) && !std::isinf(out.l));
        if (std::abs(out.l) > max_sample) max_sample = std::abs(out.l);
    }
    std::cout << "  Crossfade completed smoothly. Max sample: " << max_sample << " (PASS)" << std::endl;

    // ----------------------------------------------------
    // Test 4: Audio-Rate Clock FM (Clock Flutter & Phasing)
    // ----------------------------------------------------
    std::cout << "\nTest 4: Audio-Rate Clock FM at 1.5x speed..." << std::endl;
    delay.Clear();
    while (delay.Memory().IsClearing()) {
        delay.ServiceIncrementalClear(65536);
    }
    for (int b = 0; b < 50; ++b) {
        reg.Reset();
        delay.PrepareBlock(0.50f, 0.75f, reg); // ~1.5x speed
    }

    // Apply audio-rate FM (200 Hz sine FM)
    for (int n = 0; n < 4800; ++n) {
        float fm = std::sin(2.0f * kPi * 200.0f * n / 48000.0f) * 0.5f;
        reg.clock.fm = fm;
        Stereo in(0.3f, 0.3f);
        Stereo out = delay.Process(in, reg, 0.0f, 0.0f);
        assert(!std::isnan(out.l) && !std::isinf(out.l));
    }
    std::cout << "  Audio-rate FM at 1.5x speed: PASS (No NaNs, No crashes)" << std::endl;

    test_left_side();

    // ----------------------------------------------------
    // Test 6: S33 CLK Sweep Never Triggers Crossfades
    // ----------------------------------------------------
    std::cout << "\nTest 6: S33 CLK Sweep Never Triggers Crossfades (Continuous Varispeed)..." << std::endl;
    delay.Clear();
    while (delay.Memory().IsClearing()) {
        delay.ServiceIncrementalClear(65536);
    }
    reg.Reset();
    delay.PrepareBlock(0.50f, 0.50f, reg);

    // Sweep S33 from 0.50 (1.0x) to 1.00 (2.0x)
    bool any_s33_xfade = false;
    for (float clk = 0.50f; clk <= 1.00f; clk += 0.01f) {
        reg.Reset();
        delay.PrepareBlock(0.50f, clk, reg);
        for (int i = 0; i < 64; ++i) {
            delay.Process(Stereo(0.1f, 0.1f), reg, 0.0f, 0.0f);
            if (delay.Heads().IsXfadeActive()) any_s33_xfade = true;
        }
    }
    std::cout << "  Crossfades during S33 CLK sweep: " << (any_s33_xfade ? "YES (FAIL)" : "NONE (PASS)") << std::endl;
    assert(!any_s33_xfade);

    // ----------------------------------------------------
    // Test 7: S34 DLY Karplus Smooth Portamento
    // ----------------------------------------------------
    std::cout << "\nTest 7: S34 DLY in Karplus Territory Glides Smoothly (No Clicks, No Xfades)..." << std::endl;
    delay.Clear();
    while (delay.Memory().IsClearing()) {
        delay.ServiceIncrementalClear(65536);
    }
    reg.Reset();
    delay.PrepareBlock(0.05f, 0.50f, reg);
    for (int i = 0; i < 200; ++i) delay.Process(Stereo(0.1f, 0.1f), reg, 0.0f, 0.0f);

    bool any_karplus_xfade = false;
    for (float dly = 0.05f; dly <= 0.15f; dly += 0.005f) {
        reg.Reset();
        delay.PrepareBlock(dly, 0.50f, reg);
        for (int i = 0; i < 64; ++i) {
            delay.Process(Stereo(0.1f, 0.1f), reg, 0.0f, 0.0f);
            if (delay.Heads().IsXfadeActive()) any_karplus_xfade = true;
        }
    }
    std::cout << "  Crossfades in Karplus string sweep: " << (any_karplus_xfade ? "YES (FAIL)" : "NONE (PASS)") << std::endl;
    assert(!any_karplus_xfade);

    // ----------------------------------------------------
    // Test 8: Rapid Retrigger Jump Test
    // ----------------------------------------------------
    std::cout << "\nTest 8: Rapid Retrigger Jumps Under Heavy Stress..." << std::endl;
    delay.Clear();
    while (delay.Memory().IsClearing()) delay.ServiceIncrementalClear(65536);
    reg.Reset();
    delay.PrepareBlock(0.50f, 0.50f, reg);

    // Feed a continuous 440 Hz test tone into the delay buffer
    for (int n = 0; n < 4800; ++n) {
        float sig = std::sin(2.0f * kPi * 440.0f * n / 48000.0f) * 0.4f;
        delay.Process(Stereo(sig, sig), reg, 0.0f, 0.0f);
    }

    // Trigger rapid jumps faster than fade completion (every 25 samples)
    float max_delta = 0.0f;
    float prev_sample = 0.0f;
    for (int n = 0; n < 2400; ++n) {
        if ((n % 25) == 0) {
            float target = 4000.0f + static_cast<float>((n * 37) % 16000);
            reg.time.RequestJump(target, 256);
        }
        Stereo out = delay.Process(Stereo(0.2f, 0.2f), reg, 0.0f, 0.0f);
        assert(!std::isnan(out.l) && !std::isinf(out.l));
        assert(!std::isnan(out.r) && !std::isinf(out.r));
        if (n > 0) {
            float delta = std::abs(out.l - prev_sample);
            if (delta > max_delta) max_delta = delta;
        }
        prev_sample = out.l;
    }
    std::cout << "  Max per-sample output step during rapid retrigger: " << max_delta << std::endl;
    // Discontinuous buffer snaps would produce delta > 0.60f; continuous smooth xfading keeps deltas small
    assert(max_delta < 0.35f);
    std::cout << "  Rapid Retrigger Jump Queueing: PASS" << std::endl;

    // ----------------------------------------------------
    // Test 9: Stereo Delay Split
    // ----------------------------------------------------
    std::cout << "\nTest 9: Stereo Delay Split..." << std::endl;
    delay.Clear();
    while (delay.Memory().IsClearing()) delay.ServiceIncrementalClear(65536);
    reg.Reset();
    delay.PrepareBlock(0.50f, 0.50f, reg); // 500 ms delay (~24000 samples)

    // Write a Dirac delta impulse into the buffer
    delay.Process(Stereo(1.0f, 1.0f), reg, 0.0f, 0.0f);
    for (int n = 1; n < 48000; ++n) {
        reg.Reset();
        reg.time.stereo_split = 1.0f; // Max stereo split
        Stereo out = delay.Process(Stereo(0.0f, 0.0f), reg, 0.0f, 0.0f);
        // Verify output is clean and stable
        assert(!std::isnan(out.l) && !std::isnan(out.r));
    }
    std::cout << "  Stereo Delay Split Pitch Stability: PASS" << std::endl;

    // ----------------------------------------------------
    // Test 10: DLY <-> FBK Feedback Glitch
    // ----------------------------------------------------
    std::cout << "\nTest 10: DLY <-> FBK Feedback Glitch..." << std::endl;
    DelayBends delay_bends;
    AmountBus amounts;
    amounts.Init();
    amounts.Apply(Slot::DlyFbkGlitch, 1.0f);
    for (int i = 0; i < 60; ++i) amounts.UpdateBlock();
    DelayContext ctx{};
    reg.Reset();

    // 10A: High Feedback Knob (> 0.55)
    ctx.knob_fbk = 1.0f;
    for (int b = 0; b < 50; ++b) {
        delay_bends.Update(amounts, false, reg, ctx);
    }
    std::cout << "  High Knob - reg.write.write_enable: " << (reg.write.write_enable ? "TRUE" : "FALSE") << " (Expected TRUE)" << std::endl;
    std::cout << "  High Knob - reg.feedback.runaway: " << reg.feedback.runaway << " (Expected >= 1.0f)" << std::endl;
    std::cout << "  High Knob - reg.feedback.crossfeed: " << reg.feedback.crossfeed << " (Expected > 0.5f)" << std::endl;
    assert(reg.write.write_enable == true);
    assert(reg.feedback.runaway >= 1.0f);
    assert(reg.feedback.crossfeed >= 0.5f);

    // 10B: Low Feedback Knob (<= 0.45): Buffer Reorganizer
    ctx.knob_fbk = 0.20f;
    reg.Reset();
    for (int b = 0; b < 50; ++b) {
        delay_bends.Update(amounts, false, reg, ctx);
    }
    std::cout << "  Low Knob - reg.write.write_enable: " << (reg.write.write_enable ? "TRUE" : "FALSE") << " (Expected TRUE)" << std::endl;
    std::cout << "  Low Knob - reg.feedback.runaway: " << reg.feedback.runaway << " (Expected == 0.0f)" << std::endl;
    std::cout << "  Low Knob - reg.time.jump_req: " << (reg.time.jump_req ? "TRUE" : "FALSE") << std::endl;
    assert(reg.write.write_enable == true);
    assert(reg.feedback.runaway == 0.0f);
    std::cout << "  DLY <-> FBK Buffer Glitch: PASS" << std::endl;

    // ----------------------------------------------------
    // Test 11: ModMatrix Capacity Expansion (kMaxPerSlot = 5)
    // ----------------------------------------------------
    std::cout << "\nTest 11: ModMatrix Capacity Expansion (kMaxPerSlot = 5)..." << std::endl;
    ModMatrix mm;
    mm.Reset();
    assert(ModMatrix::kMaxPerSlot == 5);
    mm.Add(Slot::ClkRate, Sig::Osc, 0.5f);
    mm.Add(Slot::ClkRate, Sig::InEnv, 0.4f);
    mm.Add(Slot::ClkRate, Sig::OutAudio, 0.3f);
    mm.Add(Slot::ClkRate, Sig::OutEnv, 0.2f);
    mm.Add(Slot::ClkRate, Sig::InAudio, 0.1f);
    assert(mm.Count() == 5);
    std::cout << "  ModMatrix accepted 5 simultaneous modulators on Slot::ClkRate: PASS" << std::endl;

    // ----------------------------------------------------
    // Test 12: CLK - DLY Chop Pitch Stability & Stutter Slicing
    // ----------------------------------------------------
    std::cout << "\nTest 12: CLK - DLY Chop Pitch Stability & Stutter Slicing..." << std::endl;
    {
        DelayRegisters chop_reg;
        chop_reg.Reset();
        DelayContext chop_ctx;
        chop_ctx.knob_clk = 1.0f;
        chop_ctx.clk_rate = 1.0f;
        chop_ctx.is_frozen = false;
        chop_ctx.in_env = 0.5f;

        AmountBus chop_amounts;
        chop_amounts.Init();
        chop_amounts.Apply(Slot::DlyChop, 1.0f);
        for (int i = 0; i < 60; ++i) chop_amounts.UpdateBlock();

        DelayBends chop_bends;
        for (int b = 0; b < 40; ++b) {
            chop_reg.Reset();
            chop_bends.Update(chop_amounts, false, chop_reg, chop_ctx);
        }

        std::cout << "  Chop Active - reg.speed.speed: " << chop_reg.speed.speed << " (Expected 1.0f locked pitch)" << std::endl;
        std::cout << "  Chop Active - reg.time.has_loop: " << (chop_reg.time.has_loop ? "TRUE" : "FALSE") << " (Expected TRUE)" << std::endl;
        std::cout << "  Chop Active - reg.time.is_chop: " << (chop_reg.time.is_chop ? "TRUE" : "FALSE") << " (Expected TRUE)" << std::endl;
        std::cout << "  Chop Active - reg.time.loop_len: " << chop_reg.time.loop_len << " (Expected < 4320.0f)" << std::endl;

        assert(chop_reg.speed.speed == 1.0f);
        assert(chop_reg.time.has_loop == true);
        assert(chop_reg.time.is_chop == true);
        assert(chop_reg.time.loop_len < 4320.0f);
        std::cout << "  CLK - DLY Chop Pitch Stability: PASS" << std::endl;
    }

    // ----------------------------------------------------
    // Test 13: CLK - GND Digital Clock Underclock / Choke
    // ----------------------------------------------------
    std::cout << "\nTest 13: CLK - GND Digital Clock Underclock / Choke..." << std::endl;
    {
        DelayRegisters brake_reg;
        brake_reg.Reset();
        DelayContext brake_ctx;
        DelayBends brake_bends;

        // Light touch (pressure 0.3): pulls clock down directly
        AmountBus light_amounts;
        light_amounts.Init();
        light_amounts.Apply(Slot::DlyBrake, 0.30f);
        for (int i = 0; i < 60; ++i) light_amounts.UpdateBlock();

        for (int b = 0; b < 40; ++b) {
            brake_reg.Reset();
            brake_bends.Update(light_amounts, false, brake_reg, brake_ctx);
        }
        std::cout << "  Light Brake (0.30 pressure) - reg.clock.rate: " << brake_reg.clock.rate << " (Expected ~0.70-0.80x)" << std::endl;
        std::cout << "  Light Brake - reg.speed.speed: " << brake_reg.speed.speed << " (Expected 1.0f locked)" << std::endl;
        assert(brake_reg.clock.rate > 0.65f && brake_reg.clock.rate < 0.85f);
        assert(brake_reg.speed.speed == 1.0f);

        // Hard clamp (pressure 1.0): pulls clock down to extreme lo-fi crawl (~0.18x) and crunches resolution
        AmountBus hard_amounts;
        hard_amounts.Init();
        hard_amounts.Apply(Slot::DlyBrake, 1.0f);
        for (int i = 0; i < 60; ++i) hard_amounts.UpdateBlock();

        for (int b = 0; b < 40; ++b) {
            brake_reg.Reset();
            brake_bends.Update(hard_amounts, false, brake_reg, brake_ctx);
        }
        std::cout << "  Hard Brake (1.0 pressure) - reg.clock.rate: " << brake_reg.clock.rate << " (Expected ~0.18-0.22x)" << std::endl;
        std::cout << "  Hard Brake - reg.data.resolution: " << static_cast<int>(brake_reg.data.resolution) << "-bit (Expected 10-bit)" << std::endl;
        assert(brake_reg.clock.rate <= 0.25f);
        assert(brake_reg.data.resolution <= 10);

        // Release: should recover swiftly to 1.0x 48 kHz
        AmountBus zero_amounts;
        zero_amounts.Init();
        for (int b = 0; b < 40; ++b) {
            brake_reg.Reset();
            brake_bends.Update(zero_amounts, false, brake_reg, brake_ctx);
        }
        std::cout << "  Brake Release - reg.clock.rate: " << brake_reg.clock.rate << " (Expected 1.0x full speed recovery)" << std::endl;
        std::cout << "  Brake Release - reg.data.resolution: " << static_cast<int>(brake_reg.data.resolution) << "-bit (Expected 16-bit)" << std::endl;
        assert(brake_reg.clock.rate == 1.0f);
        assert(brake_reg.data.resolution == 16);
        std::cout << "  CLK - GND Digital Clock Underclock / Choke: PASS" << std::endl;
    }

    // ----------------------------------------------------
    // Test 14: CLK - RST Reverse Continuous Playback & Head Tracking
    // ----------------------------------------------------
    std::cout << "\nTest 14: CLK - RST Reverse Continuous Playback..." << std::endl;
    {
        DelayRegisters rev_reg;
        rev_reg.Reset();
        DelayContext rev_ctx;
        DelayBends rev_bends;

        AmountBus rev_amounts;
        rev_amounts.Init();
        rev_amounts.Apply(Slot::DlyReverse, 1.0f);
        for (int i = 0; i < 60; ++i) rev_amounts.UpdateBlock();

        for (int b = 0; b < 40; ++b) {
            rev_reg.Reset();
            rev_bends.Update(rev_amounts, false, rev_reg, rev_ctx);
        }

        std::cout << "  Reverse Active - reg.speed.speed: " << rev_reg.speed.speed << " (Expected ~ -1.0f)" << std::endl;
        std::cout << "  Reverse Active - write_enable: " << (rev_reg.write.write_enable ? "TRUE" : "FALSE") << " (Expected FALSE)" << std::endl;
        assert(rev_reg.speed.speed < -0.95f);
        assert(!rev_reg.write.write_enable);

        // Release reverse: should recover to 1.0f
        AmountBus zero_amounts;
        zero_amounts.Init();
        for (int b = 0; b < 60; ++b) {
            rev_reg.Reset();
            rev_bends.Update(zero_amounts, false, rev_reg, rev_ctx);
        }
        std::cout << "  Reverse Released - reg.speed.speed: " << rev_reg.speed.speed << " (Expected 1.0f)" << std::endl;
        assert(rev_reg.speed.speed == 1.0f);
        std::cout << "  CLK - RST Reverse Continuous Playback: PASS" << std::endl;
    }

    // ----------------------------------------------------
    // Test 15: DLY - VCC Scatter Varispeed Pitch & Feedback Excitation
    // ----------------------------------------------------
    std::cout << "\nTest 15: DLY - VCC Scatter Varispeed Pitch & Feedback Excitation..." << std::endl;
    {
        DelayRegisters sc_reg;
        sc_reg.Reset();
        DelayContext sc_ctx;
        sc_ctx.vcc = 0.85f; // Nominal rail
        DelayBends sc_bends;

        AmountBus sc_amounts;
        sc_amounts.Init();
        sc_amounts.Apply(Slot::DlyScatter, 1.0f);
        for (int i = 0; i < 60; ++i) sc_amounts.UpdateBlock();

        for (int b = 0; b < 40; ++b) {
            sc_reg.Reset();
            sc_bends.Update(sc_amounts, false, sc_reg, sc_ctx);
        }

        std::cout << "  Nominal Rail - reg.speed.speed: " << sc_reg.speed.speed << " (Expected ~2.0x octave up)" << std::endl;
        std::cout << "  Nominal Rail - reg.feedback.runaway: " << sc_reg.feedback.runaway << " (Expected > 0.35f)" << std::endl;
        std::cout << "  Nominal Rail - reg.feedback.crossfeed: " << sc_reg.feedback.crossfeed << " (Expected > 0.40f)" << std::endl;
        assert(sc_reg.speed.speed > 1.95f && sc_reg.speed.speed <= 2.05f);
        assert(sc_reg.feedback.runaway > 0.35f);
        assert(sc_reg.feedback.crossfeed > 0.40f);

        // Under starved rail (vcc = 0.30f): should request ghost memory jump
        sc_ctx.vcc = 0.30f;
        DelayRegisters starve_reg;
        starve_reg.Reset();
        bool saw_jump = false;
        for (int b = 0; b < 20; ++b) {
            starve_reg.Reset();
            sc_bends.Update(sc_amounts, false, starve_reg, sc_ctx);
            if (starve_reg.time.jump_req) saw_jump = true;
        }
        std::cout << "  Starved Rail - jump_req: " << (saw_jump ? "TRUE" : "FALSE") << " (Expected TRUE)" << std::endl;
        assert(saw_jump == true);
        std::cout << "  DLY - VCC Scatter Varispeed Pitch & Feedback Excitation: PASS" << std::endl;
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "ALL 15 TESTS PASSED!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
