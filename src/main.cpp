#include "daisy_seed.h"
#include "surface/touch.h"
#ifndef RUN_PERF_TEST
#define RUN_PERF_TEST 0
#endif

#include "app/instrument.h"
#if RUN_PERF_TEST
#include "../tests/on_device_perf.h"
#endif

using namespace daisy;
using namespace synthux;
using namespace bends;

static DaisySeed   hw;
static Touch       touch;
static Instrument DTCM_MEM_SECTION instrument;

// External SDRAM stereo delay buffer (2^20 stereo frames = 21.85s @ 48kHz stereo = 8 MB in SDRAM)
static Stereo DSY_SDRAM_BSS g_sdram_delay_buffer[tuning::kDelayFrames];

#include "dsp/itcm.h"

BENDS_ITCM void AudioCallback(AudioHandle::InputBuffer in,
                              AudioHandle::OutputBuffer out,
                              size_t size) {
    instrument.ProcessBlock(in[0], in[1], out[0], out[1], size);
    instrument.UpdateLedPwm(hw);
}

int main(void) {
    hw.Init(true);

    // Enable Cortex-M7 FPU Flush-to-Zero (FZ) and Default NaN (DN) modes
    __set_FPSCR(__get_FPSCR() | (1 << 24) | (1 << 25));

    // Zero out SDRAM delay buffer after SDRAM initialization
    for (size_t i = 0; i < tuning::kDelayFrames; ++i) {
        g_sdram_delay_buffer[i] = {0.0f, 0.0f};
    }

    // 64-sample block size (1.33 ms @ 48 kHz)
    hw.SetAudioBlockSize(64);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    touch.Init(hw);
    instrument.Init(hw.AudioSampleRate(), g_sdram_delay_buffer);

    // Calibrate baseline tracking on startup
    for (int i = 0; i < 40; i++) {
        touch.Process();
        System::Delay(3);
    }
    System::Delay(120);
    touch.pads().LockBaseline(true);

    // Initial surface read to populate frame buffer with physical knob/switch positions
    touch.Process();
    instrument.ProcessSurface(touch, true);

    // Startup LED sequence
    for (int b = 0; b < 3; ++b) {
        hw.SetLed(true);
        System::Delay(50);
        hw.SetLed(false);
        System::Delay(50);
    }

#if RUN_PERF_TEST
    RunOnDevicePerformanceCheck(hw, instrument, g_sdram_delay_buffer);
#else
    hw.StartAudio(AudioCallback);

    while (1) {
        bool new_scan = touch.Process();
        instrument.ProcessSurface(touch, new_scan);
        System::Delay(1);
    }
#endif
}
