#pragma once

#include "delay/registers.h"
#include "delay/memory.h"
#include "delay/read_heads.h"
#include "delay/converter.h"
#include "delay/dry_sampler.h"
#include "delay/feedback_path.h"
#include "dsp/filters.h"
#include "dsp/stereo.h"
#include "tuning/tuning.h"
#include "circuit/mappings.h"

namespace bends {

class DelayEngine {
public:
    void Init(float sample_rate, Stereo* sdram_buf) {
        _sample_rate = sample_rate;
        _memory.Init(sdram_buf);
        _heads.Init();
        _converter.Init(sample_rate);
        _dry_sampler.Init();
        _feedback.Reset();
        _knob_dly_held = 0.50f;
        _clk_dc = 0.50f;
        _smooth_clock_fm = 0.0f;
        _initialized = false;
        _aa_lp1_l = 0.0f; _aa_lp2_l = 0.0f;
        _aa_lp1_r = 0.0f; _aa_lp2_r = 0.0f;
    }

    void Clear() {
        _memory.Clear();
        _heads.Init();
        _converter.Reset();
        _dry_sampler.Reset();
        _feedback.Reset();
        _knob_dly_held = 0.50f;
        _clk_dc = 0.50f;
        _smooth_clock_fm = 0.0f;
        _initialized = false;
        _aa_lp1_l = 0.0f; _aa_lp2_l = 0.0f;
        _aa_lp1_r = 0.0f; _aa_lp2_r = 0.0f;
    }

    inline void ServiceIncrementalClear(size_t chunk_frames = 1024) {
        _memory.ServiceIncrementalClear(chunk_frames);
    }

    inline void PrepareBlock(float knob_dly, float knob_clk, DelayRegisters& reg, float scrub_offset = 0.0f, float override_delay_samples = 0.0f, bool instant_dly = false) {
        if (!_initialized) {
            _knob_dly_held = knob_dly;
            _clk_dc = knob_clk;
            _initialized = true;
        } else if (instant_dly) {
            _knob_dly_held = knob_dly;
        } else {
            _knob_dly_held += 0.06f * (knob_dly - _knob_dly_held);
        }
        _clk_dc += 0.12f * (knob_clk - _clk_dc);

        // Variclock speed & sample rate (S33 CLK):
        // Center detent (0.47 to 0.53) = exact 1.000x concert pitch, 48 kHz clean studio quality
        // Counter-clockwise (0.47 down to 0.00) = variable sample rate down to 0.200x (9.6 kHz lo-fi aliasing)
        // Clockwise (0.53 to 1.00) = overclock up to 2.000x (96 kHz equivalent / +1 octave pitch up)
        float base_speed;
        if (_clk_dc > 0.53f) {
            base_speed = 1.00f + 1.00f * ((_clk_dc - 0.53f) * (1.0f / 0.47f));
        } else if (_clk_dc < 0.47f) {
            float t = _clk_dc * (1.0f / 0.47f);
            base_speed = 0.20f + 0.80f * (t * t);
        } else {
            base_speed = 1.000f;
        }
        _base_speed = base_speed;
        reg.clock.rate *= base_speed;

        float target_samples;
        if (override_delay_samples > 0.0f) {
            target_samples = override_delay_samples;
            _base_samples = target_samples;
        } else {
            target_samples = mappings::DelaySamples(_knob_dly_held, _sample_rate);
            _base_samples = target_samples;
        }

        target_samples = std::clamp(target_samples, tuning::kMinDelaySamples, static_cast<float>(DelayMemory::kSize - 2));
        reg.time.offset = target_samples;
        if (reg.time.has_loop && !reg.time.is_chop) {
            reg.time.loop_len = std::clamp(target_samples, 48.0f, static_cast<float>(DelayMemory::kSize - 48000));
        }

        _heads.PrepareBlock(reg.time, target_samples, reg.speed.speed);

        _cached_eff_clock = std::clamp(reg.clock.rate, 0.08f, 1.0f);
        _converter.Prepare(_cached_eff_clock, reg.data);
        _dry_sampler.Prepare(_cached_eff_clock, reg.data);
        _feedback.PrepareBlock(reg.feedback, _knob_dly_held, _base_speed);

        // Stereo channel skew in crossfeed mode
        if (reg.feedback.crossfeed > 0.01f) {
            float eff_fb = _feedback.SmoothGain();
            float spread_norm = std::clamp(1.0f - eff_fb * 1.35f, 0.0f, 1.0f);
            float max_skew = std::min(768.0f, target_samples * 0.28f);
            reg.time.skew = max_skew * spread_norm;

            float pp_direct_r = 0.35f * spread_norm;
            _cached_pp_atten = pp_direct_r;
        } else {
            reg.time.skew = 0.0f;
            _cached_pp_atten = 1.0f;
        }

        if (_base_speed > 1.02f) {
            float target_fc = std::min(20000.0f, 22000.0f / _base_speed);
            float w = target_fc * (kPi * (1.0f / 48000.0f));
            _aa_alpha = std::clamp(w / (1.0f + w), 0.05f, 0.95f);
        } else {
            _aa_alpha = 0.95f;
        }
    }

    __attribute__((always_inline)) inline Stereo Process(Stereo in,
                                                         DelayRegisters& reg,
                                                         float osc_sig,
                                                         float in_env = 0.0f) {
        // 1. Check for requested head jumps
        if (reg.time.jump_req) {
            _heads.Jump(reg.time.jump_target, reg.time.jump_fade, reg.time.jump_loop_len);
            reg.time.jump_req = false;
        }

        // 2. Read delay memory through dual-head jump crossfader
        // Speed tracks variclock rate composed with reg.speed.speed.
        // Decimated update rate for clock FM transcendental evaluations
        _step_tick = (_step_tick + 1) & 3;
        if (_step_tick == 0) {
            _smooth_clock_fm += 0.25f * (reg.clock.fm - _smooth_clock_fm);
            _cached_fm_mult = (std::abs(_smooth_clock_fm) > 0.001f) ? FastPow2(_smooth_clock_fm * 0.75f) : 1.0f;
            float clock_mult = reg.clock.rate * _cached_fm_mult;
            float eff_speed = reg.clock.run ? (clock_mult * reg.speed.speed) : 0.0f;
            float abs_speed = std::abs(eff_speed);
            if (abs_speed >= 0.005f) {
                abs_speed = std::clamp(abs_speed, 0.02f, 2.00f);
                _cached_signed_speed = (eff_speed < 0.0f) ? -abs_speed : abs_speed;
            } else {
                _cached_signed_speed = 0.0f;
            }
            _cached_active_write = reg.clock.run && reg.write.write_enable && (_cached_signed_speed != 0.0f);
            _cached_write_speed = _cached_active_write ? _cached_signed_speed : 0.0f;
        }
        float signed_speed = _cached_signed_speed;

        float eff_mod = reg.time.mod_offset;

        Stereo raw = _heads.Process(_memory, signed_speed, eff_mod, reg.time);

        // 2b. Overclock anti-aliasing reconstruction filter (speeds > 1.0x):
        // Prevents supersonic harmonics from reflecting across Nyquist during varispeed pitch-up
        if (__builtin_expect(signed_speed > 1.02f, 0)) {
            _aa_lp1_l += _aa_alpha * (raw.l - _aa_lp1_l);
            _aa_lp2_l += _aa_alpha * (_aa_lp1_l - _aa_lp2_l);
            _aa_lp1_r += _aa_alpha * (raw.r - _aa_lp1_r);
            _aa_lp2_r += _aa_alpha * (_aa_lp1_r - _aa_lp2_r);
            raw.l = _aa_lp2_l;
            raw.r = _aa_lp2_r;
        }

        // 3. Zero-delay real-time lo-fi crossfade at minimum delay time (< 0.015f)
        if (_knob_dly_held < 0.015f && !reg.time.has_loop && !reg.time.bode_active) {
            float zero_amt = 1.0f - (_knob_dly_held * 66.6667f);
            raw = raw * (1.0f - zero_amt) + in * zero_amt;
        }

        // 4. Converter downsampler & data bus bit operations
        // Converter clock tracks variclock rate (reg.clock.rate), independent of read head playback speed.
        // Slowing the read head (Brake) is a playback pitch ramp, not a converter clock collapse.
        Stereo out_dac = _converter.Process(raw, _cached_eff_clock, reg.data);

        // 5. Feedback path calculation (damping, diffuser, crossfeed, erosion, loop guard, anti-mud & ducking)
        // Gentle feedback stabilization during active crossfades prevents phase accumulation crackles
        float fb_duck = 1.0f;
        if (__builtin_expect(_heads.IsXfadeActive(), 0)) {
            float p = _heads.XfadeProgress();
            fb_duck = 1.0f - 0.72f * p * (1.0f - p);
        }
        Stereo sat_fb = _feedback.Process(out_dac, osc_sig, reg.feedback, _knob_dly_held, in_env) * fb_duck;

        // 6. Write back to memory with clean 48kHz audio rate
        // In ping-pong crossfeed mode, apply precomputed right-channel attenuation
        Stereo in_routed = in;
        if (__builtin_expect(_cached_pp_atten < 0.999f, 0)) {
            in_routed.r *= _cached_pp_atten;
        }
        Stereo dly_in = in_routed + sat_fb;

        if (!reg.time.has_loop) {
            _memory.Write(dly_in, _cached_active_write, reg.memory.rot_depth, _cached_write_speed);
        } else {
            // When overdub is active, write to loop at active read position
            if (reg.write.write_enable) {
                float read_pos = _heads.ActiveReadPos(_memory.WritePos(), reg.time);
                _memory.Overdub(read_pos, dly_in, 0.65f, reg.memory.rot_depth);
            }
        }

        // 7. Output wet audio (tone-damped repeats and plate diffusion)
        Stereo wet = _feedback.WetOut();
        return wet;
    }

    __attribute__((always_inline)) inline Stereo ProcessDry(Stereo dry, const DelayRegisters& reg) {
        return _dry_sampler.Process(dry, _cached_eff_clock, reg.data);
    }

    float BaseSamples() const { return _base_samples; }
    float BaseSpeed() const { return _base_speed; }

    const DelayMemory& Memory() const { return _memory; }
    DelayMemory& Memory() { return _memory; }
    ReadHeadPair& Heads() { return _heads; }

private:
    float        _sample_rate{48000.0f};
    DelayMemory  _memory;
    ReadHeadPair _heads;
    Converter    _converter;
    DrySampler   _dry_sampler;
    FeedbackPath _feedback;

    float        _knob_dly_held{0.50f};
    float        _clk_dc{1.0f};
    float        _base_samples{24000.0f};
    float        _base_speed{1.0f};
    uint8_t      _step_tick{0};
    float        _cached_fm_mult{1.0f};
    float        _smooth_clock_fm{0.0f};
    float        _aa_alpha{0.95f};
    float        _aa_lp1_l{0.0f}, _aa_lp2_l{0.0f};
    float        _aa_lp1_r{0.0f}, _aa_lp2_r{0.0f};
    float        _cached_eff_clock{1.0f};
    float        _cached_pp_atten{1.0f};
    float        _cached_signed_speed{1.0f};
    bool         _cached_active_write{true};
    float        _cached_write_speed{1.0f};
    bool         _initialized{false};
};

} // namespace bends
