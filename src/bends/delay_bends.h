#pragma once

#include <cstdint>
#include <algorithm>
#include <cmath>

#include "bends/amount_bus.h"
#include "delay/registers.h"
#include "delay/memory.h"
#include "dsp/fast_math.h"
#include "tuning/tuning.h"

namespace bends {

// =============================================================================
// Delay Context
// =============================================================================
struct DelayContext {
    uint32_t write_pos{0};
    float    vcc{1.0f};
    bool     clk_beat{false};
    float    clk_phase{0.0f};
    float    osc_sig{0.0f};
    float    osc_ramp{0.0f};
    uint8_t  osc_mode{1};
    bool     pendulum_slice{false};
    bool     is_latched{false};
    float    in_env{0.0f};
    float    brake_amount{0.0f};
    float    knob_clk{0.5f};
    float    clk_rate{1.0f};
    bool     is_frozen{false};
    bool     osc_stutter{false};
    float    osc_freq{1.0f};
    float    osc_phase{0.0f};
    uint8_t  osc_rnd_step{0};
    float    knob_fbk{0.5f};
    bool     punch_in{false};
    float    touch_dly{0.0f};
    float    touch_clk{0.0f};
};

// =============================================================================
// Health Controller (Brownout / Bit Rot)
// =============================================================================
class HealthController {
public:
    void Reset() {
        _rng = 0xDEADBEEFu;
        _sneak_timer = 2400;
        _sneak_duration = 0;
        _sneak_active = false;
        _sneak_speed = 1.0f;
        _sneak_slice_len = 4800.0f;
        _prev_beat = false;
    }

    void Update(DelayRegisters& reg, const DelayContext& ctx) {
        // Word length reduction under starvation
        uint8_t target_res = 16;
        if (ctx.vcc <= 0.05f) {
            target_res = 8;
        } else if (ctx.vcc <= 0.15f) {
            target_res = 10;
        } else if (ctx.vcc <= 0.28f) {
            target_res = 12;
        } else if (ctx.vcc <= 0.40f) {
            target_res = 14;
        }
        reg.data.resolution = std::min(reg.data.resolution, target_res);

        // DRAM bit rot simulation (< 0.25V)
        if (ctx.vcc < 0.25f) {
            float starve = (0.25f - ctx.vcc) * (1.0f / 0.25f);
            reg.memory.rot_depth = std::max(reg.memory.rot_depth, starve * 0.25f);
        }

        // Bus bit retention under deep starvation (< 0.20V)
        if (ctx.vcc < 0.20f) {
            uint32_t sticky_mask = (ctx.vcc < 0.10f) ? 0x0040u : 0x0020u;
            reg.data.stuck_mask = sticky_mask;
        }

        // Low-voltage sneak glitch: sporadic beat repeat or reverse hiccup (< 0.32V)
        if (ctx.vcc < 0.32f && !reg.time.is_chop && !ctx.is_frozen) {
            _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;

            bool beat_edge = ctx.clk_beat && !_prev_beat;
            _prev_beat = ctx.clk_beat;

            if (_sneak_active) {
                if (_sneak_duration > 0) {
                    _sneak_duration--;
                    reg.time.has_loop = true;
                    reg.write.write_enable = false;
                    reg.time.loop_len = _sneak_slice_len;
                    reg.speed.speed = _sneak_speed;
                    reg.speed.slew = 0.50f;
                    reg.feedback.gain = std::max(reg.feedback.gain, 0.82f);
                } else {
                    _sneak_active = false;
                    _sneak_timer = static_cast<uint16_t>(3000 + (_rng % 4500));
                    if (!ctx.is_frozen) {
                        reg.time.has_loop = false;
                        reg.write.write_enable = true;
                        reg.speed.speed = 1.0f;
                        reg.time.RequestJump(reg.time.offset, 384);
                    }
                }
            } else {
                bool trigger = false;
                if (_sneak_timer > 0) {
                    _sneak_timer--;
                    // When close to trigger, allow a clock beat to synchronize the hiccup
                    if (beat_edge && _sneak_timer < 600 && ((_rng & 0x07) == 0)) {
                        trigger = true;
                    }
                } else {
                    trigger = true;
                }

                if (trigger && !reg.time.jump_req) {
                    _sneak_active = true;
                    _sneak_duration = static_cast<uint16_t>(120 + (_rng % 140)); // ~160 to 350ms
                    _sneak_timer = static_cast<uint16_t>(3200 + (_rng % 4000));

                    // Derive musical slice length from clk_rate
                    float clk_hz = std::clamp(ctx.clk_rate, 0.25f, 40.0f);
                    float beat_samples = 48000.0f / clk_hz;
                    while (beat_samples > 12000.0f) beat_samples *= 0.5f;
                    while (beat_samples < 3000.0f)  beat_samples *= 2.0f;
                    _sneak_slice_len = beat_samples;

                    // 50% chance reverse playback, 50% chance forward repeat / octave drop
                    if ((_rng & 1) != 0) {
                        _sneak_speed = -1.0f; // Reverse glitch
                    } else {
                        _sneak_speed = ((_rng & 2) != 0) ? 0.5f : 1.0f; // Forward repeat / half-speed
                    }

                    reg.time.has_loop = true;
                    reg.write.write_enable = false;
                    reg.time.loop_len = _sneak_slice_len;
                    reg.speed.speed = _sneak_speed;
                    reg.speed.slew = 0.50f;
                    reg.feedback.gain = std::max(reg.feedback.gain, 0.82f);
                    reg.time.RequestJump(reg.time.offset, 256, _sneak_slice_len);
                }
            }
        } else {
            _prev_beat = ctx.clk_beat;
            if (_sneak_active) {
                _sneak_active = false;
                if (!ctx.is_frozen) {
                    reg.time.has_loop = false;
                    reg.write.write_enable = true;
                    reg.speed.speed = 1.0f;
                    reg.time.RequestJump(reg.time.offset, 384);
                }
            }
            _sneak_timer = 2400;
        }

        // Complete brownout shutdown below 0.010V
        if (ctx.vcc < 0.010f) {
            reg.clock.run = false;
            reg.write.write_enable = false;
        }
    }

private:
    uint32_t _rng{0xDEADBEEFu};
    uint16_t _sneak_timer{2400};
    uint16_t _sneak_duration{0};
    bool     _sneak_active{false};
    float    _sneak_speed{1.0f};
    float    _sneak_slice_len{4800.0f};
    bool     _prev_beat{false};
};

// =============================================================================
// Freeze Controller
// =============================================================================
class FreezeController {
public:
    void Reset() {
        _was_frozen = false;
        _freeze_len = 4800.0f;
        _scrub_active = false;
        _scrub_init_fbk = 0.5f;
        _scrub_offset = 0.0f;
    }

    void Update(bool is_frozen, DelayRegisters& reg, const DelayContext& ctx) {
        if (is_frozen) {
            if (!_was_frozen) {
                _was_frozen = true;
                _scrub_active = false;
                _scrub_init_fbk = ctx.knob_fbk;
                _scrub_offset = 0.0f;
            }

            reg.time.has_loop = true;
            reg.time.loop_len = std::clamp(reg.time.offset, 48.0f, static_cast<float>(tuning::kDelayFrames - 48000));

            // Feedback knob scrubs loop start position when moved (> 3.5% delta)
            if (!_scrub_active) {
                if (std::abs(ctx.knob_fbk - _scrub_init_fbk) > 0.035f) {
                    _scrub_active = true;
                }
            }

            if (_scrub_active) {
                // S35 scrubs loop start position backwards through memory history (up to ~20 seconds in SDRAM)
                constexpr float kMaxScrub = static_cast<float>(tuning::kDelayFrames - 48000);
                float target_scrub = ctx.knob_fbk * kMaxScrub;
                _scrub_offset += 0.05f * (target_scrub - _scrub_offset);
            } else {
                _scrub_offset = 0.0f;
            }

            reg.time.loop_anchor = static_cast<uint32_t>(_scrub_offset);

            // Punch-in record overdub (touching Pad 06 DLY or IN-DLY enables overdub)
            if (ctx.punch_in) {
                reg.write.write_enable = true;
            } else {
                reg.write.write_enable = false;
            }

            // Memory rot in freeze mode when starving
            if (ctx.vcc < tuning::kBroken) {
                float depth = std::clamp((tuning::kBroken - ctx.vcc) * (1.0f / tuning::kBroken), 0.0f, 1.0f);
                reg.memory.rot_depth = std::max(reg.memory.rot_depth, depth);
            }
        } else {
            _was_frozen = false;
            _scrub_active = false;
        }
    }

private:
    bool  _was_frozen{false};
    float _freeze_len{4800.0f};
    bool  _scrub_active{false};
    float _scrub_init_fbk{0.5f};
    float _scrub_offset{0.0f};
};

// =============================================================================
// Reverse Controller (CLK - RST)
// =============================================================================
class ReverseController {
public:
    void Reset() {
        _speed = 1.0f;
        _was_active = false;
    }

    void Update(float amount, DelayRegisters& reg, const DelayContext& ctx) {
        (void)ctx;
        if (amount > 0.02f) {
            _was_active = true;
            // Slew playback speed toward -1.0
            _speed += 0.15f * (-1.0f - _speed);
            reg.speed.speed = _speed;
            reg.write.write_enable = false; // Freeze writing so recorded audio isn't overwritten during reverse
        } else {
            if (_was_active) {
                _speed += 0.15f * (1.0f - _speed);
                if (_speed < 0.98f) {
                    reg.speed.speed = _speed;
                    reg.write.write_enable = false;
                } else {
                    _speed = 1.0f;
                    _was_active = false;
                    reg.speed.speed = 1.0f;
                    reg.time.RequestJump(reg.time.offset, 350);
                }
            }
        }
    }

private:
    float _speed{1.0f};
    bool  _was_active{false};
};

// =============================================================================
// Brake Controller (CLK - GND: Digital Clock Underclock / Choke)
// =============================================================================
class BrakeController {
public:
    void Reset() {
        _brake_rate = 1.0f;
        _brake_amount = 0.0f;
    }

    void Update(float amount, DelayRegisters& reg, const DelayContext& ctx) {
        (void)ctx;
        if (amount > 0.015f) {
            _brake_amount = amount;
            reg.data.brake_amount = amount;

            // Progressive digital clock deceleration & sub-audio bit-stepping:
            float target_clk;
            if (amount <= 0.35f) {
                // Light touch (0.015 to 0.35):
                // Smooth analog deceleration from 1.0x down to ~0.713x (varispeed pitch dive & chorus)
                // At amount = 0.30: target_clk = 0.754x (exact match for Host Test 13)
                target_clk = 1.0f - amount * 0.82f;
            } else if (amount <= 0.55f) {
                // Medium touch (0.35 to 0.55):
                // Ramping down from 0.713x to crunchy lo-fi sampler range (0.18x)
                float u = (amount - 0.35f) * (1.0f / 0.20f);
                target_clk = 0.713f - u * (0.713f - 0.18f);
            } else {
                // Firm to hard squeeze (0.55 to 1.00):
                // Plunges from 0.18x down into true sub-audio bit-stepping (~0.00004x / ~2 Hz clicks)
                float t = (amount - 0.55f) * (1.0f / 0.45f); // 0.0 .. 1.0
                target_clk = 0.18f * FastPow2(-12.0f * t);
            }
            target_clk = std::clamp(target_clk, 0.00002f, 1.0f);

            // Responsive slew towards target clock
            _brake_rate += 0.22f * (target_clk - _brake_rate);

            // Stepping through the bits: progressively clamp bit depth as clock collapses
            if (_brake_rate < 0.05f) {
                reg.data.resolution = std::min(reg.data.resolution, static_cast<uint8_t>(8));
                reg.data.stuck_mask |= 0x0001u;
            } else if (_brake_rate < 0.25f) {
                reg.data.resolution = std::min(reg.data.resolution, static_cast<uint8_t>(10));
            } else if (_brake_rate < 0.50f) {
                reg.data.resolution = std::min(reg.data.resolution, static_cast<uint8_t>(10));
            } else if (_brake_rate < 0.75f) {
                reg.data.resolution = std::min(reg.data.resolution, static_cast<uint8_t>(12));
            }
        } else {
            _brake_amount = 0.0f;
            reg.data.brake_amount = 0.0f;
            // Fast digital recovery back to nominal 48 kHz
            _brake_rate += 0.25f * (1.0f - _brake_rate);
            if (_brake_rate > 0.98f) {
                _brake_rate = 1.0f;
            }
        }
        if (_brake_rate < 0.999f || amount > 0.015f) {
            reg.clock.rate *= _brake_rate;
        }
    }

    float BrakeAmount() const { return _brake_amount; }

private:
    float _brake_rate{1.0f};
    float _brake_amount{0.0f};
};

// =============================================================================
// Chop Controller (CLK - DLY)
// =============================================================================
class ChopController {
public:
    void Reset() {
        _active = false;
        _is_looping = false;
        _clk_trig_accum = 0.0f;
        _repeats_left = 0;
        _beat_latched = false;
        _pendulum_rev = false;
        _beat_counter = 0;
        _stutter_anchor = 0.0f;
        _retrigger_timer = 0;
        _prev_phase = 0.0f;
        _smooth_amount = 0.0f;
        _sine_speed = 1.0f;
        _rnd_speed = 1.0f;
        _rng = 0x82A1B3C4u;
    }

    void Update(float amount, DelayRegisters& reg, const DelayContext& ctx) {
        if (amount > 0.008f) {
            // Pressure smoothing
            _smooth_amount += 0.08f * (amount - _smooth_amount);
            float prob = daisysp::fclamp((_smooth_amount - 0.04f) * (1.0f / 0.76f), 0.0f, 1.0f);

            bool is_osc_stutter = ctx.osc_stutter || ctx.pendulum_slice;
            float slice_len;
            float target_speed = 1.0f;

            if (!is_osc_stutter) {
                // -------------------------------------------------------------
                // Standard CLK + DLY: Clean, clock-synced repeats at 1x forward
                // -------------------------------------------------------------
                target_speed = 1.0f;

                // Derive fixed clock-synced slice length from ctx.clk_rate
                float clk_hz = std::clamp(ctx.clk_rate, 0.25f, 40.0f);
                float beat_samples = 48000.0f / clk_hz;

                // Bring into musical subdivision range (2500 .. 10000 samples)
                while (beat_samples > 10000.0f) beat_samples *= 0.5f;
                while (beat_samples < 2500.0f)  beat_samples *= 2.0f;

                // Firm pressure provides a tight ratchet/subdivision (not overly sensitive)
                if (_smooth_amount >= 0.78f) {
                    slice_len = beat_samples * 0.5f;
                } else {
                    slice_len = beat_samples;
                }
            } else {
                // -------------------------------------------------------------
                // 3-Pad OSC + CLK + DLY: Multi-Mode Chop based on Switch A
                // -------------------------------------------------------------
                // OSC Speed drives chop slice length (with musical octave-folding)
                float osc_f = ctx.osc_freq;
                while (osc_f > 20.0f) osc_f *= 0.5f;
                while (osc_f < 1.25f) osc_f *= 2.0f;
                slice_len = 48000.0f / osc_f;

                // Mode-specific behavior:
                if (ctx.osc_mode == 1) { // ModeSaw (Switch A Center)
                    // Reverse playback / pendulum
                    if (_smooth_amount > 0.70f) {
                        target_speed = -1.0f; // Firm pressure locks continuous reverse
                    } else {
                        target_speed = _pendulum_rev ? -1.0f : 1.0f; // Alternating pendulum on triggers
                    }
                } else if (ctx.osc_mode == 0) { // ModeSin (Switch A Up)
                    // Clean repeats + musical octave variations
                    if (_smooth_amount >= 0.75f) {
                        target_speed = _sine_speed; // Octave up or down on triggers
                    } else if (_smooth_amount >= 0.45f) {
                        target_speed = 1.009f; // Subtle detune (+15 cents)
                    } else {
                        target_speed = 1.0f; // Pure unison repeat
                    }
                } else { // ModeRnd (Switch A Down)
                    // Chaotic slicing with 45_bends speed zones
                    // Loops < 2000 samples are locked to +/-1.0f to keep them clean
                    if (slice_len < 2000.0f) {
                        target_speed = (_rnd_speed < 0.0f) ? -1.0f : 1.0f;
                    } else {
                        target_speed = _rnd_speed;
                    }
                }
            }

            // Bounded slice length
            slice_len = std::clamp(slice_len, 750.0f, 16000.0f);

            // Onset: immediate capture on first touch contact
            if (!_active) {
                _active = true;
                _is_looping = true;
                _stutter_anchor = reg.time.offset;
                _beat_counter = 0;
                _retrigger_timer = 0;
                _clk_trig_accum = 0.0f;
                _repeats_left = (prob >= 0.85f) ? 999 : (1 + static_cast<int>(prob * 3.0f));
                _pendulum_rev = false;
                _sine_speed = 1.0f;
                _rnd_speed = 1.0f;
                reg.time.RequestJump(_stutter_anchor, 256, slice_len);
            } else if (_is_looping) {
                // Interactive freeze scrubbing via S34 Delay knob while looping
                float scrub_target = reg.time.offset;
                _stutter_anchor += 0.15f * (scrub_target - _stutter_anchor);
            }

            // Triggering logic with clock-accumulated probability & rate limiting:
            if (_retrigger_timer > 0) _retrigger_timer--;

            bool is_beat = ctx.clk_beat && (_retrigger_timer == 0);
            _prev_phase = ctx.osc_phase;

            if (is_beat && !_beat_latched) {
                _beat_latched = true;
                _beat_counter++;
                _retrigger_timer = 9; // ~12ms cooldown at 750Hz block rate

                bool trigger = false;
                if (prob >= 0.85f) {
                    // Firm/hard pressure: 100% deterministic retrigger on every beat
                    trigger = true;
                    _is_looping = true;
                    _repeats_left = 999;
                } else {
                    // Clock-accumulated probability: accumulates on each beat
                    // At 25% prob -> triggers every 4th beat. At 50% prob -> triggers every 2nd beat.
                    _clk_trig_accum += prob;
                    if (_clk_trig_accum >= 1.0f) {
                        trigger = true;
                        _clk_trig_accum -= 1.0f;
                        if (_clk_trig_accum > 1.0f) _clk_trig_accum = 0.0f;
                        _is_looping = true;
                        _repeats_left = 1 + static_cast<int>(prob * 3.0f);
                        _stutter_anchor = reg.time.offset;
                    } else if (_is_looping) {
                        if (_repeats_left > 0) {
                            _repeats_left--;
                        } else {
                            // Gracefully release loop back to live delay audio
                            _is_looping = false;
                            reg.time.RequestJump(reg.time.offset, 384);
                        }
                    }
                }

                if (trigger) {
                    if (is_osc_stutter) {
                        if (ctx.osc_mode == 1) { // ModeSaw
                            _pendulum_rev = !_pendulum_rev;
                        } else if (ctx.osc_mode == 0) { // ModeSin
                            // Cycle between musical octave steps {1.0x, 2.0x, 1.0x, 0.5x}
                            static constexpr float kSineOctaves[4] = { 1.0f, 2.0f, 1.0f, 0.5f };
                            _sine_speed = kSineOctaves[_beat_counter % 4];
                        } else { // ModeRnd
                            // Roll random speed from 45_bends tonal set
                            _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
                            static constexpr float kRndSpeeds[5] = { 1.0f, -1.0f, 0.5f, -0.5f, 2.0f };
                            _rnd_speed = kRndSpeeds[(_rng >> 3) % 5];

                            // CD-skip laser anchor slip
                            constexpr float max_limit = static_cast<float>(tuning::kDelayFrames - 4800);
                            float slip = ((_rng & 1) ? 1024.0f : -1024.0f);
                            _stutter_anchor = std::clamp(_stutter_anchor + slip,
                                                         tuning::kMinDelaySamples,
                                                         max_limit);
                        }
                        reg.time.RequestJump(_stutter_anchor, 256, slice_len);
                    } else {
                        // Standard CLK + DLY: clean phrase ratcheting without pitch jump
                        constexpr float max_limit = static_cast<float>(tuning::kDelayFrames - 2);
                        float step_offset = std::clamp(_stutter_anchor + static_cast<float>(_beat_counter % 8) * (slice_len * 0.5f),
                                                       tuning::kMinDelaySamples, max_limit);
                        reg.time.RequestJump(step_offset, 256, slice_len);
                    }
                }
            } else if (!ctx.clk_beat) {
                _beat_latched = false;
            }

            if (_is_looping) {
                reg.time.has_loop = true;
                reg.write.write_enable = false;
                float starved_len = slice_len;
                if (ctx.vcc < 0.35f) {
                    _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
                    float jitter_pct = (0.35f - ctx.vcc) * (0.04f / 0.35f);
                    float jitter = ((static_cast<float>(_rng & 0xFF) / 255.0f) * 2.0f - 1.0f) * jitter_pct;
                    starved_len = std::max(64.0f, slice_len * (1.0f + jitter));
                }
                reg.time.loop_len = starved_len;
                reg.time.loop_anchor = static_cast<uint32_t>(_stutter_anchor);

                // Playback speed & brake
                float brake_scale = (ctx.brake_amount > 0.02f) ? std::max(0.0f, 1.0f - ctx.brake_amount) : 1.0f;
                reg.speed.speed = target_speed * brake_scale;
                reg.speed.slew = 0.50f;

                // Sustain stutter repeats without fading
                reg.feedback.gain = std::max(reg.feedback.gain, 0.85f);
            } else {
                if (!ctx.is_frozen) {
                    reg.time.has_loop = false;
                    reg.write.write_enable = true;
                }
            }
        } else {
            if (_active) {
                _active = false;
                _is_looping = false;
                _beat_latched = false;
                _pendulum_rev = false;
                _beat_counter = 0;
                _retrigger_timer = 0;
                _smooth_amount = 0.0f;
                _clk_trig_accum = 0.0f;
                _repeats_left = 0;
                _sine_speed = 1.0f;
                _rnd_speed = 1.0f;
                reg.time.loop_anchor = 0;
                if (!ctx.is_frozen) {
                    reg.time.has_loop = false;
                    reg.write.write_enable = true;
                    reg.time.RequestJump(reg.time.offset, 384);
                } else {
                    reg.time.has_loop = true;
                    reg.write.write_enable = false;
                    reg.time.RequestJump(reg.time.offset, 384);
                }
            }
        }
    }

private:
    bool     _active{false};
    bool     _is_looping{false};
    float    _clk_trig_accum{0.0f};
    int      _repeats_left{0};
    bool     _beat_latched{false};
    bool     _pendulum_rev{false};
    uint16_t _beat_counter{0};
    uint16_t _retrigger_timer{0};
    float    _stutter_anchor{0.0f};
    float    _prev_phase{0.0f};
    float    _smooth_amount{0.0f};
    float    _sine_speed{1.0f};
    float    _rnd_speed{1.0f};
    uint32_t _rng{0x82A1B3C4u};
};

// =============================================================================
// Scatter Controller (DLY - VCC: Pitch Shift & Address Jump)
// =============================================================================
class ScatterController {
public:
    void Reset() {
        _smooth_amount = 0.0f;
        _ghost_held = false;
        _scatter_timer = 0;
        _rng = 0x5CA77E81u;
    }

    void Update(float amount, DelayRegisters& reg, const DelayContext& ctx) {
        if (amount > 0.008f) {
            _smooth_amount += 0.10f * (amount - _smooth_amount);
        } else {
            _smooth_amount += 0.10f * (0.0f - _smooth_amount);
            if (_smooth_amount < 0.005f) {
                _smooth_amount = 0.0f;
                reg.time.bode_active = false;
            }
        }

        if (_smooth_amount > 0.005f) {
            if (ctx.vcc >= 0.45f) {
                reg.time.bode_active = false;
                // High-voltage overclock shimmer: varispeed pitch multiplier up to 2.0x (+12 semitones)
                float pitch_mult = 1.0f + _smooth_amount * 1.0f;
                reg.speed.speed *= pitch_mult;

                // High-voltage feedback excitation and stereo crossfeed bloom
                reg.feedback.gain *= (1.0f + _smooth_amount * 0.25f);
                reg.feedback.runaway = std::max(reg.feedback.runaway, _smooth_amount * 0.40f);
                reg.feedback.crossfeed = std::max(reg.feedback.crossfeed, _smooth_amount * 0.50f);

                // Reduce feedback damping to open shimmering high frequencies
                reg.feedback.damp = std::clamp(reg.feedback.damp - _smooth_amount * 0.40f, 0.05f, 0.95f);
                reg.feedback.tone_hp = std::min(reg.feedback.tone_hp + _smooth_amount * 0.25f, 0.40f);

                // Address scatter micro-jumps on firm presses (> 0.40 pressure)
                if (_smooth_amount > 0.40f) {
                    if (_scatter_timer > 0) {
                        _scatter_timer--;
                    } else {
                        _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
                        _scatter_timer = 30 + (_rng % 40);
                        float scatter_offset = static_cast<float>(_rng % 9600); // short jump up to 200ms
                        reg.time.RequestJump(std::clamp(reg.time.offset - scatter_offset,
                                                        tuning::kMinDelaySamples,
                                                        static_cast<float>(tuning::kDelayFrames - 4800)),
                                             96);
                    }
                }
                _ghost_held = false;
            } else {
                // Starved rail: Ghost memory address jump (1.5s - 3.5s in the past)
                reg.time.bode_active = false;

                if (!_ghost_held) {
                    _ghost_held = true;
                    _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
                    float ghost_delta = 72000.0f + static_cast<float>(_rng % 96000); // 1.5s to 3.5s in past
                    float ghost_target = std::clamp(reg.time.offset + ghost_delta,
                                                    tuning::kMinDelaySamples,
                                                    static_cast<float>(tuning::kDelayFrames - 4800));
                    reg.time.RequestJump(ghost_target, 128);
                }

                // Increase damping with pressure
                reg.feedback.damp = std::clamp(reg.feedback.damp + _smooth_amount * 0.25f, 0.0f, 0.95f);
                reg.feedback.gain = std::min(reg.feedback.gain * (1.0f + _smooth_amount * 0.10f), 0.98f);
            }
        } else {
            if (_ghost_held) {
                _ghost_held = false;
                reg.time.RequestJump(reg.time.offset, 200);
            }
        }
    }

private:
    float    _smooth_amount{0.0f};
    bool     _ghost_held{false};
    uint16_t _scatter_timer{0};
    uint32_t _rng{0x5CA77E81u};
};

// =============================================================================
// Sparkle Controller (Pitch Shift & Crossfeed)
// =============================================================================
class SparkleController {
public:
    void Update(float amount, DelayRegisters& reg, const DelayContext& ctx) {
        (void)ctx;
        if (amount > 0.02f) {
            // Pitch shift up to 2.0x
            float pitch = 1.0f + amount * 1.0f;
            reg.speed.speed *= pitch;

            // Crossfeed and feedback boost
            reg.feedback.crossfeed = std::max(reg.feedback.crossfeed, amount * 0.70f);
            reg.feedback.runaway = std::max(reg.feedback.runaway, amount * 0.35f);
        }
    }
};

// =============================================================================
// Skip Controller (Micro-Loop Repeats)
// =============================================================================
class SkipController {
public:
    void Reset() {
        _active = false;
        _hold_time = 0;
        _smooth_amount = 0.0f;
        _smooth_len = 0.0f;
        _repeat_cnt = 0;
        _target_repeats = 3;
        _sputter_timer = 0;
        _rng = 0x4A5B6C7Du;
    }

    void Update(float amount, DelayRegisters& reg, const DelayContext& ctx) {
        if (amount > 0.008f) {
            _active = true;
            _hold_time++;

            // Smooth touch pressure to eliminate sensor jitter
            _smooth_amount += 0.12f * (amount - _smooth_amount);

            reg.time.has_loop = true;

            // Live Audio Reactive Overdub:
            // When live audio is playing, allow new audio to overdub into the glitch loop.
            // When playing stops, freeze writing so the tail loops cleanly.
            if (!ctx.is_frozen && ctx.in_env > 0.04f) {
                reg.write.write_enable = true;
            } else {
                reg.write.write_enable = false;
            }

            // Stutter loop length (90ms down to 25ms with pressure)
            float target_len = 4320.0f * FastPow2(-_smooth_amount * 1.80f);
            target_len = std::clamp(target_len, 600.0f, 8000.0f);
            if (_smooth_len < 10.0f) {
                _smooth_len = target_len;
            } else {
                _smooth_len += 0.08f * (target_len - _smooth_len);
            }
            reg.time.loop_len = _smooth_len;

            // Buffer address ratcheting with CD-skip laser slip:
            _repeat_cnt++;
            uint16_t blocks_per_repeat = static_cast<uint16_t>(_smooth_len * (1.0f / 64.0f));
            if (blocks_per_repeat < 8) blocks_per_repeat = 8;
            if (_repeat_cnt >= blocks_per_repeat * _target_repeats) {
                _repeat_cnt = 0;
                _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
                _target_repeats = 2 + (_rng % 4); // 2 to 5 loop repeats

                float jump_delta = ((_rng & 1) ? 1.5f : -1.25f) * _smooth_len;
                // Add CD laser skip slip (+/-1024 samples) under firm touch
                if (_smooth_amount > 0.35f) {
                    jump_delta += ((_rng & 2) ? 1024.0f : -1024.0f);
                }
                float target_offset = std::clamp(reg.time.offset + jump_delta,
                                                 tuning::kMinDelaySamples,
                                                 static_cast<float>(tuning::kDelayFrames - 4800));
                reg.time.RequestJump(target_offset, 256, _smooth_len);
            }

            // Intermittent contact attenuation and cautious low-bit fracture:
            _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
            uint16_t sputter_roll = _rng & 0x7FFu;

            if (_sputter_timer > 0) {
                _sputter_timer--;
                // Contact attenuation during burst
                reg.data.strobe_l = 0.85f;
                reg.data.strobe_r = 0.85f;
                // Cautious low-bit XOR fracture (only bits 1..4: 0x001E)
                reg.data.stuck_mask = 0x001Eu | ((_rng & 3) << 2);
            } else {
                reg.data.strobe_l = 1.0f;
                reg.data.strobe_r = 1.0f;
                // Subtle static fracture when pressed firmly (> 0.50)
                if (_smooth_amount > 0.50f) {
                    reg.data.stuck_mask = 0x000Cu; // subtle fracture on bits 2,3
                } else {
                    reg.data.stuck_mask = 0;
                }

                // Probability of sputter dip scales gently with touch firmness
                uint16_t threshold = static_cast<uint16_t>(100.0f * _smooth_amount);
                if (sputter_roll < threshold) {
                    _sputter_timer = 2 + (_rng % 4); // 2 to 5 blocks (~3-6ms gentle dip)
                }
            }

            // Bit depth reduction (16 to 12 bits)
            if (_smooth_amount > 0.35f) {
                int res = static_cast<int>(16.0f - (_smooth_amount - 0.35f) * 6.0f);
                reg.data.resolution = static_cast<uint8_t>(std::clamp(res, 12, 16));
            } else {
                reg.data.resolution = 16;
            }

            // High-frequency damping
            reg.feedback.damp = std::clamp(reg.feedback.damp + 0.20f + _smooth_amount * 0.45f, 0.0f, 0.95f);

            // Bounded feedback headroom limit (prevents harsh runaway during ground short)
            reg.feedback.gain = std::min(reg.feedback.gain, 0.82f);
        } else {
            // Restore registers on release
            if (_active) {
                _active = false;
                _hold_time = 0;
                _smooth_amount = 0.0f;
                _smooth_len = 0.0f;
                _repeat_cnt = 0;
                _sputter_timer = 0;

                reg.data.strobe_l = 1.0f;
                reg.data.strobe_r = 1.0f;
                reg.data.stuck_mask = 0;
                reg.data.resolution = 16;

                if (!ctx.is_frozen) {
                    reg.time.has_loop = false;
                    reg.write.write_enable = true;
                    reg.time.RequestJump(reg.time.offset, 384);
                } else {
                    reg.time.has_loop = true;
                    reg.write.write_enable = false;
                    reg.time.RequestJump(reg.time.offset, 384);
                }
            }
        }
    }

private:
    bool     _active{false};
    uint32_t _hold_time{0};
    float    _smooth_amount{0.0f};
    float    _smooth_len{0.0f};
    uint16_t _repeat_cnt{0};
    uint16_t _target_repeats{3};
    uint8_t  _sputter_timer{0};
    uint32_t _rng{0x4A5B6C7Du};
};

// =============================================================================
// Buffer Jump Controller
// =============================================================================
class BufferJumpController {
public:
    void Update(float amount, DelayRegisters& reg, const DelayContext& ctx) {
        if (amount > 0.008f) {
            bool trigger_jump = false;

            // 1. Instantaneous trigger on touch onset
            if (!_held) {
                _held = true;
                _beat_count = 0;
                _subdiv_timer = 0;
                trigger_jump = true;
            }

            // 2. Clock beat synchronization
            if (ctx.clk_beat && !_beat_latched) {
                _beat_latched = true;
                _beat_count++;

                // Subdivisions based on touch pressure:
                // Light touch: Jump every 2 beats (phrase re-splice)
                // Medium touch: Jump on every beat
                // Firm squeeze: Jump on every beat + half-beat
                uint8_t beat_div = (amount > 0.70f) ? 1 : ((amount > 0.35f) ? 2 : 4);
                if ((_beat_count % beat_div) == 0) {
                    trigger_jump = true;
                }
            } else if (!ctx.clk_beat) {
                _beat_latched = false;
            }

            // 3. Fallback / fast subdivision timer (ensures responsive shuffling even at slow clocks)
            _subdiv_timer++;
            uint16_t timer_thresh = (amount > 0.75f) ? 15 : ((amount > 0.40f) ? 35 : 75);
            if (_subdiv_timer >= timer_thresh) {
                _subdiv_timer = 0;
                if (amount > 0.50f) {
                    trigger_jump = true;
                }
            }

            if (trigger_jump) {
                _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;

                // Musical subdivision jump ratios: 1/4, 1/2, 3/4, 1x, 1.5x, 2x current delay time
                static constexpr float kRatios[6] = { 0.25f, 0.50f, 0.75f, 1.00f, 1.50f, 2.00f };
                float ratio = kRatios[_rng % 6];
                float jump_offset = std::clamp(reg.time.offset * ratio,
                                               tuning::kMinDelaySamples,
                                               static_cast<float>(DelayMemory::kSize - 2));

                // Equal-gain crossfade (80 samples)
                reg.time.RequestJump(jump_offset, 80);
            }
        } else {
            _held = false;
            _beat_latched = false;
            _beat_count = 0;
            _subdiv_timer = 0;
        }
    }

private:
    bool     _held{false};
    bool     _beat_latched{false};
    uint16_t _beat_count{0};
    uint16_t _subdiv_timer{0};
    uint32_t _rng{0x13579BDFu};
};

// =============================================================================
// Data Short Controller (DLY - GND: Resolution & Bus Faults)
// =============================================================================
class DataShortController {
public:
    void Reset() {
        _skip_timer = 0;
        _rng = 0x5EED8421u;
        _smooth_amount = 0.0f;
    }

    void Update(float amount, DelayRegisters& reg, const DelayContext& ctx) {
        (void)ctx;
        if (amount > 0.008f) {
            _smooth_amount += 0.15f * (amount - _smooth_amount);
            float amt = _smooth_amount;

            // Resolution reduction (12-bit down to 10-bit with pressure)
            uint8_t res = 16;
            if (amt > 0.70f) {
                res = 10;
            } else if (amt > 0.40f) {
                res = 11;
            } else if (amt > 0.15f) {
                res = 12;
            }
            reg.data.resolution = std::min(reg.data.resolution, res);

            // Micro-address splicing:
            if (_skip_timer > 0) {
                _skip_timer--;
            } else if (amt > 0.30f) {
                _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
                // Micro-skips every 60ms down to 25ms with pressure
                uint16_t cooldown = static_cast<uint16_t>(18 + static_cast<uint16_t>((1.0f - amt) * 28.0f));
                _skip_timer = cooldown + (_rng % 10);

                // Micro-jump offset between -384 and +384 samples
                int32_t delta = static_cast<int32_t>((_rng & 0x01FF) - 256);
                float jump_target = std::clamp(reg.time.offset + static_cast<float>(delta),
                                               tuning::kMinDelaySamples,
                                               static_cast<float>(DelayMemory::kSize - 2));
                reg.time.RequestJump(jump_target, 96);
            }

            // Filter bypass with pressure
            reg.data.filter_bypass = std::max(reg.data.filter_bypass, amt * 0.30f);
        } else {
            _smooth_amount = 0.0f;
            _skip_timer = 0;
        }
    }

private:
    uint16_t _skip_timer{0};
    uint32_t _rng{0x5EED8421u};
    float    _smooth_amount{0.0f};
};

// =============================================================================
// Dropout Controller (Micro-Looper Jumps)
// =============================================================================
class DropoutController {
public:
    void Update(float amount, DelayRegisters& reg, const DelayContext& ctx) {
        float eff_amt = amount;

        // Condition-dependent dropout dynamics:
        // Audio envelope transients and power rail starvation accelerate and deepen signal dropouts
        float starve_factor = (ctx.vcc < 0.60f) ? (1.0f + (0.60f - ctx.vcc) * 1.5f) : 1.0f;
        float env_factor = 1.0f + ctx.in_env * 0.8f;
        eff_amt = std::min(1.0f, eff_amt * starve_factor * env_factor);

        if (eff_amt > 0.008f) {
            float strobe_val = (ctx.vcc < 0.40f || ctx.in_env > 0.60f) ? 0.45f : 0.70f;
            // Service Left dropout burst
            if (_drop_len_l > 0) {
                _drop_len_l--;
                reg.data.strobe_l = strobe_val;
            } else {
                reg.data.strobe_l = 1.0f;
                if (_interval_l > 0) {
                    _interval_l--;
                } else {
                    _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
                    _drop_len_l = 8 + (_rng % 24);
                    uint16_t min_gap = static_cast<uint16_t>(80.0f * (1.0f - eff_amt * 0.75f));
                    _interval_l = min_gap + (_rng % 100);

                    // Micro-loop jump
                    float micro_offset = 64.0f + static_cast<float>(_rng & 0xBF);
                    reg.time.RequestJump(std::max(tuning::kMinDelaySamples, reg.time.offset - micro_offset), 96);
                }
            }

            // Service Right dropout burst (decorrelated)
            if (_drop_len_r > 0) {
                _drop_len_r--;
                reg.data.strobe_r = strobe_val;
            } else {
                reg.data.strobe_r = 1.0f;
                if (_interval_r > 0) {
                    _interval_r--;
                } else {
                    _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
                    _drop_len_r = 8 + ((_rng >> 6) % 24);
                    uint16_t min_gap = static_cast<uint16_t>(80.0f * (1.0f - eff_amt * 0.75f));
                    _interval_r = min_gap + ((_rng >> 4) % 100);

                    float micro_offset = 48.0f + static_cast<float>((_rng >> 5) & 0x7F);
                    reg.time.RequestJump(std::max(tuning::kMinDelaySamples, reg.time.offset - micro_offset), 96);
                }
            }
        } else {
            reg.data.strobe_l = 1.0f;
            reg.data.strobe_r = 1.0f;
            _drop_len_l = 0;
            _drop_len_r = 0;
            _interval_l = 20;
            _interval_r = 35;
        }
    }

private:
    uint16_t _drop_len_l{0};
    uint16_t _drop_len_r{0};
    uint16_t _interval_l{20};
    uint16_t _interval_r{35};
    uint32_t _rng{0xACE12480u};
};

// =============================================================================
// Restart Controller
// =============================================================================
class RestartController {
public:
    void Update(float amount, DelayRegisters& reg, const DelayContext& ctx) {
        (void)ctx;
        if (amount > 0.05f) {
            if (!_triggered) {
                _triggered = true;
                _saved_offset = reg.time.offset;
                reg.time.RequestJump(tuning::kMinDelaySamples, 48);
            }
            reg.time.offset = tuning::kMinDelaySamples;
        } else {
            if (_triggered) {
                _triggered = false;
                reg.time.RequestJump(_saved_offset, 400);
            }
        }
    }

private:
    bool  _triggered{false};
    float _saved_offset{tuning::kMinDelaySamples};
};

// =============================================================================
// Feedback Glitch Controller (DLY - FBK)
// =============================================================================
// S35 Feedback knob modulates behavior:
// 1. Low feedback knob (<= 0.45): buffer slice jumps across recorded memory.
// 2. High feedback knob (> 0.45): runaway feedback and comb delay (8ms - 32ms).
// =============================================================================
class FbkGlitchController {
public:
    void Reset() {
        _active = false;
        _smooth_amount = 0.0f;
        _scatter_timer = 0;
        _rng = 0x9E3779B9u;
    }

    void Update(float amount, DelayRegisters& reg, const DelayContext& ctx) {
        if (amount > 0.008f) {
            _active = true;
            _smooth_amount += 0.10f * (amount - _smooth_amount);
            float amt = _smooth_amount;
            float fbk_knob = std::clamp(ctx.knob_fbk, 0.0f, 1.0f);

            // Overdub enabled
            reg.write.write_enable = true;

            // Feedback scaling
            if (fbk_knob > 0.45f) {
                // High feedback: Saturated buffer runaway (up to 1.25x)
                float fbk_drive = (fbk_knob - 0.45f) * 1.818f; // 0.0 to 1.0
                reg.feedback.runaway = std::max(reg.feedback.runaway, amt * fbk_drive * 1.25f);
                reg.feedback.gain = std::max(reg.feedback.gain, 0.75f + fbk_drive * (0.20f + amt * 0.20f));
                reg.feedback.crossfeed = std::max(reg.feedback.crossfeed, 0.40f + fbk_drive * (0.20f + amt * 0.35f));
            } else {
                // Low feedback: Buffer Reorganizer mode (clean, non-runaway)
                reg.feedback.runaway = 0.0f;
                reg.feedback.gain = std::max(reg.feedback.gain, 0.35f + fbk_knob * 0.65f);
                reg.feedback.crossfeed = std::max(reg.feedback.crossfeed, fbk_knob * 0.50f * amt);
            }

            // Stereo channel offset
            reg.time.stereo_split = std::max(reg.time.stereo_split, amt * (0.35f + fbk_knob * 0.55f));

            // Granular address offsets:
            if (_scatter_timer > 0) {
                _scatter_timer--;
            } else {
                _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;

                constexpr float kMaxDelay = static_cast<float>(DelayMemory::kSize - 2);

                if (fbk_knob <= 0.45f) {
                    // ---------------------------------------------------------
                    // Low feedback knob: slice jump interval (120ms - 280ms)
                    // ---------------------------------------------------------
                    uint16_t cooldown = static_cast<uint16_t>(50 + static_cast<uint16_t>((1.0f - amt) * 90.0f));
                    _scatter_timer = cooldown + (_rng % 30);

                    // Subdivisions vs random memory jumps
                    if ((_rng & 1) != 0) {
                        static constexpr float kPhraseRatios[6] = { 0.25f, 0.50f, 0.75f, 1.00f, 1.50f, 2.00f };
                        float ratio = kPhraseRatios[(_rng >> 1) % 6];
                        float target = std::clamp(reg.time.offset * ratio,
                                                  tuning::kMinDelaySamples,
                                                  kMaxDelay);
                        reg.time.RequestJump(target, 256);
                    } else {
                        // Random memory jump across recent 0.1s to 2.5s recorded audio
                        float rand_offset = 4800.0f + static_cast<float>((_rng >> 2) % 115200);
                        float target = std::clamp(rand_offset,
                                                  tuning::kMinDelaySamples,
                                                  kMaxDelay);
                        reg.time.RequestJump(target, 256);
                    }
                } else {
                    // ---------------------------------------------------------
                    // High feedback knob: rapid jump interval (25ms - 90ms)
                    // ---------------------------------------------------------
                    uint16_t cooldown = static_cast<uint16_t>(18 + static_cast<uint16_t>((1.0f - amt) * 45.0f));
                    _scatter_timer = cooldown + (_rng % 12);

                    if (amt > 0.65f) {
                        // Short delay loop (8ms to 32ms)
                        float comb_samples = 384.0f + static_cast<float>(_rng % 1152);
                        reg.time.RequestJump(comb_samples, 256);
                    } else {
                        // Harmonic subdivision jumps of base delay (1/8, 1/6, 1/4, 1/3, 1/2, 3/4)
                        static constexpr float kHarmonics[6] = { 0.125f, 0.1667f, 0.25f, 0.3333f, 0.50f, 0.75f };
                        float ratio = kHarmonics[_rng % 6];
                        float target = std::clamp(reg.time.offset * ratio,
                                                  tuning::kMinDelaySamples,
                                                  kMaxDelay);
                        reg.time.RequestJump(target, 256);
                    }
                }
            }

            // Feedback damping modulation
            reg.feedback.damp = std::clamp(reg.feedback.damp - amt * 0.12f, 0.10f, 0.95f);
        } else {
            if (_active) {
                _active = false;
                _smooth_amount = 0.0f;
                _scatter_timer = 0;
                // Smoothly restore base delay distance
                reg.time.RequestJump(reg.time.offset, 384);
            }
        }
    }

private:
    bool     _active{false};
    float    _smooth_amount{0.0f};
    uint16_t _scatter_timer{0};
    uint32_t _rng{0x9E3779B9u};
};

// =============================================================================
// Delay Bends Coordinator
// =============================================================================
class DelayBends {
public:
    void Update(const AmountBus& amounts,
                bool is_frozen,
                DelayRegisters& regs,
                const DelayContext& ctx) {
        _freeze.Update(is_frozen, regs, ctx);
        _reverse.Update(amounts.Get(Slot::DlyReverse), regs, ctx);
        _brake.Update(amounts.Get(Slot::DlyBrake), regs, ctx);
        float chop_amt = amounts.Get(Slot::DlyChop);
        if (ctx.osc_stutter) {
            chop_amt = std::max(chop_amt, 1.0f);
        }
        regs.time.is_chop = (chop_amt > 0.008f);
        _chop.Update(chop_amt, regs, ctx);
        _scatter.Update(amounts.Get(Slot::DlyScatter), regs, ctx);
        _sparkle.Update(amounts.Get(Slot::DlySparkle), regs, ctx);
        _skip.Update(amounts.Get(Slot::DlySkip), regs, ctx);
        _buffer_jump.Update(amounts.Get(Slot::DlyBufferJump), regs, ctx);
        _data_short.Update(amounts.Get(Slot::DlyDataShort), regs, ctx);
        _dropout.Update(amounts.Get(Slot::DlyDropout), regs, ctx);
        _restart.Update(amounts.Get(Slot::DlyRestart), regs, ctx);
        _fbk_glitch.Update(amounts.Get(Slot::DlyFbkGlitch), regs, ctx);
        _health.Update(regs, ctx);

        // Stereo head decoupling
        float stereo_split = amounts.Get(Slot::DlyStereoSplit);
        if (stereo_split > 0.001f) {
            regs.time.stereo_split = std::max(regs.time.stereo_split, stereo_split);
        }

        // Feedback amounts
        float ping_pong = amounts.Get(Slot::DlyPingPong);
        if (ping_pong > 0.001f) {
            regs.feedback.crossfeed = std::max(regs.feedback.crossfeed, ping_pong);
        }
        regs.feedback.runaway       = std::max(regs.feedback.runaway, amounts.Get(Slot::DlyRunaway));
        regs.feedback.erode         = amounts.Get(Slot::DlyFilterLoop);
        regs.feedback.drain         = amounts.Get(Slot::DlyFeedbackDrain);
        regs.feedback.infinite_hold = (amounts.Get(Slot::DlyInfiniteHold) > 0.05f);
        regs.feedback.larsen_howl   = amounts.Get(Slot::DlyLarsenHowl);

        // Anti-filter reconstruction bypass bend:
        // Reconstruction filter bypass on DLY/CLK bends
        float bend_lift = amounts.Get(Slot::DlyScatter) + amounts.Get(Slot::DlySkip) +
                          amounts.Get(Slot::DlyChop) + amounts.Get(Slot::DlyDataShort) +
                          amounts.Get(Slot::DlyFbkGlitch) +
                          ctx.touch_dly + ctx.touch_clk;
        regs.data.filter_bypass = std::min(1.0f, bend_lift);
    }

private:
    FreezeController      _freeze;
    ReverseController     _reverse;
    BrakeController       _brake;
    SparkleController     _sparkle;
    ChopController        _chop;
    ScatterController     _scatter;
    SkipController        _skip;
    BufferJumpController  _buffer_jump;
    DataShortController   _data_short;
    DropoutController     _dropout;
    RestartController     _restart;
    FbkGlitchController   _fbk_glitch;
    HealthController      _health;
};

} // namespace bends
