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
        _ghost_timer = 2400;
        _ghost_duration = 0;
        _ghost_active = false;
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

        // Unstable address slips under low rail voltage (< 0.35V)
        if (ctx.vcc < 0.35f) {
            _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;

            if (_ghost_active) {
                if (_ghost_duration > 0) {
                    _ghost_duration--;
                } else {
                    _ghost_active = false;
                    reg.time.RequestJump(reg.time.offset, 128);
                }
            } else {
                if (_ghost_timer > 0) {
                    _ghost_timer--;
                } else {
                    _ghost_timer = static_cast<uint16_t>(2600 + (_rng % 3800));

                    if ((ctx.in_env > 0.02f || ctx.is_frozen) && !reg.time.jump_req) {
                        _ghost_active = true;
                        _ghost_duration = static_cast<uint16_t>(100 + (_rng % 150));
                        float ghost_delta = 72000.0f + static_cast<float>(_rng % 96000);
                        float ghost_target = std::clamp(reg.time.offset + ghost_delta,
                                                        tuning::kMinDelaySamples,
                                                        static_cast<float>(tuning::kDelayFrames - 4800));
                        reg.time.RequestJump(ghost_target, 128);
                    }
                }
            }
        } else {
            if (_ghost_active) {
                _ghost_active = false;
                reg.time.RequestJump(reg.time.offset, 128);
            }
            _ghost_timer = 2400;
        }

        // Complete brownout shutdown below 0.015V
        if (ctx.vcc < 0.015f) {
            reg.clock.run = false;
            reg.write.write_enable = false;
        }
    }

private:
    uint32_t _rng{0xDEADBEEFu};
    uint16_t _ghost_timer{2400};
    uint16_t _ghost_duration{0};
    bool     _ghost_active{false};
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
    }

    void Update(float amount, DelayRegisters& reg, const DelayContext& ctx) {
        (void)ctx;
        if (amount > 0.015f) {
            // Direct digital clock deceleration: instantaneous rate reduction without slew lag
            float target_clk = std::clamp(1.0f - amount * 0.82f, 0.18f, 1.0f);
            _brake_rate += 0.22f * (target_clk - _brake_rate);

            // Subtly crunch bit depth as clock frequency collapses
            if (_brake_rate < 0.50f) {
                reg.data.resolution = std::min(reg.data.resolution, static_cast<uint8_t>(10));
            } else if (_brake_rate < 0.75f) {
                reg.data.resolution = std::min(reg.data.resolution, static_cast<uint8_t>(12));
            }
        } else {
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

private:
    float _brake_rate{1.0f};
};

// =============================================================================
// Chop Controller (CLK - DLY)
// =============================================================================
class ChopController {
public:
    void Reset() {
        _active = false;
        _beat_latched = false;
        _pendulum_rev = false;
        _beat_counter = 0;
        _stutter_anchor = 0.0f;
        _retrigger_timer = 0;
        _prev_phase = 0.0f;
        _smooth_amount = 0.0f;
        _current_div = 2.0f;
    }

    void Update(float amount, DelayRegisters& reg, const DelayContext& ctx) {
        if (amount > 0.008f) {
            // Pressure smoothing
            _smooth_amount += 0.08f * (amount - _smooth_amount);

            reg.time.has_loop = true;
            reg.write.write_enable = false;

            // Onset: capture anchor at current delay playback position
            if (!_active) {
                _active = true;
                _stutter_anchor = reg.time.offset;
                _beat_counter = 0;
                _retrigger_timer = 0;
                _current_div = 2.0f;
                _pendulum_rev = false;
            } else {
                // Interactive freeze scrubbing via S34 Delay knob while holding CLK + DLY
                float scrub_target = reg.time.offset;
                _stutter_anchor += 0.15f * (scrub_target - _stutter_anchor);
            }

            // Slicing window decoupled from playback speed
            constexpr float kBaseGlitchSamples = 6000.0f; // 125 ms @ 48 kHz
            constexpr float kMinGlitchSamples  = 750.0f;  // ~15.6 ms audio-rate roll
            constexpr float kMaxGlitchSamples  = 16000.0f; // ~333 ms
            float slice_len;

            bool is_pendulum = ctx.osc_stutter || ctx.pendulum_slice;

            if (is_pendulum) {
                // OSC + CLK + DLY: Alternating playback direction on triggers
                float clk_hz = std::clamp(ctx.clk_rate, 1.5f, 5.5f);
                slice_len = 48000.0f / clk_hz;
            } else {
                // Standard CLK + DLY: Division steps with hysteresis
                if (_current_div >= 8.0f) {
                    if (_smooth_amount < 0.78f) _current_div = 4.0f;
                } else if (_current_div >= 4.0f) {
                    if (_smooth_amount >= 0.88f) _current_div = 8.0f;
                    else if (_smooth_amount < 0.55f) _current_div = 2.0f;
                } else {
                    if (_smooth_amount >= 0.65f) _current_div = 4.0f;
                }
                slice_len = kBaseGlitchSamples / _current_div;
            }

            // Clamp slice length
            slice_len = std::clamp(slice_len, kMinGlitchSamples, kMaxGlitchSamples);
            reg.time.loop_len = slice_len;
            reg.time.loop_anchor = static_cast<uint32_t>(_stutter_anchor);

            // Playback speed modulation
            float brake_scale = (ctx.brake_amount > 0.02f) ? std::max(0.0f, 1.0f - ctx.brake_amount) : 1.0f;
            float dir = (reg.speed.speed < -0.01f) ? -1.0f : 1.0f;
            if (is_pendulum && _pendulum_rev) {
                dir = -dir;
            }
            reg.speed.speed = dir * brake_scale;
            reg.speed.slew = 0.50f;

            // Sustain stutter repeats without fading
            reg.feedback.gain = std::max(reg.feedback.gain, 0.85f);

            // Triggering logic with rate limiting:
            if (_retrigger_timer > 0) _retrigger_timer--;

            bool trigger_beat = ctx.clk_beat && (_retrigger_timer == 0);
            _prev_phase = ctx.osc_phase;

            if (trigger_beat && !_beat_latched) {
                _beat_latched = true;
                _beat_counter++;
                _retrigger_timer = 9; // ~12ms minimum cooldown at 750Hz block rate
                if (is_pendulum) {
                    _pendulum_rev = !_pendulum_rev;
                } else {
                    constexpr float max_limit = static_cast<float>(tuning::kDelayFrames - 2);
                    float step_offset = std::clamp(_stutter_anchor + static_cast<float>(_beat_counter % 8) * (slice_len * 0.5f),
                                                   tuning::kMinDelaySamples, max_limit);
                    reg.time.RequestJump(step_offset, 256, slice_len);
                }
            } else if (!ctx.clk_beat) {
                _beat_latched = false;
            }
        } else {
            if (_active) {
                _active = false;
                _beat_latched = false;
                _pendulum_rev = false;
                _beat_counter = 0;
                _retrigger_timer = 0;
                _smooth_amount = 0.0f;
                _current_div = 2.0f;
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
    bool     _beat_latched{false};
    bool     _pendulum_rev{false};
    uint16_t _beat_counter{0};
    uint16_t _retrigger_timer{0};
    float    _stutter_anchor{0.0f};
    float    _prev_phase{0.0f};
    float    _smooth_amount{0.0f};
    float    _current_div{2.0f};
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

            // Buffer address ratcheting:
            _repeat_cnt++;
            uint16_t blocks_per_repeat = static_cast<uint16_t>(_smooth_len * (1.0f / 64.0f));
            if (blocks_per_repeat < 8) blocks_per_repeat = 8;
            if (_repeat_cnt >= blocks_per_repeat * _target_repeats) {
                _repeat_cnt = 0;
                _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
                _target_repeats = 2 + (_rng % 4); // 2 to 5 loop repeats

                float jump_delta = ((_rng & 1) ? 1.5f : -1.25f) * _smooth_len;
                float target_offset = std::clamp(reg.time.offset + jump_delta,
                                                 tuning::kMinDelaySamples,
                                                 static_cast<float>(tuning::kDelayFrames - 4800));
                reg.time.RequestJump(target_offset, 256, _smooth_len);
            }

            // Intermittent contact attenuation:
            _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
            uint16_t sputter_roll = _rng & 0x7FFu;

            if (_sputter_timer > 0) {
                _sputter_timer--;
                // Contact attenuation during burst
                reg.data.strobe_l = 0.85f;
                reg.data.strobe_r = 0.85f;
                reg.data.stuck_mask = 0;
            } else {
                reg.data.strobe_l = 1.0f;
                reg.data.strobe_r = 1.0f;
                reg.data.stuck_mask = 0;

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

            // Feedback headroom limit
            reg.feedback.gain = std::min(reg.feedback.gain, 0.85f);
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
        (void)ctx;
        float eff_amt = amount;

        if (eff_amt > 0.008f) {
            // Service Left dropout burst
            if (_drop_len_l > 0) {
                _drop_len_l--;
                reg.data.strobe_l = 0.70f;
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
                reg.data.strobe_r = 0.70f;
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
