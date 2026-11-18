#pragma once

#include <cmath>
#include <algorithm>
#include "dsp/fast_math.h"
#include "delay/memory.h"
#include "delay/registers.h"
#include "tuning/tuning.h"

namespace bends {

class ReadHeadPair {
public:
    static constexpr float kInvXfadeSamples = 1.0f / static_cast<float>(tuning::kXfadeSamples);

    void Init() {
        _head_a = tuning::kMinDelaySamples;
        _head_b = tuning::kMinDelaySamples;
        _anchor_a = tuning::kMinDelaySamples;
        _anchor_b = tuning::kMinDelaySamples;
        _len_a = 4800.0f;
        _inv_len_a = 1.0f / 4800.0f;
        _len_b = 4800.0f;
        _inv_len_b = 1.0f / 4800.0f;
        _phase_a = 0.0f;
        _phase_b = 0.0f;
        _active_head = 0;
        _xfade_active = false;
        _xfade_progress = 0.0f;
        _xfade_dir = 1;
        _xfade_step = kInvXfadeSamples;
        _target_samples = tuning::kMinDelaySamples;
        _smooth_mod = 0.0f;
        _was_looping = false;
        _unfreeze_xfade = 0.0f;
        _unfreeze_last = Stereo(0.0f, 0.0f);
        _bode_phase = 0.0f;
        _bode_mix = 0.0f;
        _bode_base_d = tuning::kMinDelaySamples;
        _smooth_anchor = 0.0f;
        _smooth_split = 0.0f;
        _smooth_skew = 0.0f;
        _karplus_d = tuning::kMinDelaySamples;
        _pending_jump = false;
        _pending_jump_tgt = 0.0f;
        _pending_jump_fade = tuning::kXfadeSamples;
        _pending_jump_loop = 0.0f;
        _initialized = false;
        std::memset(_spread_buf, 0, sizeof(_spread_buf));
        _spread_w = 0;
    }

    void SetTargetSamples(float target) {
        _target_samples = std::clamp(target, tuning::kMinDelaySamples, static_cast<float>(DelayMemory::kSize - 2));
    }

    void PrepareBlock(const TimeRegisters& time_reg, float target_samples, float read_speed = 1.0f) {
        _target_samples = std::clamp(target_samples, tuning::kMinDelaySamples, static_cast<float>(DelayMemory::kSize - 2));

        if (!_initialized) {
            _initialized = true;
            _head_a = _target_samples;
            _head_b = _target_samples;
            _karplus_d = _target_samples;
            _karplus_inc = 0.0f;
            _anchor_a = _target_samples;
            _anchor_b = _target_samples;
            _active_head = 0;
            _xfade_active = false;
            _xfade_progress = 0.0f;
            _xfade_dir = 1;
            _xfade_step = kInvXfadeSamples;
        }

        // Detect loop state transitions
        if (!_was_looping && time_reg.has_loop) {
            float current_head = (_active_head == 0) ? _head_a : _head_b;
            _anchor_a = (time_reg.loop_anchor > 0) ? static_cast<float>(time_reg.loop_anchor) : 0.0f;
            _anchor_b = _anchor_a;
            _len_a = std::max(48.0f, time_reg.loop_len);
            _inv_len_a = 1.0f / _len_a;
            _seam_len_a = std::clamp(_len_a * 0.08f, 16.0f, 64.0f);
            _inv_seam_len_a = 1.0f / _seam_len_a;
            _len_b = _len_a;
            _inv_len_b = _inv_len_a;
            _seam_len_b = _seam_len_a;
            _inv_seam_len_b = _inv_seam_len_a;
            float current_dist = (time_reg.loop_anchor > 0) ? (current_head - _anchor_a) : current_head;
            current_dist = daisysp::fclamp(current_dist, 0.0f, _len_a);
            _phase_a = _len_a - current_dist;
            _phase_b = _phase_a;
            _head_a = current_head;
            _head_b = current_head;
            _target_samples = _len_a;
        } else if (_was_looping && !time_reg.has_loop) {
            _was_looping = false;
            _smooth_anchor = 0.0f;
            _anchor_a = 0.0f;
            _anchor_b = 0.0f;
            Jump(_target_samples, 800);
        }
        _was_looping = time_reg.has_loop;

        // Short delay range (< 50ms): continuous per-sample slew
        if (_target_samples < tuning::kKarplusMaxSamples && !time_reg.has_loop) {
            if (_active_head != 0) {
                _head_a = _head_b;
                _karplus_d = _head_a;
                _active_head = 0;
            }
            _karplus_d += 0.12f * (_target_samples - _karplus_d);
            _karplus_inc = (_karplus_d - _head_a) * (1.0f / 64.0f);
            _xfade_active = false;
            _xfade_progress = 0.0f;
        } else {
            _karplus_inc = 0.0f;
            _karplus_d = (_active_head == 0) ? _head_a : _head_b;
            if (_head_a < tuning::kKarplusMaxSamples && _head_b < tuning::kKarplusMaxSamples) {
                _head_a = _karplus_d;
                _head_b = _karplus_d;
            }
            // Dual-head crossfade (>= 50ms):
            float active_val = time_reg.has_loop ? ((_active_head == 0) ? _len_a : _len_b)
                                                 : ((_active_head == 0) ? _head_a : _head_b);
            float move_thresh = std::max(160.0f, active_val * 0.02f);

            if (!_xfade_active) {
                bool trigger_xfade = false;
                if (time_reg.has_loop) {
                    float target_val = std::max(48.0f, time_reg.loop_len);
                    float active_anchor = (_active_head == 0) ? _anchor_a : _anchor_b;
                    float target_anchor = (time_reg.loop_anchor > 0) ? static_cast<float>(time_reg.loop_anchor) : 0.0f;
                    float anchor_thresh = std::max(128.0f, active_val * 0.035f);
                    if (std::abs(target_val - active_val) > move_thresh ||
                        std::abs(target_anchor - active_anchor) > anchor_thresh) {
                        trigger_xfade = true;
                    }
                } else {
                    if (read_speed >= -0.01f && std::abs(_target_samples - active_val) > move_thresh) {
                        trigger_xfade = true;
                    }
                }

                if (trigger_xfade) {
                    if (_active_head == 0) {
                        if (time_reg.has_loop) {
                            float target_val = std::max(48.0f, time_reg.loop_len);
                            float target_anchor = (time_reg.loop_anchor > 0) ? static_cast<float>(time_reg.loop_anchor) : 0.0f;
                            _anchor_b = target_anchor;
                            _len_b = target_val;
                            _inv_len_b = 1.0f / _len_b;
                            _seam_len_b = std::clamp(_len_b * 0.08f, 16.0f, 64.0f);
                            _inv_seam_len_b = 1.0f / _seam_len_b;
                            float dist = _head_a - _anchor_b;
                            _phase_b = FastWrap(_len_b - dist, _len_b, _inv_len_b);
                        } else {
                            _head_b = _target_samples;
                        }
                        _xfade_active = true;
                        _xfade_dir = 1;
                        _xfade_progress = 0.0f;
                        _xfade_step = kInvXfadeSamples;
                    } else {
                        if (time_reg.has_loop) {
                            float target_val = std::max(48.0f, time_reg.loop_len);
                            float target_anchor = (time_reg.loop_anchor > 0) ? static_cast<float>(time_reg.loop_anchor) : 0.0f;
                            _anchor_a = target_anchor;
                            _len_a = target_val;
                            _inv_len_a = 1.0f / _len_a;
                            _seam_len_a = std::clamp(_len_a * 0.08f, 16.0f, 64.0f);
                            _inv_seam_len_a = 1.0f / _seam_len_a;
                            float dist = _head_b - _anchor_a;
                            _phase_a = FastWrap(_len_a - dist, _len_a, _inv_len_a);
                        } else {
                            _head_a = _target_samples;
                        }
                        _xfade_active = true;
                        _xfade_dir = -1;
                        _xfade_progress = 1.0f;
                        _xfade_step = kInvXfadeSamples;
                    }
                }
            }
        }

        // Precompute Bode shift constants if active
        if (time_reg.bode_active) {
            _bode_mix += 0.35f * (1.0f - _bode_mix);
        } else if (_bode_mix > 0.001f) {
            _bode_mix += 0.35f * (0.0f - _bode_mix);
            if (_bode_mix < 0.001f) _bode_mix = 0.0f;
        }

        if (time_reg.bode_active || _bode_mix > 0.001f) {
            float ratio = FastPow2(time_reg.bode_shift * (1.0f / 12.0f));
            _bode_speed_delta = ratio - 1.0f;
            constexpr float kBodeWindow = 1536.0f;
            _bode_d_phi = std::abs(_bode_speed_delta) * (1.0f / kBodeWindow);
        }

        // Stereo spread smoothing and precalculation
        _smooth_split += 0.27f * (time_reg.stereo_split - _smooth_split);
        _smooth_skew  += 0.27f * (time_reg.skew - _smooth_skew);
        _has_stereo_split = (_smooth_split > 0.002f);
        _has_skew = (_smooth_skew > 0.5f);
        _has_stereo_spread = _has_stereo_split || _has_skew;
        _cached_haas_offset = _smooth_split * 576.0f;
    }

    void Jump(float target_samples, uint16_t fade_samples = tuning::kXfadeSamples, float loop_len = 0.0f) {
        float tgt = std::clamp(target_samples, 0.0f, static_cast<float>(DelayMemory::kSize - 2));
        _target_samples = tgt;
        fade_samples = std::max<uint16_t>(192, fade_samples);
        float new_len = (loop_len > 48.0f) ? loop_len : ((_active_head == 0) ? _len_a : _len_b);

        if (_xfade_active) {
            // If crossfade is in progress, queue as pending jump
            if (_xfade_progress >= 0.15f && _xfade_progress <= 0.85f) {
                _pending_jump = true;
                _pending_jump_tgt = tgt;
                _pending_jump_fade = fade_samples;
                _pending_jump_loop = loop_len;
                return;
            }

            // Otherwise, we are near the edge (p < 0.15 or p > 0.85), so one head is practically silent.
            // Retarget the near-silent head and reverse or continue direction toward it:
            if (_xfade_dir > 0) { // moving toward head_b (gain_b rising, gain_a falling)
                if (_xfade_progress < 0.15f) {
                    // Head B is quiet; retarget Head B
                    _anchor_b = tgt;
                    if (loop_len > 48.0f) { _len_b = loop_len; _inv_len_b = 1.0f / _len_b; }
                    _phase_b = 0.0f;
                    _head_b = tgt;
                    _xfade_step = 1.0f / static_cast<float>(fade_samples);
                } else {
                    // Head A is quiet; retarget Head A and reverse toward A
                    _anchor_a = tgt;
                    if (loop_len > 48.0f) { _len_a = loop_len; _inv_len_a = 1.0f / _len_a; }
                    _phase_a = 0.0f;
                    _head_a = tgt;
                    _xfade_dir = -1;
                    _xfade_step = 1.0f / static_cast<float>(fade_samples);
                }
            } else { // moving toward head_a (gain_a rising, gain_b falling)
                if (_xfade_progress > 0.85f) {
                    // Head A is quiet; retarget Head A
                    _anchor_a = tgt;
                    if (loop_len > 48.0f) { _len_a = loop_len; _inv_len_a = 1.0f / _len_a; }
                    _phase_a = 0.0f;
                    _head_a = tgt;
                    _xfade_step = 1.0f / static_cast<float>(fade_samples);
                } else {
                    // Head B is quiet; retarget Head B and reverse toward B
                    _anchor_b = tgt;
                    if (loop_len > 48.0f) { _len_b = loop_len; _inv_len_b = 1.0f / _len_b; }
                    _phase_b = 0.0f;
                    _head_b = tgt;
                    _xfade_dir = 1;
                    _xfade_step = 1.0f / static_cast<float>(fade_samples);
                }
            }
        } else {
            _pending_jump = false;
            _xfade_step = 1.0f / static_cast<float>(fade_samples);
            if (_active_head == 0) {
                _anchor_b = tgt;
                _len_b = new_len;
                _inv_len_b = 1.0f / _len_b;
                _phase_b = 0.0f;
                _head_b = tgt;
                _xfade_active = true;
                _xfade_dir = 1;
                _xfade_progress = 0.0f;
            } else {
                _anchor_a = tgt;
                _len_a = new_len;
                _inv_len_a = 1.0f / _len_a;
                _phase_a = 0.0f;
                _head_a = tgt;
                _xfade_active = true;
                _xfade_dir = -1;
                _xfade_progress = 1.0f;
            }
        }
    }

    __attribute__((always_inline)) static inline float FastWrap(float x, float len, float inv_len) {
        if (x >= 0.0f && x < len) return x;
        int32_t ik = static_cast<int32_t>(x * inv_len);
        if (x < 0.0f) ik -= 1;
        float w = x - static_cast<float>(ik) * len;
        if (w >= len) w -= len;
        if (w < 0.0f) w += len;
        return w;
    }

    __attribute__((always_inline)) inline Stereo Process(const DelayMemory& mem,
                                                         float read_speed,
                                                         float mod_offset,
                                                         const TimeRegisters& time_reg) {
        // Anti-crackle capacitive lag filter on head modulation (tracks audio-rate sweeps smoothly)
        constexpr float kModSlew = 0.08f;
        _smooth_mod += kModSlew * (mod_offset - _smooth_mod);

        // Loop window phase advancement
        if (time_reg.has_loop) {
            float active_len = (_active_head == 0) ? _len_a : _len_b;
            if (__builtin_expect(active_len > 48.0f, 1)) {
                if (_xfade_active) {
                    _phase_a += read_speed;
                    if (__builtin_expect(_phase_a >= _len_a, 0)) _phase_a -= _len_a;
                    else if (__builtin_expect(_phase_a < 0.0f, 0)) _phase_a += _len_a;

                    _phase_b += read_speed;
                    if (__builtin_expect(_phase_b >= _len_b, 0)) _phase_b -= _len_b;
                    else if (__builtin_expect(_phase_b < 0.0f, 0)) _phase_b += _len_b;

                    _head_a = _anchor_a + _len_a - _phase_a;
                    _head_b = _anchor_b + _len_b - _phase_b;
                } else if (_active_head == 0) {
                    _phase_a += read_speed;
                    if (__builtin_expect(_phase_a >= _len_a, 0)) _phase_a -= _len_a;
                    else if (__builtin_expect(_phase_a < 0.0f, 0)) _phase_a += _len_a;

                    _head_a = _anchor_a + _len_a - _phase_a;
                } else {
                    _phase_b += read_speed;
                    if (__builtin_expect(_phase_b >= _len_b, 0)) _phase_b -= _len_b;
                    else if (__builtin_expect(_phase_b < 0.0f, 0)) _phase_b += _len_b;

                    _head_b = _anchor_b + _len_b - _phase_b;
                }
            }
        } else if (read_speed < 0.0f) {
            // Non-looping continuous reverse playback through circular delay memory history
            constexpr float kMaxDelay = static_cast<float>(DelayMemory::kSize - 2);
            float step = -read_speed;
            _head_a += step;
            _head_b += step;
            if (_head_a >= kMaxDelay) _head_a -= (kMaxDelay - tuning::kMinDelaySamples);
            if (_head_b >= kMaxDelay) _head_b -= (kMaxDelay - tuning::kMinDelaySamples);
        } else if (__builtin_expect(_karplus_inc != 0.0f, 0)) {
            _head_a += _karplus_inc;
            _head_b = _head_a;
        }


        // 1. Bode Frequency Shifter Grain Reader:
        Stereo bode_out(0.0f, 0.0f);
        if (__builtin_expect(_bode_mix > 0.001f, 0)) {
            constexpr float kBodeWindow = 1536.0f;
            _bode_phase += _bode_d_phi;
            if (_bode_phase >= 1.0f) _bode_phase -= 1.0f;

            float phi_a = _bode_phase;
            float phi_b = phi_a + 0.5f;
            if (phi_b >= 1.0f) phi_b -= 1.0f;

            constexpr float kMaxDelay = static_cast<float>(DelayMemory::kSize - kBodeWindow - 2);
            float cur_head = (_active_head == 0) ? _head_a : _head_b;
            float raw_base_d = daisysp::fclamp(time_reg.has_loop ? cur_head : _target_samples,
                                               tuning::kMinDelaySamples, kMaxDelay);
            if (_bode_mix < 0.02f) {
                _bode_base_d = raw_base_d;
            } else {
                _bode_base_d += 0.015f * (raw_base_d - _bode_base_d);
            }
            float base_d = _bode_base_d;

            float pos_a, pos_b;
            if (_bode_speed_delta >= 0.0f) {
                pos_a = base_d + (1.0f - phi_a) * kBodeWindow + _smooth_mod;
                pos_b = base_d + (1.0f - phi_b) * kBodeWindow + _smooth_mod;
            } else {
                pos_a = base_d + phi_a * kBodeWindow + _smooth_mod;
                pos_b = base_d + phi_b * kBodeWindow + _smooth_mod;
            }

            Stereo sa = mem.ReadDistance(pos_a);
            Stereo sb = mem.ReadDistance(pos_b);

            float s_a = FastSin(phi_a * kPi);
            float ga = s_a * s_a;
            float gb = 1.0f - ga;

            bode_out = Stereo(sa.l * ga + sb.l * gb,
                              sa.r * ga + sb.r * gb);
        }

        // 2. Normal Delay Reader:
        Stereo normal_out(0.0f, 0.0f);
        if (__builtin_expect(_bode_mix < 0.999f, 1)) {
            constexpr float kMaxDelay = static_cast<float>(DelayMemory::kSize - 2);
            float eff_mod = _smooth_mod;

            if (_xfade_active) {
                _xfade_progress += static_cast<float>(_xfade_dir) * _xfade_step;
                if (_xfade_dir > 0 && _xfade_progress >= 1.0f) {
                    _xfade_progress = 1.0f;
                    _xfade_active = false;
                    _active_head = 1;
                    if (_pending_jump) {
                        _pending_jump = false;
                        _anchor_a = _pending_jump_tgt;
                        if (_pending_jump_loop > 48.0f) {
                            _len_a = _pending_jump_loop;
                            _inv_len_a = 1.0f / _len_a;
                            _seam_len_a = std::clamp(_len_a * 0.08f, 16.0f, 64.0f);
                            _inv_seam_len_a = 1.0f / _seam_len_a;
                            _phase_a = 0.0f;
                        }
                        _head_a = _pending_jump_tgt;
                        _target_samples = _pending_jump_tgt;
                        _xfade_active = true;
                        _xfade_dir = -1;
                        _xfade_step = 1.0f / static_cast<float>(_pending_jump_fade);
                    } else {
                        float curr = time_reg.has_loop ? _len_b : _head_b;
                        float move_thresh = std::max(160.0f, curr * 0.02f);
                        if (std::abs(_target_samples - curr) > move_thresh && _target_samples >= tuning::kKarplusMaxSamples) {
                            if (time_reg.has_loop) {
                                _anchor_a = (time_reg.loop_anchor > 0) ? static_cast<float>(time_reg.loop_anchor) : 0.0f;
                                _len_a = std::max(48.0f, _target_samples);
                                _inv_len_a = 1.0f / _len_a;
                                _seam_len_a = std::clamp(_len_a * 0.08f, 16.0f, 64.0f);
                                _inv_seam_len_a = 1.0f / _seam_len_a;
                                _phase_a = FastWrap(_phase_b, _len_a, _inv_len_a);
                            } else {
                                _head_a = _target_samples;
                            }
                            _xfade_active = true;
                            _xfade_dir = -1;
                            _xfade_step = kInvXfadeSamples;
                        }
                    }
                } else if (_xfade_dir < 0 && _xfade_progress <= 0.0f) {
                    _xfade_progress = 0.0f;
                    _xfade_active = false;
                    _active_head = 0;
                    if (_pending_jump) {
                        _pending_jump = false;
                        _anchor_b = _pending_jump_tgt;
                        if (_pending_jump_loop > 48.0f) {
                            _len_b = _pending_jump_loop;
                            _inv_len_b = 1.0f / _len_b;
                            _seam_len_b = std::clamp(_len_b * 0.08f, 16.0f, 64.0f);
                            _inv_seam_len_b = 1.0f / _seam_len_b;
                            _phase_b = 0.0f;
                        }
                        _head_b = _pending_jump_tgt;
                        _target_samples = _pending_jump_tgt;
                        _xfade_active = true;
                        _xfade_dir = 1;
                        _xfade_step = 1.0f / static_cast<float>(_pending_jump_fade);
                    } else {
                        float curr = time_reg.has_loop ? _len_a : _head_a;
                        float move_thresh = std::max(160.0f, curr * 0.02f);
                        if (std::abs(_target_samples - curr) > move_thresh && _target_samples >= tuning::kKarplusMaxSamples) {
                            if (time_reg.has_loop) {
                                _anchor_b = (time_reg.loop_anchor > 0) ? static_cast<float>(time_reg.loop_anchor) : 0.0f;
                                _len_b = std::max(48.0f, _target_samples);
                                _inv_len_b = 1.0f / _len_b;
                                _seam_len_b = std::clamp(_len_b * 0.08f, 16.0f, 64.0f);
                                _inv_seam_len_b = 1.0f / _seam_len_b;
                                _phase_b = FastWrap(_phase_a, _len_b, _inv_len_b);
                            } else {
                                _head_b = _target_samples;
                            }
                            _xfade_active = true;
                            _xfade_dir = 1;
                            _xfade_step = kInvXfadeSamples;
                        }
                    }
                }
                float p = std::clamp(_xfade_progress, 0.0f, 1.0f);
                float q = 1.0f - p;
                float gain_b = p * (1.5707963f - 0.5707963f * (p * p));
                float gain_a = q * (1.5707963f - 0.5707963f * (q * q));
                if (gain_b < 0.015f) {
                    float pos_a = std::clamp(_head_a + eff_mod, tuning::kMinDelaySamples, kMaxDelay);
                    normal_out = mem.ReadDistance(pos_a);
                } else if (gain_a < 0.015f) {
                    float pos_b = std::clamp(_head_b + eff_mod, tuning::kMinDelaySamples, kMaxDelay);
                    normal_out = mem.ReadDistance(pos_b);
                } else {
                    float pos_a = std::clamp(_head_a + eff_mod, tuning::kMinDelaySamples, kMaxDelay);
                    float pos_b = std::clamp(_head_b + eff_mod, tuning::kMinDelaySamples, kMaxDelay);
                    Stereo sa = mem.ReadDistance(pos_a);
                    Stereo sb = mem.ReadDistance(pos_b);
                    normal_out = sa * gain_a + sb * gain_b;
                }
                if (__builtin_expect(_has_stereo_spread, 0)) {
                    float spread_d = _has_stereo_split ? _cached_haas_offset : _smooth_skew;
                    normal_out.r = ProcessSpread(normal_out.r, spread_d);
                }
            } else {
                float pos_base = (_active_head == 0) ? _head_a : _head_b;
                float pos = std::clamp(pos_base + eff_mod, tuning::kMinDelaySamples, kMaxDelay);
                normal_out = mem.ReadDistance(pos);
                if (__builtin_expect(_has_stereo_spread, 0)) {
                    float spread_d = _has_stereo_split ? _cached_haas_offset : _smooth_skew;
                    normal_out.r = ProcessSpread(normal_out.r, spread_d);
                }
            }
        }

        // Crossfade normal delay with Bode grain reader
        Stereo out;
        if (_bode_mix <= 0.001f) {
            out = normal_out;
        } else if (_bode_mix >= 0.999f) {
            out = bode_out;
        } else {
            out = normal_out * (1.0f - _bode_mix) + bode_out * _bode_mix;
        }

        // Smoothstep seam crossfade in looping mode
        if (time_reg.has_loop) {
            float active_phase = (_active_head == 0) ? _phase_a : _phase_b;
            float active_len = (_active_head == 0) ? _len_a : _len_b;
            if (__builtin_expect(active_len > 48.0f, 1)) {
                float seam_len = (_active_head == 0) ? _seam_len_a : _seam_len_b;
                float inv_seam_len = (_active_head == 0) ? _inv_seam_len_a : _inv_seam_len_b;
                if (__builtin_expect(active_phase < seam_len, 0)) {
                    float norm = std::clamp(active_phase * inv_seam_len, 0.0f, 1.0f);
                    out = out * (norm * norm * (3.0f - 2.0f * norm));
                } else if (__builtin_expect(active_phase > (active_len - seam_len), 0)) {
                    float norm = std::clamp((active_len - active_phase) * inv_seam_len, 0.0f, 1.0f);
                    out = out * (norm * norm * (3.0f - 2.0f * norm));
                }
            }
            _unfreeze_last = out;
            _unfreeze_xfade = 1.0f;
        } else if (__builtin_expect(_unfreeze_xfade > 0.001f, 0)) {
            // Smooth crossfade back to live audio when unfreezing
            out = out * (1.0f - _unfreeze_xfade) + _unfreeze_last * _unfreeze_xfade;
            _unfreeze_xfade *= 0.96f;
        }

        return out;
    }

    __attribute__((always_inline)) inline float ActiveReadPos(float write_pos, const TimeRegisters& /*time_reg*/) const {
        float head = (_active_head == 0) ? _head_a : _head_b;
        float pos = head + _smooth_mod;
        return write_pos - pos;
    }

    inline bool IsXfadeActive() const { return _xfade_active; }
    inline float XfadeProgress() const { return _xfade_progress; }
    inline float HeadA() const { return _head_a; }
    inline float HeadB() const { return _head_b; }

private:
    float    _head_a{tuning::kMinDelaySamples};
    float    _head_b{tuning::kMinDelaySamples};
    uint8_t  _active_head{0};
    bool     _xfade_active{false};
    float    _xfade_progress{0.0f};
    int8_t   _xfade_dir{1};
    float    _xfade_step{0.0006f};
    float    _target_samples{tuning::kMinDelaySamples};
    float    _smooth_mod{0.0f};
    bool     _was_looping{false};
    float    _unfreeze_xfade{0.0f};
    Stereo   _unfreeze_last{0.0f, 0.0f};
    float    _bode_phase{0.0f};
    float    _bode_mix{0.0f};
    float    _anchor_a{tuning::kMinDelaySamples};
    float    _anchor_b{tuning::kMinDelaySamples};
    float    _len_a{4800.0f};
    float    _inv_len_a{1.0f / 4800.0f};
    float    _seam_len_a{64.0f};
    float    _inv_seam_len_a{1.0f / 64.0f};
    float    _len_b{4800.0f};
    float    _inv_len_b{1.0f / 4800.0f};
    float    _seam_len_b{64.0f};
    float    _inv_seam_len_b{1.0f / 64.0f};
    float    _phase_a{0.0f};
    float    _phase_b{0.0f};
    float    _bode_base_d{tuning::kMinDelaySamples};
    float    _smooth_anchor{0.0f};
    float    _smooth_split{0.0f};
    float    _smooth_skew{0.0f};
    float    _bode_speed_delta{0.0f};
    float    _bode_d_phi{0.0f};
    bool     _has_stereo_split{false};
    bool     _has_skew{false};
    bool     _has_stereo_spread{false};
    float    _cached_haas_offset{0.0f};
    float    _split_ratio{1.0f};
    float    _split_offset{0.0f};
    bool     _initialized{false};
    float    _karplus_d{tuning::kMinDelaySamples};
    float    _karplus_inc{0.0f};
    bool     _pending_jump{false};
    float    _pending_jump_tgt{0.0f};
    uint16_t _pending_jump_fade{tuning::kXfadeSamples};
    float    _pending_jump_loop{0.0f};

    static constexpr size_t kSpreadBufSize = 1024;
    float    _spread_buf[kSpreadBufSize];
    uint16_t _spread_w{0};

    __attribute__((always_inline)) inline float ProcessSpread(float in_r, float delay_samples) {
        float ds = std::clamp(delay_samples, 0.0f, static_cast<float>(kSpreadBufSize - 2));
        _spread_buf[_spread_w] = in_r;
        float r_pos = static_cast<float>(_spread_w) - ds;
        if (__builtin_expect(r_pos < 0.0f, 0)) r_pos += static_cast<float>(kSpreadBufSize);
        uint32_t idx0 = static_cast<uint32_t>(r_pos) & (kSpreadBufSize - 1);
        uint32_t idx1 = (idx0 + 1) & (kSpreadBufSize - 1);
        float frac = r_pos - static_cast<float>(static_cast<uint32_t>(r_pos));
        _spread_w = (_spread_w + 1) & (kSpreadBufSize - 1);
        return _spread_buf[idx0] + frac * (_spread_buf[idx1] - _spread_buf[idx0]);
    }
};

} // namespace bends
