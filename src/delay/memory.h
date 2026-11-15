#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <algorithm>
#include "dsp/stereo.h"
#include "tuning/tuning.h"

namespace bends {

class DelayMemory {
public:
    static constexpr size_t   kSize = tuning::kDelayFrames; // 1,048,576 frames (2^20)
    static constexpr uint32_t kMask = tuning::kDelayMask;   // 0x000FFFFF

    void Init(Stereo* sdram_buf) {
        _buffer = sdram_buf;
        _write_idx = 0;
        _write_frac = 0.0f;
        _clear_progress = kSize;
        _clearing = false;
        _rot_accum = 0.0f;
        _rng = 0x5EED7A4Cu;
        _prev_write_s = {0.0f, 0.0f};
        _prev_s0 = {0.0f, 0.0f};
        _prev_s1 = {0.0f, 0.0f};
        _prev_s2 = {0.0f, 0.0f};
        Clear();
    }

    void Clear() {
        _clearing = true;
        _clear_progress = 0;
        _write_idx = 0;
        _write_frac = 0.0f;
        _prev_write_s = {0.0f, 0.0f};
        _prev_s0 = {0.0f, 0.0f};
        _prev_s1 = {0.0f, 0.0f};
        _prev_s2 = {0.0f, 0.0f};
        if (_buffer) {
            std::memset(static_cast<void*>(&_buffer[0]), 0, 2048 * sizeof(Stereo));
            _clear_progress = 2048;
        }
    }

    void ServiceIncrementalClear(size_t chunk_frames = 1024) {
        if (!_buffer || !_clearing) return;

        size_t end = std::min(kSize, _clear_progress + chunk_frames);
        std::memset(static_cast<void*>(&_buffer[_clear_progress]), 0, (end - _clear_progress) * sizeof(Stereo));
        _clear_progress = end;
        if (_clear_progress >= kSize) {
            _clearing = false;
        }
    }

    inline float WritePos() const { return static_cast<float>(_write_idx) + _write_frac; }
    inline bool IsClearing() const { return _clearing; }

    __attribute__((always_inline)) inline void Write(Stereo s, bool write_enable, float rot_depth, float speed = 1.0f) {
        if (!_buffer || _clearing) return;
        s.l = daisysp::fclamp(s.l, -4.0f, 4.0f);
        s.r = daisysp::fclamp(s.r, -4.0f, 4.0f);

        // Low-voltage DRAM bit corruption & cell charge leakage
        if (rot_depth > 0.05f) {
            float leak = 1.0f - (rot_depth * 0.00015f);
            s.l *= leak;
            s.r *= leak;

            _rot_accum += rot_depth * 0.003f;
            if (_rot_accum >= 1.0f) {
                _rot_accum -= 1.0f;
                _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
                uint32_t bit_l = 4u + (_rng & 0x07);
                uint32_t bit_r = 4u + ((_rng >> 3) & 0x07);
                uint32_t ul = static_cast<uint32_t>(static_cast<int32_t>(daisysp::fclamp(s.l, -1.0f, 1.0f) * 32767.0f) + 32768) ^ (1 << bit_l);
                uint32_t ur = static_cast<uint32_t>(static_cast<int32_t>(daisysp::fclamp(s.r, -1.0f, 1.0f) * 32767.0f) + 32768) ^ (1 << bit_r);
                s.l = static_cast<float>(static_cast<int32_t>(ul) - 32768) * (1.0f / 32767.0f);
                s.r = static_cast<float>(static_cast<int32_t>(ur) - 32768) * (1.0f / 32767.0f);
            }
        }

        if (write_enable) {
            float eff_write_speed = std::clamp(speed, 0.02f, 2.05f);

            if (__builtin_expect(std::abs(eff_write_speed - 1.0f) < 0.001f, 1)) {
                _write_idx = (_write_idx + 1) & kMask;
                _buffer[_write_idx] = s;
                _write_frac = 0.0f;
                _prev_s2 = _prev_s1;
                _prev_s1 = _prev_s0;
                _prev_s0 = s;
                return;
            }

            if (eff_write_speed < 1.0f) {
                // S33 Left side: lo-fi downsampling decimation
                _write_frac += eff_write_speed;
                if (_write_frac >= 1.0f) {
                    _write_frac -= 1.0f;
                    _write_idx = (_write_idx + 1) & kMask;
                }
                _buffer[_write_idx] = s;
                _prev_s2 = _prev_s1;
                _prev_s1 = _prev_s0;
                _prev_s0 = s;
                return;
            }

            // S33 Right side (eff_write_speed > 1.0f):
            // Write pointer advances through memory faster than 48kHz incoming audio.
            // Advance write index in strictly monotonic chronological order,
            // interpolating smoothly between past input samples.
            float old_frac = _write_frac;
            _write_frac += eff_write_speed;
            int32_t step = static_cast<int32_t>(_write_frac);
            _write_frac -= static_cast<float>(step);

            uint32_t start_idx = _write_idx;
            float inv_speed = 1.0f / eff_write_speed;

            for (int32_t i = 1; i <= step; ++i) {
                uint32_t write_slot = (start_idx + i) & kMask;
                float frac = std::clamp((static_cast<float>(i) - old_frac) * inv_speed, 0.0f, 1.0f);
                _buffer[write_slot] = {
                    _prev_s0.l + frac * (s.l - _prev_s0.l),
                    _prev_s0.r + frac * (s.r - _prev_s0.r)
                };
            }
            _write_idx = (start_idx + step) & kMask;
            _prev_s2 = _prev_s1;
            _prev_s1 = _prev_s0;
            _prev_s0 = s;
        }
    }

    __attribute__((always_inline)) inline void Overdub(float abs_pos, Stereo s, float decay = 0.55f, float rot_depth = 0.0f) {
        if (!_buffer || _clearing) return;
        s.l = std::clamp(s.l, -4.0f, 4.0f);
        s.r = std::clamp(s.r, -4.0f, 4.0f);
        uint32_t idx = static_cast<uint32_t>(static_cast<int32_t>(abs_pos)) & kMask;
        Stereo old = _buffer[idx];

        // Low-voltage bit corruption & charge leakage during recirculation
        if (rot_depth > 0.02f) {
            float leak = 1.0f - (rot_depth * 0.00025f);
            old.l *= leak;
            old.r *= leak;

            _rot_accum += rot_depth * 0.006f;
            if (_rot_accum >= 1.0f) {
                _rot_accum -= 1.0f;
                _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
                uint32_t bit_l = 4u + (_rng & 0x07);
                uint32_t bit_r = 4u + ((_rng >> 3) & 0x07);
                uint32_t ul = static_cast<uint32_t>(static_cast<int32_t>(daisysp::fclamp(old.l, -1.0f, 1.0f) * 32767.0f) + 32768) ^ (1 << bit_l);
                uint32_t ur = static_cast<uint32_t>(static_cast<int32_t>(daisysp::fclamp(old.r, -1.0f, 1.0f) * 32767.0f) + 32768) ^ (1 << bit_r);
                old.l = static_cast<float>(static_cast<int32_t>(ul) - 32768) * (1.0f / 32767.0f);
                old.r = static_cast<float>(static_cast<int32_t>(ur) - 32768) * (1.0f / 32767.0f);
            }
        }

        // Constant-energy soft-saturated overdub mix:
        float in_g = 0.45f;
        float old_g = decay;
        _buffer[idx] = {
            FastSoftSat(s.l * in_g + old.l * old_g),
            FastSoftSat(s.r * in_g + old.r * old_g)
        };
    }

    __attribute__((always_inline)) inline Stereo ReadDistance(float distance_samples) const {
        if (!_buffer || _clearing) return {0.0f, 0.0f};
        int32_t dist_i = static_cast<int32_t>(distance_samples);
        float dist_f = distance_samples - static_cast<float>(dist_i);
        float frac = _write_frac - dist_f;
        int32_t i_pos = static_cast<int32_t>(_write_idx) - dist_i;
        if (frac < 0.0f) {
            frac += 1.0f;
            i_pos -= 1;
        }
        uint32_t idx0 = static_cast<uint32_t>(i_pos) & kMask;
        uint32_t idx1 = (idx0 + 1) & kMask;

        const Stereo& s0 = _buffer[idx0];
        const Stereo& s1 = _buffer[idx1];
        return {
            s0.l + frac * (s1.l - s0.l),
            s0.r + frac * (s1.r - s0.r)
        };
    }

    __attribute__((always_inline)) inline Stereo ReadDistanceSplit(float dist_l, float dist_r) const {
        if (!_buffer || _clearing) return {0.0f, 0.0f};
        int32_t dl_i = static_cast<int32_t>(dist_l);
        float dl_f = dist_l - static_cast<float>(dl_i);
        float frac_l = _write_frac - dl_f;
        int32_t pos_l = static_cast<int32_t>(_write_idx) - dl_i;
        if (frac_l < 0.0f) {
            frac_l += 1.0f;
            pos_l -= 1;
        }
        uint32_t idx_l0 = static_cast<uint32_t>(pos_l) & kMask;
        uint32_t idx_l1 = (idx_l0 + 1) & kMask;

        int32_t dr_i = static_cast<int32_t>(dist_r);
        float dr_f = dist_r - static_cast<float>(dr_i);
        float frac_r = _write_frac - dr_f;
        int32_t pos_r = static_cast<int32_t>(_write_idx) - dr_i;
        if (frac_r < 0.0f) {
            frac_r += 1.0f;
            pos_r -= 1;
        }
        uint32_t idx_r0 = static_cast<uint32_t>(pos_r) & kMask;
        uint32_t idx_r1 = (idx_r0 + 1) & kMask;

        const Stereo& sl0 = _buffer[idx_l0];
        const Stereo& sl1 = _buffer[idx_l1];
        const Stereo& sr0 = _buffer[idx_r0];
        const Stereo& sr1 = _buffer[idx_r1];

        return {
            sl0.l + frac_l * (sl1.l - sl0.l),
            sr0.r + frac_r * (sr1.r - sr0.r)
        };
    }

    __attribute__((always_inline)) inline float ReadDistanceL(float distance_samples) const {
        if (!_buffer || _clearing) return 0.0f;
        int32_t dist_i = static_cast<int32_t>(distance_samples);
        float dist_f = distance_samples - static_cast<float>(dist_i);
        float frac = _write_frac - dist_f;
        int32_t i_pos = static_cast<int32_t>(_write_idx) - dist_i;
        if (frac < 0.0f) {
            frac += 1.0f;
            i_pos -= 1;
        }
        uint32_t idx0 = static_cast<uint32_t>(i_pos) & kMask;
        uint32_t idx1 = (idx0 + 1) & kMask;
        return _buffer[idx0].l + frac * (_buffer[idx1].l - _buffer[idx0].l);
    }

    __attribute__((always_inline)) inline float ReadDistanceR(float distance_samples) const {
        if (!_buffer || _clearing) return 0.0f;
        int32_t dist_i = static_cast<int32_t>(distance_samples);
        float dist_f = distance_samples - static_cast<float>(dist_i);
        float frac = _write_frac - dist_f;
        int32_t i_pos = static_cast<int32_t>(_write_idx) - dist_i;
        if (frac < 0.0f) {
            frac += 1.0f;
            i_pos -= 1;
        }
        uint32_t idx0 = static_cast<uint32_t>(i_pos) & kMask;
        uint32_t idx1 = (idx0 + 1) & kMask;
        return _buffer[idx0].r + frac * (_buffer[idx1].r - _buffer[idx0].r);
    }

    __attribute__((always_inline)) inline Stereo ReadAbsolute(float abs_pos) const {
        if (!_buffer || _clearing) return {0.0f, 0.0f};

        int32_t i_pos = static_cast<int32_t>(abs_pos);
        float frac = abs_pos - static_cast<float>(i_pos);
        if (frac < 0.0f) {
            frac += 1.0f;
            i_pos -= 1;
        }
        uint32_t idx0 = static_cast<uint32_t>(i_pos) & kMask;
        uint32_t idx1 = (idx0 + 1) & kMask;

        const Stereo& s0 = _buffer[idx0];
        const Stereo& s1 = _buffer[idx1];

        return {
            s0.l + frac * (s1.l - s0.l),
            s0.r + frac * (s1.r - s0.r)
        };
    }

private:
    Stereo*  _buffer{nullptr};
    uint32_t _write_idx{0};
    float    _write_frac{0.0f};
    size_t   _clear_progress{kSize};
    bool     _clearing{false};
    float    _rot_accum{0.0f};
    uint32_t _rng{0x5EED7A4Cu};
    Stereo   _prev_write_s{0.0f, 0.0f};
    Stereo   _prev_s0{0.0f, 0.0f};
    Stereo   _prev_s1{0.0f, 0.0f};
    Stereo   _prev_s2{0.0f, 0.0f};
};

} // namespace bends
