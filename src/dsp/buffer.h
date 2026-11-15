#pragma once

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include "dsp/fast_math.h"

namespace bends {

struct StereoSample {
    float l;
    float r;
};

class StereoDelayBuffer {
public:
    void Init(StereoSample* buf, size_t max_size) {
        _buf = buf;
        _max_size = max_size;
        _write_pos = 0.0f;
        _written_distance = static_cast<float>(_max_size);
        _clear_idx = 0;
        _clear_remaining = 0;
    }

    void Clear() {
        _write_pos = 0.0f;
        _written_distance = static_cast<float>(_max_size);
        // Fast synchronous wipe of immediate 4,096 samples (~85ms) to kill active audio without ISR stalls
        if (_buf && _max_size > 0) {
            size_t wipe = std::min(_max_size, static_cast<size_t>(4096));
            std::fill_n(_buf, wipe, StereoSample{0.0f, 0.0f});
            _clear_idx = wipe;
            _clear_remaining = (_max_size > wipe) ? (_max_size - wipe) : 0;
        } else {
            _clear_idx = 0;
            _clear_remaining = 0;
        }
    }

    inline void ServiceIncrementalClear(size_t chunk_size = 256) {
        if (_clear_remaining > 0 && _buf) {
            size_t to_clear = std::min(chunk_size, _clear_remaining);
            if (_clear_idx + to_clear <= _max_size) {
                std::fill_n(&_buf[_clear_idx], to_clear, StereoSample{0.0f, 0.0f});
                _clear_idx += to_clear;
            } else {
                size_t part1 = _max_size - _clear_idx;
                std::fill_n(&_buf[_clear_idx], part1, StereoSample{0.0f, 0.0f});
                size_t part2 = to_clear - part1;
                std::fill_n(&_buf[0], part2, StereoSample{0.0f, 0.0f});
                _clear_idx = part2;
            }
            _clear_remaining -= to_clear;
        }
    }

    __attribute__((always_inline)) inline void ReadStereo(float delay_samples, float& out_l, float& out_r) const {
        if (!_buf || _max_size == 0) {
            out_l = 0.0f; out_r = 0.0f;
            return;
        }
        float d = std::clamp(delay_samples, 1.0f, static_cast<float>(_max_size - 2));
        float max_d = static_cast<float>(_max_size);

        float read_pos = _write_pos - d;
        if (read_pos < 0.0f) read_pos += max_d;
        else if (read_pos >= max_d) read_pos -= max_d;

        size_t idx0 = static_cast<size_t>(read_pos);
        if (idx0 >= _max_size) idx0 = 0;
        size_t idx1 = (idx0 + 1 < _max_size) ? idx0 + 1 : 0;
        float frac = read_pos - static_cast<float>(idx0);

        const StereoSample& s0 = _buf[idx0];
        const StereoSample& s1 = _buf[idx1];
        out_l = s0.l + frac * (s1.l - s0.l);
        out_r = s0.r + frac * (s1.r - s0.r);
    }

    __attribute__((always_inline)) inline void ReadStereoSplit(float delay_l, float delay_r, float& out_l, float& out_r) const {
        if (!_buf || _max_size == 0) {
            out_l = 0.0f; out_r = 0.0f;
            return;
        }
        float max_d = static_cast<float>(_max_size);

        float dl = std::clamp(delay_l, 1.0f, max_d - 2.0f);
        float rpos_l = _write_pos - dl;
        if (rpos_l < 0.0f) rpos_l += max_d;
        else if (rpos_l >= max_d) rpos_l -= max_d;
        size_t idx0_l = static_cast<size_t>(rpos_l);
        if (idx0_l >= _max_size) idx0_l = 0;
        size_t idx1_l = (idx0_l + 1 < _max_size) ? idx0_l + 1 : 0;
        float frac_l = rpos_l - static_cast<float>(idx0_l);
        out_l = _buf[idx0_l].l + frac_l * (_buf[idx1_l].l - _buf[idx0_l].l);

        float dr = std::clamp(delay_r, 1.0f, max_d - 2.0f);
        float rpos_r = _write_pos - dr;
        if (rpos_r < 0.0f) rpos_r += max_d;
        else if (rpos_r >= max_d) rpos_r -= max_d;
        size_t idx0_r = static_cast<size_t>(rpos_r);
        if (idx0_r >= _max_size) idx0_r = 0;
        size_t idx1_r = (idx0_r + 1 < _max_size) ? idx0_r + 1 : 0;
        float frac_r = rpos_r - static_cast<float>(idx0_r);
        out_r = _buf[idx0_r].r + frac_r * (_buf[idx1_r].r - _buf[idx0_r].r);
    }

    __attribute__((always_inline)) inline float ReadPosL(float pos) const {
        if (!_buf || _max_size == 0) return 0.0f;
        float max_d = static_cast<float>(_max_size);
        if (pos >= max_d) {
            pos -= max_d;
            if (pos >= max_d) pos = 0.0f;
        } else if (pos < 0.0f) {
            pos += max_d;
            if (pos < 0.0f) pos = 0.0f;
        }
        size_t idx0 = static_cast<size_t>(pos);
        if (idx0 >= _max_size) idx0 = 0;
        size_t idx1 = (idx0 + 1 < _max_size) ? idx0 + 1 : 0;
        float frac = pos - static_cast<float>(idx0);
        return _buf[idx0].l + frac * (_buf[idx1].l - _buf[idx0].l);
    }

    __attribute__((always_inline)) inline float ReadPosR(float pos) const {
        if (!_buf || _max_size == 0) return 0.0f;
        float max_d = static_cast<float>(_max_size);
        if (pos >= max_d) {
            pos -= max_d;
            if (pos >= max_d) pos = 0.0f;
        } else if (pos < 0.0f) {
            pos += max_d;
            if (pos < 0.0f) pos = 0.0f;
        }
        size_t idx0 = static_cast<size_t>(pos);
        if (idx0 >= _max_size) idx0 = 0;
        size_t idx1 = (idx0 + 1 < _max_size) ? idx0 + 1 : 0;
        float frac = pos - static_cast<float>(idx0);
        return _buf[idx0].r + frac * (_buf[idx1].r - _buf[idx0].r);
    }

    __attribute__((always_inline)) inline void ReadPosStereo(float pos, float& out_l, float& out_r) const {
        if (!_buf || _max_size == 0) {
            out_l = 0.0f; out_r = 0.0f;
            return;
        }
        float max_d = static_cast<float>(_max_size);
        if (pos >= max_d) {
            pos -= max_d;
            if (pos >= max_d) pos = 0.0f;
        } else if (pos < 0.0f) {
            pos += max_d;
            if (pos < 0.0f) pos = 0.0f;
        }
        size_t idx0 = static_cast<size_t>(pos);
        if (idx0 >= _max_size) idx0 = 0;
        size_t idx1 = (idx0 + 1 < _max_size) ? idx0 + 1 : 0;
        float frac = pos - static_cast<float>(idx0);
        const StereoSample& s0 = _buf[idx0];
        const StereoSample& s1 = _buf[idx1];
        out_l = s0.l + frac * (s1.l - s0.l);
        out_r = s0.r + frac * (s1.r - s0.r);
    }

    __attribute__((always_inline)) inline void Write(float sample_l, float sample_r, float speed) {
        if (!_buf || _max_size == 0) return;
        size_t prev_idx = static_cast<size_t>(_write_pos);
        if (prev_idx >= _max_size) prev_idx = 0;
        _buf[prev_idx] = StereoSample{sample_l, sample_r};

        float max_d = static_cast<float>(_max_size);
        if (_written_distance < max_d) {
            _written_distance += std::max(0.0f, speed);
            if (_written_distance > max_d) _written_distance = max_d;
        }

        Advance(speed);

        // If speed > 1.0, fill any skipped intermediate indices to prevent dropouts/holes in buffer
        if (speed > 1.0f) {
            size_t next_idx = static_cast<size_t>(_write_pos);
            if (next_idx >= _max_size) next_idx = 0;
            if (next_idx != prev_idx) {
                size_t fill_idx = prev_idx + 1;
                if (fill_idx >= _max_size) fill_idx = 0;
                size_t safety = 2;
                while (fill_idx != next_idx && --safety > 0) {
                    _buf[fill_idx] = StereoSample{sample_l, sample_r};
                    fill_idx++;
                    if (fill_idx >= _max_size) fill_idx = 0;
                }
            }
        }
    }

    __attribute__((always_inline)) inline void Advance(float speed) {
        if (_max_size == 0) return;
        _write_pos += speed;
        float max_d = static_cast<float>(_max_size);
        if (_write_pos >= max_d) {
            _write_pos -= max_d;
            if (_write_pos >= max_d) _write_pos = 0.0f;
        } else if (_write_pos < 0.0f) {
            _write_pos += max_d;
            if (_write_pos < 0.0f) _write_pos = 0.0f;
        }
    }

    float WritePos() const { return _write_pos; }
    size_t MaxSize() const { return _max_size; }
    float WrittenDistance() const { return _written_distance; }

    // Flip one bit of the 16-bit representation of the sample at a delay distance (both channels)
    inline void CorruptAt(float delay_samples, uint32_t bit) {
        if (!_buf || _max_size == 0) return;
        float d = std::clamp(delay_samples, 1.0f, static_cast<float>(_max_size - 2));
        float pos = _write_pos - d;
        float max_d = static_cast<float>(_max_size);
        if (pos < 0.0f) pos += max_d;
        else if (pos >= max_d) pos -= max_d;
        size_t idx = static_cast<size_t>(pos);
        if (idx >= _max_size) idx = 0;
        StereoSample s = _buf[idx];
        int32_t sl = static_cast<int32_t>(daisysp::fclamp(s.l, -1.0f, 1.0f) * 32767.0f) ^ (1 << bit);
        int32_t sr = static_cast<int32_t>(daisysp::fclamp(s.r, -1.0f, 1.0f) * 32767.0f) ^ (1 << bit);
        _buf[idx].l = static_cast<float>(sl) * (1.0f / 32767.0f);
        _buf[idx].r = static_cast<float>(sr) * (1.0f / 32767.0f);
    }

private:
    StereoSample* _buf{nullptr};
    size_t _max_size{0};
    float  _write_pos{0.0f};
    float  _written_distance{0.0f};
    size_t _clear_idx{0};
    size_t _clear_remaining{0};
};

} // namespace bends
