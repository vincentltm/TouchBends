#pragma once

#include "dsp/fast_math.h"
#include "dsp/filters.h"
#include "bends/mod_pack.h"
#include "circuit/health.h"
#include <cmath>
#include <algorithm>

namespace bends {

class Preamp {
public:
    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        _inv_sample_rate = (sample_rate > 0.0f) ? (1.0f / sample_rate) : (1.0f / 48000.0f);
        Reset();
    }

    void Reset() {
        _dc_block_l.Reset();
        _dc_block_r.Reset();
        _vol = 1.0f;
        _target_vol = 1.0f;
        _drive = 1.0f;
        _target_drive = 1.0f;
        _smooth_drive_eff = 1.0f;
        _smooth_master_vol = 1.0f;
        _crank = 0.0f;
        _target_crank = 0.0f;
        _sag_env = 0.0f;
        _spk1_l = 0.0f;
        _spk2_l = 0.0f;
        _spk1_r = 0.0f;
        _spk2_r = 0.0f;
        _pre_tight_l = 0.0f;
        _pre_tight_r = 0.0f;
        _slew_l = 0.0f;
        _slew_r = 0.0f;
        _smooth_rm_l = 0.0f;
        _smooth_rm_r = 0.0f;
        _osc_dc_l = 0.0f;
        _osc_dc_r = 0.0f;
    }

    inline void PrepareBlock(float knob_drive, float node_pull, float v_rail_eff = 1.0f, float starve = 0.0f,
                             float touch_amp = 0.0f, float mod_drive = 0.0f, float mod_bias = 0.0f) {
        float u = std::clamp(knob_drive + node_pull * 0.20f + touch_amp * 0.35f, 0.0f, 1.0f);
        if (u < 0.015f) {
            _target_vol = 0.0f;
            _target_drive = 1.0f;
            _target_crank = 0.0f;
        } else if (u <= 0.50f) {
            // 0.015 to 0.50: Clean gain up to unity (1.0)
            float t = (u - 0.015f) / 0.485f; // in [0, 1]
            _target_vol = t * (0.35f + 0.65f * t); // Natural audio taper
            _target_drive = 1.0f;
            _target_crank = 0.0f;
        } else if (u <= 0.75f) {
            // 0.50 to 0.75: Mild drive and saturation
            float t = (u - 0.50f) / 0.25f; // in [0, 1]
            _target_vol = 1.0f + 0.25f * t;
            _target_drive = 1.0f + 5.0f * (t * t); // Drive scales smoothly 1.0 to 6.0
            _target_crank = 0.0f;
        } else {
            // 0.75 to 1.00: High-gain saturation stage
            float t = (u - 0.75f) / 0.25f; // in [0, 1]
            _target_vol = 1.25f + 0.15f * t;
            _target_drive = 6.0f + 26.0f * (t * t * (1.3f - 0.3f * t)); // Up to 32x gain
            _target_crank = t;
        }
        // Smooth control block updates
        _vol += 0.20f * (_target_vol - _vol);
        _crank += 0.20f * (_target_crank - _crank);

        float target_drive = std::max(0.1f, _target_drive + mod_drive);
        _drive += 0.20f * (target_drive - _drive);

        // Supply starvation saturation curve
        float starve_curve = starve * (1.4f - 0.4f * starve);
        _cached_starve_boost = 1.0f + 0.20f * starve_curve;
        _cached_drive_eff = _drive * _cached_starve_boost;

        // Asymmetric clipping thresholds
        float v_eff = std::clamp(v_rail_eff, 0.0f, 1.0f);
        float p_scale = std::clamp(v_eff * 0.60f + 0.40f * (v_eff * v_eff), 0.0f, 1.0f);
        float n_scale = std::clamp(v_eff * 0.70f + 0.30f * (v_eff * v_eff), 0.0f, 1.0f);

        // Preamp bias modulation (Slot::PreampBias)
        if (mod_bias > 0.01f) {
            float b = std::min(1.0f, mod_bias);
            p_scale = std::max(0.0f, p_scale * (1.0f - b * 0.50f));
            n_scale = std::min(1.50f, n_scale * (1.0f + b * 0.40f));
        }

        float pk = (0.85f - _crank * 0.35f) * p_scale;
        float nk = (0.85f - _crank * 0.15f) * n_scale;

        float span_scale = 1.0f + 1.2f * starve;
        _cached_p_knee = pk;
        _cached_p_span = std::max(0.001f, 0.28f * p_scale * span_scale);
        _cached_inv_p_span = 1.0f / _cached_p_span;

        _cached_n_knee = nk;
        _cached_n_span = std::max(0.001f, (0.28f + _crank * 0.10f) * n_scale * span_scale);
        _cached_inv_n_span = 1.0f / _cached_n_span;

        // Slew-rate limit tracking rail voltage
        float v_sq = v_eff * v_eff;
        _cached_max_slew = 0.010f + 1.190f * (v_sq * (1.5f - 0.5f * v_eff));

        // Crossover deadzone at low voltage (< 0.20V)
        _cached_sputter_th = (v_eff < 0.20f) ? ((0.20f - v_eff) * (0.15f + 0.10f * (1.0f - v_eff))) : 0.0f;
        _cached_inv_sputter_th = (_cached_sputter_th > 0.001f) ? (1.0f / _cached_sputter_th) : 0.0f;

        // Drive loudness compensation
        float d_excess = std::max(0.0f, _drive - 1.0f);
        float drive_comp = 1.0f / (1.0f + 0.18f * FastSqrt(d_excess) + 0.045f * d_excess);
        _cached_comp = drive_comp;

        float rail_vol = (v_eff <= 0.015f) ? 0.0f : (0.55f + 0.45f * v_eff);
        _cached_master_vol = _vol * rail_vol;

        _cached_has_slew = (_cached_max_slew < 1.0f);
        _cached_has_sputter = (_cached_sputter_th > 0.001f);
        _cached_has_crank = (_crank > 0.01f);
        _cached_pre_alpha = 1.0f - _crank * 0.35f;
        _cached_spk_alpha = 1.0f - _crank * 0.55f;
    }

    __attribute__((noinline)) void Process(float in_l, float in_r,
                        float /*touch_amp*/,
                        const PreampMods& mods,
                        float osc_inject_l, float osc_inject_r, float hum_inject,
                        float v_rail_eff, float starve,
                        float shunt_gnd,
                        float& out_l, float& out_r) {
        // Master preamp volume: S30 is volume control up to 12 o'clock, with clean mute at zero
        float master_vol = _cached_master_vol * shunt_gnd;

        // Tremolo modulation
        if (mods.tremolo > 0.0f) {
            master_vol *= std::clamp(1.0f - mods.tremolo * 0.85f, 0.0f, 1.0f);
        }

        // AC-couple oscillator injection to remove sub-audio DC from low LFO speeds (<18 Hz)
        _osc_dc_l += 0.0024f * (osc_inject_l - _osc_dc_l);
        _osc_dc_r += 0.0024f * (osc_inject_r - _osc_dc_r);
        float osc_ac_l = osc_inject_l - _osc_dc_l;
        float osc_ac_r = osc_inject_r - _osc_dc_r;

        float blend_rm = std::min(1.0f, mods.ring_mod);
        float rm_carrier_l = 0.0f;
        float rm_carrier_r = 0.0f;
        if (__builtin_expect(blend_rm > 0.015f || _smooth_rm_l > 0.0001f, 0)) {
            float raw_carrier_l = (blend_rm > 0.015f) ? FastSoftSat(osc_ac_l * 10.0f) : 0.0f;
            float raw_carrier_r = (blend_rm > 0.015f) ? FastSoftSat(osc_ac_r * 10.0f) : 0.0f;
            _smooth_rm_l += 0.12f * (raw_carrier_l - _smooth_rm_l);
            _smooth_rm_r += 0.12f * (raw_carrier_r - _smooth_rm_r);
            rm_carrier_l = _smooth_rm_l;
            rm_carrier_r = _smooth_rm_r;
        } else {
            _smooth_rm_l = 0.0f;
            _smooth_rm_r = 0.0f;
        }

        // Analog pre-gain conditioning: gently rolls off supersonic/digital hash (>6 kHz) before 32x drive
        float proc_in_l = in_l;
        float proc_in_r = in_r;
        if (__builtin_expect(_cached_has_crank, 0)) {
            _pre_tight_l += _cached_pre_alpha * (proc_in_l - _pre_tight_l);
            _pre_tight_r += _cached_pre_alpha * (proc_in_r - _pre_tight_r);
            proc_in_l = _pre_tight_l;
            proc_in_r = _pre_tight_r;
        } else {
            _pre_tight_l = proc_in_l;
            _pre_tight_r = proc_in_r;
        }

        // Supply voltage sag and compression envelope
        float in_peak = std::max(std::abs(proc_in_l), std::abs(proc_in_r)) * (_cached_drive_eff * 0.08f);
        if (in_peak > _sag_env) {
            _sag_env += 0.05f * (in_peak - _sag_env); // fast attack ~0.4ms
        } else {
            _sag_env += 0.00015f * (in_peak - _sag_env); // slow release ~140ms
        }
        _sag_comp_cnt = (_sag_comp_cnt + 1) & 3;
        if (_sag_comp_cnt == 0) {
            _cached_sag_comp = (_crank > 0.01f) ? (1.0f / (1.0f + _crank * 0.35f * std::min(2.0f, _sag_env))) : 1.0f;
            float d_mod = std::min(mods.drive, 3.6f);
            float raw_eff = (_cached_drive_eff + d_mod * 2.8f * _cached_starve_boost) * _cached_sag_comp;
            _cached_eff_drive = std::min(raw_eff, 24.0f);
        }
        float eff_drive = _cached_eff_drive;

        // Branchless single-cycle 1-pole smoothing removes zipper noise without pipeline stalls
        _smooth_drive_eff += 0.04f * (eff_drive - _smooth_drive_eff);
        _smooth_master_vol += 0.04f * (master_vol - _smooth_master_vol);

        float common_bias = hum_inject * _vol + mods.bias * 0.15f;
        float osc_in_l = osc_ac_l * 0.85f;
        float osc_in_r = osc_ac_r * 0.85f;
        float eff_out_vol = _cached_comp * _smooth_master_vol;
        out_l = ProcessChannel(proc_in_l, _dc_block_l, _slew_l, blend_rm, rm_carrier_l,
                               osc_in_l, common_bias,
                               _smooth_drive_eff, eff_out_vol);
        out_r = ProcessChannel(proc_in_r, _dc_block_r, _slew_r, blend_rm, rm_carrier_r,
                               osc_in_r, common_bias,
                               _smooth_drive_eff, eff_out_vol);

        // 2-pole lowpass roll-off (12 dB/oct above 4.5 kHz at high drive)
        if (__builtin_expect(_cached_has_crank, 0)) {
            _spk1_l += _cached_spk_alpha * (out_l - _spk1_l);
            _spk2_l += _cached_spk_alpha * (_spk1_l - _spk2_l);
            _spk1_r += _cached_spk_alpha * (out_r - _spk1_r);
            _spk2_r += _cached_spk_alpha * (_spk1_r - _spk2_r);
            out_l = _spk2_l;
            out_r = _spk2_r;
        } else {
            _spk1_l = out_l; _spk2_l = out_l;
            _spk1_r = out_r; _spk2_r = out_r;
        }
    }

    float Volume() const { return _vol; }
    float EffectiveDrive() const { return _cached_drive_eff; }

private:
    __attribute__((always_inline)) inline float ProcessChannel(
        float ext_in, DcBlocker& dc_block, float& slew_state,
        float blend_rm, float rm_carrier,
        float osc_in, float common_bias,
        float drive_eff, float eff_out_vol) {
        float dry = ext_in;
        float amp_core = dry;
        if (__builtin_expect(blend_rm > 0.015f && std::abs(dry) > 0.01f, 0)) {
            amp_core = dry * (1.0f - blend_rm + blend_rm * rm_carrier * 1.6f);
        }

        // Bridged oscillator injection and transistor DC bias offset enter the saturating preamp core
        float amp_in = amp_core + osc_in + common_bias;

        // Apply drive
        float scaled_in = amp_in * drive_eff;

        // Slew-rate limiting (op-amp bandwidth collapse under starvation)
        float slew_delta = scaled_in - slew_state;
        slew_delta = std::clamp(slew_delta, -_cached_max_slew, _cached_max_slew);
        slew_state += slew_delta;
        slew_state = std::clamp(slew_state, -6.0f, 6.0f);
        float in_proc = slew_state;

        float mag = std::abs(in_proc);

        // Crossover deadzone below 0.20V
        if (__builtin_expect(_cached_has_sputter, 0)) {
            if (__builtin_expect(mag < _cached_sputter_th, 0)) {
                float ratio = mag * _cached_inv_sputter_th;
                float sputter_gain = ratio * ratio;
                in_proc *= sputter_gain;
                mag *= sputter_gain;
            }
        }

        // Asymmetric soft saturation
        float sat;
        if (in_proc >= 0.0f) {
            if (__builtin_expect(in_proc <= _cached_p_knee, 1)) {
                sat = in_proc;
            } else {
                sat = _cached_p_knee + _cached_p_span * FastSoftSat((in_proc - _cached_p_knee) * _cached_inv_p_span);
            }
        } else {
            float neg = -in_proc;
            if (__builtin_expect(neg <= _cached_n_knee, 1)) {
                sat = in_proc;
            } else {
                sat = -(_cached_n_knee + _cached_n_span * FastSoftSat((neg - _cached_n_knee) * _cached_inv_n_span));
            }
        }

        float out = sat * eff_out_vol;
        return dc_block.Process(out);
    }

    float _sample_rate{48000.0f};
    float _inv_sample_rate{1.0f / 48000.0f};
    DcBlocker _dc_block_l;
    DcBlocker _dc_block_r;
    float _vol{1.0f};
    float _target_vol{1.0f};
    float _drive{1.0f};
    float _target_drive{1.0f};
    float _smooth_drive_eff{1.0f};
    float _smooth_master_vol{1.0f};
    float _crank{0.0f};
    float _target_crank{0.0f};
    float _sag_env{0.0f};
    float _spk1_l{0.0f};
    float _spk2_l{0.0f};
    float _spk1_r{0.0f};
    float _spk2_r{0.0f};
    float _pre_tight_l{0.0f};
    float _pre_tight_r{0.0f};
    float _slew_l{0.0f};
    float _slew_r{0.0f};
    float _smooth_rm_l{0.0f};
    float _smooth_rm_r{0.0f};
    float _osc_dc_l{0.0f};
    float _osc_dc_r{0.0f};

    float _cached_p_knee{0.50f};
    float _cached_p_span{0.35f};
    float _cached_inv_p_span{1.0f / 0.35f};
    float _cached_n_knee{0.70f};
    float _cached_n_span{0.45f};
    float _cached_inv_n_span{1.0f / 0.45f};
    float _cached_sputter_th{0.0f};
    float _cached_inv_sputter_th{0.0f};
    float _cached_max_slew{2.0f};
    float _cached_drive_eff{1.0f};
    float _cached_starve_boost{1.0f};
    float _cached_comp{1.0f};
    float _cached_master_vol{1.0f};
    uint8_t _sag_comp_cnt{0};
    float _cached_sag_comp{1.0f};
    float _cached_eff_drive{1.0f};
    float _cached_pre_alpha{0.95f};
    float _cached_spk_alpha{0.95f};
    bool _cached_has_slew{false};
    bool _cached_has_sputter{false};
    bool _cached_has_crank{false};
};

} // namespace bends
