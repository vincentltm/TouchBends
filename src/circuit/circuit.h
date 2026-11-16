#pragma once

#include "surface/frame.h"
#include "circuit/contacts.h"
#include "circuit/solver.h"
#include "circuit/rail.h"
#include "circuit/block_state.h"
#include "circuit/mappings.h"
#include "circuit/topology.h"
#include "bends/bend_engine.h"
#include "bends/delay_bends.h"
#include "tuning/tuning.h"

namespace bends {

class Circuit {
public:
    void Init(float sample_rate) {
        _contacts.Reset();
        _topology.Reset();
        _solver.Reset();
        _rail.Init(sample_rate);
        _bend_engine.Init(750.0f);
    }

    void Update(const ControlFrame& frame, float audio_envelope, BlockState& state, bool clk_beat, float clk_rate,
                float osc_freq = 110.0f, float osc_phase = 0.0f, uint8_t osc_rnd_step = 0) {
        // Touch contact processing
        _contacts.Process(frame);

        // Multi-touch topology analysis
        _topology.Analyze(_contacts);

        // Condition flags
        // Condition flags (strictly hardware switches on device)
        float vcc = _rail.EffectiveVcc();
        uint8_t when_flags = 0;
        if (frame.osc_mode == 1) when_flags |= When::Saw;
        else if (frame.osc_mode == 0) when_flags |= When::Sin;
        else when_flags |= When::Rnd;

        if (clk_rate < 35.0f) when_flags |= When::ClkSlow;
        else when_flags |= When::ClkFast;

        if (frame.is_latched) when_flags |= When::Latched;

        // Declarative bend compilation
        _bend_engine.Compile(_contacts, _topology, when_flags);

        // Passive nodal solver
        _solver.Solve(_contacts, vcc, _bend_engine.RoutedMask());

        // Power rail & supply starvation
        float starve = std::clamp(1.0f - frame.s37_vcc, 0.0f, 1.0f);
        float g_vcc_amp = _contacts.Conductance(NodeVcc, NodeAmp);
        float g_vcc_flt = _contacts.Conductance(NodeVcc, NodeFlt);
        float g_vcc_fbk = _contacts.Conductance(NodeVcc, NodeFbk);
        float bend_power_drain = (g_vcc_amp * 0.90f + g_vcc_flt * 1.20f + g_vcc_fbk * 0.90f) * starve;
        // Output stage bias draw increases quadratically with S30 AMP
        float amp_bias_load = frame.s30_amp * frame.s30_amp * 0.85f;

        // Feedback loops draw dynamic power as recirculating loop amplitude builds:
        float g_flt_amp = _contacts.Conductance(NodeFlt, NodeAmp);
        float g_amp_fbk = _contacts.Conductance(NodeAmp, NodeFbk);
        float g_out_amp = _contacts.Conductance(NodeOut, NodeAmp);
        float fbk_loop_draw = (g_flt_amp * 1.20f + g_amp_fbk * 1.10f + g_out_amp * 1.30f) * audio_envelope;

        // Amplifier & feedback stages draw dynamic current proportional to audio volume
        float starve_audio = (starve > 0.15f) ? (starve - 0.15f) * 1.20f : 0.0f;
        float amp_dynamic_draw = frame.s30_amp * audio_envelope * (1.50f + 2.00f * starve);
        float audio_load = amp_dynamic_draw + fbk_loop_draw + audio_envelope * (starve_audio * 1.20f + bend_power_drain * 2.20f);

        float touch_drain = _contacts.TotalCurrentLoad() * (starve * starve * 1.50f);
        float total_load = _contacts.TotalCurrentLoad() + touch_drain + _solver.VccCurrent() + audio_load +
                           (amp_bias_load * (0.30f + 1.20f * starve)) + bend_power_drain + _topology.cluster_load;

        float vcc_pull  = _solver.Pull(NodeVcc);
        float g_vcc_gnd = _contacts.Conductance(NodeVcc, NodeGnd);
        // Direct VCC-GND short only applies for 2-pad gestures, not multi-pad clusters
        float direct_short = (_topology.active_node_count <= 2) ? g_vcc_gnd : 0.0f;
        _rail.ProcessControl(frame.s37_vcc, total_load, vcc_pull, direct_short);
        vcc = _rail.EffectiveVcc();

        // Populate block state
        state.vcc = vcc;
        state.starve = std::clamp(1.0f - vcc, 0.0f, 1.0f);
        state.osc_mode = frame.osc_mode;
        state.is_frozen = frame.is_frozen;
        state.is_latched = frame.is_latched;
        state.topology = _topology;

        state.motorboat_pulse = _rail.MotorboatPulse();
        state.dynamic_motion = _contacts.DynamicMotion();
        state.digital_motion_ratio = _contacts.DigitalMotionRatio();

        for (size_t i = 0; i < kSolverNodes; ++i) {
            state.retention[i] = _solver.Retention(static_cast<NodeId>(i));
            state.pull[i] = _solver.Pull(static_cast<NodeId>(i));
        }

        // Solo pad actions only activate when touched alone
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            state.pressures[i] = _contacts.IsAlone(i) ? _contacts.Pressure(i) : 0.0f;
        }
        // In & Out contact mic / audio probes track audio/rubbing directly
        state.pressures[NodeIn] = _contacts.Pressure(NodeIn);
        state.pressures[NodeOut] = _contacts.Pressure(NodeOut);
        state.alone_node = _contacts.AloneNode();
        state.alone_pressure = (state.alone_node >= 0) ? _contacts.Pressure(state.alone_node) : 0.0f;

        state.s30_amp = frame.s30_amp;
        state.s31_flt = std::clamp(frame.s31_flt + state.pull[NodeFlt] * 0.20f, 0.0f, 1.0f);
        state.s32_mix = std::clamp(frame.s32_mix + state.pull[NodeMix] * 0.20f, 0.0f, 1.0f);
        state.s33_clk = std::clamp(frame.s33_clk + state.pull[NodeClk] * 0.20f, 0.0f, 1.0f);
        state.s34_dly = std::clamp(frame.s34_dly + state.pull[NodeDly] * 0.20f, 0.0f, 1.0f);
        state.s35_fbk = std::clamp(frame.s35_fbk + state.pull[NodeFbk] * 0.20f, 0.0f, 1.0f);
        state.s36_osc = frame.s36_osc;

        state.pressure_in = state.pressures[NodeIn];
        state.rub_speed_in = frame.rub_speed_p10;
        state.g_osc_flt = _contacts.Conductance(NodeOsc, NodeFlt);
        state.g_osc_amp = _contacts.Conductance(NodeOsc, NodeAmp);
        state.g_flt_amp = _contacts.Conductance(NodeFlt, NodeAmp);
        state.g_clk_amp = _contacts.Conductance(NodeClk, NodeAmp);
        state.g_clk_flt = _contacts.Conductance(NodeClk, NodeFlt);
        state.g_osc_clk = _contacts.Conductance(NodeOsc, NodeClk);
        state.g_in_clk = _contacts.Conductance(NodeIn, NodeClk);
        state.g_in_dly = _contacts.Conductance(NodeIn, NodeDly);
        state.g_in_amp = _contacts.Conductance(NodeIn, NodeAmp);
        state.g_out_amp = _contacts.Conductance(NodeOut, NodeAmp);
        state.g_osc_out = _contacts.Conductance(NodeOsc, NodeOut);
        state.g_amp_dly = _contacts.Conductance(NodeAmp, NodeDly);

        float th_touch = tuning::kTouchThreshold * (1.0f - 0.20f * starve);
        float th_bridge = tuning::kBridgeThreshold * (1.0f - 0.20f * starve);

        state.touch_osc = (_contacts.Pressure(NodeOsc) > th_touch) || (state.g_osc_amp > th_bridge) || (state.g_osc_flt > th_bridge);
        state.touch_flt = (_contacts.Pressure(NodeFlt) > th_touch) || (state.g_osc_flt > th_bridge) || (state.g_flt_amp > th_bridge) || (_contacts.Conductance(NodeFlt, NodeFbk) > th_bridge);
        state.touch_clk = (_contacts.Pressure(NodeClk) > th_touch) || (_contacts.Conductance(NodeClk, NodeDly) > th_bridge) || (_contacts.Conductance(NodeClk, NodeFbk) > th_bridge);
        state.touch_dly_fbk = (_contacts.Conductance(NodeDly, NodeFbk) > th_bridge) || ((_contacts.Pressure(NodeDly) > th_touch) && (_contacts.Pressure(NodeFbk) > th_touch));
        state.touch_out_gnd = (_contacts.Conductance(NodeOut, NodeGnd) > th_bridge) || (_contacts.Pressure(NodeOut) > th_touch && _contacts.Pressure(NodeGnd) > th_touch);

        // Delay bends & registers
        state.delay_regs.Reset();
        state.delay_regs.time.offset = mappings::DelaySamples(state.s34_dly);
        mappings::BipolarFeedback(state.s35_fbk, state.delay_regs.feedback.gain, state.delay_regs.feedback.crossfeed);
        mappings::CalculateDelayTone(state.s31_flt, vcc, state.delay_regs.feedback.damp, state.delay_regs.feedback.tone_hp);

        DelayContext dctx;
        dctx.write_pos = 0;
        dctx.vcc = vcc;
        dctx.clk_beat = clk_beat;
        dctx.pendulum_slice = _topology.pendulum_slice;
        dctx.is_latched = state.is_latched;
        dctx.in_env = audio_envelope;
        dctx.brake_amount = _bend_engine.Amounts().Get(Slot::DlyBrake);
        dctx.knob_clk = state.s33_clk;
        dctx.clk_rate = clk_rate;
        dctx.is_frozen = frame.is_frozen;
        dctx.osc_stutter = _topology.lissajous_cloud ||
                           (_bend_engine.Amounts().Get(Slot::DlyChop) > 0.008f &&
                            (_bend_engine.Amounts().Get(Slot::DlyClockFm) > 0.008f ||
                             _bend_engine.Amounts().Get(Slot::DlyHeadMod) > 0.008f ||
                             state.pressures[NodeOsc] > 0.015f ||
                             state.g_osc_clk > 0.008f));
        dctx.osc_mode = state.osc_mode;
        dctx.osc_freq = osc_freq;
        dctx.osc_phase = osc_phase;
        dctx.osc_rnd_step = osc_rnd_step;
        dctx.knob_fbk = state.s35_fbk;
        dctx.punch_in = (frame.is_frozen && (state.pressures[NodeDly] > 0.05f || state.g_in_dly > 0.008f));
        dctx.touch_dly = std::max(state.pressures[NodeDly], state.pull[NodeDly]);
        dctx.touch_clk = std::max(state.pressures[NodeClk], state.pull[NodeClk]);

        _delay_bends.Update(_bend_engine.Amounts(), frame.is_frozen, state.delay_regs, dctx);

        // AC cross-injection & substrate audio bleed
        uint8_t num_br = _contacts.NumActiveBridges();
        const auto* br = _contacts.ActiveBridges();
        uint8_t max_cross = (_topology.active_node_count >= 7) ? 0 : 4;
        state.num_cross_pairs = 0;
        if (num_br > 0 && max_cross > 0) {
            const float* y_source = NodeTable::SourceConductance.data();
            float g_sum[kSolverNodes]{0.0f};
            for (uint8_t k = 0; k < num_br; ++k) {
                uint8_t a = br[k].a;
                uint8_t b = br[k].b;
                if (a == NodeVcc || b == NodeVcc || a == NodeGnd || b == NodeGnd) continue;
                g_sum[a] += br[k].g;
                g_sum[b] += br[k].g;
            }

            // Starved CMOS silicon substrate leakage factor (gently scales up under starvation, strictly bounded)
            float substrate_leak = (vcc < 0.45f) ? std::clamp((0.45f - vcc) * 0.28f, 0.0f, 0.095f) : 0.0f;

            float inv_y_sum[kSolverNodes]{0.0f};

            for (uint8_t k = 0; k < num_br && state.num_cross_pairs < max_cross; ++k) {
                uint8_t a = br[k].a;
                uint8_t b = br[k].b;
                if (a == NodeVcc || b == NodeVcc || a == NodeGnd || b == NodeGnd) continue;

                if (inv_y_sum[a] == 0.0f) inv_y_sum[a] = 1.0f / (y_source[a] + g_sum[a]);
                if (inv_y_sum[b] == 0.0f) inv_y_sum[b] = 1.0f / (y_source[b] + g_sum[b]);

                bool is_routed = _bend_engine.IsRouted(a, b);

                float ga = br[k].g * inv_y_sum[a];
                float gb = br[k].g * inv_y_sum[b];

                if (is_routed) {
                    // Under low voltage, allow gentle substrate crosstalk on routed bridges, strictly capped to prevent runaway howl
                    if (substrate_leak > 0.005f) {
                        float leak_a = std::min(0.06f, ga * substrate_leak);
                        float leak_b = std::min(0.06f, gb * substrate_leak);
                        state.cross_pairs[state.num_cross_pairs++] = { a, b, leak_a, leak_b };
                    }
                } else {
                    if (ga > 0.0005f || gb > 0.0005f) {
                        state.cross_pairs[state.num_cross_pairs++] = { a, b, std::min(0.18f, ga), std::min(0.18f, gb) };
                    }
                }
            }
        }
    }

    const Contacts& GetContacts() const { return _contacts; }
    Contacts& GetContacts() { return _contacts; }
    const MultiTouchTopology& GetTopology() const { return _topology; }
    const Rail& GetRail() const { return _rail; }
    Rail& GetRail() { return _rail; }
    const Solver& GetSolver() const { return _solver; }
    const AmountBus& Amounts() const { return _bend_engine.Amounts(); }
    const ModMatrix& Matrix() const { return _bend_engine.Matrix(); }
    uint32_t ActiveBendsMask() const { return _bend_engine.ActiveBendsMask(); }
    bool FlushRequested() const { return _bend_engine.FlushRequested(); }

private:
    Contacts           _contacts;
    MultiTouchTopology _topology;
    Solver             _solver;
    Rail               _rail;
    BendEngine         _bend_engine;
    DelayBends         _delay_bends;
};

} // namespace bends
