#pragma once

#include "daisy_seed.h"
#include "surface/frame.h"
#include "circuit/contacts.h"
#include "dsp/fast_math.h"
#include <cmath>
#include <algorithm>

namespace bends {

class MidiManager {
public:
    void Init() {
        daisy::MidiUsbHandler::Config cfg;
        cfg.transport_config.periph = daisy::MidiUsbTransport::Config::INTERNAL;
        cfg.transport_config.tx_retry_count = 0;
        _midi.Init(cfg);
        _midi.StartReceive();

        for (int i = 0; i < 8; ++i) {
            _midi_val[i] = 0.0f;
            _midi_override[i] = false;
            _last_physical[i] = -1.0f;
        }

        _target_pitch_hz = 0.0f;
        _slewed_pitch_hz = 0.0f;
        _midi_note_gate = 0.0f;
        _last_velocity = 0.0f;
        _aftertouch = 0.0f;
        _mod_wheel = 0.0f;
        _midi_trigger_cnt = 0;
        _is_legato = false;

        _num_held_notes = 0;
        _pitch_bend_semitones = 0.0f;

        _clock_tick_count = 0;
        _last_clock_ms = 0;
        _last_beat_ms = 0;
        _tempo_hz = 2.0f; // 120 BPM default
        _has_midi_clock = false;

        _midi_switch_a_override = false;
        _midi_switch_a_val = 1;
        _last_switch_a_pos = -1;

        _midi_switch_b_override = false;
        _midi_switch_b_freeze = false;
        _midi_switch_b_latched = false;
        _last_switch_b_pos = -1;
    }

    // Called from the surface thread (~1 kHz)
    void Process(ControlFrame& frame) {
        _midi.Listen();

        uint32_t now = daisy::System::GetNow();

        // Incoming MIDI events
        while (_midi.HasEvents()) {
            HandleMidiEvent(_midi.PopEvent(), frame, now);
        }

        // MIDI Clock timeout (1.0s without timing pulses reverts to physical knob)
        if (_has_midi_clock && (now - _last_clock_ms > 1000)) {
            _has_midi_clock = false;
        }

        // Physical pot takeover
        float physical_knobs[8] = {
            frame.s30_amp, frame.s31_flt, frame.s32_mix, frame.s33_clk,
            frame.s34_dly, frame.s35_fbk, frame.s36_osc, frame.s37_vcc
        };

        for (int i = 0; i < 8; ++i) {
            float phys = physical_knobs[i];
            if (_last_physical[i] < 0.0f) {
                _last_physical[i] = phys;
            } else if (std::abs(phys - _last_physical[i]) > 0.008f) {
                _midi_override[i] = false;
                _last_physical[i] = phys;
                if (i == 6) {
                    _target_pitch_hz = 0.0f;
                    _slewed_pitch_hz = 0.0f;
                }
            }
        }

        // Apply active MIDI overrides to frame
        float* controls = &frame.s30_amp;
        for (int i = 0; i < 8; ++i) {
            if (_midi_override[i]) controls[i] = _midi_val[i];
        }

        // Physical switch takeover
        int8_t phys_switch_a = static_cast<int8_t>(frame.osc_mode);
        if (_last_switch_a_pos < 0) {
            _last_switch_a_pos = phys_switch_a;
        } else if (phys_switch_a != _last_switch_a_pos) {
            _midi_switch_a_override = false;
            _last_switch_a_pos = phys_switch_a;
        }

        int8_t phys_switch_b = frame.is_frozen ? 1 : (frame.is_latched ? 2 : 0);
        if (_last_switch_b_pos < 0) {
            _last_switch_b_pos = phys_switch_b;
        } else if (phys_switch_b != _last_switch_b_pos) {
            _midi_switch_b_override = false;
            _last_switch_b_pos = phys_switch_b;
        }

        if (_midi_switch_a_override) {
            frame.osc_mode = _midi_switch_a_val;
        }
        if (_midi_switch_b_override) {
            frame.is_frozen = _midi_switch_b_freeze;
            frame.is_latched = _midi_switch_b_latched;
        }

        // Slew filter for pitch portamento
        if (_num_held_notes > 0) {
            if (_is_legato) {
                _slewed_pitch_hz += 0.045f * (_target_pitch_hz - _slewed_pitch_hz);
            } else {
                _slewed_pitch_hz = _target_pitch_hz;
            }
        }

        frame.midi_osc_freq = (_target_pitch_hz > 0.0f) ? _slewed_pitch_hz : 0.0f;
        frame.midi_gate = _midi_note_gate;
        frame.midi_velocity = _last_velocity;
        frame.midi_aftertouch = _aftertouch;
        frame.midi_mod_wheel = _mod_wheel;
        frame.midi_trigger_cnt = _midi_trigger_cnt;
        frame.midi_legato = _is_legato;
        frame.midi_clk_rate = _has_midi_clock ? _tempo_hz : -1.0f;
    }

private:
    void HandleMidiEvent(daisy::MidiEvent ev, ControlFrame& frame, uint32_t now) {
        if (ev.type == daisy::MidiMessageType::ControlChange) {
            auto cc = ev.AsControlChange();
            uint8_t num = cc.control_number;
            float norm = static_cast<float>(cc.value) * (1.0f / 127.0f);

            if (num == 1) { // CC 1 Mod Wheel
                _mod_wheel = norm;
            } else if (num >= 14 && num <= 21) {
                int idx = num - 14;
                _midi_val[idx] = norm;
                _midi_override[idx] = true;
            } else if (num == 22) { // Switch A: Osc Mode
                _midi_switch_a_val = (cc.value < 43) ? 0 : ((cc.value < 86) ? 1 : 2);
                _midi_switch_a_override = true;
                frame.osc_mode = _midi_switch_a_val;
            } else if (num == 23) { // Switch B: Memory State
                _midi_switch_b_override = true;
                if (cc.value < 43) {
                    _midi_switch_b_freeze = false;
                    _midi_switch_b_latched = false;
                } else if (cc.value < 86) {
                    _midi_switch_b_freeze = false;
                    _midi_switch_b_latched = true;
                } else {
                    _midi_switch_b_freeze = true;
                    _midi_switch_b_latched = false;
                }
                frame.is_frozen = _midi_switch_b_freeze;
                frame.is_latched = _midi_switch_b_latched;
            }
        } else if (ev.type == daisy::MidiMessageType::PitchBend) {
            auto pb = ev.AsPitchBend();
            _pitch_bend_semitones = (static_cast<float>(pb.value) * (1.0f / 8192.0f)) * 2.0f;
            UpdateNotePitch();
        } else if (ev.type == daisy::MidiMessageType::ChannelPressure) {
            _aftertouch = static_cast<float>(ev.AsChannelPressure().pressure) * (1.0f / 127.0f);
        } else if (ev.type == daisy::MidiMessageType::PolyphonicKeyPressure) {
            _aftertouch = static_cast<float>(ev.AsPolyphonicKeyPressure().pressure) * (1.0f / 127.0f);
        } else if (ev.type == daisy::MidiMessageType::NoteOn) {
            auto n = ev.AsNoteOn();
            bool was_held = (_num_held_notes > 0);
            RemoveNote(n.note);
            if (n.velocity > 0) {
                _last_velocity = static_cast<float>(n.velocity) * (1.0f / 127.0f);
                if (_num_held_notes < 8) {
                    _note_stack[_num_held_notes++] = { static_cast<uint8_t>(n.note), static_cast<uint8_t>(n.velocity) };
                }
                _midi_trigger_cnt++;
                _is_legato = was_held;
                UpdateNotePitch();
                if (!was_held) {
                    _slewed_pitch_hz = _target_pitch_hz;
                }
            } else {
                if (_num_held_notes > 0) {
                    _is_legato = true;
                    UpdateNotePitch();
                } else {
                    _midi_note_gate = 0.0f;
                    _is_legato = false;
                    _aftertouch = 0.0f;
                }
            }
        } else if (ev.type == daisy::MidiMessageType::NoteOff) {
            auto n = ev.AsNoteOff();
            RemoveNote(n.note);
            if (_num_held_notes > 0) {
                _is_legato = true;
                UpdateNotePitch();
            } else {
                _midi_note_gate = 0.0f;
                _is_legato = false;
                _aftertouch = 0.0f;
            }
        } else if (ev.type == daisy::MidiMessageType::SystemRealTime) {
            if (ev.srt_type == daisy::SystemRealTimeType::TimingClock) {
                _clock_tick_count++;
                if (_clock_tick_count >= 24) { // 24 pulses per quarter note
                    _clock_tick_count = 0;
                    if (_last_beat_ms > 0) {
                        uint32_t delta = now - _last_beat_ms;
                        if (delta >= 150 && delta <= 3000) {
                            float bpm = 60000.0f / static_cast<float>(delta);
                            _tempo_hz = bpm * (1.0f / 60.0f);
                            _has_midi_clock = true;
                        }
                    }
                    _last_beat_ms = now;
                }
                _last_clock_ms = now;
            } else if (ev.srt_type == daisy::SystemRealTimeType::Start ||
                       ev.srt_type == daisy::SystemRealTimeType::Continue) {
                _clock_tick_count = 0;
                _last_beat_ms = now;
                _last_clock_ms = now;
                _has_midi_clock = true;
            } else if (ev.srt_type == daisy::SystemRealTimeType::Stop) {
                _has_midi_clock = false;
            }
        }
    }

    void RemoveNote(uint8_t note) {
        for (uint8_t i = 0; i < _num_held_notes; ++i) {
            if (_note_stack[i].note == note) {
                for (uint8_t j = i; j + 1 < _num_held_notes; ++j) {
                    _note_stack[j] = _note_stack[j + 1];
                }
                _num_held_notes--;
                break;
            }
        }
    }

    inline void UpdateNotePitch() {
        if (_num_held_notes > 0) {
            auto top = _note_stack[_num_held_notes - 1];
            float note_rel = (static_cast<float>(top.note) - 69.0f + _pitch_bend_semitones) * (1.0f / 12.0f);
            _target_pitch_hz = 440.0f * FastPow2(note_rel);
            _midi_note_gate = static_cast<float>(top.vel) * (1.0f / 127.0f);
        } else {
            _midi_note_gate = 0.0f;
        }
    }

    daisy::MidiUsbHandler  _midi;

    float _midi_val[8];
    bool  _midi_override[8];
    float _last_physical[8];

    bool    _midi_switch_a_override{false};
    uint8_t _midi_switch_a_val{1};
    int8_t  _last_switch_a_pos{-1};

    bool    _midi_switch_b_override{false};
    bool    _midi_switch_b_freeze{false};
    bool    _midi_switch_b_latched{false};
    int8_t  _last_switch_b_pos{-1};

    float    _target_pitch_hz{0.0f};
    float    _slewed_pitch_hz{0.0f};
    float    _midi_note_gate{0.0f};
    float    _last_velocity{0.0f};
    float    _aftertouch{0.0f};
    float    _mod_wheel{0.0f};
    uint16_t _midi_trigger_cnt{0};
    bool     _is_legato{false};

    struct NoteEntry {
        uint8_t note{0};
        uint8_t vel{0};
    };
    NoteEntry _note_stack[8]{};
    uint8_t   _num_held_notes{0};
    float     _pitch_bend_semitones{0.0f};

    uint8_t  _clock_tick_count{0};
    uint32_t _last_clock_ms{0};
    uint32_t _last_beat_ms{0};
    float    _tempo_hz{2.0f};
    bool     _has_midi_clock{false};
};

} // namespace bends
