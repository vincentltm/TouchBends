#pragma once

#include "bends/slots.h"

namespace bends {

struct OscillatorMods {
    float chime{0.0f};
    float sub_octave{0.0f};
    float shape_warp{0.0f};
    float rungler_lock{0.0f};
    float sync{0.0f};
};

struct PreampMods {
    float drive{0.0f};
    float bias{0.0f};
    float tremolo{0.0f};
    float ring_mod{0.0f};
    float osc_voice{0.0f};
    float flt_loop{0.0f};
    float fbk_loop{0.0f};
};

struct FilterMods {
    float cutoff_ramp{0.0f};
    float clock_step{0.0f};
    float resonance{0.0f};
    float ping{0.0f};
    float howl{0.0f};
    float wavefold{0.0f};
};

struct MixerMods {
    float tremolo{0.0f};
    float filter_isolate{0.0f};
    float wet_only{0.0f};
    float drive{0.0f};
    float amp_direct{0.0f};
    float dropout{0.0f};
    bool  is_chop{false};
};

struct ClockMods {
    float rate_mod{0.0f};
    float hold{0.0f};
    float swing{0.0f};
};

} // namespace bends
