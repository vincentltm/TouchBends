#pragma once

#include "daisy_seed.h"
#include "knobs.h"
#include "pads.h"
#include "switches.h"

namespace synthux {

class Touch {
public:
    void Init(daisy::DaisySeed& hw) {
        _knobs.Init(hw);
        _pads.Init(hw);
        _switches.Init();
        hw.adc.Start();
    }

    bool Process() {
        bool updated = _pads.Process();
        _knobs.Process();
        return updated;
    }

    Knobs& knobs() { return _knobs; }
    Pads& pads() { return _pads; }
    Switches& switches() { return _switches; }

private:
    Knobs _knobs;
    Pads _pads;
    Switches _switches;
};

} // namespace synthux
