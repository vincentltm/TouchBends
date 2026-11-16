# TouchBends Technical Reference

A reference guide to the architecture, circuit simulation, and bend connections in TouchBends.

---

## 1. System Architecture

- **Platform**: Electro-Smith Daisy Seed (ARM Cortex-M7 @ 480 MHz) mounted on the Synthux Touch 2.
- **Audio Engine**: 48 kHz stereo with a 64-sample buffer size (750 Hz audio interrupt, ~1.33 ms block time).
- **Controls**: Capacitive touch pads are read via an MPR121 controller over I2C. Knobs and faders are read through a CD4051 multiplexer. Controls update at ~1 kHz and pass to the audio engine through a lock-free buffer.
- **Memory**:
  - The circular delay line uses 64 MB of external SDRAM, providing up to 21.8 seconds of stereo 16-bit delay.
  - DSP code, filter states, and active audio loops run in fast internal SRAM to avoid memory delays.

### Codebase Map
- `src/main.cpp`: Hardware initialization, peripheral drivers, and main control loop.
- `src/app/instrument.h`: Central engine coordinator tying the surface, circuit solver, and audio graph together.
- `src/surface/`: Touch pad polling (`pads.h`), ADC multiplexing (`knobs.h`), and lock-free thread bridge (`frame.h`).
- `src/circuit/`: Circuit model with node definitions (`nodes.h`), solver (`solver.h`), power rail (`rail.h`), and multi-touch topologies (`topology.h`).
- `src/bends/`: Master bend definition table (`table.h`), bend engine (`bend_engine.h`), delay bend controllers (`delay_bends.h`), and modulation matrix (`mod_matrix.h`).
- `src/delay/`: Circular SDRAM buffer (`memory.h`), crossfading read heads (`read_heads.h`), feedback loop (`feedback_path.h`), and delay coordinator (`delay_engine.h`).
- `src/parts/`: Core DSP blocks: preamp (`preamp.h`), state-variable filter (`filter.h`), bending oscillator (`oscillator.h`), sample clock (`clock.h`), and contact mic (`contact_mic.h`).
- `src/graph/`: Audio rendering pipeline (`signal_graph.h`) and output safety limiter (`safety/output_stage.h`).
- `tests/`: Host unit test suite (`test_delay_host.cpp`) and hardware DWT benchmark profiler (`on_device_perf.h`).

---

## 2. DSP Implementation Notes

The DSP runs entirely on the Daisy Seed's Cortex-M7 without any heap allocations.

### State-Variable Filter (`src/parts/filter.h`)
- **Structure**: 4-pole state-variable filter with low-pass, band-pass, and high-pass outputs.
- **Optimization**: Frequency tuning calculations (`tan`) update every 2 samples (24 kHz) to cut FPU load in half, while the filter integrators run at full 48 kHz.
- **Resonance Limiting**: Soft saturation inside the feedback paths keeps the filter stable and musical when driven into self-oscillation.

### Bending Oscillator (`src/parts/oscillator.h`)
- **Saw**: Anti-aliased PolyBLEP saw wave with clean high notes and minimal foldover.
- **Sine**: Polynomial soft clipping on the sine output for subtle harmonic warmth.
- **Stepped Random**: 4-bit feedback shift register decoded through a simulated R-2R ladder DAC with +/-3% resistor tolerances. Produces stepped analog-style CV melodies.

### Delay Line & Memory (`src/delay/`)
- **SDRAM Access**: The 64 MB external memory holds up to 21.8 seconds of stereo audio. Memory reads use linear interpolation to keep external bus traffic low (2 reads per head per sample), avoiding bus stalls.
- **Smooth Time Changes (`src/delay/read_heads.h`)**: Moving the delay time knob crossfades between two read heads over 20-30 ms, preventing clicks and pitch wobble during manual sweeps.
- **Resonator Range**: Below 50 ms delay time, crossfading disengages for continuous pitch tracking, turning the delay line into a tuned comb filter / Karplus-Strong string resonator.

### Preamp & Saturation (`src/parts/preamp.h`)
- **Asymmetric Clipping**: Separate positive and negative soft-clipping thresholds that shift as the power rail voltage drops.
- **Slew Limiting**: Models op-amp bandwidth loss when starved, rounding off sharp transients as the virtual supply sags.
- **High-Gain Pre-filter**: A gentle high-cut before high-gain stages (up to 32x) keeps high-frequency noise under control.

### Touch Surface / Contact Mic (`src/parts/contact_mic.h`)
- **Surface Noise**: Generates subtle friction noise when touching or rubbing the pads.
- **Body Resonators**: Two bandpass resonators tuned to 135 Hz and 720 Hz model the acoustic resonance of the hardware enclosure.

---

## 3. Circuit Simulation

Rather than treating touch pads as static MIDI or DSP controls, TouchBends models the front panel as a circuit board with virtual test points and a shared power rail.

### Touch Nodes & Impedance (`src/circuit/nodes.h`, `src/circuit/contacts.h`)
Each of the 12 pads acts as an exposed node with a resting voltage and impedance:

- **Low-impedance drivers** (VCC, GND, OSC, AMP, IN, OUT) supply or sink strong signals.
- **High-impedance receivers** (CLK, FLT, DLY, RST) are sensitive control points easily influenced by whatever they are connected to.

When you bridge pads with your fingers, your skin acts as a variable resistor (`src/circuit/contacts.h`). Pressing harder lowers resistance and strengthens the connection. At 750 Hz, the nodal solver (`src/circuit/solver.h`) balances equilibrium voltages across the network. If you bridge more than two pads at once, the circuit balances the connections so signals stay musical rather than overloading.

### The Power Rail (`src/circuit/rail.h`)
The VCC fader sets the main supply voltage for the entire circuit:

- **Starvation**: Pulling the fader down starves the circuit like a dying battery. The preamp saturates earlier with more bite, delay repeats darken and jitter, and the audio resolution drops toward 10-bit crunch.
- **Dynamic Sag**: Loud transients (like heavy bass notes or feedback bursts) briefly draw extra current, causing the supply voltage to sag and breathe in rhythm with the audio.
- **Motorboating**: When starved heavily, the power rail starts to oscillate at a low frequency (3 to 8 Hz), producing rhythmic thumping pulses before browning out into silence.
- **Shared Current**: All active bends draw power from this same rail. Bridging multiple pads simultaneously naturally pulls down the voltage.

---

## 4. Bend Directory

Master routing rules are defined in `src/bends/table.h` and compiled at 750 Hz by `src/bends/bend_engine.h`.

### Delay & Clock

| Nodes | Effect | What Happens | Code Location |
|---|---|---|---|
| **CLK + GND** | Clock Choke | Slows delay playback down to a lo-fi crawl with 10-bit crunch. Snaps back to tempo on release. | `BrakeController` in `src/bends/delay_bends.h` |
| **CLK + DLY** | Stutter Slicing | Freezes delay playback into rhythmic, clock-synced micro-loops. | `ChopController` in `src/bends/delay_bends.h` |
| **CLK + RST** | Continuous Reverse | Reverses delay playback direction through buffer history with recording frozen. | `ReverseController` in `src/bends/delay_bends.h` |
| **DLY + VCC** | Shimmer & Scatter | Injects rail voltage for an octave-up pitch boost. Firm pressure scatters memory read positions. | `ScatterController` in `src/bends/delay_bends.h` |
| **DLY + FBK** | Buffer Glitch | Destabilizes the feedback loop into runaway oscillation and buffer slice jumps. | `FbkGlitchController` in `src/bends/delay_bends.h` |
| **DLY + GND** | Memory Corruption | Glitches the delay buffer with address skips, dropouts, and digital bitcrush. | `SkipController` in `src/bends/delay_bends.h` |
| **DLY + MIX** | Data Short | Causes short circuits and momentary audio dropouts in the delay line. | `DataShortController` in `src/bends/delay_bends.h` |
| **RST + DLY** | Read Reset | Instantly snaps the playback head to the current write position. | `RestartController` in `src/bends/delay_bends.h` |
| **RST + FBK** | Subdivision Jump | Jumps delay time across musical divisions (1/4, 1/2, 3/4, 1x, 2x). | `BufferJumpController` in `src/bends/delay_bends.h` |
| **RST + GND** | Flush Buffer | Wipes the entire delay memory clean. | `src/app/instrument.h` |
| **OSC + CLK** | Pitch Vibrato & Chorus | Modulates delay clock for vibrato, tape chorus, and audio-rate FM. | `src/graph/signal_graph.h` |
| **OSC + DLY** | Delay Time Mod | Granular scrub (Saw), time sweep (Sin), and S&H tap hop (Rnd). | `src/graph/signal_graph.h` |

### Oscillator & Filter

| Nodes | Effect | What Happens | Code Location |
|---|---|---|---|
| **OSC + AMP** | Oscillator Route | Sends the bending oscillator directly into the preamp and delay path. | `src/parts/preamp.h` |
| **OSC + FLT** | Filter Cutoff FM | Modulates filter cutoff at audio rate for vocal, buzzing tones. | `src/parts/filter.h` |
| **OSC + GND** | Sub-Octave | Forces the oscillator phase down by one octave. | `src/parts/oscillator.h` |
| **OSC + VCC** | Wave Overdrive | Distorts the oscillator waveform (sine squares off, saw sharpens). | `src/parts/oscillator.h` |
| **OSC + RST** | Hard Sync | Resets the oscillator cycle on delay clock triggers. | `src/parts/oscillator.h` |
| **FLT + AMP** | Filter Feedback Howl | Feeds resonant filter output back into the preamp for self-oscillating squeal. | `src/graph/signal_graph.h` |
| **FLT + GND** | Cutoff Drain | Shunts filter cutoff down to sub-bass and reduces resonance. | `src/parts/filter.h` |
| **VCC + FLT** | Filter Boost | Opens filter cutoff wide and boosts resonance. | `src/parts/filter.h` |
| **CLK + FLT** | Stepped Cutoff | Steps the filter cutoff in sync with delay clock pulses. | `src/parts/filter.h` |
| **RST + FLT** | Filter Ping | Injects a short transient into the resonant filter, creating drum-like acoustic pings. | `src/parts/filter.h` |

### Preamp, Feedback & Output

| Nodes | Effect | What Happens | Code Location |
|---|---|---|---|
| **VCC + FBK** | Runaway Feedback | Drives feedback gain past unity into hard saturation and self-oscillation. | `src/delay/delay_engine.h` |
| **FBK + GND** | Feedback Drain | Bleeds the feedback signal to ground for quick echo decay. | `src/graph/signal_graph.h` |
| **FBK + MIX** | Loop Lock | Locks delay repeats at exact unity gain without runaway distortion. | `src/graph/signal_graph.h` |
| **AMP + FBK** | Preamp Overdrive Loop | Feeds delay repeats directly back into the preamp input for saturated drive. | `src/graph/signal_graph.h` |
| **VCC + AMP** | Wavefolding Drive | Adds non-linear wavefolding and harmonic distortion to the preamp. | `src/parts/preamp.h` |
| **VCC + GND** | Rail Sputter | Shorts the power rail to ground, creating crackling dropouts and supply pops. | `src/circuit/rail.h` |
| **CLK + AMP** | Tremolo | Chops preamp volume on delay clock beats. | `src/parts/preamp.h` |
| **CLK + FBK** | Feedback Bursts | Fires short bursts of feedback on delay clock intervals. | `src/delay/delay_engine.h` |
| **MIX + GND** | Wet Only | Mutes dry audio, leaving 100% wet delay output. | `src/parts/mixer.h` |
| **MIX + RST** | Mute Drop | Cuts output momentarily during reset triggers. | `src/parts/mixer.h` |
| **OUT + GND** | Output Mute | Shunts the main audio output to silence. | `src/graph/safety/output_stage.h` |

### Audio Probes (IN / OUT Pads)

| Nodes | Effect | What Happens | Code Location |
|---|---|---|---|
| **IN + OUT** | No-Input Feedback | Bridges output back into input for wild feedback loops. Sculpt with AMP and FLT. | `src/graph/signal_graph.h` |
| **IN + CLK** | Audio Clock FM | Incoming audio frequency-modulates the delay clock. | `src/parts/clock.h` |
| **OUT + CLK** | Output Sag | Master output volume dynamically modulates delay clock speed on loud peaks. | `src/parts/clock.h` |
| **IN + OSC** | Audio into Oscillator | Incoming audio frequency-modulates the bending oscillator. | `src/parts/oscillator.h` |
| **OUT + OSC** | Output Cross-FM | Master output frequency-modulates the bending oscillator. | `src/parts/oscillator.h` |
| **IN + FLT** | Audio into Filter | Incoming audio modulates the resonant filter cutoff. | `src/parts/filter.h` |
| **OUT + FLT** | Output into Filter | Master output modulates filter cutoff for screaming acoustic howl. | `src/parts/filter.h` |
| **IN + AMP** | Contact Mic Boost | Boosts input gain up to 32x, turning the IN pad into a sensitive acoustic contact mic. | `src/parts/contact_mic.h`, `src/parts/preamp.h` |
| **OUT + AMP** | Pumping Loop | Output volume modulates preamp bias for compression and ducking. | `src/graph/signal_graph.h` |
| **OUT + FBK** | Larsen Howl | Simulates physical acoustic feedback howl through a tuned bandpass filter. | `src/delay/feedback_path.h` |

### Multi-Pad Combinations (3+ Pads)
Higher-level multi-touch interactions are analyzed in `src/circuit/topology.h`:

- **CLK + DLY + RST (Pendulum Slice)**: Alternates playback direction between forward and reverse on successive clock triggers (`src/bends/delay_bends.h`).
- **OSC + CLK + DLY (Modulation Cloud)**: Cross-modulates oscillator pitch, delay clock speed, and memory read position all at once (`src/graph/signal_graph.h`).
- **FLT + DLY + FBK (String Resonator)**: Pulls delay time into tuned Karplus-Strong string frequencies with feedback damped by the filter (`src/graph/signal_graph.h`).
- **VCC + CLK + FBK (Clock Siren)**: Delay feedback drives clock pitch while the starved power rail sags (`src/graph/signal_graph.h`).
- **AMP + FLT + FBK (Noise Loop)**: Preamp saturation, filter resonance, and feedback combine into dense, chaotic noise (`src/graph/signal_graph.h`).
- **CLK + OSC + FLT (Stepped Rungler)**: In Stepped Random mode, clock pulses advance a stepped shift register to generate looping melodic CV sequences (`src/graph/signal_graph.h`).
- **OSC + GND + VCC (Voltage Pump)**: Injects oscillator pulses into the power rail, creating rhythmic pulsing in the supply voltage (`src/graph/signal_graph.h`).

---

## 5. Building & Testing

- **Build firmware**: `make -j8`
- **Run unit tests**: `make test` (compiles and runs `tests/test_delay_host.cpp` locally)
- **Flash via DFU**: `make program-dfu` (hold BOOT on Daisy Seed, tap RESET, release BOOT)
- **Hardware cycle benchmark**: Flash with `tests/on_device_perf.h` (`RUN_PERF_TEST=1`) to monitor DWT CPU cycles over USB CDC serial (`/dev/cu.usbmodem*`).

