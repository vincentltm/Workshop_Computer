# Drums

A generative 4-voice drum machine and trigger sequencer for the Workshop System Computer. It features role-designated slots with advanced synthesis (analog, FM, physical modeling), Direct Flash sample playback (compatible with the Grains WebUSB manager), a 4-track Euclidean sequencer with Turing Loop mutation, and master dynamics/effects.

## Controls & Navigation

Navigation is driven by the physical 3-position toggle switch:

* **Switch UP (Top): Drum Voice & Mixer Settings**
  * **Pages 0–3: Sound Edit (Voices 1–4)**
    * **Main**: Pitch / Sample playback speed.
    - **X**: Decay envelope duration.
    - **Y**: Timbre (noise balance, FM index, physical skin tension, or sample filter cutoff).
  * **Pages 4–5: Mixer & Model Selection**
    * **Page 4**: Voice 1 & 2 Mixer (Main = Volume Balance V1/V2, X = V1 Model, Y = V2 Model).
    * **Page 5**: Voice 3 & 4 Mixer (Main = Volume Balance V3/V4, X = V3 Model, Y = V4 Model).
  * **Page 6: Delay & Reverb FX**
    * **Main**: Delay Time & Feedback.
    * **X**: Delay Mix.
    * **Y**: Reverb Size & Mix.

* **Switch MIDDLE (Middle): Sequencer & Global Settings**
  * **Pages 0–3: Euclidean Sequencers (Voices 1–4)**
    * **Main**: Steps (0 to 16). Setting to 0 disables internal sequencing for this track.
    * **X**: Fills / Hits (0 to Steps).
    * **Y**: Rotation / Offset (0 to 15).
  * **Page 4: Master Filter & Dynamics**
    * **Main**: Master DJ Filter (LPF <- Center -> HPF).
    * **X**: Compression / Drive amount.
    * **Y**: Master Volume.
  * **Page 5: Global Clock & Chaos**
    * **Main**: BPM (40..240) if internal, or Clock Division if external.
    * **X**: Swing & Global Fills (forces hits on all tracks).
    * **Y**: Turing Pattern Mutation (slowly mutates Euclidean patterns over a 16-step shift register).

* **Switch DOWN (Momentary Flick / Hold)**:
  * **Short Flick DOWN** (<350ms): Cycle forward one page.
  * **Long Hold DOWN** (350ms - 1.5s): Cycle backward one page.
  * **Hold DOWN** (>1.5s): Enter Preset Menu (binary save/load of slots 1–6 on LEDs).

## I/O Connections

* **Audio In 1**: Trigger / Velocity input for **Voice 1** (Kick).
* **Audio In 2**: Trigger / Velocity input for **Voice 2** (Snare).
* **CV 1 In**: Trigger / Velocity input for **Voice 3** (Hihat).
* **CV 2 In**: Trigger / Velocity input for **Voice 4** (Perc/Tom).
* **Pulse 1 In**: External Clock input (sequencer automatically switches to this when a cable is plugged).
* **Pulse 2 In**: Run/Reset gate (sequence plays while gate is HIGH; resets to Step 0 on rising edge).

* **Audio Out 1 / 2**: Stereo master mix output.
* **Pulse Out 1**: Trigger output for **Track 1** (Kick).
* **Pulse Out 2**: Trigger output for **Track 2** (Snare).
* **CV Out 1**: Gate trigger output (+5V CV) for **Track 3** (Hihat).
* **CV Out 2**: Gate trigger output (+5V CV) for **Track 4** (Perc/Tom).

## Build Instructions

1. Configure the build environment:
   ```bash
   mkdir -p build && cd build
   cmake .. -DPICO_BOARD=pico
   make -j4
   ```
2. Put the Workshop Computer in BOOTSEL mode and flash the resulting `drums.uf2` binary.
