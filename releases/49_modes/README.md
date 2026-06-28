# Modal

A physical modeling synthesizer inspired by Mutable Instruments Elements, optimized for the Workshop System Computer. It combines multiple excitation sources with a powerful modal resonator to create metallic, wooden, and string-like textures.

## Controls

**Switch Actions**
- **Down (Tap):** Cycle forward to the next parameter page.
- **Down (Hold 2s):** Cycle back to the previous parameter page.
- **Up (Flick):** Toggle Resonator Model (Modal → Karplus-Strong → Strings).

**Page 1: STRIKE (Exciter)**
- **Main:** Strike Level
- **X:** Strike Timbre (Hardness)
- **Y:** Strike Model (Mallet, Brush, Particles)

**Page 2: BLOW (Exciter)**
- **Main:** Blow Level
- **X:** Blow Timbre (Air noise vs. Tone)
- **Y:** Blow Texture (Turbulence)

**Page 3: BOW & ENV (Exciter)**
- **Main:** Bow Level
- **X:** Bow Timbre (Friction)
- **Y:** Envelope Shape (Attack/Decay profile)

**Page 4: RESONATOR 1 (Material)**
- **Main:** Geometry (Resonator complexity)
- **X:** Brightness (High-frequency damping)
- **Y:** Damping (Overall decay length)

**Page 5: RESONATOR 2 (Acoustics)**
- **Main:** Position (Excitation point on the surface)
- **X:** Space (Stereo width)
- **Y:** Reverb (Amount and decay)

**Page 6: PERFORMANCE**
- **Main:** Pitch Coarse (Octave/Semitone)
- **X:** Fine Tune
- **Y:** Strength (Global exciter intensity)

## IO

- **Audio In 1/2:** External excitation input. Any audio fed here will "strike" the resonator.
- **CV 1 In (Pitch):** 1V/Octave tracking for the resonator.
- **CV 2 In (Strength):** Global accent/strength modulation.
- **Pulse 1 In (Trigger):** Fires the internal exciters (Strike/Blow/Bow).
- **Pulse 2 In (Model):** Cycles through the resonator models (Modal/String/Strings).

- **Audio Out 1/2:** Main stereo output.
- **CV 1 Out:** [Reserved] Resonator Level Meter.
- **CV 2 Out:** [Reserved] Exciter Envelope.
- **Pulse 1 Out (Gate):** Passthrough of the current gate/trigger state.
- **Pulse 2 Out:** [Reserved] End of Cycle (EOC) trigger.

## USB MIDI

Modal supports both **USB MIDI Host** (for keyboards) and **USB MIDI Device** (for DAWs).

### MIDI Playability
- **Note On/Off:** Triggers the exciters and sets the resonator pitch. 
- **Pitch Tracking:** Tracked relative to MIDI Note 60 (C4).

### MIDI CC Mapping
All parameters are automatable via MIDI CC. Moving a CC will automatically **lock** the corresponding physical knob on the active page to prevent jumps until you "re-grab" the control manually.

- **CC 1:** Global Strength
- **CC 10:** Resonator Position
- **CC 21:** Stereo Space
- **CC 70:** Geometry
- **CC 71:** Damping
- **CC 74:** Brightness
- **CC 12-14:** Strike (Level, Timbre, Meta)
- **CC 15-17:** Blow (Level, Timbre, Meta)
- **CC 18-20:** Bow & Env (Level, Timbre, Shape)

## Visuals

Modal uses a paging system indicated by LEDs 0-5. 
- **Chasing LEDs at boot:** Indicates USB mode detection.
- **Top LED flash at boot:** LED 0 = MIDI Host Mode, LED 1 = MIDI Device Mode.
- **Page LEDs:** Indicate the currently active control page.
- **LED 4 Pulse:** Flashes with Gate/Trigger activity.

---
Created for the Music Thing Modular Workshop System by Vincent Maurer (https://github.com/vincent-maurer/) with assistance from Google Gemini.

Thank you to Emilie Gillet (Mutable Instruments) for the original Elements algorithms and to Tom Whitwell and Chris Johnson for the Workshop platform.
