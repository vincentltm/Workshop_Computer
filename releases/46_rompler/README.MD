# Rompler & Sequencer Card Manual

## ── System Boot & Mode Selection ──
* **Startup Chasing Animation:** System mounts SoundFont and scans flash for MIDI files.
* **Persistent Mode Toggle:** Hold toggle switch **DOWN** during boot animation to switch modes. 
* **Boot Confirmation Flashes:**
  * **LEDs 0 & 1:** Live Mode.
  * **LEDs 4 & 5:** MIDI Mode.

---

## ── Live Mode ──
CV inputs and incoming MIDI control the Rompler.

### Control Pages
| Switch | Main Knob | X Knob | Y Knob |
| :--- | :--- | :--- | :--- |
| **UP** *(Tone)* | **Filter Cutoff** (LP $\rightarrow$ Bypass $\rightarrow$ HP) | **Glitch Mangle** (Probability & Type) | **Reverb Mix** (Dry $\rightarrow$ Wet) |
| **MIDDLE** *(Perf)* | **Master Pitch** ($\pm3$ Octaves coarse) | **Synth Release** (Fast $\rightarrow$ Long) | **Drum Release** (Percussion decay) |
| **DOWN** *(Patch)* | **Select Patch Preset** (Scroll SF2) | **Quantization Scale** | **Multi-Timbral Mod** (Duophony vs Layer B) |

### Page Shortcuts & Live Workflow

* **Scale Options (DOWN Page - X):** 0: Chromatic, 1: Major, 2: Minor, 3: Major Pentatonic, 4: Minor Pentatonic.
* **Voice Modes (DOWN Page - Y):**
  * **First 10%:** *Duophonic Mode.* Gates 1 & 2 trigger independent voices using the primary preset.
  * **Remaining 90%:** *Dual Preset Mode.* Gate 1 plays primary preset; Gate 2 plays "Preset B" scrolled via this knob.

### Normal Mode IO
* **CV 1 In & Pulse 1 In:** 1V/Oct pitch (quantized to scale) and gate for Voice 1.
* **Audio 1 In:** Velocity CV for Voice 1 (sampled on Pulse 1 rising edge).
* **CV 2 In & Pulse 2 In:** 1V/Oct pitch and gate for Voice 2 (routes to Voice 2 or Preset B).
* **Audio 2 In:** Preset Scanner CV (when Pulse 2 is empty). Offsets instrument index by $\pm16$ presets.
* **Audio Out 1 & 2:** Main Stereo output.
* **CV 2 Out:** 0–5V hardware envelope follower mirroring Channel 0 volume.

---

## ── Mode 2: MIDI Playback Mode ──
Standalone backing track arranger, live audio destruction loop station, and tape recorder.

### Control Pages (MIDI Playback Mode)
| Switch | Main Knob | X Knob | Y Knob |
| :--- | :--- | :--- | :--- |
| **UP** *(Tone)* | **Filter Cutoff** (LP $\rightarrow$ Bypass $\rightarrow$ HP) | **Glitch Mangle** (Buffer Destruction) | **Reverb Mix** (Dry $\rightarrow$ Wet) |
| **MID (Unfrozen)** | **Master Transpose** ($\pm24$ Semitones) | **Synth Release** (Global Envelope) | **Drum Release** (Percussion Decay) |
| **MID (Frozen)** | **Stutter Division** (1 Bar to 1/64 note) | **Scrub Position** (4-Bar Timeline) | **Sequencer Glitch** (Bend/Ratchet Prob) |
| **DOWN** *(Pattern)*| **Pattern Select** (Queue MIDI File) | **Internal Tempo** (40–280 BPM) | **Mutate Prob** (Skip / Octave Shift) |

### Sequence Controls & Navigation
* **Start / Stop:** Flick the switch **UP** to toggle MIDI file playback.
* **Bar-Synced Pattern Queuing (DOWN Page - Main):** Scrolls 16 MIDI files. Changes cue and execute precisely at the next 4-bar boundary. LED 5 flashes rapidly when a pattern switch is pending.
* **Humanize / Mutate (DOWN Page - Y):**
  * *0–33%:* Random velocity variations.
  * *33–66%:* Melodic notes randomly shift up/down by $\pm12$ semitones.
  * *66–100%:* Random $\pm24$ semitones and perfect fifth intervals + random note skips (up to 40%).

---

## ── The Live Freeze & Stutter System ──
Flick the switch **DOWN** to engage **Freeze Mode**. Locks the internal sequence into an asynchronous looping buffer. Flick **DOWN** again to instantly unfreeze and snap back to time alignment.

### 1. Audio Buffer Destruction (UP Page - X)
Progressively increases the probability of four distinct structural artifacts:
1. **Reverse (0–33%):** Playhead segments run backwards seamlessly.
2. **Micro-Looping (33–66%):** Clips playback into grain windows from a wide 170ms down to a 43ms buzz.
3. **Address Scrambling (66–100%):** Executes bitwise XOR mask against memory lookup lines for digital crunch.
4. **Wrong-Sample ROM Overrides (Top 20%):** Replaces active waves with random wave data from other instruments.
* *Reverb Glitch (Top 33%):* Inject infinite feedback bursts or gates the wet path entirely on a 32nd-note grid.
* *External Triggers:* Gate inputs into **CV 2 In** override the knob and force audio glitches instantly on clock edges.

### 2. Sequencer & Clock Glitching (MIDDLE Page - Y)
Applies behavioral mutation directly to the frozen MIDI event stream:
* **Step Address Scrambling:** Injects random time-offset XOR masks causing notes to fire out of order.
* **Melodic Ratcheting:** Clones triggers into hyper-speed repeated 32nd-note or 64th-note bursts.
* **Improvisational Transposition:** Shifts note values by random octaves or scale intervals on each repeat.

### 3. Tape Stop & Scrubbing (MIDDLE Page - Main & X)
* **Main Knob (Stutter Division):** Right of center subdivides the loop window size. Left of center slows down the virtual tape motor, producing a smooth pitch/speed drop to a dead stop.
* **X Knob (Scrub Position):** Manually moves the loop playhead location across the 4-bar timeline.

---

## ── MIDI Mode IO Connections ──
* **Pulse 1 In:** External Sync. Bypasses internal tempo; syncs sequencer to an external 16th-note clock.
* **Pulse 2 In:** Pattern Reset. Gate input instantly snaps playback back to bar 1, beat 1.
* **CV 1 In:** Transpose CV. Modulates the overall pitch of melodic tracks via external control voltages.
* **CV 2 In:** Glitch Trigger. Accepts gates/triggers to instantly punch in the **Glitch Mangle** effect.
* **Audio 1 In:** Sidechain Ducking. Inputs automatically duck melodic tracks on transients. Depth is set by **Mutate**.
* **Audio 2 In:** Live FX Processor. Routes external audio into the Master Filter, Reverb, and Glitch engine.
* **Audio Out 1 & 2:** Main Stereo outs (Stereo: Synth 1 / Drums. Dual Mono: Synth Channel 1 / Synth Channel 2).
* **CV Out 1 & Pulse Out 1:** 1V/Oct pitch and gate outputs for internal tracks routed to target channels 2 or 5.
* **CV Out 2 & Pulse Out 2:** Pitch and gate for targets 3 or 6. Generates a 10ms quarter-note clock if empty.

---

## ── Panel LED Status Guide ──
### Normal Mode (Switch Middle)
* **LEDs 0–3:** 4-bit binary display showing the pitch class of the newest note on Channel 0 (0–11, C through B).
* **LED 4 / 5:** Illuminates when Channel 0 / Channel 1 voice gate is high.

### MIDI Playback Mode (Default Navigation)
* **LED 0 / 1:** Internal Sequencer Gate activity (Melodic tracks / Drums).
* **LED 2 / 3:** Mirrors active state of **Pulse Out 1** / **Pulse Out 2**.
* **LED 4 (Tempo Beacon):** Flashes at every quarter-note beat boundary.
* **LED 5 (Sequencer Status):** Solid = Playing, Slow Blink = Frozen, Double Flash = Play-Once finished, Fast Flash = Pattern switch queued.