# Bends — Stereo Multi-FX, Glitch, and Codec Demolisher Card

Bends is a highly detailed, 6-stage sequential stereo multi-effect processor and glitch looper designed for the **Workshop Computer** system. Running its audio thread at 24kHz on the RP2040's secondary core, Bends frees up CPU cycles to deliver a continuous, high-fidelity chain of degradation, delay, granulation, filtering, and reverberation.

---

## Installation

1. Connect the Workshop Computer to your computer using a data-capable Micro-USB cable.
2. Put the board in bootloader mode: hold the **Secret Button** (located under the main knob), press the **Reset Button**, and then release the Secret Button. A drive named `RPI-RP2` will mount on your computer.
3. Drag and drop [bends.uf2](file:///Users/vmaurer/Music/Workshop_Computer/releases/45_bends/web/bends.uf2) onto the mounted drive.
4. The board will automatically reboot and start running Bends.

---

## Physical I/O Patching

The panel inputs and outputs are mapped to handle stereo audio, clock sync, CV modulation, and control voltage generation:

### Inputs
*   **Audio In 1 & 2**: Stereo audio input. If `Audio In 2` is left unplugged, the left channel input is normalled to the right channel for mono-to-stereo operation.
*   **Pulse In 1 (Stutter Clock / Trigger)**: Global clock sync input. Measures the time between rising edges to sync the delay block and the granular loop size to rhythmic beat divisions. Also acts as a gate trigger for granular glitches.
*   **Pulse In 2 (Freeze Gate)**: High voltage (> +1.2V) triggers an instant freeze, locking the current delay and stutter loops (equivalent to Switch UP).
*   **CV In 1 (Macro / Clicks Injector)**: Global modulator for grittiness. Positive voltage acts as a macro sweep, adding vinyl crackles, pops, and CD-skipping dropouts.
*   **CV In 2 (Breakup / Stutter Injector)**: Scrambles the digital loss engine and injects packet dropouts. Gate signals above +1.2V force active stutter loops.

### Outputs
*   **Audio Out 1 & 2**: Left and Right main audio outputs. Fully buffered with digital DC-blocking filters to ensure zero offset drift.
*   **CV Out 1 & 2 (Harmonic Pitch CV)**: Outputs semi-random, pitch-representative control voltages (Q11 format, 1 semitone ≈ 68 DAC counts) that step to consonant/dissonant intervals on loop boundaries. Useful for driving external oscillators.
*   **Pulse Out 1 (Loop Boundary)**: Outputs a 2ms trigger pulse (+5V) every time the granular loop wraps or resets.
*   **Pulse Out 2 (Texture / Pop Generator)**: Outputs a composite digital stream of vinyl dust crackle (scaled by CV1), loop boundary clicks, input volume transient spits, and half-way loop sub-harmonic triggers. Can be patched to trigger voice inputs or add external clicking grit.

---

## Page Navigation & Control Interface

Bends uses an 8-page virtual parameter table. Physical knobs (`Main`, `X`, `Y`) update the parameters of the active page. Because knobs are shared, **Knob Lock** logic engages on page change. To adjust a setting on a new page, turn the knob until it crosses the virtual target value (the physical LED will blink at 5Hz indicating the lock direction).

### Switch Operations
*   **Flick DOWN (Quick Release < 350ms)**: Cycles forward through pages `0` to `5`.
*   **Hold DOWN (>= 350ms)**: Enters **Macro Mode**. Knobs are temporarily mapped to:
    *   **Main**: Grittiness Macro sweep (0 to 100%)
    *   **X**: Global Stereo Width & Channel Mode:
        *   *CCW (< 500)*: **Extended Mono Mode** (when Input 2 unplugged; doubles delay/freeze buffer capacity to ~2.73s, LED 5 lights up).
        *   *Mid (500..31500)*: Standard **Stereo Input Width** (0% to 100%).
        *   *CW (> 31500)*: **Dual Mono Mode** (treats Input 1 & 2 as two completely independent mono processors with decorrelated random seeds, independent glitch triggers, independent packet dropouts, and zero crosstalk! LED 4 & 5 light up).
    *   **Y**: Global DSP Routing Preset (`[0..3]`):
        *   **Preset 0 (CCW)**: **Series Standard** (`Chorus -> Codec -> Delay -> Glitcher -> Filter -> Reverb`)
        *   **Preset 1**: **Space Wash** (`Reverb -> Filter -> Chorus -> Delay -> Glitcher -> Codec`)
        *   **Preset 2**: **Scatter Cloud** (`Glitcher -> Codec -> Delay -> Chorus -> Filter -> Reverb`)
        *   **Preset 3 (CW)**: **Filtered Dub** (`Filter -> Codec -> Chorus -> Delay -> Glitcher -> Reverb`)
    *   *Releasing* the switch bakes the macro and width adjustments into the underlying page parameters, and resets the macro value to center (the routing preset is retained globally).
    *   **Manual Save**: Hold the switch DOWN for **3.0 seconds** without adjusting any knobs to commit all 7 parameter pages, routing mode, stereo width, and mono mode permanently to flash memory (all LEDs pulse cyan/white to confirm).
*   **Switch UP**: Instant freeze toggle on press.
    *   **First Press**: Latches the freeze and jumps to **Page 3 (Glitcher / Freeze)**, displaying the playhead animation on the LED ring (allowing you to adjust scrub position, speed, and loop length). You can cycle away (flick DOWN) to adjust other pages while keeping the audio frozen in the background.
    *   **Second Press**: Unfreezes the audio and instantly returns you to the parameter page you were on.
*   **Boot Factory Reset**: Hold the switch DOWN continuously for **3.0 seconds on power-up / boot** to wipe all flash settings and restore the module to clean factory default parameters (all LEDs blink rapidly in red/pink for 1 second to confirm).

---

## Parameter Pages Directory

The virtual page number corresponds to the glowing LED (0 through 5).

| Page | Effect Stage | Main Knob | Knob X | Knob Y |
|:---:|---|---|---|---|
| **0** | **Chorus / Tape Loss** | Chorus Mix | LFO Rate (Drift) | Chorus Depth / Feedback / BBD XOR |
| **1** | **Digital Loss (Codec)** | Decimation Mix | Decimation Factor (Bitcrush) | Corruption (MP3 Ring / Bad Link Dropouts) |
| **2** | **Multi-Tap Delay** | Delay Wet/Dry Mix | Delay Time (Sync subdivisions if clocked) | Delay Feedback (Overdrives into XOR glitches) |
| **3** | **Glitcher / Freeze** | *Unfrozen*: Glitch Mix/Prob<br>*Frozen*: Loop Scrub Position | *Unfrozen*: Granular Loop Size<br>*Frozen*: Loop Length | *Unfrozen*: Playback Speed (Quantized)<br>*Frozen*: Playback Speed & Pitch |
| **4** | **Resonant Filter** | Cutoff (Lowpass CCW, Highpass CW) | Filter Resonance (Q) | Wavefolder Grit (Drive & Fuzz) |
| **5** | **Reverb Engine** | Reverb Wet/Dry Mix | Bipolar Reverb Type & Size<br>(*CCW*: OP-1 Spring Tank, *CW*: Ambient Hall) | Reverb High-Frequency Damping & Circuit-Bent Jitter |

---

## Detailed DSP Architecture

### Stage 1: BBD Chorus & Tape Loss
Emulates a classic bucket-brigade analog delay. Uses two delay lines (1,024 samples) modulated by dual-phase triangle LFOs running 180° out of phase to maximize stereo width. A vintage-style high-pass filter cuts sub-bass mud from the feedback loop, while high values on Knob Y inject digital bit-scrambling directly into the BBD memory.

### Stage 2: Codec Demolisher
Simulates vintage telecom and digital compression failures. Combines a polynomial tape saturator, a 2nd-order Chamberlin bandpass filter to replicate telephone acoustics, a mu-law logarithmic compander (simulating variable-bitrate G.711 compression), fast-modulated delay lines for watery MP3 ringing, and a packet loss engine that loops 256-sample chunks on dropouts.

### Stage 3: Multi-Tap Delay
Features a PT2399-style slow-clock decimation engine. At high delay times, the sample rate drops, introducing clock whine and crunch. If a clock is patched into `Pulse In 1`, the time parameter maps Knob X to 10 rhythmic beat divisions (unclogged, dotted, triplet subdivisions).

### Stage 4: Granular Glitcher
A phrase-sampler that records continuous stereo audio into a circular SRAM buffer. It supports real-time stuttering, random pitch-shifting, and phrase looping. In freeze mode (Page 3 when frozen), the playback head is scrubbed manually using the Main knob (or set fully CCW for Auto-Chaos Drift), with speed/pitch and window size controlled by Y and X.

### Stage 5: Resonant Filter & Wavefolder
Uses a corrected Chamberlin SVF. The input can be overdriven up to 8x, routed through a sine-lookup wavefolder, and boosted with high-gain digital fuzz. An envelope-following feed-forward compressor compensates for volume spikes during wavefolding and self-oscillation.

### Stage 6: Dual Reverb Engine (Spring & Ambient Hall)
Features a dual-mode Schroeder-Dattorro reverb network controlled by Knob X:
* **Counter-Clockwise (CCW < 50%)**: **OP-1 Style Spring Reverb Engine**. Applies all-pass phase dispersion to generate iconic metallic "spring drip" / "boing" chirps on percussive transients, high-pass sub-bass filtering, and 14Hz spring tank flutter.
* **Clockwise (CW > 50%)**: **Lush Ambient Hall & Plate Engine**. Delivers smooth diffusion, wide stereo decorrelation, and near-infinite cathedral washes at high size settings.

---

## Visual Feedback (LED Ring)

*   **Page Indication**: The currently selected page (0 to 5) glows at steady brightness.
*   **Freeze Mode (Page 3 when frozen)**: The 6 LEDs visualize the relative position of the loop window as a low glow, while a bright blinking LED represents the active playback head sweeping through the buffer.
*   **Macro Adjustments**: The LEDs function as a bar graph displaying the sweep value of the macro or stereo width. When adjusting the routing preset (Knob Y), a single LED (LED 0, 1, 2, or 3) lights up brightly to indicate the selected routing preset (Series Standard, Space Wash, Scatter Cloud, or Filtered Dub, respectively).
*   **Routing Chain Visualizer**: On boot, after a manual save, or when changing the routing preset via the Y knob, the module plays a visualization sequence: first flashing all 6 LEDs, then lighting each of the corresponding page LEDs in order of the active audio processing chain, and finally flashing all LEDs again to confirm. If you adjust the Main or X knob during this display, the visualization interrupts instantly for immediate interface response.
