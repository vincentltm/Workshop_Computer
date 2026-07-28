# Bends — Stereo Multi-FX, Glitch, and Codec Demolisher

Bends is a 6-stage sequential stereo multi-effect processor and glitch looper for the **Music Thing Workshop Computer**. Operating at 24kHz on the RP2040 second core, it combines chorusing, codec corruption, multi-tap delay, granular stutter/freeze, resonant state-variable filtering, and dual spring/hall reverb.

---

## Installation

1. Connect the Workshop Computer to your computer using a data-capable Micro-USB cable.
2. Put the board in bootloader mode: hold the **Secret Button** (under the main knob), press **Reset**, then release the Secret Button. A drive named `RPI-RP2` will mount.
3. Copy [bends.uf2](./bends.uf2) onto the mounted drive.
4. The board will reboot automatically.

---

## Physical I/O Patching

### Inputs
* **Audio In 1 & 2**: Stereo audio input. Unplugging `Audio In 2` normals input 1 to both channels.
* **Pulse In 1 (Stutter Clock / Trigger)**: Syncs delay and granular loop size to incoming pulse timing. Also acts as a gate trigger for granular stuttering.
* **Pulse In 2 (Freeze Gate)**: High gate (> +1.2V) locks the current delay and stutter buffers (equivalent to Switch UP).
* **CV In 1 (Macro / Crackle Injector)**: Modulates global grittiness, vinyl crackle, and CD-skip dropouts.
* **CV In 2 (Breakup / Stutter Injector)**: Modulates digital loss/packet dropouts. Gates above +1.2V trigger stutter loops.

### Outputs
* **Audio Out 1 & 2**: Stereo audio output with digital DC-blocking filters.
* **CV Out 1 & 2 (Harmonic Pitch CV)**: Outputs quantized pitch-representative CVs (1 semitone ≈ 68 DAC counts) updated on loop boundaries.
* **Pulse Out 1 (Loop Boundary)**: +5V trigger pulse (2ms) emitted on granular loop resets.
* **Pulse Out 2 (Texture / Pop Generator)**: Composite pulse output of vinyl crackle, clicks, and sub-harmonic loop ticks.

---

## Control Interface & Navigation

Bends features an 8-page virtual parameter table. Physical knobs (`Main`, `X`, `Y`) adjust parameters for the active page. Knob catch logic prevents parameter jumps when switching pages (the LED blinks at 5Hz to show pickup direction).

### Switch Operations
* **Flick DOWN (Short Press < 350ms)**: Cycles forward through parameter pages `0` to `5`.
* **Hold DOWN (Long Press >= 350ms)**: Enters **Macro Mode**. Knobs control global settings:
  * **Main**: Grittiness Macro sweep (0–100%)
  * **X**: Stereo Width & Buffer Mode:
    * *CCW*: **Extended Mono Mode** (~2.73s buffer when input 2 unplugged; LED 5 lights)
    * *Mid*: **Stereo Width** (0% to 100%)
    * *CW*: **Dual Mono Mode** (independent dual mono processing; LEDs 4 & 5 light)
  * **Y**: Global DSP Routing Preset:
    * `0` (CCW): **Series Standard** (`Chorus -> Codec -> Delay -> Glitcher -> Filter -> Reverb`)
    * `1`: **Space Wash** (`Reverb -> Filter -> Chorus -> Delay -> Glitcher -> Codec`)
    * `2`: **Scatter Cloud** (`Glitcher -> Codec -> Delay -> Chorus -> Filter -> Reverb`)
    * `3` (CW): **Filtered Dub** (`Filter -> Codec -> Chorus -> Delay -> Glitcher -> Reverb`)
  * Releasing the switch returns to normal operation.
  * **Save to Flash**: Hold DOWN for **3 seconds** without moving knobs to save settings to flash memory (LEDs pulse white).
* **Switch UP**: Toggles Freeze mode.
  * *First Press*: Latches freeze and jumps to **Page 3 (Glitcher / Freeze)** with playhead animation on the LED ring.
  * *Second Press*: Unfreezes audio and returns to the previous page.
* **Factory Reset**: Hold DOWN continuously for **3 seconds during boot** to reset all flash settings to defaults (LEDs flash red/pink).

---

## Parameter Pages

The glowing LED indicates the active page (0–5):

| Page | Effect Stage | Main Knob | Knob X | Knob Y |
|:---:|---|---|---|---|
| **0** | **Chorus / Tape Loss** | Chorus Mix | LFO Rate (Drift) | Chorus Depth / Feedback / BBD XOR |
| **1** | **Codec Demolisher** | Decimation Mix | Decimation Factor (Bitcrush) | Corruption (MP3 Ringing / Dropouts) |
| **2** | **Multi-Tap Delay** | Delay Wet/Dry Mix | Delay Time (Clock subdivisions if patched) | Delay Feedback (Overdrives into XOR glitches) |
| **3** | **Glitcher / Freeze** | *Unfrozen*: Glitch Mix<br>*Frozen*: Loop Scrub Position | *Unfrozen*: Loop Size<br>*Frozen*: Loop Length | *Unfrozen*: Playback Speed (Quantized)<br>*Frozen*: Playback Speed & Pitch |
| **4** | **Resonant Filter** | Cutoff (Lowpass CCW, Highpass CW) | Resonance (Q) | Drive & Wavefolder Fuzz |
| **5** | **Reverb Engine** | Reverb Wet/Dry Mix | Reverb Type & Size (CCW: Spring, CW: Ambient Hall) | Damping & Circuit-Bent Jitter |

---

## Visual Feedback (LED Ring)

* **Page Indicator**: The active page LED (0–5) glows steadily.
* **Freeze Playhead (Page 3)**: Low glow shows loop window; bright blinking LED shows playback head position.
* **Macro / Routing Display**: Shows macro level or selected routing preset (LED 0, 1, 2, or 3).
* **Routing Chain Visualization**: Plays audio signal path sequence on boot, save, or routing change.
