# Implementation Plan: SoundFont Optimizer & Hardware UX Bank Selector

We are adding advanced SoundFont optimization options to the Web Manager, real-time size updates, and a redesigned hardware preset/bank selection system to make navigating 128 General MIDI instruments on the Workshop Computer effortless and professional.

---

## User Review Required

Please review the following proposed architectural and UX features:

> [!IMPORTANT]
> **Real-Time Size Estimation & Active Preset Filtering:**
> - The Web Manager will list all presets (instruments) with **individual checkboxes**!
> - When you uncheck an instrument, its sample payload will be **completely stripped** from the final repacked SoundFont.
> - The UI will estimate and display the new SoundFont size and flash usage **immediately** as you toggle checkboxes, change target sample rates, or toggle stereo-to-mono, without needing to perform the actual repack first!

> [!TIP]
> **Sequential Auto-Splitting into Banks of 16:**
> - When loading 128 General MIDI presets, scrolling through all of them using a single knob is jumpy and hard to control.
> - We will implement a Web Manager option to **restructure melodic presets into sequential banks of 16** (mapping presets 0-15 to Bank 0, 16-31 to Bank 1, etc., while leaving drums in Bank 128).
> - On Page 4 (Bank Select page), the **X knob** will select from the actually available banks in the SoundFont.
> - In the Preset Menu, the **Main knob** will scroll *only* through presets present in the active bank, and the 4 LEDs will display the preset's index (0-15) in pristine 4-bit binary!

---

## Proposed Changes

### 1. Web Manager UI & Packager
#### [MODIFY] [samples_manager.html](file:///Users/vmaurer/Music/Workshop_Computer/releases/46_samples/web/samples_manager.html)
- **Dynamic Size Estimator:** Implements `estimateNewSize()` which calculates on-the-fly estimated bytes of:
  - Header chunks (`phdr`, `shdr`, etc.).
  - PCM sample payload, factoring in downsampling ratio ($R = s.sampleRate / targetRate$) and stereo-to-mono stripping.
- **Instrument Checkbox Tree:** Renders checkboxes next to presets with "Select All" / "Deselect All" options. Unchecked presets are excluded during optimization.
- **Bank Restructuring Option:** Adds a checkbox `"Restructure into Banks of 16"`. If active, the `phdr` chunk repacker will re-map the `.wBank` and `.wPreset` fields of all checked melodic presets to Bank 0, 1, 2... and Presets 0-15 sequentially.
- **Selective Sample Repacker:** Modifies `optimizeSF2()` to:
  1. Identify only samples referenced by *checked* presets.
  2. Discard unreferenced samples from the final binary, yielding massive space savings.

### 2. Hardware Synthesis Firmware
#### [MODIFY] [samples.cpp](file:///Users/vmaurer/Music/Workshop_Computer/releases/46_samples/samples.cpp)
- **Dynamic Bank Indexing:** At startup, `parse_sf2()` will index up to 16 unique bank numbers present in the file and sort them in ascending order.
- **X Knob Bank Selection:** On Page 4 (Bank Select), the **X knob** (`smoothX`) selects from the sorted list of `unique_banks`, supporting custom or auto-split banks seamlessly.
- **Scoped Preset Selection:** In the Preset Menu (`STATE_PRESET_MENU`), the **Main knob** scrolls only through the presets actually present in the currently selected `activeBank` (up to 32 presets).
- **Binary LED UX:** In the Preset Menu, LEDs 0-3 display the active preset index in the bank (0-15) as binary feedback.

---

## Verification Plan

### Automated Verification
We will verify compilation of the card's firmware using:
```bash
cd /Users/vmaurer/Music/Workshop_Computer/releases/46_samples/build
cmake .. && make -j4
```

### Manual Verification
1. **Dynamic Size Updates:** Load `LiteGM_v1.03.sf2`, toggle checkboxes and change sample rates, verify the size estimation updates instantly and matches the actual optimized size after repacking.
2. **Selective Stripping:** Uncheck all but 4 presets (e.g. Piano, Bass, Synth Lead, Drums) and verify the exported file size is extremely small (< 2MB).
3. **Restructuring:** Enable "Banks of 16", optimize, and verify that presets are successfully mapped to Bank 0 (Presets 0-15), Bank 1 (Presets 0-15), etc.
4. **Hardware Navigation:** Flash the restructured `.sf2`, navigate to Page 4 and verify the X knob shifts banks, then hold the switch to verify the Main knob scrolls through exactly the 16 presets inside that bank, with LEDs showing the binary value of the selection.
