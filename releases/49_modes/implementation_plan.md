# 49_modal: Mutable Instruments Elements → Workshop Computer

## Implementation Plan

> [!IMPORTANT]
> This plan is designed to be handed to any LLM or developer. It contains all architectural decisions, constraints, and step-by-step instructions needed to execute the port.

---

## 1. Project Overview

**Goal:** Port Mutable Instruments Elements (modal synthesis) from the SuperCollider mi-UGens port to the Music Thing Modular Workshop Computer (RP2040).

**Source code:** `lib/mi-UGens-main/eurorack/elements/` — Émilie Gillet's original C++ DSP code, adapted by Volker Böhm for SuperCollider.

**Target:** RP2040 (dual-core Cortex-M0+, 125MHz default / 144MHz recommended, **no FPU**, 264KB RAM, 2MB flash).

**Reference project:** `releases/51_grains/grains.cpp` — proven IO/UX patterns for the Workshop Computer.

---

## 2. Critical Constraints Analysis

### 2.1 Memory Budget — THE HARD WALL

| Resource | Original Size | RP2040 Budget |
|----------|--------------|---------------|
| `resources.cc` float LUTs | 32.8 KB | Must convert to Q15 int16_t → **~16.4 KB** |
| `smp_sample_data` (exciter samples) | 250 KB (128013 × int16) | **Cannot fit in RAM.** Must read from flash (XIP) |
| `smp_noise_sample` | 80 KB (40963 × int16) | **Cannot fit in RAM.** Must read from flash (XIP) |
| Reverb buffer | 64 KB (32768 × uint16) | Too large. Replace with lightweight reverb (~8-12 KB) |
| Resonator SVFs (64 modes) | 1.3 KB state + heavy CPU | Reduce to **24-32 modes** |
| String delay lines (5×2KB) | 40 KB (float) | Convert to int16 → **20 KB** |
| Bow delay lines (8×1KB) | 32 KB (float) | Convert to int16 → **8 KB** or reduce |
| **Total RP2040 RAM** | — | **264 KB (must fit everything + stack + ComputerCard)** |

> [!CAUTION]
> The original `resources.cc` contains 1.6MB of data. The 330KB of sample data **must** be stored in flash and accessed via XIP (execute-in-place) pointers, exactly as `51_grains` does with `getFlashSamplePtr()`. The float lookup tables must be converted to Q15 int16_t or Q16 int32_t.

### 2.2 CPU Budget — NO FLOATING POINT

The RP2040 Cortex-M0+ has **no FPU**. Every `float` operation is software-emulated (~20-50 cycles per multiply). The original Elements code is **100% float**.

| Operation | Cost on M0+ |
|-----------|-------------|
| `int32 × int32` | 1 cycle (hardware MUL) |
| `float × float` | ~20-50 cycles (software) |
| `sinf()` | ~200+ cycles |
| `tanf()` | ~300+ cycles |
| `logf()` | ~200+ cycles |

**Strategy:** Convert ALL DSP to Q15 (signed 16-bit, ±1.0 = ±32767) or Q16 (32-bit with 16 fractional bits) fixed-point arithmetic using `int32_t` with `>>` shifts.

### 2.3 Sample Rate

Original Elements runs at 32kHz. Workshop Computer runs at 48kHz (ComputerCard default). Options:

- **Option A (Recommended):** Run DSP at 24kHz (every other ProcessSample), matching the `51_grains` approach of `gPh >= 2`. This gives 2× CPU headroom and is closer to the original 32kHz.
- **Option B:** Run at 48kHz but with reduced mode count (~16 modes max). Very tight CPU budget.
- **Option C:** Run at 16kHz with 3:1 decimation. Lower quality but maximum headroom.

**Decision: Option A — 24kHz DSP rate, using every-other-sample pattern from grains.**

---

## 3. Architecture Design

### 3.1 Dual-Core Split

```
┌─────────────────────────────────────────┐
│ CORE 0 — Audio Interrupt (48kHz)        │
│                                          │
│  ProcessSample() {                       │
│    • Read knobs, CVs, switch, pulse      │
│    • Parameter smoothing (IIR)           │
│    • Every 2nd sample:                   │
│      - Send params to Core 1 via FIFO   │
│      - Receive audio from Core 1         │
│    • Output audio to DAC                 │
│    • UI state machine (page switching)   │
│  }                                       │
└─────────────────────────────────────────┘

┌─────────────────────────────────────────┐
│ CORE 1 — DSP Engine (24kHz effective)   │
│                                          │
│  while(1) {                              │
│    • Wait for params from Core 0 (FIFO) │
│    • Run Exciter(s)                      │
│    • Run Resonator (modal/string)        │
│    • Lightweight reverb/space            │
│    • Send stereo audio back via FIFO     │
│  }                                       │
└─────────────────────────────────────────┘
```

**FIFO Protocol (multicore_fifo):**
- Core 0 → Core 1: packed parameter word(s) + gate/trigger flags
- Core 1 → Core 0: left sample (int16), right sample (int16), status

### 3.2 DSP Signal Flow (Simplified for RP2040)

```
                   ┌──────────┐
  Audio In 1 ────→ │ BLOW IN  │──→┐
                   └──────────┘   │
                                   │   ┌──────────────┐
  Gate (Pulse 1) ────────────────→├──→│  EXCITER MIX  │
                                   │   │  (bow+blow+   │
                   ┌──────────┐   │   │   strike)     │
  Audio In 2 ────→ │STRIKE IN │──→┘   └──────┬───────┘
                   └──────────┘              │
                                              ▼
                                   ┌──────────────┐     ┌────────┐
                                   │  RESONATOR   │────→│ SPACE  │──→ Audio Out 1/2
                                   │  (modal or   │     │(reverb)│
                                   │   string)    │     └────────┘
                                   └──────────────┘
```

### 3.3 What to Cut / Simplify

| Original Feature | Decision | Rationale |
|-----------------|----------|-----------|
| Easter egg (OminousVoice) | **CUT entirely** | It's an 8× oversampled FM synth with FIR downsampler — far too expensive |
| Granular sample player exciter | **CUT** | Requires large sample data, complex |
| Sample player exciter | **SIMPLIFY** — use a few short samples from flash | Reduce sample count from 9 to 3-4 |
| Full reverb (FxEngine) | **REPLACE** with plate reverb from 51_grains | Already proven on RP2040, ~8KB |
| 64 resonator modes | **REDUCE** to 24-32 | CPU bound; 24 modes still sounds excellent |
| 5 string models | **REDUCE** to 3 | Memory bound (delay lines) |
| Tube waveguide | **KEEP** but convert to fixed-point | Small and important for blow exciter |
| Diffuser | **KEEP** | Small allpass chain, easy to convert |
| Multistage envelope | **KEEP** but convert LUTs to Q15 | Essential for exciter gating |

---

## 4. IO / UX Design — 6-Page System

### 4.1 Physical Controls Mapping

```
Hardware:
  3 Knobs:  Main, X, Y  (0–4095)
  1 Switch: (ON)-OFF-ON  (Down=momentary, Up=toggle)
  6 LEDs:   0-5 in 2×3 grid
  2 Audio In/Out, 2 CV In/Out, 2 Pulse In/Out
```

### 4.2 Page Layout

Elements has 20 parameters. We map them across **6 pages** (indicated by LEDs):

| Page | LED | Main Knob | X Knob | Y Knob |
|------|-----|-----------|--------|--------|
| **0: Exciter Levels** | LED 0 | Strike Level | Blow Level | Bow Level |
| **1: Exciter Timbre** | LED 1 | Strike Timbre | Blow Timbre (Flow) | Bow Timbre |
| **2: Exciter Shape** | LED 2 | Strike Meta (mallet→particles) | Blow Meta (granular→noise) | Envelope Shape (contour) |
| **3: Resonator Core** | LED 3 | Geometry (partials) | Brightness | Damping |
| **4: Resonator Space** | LED 4 | Position (pickup) | Space (reverb) | Resonator Model (modal/string/strings) |
| **5: Performance** | LED 5 | Pitch (coarse) | Strength | — (reserved/fine tune) |

### 4.3 CV / Audio / Pulse Mapping

| Jack | Function |
|------|----------|
| **Audio In 1** | Blow exciter external input |
| **Audio In 2** | Strike exciter external input |
| **Audio Out 1** | Main output (center) |
| **Audio Out 2** | Aux output (sides/reverb) |
| **CV In 1** | V/Oct pitch (±5V) |
| **CV In 2** | Strength / accent |
| **CV Out 1** | Exciter level meter |
| **CV Out 2** | Resonator level meter |
| **Pulse In 1** | Gate (exciter trigger) |
| **Pulse In 2** | Model select / mode toggle |
| **Pulse Out 1** | Envelope end-of-cycle |
| **Pulse Out 2** | Gate passthrough |

### 4.4 Switch Functions

- **Down (momentary):** Tap = cycle page forward. Hold >1.5s = cycle backward.
- **Up (toggle):** Toggle resonator model (Modal → String → Strings → Modal)
- **Middle:** Normal operation.

### 4.5 LED Feedback

```
| 0 1 |  ← Page indicators (current page LED = ON, others OFF)
| 2 3 |  ← LED 2,3: exciter/resonator activity meters
| 4 5 |  ← LED 4: gate indicator, LED 5: model indicator
```

During page display (first 500ms after page change):
- Current page LED = bright ON
- All others = OFF

During normal operation:
- Current page LED = dim ON (brightness 512)
- LED 4 = gate activity (follows Pulse In 1)
- LED 5 = resonator model indicator (off=modal, on=string, blinking=strings)

---

## 5. Fixed-Point Conversion Strategy

### 5.1 Number Formats

| Format | Range | Precision | Use Case |
|--------|-------|-----------|----------|
| **Q15** (int16_t) | ±1.0 (±32767) | 1/32768 ≈ 0.00003 | Audio samples, filter state, small coefficients |
| **Q16** (int32_t) | ±32768.0 | 1/65536 ≈ 0.000015 | Frequency, phase accumulators, larger ranges |
| **Q31** (int32_t) | ±1.0 (±2^31) | Very high | Phase accumulators (wrapping) |
| **Q0** (int32_t) | ±2^31 | 1 | Intermediate multiply results before shift |

### 5.2 Conversion Rules

```cpp
// Float → Q15:  q15_value = (int16_t)(float_value * 32767.0f)
// Q15 → Float:  float_value = (float)q15_value / 32767.0f

// Multiply two Q15 values:
// result_q15 = (int16_t)(((int32_t)a * b) >> 15);

// Multiply Q15 × Q16:
// result_q15 = (int16_t)(((int32_t)a * (b >> 1)) >> 15);

// SVF filter in Q15:
// hp = ((int32_t)(in - ((r * s1) >> 15) - ((g * s1) >> 15) - s2) * h) >> 15;
// bp = ((int32_t)g * hp >> 15) + s1;
// s1 = ((int32_t)g * hp >> 15) + bp;
// lp = ((int32_t)g * bp >> 15) + s2;
// s2 = ((int32_t)g * bp >> 15) + lp;
```

### 5.3 Lookup Table Conversion

All float LUTs in `resources.cc` must be converted to Q15 int16_t:

```cpp
// Original (float):
// const float lut_approx_svf_g[] = { 0.001f, 0.002f, ... };

// Converted (Q15):
// const int16_t lut_approx_svf_g_q15[] = { 33, 66, ... };
// (each value = original * 32767)

// Interpolation helper:
inline int32_t InterpolateQ15(const int16_t* table, int32_t index_q8) {
    int32_t i = index_q8 >> 8;
    int32_t f = index_q8 & 0xFF;
    int32_t a = table[i];
    int32_t b = table[i + 1];
    return a + (((b - a) * f) >> 8);
}
```

### 5.4 Critical Math Replacements

| Original | Fixed-Point Replacement |
|----------|------------------------|
| `sinf(x)` | 1024-entry Q15 sine LUT with linear interpolation (from grains) |
| `tanf(π*f)` | Pre-computed in SVF LUTs (already done in original via `lut_approx_svf_g`) |
| `SemitonesToRatio(x)` | `exp2_q16()` function (from grains) |
| `logf(x)` | Approximation or pre-computed LUT |
| `fabsf(x)` | `x > 0 ? x : -x` |
| `sqrtf(x)` | Newton's method integer approximation or LUT |
| `Random::GetWord()` | `fast_rand()` (from grains) |
| `a + (b-a) * frac` | `a + (((b - a) * frac) >> 15)` |

---

## 6. Phased Development Plan

### Phase 0: Skeleton & Build System ✅ DONE
**Commit:** `662667d` — **Files:** `modal.cpp`, `CMakeLists.txt`

Delivered: IO/UX skeleton with 6-page navigation, knob locking, dual-core FIFO, boot animation. Build: 20.5KB flash, 23.5KB RAM.

### Phase 1: Fixed-Point Infrastructure ✅ DONE
**Commit:** `38bb72d` — **Files:** `dsp_q15.h`, `svf_q15.h`, `envelope_q15.h`, `resources_q15.h/cpp`, `tools/convert_luts.py`

Delivered:
- `dsp_q15.h` (13KB): Q15/Q14/Q16 multiply, interpolation, sine/cosine, soft limiter, DC blocker, PRNG, pitch conversion, cosine oscillator
- `svf_q15.h` (10KB): Zero-delay-feedback SVF (LP/BP/HP/BPN), single-sample and block processing
- `envelope_q15.h` (8KB): Multistage envelope (AD/AR/ADSR/ADR/looped)
- `resources_q15.cpp` (82KB source → 19.5KB data): 19 LUT arrays converted from float
- `tools/convert_luts.py`: Automated float→Q15/Q16 converter
- Build: 20.5KB flash, 23.5KB RAM (LUTs not yet linked)

### Phase 2: Exciter Port ✅ DONE
**Commit:** `13a466d` — **Files:** `exciter_q15.h`, `modal.cpp` (DSP loop)

Delivered:
- `exciter_q15.h` (10KB): 5 excitation models (Mallet, Plectrum, Particles, Flow, Noise)
- Core 1 DSP loop: full exciter processing per sample with envelope, accent, mix logic
- Envelope shape parameter maps AD/ADSR/AR (faithfully ported from voice.cc)
- Strike meta sweeps Mallet→Plectrum→Particles
- Bow strength, damping feedback, strike bleed all computed
- Sample player exciter SIMPLIFIED — use a few short samples from flash
- Tube waveguide ✅ DONE (parallel feedback delay model for flute sounds)
- Diffuser ✅ DONE (4x allpass chain with "Mallet Smearing" logic)
- Particles & Scattering ✅ DONE (stochastic impulse cloud with jitter and timbre randomization)
- Build: 32KB flash, 38KB RAM (57% total RAM used)

### Phase 3: Resonator Port ✱ NEXT
**Files:** `resonator_q15.h`, `diffuser_q15.h` (DONE), `tube_q15.h` (DONE), `exciter_q15.h` (SCATTERING DONE)

1. Port `Resonator` (modal synthesis) to Q15:
   - Start with **24 modes** (adjustable)
   - SVF bank uses `SvfQ15` from Phase 1
   - `ComputeFilters()` recomputes coefficients every N samples
   - `CosineOscillator` for position-dependent mixing → `CosineOscQ15`
   - Bow table (friction characteristic) → Q15 rational approximation
2. Wire resonator into Core 1 loop after exciter
3. Test: full exciter → resonator chain produces pitched modal sounds
4. If time/RAM permits: port String model (Karplus-Strong)

**Deliverable:** Full modal synthesis voice producing musical output.

### Phase 4: Space / Reverb & Output Stage
**Files:** `reverb_lite.h` (or reuse grains' `PlateReverb`)

1. Use the `PlateReverb` from `51_grains` directly (proven, ~36KB int16 buffers)
2. Implement the "space" metaparameter:
   - 0.0–0.05: raw exciter signal bleed
   - 0.05–0.1: crossfade to reverb
   - 0.1–0.8: increasing reverb amount
   - 0.8–1.0: reverb time increases toward freeze
3. Stereo spread from position LFO (simple triangle LFO in Q15)
4. Output soft-limiter (`SoftLimitQ15`)
5. Wire into full signal chain

**Deliverable:** Complete Elements voice with space/reverb.

### Phase 5: Polish & Optimize

1. CPU profiling: toggle GPIO pin, measure with scope
2. Optimize hot loops:
   - `__not_in_flash_func()` on all DSP functions
   - Unroll inner loops where beneficial
   - Consider `pico_set_binary_type(copy_to_ram)` if it fits
3. Knob response refinement:
   - Dead zones at extremes
   - Exponential curves for musical parameters
4. CV input calibration:
   - V/Oct tracking on CV In 1
   - Smooth CV In 2 for strength
5. LED metering:
   - Exciter level → LED brightness
   - Resonator level → LED brightness
6. Final memory audit and optimization

**Deliverable:** Production-ready program card.

---

## 7. File Structure

```
releases/49_modal/
├── CMakeLists.txt              # Build configuration
├── modal.cpp                   # Main application (ComputerCard class + Core 1)
├── dsp_q15.h                   # Fixed-point math utilities
├── svf_q15.h                   # Fixed-point SVF filter
├── envelope_q15.h              # Fixed-point multistage envelope
├── exciter_q15.h               # Fixed-point exciter (header)
├── exciter_q15.cpp             # Fixed-point exciter (implementation)
├── tube_q15.h                  # Fixed-point tube waveguide
├── diffuser_q15.h              # Fixed-point diffuser (4x allpass)
├── resonator_q15.h             # Fixed-point modal resonator (header)
├── resonator_q15.cpp           # Fixed-point modal resonator (implementation)
├── string_q15.h                # Fixed-point string model (header)
├── string_q15.cpp              # Fixed-point string model (implementation)
├── resources_q15.h             # Converted LUTs (header)
├── resources_q15.cpp           # Converted LUTs (data)
├── voice_q15.h                 # Voice orchestrator (header)
├── voice_q15.cpp               # Voice orchestrator (implementation)
├── reverb_lite.h               # Lightweight plate reverb (from grains)
├── ComputerCard/               # Symlink or copy of ComputerCard library
├── pico_sdk_import.cmake       # Pico SDK import
├── info.yaml                   # Card metadata
├── README.md                   # Documentation
└── lib/                        # Original MI source (reference only)
    └── mi-UGens-main/
```

---

## 8. Conversion Cookbook (for LLMs)

### Converting a float SVF filter to Q15

```cpp
// ORIGINAL (float):
// hp = (in - r_ * state_1_ - g_ * state_1_ - state_2_) * h_;
// bp = g_ * hp + state_1_;

// CONVERTED (Q15):
// All values are int32_t (wide) during computation, stored as int16_t
int32_t hp = (((int32_t)in - mul_q15(r, s1) - mul_q15(g, s1) - s2) * h) >> 15;
int32_t bp = mul_q15(g, hp) + s1;
s1 = mul_q15(g, hp) + bp;  // same as bp but we store the double-integrated value
int32_t lp = mul_q15(g, bp) + s2;
s2 = mul_q15(g, bp) + lp;

// Where:
static inline int32_t mul_q15(int32_t a, int32_t b) {
    return (int32_t)(((int64_t)a * b) >> 15);
}
```

### Converting a float LUT interpolation to Q15

```cpp
// ORIGINAL:
// float result = Interpolate(lut_stiffness, geometry_, 256.0f);
//   → index = geometry_ * 256.0f
//   → a = table[integral], b = table[integral+1]
//   → return a + (b - a) * fractional

// CONVERTED (parameter is Q15 0..32767, table is Q15):
int32_t InterpolateQ15(const int16_t* table, int32_t param_q15, int32_t table_size) {
    int32_t index = (param_q15 * table_size) >> 15;  // Q15 * int = scaled index
    int32_t i = index >> 8;                           // integral part
    int32_t f = index & 0xFF;                         // fractional (0-255)
    int32_t a = table[i];
    int32_t b = table[i + 1];
    return a + (((b - a) * f) >> 8);
}
```

### Converting float audio to Workshop Computer I/O

```cpp
// Elements internal: float ±1.0 (approximately)
// Workshop Computer: int16_t ±2047 (12-bit signed)

// After resonator output (Q15 range ±32767):
int32_t out = resonator_output_q15;

// Apply soft limit
out = SoftLimitQ15(out);

// Scale to 12-bit DAC range
out = out >> 4;  // 32767 >> 4 = 2047

// Clamp and output
if (out > 2047) out = 2047;
if (out < -2048) out = -2048;
AudioOut1((int16_t)out);
```

---

## 9. Risk Assessment

| Risk | Impact | Mitigation |
|------|--------|------------|
| CPU overrun at 24kHz with 24 modes | HIGH | Profile early; reduce to 16 modes if needed |
| Fixed-point precision causes filter instability | MEDIUM | Use int32_t for filter state (not int16_t); add saturation |
| Resources too large for flash | LOW | Sample data is 330KB, flash is 2MB; plenty of room |
| Knob resolution insufficient for subtle params | MEDIUM | Use exponential curves; CV inputs for fine control |
| Inter-core FIFO latency adds jitter | LOW | Use blocking FIFO; Core 1 runs free, Core 0 syncs |

---

## 10. License

Original code: MIT (Émilie Gillet) + GPL3 (Volker Böhm's SC port).
The Workshop Computer port should be GPL3 to satisfy the most restrictive license in the chain.

Credit: Émilie Gillet (Mutable Instruments), Volker Böhm (mi-UGens SC port), Music Thing Modular (Workshop Computer platform).
