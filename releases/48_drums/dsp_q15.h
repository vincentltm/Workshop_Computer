// =============================================================================
// dsp_q15.h — Fixed-point DSP utilities for Elements port
//
// All audio processing on the RP2040 (Cortex-M0+, no FPU) uses Q15 fixed-point
// arithmetic. This header provides the fundamental math operations.
//
// Naming convention:
//   Q15: int16_t or int32_t with 15 fractional bits. ±1.0 = ±32767.
//   Q14: int16_t with 14 fractional bits. ±2.0 = ±16384 per unit.
//   Q16: int32_t with 16 fractional bits. Range depends on integer part.
//   Q20: int32_t with 20 fractional bits. Used for high-precision small values.
//   Q31: int32_t with 31 fractional bits. ±1.0 = ±2^31. Phase accumulators.
// =============================================================================

#ifndef DSP_Q15_H_
#define DSP_Q15_H_

#include <stdint.h>
#include "resources_q15.h"

// ── Q15 Arithmetic ──────────────────────────────────────────────────────────

/// Multiply two Q15 values → Q15 result.
static inline int32_t mul_q15(int32_t a, int32_t b) {
    return (a * b) >> 15;
}

/// Multiply Q15 × Q14 → Q15 result (for SVF g/r coefficients)
static inline int32_t mul_q15_q14(int32_t a_q15, int32_t b_q14) {
    return (a_q15 * b_q14) >> 14;
}

/// Multiply Q15 × Q16 → Q15 result
static inline int32_t mul_q15_q16(int32_t a_q15, int32_t b_q16) {
    return (a_q15 * b_q16) >> 16;
}

/// Multiply Q15 × Q20 → Q15 result (for envelope increments)
static inline int32_t mul_q15_q20(int32_t a_q15, int32_t b_q20) {
    return (int32_t)(((int64_t)a_q15 * b_q20) >> 20);
}

/// Saturate an int32 to Q15 range (±32767)
static inline int32_t sat_q15(int32_t x) {
    if (x > 32767)  return 32767;
    if (x < -32768) return -32768;
    return x;
}

/// Saturate to positive Q15 range (0..32767)
static inline int32_t sat_q15u(int32_t x) {
    if (x > 32767) return 32767;
    if (x < 0)     return 0;
    return x;
}

/// Absolute value
static inline int32_t abs_q15(int32_t x) {
    return x < 0 ? -x : x;
}

/// Simple soft clipper for Q15 range (±32767).
/// Threshold is 0.75 (24576). Above this, the curve compresses 4:1.
static inline int32_t SoftLimitQ15(int32_t x) {
    if (x > 24576) {
        int32_t delta = x - 24576;
        int32_t limited = 24576 + (delta >> 2);
        return (limited > 32767) ? 32767 : limited;
    }
    if (x < -24576) {
        int32_t delta = x + 24576;
        int32_t limited = -24576 + (delta >> 2);
        return (limited < -32768) ? -32768 : limited;
    }
    return x;
}

// ── Linear Interpolation ────────────────────────────────────────────────────

/// Interpolate a Q15 int16 table with 8-bit fractional index.
/// index = integral.fractional where integral is the array index
/// and fractional (0..255) is the sub-index.
///
/// Usage: InterpolateQ15(table, param_q15, table_size)
///   where param_q15 is 0..32767 and table_size is the index range.
///   Returns Q15 value.
static inline int32_t InterpolateQ15(const int16_t* table, int32_t index_q8) {
    int32_t i = index_q8 >> 8;
    int32_t f = index_q8 & 0xFF;
    int32_t a = table[i];
    int32_t b = table[i + 1];
    return a + (((b - a) * f) >> 8);
}

/// Interpolate a Q15 table using a Q15 parameter (0..32767) over a table of
/// size `table_size`. The parameter is scaled to a fixed-point index.
static inline int32_t InterpolateQ15_Param(const int16_t* table,
                                            int32_t param_q15,
                                            int32_t table_size) {
    // Scale param (0..32767) to index (0..table_size) in Q8 format
    int32_t index_q8 = (int32_t)(((int64_t)param_q15 * table_size) >> 7);
    return InterpolateQ15(table, index_q8);
}

/// Interpolate a Q16 int32 table with 8-bit fractional index.
static inline int32_t InterpolateQ16(const int32_t* table, int32_t index_q8) {
    int32_t i = index_q8 >> 8;
    int32_t f = index_q8 & 0xFF;
    int32_t a = table[i];
    int32_t b = table[i + 1];
    return a + (((int64_t)(b - a) * f) >> 8);
}

/// Interpolate an int32 table using a Q15 parameter
static inline int32_t InterpolateQ32_Param(const int32_t* table,
                                            int32_t param_q15,
                                            int32_t table_size) {
    int32_t index_q8 = (int32_t)(((int64_t)param_q15 * table_size) >> 7);
    return InterpolateQ16(table, index_q8);
}

// ── Crossfade / Mix ─────────────────────────────────────────────────────────

/// Linear crossfade: result = a + (b - a) * fade, where fade is Q15 (0..32767)
static inline int32_t CrossfadeQ15(int32_t a, int32_t b, int32_t fade_q15) {
    return a + (((b - a) * fade_q15) >> 15);
}

/// One-pole filter: out += coefficient * (in - out)
/// Coefficient is Q15 (0..32767 = 0..1.0)
static inline int32_t OnePoleQ15(int32_t out, int32_t in, int32_t coeff_q15) {
    return out + (int32_t)(((int64_t)(in - out) * coeff_q15) >> 15);
}

// ── Sine / Cosine ───────────────────────────────────────────────────────────

/// Look up sine from the Q15 sine table.
/// phase is a Q15 value (0..32767 = 0..2π, wrapping)
/// Returns Q15 (-32767..32767)
static inline int32_t SineQ15(int32_t phase_q15) {
    // Map Q15 phase (0..32767) to table index (0..4096) in Q8
    int32_t index_q8 = (int32_t)(((int64_t)(phase_q15 & 0x7FFF) * 4096) >> 7);
    return InterpolateQ15(lut_sine_q15, index_q8);
}

/// Look up sine from the Q15 table using a uint32 phase accumulator.
/// phase uses the full 32-bit range (0..0xFFFFFFFF = 0..2π)
/// Returns Q15 (-32767..32767)
static inline int32_t SineQ15_U32(uint32_t phase) {
    // Map top 12 bits to table index, next 8 bits for interpolation
    uint32_t index = phase >> 20;          // 0..4095
    uint32_t frac = (phase >> 12) & 0xFF;  // 0..255
    int32_t a = lut_sine_q15[index];
    int32_t b = lut_sine_q15[index + 1];
    return a + (((b - a) * (int32_t)frac) >> 8);
}

/// Cosine via sine table: cos(x) = sin(x + π/2)
static inline int32_t CosineQ15_U32(uint32_t phase) {
    return SineQ15_U32(phase + 0x40000000u);  // +90° = +0.25 of full range
}

// ── Soft Limiting ───────────────────────────────────────────────────────────

/// Soft limiter using rational approximation: x * (27 + x²) / (27 + 9x²)
/// Simple piecewise linear soft clip. Very cheap (no 64-bit division).
static inline int32_t SoftClipQ15(int32_t x) {
    if (x > 24000) {
        x = 24000 + ((x - 24000) >> 2);
    } else if (x < -24000) {
        x = -24000 + ((x + 24000) >> 2);
    }
    
    // Hard limit at Q15 bounds
    if (x > 32767) return 32767;
    if (x < -32768) return -32768;
    return x;
}

// ── DC Blocker ──────────────────────────────────────────────────────────────

/// First-order DC blocking filter.
/// state_x and state_y must persist between calls.
/// Pole at ~0.996 (>>8 ≈ 1/256 ≈ 0.004 error coefficient)
static inline int32_t DCBlockQ15(int32_t x, int32_t &px, int32_t &py) {
    int32_t py_div = (py > 0) ? ((py + 255) >> 8) : ((py < 0) ? ((py - 255) >> 8) : 0);
    int32_t y = x - px + py - py_div;
    px = x;
    py = y;
    return y;
}

// ── PRNG ────────────────────────────────────────────────────────────────────

/// Fast 32-bit PRNG (linear congruential)
static inline uint32_t FastRandQ15(uint32_t &seed) {
    seed = 1103515245u * seed + 12345u;
    return seed;
}

/// Random Q15 sample (-32768..32767) for noise generation
static inline int32_t RandomQ15(uint32_t &seed) {
    return (int32_t)(FastRandQ15(seed) >> 17) - 16384;  // ~±16384
}

// ── Pitch Utilities ─────────────────────────────────────────────────────────

/// Convert a MIDI pitch (Q8 format: pitch * 256) to a phase increment.
/// pitch_q8 should be in range 0..32767 (maps to MIDI 0..127.99)
/// Returns a uint32 phase increment for a uint32 accumulator.
static inline uint32_t MidiToIncrementU32(int32_t pitch_q8) {
    // pitch_q8 is MIDI * 256. Index 107 in LUT is MIDI 69 (A4).
    // So index = (pitch_q8 >> 8) + (107 - 69) = pitch + 38.
    int32_t index_q8 = pitch_q8 + (38 << 8);
    if (index_q8 < 0) index_q8 = 0;
    if (index_q8 > 65280) index_q8 = 65280;
    
    int32_t hi = index_q8 >> 8;
    int32_t lo = index_q8 & 0xFF;
    
    uint32_t inc_hi = lut_midi_to_increment_high_u32[hi];
    int32_t f_lo = lut_midi_to_f_low_q15[lo];
    
    return (uint32_t)(((uint64_t)inc_hi * (uint32_t)f_lo) >> 15);
}

/// Convert a MIDI pitch (Q8) to a frequency ratio (Q8).
static inline int32_t MidiToFreqRatioQ8(int32_t pitch_q8) {
    int32_t index_q8 = pitch_q8 + (38 << 8);
    if (index_q8 < 0) index_q8 = 0;
    if (index_q8 > 65280) index_q8 = 65280;
    
    int32_t hi = index_q8 >> 8;
    int32_t lo = index_q8 & 0xFF;
    
    int32_t f_hi = lut_midi_to_f_high_q8[hi];
    int32_t f_lo = lut_midi_to_f_low_q15[lo];
    
    return (int32_t)(((int64_t)f_hi * f_lo) >> 15);
}

// ── Parameter Scaling ───────────────────────────────────────────────────────

/// Scale a Q15 parameter (0..32767) through the 4-decades exponential table.
/// Returns Q16 value in range ~0.001..10.0 (as integer × 65536).
static inline int32_t FourDecadesQ16(int32_t param_q15) {
    return InterpolateQ32_Param(lut_4_decades_q16, param_q15, LUT_4_DECADES_SIZE);
}

/// Look up accent gain from coarse + fine tables.
/// strength is Q15 (0..32767). Returns Q14 gain.
static inline int32_t AccentGainQ14(int32_t strength_q15) {
    int32_t index_q8 = (int32_t)(((int64_t)strength_q15 * LUT_SVF_SIZE) >> 7);
    int32_t hi = index_q8 >> 8;
    int32_t lo = index_q8 & 0xFF;
    
    // Use coarse table for main value, fine for interpolation
    int32_t coarse = InterpolateQ16(lut_accent_gain_coarse_q14, hi << 8);
    int32_t fine = InterpolateQ15(lut_accent_gain_fine_q15, lo << 8);
    
    return (int32_t)(((int64_t)coarse * fine) >> 15);
}

// ── Cosine Oscillator (for resonator position) ─────────────────────────────

/// Efficient cosine oscillator using recursive IIR.
/// Generates cos(2πf) for a given normalized frequency.
/// All state in Q15.
struct CosineOscQ15 {
    uint32_t phase;
    uint32_t phase_inc;
    
    void Init(int32_t freq_q15) {
        // freq_q15 (0..32767) maps to 0..0.5 normalized frequency
        // phase_inc = freq * 2^32
        phase_inc = (uint32_t)freq_q15 << 17;
        Start();
    }
    
    void Start() {
        phase = phase_inc;
    }
    
    void NextQuadrature(int32_t &c, int32_t &s) {
        // c = cos(phase), s = sin(phase)
        c = lut_sine_q15[(phase + 1073741824u) >> 20];
        s = lut_sine_q15[phase >> 20];
        phase += phase_inc;
    }
};

#endif  // DSP_Q15_H_
