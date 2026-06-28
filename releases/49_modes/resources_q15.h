// =============================================================================
// resources_q15.h — Fixed-point lookup tables for Elements port
//
// Auto-generated from MI Elements resources.cc via tools/convert_luts.py
// All float LUTs converted to Q15 (int16_t, ÷32767) or Q16 (int32_t, ÷65536)
// =============================================================================

#ifndef RESOURCES_Q15_H_
#define RESOURCES_Q15_H_

#include <stdint.h>
#include <stddef.h>

// ── Sine table (Q15, ±32767 = ±1.0, 4097 entries, one full period) ──────────
extern const int16_t lut_sine_q15[];       // [4097]

// ── SVF filter coefficients (pre-computed for 257 frequency steps) ───────────
// g: frequency coefficient (Q14, ÷16384 — values can exceed 1.0)
// r: 1/Q resonance coefficient (Q14, ÷16384 — values can exceed 1.0)
// h: 1/(1+r*g+g²) normalization (Q15, ÷32767)
// gain: overall gain compensation (Q15, ÷32767)
extern const int16_t lut_approx_svf_g_q14[];     // [257]
extern const int16_t lut_approx_svf_r_q14[];     // [257]
extern const int16_t lut_approx_svf_h_q15[];     // [257]
extern const int32_t lut_approx_svf_gain_q15[];  // [257]

// ── Stiffness table (Q15, maps geometry to string inharmonicity) ────────────
extern const int16_t lut_stiffness_q15[];  // [257]

// ── 4-decades exponential scaling (Q16 int32, range 0.001..10+) ─────────────
extern const int32_t lut_4_decades_q16[];  // [257]

// ── Accent gain (for velocity/strength scaling) ─────────────────────────────
extern const int32_t lut_accent_gain_coarse_q14[];  // [257] Q14 int32
extern const int16_t lut_accent_gain_fine_q15[];    // [257] Q15

// ── Envelope tables ─────────────────────────────────────────────────────────
extern const int32_t lut_env_increments_q20[];  // [258] phase increment (Q20)
extern const int16_t lut_env_linear_q15[];      // [258] linear shape
extern const int16_t lut_env_expo_q15[];        // [258] exponential shape
extern const int16_t lut_env_quartic_q15[];     // [258] quartic shape

// Pointer table for envelope shapes (indexed by EnvelopeShape enum)
extern const int16_t* const lut_env_shapes_q15[];  // [3]

// ── MIDI pitch tables ───────────────────────────────────────────────────────
// Used for pitch-to-frequency and pitch-to-phase-increment conversion.
// midi_to_f: freq_ratio = f_high[note>>8] * f_low[note&0xFF] * base_freq
// midi_to_increment: phase_increment = inc_high[note>>8] * f_low[note&0xFF]
extern const int32_t lut_midi_to_f_high_q8[];         // [256] Q8 (÷256)
extern const int16_t lut_midi_to_f_low_q15[];          // [256] Q15
extern const uint32_t lut_midi_to_increment_high_u32[]; // [256] raw phase increment

// ── SVF shift table (for resonator frequency spread) ────────────────────────
extern const int16_t lut_svf_shift_q15[];  // [257]

// ── LED brightness (already int16 in original) ──────────────────────────────
extern const int16_t lut_db_led_brightness[];  // [513]

// ── Table sizes ─────────────────────────────────────────────────────────────
#define LUT_SINE_SIZE          4096   // Phase range: 0..4096 maps to 0..2π
#define LUT_SVF_SIZE           256    // Index range: 0..256
#define LUT_STIFFNESS_SIZE     256
#define LUT_4_DECADES_SIZE     256
#define LUT_ENV_SIZE           257    // 258 entries, index range 0..257
#define LUT_MIDI_SIZE          255    // 256 entries, index range 0..255
#define LUT_SVF_SHIFT_SIZE     256

// ── Envelope shape enum (matches original) ──────────────────────────────────
enum EnvShapeQ15 {
    ENV_SHAPE_LINEAR_Q15   = 0,
    ENV_SHAPE_EXPO_Q15     = 1,
    ENV_SHAPE_QUARTIC_Q15  = 2
};

#endif  // RESOURCES_Q15_H_
