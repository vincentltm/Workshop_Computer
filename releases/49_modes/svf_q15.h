// =============================================================================
// svf_q15.h — Fixed-point State Variable Filter for Elements port
//
// Port of stmlib::Svf to Q15 integer arithmetic.
// Zero-delay-feedback topology (Vadim Zavalishin / Andrew Simper).
//
// The SVF provides simultaneous low-pass, band-pass, and high-pass outputs.
// It is the core building block of the Elements resonator (one SVF per mode).
//
// Coefficients:
//   g (Q14): frequency coefficient = tan(π * f / sr)
//   r (Q14): damping = 1/Q
//   h (Q15): normalization = 1/(1 + r*g + g²)
//
// These can be set directly from the pre-computed LUTs in resources_q15.
// =============================================================================

#ifndef SVF_Q15_H_
#define SVF_Q15_H_

#include <stdint.h>
#include "dsp_q15.h"

// ── Filter Mode ─────────────────────────────────────────────────────────────
enum FilterModeQ15 {
    FILT_LP = 0,    // Low-pass
    FILT_BP = 1,    // Band-pass
    FILT_BPN = 2,   // Band-pass normalized (BP * r)
    FILT_HP = 3     // High-pass
};

// ── State Variable Filter ───────────────────────────────────────────────────

struct SvfQ15 {
    int32_t g;          // Frequency coefficient (Q14)
    int32_t r;          // Damping 1/Q (Q14)
    int32_t h;          // Normalization (Q15)
    int32_t state1;     // Band-pass state (Q15)
    int32_t state2;     // Low-pass state (Q15)
    
    void Init() {
        g = 0;
        r = 16384;  // Q14 = 1.0 (moderate damping)
        h = 16384;  // Q15 = 0.5
        Reset();
    }
    
    void Reset() {
        state1 = 0;
        state2 = 0;
    }
    
    /// Set all three coefficients directly from LUT values.
    /// g_q14, r_q14 from lut_approx_svf_g/r, h_q15 from lut_approx_svf_h.
    void SetGRH(int32_t g_q14, int32_t r_q14, int32_t h_q15) {
        g = g_q14;
        r = r_q14;
        h = h_q15;
    }
    
    /// Set frequency (g) and resonance (Q), compute h.
    /// g_q14: from LUT. resonance_q15: Q factor (higher = more resonant)
    void SetGQ(int32_t g_q14, int32_t resonance_q15) {
        g = g_q14;
        // r = 1/Q in Q14. If resonance_q15 = 32767 (=1.0), r = 16384 (Q14 1.0)
        // For higher Q, r is smaller.
        if (resonance_q15 < 328) resonance_q15 = 328;  // Min Q ~0.01, prevent division by 0
        r = (int32_t)((16384LL * 32767) / resonance_q15);
        if (r > 32767) r = 32767;
        
        // h = 1/(1 + r*g + g²) in Q15
        // r*g is Q14*Q14 = Q28, shift to Q15 = >>13
        // g² is Q14*Q14 = Q28, shift to Q15 = >>13
        int32_t rg = (int32_t)(((int64_t)r * g) >> 13);  // Q15
        int32_t g2 = (int32_t)(((int64_t)g * g) >> 13);  // Q15
        int32_t denom = 32767 + rg + g2;
        if (denom < 1) denom = 1;
        h = (int32_t)((32767LL * 32767) / denom);
        if (h > 32767) h = 32767;
    }
    
    /// Set from pre-computed LUT index (0..256).
    /// Uses the approx SVF tables from resources_q15.
    void SetFromLUT(int32_t index) {
        if (index < 0) index = 0;
        if (index > 256) index = 256;
        g = lut_approx_svf_g_q14[index];
        r = lut_approx_svf_r_q14[index];
        h = lut_approx_svf_h_q15[index];
    }
    
    /// Copy coefficients from another filter
    void CopyFrom(const SvfQ15 &other) {
        g = other.g;
        r = other.r;
        h = other.h;
    }
    
    /// Process one sample, returning the selected filter mode output.
    /// Input x is Q15. Returns Q15.
    /// OPTIMIZED: Uses 32-bit multiplies where possible.
    /// g is Q14 (max ~16384), states are clamped to ±2M.
    /// g * state ≤ 16384 * 2000000 = 32,768,000,000 — EXCEEDS int32!
    /// So we keep int64 for g*state but optimize the shift pattern.
    /// h is Q15 (max 32767), hp is bounded → h*hp fits in 32-bit after shift.
    int32_t Process(int32_t x_in, FilterModeQ15 mode) {
        // Shift input up by 6 bits to preserve internal precision at low frequencies
        int32_t x = x_in << 6;
        
        // hp = (in - r*s1 - g*s1 - s2) * h
        int32_t rg_sum = r + g;
        int32_t rgs1 = (int32_t)(((int64_t)rg_sum * state1) >> 14);
        int32_t hp = (int32_t)(((int64_t)(x - rgs1 - state2) * h) >> 15);
        
        // Use int64 for state update intermediates to prevent overflow at high frequencies
        int64_t gbp_64 = ((int64_t)g * hp) >> 14;
        // Safety cap to prevent int32 overflow during state addition
        if (gbp_64 > 1000000000) gbp_64 = 1000000000;
        else if (gbp_64 < -1000000000) gbp_64 = -1000000000;
        int32_t gbp = (int32_t)gbp_64;

        int32_t bp = gbp + state1;
        state1 = gbp + bp;  // Double integration
        
        int64_t glp_64 = ((int64_t)g * bp) >> 14;
        if (glp_64 > 1000000000) glp_64 = 1000000000;
        else if (glp_64 < -1000000000) glp_64 = -1000000000;
        int32_t glp = (int32_t)glp_64;

        int32_t lp = glp + state2;
        state2 = glp + lp;
        
        // Anti-windup: clamp slightly below int32 max to prevent overflow
        if (state1 > 500000000) state1 = 500000000;
        else if (state1 < -500000000) state1 = -500000000;
        if (state2 > 500000000) state2 = 500000000;
        else if (state2 < -500000000) state2 = -500000000;
        
        // Branchless mode selection
        if (mode == FILT_BP) return bp >> 6;
        if (mode == FILT_LP) return lp >> 6;
        if (mode == FILT_BPN) return (int32_t)(((int64_t)bp * r) >> 14) >> 6;
        return hp >> 6;
    }
    
    /// Process one sample, returning both BP and LP simultaneously.
    /// Used by the resonator where we need both for output mixing.
    void Process2(int32_t x_in, int32_t &out_bp, int32_t &out_lp) {
        int32_t x = x_in << 6;
        int32_t rg_sum = r + g;
        int32_t rgs1 = (int32_t)(((int64_t)rg_sum * state1) >> 14);
        int32_t hp = (int32_t)(((int64_t)(x - rgs1 - state2) * h) >> 15);
        
        int64_t gbp_64 = ((int64_t)g * hp) >> 14;
        if (gbp_64 > 1000000000) gbp_64 = 1000000000;
        else if (gbp_64 < -1000000000) gbp_64 = -1000000000;
        int32_t gbp = (int32_t)gbp_64;
        
        int32_t bp = gbp + state1;
        state1 = gbp + bp;
        
        int64_t glp_64 = ((int64_t)g * bp) >> 14;
        if (glp_64 > 1000000000) glp_64 = 1000000000;
        else if (glp_64 < -1000000000) glp_64 = -1000000000;
        int32_t glp = (int32_t)glp_64;
        
        int32_t lp = glp + state2;
        state2 = glp + lp;
        
        if (state1 > 500000000) state1 = 500000000;
        else if (state1 < -500000000) state1 = -500000000;
        if (state2 > 500000000) state2 = 500000000;
        else if (state2 < -500000000) state2 = -500000000;
        
        out_bp = bp >> 6;
        out_lp = lp >> 6;
    }
    
    // ── Block processing ────────────────────────────────────────────────
    
    /// Process a block of samples, accumulating the result into the output.
    /// gain1/gain2 are Q15 mixing gains for two output buses.
    /// This is the pattern used by the resonator: each mode's SVF adds its
    /// contribution to the output with position-dependent L/R gains.
    void ProcessAdd(const int32_t* in, 
                                         int32_t* out1, int32_t* out2,
                                         int32_t size,
                                         int32_t gain1_q15, int32_t gain2_q15,
                                         FilterModeQ15 mode) {
        int32_t s1 = state1;
        int32_t s2 = state2;
        
        int32_t rg_sum = r + g;
        
        while (size--) {
            int32_t rgs1 = (int32_t)(((int64_t)rg_sum * s1) >> 14);
            int32_t diff = (*in << 6) - rgs1 - s2;
            int32_t hp = (int32_t)(((int64_t)diff * h) >> 15);
            
            int64_t gbp_64 = ((int64_t)g * hp) >> 14;
            if (gbp_64 > 1000000000) gbp_64 = 1000000000;
            else if (gbp_64 < -1000000000) gbp_64 = -1000000000;
            int32_t gbp = (int32_t)gbp_64;
            
            int32_t bp = gbp + s1;
            s1 = gbp + bp;
            
            int64_t glp_64 = ((int64_t)g * bp) >> 14;
            if (glp_64 > 1000000000) glp_64 = 1000000000;
            else if (glp_64 < -1000000000) glp_64 = -1000000000;
            int32_t glp = (int32_t)glp_64;
            
            int32_t lp = glp + s2;
            s2 = glp + lp;
            
            // Anti-windup
            if (s1 > 500000000) s1 = 500000000;
            else if (s1 < -500000000) s1 = -500000000;
            if (s2 > 500000000) s2 = 500000000;
            else if (s2 < -500000000) s2 = -500000000;
            
            int32_t value;
            switch (mode) {
                case FILT_LP:  value = lp >> 6; break;
                case FILT_BP:  value = bp >> 6; break;
                case FILT_BPN: value = (int32_t)(((int64_t)bp * r) >> 14) >> 6; break;
                case FILT_HP:  value = hp >> 6; break;
                default:       value = lp >> 6; break;
            }
            
            *out1 += (value * gain1_q15) >> 15;
            *out2 += (value * gain2_q15) >> 15;
            ++out1;
            ++out2;
            ++in;
        }
        
        state1 = s1;
        state2 = s2;
    }
    
    /// Simple block process: filter in-place.
    void ProcessBlock(int32_t* in_out, int32_t size, FilterModeQ15 mode) {
        int32_t s1 = state1;
        int32_t s2 = state2;
        
        int32_t rg_sum = r + g;
        
        while (size--) {
            int32_t rgs1 = (int32_t)(((int64_t)rg_sum * s1) >> 14);
            int32_t diff = (*in_out << 6) - rgs1 - s2;
            int32_t hp = (int32_t)(((int64_t)diff * h) >> 15);
            
            int64_t gbp_64 = ((int64_t)g * hp) >> 14;
            if (gbp_64 > 1000000000) gbp_64 = 1000000000;
            else if (gbp_64 < -1000000000) gbp_64 = -1000000000;
            int32_t gbp = (int32_t)gbp_64;
            
            int32_t bp = gbp + s1;
            s1 = gbp + bp;
            
            int64_t glp_64 = ((int64_t)g * bp) >> 14;
            if (glp_64 > 1000000000) glp_64 = 1000000000;
            else if (glp_64 < -1000000000) glp_64 = -1000000000;
            int32_t glp = (int32_t)glp_64;
            
            int32_t lp = glp + s2;
            s2 = glp + lp;
            
            // Anti-windup
            if (s1 > 500000000) s1 = 500000000;
            else if (s1 < -500000000) s1 = -500000000;
            if (s2 > 500000000) s2 = 500000000;
            else if (s2 < -500000000) s2 = -500000000;
            
            switch (mode) {
                case FILT_LP:  *in_out = lp >> 6; break;
                case FILT_BP:  *in_out = bp >> 6; break;
                case FILT_BPN: *in_out = (int32_t)(((int64_t)bp * r) >> 14) >> 6; break;
                case FILT_HP:  *in_out = hp >> 6; break;
                default:       *in_out = lp >> 6; break;
            }
            ++in_out;
        }
        
        state1 = s1;
        state2 = s2;
    }
};

// ── One-Pole Filter (Q15) ───────────────────────────────────────────────────
// Simple first-order low-pass/high-pass for parameter smoothing and
// damping filters.

struct OnePoleFilterQ15 {
    int32_t g;      // Frequency coefficient (Q14)
    int32_t gi;     // 1/(1+g) normalization (Q15)
    int32_t state;  // Filter state (Q15)
    
    void Init() {
        g = 164;    // ~0.01 in Q14
        gi = 32567;
        state = 0;
    }
    
    void Reset() { state = 0; }
    
    /// Set frequency from a Q14 g coefficient
    void SetG(int32_t g_q14) {
        g = g_q14;
        // gi = 1/(1+g) in Q15
        int32_t denom = 16384 + g;  // Q14
        gi = (int32_t)((16384LL * 32767) / denom);
        if (gi > 32767) gi = 32767;
    }
    
    /// Low-pass: returns LP output
    int32_t ProcessLP(int32_t x) {
        int32_t lp = (int32_t)(((int64_t)(((int64_t)g * x) >> 14) + state) * gi >> 15);
        state = (int32_t)(((int64_t)g * (x - lp)) >> 14) + lp;
        return lp;
    }
    
    /// High-pass: returns HP output
    int32_t ProcessHP(int32_t x) {
        int32_t lp = ProcessLP(x);
        return x - lp;
    }
};

#endif  // SVF_Q15_H_
