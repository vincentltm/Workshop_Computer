// =============================================================================
// resonator_q15.cpp — Fixed-point Modal Resonator for Elements port
//
// Port of elements::Resonator to Q15 integer arithmetic.
// Based on original code by Émilie Gillet (MIT license).
// =============================================================================

#include "resonator_q15.h"

// ── Fast tan(π·f) approximation ────────────────────────────────────────────
// 5th-order polynomial: g = f·(π + a·f² + b·f⁴)
// Accurate to <0.5% for f < 0.35 (8.4kHz at 24kHz SR), good enough for SVFs.
// Input: f_q16 = normalized frequency (0..32768 = 0..0.5). Output: Q14.
static inline int32_t __attribute__((section(".time_critical.FastTan"))) FastTanQ14(int32_t f_q16) {
    if (f_q16 > 32112) f_q16 = 32112; // cap at ~0.49
    int64_t f2 = ((int64_t)f_q16 * f_q16) >> 16;
    int64_t f4 = (f2 * f2) >> 16;
    
    // Coefficients from Elements' FREQUENCY_FAST approximation
    // a = 3.260e-01 * π³ ≈ 10.108,  b = 1.823e-01 * π⁵ ≈ 55.787
    int32_t a_q16 = 662438;  // 10.108 * 65536
    int32_t b_q16 = 3656057; // 55.787 * 65536
    
    int32_t term1 = (int32_t)(((int64_t)a_q16 * (int32_t)f2) >> 16);
    int32_t term2 = (int32_t)(((int64_t)b_q16 * (int32_t)f4) >> 16);
    int32_t sum = 205887 + term1 + term2; // π * 65536 = 205887
    int32_t g_q16 = (int32_t)(((int64_t)f_q16 * sum) >> 16);
    return g_q16 >> 2; // Q16 → Q14
}

void ResonatorQ15::Init() {
    for (size_t i = 0; i < kMaxModesQ15; ++i) {
        f_[i].Init();
    }
    for (size_t i = 0; i < kMaxBowedModesQ15; ++i) {
        f_bow_[i].Init();
        d_bow_[i].Init();
    }
    
    frequency_q15 = 300;     // ~10Hz / 24kHz = 0.0004 → Q15 ≈ 13
    geometry_q15 = 8192;     // 0.25
    brightness_q15 = 16384;  // 0.5
    damping_q15 = 9830;      // 0.3
    position_q15 = 32734;    // 0.999
    
    previous_position_q15 = position_q15;
    previous_geometry_q15 = geometry_q15;
    // ~0.5Hz / 24kHz = 2.08e-5 → Q15 ≈ 1
    modulation_frequency_q15 = 1;
    modulation_offset_q15 = 3276; // 0.1
    lfo_phase_q15 = 0;
    
    resolution = kMaxModesQ15;
    clock_divider = 0;
    num_modes_cached = 0;
    bow_signal_q15 = 0;
    structure = STRUC_MODAL;
}

size_t __attribute__((section(".time_critical.ComputeFilters"))) ResonatorQ15::ComputeFilters() {
    // stiffness = Interpolate(lut_stiffness, geometry_, 256.0f)
    int32_t stiffness_idx = (geometry_q15 * 256) >> 7;
    int32_t stiffness = (structure == STRUC_WIND) ? 0 : InterpolateQ15(lut_stiffness_q15, stiffness_idx);
    
    int32_t harmonic = frequency_q15;
    int32_t stretch_factor = 32767; // 1.0 in Q15
    
    // q = 500.0f * Interpolate(lut_4_decades, damping_ * 0.8f, 256.0f)
    int32_t damping_adj = mul_q15(damping_q15, 26214); // * 0.8
    int32_t q_idx = (damping_adj * 256) >> 7;
    int32_t q_raw_q16 = InterpolateQ16(lut_4_decades_q16, q_idx);
    int64_t q_running = (int64_t)q_raw_q16 * 500;
    
    // Wind instruments are heavily damped air columns, they shouldn't ring like metal plates.
    if (structure == STRUC_WIND) {
        // q_running is heavily scaled. Max is typically ~100M+ for metal.
        // Divide by 8 and cap to ~20M to keep Q around 30-50.
        q_running = (q_running >> 3) + 2000000;
        if (q_running > 25000000) q_running = 25000000;
    }
    
    // brightness attenuation (ba)
    int32_t ba = 32767 - geometry_q15;
    int64_t ba2 = ((int64_t)ba * ba) >> 15;
    int32_t ba4 = (int32_t)((ba2 * ba2) >> 15);
    int32_t bright_mod = 32767 - (int32_t)(((int64_t)6553 * ba4) >> 15);
    int32_t brightness = mul_q15(brightness_q15, (bright_mod < 0) ? 0 : bright_mod);
    
    // q_loss = brightness * (2.0 - brightness) * 0.85 + 0.15
    int32_t two_minus_b = 65534 - brightness;
    int32_t q_loss_base = mul_q15(brightness, two_minus_b);
    int32_t q_loss = mul_q15(q_loss_base, 27852) + 4915; // *0.85 + 0.15
    
    // q_loss_damping_rate = geometry_ * (2.0 - geometry_) * 0.1
    int32_t two_minus_g = 65534 - geometry_q15;
    int32_t q_loss_damping_rate = mul_q15(mul_q15(geometry_q15, two_minus_g), 3277);
    
    size_t num_modes = 0;
    
    for (size_t i = 0; i < kMaxModesQ15; ++i) {
        int32_t partial_frequency = (int32_t)(((int64_t)harmonic * stretch_factor) >> 15);
        
        if (partial_frequency >= 16056) {
            partial_frequency = 16056;
        } else {
            num_modes = i + 1;
        }
        
        int32_t g_q14 = FastTanQ14(partial_frequency << 1);
        int64_t res_term = ((int64_t)partial_frequency * q_running) >> 16;
        int32_t resonance_q15 = 32767 + (int32_t)res_term;
        if (resonance_q15 < 328) resonance_q15 = 328;
        if (resonance_q15 > 32767000) resonance_q15 = 32767000;
        
        f_[i].SetGQ(g_q14, resonance_q15);
        
        if (i < kMaxBowedModesQ15) {
            int32_t period_q15 = (partial_frequency > 0) ? (1073741824 / partial_frequency) : (511 << 15);
            while (period_q15 >= (512 << 15) && period_q15 > (1 << 15)) {
                period_q15 >>= 1;
            }
            d_bow_[i].set_delay(period_q15);
            int32_t bow_res = 32767 + partial_frequency * 1500;
            f_bow_[i].SetGQ(g_q14, (bow_res < 328) ? 328 : bow_res);
        }
        
        stretch_factor += stiffness;
        if (stiffness < 0) {
            stiffness = mul_q15(stiffness, 30474); // * 0.93
        } else {
            stiffness = mul_q15(stiffness, 32112); // * 0.98
        }
        
        // Update frequency-dependent damping
        q_loss += mul_q15(q_loss_damping_rate, 32767 - q_loss);
        q_running = (q_running * q_loss) >> 15;
        
        harmonic += frequency_q15;
    }
    return num_modes;
}

void __attribute__((section(".time_critical.resonator"))) ResonatorQ15::Process1(int32_t bow_strength, int32_t in,
                            int32_t &center, int32_t &sides) {
    // Throttle filter coefficient updates
    if ((clock_divider++ % 24) == 0) {
        num_modes_cached = ComputeFilters();
    }
    size_t num_modes = num_modes_cached;
    if (num_modes == 0) num_modes = 1;
    
    size_t num_banded_wg = (kMaxBowedModesQ15 < num_modes)
        ? kMaxBowedModesQ15 : num_modes;
    
    // Smooth position and geometry
    previous_position_q15 += (position_q15 - previous_position_q15) >> 6;
    previous_geometry_q15 += (geometry_q15 - previous_geometry_q15) >> 6;
    
    // 0.5Hz LFO for stereo modulation
    // Increment for 0.5Hz at 24kHz = 32768 / (24000 * 2) ≈ 0.68.
    // lfo_phase_q15 is Q15.
    lfo_phase_q15 = (lfo_phase_q15 + 1) & 0x7FFF;
    int32_t triangle = (lfo_phase_q15 > 16384) ? (32767 - lfo_phase_q15) : lfo_phase_q15;
    int32_t lfo = triangle << 1; // 0..32767
    
    // Position-dependent amplitude oscillators
    CosineOscQ15 amplitudes;
    CosineOscQ15 aux_amplitudes;
    
    amplitudes.Init(previous_position_q15);
    // Side channel pickup is offset + LFO
    int32_t side_pos = (modulation_offset_q15 + (lfo >> 1)) & 0x7FFF;
    aux_amplitudes.Init(side_pos);

    int32_t input = in >> 3;
    int32_t sum_center = 0;
    int32_t sum_side = 0;
    
    int32_t fade_even = 32767;
    int32_t fade_odd = 32767;
    if (structure == STRUC_WIND) {
        // Pre-calculate fades outside the loop
        fade_even = (previous_geometry_q15 - 16384) << 1;
        if (fade_even < 0) fade_even = 0;
        else if (fade_even > 32767) fade_even = 32767;
        
        fade_odd = previous_geometry_q15 << 1;
        if (fade_odd > 32767) fade_odd = 32767;
    }

    amplitudes.Start();
    aux_amplitudes.Start();
    for (size_t i = 0; i < num_modes; ++i) {
        int32_t s = f_[i].Process(input, FILT_BP);
        int32_t amp_c, dummy_s;
        int32_t dummy_c, amp_side;
        
        amplitudes.NextQuadrature(amp_c, dummy_s);
        aux_amplitudes.NextQuadrature(amp_side, dummy_c);
        
        // Shift cosine oscillator output to [0, 1] range to match original (temp + 0.5f)
        amp_c = (amp_c + 32768) >> 1;
        amp_side = (amp_side + 32768) >> 1;
        
        if (structure == STRUC_WIND) {
            if (i > 0) {
                int32_t f = (i & 1) ? fade_even : fade_odd;
                amp_c = mul_q15(amp_c, f);
                amp_side = mul_q15(amp_side, f);
            }
        }

        sum_center += mul_q15(s, amp_c);
        sum_side   += mul_q15(s, amp_side);
    }
    
    // Bowed modes & Bow Table
    // Skip if bow_strength is low to save CPU (avoid division) and prevent ringing
    if (bow_strength > 400) {
        int32_t bow_signal = 0;
        int32_t input_bow = input + bow_signal_q15;
        
        amplitudes.Start();
        for (size_t i = 0; i < num_banded_wg; ++i) {
            int32_t s = mul_q15(d_bow_[i].Read(), 32440); // 0.99
            bow_signal += s;
            s = f_bow_[i].Process(input_bow + s, FILT_BPN);
            d_bow_[i].Write(s);
            
            int32_t amp_c, dummy_s;
            amplitudes.NextQuadrature(amp_c, dummy_s);
            
            // Shift cosine oscillator output to [0, 1] range
            amp_c = (amp_c + 32768) >> 1;
            sum_center += mul_q15(s, amp_c) << 2; // Boosted gain to <<2 (gain of 4)
        }
        
        // Bow Table (Expensive Division!)
        int32_t velocity = bow_strength;
        int32_t x = mul_q15(4259, velocity) - bow_signal;
        int32_t abs_six_x = abs_q15(x * 6);
        int32_t denom = abs_six_x + 24576;
        
        int64_t d2 = ((int64_t)denom * denom) >> 15;
        int32_t d4 = (int32_t)((d2 * d2) >> 15);
        
        // Use a 32-bit division instead of 64-bit if possible for speed
        int32_t bow_gain = (d4 < 1) ? 8028 : (int32_t)(268435456L / d4);
        if (bow_gain < 82) bow_gain = 82;
        if (bow_gain > 8028) bow_gain = 8028;
        
        bow_signal_q15 = mul_q15(x, bow_gain);
    } else {
        bow_signal_q15 = 0;
    }
    
    // Saturation checks for final output
    if (sum_center > 32767) sum_center = 32767;
    if (sum_center < -32768) sum_center = -32768;
    if (sum_side > 32767) sum_side = 32767;
    if (sum_side < -32768) sum_side = -32768;

    center = sum_center;
    sides = sum_side - sum_center;
}
