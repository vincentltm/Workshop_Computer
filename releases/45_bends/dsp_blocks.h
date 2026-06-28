#ifndef DSP_BLOCKS_H
#define DSP_BLOCKS_H

// ============================================================================
// dsp_blocks.h  —  45_bends Multi-FX DSP Blocks
// ============================================================================
// Six sequential stereo effect stages, each a self-contained fixed-point C++
// struct. All blocks run on Core 1 at 48 kHz inside ProcessSample().
//
// Signal convention: int16_t Q15, range [-32768, 32767] ≡ [-1.0, 1.0).
// All internal state that accumulates energy is promoted to int32_t to keep
// sufficient headroom without the FPU the RP2040 lacks.
//
// Block order in the chain:
//   1. ChorusBlock           — 90s BBD-style stereo chorus / vibrato
//   2. CodecDemolisherBlock  — lo-fi decimator, MP3-ring artifacts, glitch gate
//   3. MultiTapDelayBlock    — stereo tape echo with 3 rhythmic taps
//   4. GlitcherBlock         — phrase-loop granular stutter sampler
//   5. FilterBlock           — morphable Chamberlin SVF (LP/BP/HP)
//   6. ReverbBlock           — Schroeder plate (4 comb + 2 allpass per side)
// ============================================================================

#include "fixed_math.h"
#include <string.h>   // memset

// ============================================================================
// Fixed-Point DC Blocker (High-Pass Filter with cutoff at ~10 Hz @ 48 kHz)
// Prevents feedback DC accumulation in feedback loops.
// ============================================================================
struct DCBlocker {
    int32_t x1 = 0;
    int32_t y1 = 0;

    void init() {
        x1 = 0;
        y1 = 0;
    }

    int16_t process(int16_t in) {
        int32_t x = in;
        // y[n] = x[n] - x[n-1] + 0.995 * y[n-1]
        // 32604 / 32768 ≈ 0.995
        int32_t y = x - x1 + ((y1 * 32604) >> 15);
        x1 = x;
        y1 = clamp_i32(y, -1000000, 1000000);
        return saturate_q15(y);
    }
};

// ============================================================================
// 1.  CHORUS BLOCK
// ============================================================================
// Classic bucket-brigade delay (BBD) chorus emulation.
// Overhauled for classic vintage dual-phase triangle LFO modulation (180° out of phase)
// and a high-pass filter in the wet path to keep the low end clean.
//
// Parameters (all int32_t in range [0, 32767]):
//   mainMix     — Wet / dry blend.  0 = fully dry, 32767 = fully wet.
//   rate        — LFO speed.       Slow (0) → Fast (32767). CV1 warps it.
//   depthFeedback — Dual-function:
//                   [0, 16383] → Depth scales from 0 to max. Feedback = 0.
//                   [16384, 32767] → Depth stays at max; Feedback rises to 100%.
// ============================================================================
struct ChorusBlock {
    // Stereo BBD delay lines – 1024 samples each (~21 ms @ 48 kHz)
    int16_t  delayL[1024];
    int16_t  delayR[1024];
    uint16_t write_ptr  = 0;

    // LFO phase accumulator – wraps naturally
    uint16_t lfo_phase  = 0;

    // 1-pole LPF state for BBD emulation
    int32_t lp_stateL = 0;
    int32_t lp_stateR = 0;

    // Vintage wet-path high-pass filters to cut sub-bass mud (cutoff ~110 Hz)
    int32_t hp_x1L = 0, hp_y1L = 0;
    int32_t hp_x1R = 0, hp_y1R = 0;

    void init() {
        memset(delayL, 0, sizeof(delayL));
        memset(delayR, 0, sizeof(delayR));
        write_ptr = 0;
        lfo_phase = 0;
        lp_stateL = 0;
        lp_stateR = 0;
        hp_x1L = hp_y1L = hp_x1R = hp_y1R = 0;
    }

    void process(int16_t inL, int16_t &outL, int16_t inR, int16_t &outR,
                 int32_t mainMix, int32_t rate, int32_t depthFeedback,
                 int32_t cv1Warp)
    {
        if (mainMix < 50) {
            outL = inL;
            outR = inR;
            delayL[write_ptr] = inL;
            delayR[write_ptr] = inR;
            write_ptr = (write_ptr + 1) & 0x3FF;
            
            int32_t eff_rate = rate + (cv1Warp * 4);
            eff_rate = clamp_i32(eff_rate, 0, 32767);
            uint16_t phase_inc = (uint16_t)(1 + (eff_rate >> 11));
            lfo_phase += phase_inc;
            return;
        }

        // LFO rate modulation (CV1 shifts pitch by modulating rate)
        int32_t eff_rate = rate + (cv1Warp * 4);
        eff_rate = clamp_i32(eff_rate, 0, 32767);

        // Phase increment
        uint16_t phase_inc = (uint16_t)(1 + (eff_rate >> 11));
        lfo_phase += phase_inc;

        // Decode depth / feedback from unified knob
        int16_t depth    = 0;
        int16_t feedback = 0;
        if (depthFeedback < 16384) {
            depth    = (int16_t)(depthFeedback * 2); // 0 → 32767
            feedback = 0;
        } else {
            depth    = 32767;
            feedback = (int16_t)((depthFeedback - 16384) * 2); // 0 → 32767
        }

        // Classic dual-phase triangle LFO (L = 0°, R = 180° for deep stereo spread)
        auto get_tri = [](uint16_t phase) -> int16_t {
            int32_t tmp = (phase < 32768) ? ((phase << 1) - 32768) : (32767 - ((phase - 32768) << 1));
            return (int16_t)tmp;
        };
        int16_t lfoL = get_tri(lfo_phase);
        int16_t lfoR = get_tri((uint16_t)(lfo_phase + 32768u));

        // Base delay: 350 samples (~7.3 ms). Modulation range: ±200 samples (~4.2 ms).
        int32_t depth_samples = (int32_t)depth * 200 >> 15;
        int32_t delay_L_q16 = (350 << 16) + (((int32_t)lfoL * depth_samples) >> (15 - 16 + 1));
        int32_t delay_R_q16 = (350 << 16) + (((int32_t)lfoR * depth_samples) >> (15 - 16 + 1));
        
        if (delay_L_q16 < (1 << 16)) delay_L_q16 = (1 << 16);
        if (delay_R_q16 < (1 << 16)) delay_R_q16 = (1 << 16);

        delayL[write_ptr] = inL;
        delayR[write_ptr] = inR;

        auto read_frac = [](const int16_t *buf, int32_t wp_q16, int32_t delay_q16) -> int16_t {
            int32_t  rp_q16 = wp_q16 - delay_q16;
            int32_t  idx    = (rp_q16 >> 16) & 0x3FF;
            int32_t  nxt    = (idx + 1)       & 0x3FF;
            uint16_t frac   = (uint16_t)(rp_q16 & 0xFFFF);
            int16_t  y0     = buf[idx];
            int16_t  y1     = buf[nxt];
            return (int16_t)(y0 + (((int64_t)(y1 - y0) * (int64_t)frac) >> 16));
        };

        int32_t wp_q16 = (int32_t)write_ptr << 16;
        int16_t wetL = read_frac(delayL, wp_q16, delay_L_q16);
        int16_t wetR = read_frac(delayR, wp_q16, delay_R_q16);

        // BBD low-pass filter
        int32_t damp = 32767 - ((mainMix * 24000) >> 15);
        lp_stateL += ((int32_t)wetL - lp_stateL) * damp >> 15;
        lp_stateR += ((int32_t)wetR - lp_stateR) * damp >> 15;
        wetL = (int16_t)lp_stateL;
        wetR = (int16_t)lp_stateR;

        // Vintage HPF on wet path (cutoff ~110 Hz: y = x - x1 + 0.976 * y1)
        int32_t yL = (int32_t)wetL - hp_x1L + ((hp_y1L * 32000) >> 15);
        hp_x1L = (int32_t)wetL;
        hp_y1L = clamp_i32(yL, -1000000, 1000000);
        wetL = saturate_q15(yL);

        int32_t yR = (int32_t)wetR - hp_x1R + ((hp_y1R * 32000) >> 15);
        hp_x1R = (int32_t)wetR;
        hp_y1R = clamp_i32(yR, -1000000, 1000000);
        wetR = saturate_q15(yR);

        // Feedback loop
        if (feedback > 0) {
            int32_t fbL = (int32_t)inL + (((int32_t)wetL * feedback) >> 15);
            int32_t fbR = (int32_t)inR + (((int32_t)wetR * feedback) >> 15);
            delayL[write_ptr] = soft_limit_q15(fbL);
            delayR[write_ptr] = soft_limit_q15(fbR);
        }

        write_ptr = (write_ptr + 1) & 0x3FF;

        outL = lerp_q15(inL, wetL, (int16_t)mainMix);
        outR = lerp_q15(inR, wetR, (int16_t)mainMix);
    }
};

// ============================================================================
// 2.  CODEC DEMOLISHER BLOCK
// ============================================================================
// Simulates the artefacts of digital audio degradation: sample-rate reduction,
// MP3-style resonant ringing at codec crossover frequencies, packet dropout
// (CD-skip / lossy-stream style), and raw bit-level XOR corruption.
//
// Internal bandpass filter: second-order state variable, promoted to int32
// states for numerical stability at high Q.
//
// Parameters:
//   mainMix      — Wet / dry blend.
//   downsample   — Decimation factor.  0 = no change; 32767 = ~64× hold.
//   ringingXor   — [0,16383] → MP3 ring amount. [16384,32767] → + bit corruption
//                  and frame dropout probability.
//   cv2Corruption — CV2: additional corruption/destruction amount.
//   pulse2Scramble — Pulse 2 jack: instant burst of frame-drop + XOR chaos.
// ============================================================================

// Second-order bandpass: Chamberlin SVF in bandpass mode.
// States promoted to int32_t for Q stability at high resonance.
// Uses the "corrected" Chamberlin update with a single integration per step.
struct CodecDemolisherBlock {
    // Telecom SVF states
    int32_t codec_v1L = 0, codec_v2L = 0;
    int32_t codec_v1R = 0, codec_v2R = 0;

    // Broken Transmission history buffers (256 samples per channel)
    int16_t trans_historyL[256];
    int16_t trans_historyR[256];
    uint8_t trans_wr = 0;
    uint8_t trans_drop_rd = 0;
    uint16_t trans_frame_ctr = 0;
    uint16_t trans_frame_size = 320;
    bool trans_dropped = false;
    uint16_t trans_loop_size = 128;

    // Sample-hold (decimator) state
    int16_t  decL = 0, decR = 0;
    uint32_t dec_ctr = 0;
    uint32_t current_dec_step = 1;

    // Compressor envelope state
    int32_t env = 0;

    // LPF states to filter out harsh high frequency aliasing
    int32_t lp_newL = 0, lp_newR = 0;
    int32_t dec_lpL = 0, dec_lpR = 0;
    uint32_t vibe_lfo = 0;
    uint32_t carrier_phase = 0;
    uint16_t sputter_timer = 0;
    bool sputter_active = false;

    void init() {
        codec_v1L = codec_v2L = codec_v1R = codec_v2R = 0;
        for (int i = 0; i < 256; i++) {
            trans_historyL[i] = trans_historyR[i] = 0;
        }
        trans_wr = 0;
        trans_drop_rd = 0;
        trans_frame_ctr = 0;
        trans_frame_size = 320;
        trans_dropped = false;
        trans_loop_size = 128;
        
        decL = decR = 0;
        dec_ctr = 0;
        current_dec_step = 1;
        env = 0;
        lp_newL = lp_newR = 0;
        dec_lpL = dec_lpR = 0;
        vibe_lfo = 0;
        carrier_phase = 0;
        sputter_timer = 0;
        sputter_active = false;
    }

    // Helper for polynomial tape saturation
    inline int16_t tape_saturate(int16_t in) {
        int32_t x = in;
        int32_t x3 = (((x * x) >> 15) * x) >> 15;
        int32_t y = x - ((x3 * 5461) >> 15);
        return saturate_q15(y);
    }

    // Real-time variable-bitrate G.711 mu-law logarithmic compander simulation
    inline int16_t compress_expand_mulaw_variable(int16_t sample, int32_t insanity) {
        int32_t x = sample;
        int32_t sign = (x < 0) ? -1 : 1;
        if (x < 0) x = -x;
        
        uint16_t abs_val = x;
        uint8_t exponent = 0;
        if (abs_val >= 256) {
            exponent = (31 - __builtin_clz(abs_val)) - 7;
        }
        
        uint8_t mantissa = 0;
        if (exponent > 0) {
            mantissa = (abs_val >> (exponent + 1)) & 0xF;
        } else {
            mantissa = (abs_val >> 2) & 0xF;
        }
        
        // Mask/quantize the mantissa based on insanity (0 to 32767)
        // At insanity = 0, keep all 4 bits.
        // At insanity = 32767, mask out all 4 bits (mantissa = 0).
        int32_t mant_bits = 4 - ((insanity * 4) >> 15);
        if (mant_bits < 0) mant_bits = 0;
        if (mant_bits > 4) mant_bits = 4;
        uint8_t mask = 0xF << (4 - mant_bits);
        mantissa &= mask;
        
        int32_t reconstructed = 0;
        if (exponent > 0) {
            reconstructed = ((mantissa << 1) + 33) << (exponent + 1);
        } else {
            reconstructed = (mantissa << 2) + 2;
        }
        if (reconstructed > 32767) reconstructed = 32767;
        return (int16_t)(sign * reconstructed);
    }

    void process(int16_t inL, int16_t &outL, int16_t inR, int16_t &outR,
                 int32_t strength, int32_t downsample, int32_t ringingXor,
                 int32_t cv2Corruption, uint32_t &rand_seed, int32_t globalNoiseScale = 16384)
    {
        if (strength < 50) {
            outL = inL;
            outR = inR;
            decL = inL;
            decR = inR;
            dec_ctr = 0;
            env = 0;
            vibe_lfo = 0;
            trans_wr = 0;
            trans_dropped = false;
            trans_frame_ctr = 0;
            sputter_timer = 0;
            sputter_active = false;
            codec_v1L = codec_v2L = codec_v1R = codec_v2R = 0;
            return;
        }

        // ── 1. Temporal Breathing / Vibe LFO (breathes at ~1.4 Hz) ─────────────
        vibe_lfo += 2;
        int16_t vibe_sine = lookup_sine(vibe_lfo); // [-32768, 32767]

        // ── 2. Engine Mix Scaling by Strength & Y Knob ────────────────────────
        int32_t Y = ringingXor + (cv2Corruption * 8);
        Y = clamp_i32(Y, 0, 32767);

        int32_t fuzz_level = 0;
        int32_t bad_conn_level = 0;
        int32_t scramble_level = 0;

        if (Y < 19660) { // 0% to 60%
            fuzz_level = (Y * 32767) / 19660;
            bad_conn_level = 0;
            scramble_level = 0;
        } else if (Y < 27852) { // 60% to 85%
            int32_t range = 27852 - 19660;
            int32_t diff = Y - 19660;
            fuzz_level = 32767 - ((diff * 32767) / range);
            bad_conn_level = (diff * 32767) / range;
            scramble_level = 0;
        } else { // 85% to 100% (circuit-bent scramble)
            int32_t range = 32767 - 27852;
            int32_t diff = Y - 27852;
            fuzz_level = 0;
            bad_conn_level = 32767 - ((diff * 16383) / range); // drops to 16384 at the end so dropouts remain active
            scramble_level = (diff * 32767) / range;
        }

        // Scale engine levels by Main knob (strength)
        fuzz_level = (fuzz_level * strength) >> 15;
        bad_conn_level = (bad_conn_level * strength) >> 15;
        scramble_level = (scramble_level * strength) >> 15;

        // Apply Global Noise Scale modifier (16384 is 1.0x)
        fuzz_level = (fuzz_level * globalNoiseScale) >> 14;
        bad_conn_level = (bad_conn_level * globalNoiseScale) >> 14;
        scramble_level = (scramble_level * globalNoiseScale) >> 14;

        if (fuzz_level > 32767) fuzz_level = 32767;
        if (bad_conn_level > 32767) bad_conn_level = 32767;
        if (scramble_level > 32767) scramble_level = 32767;

        // Apply vibe modulation (dynamic breathing)
        int32_t mod_factor = 32768 + (vibe_sine >> 3); // 0.9x to 1.1x scaling
        fuzz_level = clamp_i32(((int64_t)fuzz_level * mod_factor) >> 15, 0, 32767);
        bad_conn_level = clamp_i32(((int64_t)bad_conn_level * mod_factor) >> 15, 0, 32767);
        scramble_level = clamp_i32(((int64_t)scramble_level * mod_factor) >> 15, 0, 32767);

        int16_t wetL = inL;
        int16_t wetR = inR;
        int16_t wet_codec_L = wetL, wet_codec_R = wetR;
        int16_t wet_trans_L = wetL, wet_trans_R = wetR;
        int16_t wet_new_L = wetL, wet_new_R = wetR;

        // ── Engine 1: Warm Fuzz (Logarithmic Mu-law + Bitcrusher + Saturation + Bandpass) ────
        if (fuzz_level > 0) {
            int16_t compL = compress_expand_mulaw_variable(wetL, fuzz_level);
            int16_t compR = compress_expand_mulaw_variable(wetR, fuzz_level);

            // Continuous fractional bitcrusher for fuzz
            int32_t shift_q15 = (fuzz_level * 10);
            int32_t int_shift = shift_q15 >> 15;
            int32_t frac_shift = shift_q15 & 0x7FFF;
            if (int_shift > 0 || frac_shift > 0) {
                int16_t q1L = (compL >> int_shift) << int_shift;
                int16_t q2L = (compL >> (int_shift + 1)) << (int_shift + 1);
                compL = lerp_q15(q1L, q2L, frac_shift);

                int16_t q1R = (compR >> int_shift) << int_shift;
                int16_t q2R = (compR >> (int_shift + 1)) << (int_shift + 1);
                compR = lerp_q15(q1R, q2R, frac_shift);
            }

            // Warm fuzz saturation
            compL = tape_saturate(((int32_t)compL * (32768 + fuzz_level)) >> 15);
            compR = tape_saturate(((int32_t)compR * (32768 + fuzz_level)) >> 15);

            // Telecom bandpass filter around ~800Hz
            int32_t rg_c = 2200; 
            int32_t rr_c = 28000 - ((fuzz_level * 22000) >> 15);

            int32_t hp_cL = (int32_t)compL - ((rr_c * codec_v1L) >> 15) - codec_v2L;
            codec_v1L += (rg_c * hp_cL) >> 15;
            codec_v1L = soft_limit_q15(codec_v1L);
            int32_t bp_cL = codec_v1L;
            codec_v2L = codec_v2L + ((rg_c * codec_v1L) >> 15);
            codec_v2L = soft_limit_q15(codec_v2L);

            int32_t hp_cR = (int32_t)compR - ((rr_c * codec_v1R) >> 15) - codec_v2R;
            codec_v1R += (rg_c * hp_cR) >> 15;
            codec_v1R = soft_limit_q15(codec_v1R);
            int32_t bp_cR = codec_v1R;
            codec_v2R = codec_v2R + ((rg_c * codec_v1R) >> 15);
            codec_v2R = soft_limit_q15(codec_v2R);

            // Mix full-range saturated fuzz with the bandpass filter output (64% / 36% blend) to keep it bright and full
            wet_codec_L = lerp_q15((int16_t)compL, saturate_q15(bp_cL), 12000);
            wet_codec_R = lerp_q15((int16_t)compR, saturate_q15(bp_cR), 12000);
        }

        // ── Engine 2: Bad Connection (Packet Drops & Stutter Repetition) ──────
        if (bad_conn_level > 0) {
            if (!trans_dropped) {
                trans_historyL[trans_wr] = wetL;
                trans_historyR[trans_wr] = wetR;
            }

            if (trans_frame_ctr >= trans_frame_size) {
                trans_frame_ctr = 0;
                trans_frame_size = 240 + (fast_rand(rand_seed) % 720);

                int32_t drop_thresh = 300 + ((bad_conn_level * 14000) >> 15);
                uint32_t roll = fast_rand(rand_seed) & 0x7FFF;
                trans_dropped = ((int32_t)roll < drop_thresh);

                if (trans_dropped) {
                    trans_loop_size = 64 + ((bad_conn_level * 192) >> 15);
                    trans_drop_rd = (trans_wr - trans_loop_size) & 0xFF;
                }
            }
            trans_frame_ctr++;

            int16_t out_tL = wetL;
            int16_t out_tR = wetR;

            if (trans_dropped) {
                out_tL = trans_historyL[trans_drop_rd];
                out_tR = trans_historyR[trans_drop_rd];
                trans_drop_rd = (trans_drop_rd + 1) & 0xFF;
            } else {
                trans_wr = (trans_wr + 1) & 0xFF;
            }

            int32_t scramble_prob = (bad_conn_level * 4000) >> 15; 
            if ((int32_t)(fast_rand(rand_seed) & 0x7FFF) < scramble_prob) {
                uint32_t limit = 1 + (bad_conn_level >> 11);
                uint16_t mask = (uint16_t)(((uint64_t)fast_rand(rand_seed) * limit) >> 32);
                out_tL ^= mask;
                out_tR ^= mask;
            }

            wet_trans_L = out_tL;
            wet_trans_R = out_tR;
        }

        // ── Engine 3: Circuit-Bent Scramble ──
        if (scramble_level > 0) {
            carrier_phase += 150 + (scramble_level >> 5);

            int16_t carrier = (carrier_phase & 0x8000) ? 0xFFFF : 0x0000;

            int16_t modL = wetL ^ carrier;
            int16_t modR = wetR ^ carrier;

            // Continuous fractional bitcrusher for scramble
            int32_t shift_q15 = (scramble_level * 12);
            int32_t int_shift = shift_q15 >> 15;
            int32_t frac_shift = shift_q15 & 0x7FFF;

            int16_t q1L = (modL >> int_shift) << int_shift;
            int16_t q2L = (modL >> (int_shift + 1)) << (int_shift + 1);
            modL = lerp_q15(q1L, q2L, frac_shift);

            int16_t q1R = (modR >> int_shift) << int_shift;
            int16_t q2R = (modR >> (int_shift + 1)) << (int_shift + 1);
            modR = lerp_q15(q1R, q2R, frac_shift);

            int32_t foldL = modL * 2;
            int32_t foldR = modR * 2;

            if (foldL > 32767) foldL = 32767 - (foldL - 32767);
            if (foldL < -32768) foldL = -32768 - (foldL + 32768);
            if (foldR > 32767) foldR = 32767 - (foldR - 32767);
            if (foldR < -32768) foldR = -32768 - (foldR + 32768);

            int16_t foldL_sat = saturate_q15(foldL);
            int16_t foldR_sat = saturate_q15(foldR);

            // Open up scramble LPF cutoff (minimum coefficient 10000 instead of 2000) to keep it bright
            int32_t scram_coef = 24000 - ((scramble_level * 14000) >> 15);
            lp_newL += (((int32_t)foldL_sat - lp_newL) * scram_coef) >> 15;
            lp_newR += (((int32_t)foldR_sat - lp_newR) * scram_coef) >> 15;
            wet_new_L = (int16_t)lp_newL;
            wet_new_R = (int16_t)lp_newR;
        } else {
            lp_newL = wetL;
            lp_newR = wetR;
        }

        // Mix the engines continuously
        int32_t total_w = fuzz_level + bad_conn_level + scramble_level;
        if (total_w > 0) {
            int32_t mixedL = ((int32_t)wet_codec_L * fuzz_level + 
                              (int32_t)wet_trans_L * bad_conn_level + 
                              (int32_t)wet_new_L * scramble_level) / total_w;
            int32_t mixedR = ((int32_t)wet_codec_R * fuzz_level + 
                              (int32_t)wet_trans_R * bad_conn_level + 
                              (int32_t)wet_new_R * scramble_level) / total_w;
            wetL = saturate_q15(mixedL);
            wetR = saturate_q15(mixedR);
        }

        // ── 3. VCA Compression & Analog Saturation (scaled by Strength) ───────
        int32_t absL = wetL < 0 ? -wetL : wetL;
        int32_t absR = wetR < 0 ? -wetR : wetR;
        int32_t peak = absL > absR ? absL : absR;
        if (peak > 32767) peak = 32767;

        // Envelope follower (Vibe LFO creates breathing release fluctuations)
        int32_t attack_shift = 5;
        int32_t release_shift = 11 + (vibe_sine >> 13);
        if (peak > env) env += (peak - env) >> attack_shift;
        else env += (peak - env) >> release_shift;

        int32_t thresh = 28000 - ((strength * 25000) >> 15);
        int32_t slope = (strength * 27000) >> 15;

        int32_t gain_coef = 32768;
        if (env > thresh) {
            int32_t overshoot = env - thresh;
            int32_t gain_reduction = ((int64_t)overshoot * slope) >> 15;
            gain_coef = 32768 - gain_reduction;
            if (gain_coef < 3000) gain_coef = 3000;
        }

        int32_t drive_gain = 32768 + ((strength * 27232) >> 15); 
        int32_t compL = ((int32_t)wetL * drive_gain) >> 15;
        int32_t compR = ((int32_t)wetR * drive_gain) >> 15;
        
        compL = (compL * gain_coef) >> 15;
        compR = (compR * gain_coef) >> 15;

        // Apply makeup gain to compensate for compression gain reduction
        int32_t makeup_gain = 32768 + ((strength * 40000) >> 15); // up to 2.22x makeup
        compL = (int32_t)(((int64_t)compL * makeup_gain) >> 15);
        compR = (int32_t)(((int64_t)compR * makeup_gain) >> 15);

        wetL = tape_saturate(soft_limit_q15(compL));
        wetR = tape_saturate(soft_limit_q15(compR));

        // ── 4. Downsampling (from X Knob) ─────────────────────────────────────
        if (downsample > 150) {
            if (dec_ctr >= current_dec_step) {
                decL    = wetL;
                decR    = wetR;
                dec_ctr = 0;

                // Calculate next interval step size with jitter
                uint32_t base_step = 1 + ((uint32_t)downsample >> 9);
                // Jitter scale depends on strength and downsample
                uint32_t jitter_range = ((strength >> 11) * (downsample >> 11)) >> 4;
                if (jitter_range < 2) jitter_range = 2; // minimum range
                uint32_t jitter = ((uint64_t)fast_rand(rand_seed) * jitter_range) >> 32;
                current_dec_step = base_step + jitter;
                if (current_dec_step > 48) current_dec_step = 48;
            } else {
                wetL = decL;
                wetR = decR;
            }
            dec_ctr++;

            // Continuous LPF to smooth out downsampling steps (scaled to preserve crunch/aliasing highs, min cutoff ~700Hz)
            int32_t dec_coef = 32768 / (1 + (current_dec_step >> 2));
            if (dec_coef > 32767) dec_coef = 32767;
            if (dec_coef < 3000) dec_coef = 3000;

            dec_lpL += (((int32_t)wetL - dec_lpL) * dec_coef) >> 15;
            dec_lpR += (((int32_t)wetR - dec_lpR) * dec_coef) >> 15;
            wetL = (int16_t)dec_lpL;
            wetR = (int16_t)dec_lpR;
        } else {
            decL    = wetL;
            decR    = wetR;
            dec_ctr = 0;
            current_dec_step = 1;
            dec_lpL = wetL;
            dec_lpR = wetR;
        }

        // ── 5. Signal Integrity Inconsistencies (Dropouts & Crackles) ─────────
        int16_t out_wetL = wetL;
        int16_t out_wetR = wetR;

        if (strength > 16384) { // only above 50% strength
            int32_t sputter_prob = ((strength - 16384) * 80) >> 15;
            uint32_t roll = fast_rand(rand_seed) & 0x7FFF;

            if (sputter_timer > 0) {
                sputter_timer--;
                if (sputter_active) {
                    int16_t noise = (int16_t)(fast_rand(rand_seed) & 0xFFFF);
                    out_wetL = (noise * 600) >> 15;
                    out_wetR = (noise * 600) >> 15;
                } else {
                    out_wetL = 0;
                    out_wetR = 0;
                }
            } else {
                sputter_active = false;
                if ((int32_t)roll < sputter_prob) {
                    sputter_timer = 5 + (fast_rand(rand_seed) % 115);
                    sputter_active = (fast_rand(rand_seed) % 10 < 3);
                    if (!sputter_active) {
                        out_wetL = 0;
                        out_wetR = 0;
                    }
                }
            }
        } else {
            sputter_timer = 0;
            sputter_active = false;
        }

        // Final mix: reaches 100% wet at 25% strength (8192) to prevent dry masking
        int32_t mix_coeff = strength * 4;
        if (mix_coeff > 32767) mix_coeff = 32767;

        outL = lerp_q15(inL, out_wetL, (int16_t)mix_coeff);
        outR = lerp_q15(inR, out_wetR, (int16_t)mix_coeff);
    }

    bool isFrameDropped() const { return trans_dropped; }
};

// ============================================================================
// 3.  MULTI-TAP DELAY BLOCK
// ============================================================================
// Stereo tape echo with three rhythmic taps. Delay time is slewed through a
// one-pole IIR for "tape inertia" pitch-glide on time changes.
// Cross-feedback (L→R, R→L) gives rich stereo spatial spread.
// CV1 warps delay time for pitch effects (chorus-style).
// Freeze (switch UP) locks the write pointer — the delay buffer loops forever.
//
// Parameters:
//   mainMix  — Wet / dry blend.
//   time     — Primary tap delay time. [0..32767] → [128..16300] samples.
//   feedback — Cross-feedback amount. High values → infinite wash.
//   freeze   — When true: write pointer frozen; buffer loops without new input.
// ============================================================================
struct MultiTapDelayBlock {
    // Stereo ring buffer — 28672 samples ≈ 597 ms @ 48 kHz
    int16_t  bufL[28672];
    int16_t  bufR[28672];
    uint16_t wr = 0;

    // IIR-smoothed delay time (prevents zipper on rapid changes)
    int32_t  smooth_t = 8000;

    // Wow & flutter LFO phase accumulator
    uint16_t flutter_phase = 0;

    // Feedback 1-pole LPF state variables
    int32_t  lp_feedback_L = 0;
    int32_t  lp_feedback_R = 0;

    // PT2399 clock decimation state variables
    uint32_t clk_phase = 0;
    int16_t  last_outL = 0;
    int16_t  last_outR = 0;
    int32_t  lp_outL = 0;
    int32_t  lp_outR = 0;

    void init() {
        memset(bufL, 0, sizeof(bufL));
        memset(bufR, 0, sizeof(bufR));
        wr            = 0;
        smooth_t      = 8000;
        flutter_phase = 0;
        lp_feedback_L = 0;
        lp_feedback_R = 0;
        clk_phase     = 0;
        last_outL     = 0;
        last_outR     = 0;
        lp_outL       = 0;
        lp_outR       = 0;
    }

    void process(int16_t inL, int16_t &outL, int16_t inR, int16_t &outR,
                 int32_t mainMix, int32_t time, int32_t feedback,
                 bool freeze, int32_t cv1Warp, int32_t cv2Corruption, int32_t globalNoiseScale = 16384)
    {
        // ── PT2399 Slow-Clock Delay Decimation ──
        uint32_t clk_inc = 65536;
        if (time > 16384) {
            int32_t diff = time - 16384;
            clk_inc = 65536 - ((diff * (65536 - 3276)) / 16383);
        }

        // Calculate filter coefficient for continuous reconstruction filter
        int32_t filter_coef = 32767;
        if (clk_inc < 65536) {
            filter_coef = 3000 + (((int32_t)(clk_inc - 3276) * (32767 - 3000)) / (65536 - 3276));
        }

        clk_phase += clk_inc;
        if (clk_phase < 65536) {
            // Run LPF continuously even on early-return samples
            lp_outL += (((int32_t)last_outL - lp_outL) * filter_coef) >> 15;
            lp_outR += (((int32_t)last_outR - lp_outR) * filter_coef) >> 15;

            outL = lerp_q15(inL, (int16_t)lp_outL, (int16_t)mainMix);
            outR = lerp_q15(inR, (int16_t)lp_outR, (int16_t)mainMix);
            return;
        }
        clk_phase -= 65536;

        if (mainMix < 50) {
            outL = inL;
            outR = inR;
            last_outL = inL;
            last_outR = inR;
            lp_outL = inL;
            lp_outR = inR;
            if (!freeze) {
                bufL[wr] = inL;
                bufR[wr] = inR;
                wr = wr + 1;
                if (wr >= 28672) wr = 0;
            }
            int32_t mapped_time = 128 + (((int64_t)time * 27872) >> 15);
            int32_t target_t = mapped_time + (cv1Warp * 4);
            target_t = clamp_i32(target_t, 128, 28500);
            IIR_SMOOTH(smooth_t, target_t, 12);
            return;
        }

        // Slew delay time with CV1 pitch-warp
        int32_t mapped_time = 128 + (((int64_t)time * 27872) >> 15);
        int32_t target_t = mapped_time + (cv1Warp * 4);
        target_t = clamp_i32(target_t, 128, 28500);

        IIR_SMOOTH(smooth_t, target_t, 12);

        // Increment wow & flutter LFO (~2.2 Hz)
        flutter_phase += 3;
        int16_t lfo = lookup_sine_fast(flutter_phase);
        int32_t flutter = (lfo * 8) >> 15; // up to ±8 samples of flutter

        // Fractional-sample read with linear interpolation (optimised: division-free)
        auto read_stereo = [&](int32_t delay_q16,
                               int16_t &rL, int16_t &rR) {
            int32_t rp_i = (int32_t)wr - (delay_q16 >> 16);
            if (rp_i < 0) {
                rp_i += 28672;
                if (rp_i < 0) rp_i = 0;
            } else if (rp_i >= 28672) {
                rp_i -= 28672;
                if (rp_i >= 28672) rp_i = 0;
            }
            int32_t rp_n = rp_i + 1;
            if (rp_n >= 28672) rp_n = 0;
            uint16_t frac = (uint16_t)(delay_q16 & 0xFFFF);
            {
                int16_t y0 = bufL[rp_i], y1 = bufL[rp_n];
                rL = (int16_t)(y0 + (((int64_t)(y1 - y0) * frac) >> 16));
            }
            {
                int16_t y0 = bufR[rp_i], y1 = bufR[rp_n];
                rR = (int16_t)(y0 + (((int64_t)(y1 - y0) * frac) >> 16));
            }
        };

        // Apply wow & flutter directly to panned tap read pointers
        int32_t t1_q16 = (smooth_t + flutter) << 16;
        int32_t t2_q16 = ((smooth_t + flutter) * 3) << 14;
        int32_t t3_q16 = (int32_t)((int64_t)(smooth_t + flutter) * 40503);

        int16_t w1L, w1R, w2L, w2R, w3L, w3R;
        read_stereo(t1_q16, w1L, w1R);
        read_stereo(t2_q16, w2L, w2R);
        read_stereo(t3_q16, w3L, w3R);

        // Scale secondary and tertiary taps based on feedback to blend them in slowly
        // and maintain a constant total gain sum of exactly 32768 (unity) to prevent volume loss
        int32_t gain1 = 32768 - (feedback >> 1); // primary tap drops from 100% to 50%
        int32_t gain2L = (feedback * 12000) >> 15; // Tap 2 panned Left
        int32_t gain2R = (feedback * 4384) >> 15;  // Tap 2 panned Right
        int32_t gain3L = (feedback * 4384) >> 15;  // Tap 3 panned Left
        int32_t gain3R = (feedback * 12000) >> 15; // Tap 3 panned Right

        int16_t mixL = saturate_q15(
            ((int32_t)w1L * gain1 + (int32_t)w2L * gain2L + (int32_t)w3L * gain3L) >> 15);
        int16_t mixR = saturate_q15(
            ((int32_t)w1R * gain1 + (int32_t)w2R * gain2R + (int32_t)w3R * gain3R) >> 15);

        // Write to buffer (unless frozen)
        if (!freeze) {
            // Cross-feedback: L is fed back by R's output, R is fed back by L's output
            int32_t feedL = (int32_t)inL + (((int32_t)mixR * feedback) >> 15);
            int32_t feedR = (int32_t)inR + (((int32_t)mixL * feedback) >> 15);
            
            // 1-pole low-pass filter on feedback loops (analog decay warmth)
            lp_feedback_L += ((feedL - lp_feedback_L) * 8000) >> 15;
            lp_feedback_R += ((feedR - lp_feedback_R) * 8000) >> 15;

            int16_t wrL = soft_limit_q15(lp_feedback_L);
            int16_t wrR = soft_limit_q15(lp_feedback_R);
            
            // Bipolar CV2 controls feedback XOR corruption (circuit bend!) scaled by global noise scale
            int32_t cv2_abs = cv2Corruption < 0 ? -cv2Corruption : cv2Corruption;
            int32_t corr = (cv2_abs * 8 * globalNoiseScale) >> 14;
            if (corr > 200) {
                uint16_t mask = (uint16_t)(corr >> 3);
                wrL ^= mask;
                wrR ^= mask;
            }
            
            bufL[wr] = wrL;
            bufR[wr] = wrR;
            wr = wr + 1;
            if (wr >= 28672) wr = 0;
        }

        last_outL = mixL;
        last_outR = mixR;

        // Run reconstruction filter on the wet output
        lp_outL += (((int32_t)last_outL - lp_outL) * filter_coef) >> 15;
        lp_outR += (((int32_t)last_outR - lp_outR) * filter_coef) >> 15;

        outL = lerp_q15(inL, (int16_t)lp_outL, (int16_t)mainMix);
        outR = lerp_q15(inR, (int16_t)lp_outR, (int16_t)mainMix);
    }
};

// ============================================================================
// 4.  CIRCUIT-BENT GLITCHER BLOCK
// ============================================================================
// Phrase-loop stutter sampler. When inactive, it continuously writes incoming
// audio to a 16384-sample (341 ms) circular buffer.
//
// Triggering (either via Pulse 1 trigger or random roll exceeding mainProb)
// locks the write pointer, and starts playing a loop of 'size' samples.
//
// Parameters:
//   mainMix     — Wet / dry blend.
//   size        — Loop length: [0..32767] → [32..16384] samples.
//   speedDir    — Playback speed + direction:
//                   [0, 15000]   → reverse, −2.0× to −0.05×
//                   [15001,17000] → freeze (0× speed deadzone)
//                   [17001,32767] → forward, +0.05× to +2.0×
//   gateTrigger — Pulse 1 jack: stutter gate (momentary).
//   switchFreeze — Switch UP: latched freeze.
// ============================================================================
struct GlitcherBlock {
    int16_t  bufL[16384];
    int16_t  bufR[16384];
    uint16_t wr = 0;

    bool     active     = false;
    uint16_t freeze_wr  = 0;       // write pointer at moment of freeze

    // Playback pointer (Q16: integer + 16-bit fraction relative to loop_start)
    int32_t  rd_q16     = 0;

    // Crossfade for click-free loop boundaries
    int32_t  xfade_ctr  = 0;       // countdown: xfade_len → 0
    int32_t  xfade_rd   = 0;       // secondary read pointer (Q16) for crossfade
    int32_t  xfade_len  = 256;
    int32_t  xfade_phase = 0;
    int32_t  xfade_step  = 0;

    // Dynamic loop length tracking
    int32_t  current_loop_len = 512;
    int32_t  sample_ctr = 0;

    // Smooth exit crossfade (stutter playback → live dry)
    int32_t  dry_fade_ctr = 0;
    int32_t  dry_fade_rd  = 0;
    int32_t  dry_fade_len = 256;
    int32_t  dry_fade_phase = 0;
    int32_t  dry_fade_step  = 0;

    // Loop onset crossfade (fading from live dry to loop playback)
    int32_t  onset_fade_ctr = 0;
    int32_t  onset_fade_len = 256;
    int32_t  onset_fade_phase = 0;
    int32_t  onset_fade_step  = 0;

    // Latched playback speed (for random timings and reverses probability field)
    int32_t  current_speed_q16 = 65536;

    void init() {
        memset(bufL, 0, sizeof(bufL));
        memset(bufR, 0, sizeof(bufR));
        wr         = 0;
        active     = false;
        freeze_wr  = 0;
        rd_q16     = 0;
        xfade_ctr  = 0;
        xfade_rd   = 0;
        xfade_len  = 256;
        xfade_phase = 0;
        xfade_step  = 0;
        current_loop_len = 512;
        sample_ctr = 0;
        dry_fade_ctr = 0;
        dry_fade_rd  = 0;
        dry_fade_len = 256;
        dry_fade_phase = 0;
        dry_fade_step  = 0;
        onset_fade_ctr = 0;
        onset_fade_len = 256;
        onset_fade_phase = 0;
        onset_fade_step  = 0;
        current_speed_q16 = 65536;
    }

    void process(int16_t inL, int16_t &outL, int16_t inR, int16_t &outR,
                 int32_t mainProb, int32_t size, int32_t speedQuant,
                 bool glitchInjector, bool freezeGate, int32_t cv1Warp, int32_t cv2Corruption,
                 uint32_t &rand_seed, int32_t scrubOffset = 0, int32_t glitchFeedback = 0, int32_t globalNoiseScale = 16384)
    {
        bool want_active = glitchInjector || freezeGate;

        if (mainProb < 50 && !want_active && !active && dry_fade_ctr == 0) {
            outL = inL;
            outR = inR;
            bufL[wr] = inL;
            bufR[wr] = inR;
            wr = (wr + 1) & 0x3FFF;
            return;
        }

        // ── Loop size snap (rhythmic power-of-two subdivisions if CV1 unplugged)
        int32_t loop_size;
        if (cv1Warp == 0) {
            // Snaps to 8 distinct rhythmic sizes: 128, 256, 512, 1024, 2048, 4096, 8192, 16384 samples
            int32_t step = size / 4100;
            if (step < 0) step = 0;
            if (step > 7) step = 7;
            loop_size = 128 << step;
        } else {
            // CV1 is plugged: allow continuous sweep / circuit-bent size warping
            int32_t base_size = 128 + (size >> 1);
            loop_size = base_size + (cv1Warp * 4);
        }
        loop_size = clamp_i32(loop_size, 128, 16384);

        int32_t cur_xfade = loop_size < 512 ? (loop_size >> 1) : 256;
        if (cur_xfade < 4) cur_xfade = 4;

        // ── Playback speed probability helper ──────────────────────────────────
        auto determine_random_speed = [globalNoiseScale](int32_t sq, int32_t cv2_corr, uint32_t &seed) -> int32_t {
            int32_t base_speed = 65536; // 1.0x in Q16
            
            // sq represents the probability of choosing a random speed/direction.
            int32_t eff_sq = (sq * globalNoiseScale) >> 14;
            if (eff_sq > 32767) eff_sq = 32767;

            uint32_t roll = fast_rand(seed) & 0x7FFF;
            if ((int32_t)roll < eff_sq) {
                uint32_t choice = fast_rand(seed) % 6;
                switch (choice) {
                    case 0: base_speed = 65536;    // 1x forward
                            break;
                    case 1: base_speed = 131072;   // 2x forward (octave up)
                            break;
                    case 2: base_speed = 32768;    // 0.5x forward (octave down)
                            break;
                    case 3: base_speed = -65536;   // 1x reverse
                            break;
                    case 4: base_speed = -131072;  // 2x reverse (reverse octave up)
                            break;
                    case 5: base_speed = -32768;   // 0.5x reverse (reverse octave down)
                }
            } else {
                base_speed = 65536; // default 1x
            }
            
            int32_t cv2_mod = cv2_corr * 32;
            int32_t speed = base_speed + cv2_mod;
            if (speed == 0) speed = 3277;
            return speed;
        };

        // Determine current playback speed
        int32_t speed_q16 = 65536;
        if (freezeGate) {
            // Direct quantized speed control on Freeze Scrub manual pages
            int32_t base_speed = 65536;
            if (speedQuant < 5461)       base_speed = 65536;
            else if (speedQuant < 10922)  base_speed = 131072;
            else if (speedQuant < 16383)  base_speed = 32768;
            else if (speedQuant < 21844)  base_speed = -65536;
            else if (speedQuant < 27305)  base_speed = -131072;
            else                          base_speed = -32768;
            
            int32_t cv2_mod = cv2Corruption * 32;
            speed_q16 = base_speed + cv2_mod;
            if (speed_q16 == 0) speed_q16 = 3277;
        } else {
            // Use the speed latched at loop boundary
            speed_q16 = current_speed_q16;
        }

        // ── State Machine: trigger/check stutter ──────────────────────────────
        if (!active) {
            bufL[wr] = inL;
            bufR[wr] = inR;

            bool trigger = want_active || ((wr % (uint16_t)loop_size) == 0 && (fast_rand(rand_seed) & 0x7FFF) < (uint32_t)mainProb);

            if (trigger) {
                active = true;
                freeze_wr = wr;
                current_loop_len = loop_size;
                
                // Determine initial speed/direction for this glitch loop
                current_speed_q16 = determine_random_speed(speedQuant, cv2Corruption, rand_seed);
                speed_q16 = current_speed_q16;
                
                rd_q16 = (speed_q16 >= 0) ? 0 : (current_loop_len << 16);
                xfade_ctr = 0;
                dry_fade_ctr = 0;
                sample_ctr = 0; // Initialize sample_ctr to 0 on trigger
                
                onset_fade_len = cur_xfade;
                onset_fade_ctr = cur_xfade;
                onset_fade_step = (32767 << 15) / cur_xfade;
                onset_fade_phase = 0;
            }
            wr = (wr + 1) & 0x3FFF;
        }

        if (active) {
            auto read_buf = [&](int32_t ptr, int16_t &sL, int16_t &sR) {
                int32_t  idx  = (ptr >> 16) & 0x3FFF;
                int32_t  nxt  = (idx + 1)   & 0x3FFF;
                uint16_t frac = (uint16_t)(ptr & 0xFFFF);
                {
                    int16_t y0 = bufL[idx], y1 = bufL[nxt];
                    sL = (int16_t)(y0 + (((int64_t)(y1 - y0) * frac) >> 16));
                }
                {
                    int16_t y0 = bufR[idx], y1 = bufR[nxt];
                    sR = (int16_t)(y0 + (((int64_t)(y1 - y0) * frac) >> 16));
                }
            };

            int32_t offset_samples = (scrubOffset * 16384) >> 15;
            int32_t loop_start = (((int32_t)freeze_wr - current_loop_len - offset_samples) & 0x3FFF) << 16;

            rd_q16 += speed_q16;
            sample_ctr++;

            // ── Boundary detection ────────────────────────────────────────────
            bool crossed = false;
            if (speed_q16 >= 0) {
                if (rd_q16 >= (current_loop_len << 16)) {
                    crossed = true;
                }
            } else {
                if (rd_q16 < 0) {
                    crossed = true;
                }
            }

            if (crossed) {
                uint32_t roll = fast_rand(rand_seed) & 0x7FFF;
                bool keep_looping = (roll < (uint32_t)mainProb) || want_active;

                if (keep_looping) {
                    xfade_rd = loop_start + rd_q16;
                    
                    // Boundary crossed: select next speed/timing from probability field
                    // Only change speed if enough samples (1024) have elapsed to prevent alien bubble noise at short loops.
                    if (sample_ctr >= 1024) {
                        current_speed_q16 = determine_random_speed(speedQuant, cv2Corruption, rand_seed);
                        sample_ctr = 0;
                    }
                    speed_q16 = current_speed_q16;
                    
                    if (speed_q16 >= 0) {
                        rd_q16 = 0;
                    } else {
                        rd_q16 = loop_size << 16;
                    }
                    xfade_len = cur_xfade;
                    xfade_ctr = cur_xfade;
                    xfade_step = (32767 << 15) / cur_xfade;
                    xfade_phase = 0;
                    
                    current_loop_len = loop_size;
                    loop_start = (((int32_t)freeze_wr - current_loop_len - offset_samples) & 0x3FFF) << 16;
                } else {
                    active = false;
                    dry_fade_rd = loop_start + rd_q16;
                    dry_fade_len = cur_xfade;
                    dry_fade_ctr = cur_xfade;
                    dry_fade_step = (32767 << 15) / cur_xfade;
                    dry_fade_phase = 32767 << 15;
                }
            }

            if (active) {
                int16_t sL, sR;
                read_buf(loop_start + rd_q16, sL, sR);

                if (xfade_ctr > 0) {
                    int16_t xL, xR;
                    xfade_rd += speed_q16;
                    read_buf(xfade_rd, xL, xR);

                    xfade_phase += xfade_step;
                    int32_t val = xfade_phase >> 15;
                    if (val > 32767) val = 32767;
                    int16_t t = (int16_t)val;
                    sL = lerp_q15(xL, sL, t);
                    sR = lerp_q15(xR, sR, t);
                    xfade_ctr--;
                }

                if (onset_fade_ctr > 0) {
                    onset_fade_phase += onset_fade_step;
                    int32_t val = onset_fade_phase >> 15;
                    if (val > 32767) val = 32767;
                    int16_t t = (int16_t)val;
                    sL = lerp_q15(inL, sL, t);
                    sR = lerp_q15(inR, sR, t);
                    onset_fade_ctr--;
                }

                // --- NEW: Glitcher Feedback Loop ---
                if (glitchFeedback > 0) {
                    int32_t idx = ((loop_start + rd_q16) >> 16) & 0x3FFF;
                    int32_t scaled_fb = (glitchFeedback * 29491) >> 15; // cap at ~90% feedback
                    bufL[idx] = soft_limit_q15(((int32_t)bufL[idx] * (32768 - scaled_fb) + (int32_t)sL * scaled_fb) >> 15);
                    bufR[idx] = soft_limit_q15(((int32_t)bufR[idx] * (32768 - scaled_fb) + (int32_t)sR * scaled_fb) >> 15);
                }

                outL = lerp_q15(inL, sL, (int16_t)mainProb);
                outR = lerp_q15(inR, sR, (int16_t)mainProb);
                return;
            }
        }

        // ── Dry throughput / de-clicked fade out ──────────────────────────────
        if (dry_fade_ctr > 0) {
            auto read_buf = [&](int32_t ptr, int16_t &sL, int16_t &sR) {
                int32_t  idx  = (ptr >> 16) & 0x3FFF;
                int32_t  nxt  = (idx + 1)   & 0x3FFF;
                uint16_t frac = (uint16_t)(ptr & 0xFFFF);
                sL = (int16_t)(bufL[idx] + (((int64_t)(bufL[nxt] - bufL[idx]) * frac) >> 16));
                sR = (int16_t)(bufR[idx] + (((int64_t)(bufR[nxt] - bufR[idx]) * frac) >> 16));
            };

            int16_t sL, sR;
            dry_fade_rd += speed_q16;
            read_buf(dry_fade_rd, sL, sR);

            dry_fade_phase -= dry_fade_step;
            int32_t val = dry_fade_phase >> 15;
            if (val < 0) val = 0;
            if (val > 32767) val = 32767;
            int16_t t = (int16_t)val;
            int16_t wetL = lerp_q15(inL, sL, (int16_t)mainProb);
            int16_t wetR = lerp_q15(inR, sR, (int16_t)mainProb);
            outL = lerp_q15(inL, wetL, t);
            outR = lerp_q15(inR, wetR, t);
            dry_fade_ctr--;
        } else {
            outL = inL;
            outR = inR;
        }
    }
};

// ============================================================================
// 5.  RESONANT FILTER BLOCK
// ============================================================================
// Chamberlin State Variable Filter (SVF) — morphable LP / BP / HP.
// The SVF is the gold standard for musical fixed-point filters:
// one set of state variables produces all three outputs simultaneously,
// with no additional cost.
//
// KEY CORRECTNESS FIX vs. v1: The canonical Chamberlin update is:
//   hp  = in − r·v1 − v2
//   v1 += g·hp              ← single integration (velocity)
//   lp  = v2 + g·v1         ← position before integration step
//   v2  = lp                ← integrate position
// This avoids the double-integration bug in the previous version.
//
// RESONANCE COMPENSATION: At high Q, the SVF produces a strong resonance
// peak at the cutoff but loses energy in the passband. A makeup gain term
// (derived from the r coefficient) re-balances the output level so the
// filter sounds loud and present even at extreme Q settings.
//
// All state variables kept as int32_t for headroom at high resonance.
// Coefficients g and r are kept as int32_t (not int16_t) to preserve
// precision after the coefficient smoothing IIR.
//
// Parameters:
//   cutoff    — [0..32767] → cutoff from ~50 Hz to ~11 kHz (exponential sweep).
//   resonance — [0..32767] → Q from 0.5 to ~50 (low r → high Q).
//   morph     — [0..32767] → LP (0) → BP (16383) → HP (32767).
// Note: On page 5, Main knob = Cutoff (not wet/dry — filter is always inline).
// ============================================================================
struct FilterBlock {
    // Filter state — Q15 units but promoted to int32_t for headroom
    int32_t v1L = 0, v2L = 0;
    int32_t v1R = 0, v2R = 0;

    // Smoothed coefficients in Q15
    int32_t sm_g = 1000;
    int32_t sm_r = 8192;

    // Decimation states for filter output
    int16_t f_decL = 0, f_decR = 0;
    int32_t f_dec_ctr = 0;

    inline int32_t filter_saturate(int32_t x) {
        int32_t abs_x = x < 0 ? -x : x;
        if (abs_x > 16384) {
            int32_t diff = abs_x - 16384;
            int32_t compressed = 16384 + ((diff * 12288) / (diff + 12288));
            return x < 0 ? -compressed : compressed;
        }
        return x;
    }

    void init() {
        v1L = v2L = v1R = v2R = 0;
        sm_g = 1000;
        sm_r = 8192;
        f_decL = f_decR = 0;
        f_dec_ctr = 0;
    }

    void process(int16_t inL, int16_t &outL, int16_t inR, int16_t &outR,
                 int32_t cutoff, int32_t resonance, int32_t morph, int32_t cv1Warp)
    {
        // ── 1. CV1 manual sweep modulation ───────────────────────────────────
        // CV1: raw bipolar ADC [-2048, 2047]. Scale to ±8192 units.
        int32_t cv1_mod = cv1Warp * 4;

        // Combine cutoff and manual sweep and clamp to valid parameter range
        int32_t eff_cutoff = cutoff + cv1_mod;
        eff_cutoff = clamp_i32(eff_cutoff, 0, 32767);

        // ── Coefficient mapping ───────────────────────────────────────────────
        // Cutoff: exponential curve so low end feels musical.
        int32_t target_g = 300 + ((eff_cutoff * eff_cutoff) >> 15);
        target_g = clamp_i32(target_g, 300, 22000);

        // Resonance knob (X knob) re-mapping
        int32_t target_r = 32000;
        int32_t fold_thresh = 32767;
        int32_t dec_step = 0;
        int32_t xor_mask = 0;
        int32_t drive = 32768;
        int32_t makeup = 32768;

        if (resonance < 18000) {
            // Low to Mid range: liquid squelchy resonance sweeps down (r decreases) non-linearly (quadratic curve)
            int32_t scale = 18000 - resonance;
            int32_t scale_q15 = (scale * 59650) >> 15;
            int32_t scale_sq = (scale_q15 * scale_q15) >> 15;
            target_r = 1000 + ((31000 * scale_sq) >> 15);
            fold_thresh = 32767;
            dec_step = 0;
            xor_mask = 0;
            drive = 32768;
            makeup = 32768;
        } else {
            // Mid to High range: resonance drops back down, wavefolding and decimation/XOR fade in
            int32_t diff = resonance - 18000;
            int32_t diff_q15 = (diff * 72710) >> 15;
            if (diff_q15 > 32767) diff_q15 = 32767;
            
            // Damping sweeps up to 28000 to eliminate self-oscillation ringing
            target_r = 1000 + ((diff_q15 * 27000) >> 15);
            
            // Input drive sweeps from 1.0x to 2.37x
            drive = 32768 + ((diff_q15 * 45000) >> 15);

            // Fold threshold sweeps down from 32767 to 4000
            fold_thresh = 32767 - ((diff_q15 * 28767) >> 15);

            // Output makeup gain sweeps from 1.0x to 4.0x
            makeup = 32768 + ((diff_q15 * 98304) >> 15);

            dec_step = (diff * 16) / (32767 - 18000);
            xor_mask = (diff * 127) / (32767 - 18000);
        }
        target_r = clamp_i32(target_r, 400, 32000);

        // Smooth coefficients (tau ≈ 256 samples / ~5 ms) to eliminate zipper noise
        IIR_SMOOTH(sm_g, target_g, 8);
        IIR_SMOOTH(sm_r, target_r, 8);

        int32_t g = sm_g;
        int32_t r = sm_r;

        // ── Resonance makeup gain ─────────────────────────────────────────────
        int32_t gain_q15 = 16384 + ((32000 - r) >> 2);
        gain_q15 = clamp_i32(gain_q15, 16384, 24576);

        // ── Canonical Chamberlin SVF update — Left channel ───────────────────
        {
            int32_t feedbackL = ((r * v1L) >> 15) + v2L;
            int32_t hp = (int32_t)inL - filter_saturate(feedbackL);
            v1L       += (g * hp) >> 15;  // integrate velocity
            v1L        = soft_limit_q15(v1L); // stabilize resonance in loop
            int32_t lp = v2L + ((g * v1L) >> 15);
            v2L        = lp;               // integrate position
            v2L        = soft_limit_q15(v2L); // stabilize position in loop

            int16_t lp16 = saturate_q15(lp);
            int16_t bp16 = saturate_q15(v1L);
            int16_t hp16 = saturate_q15(hp);

            int16_t morphed;
            if (morph < 16384) {
                morphed = lerp_q15(lp16, bp16, (int16_t)(morph * 2));
            } else {
                morphed = lerp_q15(bp16, hp16, (int16_t)((morph - 16384) * 2));
            }

            // Apply asymmetric multi-bounce digital wavefolding with input drive and output makeup gain
            if (fold_thresh < 32767) {
                int32_t x = ((int32_t)morphed * drive) >> 15;
                int32_t pos_t = fold_thresh + (fold_thresh >> 4); // +6.25% shift
                int32_t neg_t = fold_thresh - (fold_thresh >> 4); // -6.25% shift
                if (neg_t < 1000) neg_t = 1000;
                for (int i = 0; i < 5; i++) {
                    if (x > pos_t) {
                        x = pos_t - (x - pos_t);
                    } else if (x < -neg_t) {
                        x = -neg_t - (x + neg_t);
                    } else {
                        break;
                    }
                }
                x = (x * makeup) >> 15;
                morphed = saturate_q15(x);
            }

            // Decimation
            if (dec_step > 1) {
                if (f_dec_ctr >= dec_step) {
                    f_decL = morphed;
                } else {
                    morphed = f_decL;
                }
            } else {
                f_decL = morphed;
            }

            // XOR bitwise corruption
            if (xor_mask > 0) {
                morphed = morphed ^ xor_mask;
            }

            outL = saturate_q15(((int32_t)morphed * gain_q15) >> 14);
        }

        // ── Right channel ────────────────────────────────────────────────────
        {
            int32_t feedbackR = ((r * v1R) >> 15) + v2R;
            int32_t hp = (int32_t)inR - filter_saturate(feedbackR);

            v1R       += (g * hp) >> 15;
            v1R        = soft_limit_q15(v1R); // stabilize resonance in loop
            int32_t lp = v2R + ((g * v1R) >> 15);
            v2R        = lp;
            v2R        = soft_limit_q15(v2R); // stabilize position in loop

            int16_t lp16 = saturate_q15(lp);
            int16_t bp16 = saturate_q15(v1R);
            int16_t hp16 = saturate_q15(hp);

            int16_t morphed;
            if (morph < 16384) {
                morphed = lerp_q15(lp16, bp16, (int16_t)(morph * 2));
            } else {
                morphed = lerp_q15(bp16, hp16, (int16_t)((morph - 16384) * 2));
            }

            // Apply asymmetric multi-bounce digital wavefolding with input drive and output makeup gain
            if (fold_thresh < 32767) {
                int32_t x = ((int32_t)morphed * drive) >> 15;
                int32_t pos_t = fold_thresh + (fold_thresh >> 4); // +6.25% shift
                int32_t neg_t = fold_thresh - (fold_thresh >> 4); // -6.25% shift
                if (neg_t < 1000) neg_t = 1000;
                for (int i = 0; i < 5; i++) {
                    if (x > pos_t) {
                        x = pos_t - (x - pos_t);
                    } else if (x < -neg_t) {
                        x = -neg_t - (x + neg_t);
                    } else {
                        break;
                    }
                }
                x = (x * makeup) >> 15;
                morphed = saturate_q15(x);
            }

            // Decimation
            if (dec_step > 1) {
                if (f_dec_ctr >= dec_step) {
                    f_decR = morphed;
                    f_dec_ctr = 0; // reset L/R sync counter
                } else {
                    morphed = f_decR;
                }
            } else {
                f_decR = morphed;
            }
            if (dec_step > 1) {
                f_dec_ctr++;
            }

            // XOR bitwise corruption
            if (xor_mask > 0) {
                morphed = morphed ^ xor_mask;
            }

            outR = saturate_q15(((int32_t)morphed * gain_q15) >> 14);
        }
    }
};

// ============================================================================
// 6.  STUDIO REVERB BLOCK
// ============================================================================
// Classic Schroeder plate reverb topology:
//   4 parallel comb filters → summed → 2 series all-pass diffusers.
// Left and right channels use slightly different prime-length delays for
// natural decorrelation and wide stereo image.
//
// Comb filter improvement: the first-order damping LPF state is kept as
// int32_t (not int16_t) so it doesn't quantise the HF rolloff.
// The comb buffer size is capped at 1700 to comfortably fit prime lengths
// up to ≈1700 samples (≈35 ms at 48 kHz).
//
// Parameters:
//   mainMix — Wet / dry blend.
//   decay   — RT60 / tail length: [0..32767] → feedback [0.70 to 0.97].
//   damping — HF damping per loop. 0 = bright; 32767 = very warm/muffled.
// ============================================================================

struct ReverbBlock {
    struct AP {
        int16_t *bufIn, *bufOut;
        uint16_t mask, ptr, len;
        int16_t g;

        void init(int16_t *b, uint16_t m, uint16_t l, int16_t gain) {
            bufIn = b;
            bufOut = b + m + 1;
            mask = m;
            len = l & m; // Store effective length
            g = gain;
            ptr = 0;
        }

        int16_t process(int16_t in, int32_t len_scale) { // len_scale is Q15
            int32_t scaled_len_q16 = len_scale * len * 2;
            int32_t scaled_len_int = scaled_len_q16 >> 16;
            uint16_t frac = (uint16_t)(scaled_len_q16 & 0xFFFF);

            if (scaled_len_int < 2) {
                scaled_len_int = 2;
                frac = 0;
            }
            if (scaled_len_int >= mask) {
                scaled_len_int = mask - 1;
                frac = 0xFFFF;
            }

            uint16_t rd1 = (ptr - scaled_len_int) & mask;
            uint16_t rd2 = (ptr - (scaled_len_int + 1)) & mask;

            // Linear interpolation of both bufIn and bufOut
            int16_t dIn = (int16_t)(bufIn[rd1] + (((int32_t)(bufIn[rd2] - bufIn[rd1]) * frac) >> 16));
            int16_t dOut = (int16_t)(bufOut[rd1] + (((int32_t)(bufOut[rd2] - bufOut[rd1]) * frac) >> 16));

            int32_t interm = ((int32_t)(g * in) >> 15) + dIn - ((int32_t)(dOut * g) >> 15);
            if (interm > 32767) interm = 32767;
            else if (interm < -32768) interm = -32768;

            int16_t out = (int16_t)interm;
            bufIn[ptr] = in;
            bufOut[ptr] = out;
            ptr = (ptr + 1) & mask;
            return out;
        }
    };

    struct Delay {
        int16_t *buf;
        uint16_t mask, ptr, len;

        void init(int16_t *b, uint16_t m, uint16_t l) {
            buf = b;
            mask = m;
            len = l & m; // Store effective length
            ptr = 0;
        }

        void write(int16_t in) {
            buf[ptr] = in;
            ptr = (ptr + 1) & mask;
        }

        int16_t read(int32_t len_scale) { // len_scale is Q15
            int32_t scaled_len_q16 = len_scale * len * 2;
            int32_t scaled_len_int = scaled_len_q16 >> 16;
            uint16_t frac = (uint16_t)(scaled_len_q16 & 0xFFFF);

            if (scaled_len_int < 2) {
                scaled_len_int = 2;
                frac = 0;
            }
            if (scaled_len_int >= mask) {
                scaled_len_int = mask - 1;
                frac = 0xFFFF;
            }

            uint16_t rd1 = (ptr - scaled_len_int) & mask;
            uint16_t rd2 = (ptr - (scaled_len_int + 1)) & mask;

            int16_t val1 = buf[rd1];
            int16_t val2 = buf[rd2];
            return (int16_t)(val1 + (((int32_t)(val2 - val1) * frac) >> 16));
        }
    };

    int16_t mem[28672];
    AP apIn[4], apTankL, apTankR;
    Delay modL, d1L, d2L, modR, d1R, d2R;
    int32_t lpL = 0, lpR = 0, lpIn = 0;
    uint32_t lfo = 0;

    // Decimation state for glitch effect
    uint16_t decimate_phase = 0;
    int16_t last_outL = 0;
    int16_t last_outR = 0;
    int32_t dec_lpL = 0, dec_lpR = 0;

    DCBlocker dc_loopL;
    DCBlocker dc_loopR;

    void init() {
        memset(mem, 0, sizeof(mem));
        int16_t *p = mem;
        apIn[0].init(p, 127, 229, 24576);
        p += 256;
        apIn[1].init(p, 127, 172, 24576);
        p += 256;
        apIn[2].init(p, 511, 611, 20480);
        p += 1024;
        apIn[3].init(p, 255, 447, 20480);
        p += 512;
        modL.init(p, 1023, 1083);
        p += 1024;
        d1L.init(p, 4095, 6000);
        p += 4096;
        apTankL.init(p, 2047, 2903, 16384);
        p += 4096;
        d2L.init(p, 4095, 5800);
        p += 4096;
        modR.init(p, 1023, 1464);
        p += 1024;
        d1R.init(p, 4095, 6200);
        p += 4096;
        apTankR.init(p, 2047, 3850, 16384);
        p += 4096;
        d2R.init(p, 4095, 5500);
        p += 4096;

        lpL = 0;
        lpR = 0;
        lpIn = 0;
        lfo = 0;
        decimate_phase = 0;
        last_outL = 0;
        last_outR = 0;
        dec_lpL = 0;
        dec_lpR = 0;
        dc_loopL.init();
        dc_loopR.init();
    }

    void process(int16_t &L, int16_t &R, int32_t mix, int32_t size, int32_t fb_glitch) {
        if (mix < 50) {
            return;
        }

        // Map Size (X) to scale factor: [0..32767] -> [4915..32767] (0.15x to 1.0x)
        int32_t size_scale = 4915 + (((int32_t)size * 27852) >> 15);

        // Max decay limit is dynamic based on size to prevent self-oscillation explosion
        // [4915..32767] size_scale maps to [18000..28672] max_decay (0.55x to 0.875x decay coefficient)
        int32_t max_decay = 18000 + (((size_scale - 4915) * 10672) / (32767 - 4915));

        // Map Feedback/Glitch (Y):
        // - 0 to 50% (0..16384): clean decay [0..max_decay], no lofi, no circuit-bend
        // - 50% to 80% (16384..26214): decay stays at max_decay, scale lofi_level [0..32767] (bitcrush + LPF damp)
        // - 80% to 100% (26215..32767): decay drops slightly, lofi_level = 32767, scale circuit_bent_level [0..32767] (XOR + decimation)
        int32_t decay = 0;
        int32_t lofi_level = 0;
        int32_t circuit_bent_level = 0;
        if (fb_glitch < 16384) {
            decay = (fb_glitch * max_decay) / 16384;
            lofi_level = 0;
            circuit_bent_level = 0;
        } else if (fb_glitch < 26214) {
            decay = max_decay;
            lofi_level = ((fb_glitch - 16384) * 32767) / 9830;
            circuit_bent_level = 0;
        } else {
            int32_t diff = fb_glitch - 26214;
            decay = max_decay - ((diff * 4000) / 6553);
            lofi_level = 32767;
            circuit_bent_level = (diff * 32767) / 6553;
        }

        // LFO Chorus: constant slow speed for high-fidelity lushness (no fast warbles)
        lfo += 128;
        int16_t mod = (lfo >> 16) & 0x7FFF;
        if (lfo & 0x80000000) mod = 32767 - mod;

        // Modulate size_scale slightly by LFO for lush chorus effect
        // Keep modulation depth small and constant (120 ≡ ~0.37% size) for rich movement
        int32_t mod_depth = 120;
        int32_t modulated_size_scale = size_scale + (((mod - 16384) * mod_depth) >> 15);
        if (modulated_size_scale < 3276) modulated_size_scale = 3276;
        if (modulated_size_scale > 32767) modulated_size_scale = 32767;

        // Input mono summing and low-pass filtering (fixed damping)
        int16_t mono = (int16_t)(((int32_t)L + (int32_t)R) >> 1);
        lpIn += (((int32_t)mono - lpIn) * 16000) >> 15;
        mono = (int16_t)(lpIn >> 1);

        // Input all-passes
        for (int i = 0; i < 4; i++) {
            mono = apIn[i].process(mono, modulated_size_scale);
        }

        // Read tank loop outputs
        int16_t tOutL = d2L.read(modulated_size_scale);
        int16_t tOutR = d2R.read(modulated_size_scale);

        // 1. Continuous Bitcrushing (word-length truncation) based on lofi_level
        if (lofi_level > 0) {
            int32_t shift_q15 = (lofi_level * 6); // scale from 0 to 6 in Q15
            int32_t int_shift = shift_q15 >> 15;
            int32_t frac_shift = shift_q15 & 0x7FFF;

            int16_t q1L = (tOutL >> int_shift) << int_shift;
            int16_t q2L = (tOutL >> (int_shift + 1)) << (int_shift + 1);
            tOutL = lerp_q15(q1L, q2L, frac_shift);

            int16_t q1R = (tOutR >> int_shift) << int_shift;
            int16_t q2R = (tOutR >> (int_shift + 1)) << (int_shift + 1);
            tOutR = lerp_q15(q1R, q2R, frac_shift);
        }

        // 2. XOR Scrambling (only in the last 20% circuit-bent range)
        if (circuit_bent_level > 0) {
            int16_t xor_mask = (int16_t)((circuit_bent_level * 255) >> 15);
            tOutL ^= xor_mask;
            tOutR ^= xor_mask;
        }

        // Run loop DC blockers to eliminate DC accumulation from bit scrambling
        tOutL = dc_loopL.process(tOutL);
        tOutR = dc_loopR.process(tOutR);

        // Warm High-Frequency Damping: increase damping filter from 16384 to 24000 as lofi_level rises
        int32_t damp = 16384 + ((lofi_level * 7616) >> 15);

        // Left Tank
        int32_t iL = (int32_t)mono + (((int32_t)decay * tOutR) >> 15);
        iL = soft_limit_q15(iL);
        int16_t sL = (int16_t)iL + (int16_t)((16384 * modL.read(modulated_size_scale)) >> 15);
        modL.write(soft_limit_q15((int32_t)iL - (int16_t)((16384 * sL) >> 15)));
        d1L.write(sL);
        sL = d1L.read(modulated_size_scale);
        lpL += (((int32_t)sL - lpL) * damp) >> 15;
        sL = (int16_t)lpL;
        sL = apTankL.process(sL, modulated_size_scale);
        d2L.write(sL);

        // Right Tank
        int32_t iR = (int32_t)mono + (((int32_t)decay * tOutL) >> 15);
        iR = soft_limit_q15(iR);
        int16_t sR = (int16_t)iR + (int16_t)((16384 * modR.read(modulated_size_scale)) >> 15);
        modR.write(soft_limit_q15((int32_t)iR - (int16_t)((16384 * sR) >> 15)));
        d1R.write(sR);
        sR = d1R.read(modulated_size_scale);
        lpR += (((int32_t)sR - lpR) * damp) >> 15;
        sR = (int16_t)lpR;
        sR = apTankR.process(sR, modulated_size_scale);
        d2R.write(sR);

        int32_t wetL = sL;
        int32_t wetR = sR;

        // Heavy Decimation glitch (only in the last 20% circuit-bent range)
        if (circuit_bent_level > 0) {
            int32_t dec_factor = 1 + ((circuit_bent_level * 15) >> 15); // up to 16x decimation
            decimate_phase++;
            if (decimate_phase >= dec_factor) {
                decimate_phase = 0;
                last_outL = (int16_t)wetL;
                last_outR = (int16_t)wetR;
            }
            wetL = last_outL;
            wetR = last_outR;

            // Apply 1-pole LPF to smooth out decimation steps
            int32_t dec_coef = 32768 / dec_factor;
            if (dec_coef > 32767) dec_coef = 32767;
            if (dec_coef < 1000) dec_coef = 1000;

            dec_lpL += (((int32_t)wetL - dec_lpL) * dec_coef) >> 15;
            dec_lpR += (((int32_t)wetR - dec_lpR) * dec_coef) >> 15;
            wetL = (int16_t)dec_lpL;
            wetR = (int16_t)dec_lpR;
        } else {
            dec_lpL = wetL;
            dec_lpR = wetR;
        }

        // Wet/Dry mix
        L = (int16_t)(((int32_t)L * (32767 - mix) + wetL * mix) >> 15);
        R = (int16_t)(((int32_t)R * (32767 - mix) + wetR * mix) >> 15);
    }
};

#endif // DSP_BLOCKS_H
