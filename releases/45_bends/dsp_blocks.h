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
#include "hardware/interp.h"

extern uint32_t rand_seed;

// ============================================================================
// Hardware Interpolator Setup (Core 1 only — must be called once from Core 1)
// INTERP0 → BLEND mode  : single-cycle linear interpolation for delay reads
// INTERP1 → CLAMP mode  : branch-free Q15 saturation (replaces saturate_q15)
// ============================================================================
inline void init_hardware_interp() {
    // INTERP0: blend mode (INTERP0 lane 0 only supports BLEND bit)
    interp_config c0 = interp_default_config();
    interp_config_set_blend(&c0, true);   // lane1 result = lerp(BASE0, BASE1, frac8)
    interp_config_set_signed(&c0, true);  // signed audio samples
    interp_set_config(interp0, 0, &c0);
    // Lane 1: default pass-through (blend reads from lane 0 automatically)
    interp_config c0l1 = interp_default_config();
    interp_config_set_signed(&c0l1, true);
    interp_set_config(interp0, 1, &c0l1);

    // INTERP1: clamp mode (INTERP1 lane 0 only supports CLAMP bit)
    interp_config c1 = interp_default_config();
    interp_config_set_clamp(&c1, true);
    interp_config_set_signed(&c1, true);
    interp_set_config(interp1, 0, &c1);
    interp1->base[0] = (uint32_t)(int32_t)(-32768); // lower clamp bound
    interp1->base[1] = (uint32_t)(int32_t)(32767);  // upper clamp bound
}

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

        // Phase increment (doubled for 24kHz)
        uint16_t phase_inc = (uint16_t)(1 + (eff_rate >> 10));
        lfo_phase += phase_inc;

        // Decode depth / feedback from unified knob
        int16_t depth    = 0;
        int32_t feedback = 0;
        bool destroy = false;
        if (depthFeedback < 16384) {
            depth    = (int16_t)(depthFeedback * 2); // 0 → 32767
            feedback = 0;
        } else {
            depth    = 32767;
            if (depthFeedback < 26214) {
                // Standard range (Y < 80%): feedback rises up to 60% (19660)
                // 26214 - 16384 = 9830. 19660 * 32768 / 9830 = 65536
                feedback = ((depthFeedback - 16384) * 65536) >> 15;
            } else {
                // Destruction range (80% to 100% knob): feedback rises up to 95% (31128)
                destroy = true;
                // 32767 - 26214 = 6553. (31128 - 19660) * 32768 / 6553 = 57343
                feedback = 19660 + (((depthFeedback - 26214) * 57343) >> 15);
            }
        }

        // Classic dual-phase triangle LFO (L = 0°, R = 180° for deep stereo spread)
        auto get_tri = [](uint16_t phase) -> int16_t {
            int32_t tmp = (phase < 32768) ? ((phase << 1) - 32768) : (32767 - ((phase - 32768) << 1));
            return (int16_t)tmp;
        };
        int16_t lfoL = get_tri(lfo_phase);
        int16_t lfoR = get_tri((uint16_t)(lfo_phase + 32768u));

        // Base delay: 180 samples (~7.5 ms @ 24 kHz). Max modulation range: ±28 samples (~1.1 ms @ 24 kHz).
        // This tightens the time window to achieve a lush, warm BBD chorus instead of a wide, seasick vibrato.
        int32_t depth_samples = (int32_t)depth * 28 >> 15;
        int32_t delay_L_q16 = (180 << 16) + (((int32_t)lfoL * depth_samples) << 1);
        int32_t delay_R_q16 = (180 << 16) + (((int32_t)lfoR * depth_samples) << 1);
        
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
            // int32_t safe: (y1-y0) in [-65535,65535], frac in [0,65535]; product fits int32
            return lerp_delay_q15(y0, y1, frac);
        };

        int32_t wp_q16 = (int32_t)write_ptr << 16;
        int16_t wetL = read_frac(delayL, wp_q16, delay_L_q16);
        int16_t wetR = read_frac(delayR, wp_q16, delay_R_q16);

        // BBD low-pass filter
        int32_t damp = 32767 - ((mainMix * 24000) >> 15);
        lp_stateL += ((int32_t)wetL - lp_stateL) * damp >> 15;
        lp_stateR += ((int32_t)wetR - lp_stateR) * damp >> 15;
        if (lp_stateL >  32767) lp_stateL =  32767;
        if (lp_stateL < -32768) lp_stateL = -32768;
        if (lp_stateR >  32767) lp_stateR =  32767;
        if (lp_stateR < -32768) lp_stateR = -32768;
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
            
            int16_t wrL = soft_limit_q15(fbL);
            int16_t wrR = soft_limit_q15(fbR);
            
            if (destroy) {
                // Inject XOR bit corruption into Chorus BBD line (scale with destruction intensity)
                uint16_t xor_mask = (uint16_t)((depthFeedback - 26214) >> 9); // up to ~12 bits of mask
                wrL ^= xor_mask;
                wrR ^= xor_mask;
            }
            
            delayL[write_ptr] = wrL;
            delayR[write_ptr] = wrR;
        }

        write_ptr = (write_ptr + 1) & 0x3FF;

        outL = split_mix_q15(inL, wetL, (int16_t)mainMix);
        outR = split_mix_q15(inR, wetR, (int16_t)mainMix);
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
    uint16_t trans_loop_ctr = 0;

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
        trans_loop_ctr = 0;
        
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
        vibe_lfo += 4; // doubled for 24kHz
        int16_t vibe_sine = lookup_sine(vibe_lfo); // [-32768, 32767]

        // ── 2. Series Degradation Chain based on Y Knob ──────────────────────
        int32_t Y = ringingXor + (cv2Corruption * 8);
        Y = clamp_i32(Y, 0, 32767);

        // Map Y to series engine levels
        // Fuzz level: rises from 0 to 32767 over the first 12000 units, stays at max, then fades to 0 above 24000 to save CPU during scramble
        int32_t fuzz_level = 0;
        if (Y < 12000) {
            fuzz_level = (Y * 178956) >> 16; // Y * 32767 / 12000
        } else if (Y < 24000) {
            fuzz_level = 32767;
        } else {
            fuzz_level = 32767 - ((Y - 24000) << 2);
            if (fuzz_level < 0) fuzz_level = 0;
        }

        // Bad Connection level: rises from 0 to 32767 between 12000 and 24000, then stays at max
        int32_t bad_conn_level = 0;
        if (Y >= 12000) {
            if (Y < 24000) {
                bad_conn_level = ((Y - 12000) * 174762) >> 16;
            } else {
                bad_conn_level = 32767;
            }
        }

        // Scramble level: rises from 0 to 32767 between 24000 and 32767
        int32_t scramble_level = 0;
        if (Y >= 24000) {
            scramble_level = (Y - 24000) << 2;
            if (scramble_level > 32767) scramble_level = 32767;
        }

        // Scale levels by Main knob (strength)
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

        // Apply vibe modulation — mod_factor in [28672, 36864]
        int32_t mod_factor = 32768 + (vibe_sine >> 3);
        fuzz_level     = clamp_i32(((int32_t)fuzz_level     * mod_factor) >> 15, 0, 32767);
        bad_conn_level = clamp_i32(((int32_t)bad_conn_level * mod_factor) >> 15, 0, 32767);
        scramble_level = clamp_i32(((int32_t)scramble_level * mod_factor) >> 15, 0, 32767);

        // We process the signal sequentially!
        int16_t sigL = inL;
        int16_t sigR = inR;

        // ── Stage 1: Warm Fuzz ───────────────────────────────────────────────
        if (fuzz_level > 0) {
            int16_t compL = compress_expand_mulaw_variable(sigL, fuzz_level);
            int16_t compR = compress_expand_mulaw_variable(sigR, fuzz_level);

            // Continuous fractional bitcrusher for fuzz
            // fuzz_level max = 32767; 32767^2 = 1,073M < 2,147M INT32_MAX — safe in int32
            int32_t fuzz_sq = ((int32_t)fuzz_level * fuzz_level) >> 15;
            int32_t shift_q15 = (fuzz_sq * 10);
            int32_t int_shift = shift_q15 >> 15;
            int32_t frac_shift = shift_q15 & 0x7FFF;
            if (int_shift > 0 || frac_shift > 0) {
                // Left channel symmetric bitcrushing
                int32_t signL = compL < 0 ? -1 : 1;
                int32_t absL = compL < 0 ? -compL : compL;
                int32_t q1L = (absL >> int_shift) << int_shift;
                int32_t q2L = (absL >> (int_shift + 1)) << (int_shift + 1);
                compL = signL * lerp_q15(q1L, q2L, frac_shift);

                // Right channel symmetric bitcrushing
                int32_t signR = compR < 0 ? -1 : 1;
                int32_t absR = compR < 0 ? -compR : compR;
                int32_t q1R = (absR >> int_shift) << int_shift;
                int32_t q2R = (absR >> (int_shift + 1)) << (int_shift + 1);
                compR = signR * lerp_q15(q1R, q2R, frac_shift);
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

            sigL = lerp_q15((int16_t)compL, saturate_q15(bp_cL), 12000);
            sigR = lerp_q15((int16_t)compR, saturate_q15(bp_cR), 12000);
        }

        // ── Stage 2: Bad Connection (Packet Drops & Stutter Repetition) ──────
        if (bad_conn_level > 0) {
            if (!trans_dropped) {
                trans_historyL[trans_wr] = sigL;
                trans_historyR[trans_wr] = sigR;
            }

            if (trans_frame_ctr >= trans_frame_size) {
                trans_frame_ctr = 0;
                trans_frame_size = 240 + (((fast_rand(rand_seed) & 0xFFFF) * 720) >> 16);

                int32_t drop_thresh = 300 + ((bad_conn_level * 14000) >> 15);
                uint32_t roll = fast_rand(rand_seed) & 0x7FFF;
                trans_dropped = ((int32_t)roll < drop_thresh);

                if (trans_dropped) {
                    trans_loop_size = 64 + ((bad_conn_level * 192) >> 15);
                    trans_drop_rd = (trans_wr - trans_loop_size) & 0xFF;
                }
            }
            trans_frame_ctr++;

            if (trans_dropped) {
                sigL = trans_historyL[trans_drop_rd];
                sigR = trans_historyR[trans_drop_rd];
                
                trans_loop_ctr++;
                if (trans_loop_ctr >= trans_loop_size) {
                    trans_loop_ctr = 0;
                    trans_drop_rd = (trans_wr - trans_loop_size) & 0xFF;
                } else {
                    trans_drop_rd = (trans_drop_rd + 1) & 0xFF;
                }
            } else {
                trans_wr = (trans_wr + 1) & 0xFF;
                trans_loop_ctr = 0;
            }

            int32_t scramble_prob = (bad_conn_level * 4000) >> 15; 
            if ((int32_t)(fast_rand(rand_seed) & 0x7FFF) < scramble_prob) {
                uint32_t limit = 1 + (bad_conn_level >> 11);
                // limit max = 1 + (32767 >> 11) = 17; use >> 27 instead of >> 32 (same bit-count effect at smaller scale)
                uint16_t mask = (uint16_t)((fast_rand(rand_seed) >> (32 - 4)) & (limit - 1));
                sigL ^= mask;
                sigR ^= mask;
            }
        }

        // ── Stage 3: Circuit-Bent Scramble ──
        if (scramble_level > 0) {
            carrier_phase += 150 + (scramble_level >> 5);
            int16_t carrier = (carrier_phase & 0x8000) ? 0xFFFF : 0x0000;

            int16_t modL = sigL ^ carrier;
            int16_t modR = sigR ^ carrier;

            // Continuous fractional bitcrusher for scramble
            int32_t scram_sq = ((int32_t)scramble_level * scramble_level) >> 15;
            int32_t shift_q15 = (scram_sq * 10);
            int32_t int_shift = shift_q15 >> 15;
            int32_t frac_shift = shift_q15 & 0x7FFF;

            int16_t q1L = (modL >> int_shift) << int_shift;
            int16_t q2L = (modL >> (int_shift + 1)) << (int_shift + 1);
            modL = lerp_q15(q1L, q2L, frac_shift);

            int16_t q1R = (modR >> int_shift) << int_shift;
            int16_t q2R = (modR >> (int_shift + 1)) << (int_shift + 1);
            modR = lerp_q15(q1R, q2R, frac_shift);

            // Continuous wavefolder gain: scale from 1.0x to 2.0x based on scramble_level
            int32_t fold_gain = 32768 + scramble_level;
            int32_t foldL = ((int32_t)modL * fold_gain) >> 15;
            int32_t foldR = ((int32_t)modR * fold_gain) >> 15;

            if (foldL > 32767) foldL = 32767 - (foldL - 32767);
            if (foldL < -32768) foldL = -32768 - (foldL + 32768);
            if (foldR > 32767) foldR = 32767 - (foldR - 32767);
            if (foldR < -32768) foldR = -32768 - (foldR + 32768);

            // Scale down by 75% to balance RMS volume boost
            int16_t foldL_sat = saturate_q15((foldL * 24576) >> 15);
            int16_t foldR_sat = saturate_q15((foldR * 24576) >> 15);

            int32_t scram_coef = 24000 - ((scramble_level * 14000) >> 15);
            lp_newL += (((int32_t)foldL_sat - lp_newL) * scram_coef) >> 15;
            lp_newR += (((int32_t)foldR_sat - lp_newR) * scram_coef) >> 15;
            if (lp_newL >  32767) lp_newL =  32767;
            if (lp_newL < -32768) lp_newL = -32768;
            if (lp_newR >  32767) lp_newR =  32767;
            if (lp_newR < -32768) lp_newR = -32768;
            sigL = (int16_t)lp_newL;
            sigR = (int16_t)lp_newR;
        } else {
            lp_newL = sigL;
            lp_newR = sigR;
        }

        int16_t wetL = sigL;
        int16_t wetR = sigR;

        // ── 3. VCA Compression & Analog Saturation (scaled by Strength) ───────
        int32_t absL = wetL < 0 ? -wetL : wetL;
        int32_t absR = wetR < 0 ? -wetR : wetR;
        int32_t peak = absL > absR ? absL : absR;
        if (peak > 32767) peak = 32767;

        // Envelope follower (Vibe LFO creates breathing release fluctuations)
        int32_t attack_shift = 4; // sped up for 24kHz
        int32_t release_shift = 10 + (vibe_sine >> 13); // sped up for 24kHz
        if (peak > env) env += (peak - env) >> attack_shift;
        else env += (peak - env) >> release_shift;

        // Threshold matched to 6dB input headroom scaling
        int32_t thresh = 14000 - ((strength * 12500) >> 15);
        int32_t slope = (strength * 27000) >> 15;

        int32_t gain_coef = 32768;
        if (env > thresh) {
            int32_t overshoot = env - thresh;
            int32_t gain_reduction = ((int32_t)overshoot * slope) >> 15;
            gain_coef = 32768 - gain_reduction;
            if (gain_coef < 4096) gain_coef = 4096; // limit GR to -18dB
        }

        // Clamp to int16 range before makeup so the multiply stays 32-bit safe
        // Drive gain: max 1.35x (44236/32768)
        int32_t drive_gain = 32768 + ((strength * 11468) >> 15);
        int32_t compLi = soft_limit_q15(((int32_t)wetL * drive_gain) >> 15);
        int32_t compRi = soft_limit_q15(((int32_t)wetR * drive_gain) >> 15);
        compLi = (compLi * gain_coef) >> 15;
        compRi = (compRi * gain_coef) >> 15;

        // Makeup gain: max 1.35x (44236/32768)
        // Keeps total block gain at max 1.8x (drive * makeup) instead of 3.6x!
        int32_t makeup_gain = 32768 + ((strength * 11468) >> 15);
        compLi = (compLi * makeup_gain) >> 15;
        compRi = (compRi * makeup_gain) >> 15;

        wetL = tape_saturate(soft_limit_q15(compLi));
        wetR = tape_saturate(soft_limit_q15(compRi));

        // ── 4. Downsampling (from X Knob) ─────────────────────────────────────
        if (downsample > 150) {
            if (dec_ctr >= current_dec_step) {
                decL    = wetL;
                decR    = wetR;
                dec_ctr = 0;

                // Quadratic scaling for ds_sq: fits 32-bit (32767^2 = 1,073,676,289 < INT32_MAX)
                uint32_t ds_sq = ((uint32_t)downsample * downsample) >> 15;
                uint32_t base_step = 1 + ((ds_sq * 23) >> 15);
                uint32_t jitter_range = ((strength >> 11) * (downsample >> 11)) >> 4;
                if (jitter_range < 2) jitter_range = 2;
                uint32_t jitter = (((fast_rand(rand_seed) >> 16) * jitter_range) >> 16);
                current_dec_step = base_step + jitter;
                if (current_dec_step > 32) current_dec_step = 32;
            } else {
                wetL = decL;
                wetR = decR;
            }
            dec_ctr++;

            // Continuous LPF to smooth out downsampling steps (scaled to preserve crunch/aliasing highs, min cutoff ~700Hz)
            // LUT replaces 32768 / (1 + (current_dec_step >> 2)) — no division
            // dec_idx = 1 + (step >> 2) ranges from 1 to 9
            static const int32_t dec_coef_lut[10] = {
                32767, 32767, 16384, 10922, 8192, 6553, 5461, 4681, 4096, 3640
            };
            int32_t dec_idx = 1 + (current_dec_step >> 2);
            if (dec_idx > 9) dec_idx = 9;
            int32_t dec_coef = dec_coef_lut[dec_idx];

            dec_lpL += (((int32_t)wetL - dec_lpL) * dec_coef) >> 15;
            dec_lpR += (((int32_t)wetR - dec_lpR) * dec_coef) >> 15;
            if (dec_lpL >  32767) dec_lpL =  32767;
            if (dec_lpL < -32768) dec_lpL = -32768;
            if (dec_lpR >  32767) dec_lpR =  32767;
            if (dec_lpR < -32768) dec_lpR = -32768;
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
                    sputter_timer = 5 + (((fast_rand(rand_seed) & 0xFFFF) * 115) >> 16);
                    sputter_active = ((fast_rand(rand_seed) & 0x7FFF) < 9830);
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
    // Stereo ring buffer — 20480 samples ≈ 853 ms @ 24 kHz
    int16_t  bufL[20480];
    int16_t  bufR[20480];
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

    DCBlocker dcL;
    DCBlocker dcR;

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
        dcL.init();
        dcR.init();
    }

    void process(int16_t inL, int16_t &outL, int16_t inR, int16_t &outR,
                 int32_t mainMix, int32_t time, int32_t feedback,
                 bool freeze, int32_t cv1Warp, int32_t cv2Corruption, int32_t globalNoiseScale = 16384)
    {
        // ── PT2399 Slow-Clock Delay Decimation ──
        uint32_t clk_inc = 65536;
        if (time > 16384) {
            int32_t diff = time - 16384;
            // Division-free scaling (62260 / 16383 ≈ 124528 / 32768)
            // diff max = 16383; 124528 * 16383 = 2,039M < 2,147M INT32_MAX — safe
            clk_inc = 65536 - ((diff * 124528) >> 15);
        }

        // Calculate filter coefficient for continuous reconstruction filter
        int32_t filter_coef = 32767;
        if (clk_inc < 65536) {
            // Division-free scaling (29767 / 62260 ≈ 15666 / 32768)
            filter_coef = 3000 + (((int32_t)(clk_inc - 3276) * 15666) >> 15);
        }

        clk_phase += clk_inc;
        if (clk_phase < 65536) {
            // Run LPF continuously even on early-return samples
            lp_outL += (((int32_t)last_outL - lp_outL) * filter_coef) >> 15;
            lp_outR += (((int32_t)last_outR - lp_outR) * filter_coef) >> 15;

            outL = split_mix_q15(inL, (int16_t)lp_outL, (int16_t)mainMix);
            outR = split_mix_q15(inR, (int16_t)lp_outR, (int16_t)mainMix);
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
                if (wr >= 20480) wr = 0;
            }
        int32_t mapped_time = 24 + ((time * 19700) >> 15);
            int32_t target_t = mapped_time + (cv1Warp * 4);
            target_t = clamp_i32(target_t, 24, 20350);
            IIR_SMOOTH(smooth_t, target_t, 12);
            return;
        }

        // Slew delay time with CV1 pitch-warp
        int32_t mapped_time = 24 + ((time * 19700) >> 15);
        int32_t target_t = mapped_time + (cv1Warp * 4);
        target_t = clamp_i32(target_t, 24, 20350);

        IIR_SMOOTH(smooth_t, target_t, 12);

        // Increment wow & flutter LFO (~2.2 Hz) (doubled for 24kHz)
        flutter_phase += 6;
        int16_t lfo = lookup_sine_fast(flutter_phase);
        int32_t flutter = (lfo * 8) >> 15; // up to ±8 samples of flutter

        // Fractional-sample read with linear interpolation (optimised: division-free)
        auto read_stereo = [&](int32_t delay_q16,
                               int16_t &rL, int16_t &rR) {
            int32_t rp_i = (int32_t)wr - (delay_q16 >> 16);
            if (rp_i < 0) {
                rp_i += 20480;
                if (rp_i < 0) rp_i = 0;
            } else if (rp_i >= 20480) {
                rp_i -= 20480;
                if (rp_i >= 20480) rp_i = 0;
            }
            int32_t rp_n = rp_i + 1;
            if (rp_n >= 20480) rp_n = 0;
            uint16_t frac = (uint16_t)(delay_q16 & 0xFFFF);
            {
                int16_t y0 = bufL[rp_i], y1 = bufL[rp_n];
                rL = lerp_delay_q15(y0, y1, frac);
            }
            {
                int16_t y0 = bufR[rp_i], y1 = bufR[rp_n];
                rR = lerp_delay_q15(y0, y1, frac);
            }
        };

        // Apply wow & flutter directly to panned tap read pointers
        int32_t t1_q16 = (smooth_t + flutter) << 16;
        int32_t t2_q16 = ((smooth_t + flutter) * 3) << 14;
        int32_t t3_q16 = (smooth_t + flutter) * 40503;

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
            // Direct loop feedback (L->L, R->R) for stable Karplus-Strong string synthesis
            int32_t feedL = (int32_t)inL + (((int32_t)w1L * feedback) >> 15);
            int32_t feedR = (int32_t)inR + (((int32_t)w1R * feedback) >> 15);
            
            // Dynamic 1-pole damping: roll off high frequencies more aggressively at short times
            // to stabilize extreme high-pitched ringing and model acoustic losses
            int32_t damp_coef = 15000; // standard damping
            if (smooth_t < 500) {
                // scale damping down (more HF roll-off) for short delay times
                damp_coef = 6000 + ((smooth_t * 9000) / 500);
            }
            lp_feedback_L += ((feedL - lp_feedback_L) * damp_coef) >> 15;
            lp_feedback_R += ((feedR - lp_feedback_R) * damp_coef) >> 15;

            // DC blocking + soft-clipper to prevent digital overflow blowout
            int16_t wrL = dcL.process(soft_limit_q15(lp_feedback_L));
            int16_t wrR = dcR.process(soft_limit_q15(lp_feedback_R));
            
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
            if (wr >= 20480) wr = 0;
        }

        last_outL = mixL;
        last_outR = mixR;

        // Run reconstruction filter on the wet output
        lp_outL += (((int32_t)last_outL - lp_outL) * filter_coef) >> 15;
        lp_outR += (((int32_t)last_outR - lp_outR) * filter_coef) >> 15;

        outL = split_mix_q15(inL, (int16_t)lp_outL, (int16_t)mainMix);
        outR = split_mix_q15(inR, (int16_t)lp_outR, (int16_t)mainMix);
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
// Fast G.711 mu-law encoder
inline uint8_t encode_mulaw(int16_t sample) {
    int32_t x = sample;
    int32_t sign = (x < 0) ? 0x80 : 0x00;
    if (x < 0) x = -x;
    x += 132;
    if (x > 32767) x = 32767;
    int32_t exponent = 31 - __builtin_clz(x);
    if (exponent < 7) {
        return (uint8_t)(sign | ((x - 132) >> 3));
    }
    int32_t mantissa = (x >> (exponent - 4)) & 0x0F;
    return (uint8_t)(sign | ((exponent - 7) << 4) | mantissa);
}

extern int16_t mulaw_decode_table[256];

// G.711 mu-law decoder optimized to a single memory lookup
inline int16_t decode_mulaw(uint8_t u_val) {
    return mulaw_decode_table[u_val];
}

// Zoned speed determination helper. Placed in FLASH (not RAM) to save memory,
// since it is only called on grain boundaries/initialization, not per-sample.
__attribute__((noinline)) int32_t determine_speed_zoned(int32_t sq, int32_t cv2_corr, uint32_t &seed, uint8_t arp_step, int32_t loop_len) {
    int32_t base_speed;

    // If the loop length is short, keep it clean (stuck CD/tape scrub style) rather than chaotic metallic buzzes
    bool force_clean = (loop_len < 768);

    if (force_clean) {
        // At short loop lengths, limit to 1x forward or 1x reverse
        if (sq < 6554) {
            base_speed = 65536; // 1x fwd
        } else {
            base_speed = (fast_rand(seed) & 1) ? 65536 : -65536; // 1x fwd or 1x rev
        }
    } else {
        if (sq < 6554) {
            // Zone 0: always 1x forward — pure rhythmic stutter, no pitch change
            base_speed = 65536;
        } else if (sq < 13107) {
            // Zone 1: always 1x forward or 1x reverse (direction changes, no pitch shifts)
            base_speed = (fast_rand(seed) & 1) ? 65536 : -65536;
        } else if (sq < 19661) {
            // Zone 2: tonal octave family {0.5x, 1x, 2x, -0.5x, -1x}
            uint32_t choice = ((fast_rand(seed) & 0xFFFF) * 5) >> 16;
            switch (choice) {
                case 0:  base_speed =  65536;  break; // 1x fwd
                case 1:  base_speed =  131072; break; // 2x fwd
                case 2:  base_speed =  32768;  break; // 0.5x fwd
                case 3:  base_speed = -65536;  break; // 1x rev
                default: base_speed = -32768;  break; // 0.5x rev
            }
        } else if (sq < 26214) {
            // Zone 3: Melodic Pentatonic Arpeggiator (advances step on every repeat)
            static const int32_t arp_seq[8] = {65536, 81920, 98304, 131072, 32768, 49152, -65536, -32768};
            base_speed = arp_seq[arp_step & 7];
        } else {
            // Zone 4: full chaos — all 6 speeds equally weighted
            uint32_t c = ((fast_rand(seed) & 0xFFFF) * 6) >> 16; // 0..5
            switch (c) {
                case 0:  base_speed =  65536;  break; // 1x fwd
                case 1:  base_speed =  131072; break; // 2x fwd
                case 2:  base_speed =  32768;  break; // 0.5x fwd
                case 3:  base_speed = -65536;  break; // 1x rev
                case 4:  base_speed = -131072; break; // 2x rev
                default: base_speed = -32768;  break; // 0.5x rev
            }
        }
    }

    // Quantize CV2 to ±12 chromatic semitones using a lookup table
    // CV2 ranges -2048 to 2047. 2048 / 170 ≈ 12
    static const int32_t chromatic_scale_q16[25] = {
        32768, 34716, 36780, 38967, 41284, 43739, 46340, 49096, 52016, 55109, 58386, 61858,
        65536,
        69433, 73562, 77936, 82570, 87480, 92682, 98193, 104032, 110218, 116772, 123715, 131072
    };
    int32_t semitones = cv2_corr / 170;
    if (semitones < -12) semitones = -12;
    if (semitones > 12) semitones = 12;
    int32_t scale_factor = chromatic_scale_q16[semitones + 12];

    int32_t speed = ((int64_t)base_speed * scale_factor) >> 16;
    if (speed == 0) speed = 3277;
    return speed;
}

struct GlitcherBlock {
    uint8_t  bufL[32768];
    uint8_t  bufR[32768];
    uint16_t wr = 0;

    bool     active     = false;
    uint16_t freeze_wr  = 0;       // write pointer at moment of freeze

    // Playback pointer (Q16: integer + 16-bit fraction relative to loop_start)
    int64_t  rd_q16     = 0;

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
    
    // Active offset of loop_start relative to freeze_wr
    int32_t  active_offset = 0;

    // Clustering and probability warping states
    int32_t  cluster_state = 16384;
    uint16_t cluster_timer = 0;

    // Evolve freeze states
    uint32_t freeze_evolve_ctr = 0;
    bool     evolve_active = false;
    int32_t  evolve_samples_left = 0;

    uint8_t  arpeggio_step = 0;
    bool     trig_out1 = false;
    bool     trig_out2 = false;
    uint8_t  current_g711_sample = 128;

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
        active_offset = 0;
        cluster_state = 16384;
        cluster_timer = 0;
        freeze_evolve_ctr = 0;
        evolve_active = false;
        evolve_samples_left = 0;
        arpeggio_step = 0;
        trig_out1 = false;
        trig_out2 = false;
        current_g711_sample = 128;
    }

    void process(int16_t inL, int16_t &outL, int16_t inR, int16_t &outR,
                 int32_t mainProb, int32_t size, int32_t speedQuant,
                 bool glitchInjector, bool freezeGate, int32_t cv1Warp, int32_t cv2Corruption,
                 uint32_t &rand_seed, int32_t scrubOffset, int32_t glitchFeedback, int32_t globalNoiseScale,
                 bool pulse1_live, bool p1_rising, bool p1_gate,
                 bool pulse2_live, bool p2_rising, bool p2_gate,
                 uint32_t clk_period_samples)
    {
        bool is_clock_sync = pulse1_live && (clk_period_samples > 240);
        bool eff_glitchInjector = glitchInjector;
        if (is_clock_sync) {
            eff_glitchInjector = false;
        }

        bool want_active = eff_glitchInjector || freezeGate;
        if (pulse1_live) {
            want_active = p1_gate || freezeGate;
        }
        int32_t speed_q16 = 65536;

        if (mainProb < 50 && !want_active && !active && dry_fade_ctr == 0) {
            outL = inL;
            outR = inR;
            bufL[wr] = encode_mulaw(inL);
            bufR[wr] = encode_mulaw(inR);
            wr = (wr + 1) & 0x7FFF;
            return;
        }
        if (is_clock_sync && mainProb < 50 && !active && dry_fade_ctr == 0) {
            outL = inL;
            outR = inR;
            bufL[wr] = encode_mulaw(inL);
            bufR[wr] = encode_mulaw(inR);
            wr = (wr + 1) & 0x7FFF;
            return;
        }

        // Update slow-moving cluster state (cutoff ~3Hz at 24kHz)
        cluster_timer++;
        if (cluster_timer >= 256) {
            cluster_timer = 0;
            cluster_state += (((int32_t)(fast_rand(rand_seed) & 0x7FFF)) - cluster_state) >> 5;
        }

        // Warp input probability curve quadratically for finer control on the left side of the knob
        int32_t warpedProb = (mainProb * mainProb) >> 15;

        // Modulate probability with cluster state, scaling down depth near 0% and 100% knob
        int32_t mod_depth = (mainProb * (32767 - mainProb)) >> 14;
        int32_t cluster_mod = ((cluster_state - 16384) * mod_depth) >> 15;
        int32_t finalProb = warpedProb + cluster_mod;
        if (finalProb < 0) finalProb = 0;
        if (finalProb > 32767) finalProb = 32767;

        // ── FREEZE MODE ──────────────────────────────────────────────────────
        if (freezeGate || (pulse2_live && p2_gate)) {
            // 1. Lock recording and initialize freeze on transition
            if (!active) {
                active = true;
                freeze_wr = wr;
                
                int32_t init_len = 128 + size;
                if (pulse1_live && clk_period_samples > 240) {
                    if (size < 6000) {
                        init_len = clk_period_samples / 16;
                    } else if (size < 12000) {
                        init_len = clk_period_samples / 8;
                    } else if (size < 18000) {
                        init_len = clk_period_samples / 4;
                    } else if (size < 24000) {
                        init_len = clk_period_samples / 2;
                    } else {
                        init_len = clk_period_samples;
                    }
                }
                current_loop_len = clamp_i32(init_len, 128, 32760);

                int32_t init_offset = ((32767 - scrubOffset) * 32760) >> 15;
                if (pulse1_live && clk_period_samples > 240) {
                    int32_t step_size = clk_period_samples / 16;
                    if (step_size < 1) step_size = 1;
                    int32_t total_steps = 32760 / step_size;
                    if (total_steps < 1) total_steps = 1;
                    int32_t step = (init_offset + (step_size / 2)) / step_size;
                    init_offset = step * step_size;
                }
                active_offset = clamp_i32(init_offset, 0, 32760);

                rd_q16 = 0;
                xfade_ctr = 0;
                dry_fade_ctr = 0;
                sample_ctr = 0;
                
                int32_t cur_xfade = current_loop_len < 512 ? (current_loop_len >> 1) : 256;
                if (cur_xfade < 4) cur_xfade = 4;
                onset_fade_len = cur_xfade;
                onset_fade_ctr = cur_xfade;
                onset_fade_step = (32767 << 15) / cur_xfade;
                onset_fade_phase = 0;
            }

            int32_t loop_size = 128 + size;
            if (pulse1_live && clk_period_samples > 240) {
                if (size < 6000) {
                    loop_size = clk_period_samples / 16;
                } else if (size < 12000) {
                    loop_size = clk_period_samples / 8;
                } else if (size < 18000) {
                    loop_size = clk_period_samples / 4;
                } else if (size < 24000) {
                    loop_size = clk_period_samples / 2;
                } else {
                    loop_size = clk_period_samples;
                }
            }
            loop_size = clamp_i32(loop_size, 128, 32760);
            int32_t cur_xfade = loop_size < 512 ? (loop_size >> 1) : 256;
            if (cur_xfade < 4) cur_xfade = 4;

            // CV1 scrubs loop position when frozen:
            int32_t cv1_offset = cv1Warp * 6; // sweeps ~±12288 samples
            int32_t raw_offset = ((32767 - scrubOffset) * 32760) >> 15;
            int32_t target_offset = raw_offset + cv1_offset;
            if (pulse1_live && clk_period_samples > 240) {
                // Snap scrub position to 1/16 note steps of the clock
                int32_t step_size = clk_period_samples / 16;
                if (step_size < 1) step_size = 1;
                int32_t total_steps = 32760 / step_size;
                if (total_steps < 1) total_steps = 1;
                int32_t step = (target_offset + (step_size / 2)) / step_size;
                target_offset = step * step_size;
            }
            target_offset = clamp_i32(target_offset, 0, 32760);
            
            // Linear speed mapping: [0..32767] -> [0..131068] Q16 (0x to 2.0x, center is 1.0x)
            int32_t base_speed = speedQuant << 2;
            speed_q16 = determine_speed_zoned(speedQuant, cv2Corruption, rand_seed, arpeggio_step, current_loop_len);
            if (speed_q16 < 0) speed_q16 = 0;

            int32_t loop_start = (((int32_t)freeze_wr - active_offset) & 0x7FFF) << 16;

            rd_q16 += speed_q16;
            sample_ctr++;

            // Natural boundary check
            bool crossed = (rd_q16 >= (int64_t)(current_loop_len << 16));
            if (pulse1_live && p1_rising) {
                crossed = true;
            }

            if (crossed) {
                trig_out1 = true; // Output loop sync pulse

                // Spawn next grain repeat at updated position/length targets
                xfade_rd = loop_start + rd_q16;
                rd_q16 = 0;
                xfade_len = cur_xfade;
                xfade_ctr = cur_xfade;
                xfade_step = (32767 << 15) / xfade_len;
                xfade_phase = 0;

                current_loop_len = loop_size;
                
                // Evolve freeze logic based on Main knob (scrubOffset):
                if (scrubOffset < 1500) {
                    if (!evolve_active) {
                        freeze_evolve_ctr++;
                        if (freeze_evolve_ctr >= 8) {
                            freeze_evolve_ctr = 0;
                            evolve_active = true;
                            evolve_samples_left = current_loop_len;
                        }
                    }
                } else if (scrubOffset > 31200) {
                    if (!evolve_active) {
                        uint32_t roll = fast_rand(rand_seed) & 0x7FFF;
                        if (roll < 4096) {
                            evolve_active = true;
                            int32_t max_len = current_loop_len - 128;
                            if (max_len < 1) max_len = 1;
                            evolve_samples_left = 128 + (((fast_rand(rand_seed) & 0x7FFF) * max_len) >> 15);
                        }
                    }
                } else {
                    freeze_evolve_ctr = 0;
                    evolve_active = false;
                }
                
                // Update active_offset with a 16-sample hysteresis filter for stability
                int32_t diff = target_offset - active_offset;
                if (diff < 0) diff = -diff;
                if (diff > 16) {
                    active_offset = target_offset;
                }
                
                loop_start = (((int32_t)freeze_wr - active_offset) & 0x7FFF) << 16;
            }

            int16_t sL = 0, sR = 0;
            auto read_buf = [&](int32_t ptr, int16_t &valL, int16_t &valR) {
                int32_t  idx  = (ptr >> 16) & 0x7FFF;
                int32_t  nxt  = (idx + 1)   & 0x7FFF;
                uint16_t frac = (uint16_t)(ptr & 0xFFFF);
                int16_t y0L = decode_mulaw(bufL[idx]), y1L = decode_mulaw(bufL[nxt]);
                valL = lerp_delay_q15(y0L, y1L, frac);
                int16_t y0R = decode_mulaw(bufR[idx]), y1R = decode_mulaw(bufR[nxt]);
                valR = lerp_delay_q15(y0R, y1R, frac);
            };

            read_buf(loop_start + rd_q16, sL, sR);
            current_g711_sample = bufL[((int32_t)(loop_start + rd_q16) >> 16) & 0x7FFF];

            if (evolve_active) {
                int32_t idx = ((loop_start + rd_q16) >> 16) & 0x7FFF;
                bufL[idx] = encode_mulaw(inL);
                bufR[idx] = encode_mulaw(inR);
                evolve_samples_left--;
                if (evolve_samples_left <= 0) {
                    evolve_active = false;
                }
            }

            // Smooth loop boundary crossfade
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

            // Smooth onset crossfade
            if (onset_fade_ctr > 0) {
                onset_fade_phase += onset_fade_step;
                int32_t val = onset_fade_phase >> 15;
                if (val > 32767) val = 32767;
                int16_t t = (int16_t)val;
                sL = lerp_q15(inL, sL, t);
                sR = lerp_q15(inR, sR, t);
                onset_fade_ctr--;
            }

            // Glitcher Feedback Loop
            if (glitchFeedback > 0) {
                int32_t idx = ((loop_start + rd_q16) >> 16) & 0x7FFF;
                int32_t scaled_fb = (glitchFeedback * 29491) >> 15;
                int16_t oldL = decode_mulaw(bufL[idx]);
                int16_t oldR = decode_mulaw(bufR[idx]);
                int16_t newL = soft_limit_q15(((int32_t)oldL * (32768 - scaled_fb) + (int32_t)sL * scaled_fb) >> 15);
                int16_t newR = soft_limit_q15(((int32_t)oldR * (32768 - scaled_fb) + (int32_t)sR * scaled_fb) >> 15);
                bufL[idx] = encode_mulaw(newL);
                bufR[idx] = encode_mulaw(newR);
            }

            outL = lerp_q15(inL, sL, (int16_t)mainProb);
            outR = lerp_q15(inR, sR, (int16_t)mainProb);
            return;
        }

        // ── NORMAL GLITCH MODE ───────────────────────────────────────────────
        else {
            int32_t norm_loop_size = 512;
            if (pulse1_live && clk_period_samples > 240) {
                // Clock-synced division based on Size knob
                if (size < 6000) {
                    norm_loop_size = clk_period_samples / 16;
                } else if (size < 12000) {
                    norm_loop_size = clk_period_samples / 8;
                } else if (size < 18000) {
                    norm_loop_size = clk_period_samples / 4;
                } else if (size < 24000) {
                    norm_loop_size = clk_period_samples / 2;
                } else {
                    norm_loop_size = clk_period_samples;
                }
                if (norm_loop_size < 128) norm_loop_size = 128;
                if (norm_loop_size > 16384) norm_loop_size = 16384;
            } else {
                if (cv1Warp == 0) {
                    static const int32_t size_lut[8] = {128, 256, 512, 1024, 2048, 4096, 8192, 16384};
                    int32_t step = (size * 8) >> 15;
                    if (step < 0) step = 0;
                    if (step > 7) step = 7;
                    norm_loop_size = size_lut[step];
                } else {
                    int32_t base_size = 128 + (size >> 1);
                    norm_loop_size = base_size + (cv1Warp * 4);
                }
            }
            norm_loop_size = clamp_i32(norm_loop_size, 128, 16384);

            int32_t cur_xfade = norm_loop_size < 512 ? (norm_loop_size >> 1) : 256;
            if (cur_xfade < 4) cur_xfade = 4;

            if (!active) {
                bufL[wr] = encode_mulaw(inL);
                bufR[wr] = encode_mulaw(inR);

                bool trigger = false;
                if (pulse1_live) {
                    trigger = (p1_rising && (((fast_rand(rand_seed) & 0x7FFF) < (uint32_t)finalProb) || eff_glitchInjector)) || eff_glitchInjector;
                } else {
                    trigger = ((wr & (uint16_t)(norm_loop_size - 1)) == 0 && (fast_rand(rand_seed) & 0x7FFF) < (uint32_t)finalProb) || glitchInjector;
                }

                if (trigger) {
                    active = true;
                    freeze_wr = wr;
                    
                    int32_t final_size = norm_loop_size;
                    int32_t mid_amount = 16384 - abs(mainProb - 16384);
                    if (!pulse1_live) {
                        if (cv1Warp == 0) {
                            if ((int32_t)(fast_rand(rand_seed) & 0x7FFF) < mid_amount) {
                                int32_t step = (size * 8) >> 15;
                                uint32_t r = fast_rand(rand_seed) & 3;
                                if (r == 3) r = 0;
                                int32_t offset = (int32_t)r - 1;
                                step += offset;
                                if (step < 0) step = 0;
                                if (step > 7) step = 7;
                                static const int32_t size_lut[8] = {128, 256, 512, 1024, 2048, 4096, 8192, 16384};
                                final_size = size_lut[step];
                            }
                        } else {
                            if ((int32_t)(fast_rand(rand_seed) & 0x7FFF) < mid_amount) {
                                int32_t range = final_size >> 2;
                                if (range > 0) {
                                    int32_t double_range = range * 2;
                                    if (double_range < 1) double_range = 1;
                                    int32_t offset = (((int32_t)(fast_rand(rand_seed) & 0x7FFF) * double_range) >> 15) - range;
                                    final_size += offset;
                                }
                            }
                        }
                    }
                    if (speedQuant >= 19661) {
                        int32_t jitter_range = final_size >> 2;
                        if (jitter_range > 0) {
                            int32_t offset = (((int32_t)(fast_rand(rand_seed) & 0x7FFF) * (jitter_range * 2)) >> 15) - jitter_range;
                            final_size += offset;
                        }
                    }
                    current_loop_len = clamp_i32(final_size, 128, 16384);
                    
                    arpeggio_step = 0;
                    current_speed_q16 = determine_speed_zoned(speedQuant, cv2Corruption, rand_seed, arpeggio_step, current_loop_len);
                    speed_q16 = current_speed_q16;
                    
                    rd_q16 = (speed_q16 >= 0) ? 0 : (int64_t)(current_loop_len << 16);
                    xfade_ctr = 0;
                    dry_fade_ctr = 0;
                    sample_ctr = 0;
                    
                    onset_fade_len = cur_xfade;
                    onset_fade_ctr = cur_xfade;
                    onset_fade_step = (32767 << 15) / cur_xfade;
                    onset_fade_phase = 0;
                }
                wr = (wr + 1) & 0x7FFF;
            }

            if (active) {
                auto read_buf = [&](int32_t ptr, int16_t &sL, int16_t &sR) {
                    int32_t  idx  = (ptr >> 16) & 0x7FFF;
                    int32_t  nxt  = (idx + 1)   & 0x7FFF;
                    uint16_t frac = (uint16_t)(ptr & 0xFFFF);
                    int16_t y0L = decode_mulaw(bufL[idx]), y1L = decode_mulaw(bufL[nxt]);
                    sL = lerp_delay_q15(y0L, y1L, frac);
                    int16_t y0R = decode_mulaw(bufR[idx]), y1R = decode_mulaw(bufR[nxt]);
                    sR = lerp_delay_q15(y0R, y1R, frac);
                };

                int32_t offset_samples = (scrubOffset * 16384) >> 15;
                int32_t lookback = current_loop_len;
                if (pulse1_live && clk_period_samples > 240) {
                    lookback = clk_period_samples;
                }
                int32_t loop_start = (((int32_t)freeze_wr - lookback - offset_samples) & 0x7FFF) << 16;

                // Step arpeggiator on Pulse 2 rising edge
                if (pulse2_live && p2_rising) {
                    if (speedQuant >= 19661 && speedQuant < 26214) {
                        arpeggio_step++;
                        current_speed_q16 = determine_speed_zoned(speedQuant, cv2Corruption, rand_seed, arpeggio_step, current_loop_len);
                        trig_out2 = true;
                    }
                }

                speed_q16 = current_speed_q16;
                rd_q16 += speed_q16;
                sample_ctr++;

                bool crossed = false;
                if (speed_q16 >= 0) {
                    if (rd_q16 >= (int64_t)(current_loop_len << 16)) {
                        crossed = true;
                    }
                } else {
                    if (rd_q16 < 0) {
                        crossed = true;
                    }
                }

                // Force re-trigger on Pulse 1 clock sync trigger
                if (pulse1_live && p1_rising) {
                    crossed = true;
                }

                if (crossed) {
                    trig_out1 = true; // Output loop sync trigger

                    uint32_t roll = fast_rand(rand_seed) & 0x7FFF;
                    int32_t log2_size  = 31 - __builtin_clz((uint32_t)current_loop_len);
                    int32_t size_steps = log2_size - 7;
                    if (size_steps < 0) size_steps = 0;
                    if (size_steps > 7) size_steps = 7;
                    static const int32_t sf_lut[8] = {32767, 28086, 23405, 18724, 14043, 9362, 4681, 0};
                    int32_t size_factor = sf_lut[size_steps];
                    int32_t loop_prob = finalProb + (((32767 - finalProb) * size_factor) >> 15);
                    if (loop_prob > 32767) loop_prob = 32767;
                    if (is_clock_sync) {
                        loop_prob = finalProb;
                    }

                    bool keep_looping = (roll < (uint32_t)loop_prob) || eff_glitchInjector;
                    if (pulse1_live) {
                        if (is_clock_sync) {
                            if (p1_rising) {
                                keep_looping = (roll < (uint32_t)loop_prob);
                            } else {
                                keep_looping = true;
                            }
                        } else {
                            keep_looping = keep_looping || p1_gate;
                        }
                    }

                    if (keep_looping) {
                        xfade_rd = loop_start + rd_q16;
                        
                        bool reroll_every_boundary = (speedQuant >= 19661);
                        if (reroll_every_boundary || sample_ctr >= 1024) {
                            if (!pulse2_live) {
                                arpeggio_step++;
                                trig_out2 = true;
                            }
                            current_speed_q16 = determine_speed_zoned(speedQuant, cv2Corruption, rand_seed, arpeggio_step, current_loop_len);
                            sample_ctr = 0;
                        }
                        speed_q16 = current_speed_q16;
                        
                        int32_t final_size = norm_loop_size;
                        int32_t mid_amount = 16384 - abs(mainProb - 16384);
                        if (!pulse1_live) {
                            if (cv1Warp == 0) {
                                if ((int32_t)(fast_rand(rand_seed) & 0x7FFF) < mid_amount) {
                                    int32_t step = (size * 8) >> 15;
                                    uint32_t r = fast_rand(rand_seed) & 3;
                                    if (r == 3) r = 0;
                                    int32_t offset = (int32_t)r - 1;
                                    step += offset;
                                    if (step < 0) step = 0;
                                    if (step > 7) step = 7;
                                    static const int32_t size_lut[8] = {128, 256, 512, 1024, 2048, 4096, 8192, 16384};
                                    final_size = size_lut[step];
                                }
                            } else {
                                if ((int32_t)(fast_rand(rand_seed) & 0x7FFF) < mid_amount) {
                                    int32_t range = final_size >> 2;
                                    if (range > 0) {
                                        int32_t double_range = range * 2;
                                        if (double_range < 1) double_range = 1;
                                        int32_t offset = (((int32_t)(fast_rand(rand_seed) & 0x7FFF) * double_range) >> 15) - range;
                                        final_size += offset;
                                    }
                                }
                            }
                        }
                        if (speedQuant >= 19661) {
                            int32_t jitter_range = final_size >> 2;
                            if (jitter_range > 0) {
                                int32_t offset = (((int32_t)(fast_rand(rand_seed) & 0x7FFF) * (jitter_range * 2)) >> 15) - jitter_range;
                                final_size += offset;
                            }
                        }
                        current_loop_len = clamp_i32(final_size, 128, 16384);

                        if (speed_q16 >= 0) {
                            rd_q16 = 0;
                        } else {
                            rd_q16 = (int64_t)(current_loop_len << 16);
                        }
                        xfade_len = cur_xfade;
                        xfade_ctr = cur_xfade;
                        xfade_step = (32767 << 15) / cur_xfade;
                        xfade_phase = 0;
                        
                        loop_start = (((int32_t)freeze_wr - current_loop_len - offset_samples) & 0x7FFF) << 16;
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
                    current_g711_sample = bufL[((int32_t)(loop_start + rd_q16) >> 16) & 0x7FFF];

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

                    if (glitchFeedback > 0) {
                        int32_t idx = ((loop_start + rd_q16) >> 16) & 0x7FFF;
                        int32_t scaled_fb = (glitchFeedback * 29491) >> 15;
                        int16_t oldL = decode_mulaw(bufL[idx]);
                        int16_t oldR = decode_mulaw(bufR[idx]);
                        int16_t newL = soft_limit_q15(((int32_t)oldL * (32768 - scaled_fb) + (int32_t)sL * scaled_fb) >> 15);
                        int16_t newR = soft_limit_q15(((int32_t)oldR * (32768 - scaled_fb) + (int32_t)sR * scaled_fb) >> 15);
                        bufL[idx] = encode_mulaw(newL);
                        bufR[idx] = encode_mulaw(newR);
                    }

                    outL = sL;
                    outR = sR;
                    return;
                }
            }
        }

        if (dry_fade_ctr > 0) {
            auto read_buf = [&](int32_t ptr, int16_t &sL, int16_t &sR) {
                int32_t  idx  = (ptr >> 16) & 0x7FFF;
                int32_t  nxt  = (idx + 1)   & 0x7FFF;
                uint16_t frac = (uint16_t)(ptr & 0xFFFF);
                int16_t y0L = decode_mulaw(bufL[idx]), y1L = decode_mulaw(bufL[nxt]);
                sL = lerp_delay_q15(y0L, y1L, frac);
                int16_t y0R = decode_mulaw(bufR[idx]), y1R = decode_mulaw(bufR[nxt]);
                sR = lerp_delay_q15(y0R, y1R, frac);
            };

            int16_t sL, sR;
            dry_fade_rd += speed_q16;
            read_buf(dry_fade_rd, sL, sR);

            dry_fade_phase -= dry_fade_step;
            int32_t val = dry_fade_phase >> 15;
            if (val < 0) val = 0;
            if (val > 32767) val = 32767;
            int16_t t = (int16_t)val;
            int16_t wetL = lerp_q15(inL, sL, t);
            int16_t wetR = lerp_q15(inR, sR, t);
            outL = wetL;
            outR = wetR;
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
        // Stabilize and soft-saturate using a division-free cubic curve: y = x - x^3 / 6
        int32_t clamped_x = x;
        if (clamped_x > 32767) clamped_x = 32767;
        else if (clamped_x < -32768) clamped_x = -32768;
        
        int32_t x3 = (((clamped_x * clamped_x) >> 15) * clamped_x) >> 15;
        // x3 * 5461 >> 15 is x^3 / 6 (since 5461/32768 ≈ 1/6)
        int32_t y = clamped_x - ((x3 * 5461) >> 15);
        return y;
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
            
            // Damping sweeps up to 32000 (fully damped) to suppress high-pitched resonant squelch
            target_r = 1000 + ((diff_q15 * 31000) >> 15);
            
            // Quadratic scaling for a softer, more musical wavefolder fade-in
            int32_t diff_sq = (diff_q15 * diff_q15) >> 15;
            
            // Input drive sweeps from 1.0x to 2.37x
            drive = 32768 + ((diff_sq * 45000) >> 15);

            // Fold threshold sweeps down from 32767 to 4000
            fold_thresh = 32767 - ((diff_sq * 28767) >> 15);

            // Output makeup gain sweeps from 1.0x to 1.35x (prevents volume jump)
            makeup = 32768 + ((diff_sq * 11468) >> 15);

            // Division-free scaling (range is 14767):
            // 16 / 14767 ≈ 36 / 32768
            // 127 / 14767 ≈ 282 / 32768
            dec_step = (diff * 36) >> 15;
            xor_mask = (diff * 282) >> 15;
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

            // Hardware-accelerated linear interpolation via INTERP0 blend mode
            int16_t dIn  = lerp_delay_q15(bufIn[rd1],  bufIn[rd2],  frac);
            int16_t dOut = lerp_delay_q15(bufOut[rd1], bufOut[rd2], frac);

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
            return lerp_delay_q15(val1, val2, frac);
        }
    };

    int16_t mem[28672];
    AP apIn[4], apTankL, apTankR;
    Delay modL, d1L, d2L, modR, d1R, d2R;
    int32_t lpL = 0, lpR = 0, lpIn = 0;
    uint32_t lfo = 0;
    int32_t lp_size_scale = 32767;

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
        lp_size_scale = 32767;
    }

    void process(int16_t &L, int16_t &R, int32_t mix, int32_t size, int32_t fb_glitch) {
        // Map Size (X) to scale factor: [0..32767] -> [4915..32767] (0.15x to 1.0x)
        int32_t size_scale = 4915 + (((int32_t)size * 27852) >> 15);

        // Max decay limit is dynamic based on size to prevent self-oscillation explosion
        // [4915..32767] size_scale maps to [18000..28672] max_decay (0.55x to 0.875x decay coefficient)
        // (size_scale - 4915) * 10672 / 27852 ≈ (size_scale - 4915) * 38429 >> 20
        int32_t max_decay = 18000 + (((size_scale - 4915) * 38429) >> 20);

        // Map Feedback/Glitch (Y) to smooth overlapping morph curves:
        // - Decay time rises cleanly from 0 to max_decay over 0..20000.
        // - Lo-fi bitcrush fuzz starts rising at 12000.
        // - Sparkle (XOR sizzle) starts rising at 14000 (stable high-fidelity digital crackle).
        // - Circuit-bent address jitter + decimation starts rising at 20000 (introducing pitch-warping).
        int32_t decay = 0;
        int32_t lofi_level = 0;
        int32_t sparkle_level = 0;
        int32_t circuit_bent_level = 0;

        // Decay
        if (fb_glitch < 20000) {
            int32_t val = (fb_glitch * 107374) >> 16; // scales [0..20000] -> [0..32767]
            decay = (val * max_decay) >> 15;
        } else {
            decay = max_decay;
        }

        // Lo-fi Level (bitcrush fuzz)
        if (fb_glitch < 12000) {
            lofi_level = 0;
        } else if (fb_glitch < 26000) {
            int32_t diff = fb_glitch - 12000;
            int32_t raw_lofi = (diff * 153391) >> 16;
            lofi_level = (raw_lofi * raw_lofi) >> 15; // quadratic curve for fine control of subtle fuzz
        } else {
            lofi_level = 32767;
        }

        // Sparkle Level (XOR sizzle)
        if (fb_glitch < 14000) {
            sparkle_level = 0;
        } else if (fb_glitch < 28000) {
            int32_t diff = fb_glitch - 14000;
            sparkle_level = (diff * 153391) >> 16; // 14000 width
        } else {
            sparkle_level = 32767;
        }

        // Circuit-Bent Level (Jitter + Decimation)
        if (fb_glitch < 20000) {
            circuit_bent_level = 0;
        } else {
            int32_t diff = fb_glitch - 20000;
            circuit_bent_level = (diff * 168188) >> 16;
            if (circuit_bent_level > 32767) circuit_bent_level = 32767;
        }

        // LFO Chorus: constant slow speed for high-fidelity lushness (no fast warbles) (doubled for 24kHz)
        lfo += 256;
        int16_t mod = (lfo >> 16) & 0x7FFF;
        if (lfo & 0x80000000) mod = 32767 - mod;

        // Modulate size_scale slightly by LFO for lush chorus effect
        // Keep modulation depth small and constant (120 ≡ ~0.37% size) for rich movement
        int32_t mod_depth = 120;
        int32_t modulated_size_scale = size_scale + (((mod - 16384) * mod_depth) >> 15);
        if (modulated_size_scale < 3276) modulated_size_scale = 3276;
        if (modulated_size_scale > 32767) modulated_size_scale = 32767;

        // Smooth size scale slowly to eliminate pitch-glide howling when changing room sizes
        lp_size_scale += (modulated_size_scale - lp_size_scale) >> 11; // ~85ms time constant

        // Apply Address Jitter / Read Head Flutter in circuit-bent mode to left/right channels
        int32_t scaleL = lp_size_scale;
        int32_t scaleR = lp_size_scale;
        if (circuit_bent_level > 0) {
            uint32_t r = fast_rand(rand_seed);
            int32_t scale_jitterL = ((r & 0xFF) * circuit_bent_level) >> 13; // max ~1019 (~3.1% size)
            int32_t scale_jitterR = (((r >> 8) & 0xFF) * circuit_bent_level) >> 13;
            scaleL -= scale_jitterL;
            scaleR -= scale_jitterR;
            if (scaleL < 3276) scaleL = 3276;
            if (scaleR < 3276) scaleR = 3276;
        }

        // Input mono summing — no pre-filter, keep full bandwidth into the tank
        int16_t mono = (int16_t)(((int32_t)L + (int32_t)R) >> 1);

        // Input all-passes are kept at fixed scale to prevent pitch-glide in the diffusion network
        for (int i = 0; i < 4; i++) {
            mono = apIn[i].process(mono, 32767);
        }

        // Read tank loop outputs using jittered scales
        int16_t tOutL = d2L.read(scaleL);
        int16_t tOutR = d2R.read(scaleR);

        // Tank HF damping: bright and transparent at Y=0, warms up as lofi_level rises.
        // Base = 30000/32768 ≈ 0.91 (~25 kHz); max = 24000/32768 ≈ 0.73 (~12 kHz) at full Y.
        int32_t damp = 30000 - ((lofi_level * 6000) >> 15);

        // Left Tank
        int32_t iL = (int32_t)mono + (((int32_t)decay * tOutR) >> 15);
        iL = soft_limit_q15(iL);
        int16_t sL = (int16_t)iL + (int16_t)((16384 * modL.read(scaleL)) >> 15);
        modL.write(soft_limit_q15((int32_t)iL - (int16_t)((16384 * sL) >> 15)));
        d1L.write(sL);
        sL = d1L.read(scaleL);
        lpL += (((int32_t)sL - lpL) * damp) >> 15;
        if (lpL >  32767) lpL =  32767;
        if (lpL < -32768) lpL = -32768;
        sL = (int16_t)lpL;
        sL = apTankL.process(sL, 32767);
        d2L.write(sL);

        // Right Tank
        int32_t iR = (int32_t)mono + (((int32_t)decay * tOutL) >> 15);
        iR = soft_limit_q15(iR);
        int16_t sR = (int16_t)iR + (int16_t)((16384 * modR.read(scaleR)) >> 15);
        modR.write(soft_limit_q15((int32_t)iR - (int16_t)((16384 * sR) >> 15)));
        d1R.write(sR);
        sR = d1R.read(scaleR);
        lpR += (((int32_t)sR - lpR) * damp) >> 15;
        if (lpR >  32767) lpR =  32767;
        if (lpR < -32768) lpR = -32768;
        sR = (int16_t)lpR;
        sR = apTankR.process(sR, 32767);
        d2R.write(sR);

        int32_t wetL = sL;
        int32_t wetR = sR;

        // Apply Bitcrushing and XOR Scrambling OUTSIDE the feedback loop to keep the reverb tail natural and long
        // 1. Continuous Bitcrushing (word-length truncation) based on lofi_level
        if (lofi_level > 0) {
            int32_t shift_q15 = (lofi_level * 6); // scale from 0 to 6 in Q15
            int32_t int_shift = shift_q15 >> 15;
            int32_t frac_shift = shift_q15 & 0x7FFF;

            int16_t q1L = (wetL >> int_shift) << int_shift;
            int16_t q2L = (wetL >> (int_shift + 1)) << (int_shift + 1);
            wetL = lerp_q15(q1L, q2L, frac_shift);

            int16_t q1R = (wetR >> int_shift) << int_shift;
            int16_t q2R = (wetR >> (int_shift + 1)) << (int_shift + 1);
            wetR = lerp_q15(q1R, q2R, frac_shift);
        }

        // 2. XOR Scrambling (controls the sparkle)
        if (sparkle_level > 0) {
            int16_t xor_mask = (int16_t)((sparkle_level * 31) >> 15);
            wetL ^= xor_mask;
            wetR ^= xor_mask;
        }

        // Run loop DC blockers to eliminate DC accumulation
        wetL = dc_loopL.process(wetL);
        wetR = dc_loopR.process(wetR);

        // Heavy Decimation glitch (only in the last 20% circuit-bent range)
        if (circuit_bent_level > 0) {
            int32_t dec_factor = 1 + ((circuit_bent_level * 3) >> 15); // up to 4x decimation to prevent piercing squeals
            decimate_phase++;
            if (decimate_phase >= dec_factor) {
                decimate_phase = 0;
                last_outL = (int16_t)wetL;
                last_outR = (int16_t)wetR;
            }
            wetL = last_outL;
            wetR = last_outR;

            // Apply 1-pole LPF to smooth out decimation steps (use division-free lookup table)
            static const int16_t dec_coef_table[17] = {
                0, 32767, 16384, 10922, 8192, 6553, 5461, 4681, 4096, 3640, 3276, 2978, 2730, 2520, 2340, 2184, 2048
            };
            int32_t dec_coef = dec_coef_table[dec_factor <= 16 ? (dec_factor >= 1 ? dec_factor : 1) : 16];

            dec_lpL += (((int32_t)wetL - dec_lpL) * dec_coef) >> 15;
            dec_lpR += (((int32_t)wetR - dec_lpR) * dec_coef) >> 15;
            wetL = (int16_t)dec_lpL;
            wetR = (int16_t)dec_lpR;
        } else {
            dec_lpL = wetL;
            dec_lpR = wetR;
        }

        // Wet/Dry mix using split_mix_q15 to prevent volume drops
        L = split_mix_q15(L, wetL, (int16_t)mix);
        R = split_mix_q15(R, wetR, (int16_t)mix);
    }
};

#endif // DSP_BLOCKS_H
