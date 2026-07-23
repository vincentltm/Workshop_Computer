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
extern int16_t mulaw_decode_table[256];

// Fast G.711 mu-law encoder
inline uint8_t encode_mulaw(int16_t sample) {
    int32_t x = sample;
    int32_t sign = (x < 0) ? 0x80 : 0x00;
    if (x < 0) x = -x;
    x += 132;
    if (x > 32767) x = 32767;

    int32_t exponent;
    if (x < 2048) {
        if (x < 512) {
            if (x < 256) {
                exponent = 7;
            } else {
                exponent = 8;
            }
        } else {
            if (x < 1024) {
                exponent = 9;
            } else {
                exponent = 10;
            }
        }
    } else {
        if (x < 8192) {
            if (x < 4096) {
                exponent = 11;
            } else {
                exponent = 12;
            }
        } else {
            if (x < 16384) {
                exponent = 13;
            } else {
                exponent = 14;
            }
        }
    }

    int32_t mantissa = (x >> (exponent - 4)) & 0x0F;
    return (uint8_t)(sign | ((exponent - 7) << 4) | mantissa);
}

// G.711 mu-law decoder optimized to a single memory lookup
inline int16_t decode_mulaw(uint8_t u_val) {
    return mulaw_decode_table[u_val];
}

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

    int16_t process(int16_t in, int32_t r_q15 = 32751) {
        int32_t x = in;
        // y[n] = x[n] - x[n-1] + R * y[n-1]
        // 32751 / 32768 ≈ 0.9995 (ultra-tight DC cutoff ~1.9 Hz at 24 kHz)
        int32_t y = x - x1 + ((y1 * r_q15) >> 15);
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

    __attribute__((always_inline)) inline void process(int16_t inL, int16_t &outL, int16_t inR, int16_t &outR,
                 int32_t mainMix, int32_t rate, int16_t depth, int32_t feedback, int32_t xor_mask,
                 int32_t cv1Warp, int32_t raw_depth_fb)
    {
        auto get_tri = [](uint16_t phase) -> int16_t {
            int32_t tmp = (phase < 32768) ? ((phase << 1) - 32768) : (32767 - ((phase - 32768) << 1));
            return (int16_t)tmp;
        };

        int32_t eff_feedback = feedback;
        int32_t res_xor = 0;
        int32_t lp_coef = 16384; // Default cutoff ~6kHz

        if (raw_depth_fb >= 24000) {
            
            if (raw_depth_fb < 28500) {
                // Clean zone: feedback ramps from 31000 (long ring) up to 32500 (infinite clean ring, no clipping)
                int32_t t = ((raw_depth_fb - 24000) * 32767) / (28500 - 24000);
                eff_feedback = 31000 + ((t * (32500 - 31000)) >> 15);
                res_xor = 0;
            } else {
                // Corrupt zone: feedback ramps from 32500 to 33000 (slightly above unity for grit), scramble ramps 0 to 15
                int32_t t = ((raw_depth_fb - 28500) * 32767) / (32767 - 28500);
                eff_feedback = 32500 + ((t * (33000 - 32500)) >> 15);
                res_xor = (t * 15) >> 15;
            }
        }

        if (mainMix < 50) {
            outL = inL;
            outR = inR;
            delayL[write_ptr] = inL;
            delayR[write_ptr] = inR;
            write_ptr = (write_ptr + 1) & 0x3FF;
            
            if (raw_depth_fb >= 24000) {
                uint16_t phase_inc = (uint16_t)(1 + (rate >> 11));
                lfo_phase += phase_inc;
            } else {
                int32_t eff_rate = rate + (cv1Warp * 4);
                eff_rate = clamp_i32(eff_rate, 0, 32767);
                uint16_t phase_inc = (uint16_t)(1 + (eff_rate >> 11));
                lfo_phase += phase_inc;
            }
            return;
        }

        int32_t delay_L_q16;
        int32_t delay_R_q16;

        if (raw_depth_fb >= 24000) {
            // Resonator Mode: Knob X (rate) directly controls pitch (delay time)
            int32_t pitch_ctrl = rate;
            int32_t base_delay = 1000 - ((pitch_ctrl * (1000 - 12)) >> 15);
            
            // Bipolar CV1 (cv1Warp) V/Oct scaling: delay = base_delay * 1024 / (1024 + cv1Warp)
            int32_t delay_val = base_delay;
            if (cv1Warp != 0) {
                int32_t denominator = 1024 + cv1Warp;
                if (denominator < 24) denominator = 24; // clamp to prevent division by zero
                delay_val = (base_delay * 1024) / denominator;
            }
            delay_val = clamp_i32(delay_val, 12, 1000);

            // Pitch-dependent LPF cutoff: high notes are bright, low notes are damped and warm
            lp_coef = clamp_i32(18000 - ((delay_val * 13500) >> 10), 4500, 20000);

            // Add a tiny organic LFO drift (depth = 1 sample) to keep the drone alive
            uint16_t phase_inc = (uint16_t)(1 + (rate >> 11));
            lfo_phase += phase_inc;
            
            int16_t lfoL = get_tri(lfo_phase);
            int16_t lfoR = get_tri((uint16_t)(lfo_phase + 32768u));
            
            // Tiny ±1 sample BBD drift (depth_samples = 1)
            delay_L_q16 = (delay_val << 16) + (((int32_t)lfoL * 1) << 1);
            delay_R_q16 = (delay_val << 16) + (((int32_t)lfoR * 1) << 1);
        } else {
            // Normal Chorus: Knob X controls LFO speed
            int32_t eff_rate = rate + (cv1Warp * 4);
            eff_rate = clamp_i32(eff_rate, 0, 32767);
            uint16_t phase_inc = (uint16_t)(1 + (eff_rate >> 10));
            lfo_phase += phase_inc;

            int16_t lfoL = get_tri(lfo_phase);
            int16_t lfoR = get_tri((uint16_t)(lfo_phase + 32768u));

            int32_t depth_samples = (int32_t)depth * 28 >> 15;
            delay_L_q16 = (180 << 16) + (((int32_t)lfoL * depth_samples) << 1);
            delay_R_q16 = (180 << 16) + (((int32_t)lfoR * depth_samples) << 1);
        }
        
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
            return lerp_delay_q15(y0, y1, frac);
        };

        int16_t wetL = read_frac(delayL, (write_ptr << 16), delay_L_q16);
        int16_t wetR = read_frac(delayR, (write_ptr << 16), delay_R_q16);

        // BBD low-pass filter (cutoff depends on mode)
        lp_stateL += ((wetL - lp_stateL) * lp_coef) >> 15;
        wetL = (int16_t)lp_stateL;
        lp_stateR += ((wetR - lp_stateR) * lp_coef) >> 15;
        wetR = (int16_t)lp_stateR;

        // Vintage HPF on wet path (cutoff ~110 Hz)
        int32_t yL = (int32_t)wetL - hp_x1L + ((hp_y1L * 32000) >> 15);
        hp_x1L = (int32_t)wetL;
        hp_y1L = clamp_i32(yL, -1000000, 1000000);
        wetL = saturate_q15(yL);

        int32_t yR = (int32_t)wetR - hp_x1R + ((hp_y1R * 32000) >> 15);
        hp_x1R = (int32_t)wetR;
        hp_y1R = clamp_i32(yR, -1000000, 1000000);
        wetR = saturate_q15(yR);

        // Feedback loop
        if (eff_feedback > 0) {
            int32_t fbL = (int32_t)inL + (((int32_t)wetL * eff_feedback) >> 15);
            int32_t fbR = (int32_t)inR + (((int32_t)wetR * eff_feedback) >> 15);
            
            int16_t wrL = soft_limit_q15(fbL);
            int16_t wrR = soft_limit_q15(fbR);
            
            if (raw_depth_fb >= 24000) {
                if (res_xor > 0) {
                    uint8_t byteL = encode_mulaw(wrL);
                    uint8_t byteR = encode_mulaw(wrR);
                    byteL ^= (res_xor & 0x0F);
                    byteR ^= (res_xor & 0x0F);
                    wrL = decode_mulaw(byteL);
                    wrR = decode_mulaw(byteR);
                }
            } else if (xor_mask > 0) {
                wrL ^= (uint16_t)xor_mask;
                wrR ^= (uint16_t)xor_mask;
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

    // Watery MP3 lossy subband simulation delay lines
    int16_t mp3_delayL[256];
    int16_t mp3_delayR[256];
    uint8_t mp3_wr = 0;
    uint16_t mp3_phase = 0;
    bool     link_state_bad = false;

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
    int32_t pop_decayL = 0;
    int32_t pop_decayR = 0;
    int32_t pop_decay_rate = 28000;
    int32_t vinyl_lpL = 0;
    int32_t vinyl_lpR = 0;

    // Upgraded engines states
    int16_t tape_delayL[128];
    int16_t tape_delayR[128];
    uint8_t tape_wr = 0;
    uint16_t flutter_phase = 0;
    uint32_t vinyl_timer = 0;
    int16_t trans_rep_ctr = 0;
    int16_t trans_max_reps = 0;
    uint32_t shred_state = 0;

    // Split-band low-end preservation state (< 120Hz)
    int32_t sub_hpL = 0, sub_hpR = 0;
    // Data howl feedback state
    int16_t howl_fbL = 0, howl_fbR = 0;
    // Sample rate decimation clock slew phase
    uint32_t dec_slew_phase = 0;

    void init() {
        codec_v1L = codec_v2L = codec_v1R = codec_v2R = 0;
        for (int i = 0; i < 256; i++) {
            trans_historyL[i] = trans_historyR[i] = 0;
            mp3_delayL[i] = mp3_delayR[i] = 0;
        }
        for (int i = 0; i < 128; i++) {
            tape_delayL[i] = tape_delayR[i] = 0;
        }
        tape_wr = 0;
        flutter_phase = 0;
        vinyl_timer = 0;
        trans_rep_ctr = 0;
        trans_max_reps = 0;
        shred_state = 0;

        sub_hpL = sub_hpR = 0;
        howl_fbL = howl_fbR = 0;
        dec_slew_phase = 0;
        pop_decayL = pop_decayR = 0;
        vinyl_lpL = vinyl_lpR = 0;

        trans_wr = 0;
        mp3_wr = 0;
        mp3_phase = 0;
        link_state_bad = false;
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
        pop_decayL = pop_decayR = 0;
        pop_decay_rate = 28000;
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
        if (insanity <= 0) return sample;

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

        auto reconstruct = [&](uint8_t mant) -> int16_t {
            int32_t rec = 0;
            if (exponent > 0) {
                rec = ((mant << 1) + 33) << (exponent + 1);
            } else {
                rec = (mant << 2) + 2;
            }
            if (rec > 32767) rec = 32767;
            return (int16_t)(rec * sign);
        };

        // Compute reconstructed samples at each bit depth (4, 3, 2, 1 mantissa bits)
        int16_t s4 = reconstruct(mantissa);
        int16_t s3 = reconstruct(mantissa & 0xE);
        int16_t s2 = reconstruct(mantissa & 0xC);
        int16_t s1 = reconstruct(mantissa & 0x8);

        // Scale insanity to 0..3 index
        int32_t val = insanity * 3; // 0 to 98301
        int32_t idx = val >> 15;
        int16_t fade = val & 0x7FFF;

        if (idx == 0) {
            return lerp_q15(s4, s3, fade);
        } else if (idx == 1) {
            return lerp_q15(s3, s2, fade);
        } else {
            return lerp_q15(s2, s1, fade);
        }
    }

    __attribute__((always_inline)) inline void process(int16_t inL, int16_t &outL, int16_t inR, int16_t &outR,
                 int32_t strength,
                 int32_t mp3_ring_level, int32_t fuzz_level, int32_t decimate_level,
                 int32_t pop_prob, int32_t click_depth, int32_t bad_conn_level, int32_t scramble_level,
                 int32_t sputter_prob, int32_t tape_sat, int32_t tape_hiss, int32_t active_loss,
                 int32_t tape_hicut, uint32_t &rand_seed)
    {
        if (strength < 800) {
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
            lp_newL = inL;
            lp_newR = inR;
            return;
        }

        // ── 1. Temporal Breathing / Vibe LFO (breathes at ~1.4 Hz) ─────────────
        vibe_lfo += 4; // doubled for 24kHz
        int16_t vibe_sine = lookup_sine(vibe_lfo); // [-32768, 32767]

        int16_t sigL = inL;
        int16_t sigR = inR;

        // Scale gating by absolute input amplitude to keep silence clean
        int32_t input_amp = (sigL < 0 ? -sigL : sigL) + (sigR < 0 ? -sigR : sigR);

        // ── 0. Split-Band Low-End Preservation Filter (< 120 Hz) ───────────────
        int32_t subL = sub_hpL + (((int32_t)inL - sub_hpL) * 256 >> 15);
        int32_t subR = sub_hpR + (((int32_t)inR - sub_hpR) * 256 >> 15);
        sub_hpL = subL; sub_hpR = subR;

        // ── Data Howl Feedback (Self-sustaining digital resonance when silent) ─
        if (active_loss > 16384 && input_amp < 300) {
            int32_t howl_gain = (active_loss - 16384);
            int32_t howlL = ((int32_t)howl_fbL * howl_gain) >> 15;
            int32_t howlR = ((int32_t)howl_fbR * howl_gain) >> 15;
            sigL = saturate_q15(sigL + howlL);
            sigR = saturate_q15(sigR + howlR);
        }

        // ── Stage 0: Analog Tape Saturation, Hiss, and Wow & Flutter ──
        if (tape_sat > 0) {
            // 1. Saturation
            int32_t satL = tape_saturate(((int32_t)sigL * (32768 + tape_sat)) >> 15);
            int32_t satR = tape_saturate(((int32_t)sigR * (32768 + tape_sat)) >> 15);
            sigL = lerp_q15(sigL, satL, tape_sat);
            sigR = lerp_q15(sigR, satR, tape_sat);
            
            // 2. Write to Wow/Flutter Delay Line
            tape_delayL[tape_wr] = sigL;
            tape_delayR[tape_wr] = sigR;
            
            // 3. Modulate Read Pointer
            // Slow wow: vibe_sine has period ~1.4 Hz
            int32_t wow = (lookup_sine(vibe_lfo) * tape_sat) >> 22; // ranges from -8 to +8 samples
            
            // Fast flutter: faster random noise LFO
            flutter_phase += 380;
            int32_t flutter = (lookup_sine(flutter_phase) * tape_sat) >> 24; // ranges from -2 to +2 samples
            
            int32_t delay_offset = 32 + wow + flutter; // offset buffer is 32 samples (safe middle)
            uint8_t rd = (tape_wr - (uint8_t)delay_offset) & 127;
            sigL = tape_delayL[rd];
            sigR = tape_delayR[rd];
            
            tape_wr = (tape_wr + 1) & 127;
        } else {
            tape_delayL[tape_wr] = sigL;
            tape_delayR[tape_wr] = sigR;
            tape_wr = (tape_wr + 1) & 127;
        }

        if (tape_hiss > 0) {
            int32_t noiseL = (((int32_t)(fast_rand(rand_seed) & 0x1FF)) - 256) * tape_hiss >> 8;
            int32_t noiseR = (((int32_t)(fast_rand(rand_seed) & 0x1FF)) - 256) * tape_hiss >> 8;
            sigL = saturate_q15(sigL + noiseL);
            sigR = saturate_q15(sigR + noiseR);
        }

        // ── Stage 0.2: Warm Vinyl Dust Crackle & Needle Thuds (Low-pass acoustic response) ──
        pop_decayL = (pop_decayL * pop_decay_rate) >> 15;
        pop_decayR = (pop_decayR * pop_decay_rate) >> 15;

        if (pop_prob > 0 && input_amp > 80) {
            vinyl_timer++;
            // 33.3 RPM revolution period (~1.8 sec) with subtle speed drift
            uint32_t vinyl_period = 43200 + (fast_rand(rand_seed) & 1023);
            if (vinyl_timer >= vinyl_period) {
                vinyl_timer = 0;
                pop_decay_rate = 22000 + (fast_rand(rand_seed) & 2047); // warm, deep scratch decay
                int32_t impulse = (((int32_t)(fast_rand(rand_seed) & 0xFFFF)) - 32768) * click_depth >> 15;
                pop_decayL = clamp_i32(pop_decayL + impulse, -32768, 32767);
                pop_decayR = clamp_i32(pop_decayR + impulse, -32768, 32767);
            }

            uint32_t roll = fast_rand(rand_seed) & 0x7FFF;
            if ((int32_t)roll < pop_prob) {
                pop_decay_rate = 25000 + (fast_rand(rand_seed) & 2047); // soft dust crackle
                int32_t impulse = (((int32_t)(fast_rand(rand_seed) & 0xFFFF)) - 32768) * click_depth >> 16;
                pop_decayL = clamp_i32(pop_decayL + impulse, -32768, 32767);
                
                int32_t impulseR = impulse + (((((int32_t)(fast_rand(rand_seed) & 0x1FF)) - 256) * click_depth) >> 17);
                pop_decayR = clamp_i32(pop_decayR + impulseR, -32768, 32767);
            }
        } else {
            vinyl_timer = 0;
        }

        // Low-pass filter the pop impulse through a 1.8kHz acoustic vinyl LPF to eliminate sharp digital PCM spitting
        vinyl_lpL += (((int32_t)pop_decayL - vinyl_lpL) * 3500) >> 15;
        vinyl_lpR += (((int32_t)pop_decayR - vinyl_lpR) * 3500) >> 15;

        sigL = saturate_q15(sigL + (int16_t)vinyl_lpL);
        sigR = saturate_q15(sigR + (int16_t)vinyl_lpR);

        // ── Stage 0.5: Digital Hash Noise (Zone 3 of Y) ──
        if (scramble_level > 0) {
            int32_t noise_amp = (scramble_level * 20) >> 15; // subtle pre-bitcrush digital noise
            int32_t hashL = (((int32_t)(fast_rand(rand_seed) & 0x1FF)) - 256) * noise_amp >> 8;
            int32_t hashR = (((int32_t)(fast_rand(rand_seed) & 0x1FF)) - 256) * noise_amp >> 8;
            sigL = saturate_q15(sigL + hashL);
            sigR = saturate_q15(sigR + hashR);
        }

        // ── Stage 1: Warm Fuzz (Bitcrushing) ──
        if (fuzz_level > 0) {
            int16_t compL = compress_expand_mulaw_variable(sigL, fuzz_level);
            int16_t compR = compress_expand_mulaw_variable(sigR, fuzz_level);

            // Continuous fractional bitcrusher for fuzz
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
            int32_t satL = tape_saturate(((int32_t)compL * (32768 + fuzz_level)) >> 15);
            int32_t satR = tape_saturate(((int32_t)compR * (32768 + fuzz_level)) >> 15);
            
            // Smoothly crossfade clean to fuzz based on fuzz_level
            sigL = lerp_q15(sigL, satL, fuzz_level);
            sigR = lerp_q15(sigR, satR, fuzz_level);
        }

        // ── Telecom Bandpass Filter (Always active for stability and Zone 3 AM Radio) ──
        int32_t rg_c = 2200; 
        int32_t rr_c = 28000 - ((fuzz_level * 22000) >> 15);

        int32_t hp_cL = (int32_t)sigL - ((rr_c * codec_v1L) >> 15) - codec_v2L;
        codec_v1L += (rg_c * hp_cL) >> 15;
        codec_v1L = saturate_q15(codec_v1L);
        int32_t bp_cL = codec_v1L;
        codec_v2L = codec_v2L + ((rg_c * codec_v1L) >> 15);
        codec_v2L = saturate_q15(codec_v2L);

        int32_t hp_cR = (int32_t)sigR - ((rr_c * codec_v1R) >> 15) - codec_v2R;
        codec_v1R += (rg_c * hp_cR) >> 15;
        codec_v1R = saturate_q15(codec_v1R);
        int32_t bp_cR = codec_v1R;
        codec_v2R = codec_v2R + ((rg_c * codec_v1R) >> 15);
        codec_v2R = saturate_q15(codec_v2R);

        // Scale the bandpass filter mix continuously with fuzz_level (up to 12000)
        int32_t bp_mix_amount = (fuzz_level * 12000) >> 15;
        if (bp_mix_amount > 0) {
            sigL = lerp_q15(sigL, saturate_q15(bp_cL), bp_mix_amount);
            sigR = lerp_q15(sigR, saturate_q15(bp_cR), bp_mix_amount);
        }

        // ── Stage 1.5: Watery Lossy MP3 Subband Ringing ──
        if (mp3_ring_level > 0) {
            // Modulate the phase fast based on mp3_ring_level
            mp3_phase += 150 + (mp3_ring_level >> 4);
            int16_t lfo_val = lookup_sine(mp3_phase); // [-32768, 32767]

            // Delay time between 10 and 40 samples, modulated fast
            int32_t delay_samples = 25 + (((int32_t)lfo_val * (15 + (mp3_ring_level >> 11))) >> 15);
            int32_t idx = (mp3_wr - delay_samples) & 0xFF;
            int16_t delay_sampleL = mp3_delayL[idx];
            int16_t delay_sampleR = mp3_delayR[idx];

            // Resonant comb feedback loop
            int32_t fbL = (int32_t)sigL + (((int32_t)delay_sampleL * 16384) >> 15);
            int32_t fbR = (int32_t)sigR + (((int32_t)delay_sampleR * 16384) >> 15);
            mp3_delayL[mp3_wr] = saturate_q15(fbL);
            mp3_delayR[mp3_wr] = saturate_q15(fbR);
            mp3_wr = (mp3_wr + 1) & 0xFF;

            // Phase-additive combination to create watery ringing subbands while preserving low-end volume
            int16_t ringL = saturate_q15(sigL + (((int32_t)delay_sampleL * 8000) >> 15));
            int16_t ringR = saturate_q15(sigR + (((int32_t)delay_sampleR * 8000) >> 15));

            // Blend based on mp3_ring_level
            sigL = lerp_q15(sigL, ringL, mp3_ring_level);
            sigR = lerp_q15(sigR, ringR, mp3_ring_level);
        } else {
            // Keep delay lines silent but cleared if not active
            mp3_delayL[mp3_wr] = sigL;
            mp3_delayR[mp3_wr] = sigR;
            mp3_wr = (mp3_wr + 1) & 0xFF;
        }

        // ── Stage 2: Bad Connection (Packet Drops & Stutter Repetition) ──────
        if (bad_conn_level > 0 || scramble_level > 0) {
            if (!trans_dropped) {
                trans_historyL[trans_wr] = sigL;
                trans_historyR[trans_wr] = sigR;
            }

            if (trans_frame_ctr >= trans_frame_size) {
                trans_frame_ctr = 0;
                trans_frame_size = 240 + (((fast_rand(rand_seed) & 0xFFFF) * 720) >> 16);

                // Gilbert-Elliott packet loss model (bursty clustering)
                int32_t p_good_to_bad = (active_loss * 400) >> 15;
                int32_t p_bad_to_good = 4000 - ((active_loss * 2500) >> 15);

                uint32_t state_roll = fast_rand(rand_seed) & 0x7FFF;
                if (link_state_bad) {
                    if ((int32_t)state_roll < p_bad_to_good) {
                        link_state_bad = false;
                    }
                } else {
                    if ((int32_t)state_roll < p_good_to_bad) {
                        link_state_bad = true;
                    }
                }

                int32_t drop_thresh = 0;
                if (link_state_bad) {
                    drop_thresh = (active_loss * 12000) >> 15;
                    if (scramble_level > 0) {
                        drop_thresh += (scramble_level * 4000) >> 15;
                    }
                }

                uint32_t roll = fast_rand(rand_seed) & 0x7FFF;
                if ((int32_t)roll < drop_thresh) {
                    trans_dropped = true;
                    trans_loop_size = 64 + ((active_loss * 192) >> 15);
                    if (scramble_level > 0) {
                        trans_loop_size += (scramble_level * 64) >> 15;
                    }
                    if (trans_loop_size > 255) trans_loop_size = 255;
                    trans_drop_rd = (trans_wr - trans_loop_size) & 0xFF;
                    trans_rep_ctr = 0;
                    trans_max_reps = 3 + (fast_rand(rand_seed) & 7); // repeat 3 to 10 times rhythmically!
                }
            }
            trans_frame_ctr++;

            if (trans_dropped) {
                sigL = trans_historyL[trans_drop_rd];
                sigR = trans_historyR[trans_drop_rd];
                howl_fbL = sigL;
                howl_fbR = sigR;
                
                trans_loop_ctr++;
                if (trans_loop_ctr >= trans_loop_size) {
                    trans_loop_ctr = 0;
                    trans_drop_rd = (trans_wr - trans_loop_size) & 0xFF;
                    
                    trans_rep_ctr++;
                    if (trans_rep_ctr >= trans_max_reps) {
                        trans_dropped = false; // release loop
                        trans_rep_ctr = 0;
                    }
                } else {
                    trans_drop_rd = (trans_drop_rd + 1) & 0xFF;
                }
            } else {
                trans_wr = (trans_wr + 1) & 0xFF;
                trans_loop_ctr = 0;
                trans_rep_ctr = 0;
            }

            int32_t scramble_prob = (active_loss * 4000) >> 15; 
            if ((int32_t)(fast_rand(rand_seed) & 0x7FFF) < scramble_prob) {
                uint32_t limit = 1 + (active_loss >> 11);
                uint16_t mask = (uint16_t)((fast_rand(rand_seed) >> (32 - 4)) & (limit - 1));
                if (input_amp > 100) {
                    // Shred logic: dynamically toggle between XOR, bit-mask AND, and bit-shifts
                    shred_state++;
                    uint32_t shred_mode = (shred_state >> 3) & 3;
                    if (shred_mode == 0) {
                        sigL ^= mask;
                        sigR ^= mask;
                    } else if (shred_mode == 1) {
                        sigL = sigL & ~mask;
                        sigR = sigR & ~mask;
                    } else if (shred_mode == 2) {
                        sigL = sigL << 1;
                        sigR = sigR << 1;
                    } else {
                        sigL = (sigL >> 2) << 2;
                        sigR = (sigR >> 2) << 2;
                    }
                }
            }
        }

        // No Y knob filtering to prevent losing low frequencies (kick drum)
        lp_newL = sigL;
        lp_newR = sigR;

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

        int32_t drive_gain = 32768 + ((strength * 11468) >> 15);
        int32_t compLi = soft_limit_q15(((int32_t)wetL * drive_gain) >> 15);
        int32_t compRi = soft_limit_q15(((int32_t)wetR * drive_gain) >> 15);
        compLi = (compLi * gain_coef) >> 15;
        compRi = (compRi * gain_coef) >> 15;

        // Compensated makeup gain to keep overall wet path loudness constant
        int32_t makeup_gain = 32768 - ((strength * 6000) >> 15);
        compLi = (compLi * makeup_gain) >> 15;
        compRi = (compRi * makeup_gain) >> 15;

        wetL = tape_saturate(saturate_q15(compLi));
        wetR = tape_saturate(saturate_q15(compRi));

        // ── 4. Downsampling (from X Knob) ─────────────────────────────────────
        if (decimate_level > 800) {
            int32_t xor_mask = ((decimate_level - 800) * 127) / (32767 - 800);
            if (xor_mask > 0) {
                uint8_t byteL = encode_mulaw(wetL);
                uint8_t byteR = encode_mulaw(wetR);
                byteL ^= xor_mask;
                byteR ^= xor_mask;
                wetL = decode_mulaw(byteL);
                wetR = decode_mulaw(byteR);
            }

            int16_t pre_decL = wetL;
            int16_t pre_decR = wetR;

            if (dec_ctr >= current_dec_step) {
                decL    = wetL;
                decR    = wetR;
                dec_ctr = 0;

                uint32_t ds_sq = ((uint32_t)decimate_level * decimate_level) >> 15;
                uint32_t base_step = 1 + ((ds_sq * 23) >> 15);
                uint32_t jitter_range = ((strength >> 11) * (decimate_level >> 11)) >> 4;
                if (jitter_range < 2) jitter_range = 2;
                uint32_t jitter = (((fast_rand(rand_seed) >> 16) * jitter_range) >> 16);
                current_dec_step = base_step + jitter;
                if (current_dec_step > 32) current_dec_step = 32;
            } else {
                wetL = decL;
                wetR = decR;
            }
            dec_ctr++;

            static const int32_t dec_coef_lut[10] = {
                32767, 32767, 16384, 10922, 8192, 6553, 5461, 4681, 4096, 3640
            };
            int32_t dec_idx = 1 + (current_dec_step >> 2);
            if (dec_idx > 9) dec_idx = 9;
            int32_t dec_coef = dec_coef_lut[dec_idx];

            dec_lpL += (((int32_t)wetL - dec_lpL) * dec_coef) >> 15;
            dec_lpR += (((int32_t)wetR - dec_lpR) * dec_coef) >> 15;
            dec_lpL = clamp_i32(dec_lpL, -32768, 32767);
            dec_lpR = clamp_i32(dec_lpR, -32768, 32767);
            
            wetL = lerp_q15(pre_decL, (int16_t)dec_lpL, decimate_level);
            wetR = lerp_q15(pre_decR, (int16_t)dec_lpR, decimate_level);
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

        // Sputter / Crackle probability scales with Y (ringingXor) and strength (Main)
        if (sputter_prob > 0) {
            uint32_t roll = fast_rand(rand_seed) & 0x7FFF;

            if (sputter_timer > 0) {
                sputter_timer--;
                if (sputter_active) {
                    int16_t noise = (int16_t)(fast_rand(rand_seed) & 0xFFFF);
                    int32_t wet_amp = (wetL < 0 ? -wetL : wetL) + (wetR < 0 ? -wetR : wetR);
                    int32_t sputter_scale = wet_amp >> 4;
                    if (sputter_scale > 600) sputter_scale = 600;
                    out_wetL = (noise * (int16_t)sputter_scale) >> 15;
                    out_wetR = (noise * (int16_t)sputter_scale) >> 15;
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

        // ── Stage 5: Tape High-Cut Filter (LPF) ──
        if (tape_hicut > 0) {
            int32_t hicut_coef = 32768 - ((tape_hicut * 31768) >> 15);
            lp_newL += (((int32_t)out_wetL - lp_newL) * hicut_coef) >> 15;
            out_wetL = (int16_t)lp_newL;
            lp_newR += (((int32_t)out_wetR - lp_newR) * hicut_coef) >> 15;
            out_wetR = (int16_t)lp_newR;
        } else {
            lp_newL = out_wetL;
            lp_newR = out_wetR;
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
    // Contiguous ring buffer — 40960 samples total.
    // Stereo: L uses [0..20479], R uses [20480..40959]  (853 ms each @ 24 kHz)
    // Mono:   full [0..40959] used as one delay line    (1706 ms @ 24 kHz)
    int16_t  buf[40960];
    uint16_t wr = 0;   // wraps at BUF_HALF (20480) in stereo, BUF_FULL (40960) in mono

    // IIR-smoothed delay time (prevents zipper on rapid changes)
    int32_t  smooth_t = 8000;

    // Crossfade state for click-free large time jumps (sync mode division changes)
    int32_t  xfade_t    = 0;   // read position of the "old" head being faded out
    int32_t  xfade_gain = 0;   // 128 → 0: old weight / (128 − xfade_gain) = new weight

    // Wow & flutter LFO phase accumulator and smoothed depth
    uint16_t flutter_phase = 0;
    int32_t  smooth_flutter_depth = 0;

    // Feedback 1-pole LPF state variables
    int32_t  lp_feedback_L = 0;
    int32_t  lp_feedback_R = 0;

    // PT2399 clock decimation state variables (only used when time > 16384)
    uint32_t clk_phase = 0;
    int16_t  last_outL = 0;
    int16_t  last_outR = 0;
    int32_t  lp_outL = 0;
    int32_t  lp_outR = 0;

    DCBlocker dcL;
    DCBlocker dcR;

    static constexpr int32_t BUF_HALF = 20480;
    static constexpr int32_t BUF_FULL = 40960;

    void init() {
        memset(buf, 0, sizeof(buf));
        wr                   = 0;
        smooth_t             = 8000;
        xfade_t              = 0;
        xfade_gain           = 0;
        flutter_phase        = 0;
        smooth_flutter_depth = 0;
        lp_feedback_L        = 0;
        lp_feedback_R        = 0;
        clk_phase            = 0;
        last_outL            = 0;
        last_outR            = 0;
        lp_outL              = 0;
        lp_outR              = 0;
        dcL.init();
        dcR.init();
    }

    __attribute__((always_inline)) inline void process(int16_t inL, int16_t &outL, int16_t inR, int16_t &outR,
                 int32_t mainMix, int32_t time, int32_t feedback,
                 bool freeze, int32_t cv1Warp, int32_t cv2Corruption, int32_t globalNoiseScale = 16384,
                 bool pulse1_live = false, uint32_t clk_period_samples = 0,
                 bool mono_mode = false)
    {
        const int32_t buf_len = mono_mode ? BUF_FULL : BUF_HALF; // 40960 or 20480
        const int32_t max_t   = mono_mode ? 40800 : 20350;

        // ── PT2399 Slow-Clock Decimation (long delay times only, time > 16384) ──
        // At short times every sample is processed — this is essential for
        // ── PT2399 Slow-Clock Decimation (long delay times only, time > 24576 / 75% knob) ──
        // Short/medium delays (0..75% knob = 1..477ms) process every sample at full 24kHz bandwidth.
        // At long delays (>75% knob / >477ms) the clock is slowed to emulate lo-fi bucket-brigade chips.
        int32_t filter_coef = 32767; // reconstruction LPF coef; unity for short/medium times
        if (time > 24576) {
            int32_t diff     = time - 24576;
            uint32_t clk_inc = (uint32_t)(65536 - (diff * 4));
            filter_coef      = 10000 + (((int32_t)(clk_inc - 32768) * 22767) >> 15);

            clk_phase += clk_inc;
            if (clk_phase < 65536) {
                // Skipped sample — keep reconstruction LPF running
                lp_outL += (((int32_t)last_outL - lp_outL) * filter_coef) >> 15;
                lp_outR += (((int32_t)last_outR - lp_outR) * filter_coef) >> 15;
                outL = split_mix_q15(inL, (int16_t)lp_outL, (int16_t)mainMix);
                outR = split_mix_q15(inR, (int16_t)lp_outR, (int16_t)mainMix);
                return;
            }
            clk_phase -= 65536;
        }

        // ── Compute target delay time (quadratic taper expands short times across 25% of knob) ──
        int32_t time_sq  = (time * time) >> 15;
        int32_t target_t = 24 + ((time_sq * (max_t - 24)) >> 15);
        if (pulse1_live && clk_period_samples > 240) {
            // 12 rhythmic subdivisions (straight & dotted) mapped across knob
            static const int32_t div_num[12] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64};
            int32_t step = (time_sq * 12) >> 15;
            if (step < 0) step = 0;
            if (step > 11) step = 11;
            target_t = (clk_period_samples * div_num[step]) >> 4;
        }
        target_t = target_t + (cv1Warp * 4);
        target_t = clamp_i32(target_t, 24, max_t);

        if (mainMix < 50) {
            outL      = inL;  outR      = inR;
            last_outL = inL;  last_outR = inR;
            lp_outL   = inL;  lp_outR   = inR;
            if (!freeze) {
                if (mono_mode) {
                    // Mono: write one sample to the single buffer
                    int16_t in_mono = (int16_t)(((int32_t)inL + inR) >> 1);
                    buf[wr] = in_mono;
                } else {
                    buf[wr]            = inL;
                    buf[BUF_HALF + wr] = inR;
                }
                wr++; if (wr >= buf_len) wr = 0;
            }
            smooth_t   = target_t; // snap when dry so re-engage is glitch-free
            xfade_gain = 0;
            return;
        }

        // ── Time smoothing: crossfade on large jumps or clock division changes ──
        // Large jump on long delays (>1000 samples) or clocked division change:
        // snap to target & crossfade old read position to silence clicks.
        // Short delays (<1000 samples / KS range): smooth IIR tracking without crossfading
        // to keep string resonance alive during pitch sweeps.
        {
            int32_t delta = target_t - smooth_t;
            if (delta < 0) delta = -delta;

            if (xfade_gain > 0) {
                // Mid-crossfade: gently track target in case it keeps moving
                smooth_t += (target_t - smooth_t) >> 4;
            } else if ((smooth_t > 1000 || pulse1_live) && delta > 64) {
                // Large jump or clock division change — snap & launch crossfade from old position
                xfade_t    = smooth_t;
                xfade_gain = 128;   // counts down: 128=100% old → 0=100% new
                smooth_t   = target_t;
            } else {
                // Smooth tracking for continuous pitch sweeps (fast & responsive at short times)
                int32_t shift = (smooth_t < 1000) ? 3 : 4;
                smooth_t += (target_t - smooth_t) >> shift;
            }
        }

        // ── Wow & Flutter LFO (~2.2 Hz) ──
        flutter_phase += 6;
        int16_t lfo = lookup_sine_fast(flutter_phase);
        int32_t cv2_abs = cv2Corruption < 0 ? -cv2Corruption : cv2Corruption;
        int32_t cv2_warp = cv2_abs * 12;
        int32_t target_flutter_depth = ((feedback * 40) >> 15) + ((cv2_warp * 20) >> 15);
        smooth_flutter_depth += ((target_flutter_depth - smooth_flutter_depth) * 64) >> 10;

        // Flutter fades to zero below 300 samples — at KS/resonator range even
        // a tiny ±1 sample wobble is a large pitch deviation.  Above 1000 samples
        // (≈42 ms) full tape-style warble is allowed.
        int32_t flutter_scale;
        if (smooth_t < 300) {
            flutter_scale = 0;
        } else if (smooth_t < 1000) {
            flutter_scale = ((smooth_t - 300) * 32767) / 700;
        } else {
            flutter_scale = 32767;
        }
        int32_t flutter = ((lfo * smooth_flutter_depth) >> 15) * flutter_scale >> 15;

        // ── Fractional-sample read with linear interpolation ──
        // In mono mode both outputs read from the same (L-only) buffer.
        // In stereo mode L reads from buf[0..BUF_HALF-1], R from buf[BUF_HALF..BUF_FULL-1].
        auto read_stereo = [&](int32_t delay_q16, int16_t &rL, int16_t &rR) {
            int32_t rp_i = (int32_t)wr - (delay_q16 >> 16);
            if (rp_i < 0) {
                rp_i += buf_len;
                if (rp_i < 0) rp_i = 0;
            } else if (rp_i >= buf_len) {
                rp_i -= buf_len;
                if (rp_i >= buf_len) rp_i = 0;
            }
            int32_t rp_n = rp_i + 1;
            if (rp_n >= buf_len) rp_n = 0;
            uint16_t frac = (uint16_t)(delay_q16 & 0xFFFF);
            if (mono_mode) {
                int16_t y0 = buf[rp_i], y1 = buf[rp_n];
                int16_t val = lerp_delay_q15(y0, y1, frac);
                rL = val; rR = val;
            } else {
                int16_t y0 = buf[rp_i],            y1 = buf[rp_n];
                rL = lerp_delay_q15(y0, y1, frac);
                int16_t y0r = buf[BUF_HALF + rp_i], y1r = buf[BUF_HALF + rp_n];
                rR = lerp_delay_q15(y0r, y1r, frac);
            }
        };

        int32_t t1_q16 = (smooth_t + flutter) << 16;
        int32_t t2_q16 = ((smooth_t + flutter) * 3) << 14;
        int32_t t3_q16 = (smooth_t + flutter) * 40503;

        int16_t w1L, w1R, w2L, w2R, w3L, w3R;
        read_stereo(t1_q16, w1L, w1R);
        read_stereo(t2_q16, w2L, w2R);
        read_stereo(t3_q16, w3L, w3R);

        // ── Crossfade: blend old read head out over 128 samples ──
        // Eliminates click/pop when sync division changes jump the delay time.
        if (xfade_gain > 0) {
            int32_t old_wgt = xfade_gain;        // 128..1 → old fades out
            int32_t new_wgt = 128 - xfade_gain;  // 0..127 → new fades in
            int32_t t_old_q16 = (xfade_t + flutter) << 16;
            int16_t oldL, oldR;
            read_stereo(t_old_q16, oldL, oldR);
            w1L = (int16_t)(((int32_t)w1L * new_wgt + (int32_t)oldL * old_wgt) >> 7);
            w1R = (int16_t)(((int32_t)w1R * new_wgt + (int32_t)oldR * old_wgt) >> 7);
            xfade_gain--;
        }

        // ── Gain mix: single-tap below 512 samples (KS / resonator mode) ──
        // Multi-tap taps sit at 1× / 1.5× / 1.24× — at short times they land
        // on out-of-tune harmonics and destroy the resonance pitch.
        int32_t gain1, gain2L, gain2R, gain3L, gain3R;
        if (smooth_t < 512) {
            // KS mode: full gain on primary tap only
            gain1  = 32768;
            gain2L = gain2R = gain3L = gain3R = 0;
        } else {
            // Long echo: gradually blend secondary panned taps
            gain1  = 32768 - (feedback >> 1);
            gain2L = (feedback * 12000) >> 15;
            gain2R = (feedback *  4384) >> 15;
            gain3L = (feedback *  4384) >> 15;
            gain3R = (feedback * 12000) >> 15;
        }

        int16_t mixL = saturate_q15(
            ((int32_t)w1L * gain1 + (int32_t)w2L * gain2L + (int32_t)w3L * gain3L) >> 15);
        int16_t mixR = saturate_q15(
            ((int32_t)w1R * gain1 + (int32_t)w2R * gain2R + (int32_t)w3R * gain3R) >> 15);

        // ── Write to buffer (feedback path) ──
        if (!freeze) {
            // Direct loop feedback (L→L, R→R) for stable Karplus-Strong string synthesis
            int32_t feedL = (int32_t)inL + (((int32_t)w1L * feedback) >> 15);
            int32_t feedR = (int32_t)inR + (((int32_t)w1R * feedback) >> 15);
            feedL = soft_limit_q15(feedL);
            feedR = soft_limit_q15(feedR);

            // Dynamic 1-pole damping: keep bright at short times (damp_coef=28000) so short loops don't decay instantly
            int32_t damp_coef = 28000;
            if (smooth_t >= 500) {
                damp_coef = 15000 + (((smooth_t - 500) * 15000) >> 14);
                if (damp_coef > 30000) damp_coef = 30000;
            }
            lp_feedback_L += ((feedL - lp_feedback_L) * damp_coef) >> 15;
            lp_feedback_R += ((feedR - lp_feedback_R) * damp_coef) >> 15;
            lp_feedback_L = soft_limit_q15(lp_feedback_L);
            lp_feedback_R = soft_limit_q15(lp_feedback_R);

            // DC blocking + soft-clip: 0.9995 pole removes DC bias without draining AC audio energy per pass
            int16_t wrL = dcL.process((int16_t)lp_feedback_L, 32751);
            int16_t wrR = dcR.process((int16_t)lp_feedback_R, 32751);

            // Bipolar CV2 controls feedback XOR corruption (circuit bend!) scaled by global noise scale
            int32_t cv2_abs2 = cv2Corruption < 0 ? -cv2Corruption : cv2Corruption;
            int32_t corr = (cv2_abs2 * 8 * globalNoiseScale) >> 14;
            if (corr > 200) {
                uint16_t mask = (uint16_t)(corr >> 3);
                wrL ^= mask;
                wrR ^= mask;
            }

            if (mono_mode) {
                // Mono: write summed signal to the single buffer
                buf[wr] = (int16_t)(((int32_t)wrL + wrR) >> 1);
            } else {
                buf[wr]            = wrL;
                buf[BUF_HALF + wr] = wrR;
            }
            wr++;
            if (wr >= buf_len) wr = 0;
        }

        last_outL = mixL;
        last_outR = mixR;

        // Reconstruction LPF (unity coef at short times, LP-filtered at long times)
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


// Zoned speed determination helper. Placed in FLASH (not RAM) to save memory,
// since it is only called on grain boundaries/initialization, not per-sample.
inline int32_t determine_speed_zoned(int32_t sq, int32_t cv2_corr, uint32_t &seed, uint8_t arp_step, int32_t loop_len) {
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
            // Zone 3: Subtle detuned tape drift (advances step on every repeat, but stays close to unison)
            static const int32_t arp_seq[8] = {
                65536,         // 1x fwd (unison)
                66100,         // +15 cents
                65000,         // -14 cents
                65536,         // unison
                -65536,        // -1x rev (unison)
                -66100,        // -1x rev + 15 cents
                -65000,        // -1x rev - 14 cents
                -65536         // -1x rev (unison)
            };
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

    int32_t speed = base_speed;

    // Add micro pitch jitter under chaos
    int32_t eff_chaos = cv2_corr < 0 ? -cv2_corr : cv2_corr;
    if (eff_chaos > 200) {
        int32_t jitter = ((((int32_t)(fast_rand(seed) & 0x7FFF)) - 16384) * (eff_chaos >> 6)) >> 15; // up to ±5% speed drift
        speed += jitter;
    }

    if (speed == 0) speed = 3277;
    return speed;
}

struct GlitcherBlock {
    // Contiguous mu-law buffer — 65536 bytes total.
    // Stereo: L uses [0..32767], R uses [32768..65535]  (1.365 s each @ 24 kHz)
    // Mono:   full [0..65535] used as one loop buffer    (2.730 s @ 24 kHz)
    uint8_t  buf[65536];
    static constexpr int32_t BUF_HALF = 32768;
    static constexpr int32_t BUF_FULL = 65536;
    uint16_t wr = 0;

    bool     active     = false;
    uint16_t freeze_wr  = 0;       // write pointer at moment of freeze
    uint32_t frozen_clk_period = 0; // clock period captured at moment of freeze

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
    bool     last_freezeGate = false;
    int32_t  trigger_ctr = 0;
    int32_t  active_duration_ctr = 0;
    int32_t  refill_ctr = 0;
    uint32_t clock_pulse_counter = 0;
    uint32_t auto_chaos_phase = 0;

    void init() {
        memset(buf, 0, sizeof(buf));
        wr         = 0;
        active     = false;
        freeze_wr  = 0;
        last_freezeGate = false;
        rd_q16     = 0;
        auto_chaos_phase = 0;
        xfade_ctr  = 0;
        xfade_rd   = 0;
        xfade_len  = 256;
        xfade_phase = 0;
        xfade_step  = 0;
        current_loop_len = 512;
        sample_ctr = 0;
        active_duration_ctr = 0;
        refill_ctr = 0;
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
        frozen_clk_period = 0;
        trigger_ctr = 0;
        clock_pulse_counter = 0;
    }

    __attribute__((always_inline)) inline void process(int16_t inL, int16_t &outL, int16_t inR, int16_t &outR,
                 int32_t mainProb, int32_t size, int32_t speedQuant, bool isFreezePage,
                 bool glitchInjector, bool freezeGate, int32_t cv1Warp, int32_t cv2Corruption,
                 uint32_t &rand_seed, int32_t scrubOffset, int32_t glitchFeedback, int32_t globalNoiseScale,
                 bool pulse1_live, bool p1_rising, bool p1_gate,
                 bool pulse2_live, bool p2_rising, bool p2_gate,
                 uint32_t clk_period_samples, uint32_t clk_timer,
                 int32_t target_loop_size, int32_t target_offset, int32_t target_speed_q16,
                 bool mono_mode = false, bool dual_mono_mode = false)
    {
        const int32_t buf_mask = mono_mode ? (BUF_FULL - 1) : (BUF_HALF - 1);
        const int32_t buf_size = mono_mode ? BUF_FULL : BUF_HALF;

        // Helper: write one stereo sample into the appropriate buffer slot(s)
        auto write_buf = [&](uint16_t pos, int16_t sL, int16_t sR) {
            if (mono_mode) {
                buf[pos & buf_mask] = encode_mulaw((int16_t)(((int32_t)sL + sR) >> 1));
            } else {
                buf[pos & buf_mask]              = encode_mulaw(sL);
                buf[BUF_HALF + (pos & buf_mask)] = encode_mulaw(sR);
            }
        };

        bool is_clock_sync = pulse1_live && (clk_period_samples > 240);
        bool eff_glitchInjector = glitchInjector;
        bool want_active = eff_glitchInjector || freezeGate || p1_gate;
        bool is_loop_frozen = freezeGate || (mainProb >= 32760);
        int32_t speed_q16 = 65536;

        if (mainProb < 50 && !want_active && !active && dry_fade_ctr == 0) {
            outL = inL;
            outR = inR;
            write_buf(wr, inL, inR);
            wr = (wr + 1) & buf_mask;
            return;
        }
        if (is_clock_sync && mainProb < 50 && !active && dry_fade_ctr == 0) {
            outL = inL;
            outR = inR;
            write_buf(wr, inL, inR);
            wr = (wr + 1) & buf_mask;
            return;
        }

        // Update slow-moving cluster state (cutoff ~3Hz at 24kHz)
        cluster_timer++;
        if (cluster_timer >= 256) {
            cluster_timer = 0;
            if (dual_mono_mode) {
                rand_seed ^= 0x55555555u; // Decorrelate 2-channel random seed sequence in Dual Mono mode
            }
            cluster_state += (((int32_t)(fast_rand(rand_seed) & 0x7FFF)) - cluster_state) >> 5;
        }

        // Warp input probability curve cubicly for sparser, more musical triggering at medium knob settings.
        // If clocked, use a linear response so stutters trigger easily and feel responsive.
        int32_t warpedProb = 0;
        if (pulse1_live) {
            warpedProb = mainProb;
        } else {
            int32_t mainProbSq = (mainProb * mainProb) >> 15;
            warpedProb = (mainProbSq * mainProb) >> 15;
        }

        // Modulate probability with cluster state, scaling down depth near 0% and 100% knob
        int32_t mod_depth = (mainProb * (32767 - mainProb)) >> 14;
        int32_t cluster_mod = ((cluster_state - 16384) * mod_depth) >> 15;
        int32_t finalProb = warpedProb + cluster_mod;
        if (finalProb < 0) finalProb = 0;
        if (finalProb > 32767) finalProb = 32767;

        // ── FREEZE MODE ──────────────────────────────────────────────────────
        bool is_freezing = freezeGate || (pulse2_live && p2_gate);
        if (is_freezing) {
            if (!last_freezeGate) {
                active = false;
            }
            last_freezeGate = true;

            // 1. Lock recording and initialize freeze on transition
            if (!active) {
                active = true;
                int32_t target_wr = wr;
                if (pulse1_live && clk_period_samples > 240) {
                    target_wr = wr - (int32_t)clk_timer;
                    frozen_clk_period = clk_period_samples;
                } else {
                    frozen_clk_period = 0;
                }
                freeze_wr = target_wr & buf_mask;
                
                current_loop_len = target_loop_size;
                active_offset = target_offset;

                // Determine initial speed and rd_q16 direction
                current_speed_q16 = isFreezePage ? target_speed_q16 : 65536;
                speed_q16 = current_speed_q16;
                rd_q16 = (speed_q16 >= 0) ? 0 : ((int64_t)current_loop_len << 16);

                xfade_ctr = 0;
                dry_fade_ctr = 0;
                sample_ctr = 0;
                
                int32_t cur_xfade = current_loop_len < 2048 ? (current_loop_len >> 1) : 1024;
                if (cur_xfade < 4) cur_xfade = 4;
                onset_fade_len = cur_xfade;
                onset_fade_ctr = cur_xfade;
                onset_fade_step = (32767 << 15) / cur_xfade;
                onset_fade_phase = 0;
            }

            int32_t loop_size = target_loop_size;
            int32_t cur_xfade = loop_size < 512 ? (loop_size >> 1) : 256;
            if (cur_xfade < 4) cur_xfade = 4;

            speed_q16 = isFreezePage ? target_speed_q16 : 65536;

            int32_t loop_start = (((int32_t)freeze_wr - active_offset) & buf_mask) << 16;

            rd_q16 += speed_q16;
            sample_ctr++;

            // Natural boundary check
            bool crossed = false;
            if (speed_q16 >= 0) {
                if (rd_q16 >= ((int64_t)current_loop_len << 16)) {
                    crossed = true;
                }
            } else {
                if (rd_q16 < 0) {
                    crossed = true;
                }
            }
            if (pulse1_live && p1_rising) {
                clock_pulse_counter++;
                int32_t beats_needed = 1;
                int32_t abs_speed = speed_q16 < 0 ? -speed_q16 : speed_q16;
                if (abs_speed > 512) {
                    beats_needed = (int32_t)(((int64_t)current_loop_len * 65536) / ((int64_t)abs_speed * clk_period_samples));
                    if (beats_needed < 1) beats_needed = 1;
                }
                if (clock_pulse_counter >= (uint32_t)beats_needed) {
                    crossed = true;
                    clock_pulse_counter = 0;
                }
            }

            if (crossed) {
                clock_pulse_counter = 0;
                trig_out1 = true; // Output loop sync pulse

                // CD-skip buffer sliding under chaos:
                int32_t chaos_depth = (globalNoiseScale - 16384);
                if (chaos_depth < 0) chaos_depth = 0;
                int32_t abs_cv2 = cv2Corruption < 0 ? -cv2Corruption : cv2Corruption;
                chaos_depth += abs_cv2 * 8;
                if (chaos_depth > 2000) {
                    uint32_t slip_roll = fast_rand(rand_seed) & 0x7FFF;
                    int32_t slip_prob = (chaos_depth * 10) >> 15;
                    if ((int32_t)slip_roll < slip_prob) {
                        int32_t slip_samples = (((int32_t)(fast_rand(rand_seed) & 0x7FFF)) - 16384) >> 4;
                        freeze_wr = (freeze_wr + slip_samples) & buf_mask;
                    }
                }

                // Spawn next grain repeat at updated position/length targets
                xfade_rd = loop_start + rd_q16;
                rd_q16 = (speed_q16 >= 0) ? 0 : ((int64_t)current_loop_len << 16);
                xfade_len = cur_xfade;
                xfade_ctr = cur_xfade;
                xfade_step = (32767 << 15) / xfade_len;
                xfade_phase = 0;

                current_loop_len = loop_size;

                // ── AUTO-CHAOS DRIFT MODE: scrubOffset CCW (< 1600 / 5% Main Knob) ───────
                // Slowly wanders the read head across the circular buffer for continuous ambient textures.
                if (is_loop_frozen && scrubOffset < 1600) {
                    auto_chaos_phase += 16; // ~0.25Hz wander LFO @ 24kHz
                    int32_t wander = lookup_sine(auto_chaos_phase); // [-32768, 32767]
                    int32_t buf_cap = mono_mode ? BUF_FULL : BUF_HALF;
                    active_offset = (((wander + 32768) * (buf_cap - 1)) >> 16) & buf_mask;
                } else if (is_loop_frozen && scrubOffset > 30500) {
                    // ── SLICER MODE: scrubOffset fully right (> 30500) ──────────────
                    // On every loop boundary, jump to a new random position anywhere in
                    // the full frozen buffer — builds rhythmic stutters and buffer chops.
                    active_offset = (int32_t)(fast_rand(rand_seed) & buf_mask);
                } else {
                    // Normal freeze: track scrub position with 16-sample hysteresis and snap to zero crossing
                    int32_t diff = target_offset - active_offset;
                    if (diff < 0) diff = -diff;
                    if (diff > 16) {
                        // Search for the nearest zero crossing within a ±128 sample window around target_offset
                        int32_t best_dist = 999999;
                        int32_t best_offset = target_offset;
                        for (int d = -128; d <= 128; d++) {
                            int32_t test_offset = (target_offset + d) & buf_mask;
                            // Calculate circular buffer index relative to freeze_wr
                            int32_t idx = ((int32_t)freeze_wr - test_offset) & buf_mask;
                            int32_t nxt = (idx + 1) & buf_mask;
                            int16_t val0 = decode_mulaw(buf[idx]);
                            int16_t val1 = decode_mulaw(buf[nxt]);
                            
                            // Check for zero crossing
                            if ((val0 <= 0 && val1 > 0) || (val0 >= 0 && val1 < 0)) {
                                int32_t dist = d < 0 ? -d : d;
                                if (dist < best_dist) {
                                    best_dist = dist;
                                    best_offset = test_offset;
                                }
                            }
                        }
                        if (best_dist < 128) {
                            active_offset = best_offset;
                        } else {
                            active_offset = target_offset;
                        }
                    }
                }

                loop_start = (((int32_t)freeze_wr - active_offset) & buf_mask) << 16;
            }

            int16_t sL = 0, sR = 0;
            auto read_buf = [&](int32_t ptr, int16_t &valL, int16_t &valR) {
                int32_t  idx  = (ptr >> 16) & buf_mask;
                int32_t  nxt  = (idx + 1)   & buf_mask;
                uint16_t frac = (uint16_t)(ptr & 0xFFFF);
                if (mono_mode) {
                    int16_t y0 = decode_mulaw(buf[idx]), y1 = decode_mulaw(buf[nxt]);
                    int16_t val = lerp_delay_q15(y0, y1, frac);
                    valL = val; valR = val;
                } else {
                    int16_t y0L = decode_mulaw(buf[idx]),            y1L = decode_mulaw(buf[nxt]);
                    valL = lerp_delay_q15(y0L, y1L, frac);
                    int16_t y0R = decode_mulaw(buf[BUF_HALF + idx]), y1R = decode_mulaw(buf[BUF_HALF + nxt]);
                    valR = lerp_delay_q15(y0R, y1R, frac);
                }
            };

            read_buf(loop_start + rd_q16, sL, sR);
            current_g711_sample = buf[((int32_t)(loop_start + rd_q16) >> 16) & buf_mask];

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

            // Smooth microsound grain windowing for short grain windows (< 500 samples / ~20ms)
            // Bypassed when glitchFeedback >= 16384 to preserve self-oscillating Karplus-Strong string delays!
            if (current_loop_len < 500 && glitchFeedback < 16384) {
                int32_t grain_phase = (sample_ctr * 32767) / current_loop_len;
                if (grain_phase > 32767) grain_phase = 32767;
                int32_t win = lookup_sine(grain_phase);
                if (win < 0) win = 0;
                sL = (sL * win) >> 15;
                sR = (sR * win) >> 15;
            }

            // Glitcher Feedback Loop (disabled when frozen to prevent volume build-up)
            if (glitchFeedback > 0 && !freezeGate) {
                int32_t idx = ((loop_start + rd_q16) >> 16) & buf_mask;
                int32_t scaled_fb = (glitchFeedback * 29491) >> 15;
                int16_t oldL = decode_mulaw(buf[idx]);
                int16_t oldR = mono_mode ? oldL : decode_mulaw(buf[BUF_HALF + idx]);
                int16_t newL = soft_limit_q15(((int32_t)oldL * (32768 - scaled_fb) + (int32_t)sL * scaled_fb) >> 15);
                int16_t newR = soft_limit_q15(((int32_t)oldR * (32768 - scaled_fb) + (int32_t)sR * scaled_fb) >> 15);
                buf[idx] = encode_mulaw(newL);
                if (!mono_mode) buf[BUF_HALF + idx] = encode_mulaw(newR);
            }

            int32_t mix_coef = (mainProb * 5) >> 1;
            if (mix_coef > 32767) mix_coef = 32767;
            outL = lerp_q15(inL, sL, (int16_t)mix_coef);
            outR = lerp_q15(inR, sR, (int16_t)mix_coef);
            return;
        }

        // ── NORMAL GLITCH MODE ───────────────────────────────────────────────
        else {
            if (last_freezeGate) {
                last_freezeGate = false;
                if (active) {
                    active = false;
                    int32_t loop_start = (((int32_t)freeze_wr - active_offset) & buf_mask) << 16;
                    dry_fade_rd = loop_start + rd_q16;
                    dry_fade_len = 512;
                    dry_fade_ctr = 512;
                    dry_fade_step = (32767 << 15) / 512;
                    dry_fade_phase = 32767 << 15;
                    refill_ctr = 4096; // Set a 170ms glitch-free refill period to load fresh audio
                }
            }
            last_freezeGate = false;
            int32_t norm_loop_size = 512;
            if (pulse1_live && clk_period_samples > 240) {
                // Clock-synced subdivisions (straight & dotted: 13 steps)
                static const int32_t clk_div_num[13] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 128};
                int32_t num_steps = 13;
                int32_t size_sq = (size * size) >> 15;
                int32_t step = (size_sq * num_steps) >> 15;
                if (step < 0) step = 0;
                if (step > num_steps - 1) step = num_steps - 1;
                if (step == num_steps - 1) {
                    norm_loop_size = buf_size; // Max knob position ALWAYS freezes full buffer capacity!
                } else {
                    norm_loop_size = (clk_period_samples * clk_div_num[step]) / 16;
                }
                if (norm_loop_size < 128) norm_loop_size = 128;
                if (norm_loop_size > buf_size) norm_loop_size = buf_size;
            } else {
                if (cv1Warp == 0) {
                    static const int32_t size_lut[9] = {256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536};
                    int32_t num_steps = mono_mode ? 9 : 8;
                    int32_t step = (size * num_steps) >> 15;
                    if (step < 0) step = 0;
                    if (step > num_steps - 1) step = num_steps - 1;
                    norm_loop_size = size_lut[step];
                } else {
                    int32_t base_size = 128 + (size >> 1);
                    norm_loop_size = base_size + (cv1Warp * 4);
                }
            }
            norm_loop_size = clamp_i32(norm_loop_size, 128, buf_size);

            int32_t cur_xfade = norm_loop_size < 512 ? (norm_loop_size >> 1) : 256;
            if (cur_xfade < 4) cur_xfade = 4;

            if (!active) {
                write_buf(wr, inL, inR);

                bool boundary = false;
                if (pulse1_live && (clk_period_samples > 240)) {
                    boundary = p1_rising;
                } else {
                    trigger_ctr--;
                    if (trigger_ctr <= 0) {
                        trigger_ctr = norm_loop_size < 3072 ? 3072 : norm_loop_size;
                        boundary = true;
                    }
                }

                bool trigger = false;
                if (refill_ctr > 0) {
                    refill_ctr--;
                } else if (boundary) {
                    if (pulse1_live) {
                        trigger = (((fast_rand(rand_seed) & 0x7FFF) < (uint32_t)finalProb) || eff_glitchInjector);
                    } else {
                        bool immediate = (finalProb > 31000);
                        trigger = immediate || ((fast_rand(rand_seed) & 0x7FFF) < (uint32_t)finalProb) || eff_glitchInjector;
                    }
                } else if (eff_glitchInjector) {
                    trigger = true;
                }

                if (trigger) {
                    active = true;
                    freeze_wr = wr;
                    trigger_ctr = 0; // reset counter on trigger exit
                    active_duration_ctr = 0;
                    
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
                                static const int32_t size_lut[8] = {256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
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
                    if (speedQuant >= 30000) {
                        uint32_t r = fast_rand(rand_seed) & 7;
                        if (pulse1_live && clk_period_samples > 240) {
                            uint32_t r_div = ((fast_rand(rand_seed) & 0xFFFF) * 5) >> 16;
                            static const int32_t shifts[5] = {4, 3, 2, 1, 0};
                            final_size = clk_period_samples >> shifts[r_div];
                        } else {
                            static const int32_t size_lut[8] = {256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
                            final_size = size_lut[r];
                        }
                    } else if (speedQuant >= 19661) {
                        int32_t jitter_range = final_size >> 2;
                        if (jitter_range > 0) {
                            int32_t offset = (((int32_t)(fast_rand(rand_seed) & 0x7FFF) * (jitter_range * 2)) >> 15) - jitter_range;
                            final_size += offset;
                        }
                    }
                    current_loop_len = clamp_i32(final_size, 128, buf_size);
                    
                    arpeggio_step = 0;
                    if (is_loop_frozen) {
                        current_speed_q16 = isFreezePage ? target_speed_q16 : 65536;
                    } else {
                        current_speed_q16 = determine_speed_zoned(speedQuant, cv2Corruption, rand_seed, arpeggio_step, current_loop_len);
                    }
                    speed_q16 = current_speed_q16;
                    
                    rd_q16 = (speed_q16 >= 0) ? 0 : ((int64_t)current_loop_len << 16);
                    xfade_ctr = 0;
                    dry_fade_ctr = 0;
                    sample_ctr = 0;
                    
                    onset_fade_len = cur_xfade;
                    onset_fade_ctr = cur_xfade;
                    onset_fade_step = (32767 << 15) / cur_xfade;
                    onset_fade_phase = 0;
                }
                wr = (wr + 1) & buf_mask;
            }

            if (active) {
                active_duration_ctr++;
                auto read_buf = [&](int32_t ptr, int16_t &sL, int16_t &sR) {
                    int32_t  idx  = (ptr >> 16) & buf_mask;
                    int32_t  nxt  = (idx + 1)   & buf_mask;
                    uint16_t frac = (uint16_t)(ptr & 0xFFFF);
                    if (mono_mode) {
                        int16_t y0 = decode_mulaw(buf[idx]), y1 = decode_mulaw(buf[nxt]);
                        int16_t val = lerp_delay_q15(y0, y1, frac);
                        sL = val; sR = val;
                    } else {
                        int16_t y0L = decode_mulaw(buf[idx]),            y1L = decode_mulaw(buf[nxt]);
                        sL = lerp_delay_q15(y0L, y1L, frac);
                        int16_t y0R = decode_mulaw(buf[BUF_HALF + idx]), y1R = decode_mulaw(buf[BUF_HALF + nxt]);
                        sR = lerp_delay_q15(y0R, y1R, frac);
                    }
                };

                int32_t offset_samples = (scrubOffset * 32768) >> 15;
                int32_t lookback = current_loop_len;
                if (pulse1_live && clk_period_samples > 240) {
                    lookback = clk_period_samples;
                }
                if (is_loop_frozen) {
                    lookback = 0;
                }
                int32_t loop_start = (((int32_t)(freeze_wr) - lookback - offset_samples) & buf_mask) << 16;

                // Step arpeggiator on Pulse 2 rising edge (disabled when loop is frozen)
                if (!is_loop_frozen && pulse2_live && p2_rising) {
                    if (speedQuant >= 19661 && speedQuant < 26214) {
                        arpeggio_step++;
                        current_speed_q16 = determine_speed_zoned(speedQuant, cv2Corruption, rand_seed, arpeggio_step, current_loop_len);
                        trig_out2 = true;
                    }
                }

                if (isFreezePage) {
                    current_speed_q16 = target_speed_q16;
                } else if (is_loop_frozen) {
                    current_speed_q16 = 65536; // force unison pitch
                }
                speed_q16 = current_speed_q16;
                rd_q16 += speed_q16;
                sample_ctr++;

                bool crossed = false;
                if (speed_q16 >= 0) {
                    if (rd_q16 >= ((int64_t)current_loop_len << 16)) {
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

                    // CD-skip buffer sliding under chaos:
                    int32_t chaos_depth = (globalNoiseScale - 16384);
                    if (chaos_depth < 0) chaos_depth = 0;
                    int32_t abs_cv2 = cv2Corruption < 0 ? -cv2Corruption : cv2Corruption;
                    chaos_depth += abs_cv2 * 8;
                    if (chaos_depth > 2000) {
                        uint32_t slip_roll = fast_rand(rand_seed) & 0x7FFF;
                        int32_t slip_prob = (chaos_depth * 10) >> 15;
                        if ((int32_t)slip_roll < slip_prob) {
                            int32_t slip_samples = (((int32_t)(fast_rand(rand_seed) & 0x7FFF)) - 16384) >> 4;
                            freeze_wr = (freeze_wr + slip_samples) & buf_mask;
                        }
                    }

                    uint32_t roll = fast_rand(rand_seed) & 0x7FFF;
                    int32_t log2_size  = 31 - __builtin_clz((uint32_t)current_loop_len);
                    int32_t size_steps = log2_size - 7;
                    if (size_steps < 0) size_steps = 0;
                    if (size_steps > 7) size_steps = 7;
                    
                    // sf_lut now scales up with size: smaller loops repeat less to prevent buzzy chaos
                    static const int32_t sf_lut[8] = {12000, 15000, 18000, 21000, 24000, 27000, 30000, 32767};
                    int32_t size_factor = sf_lut[size_steps];
                    int32_t loop_prob = (finalProb * size_factor) >> 15;
                    
                    if (loop_prob > 28000 && finalProb < 28000) {
                        loop_prob = 28000; // Cap repeat probability at ~85% unless Main knob is turned high
                    }

                    if (is_clock_sync) {
                        if (current_loop_len < (int32_t)(clk_period_samples >> 1)) {
                            loop_prob = (finalProb * size_factor) >> 15;
                        } else {
                            loop_prob = finalProb;
                        }
                    }

                    if (finalProb >= 32760) {
                        loop_prob = 32767; // 100% loop probability at fully right (kind of freezes)
                    }
                    if (loop_prob > 32767) loop_prob = 32767;

                    bool keep_looping = (roll < (uint32_t)loop_prob) || eff_glitchInjector;
                    if (pulse1_live) {
                        if (is_clock_sync) {
                            if (p1_rising) {
                                keep_looping = (roll < (uint32_t)loop_prob);
                            } else {
                                // For smaller loop subdivisions (< 1/2 beat), allow early exit to avoid buzzy chaos
                                if (current_loop_len < (int32_t)(clk_period_samples >> 1)) {
                                    keep_looping = (roll < (uint32_t)loop_prob);
                                } else {
                                    keep_looping = true;
                                }
                            }
                        } else {
                            keep_looping = keep_looping || p1_gate;
                        }
                    }

                    // Enforce a minimum glitch loop duration of 128ms (3072 samples)
                    // so that very short loops do not trigger and instantly exit.
                    if (active_duration_ctr < 3072) {
                        keep_looping = true;
                    }

                    if (keep_looping) {
                        xfade_rd = loop_start + rd_q16;
                        
                        bool reroll_every_boundary = (speedQuant >= 19661) && !is_loop_frozen;
                        if (reroll_every_boundary || (sample_ctr >= 1024 && !is_loop_frozen)) {
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
                                    static const int32_t size_lut[8] = {256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
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
                        if (speedQuant >= 30000) {
                            uint32_t r = fast_rand(rand_seed) & 7;
                            if (pulse1_live && clk_period_samples > 240) {
                                uint32_t r_div = ((fast_rand(rand_seed) & 0xFFFF) * 5) >> 16;
                                static const int32_t shifts[5] = {4, 3, 2, 1, 0};
                                final_size = clk_period_samples >> shifts[r_div];
                            } else {
                                static const int32_t size_lut[8] = {256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
                                final_size = size_lut[r];
                            }
                        } else if (speedQuant >= 19661) {
                            int32_t jitter_range = final_size >> 2;
                            if (jitter_range > 0) {
                                int32_t offset = (((int32_t)(fast_rand(rand_seed) & 0x7FFF) * (jitter_range * 2)) >> 15) - jitter_range;
                                final_size += offset;
                            }
                        }
                        current_loop_len = clamp_i32(final_size, 128, buf_size);

                        if (speed_q16 >= 0) {
                            rd_q16 = 0;
                        } else {
                            rd_q16 = ((int64_t)current_loop_len << 16);
                        }
                        xfade_len = cur_xfade;
                        xfade_ctr = cur_xfade;
                        xfade_step = (32767 << 15) / cur_xfade;
                        xfade_phase = 0;
                        
                        int32_t active_lookback = current_loop_len;
                        if (is_loop_frozen) {
                            active_lookback = 0;
                        }
                        loop_start = (((int32_t)freeze_wr - active_lookback - offset_samples) & buf_mask) << 16;
                    } else {
                        active = false;
                        trigger_ctr = norm_loop_size < 3072 ? 3072 : norm_loop_size;
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
                    current_g711_sample = buf[((int32_t)(loop_start + rd_q16) >> 16) & buf_mask];

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

                    if (glitchFeedback > 0 && !freezeGate) {
                        int32_t idx = ((loop_start + rd_q16) >> 16) & buf_mask;
                        int32_t scaled_fb = (glitchFeedback * 29491) >> 15;
                        int16_t oldL = decode_mulaw(buf[idx]);
                        int16_t oldR = mono_mode ? oldL : decode_mulaw(buf[BUF_HALF + idx]);
                        int16_t newL = soft_limit_q15(((int32_t)oldL * (32768 - scaled_fb) + (int32_t)sL * scaled_fb) >> 15);
                        int16_t newR = soft_limit_q15(((int32_t)oldR * (32768 - scaled_fb) + (int32_t)sR * scaled_fb) >> 15);
                        buf[idx] = encode_mulaw(newL);
                        if (!mono_mode) buf[BUF_HALF + idx] = encode_mulaw(newR);
                    }

                    int32_t mix_coef = (mainProb * 5) >> 1;
                    if (mix_coef > 32767) mix_coef = 32767;
                    outL = lerp_q15(inL, sL, (int16_t)mix_coef);
                    outR = lerp_q15(inR, sR, (int16_t)mix_coef);
                    return;
                }
            }
        }

        if (dry_fade_ctr > 0) {
            auto read_buf = [&](int32_t ptr, int16_t &sL, int16_t &sR) {
                int32_t  idx  = (ptr >> 16) & buf_mask;
                int32_t  nxt  = (idx + 1)   & buf_mask;
                uint16_t frac = (uint16_t)(ptr & 0xFFFF);
                if (mono_mode) {
                    int16_t y0 = decode_mulaw(buf[idx]), y1 = decode_mulaw(buf[nxt]);
                    int16_t val = lerp_delay_q15(y0, y1, frac);
                    sL = val; sR = val;
                } else {
                    int16_t y0L = decode_mulaw(buf[idx]),            y1L = decode_mulaw(buf[nxt]);
                    sL = lerp_delay_q15(y0L, y1L, frac);
                    int16_t y0R = decode_mulaw(buf[BUF_HALF + idx]), y1R = decode_mulaw(buf[BUF_HALF + nxt]);
                    sR = lerp_delay_q15(y0R, y1R, frac);
                }
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
struct FilterBlock {
    // Filter state — Q15 units but promoted to int32_t for headroom
    int32_t v1L = 0, v2L = 0;
    int32_t v1R = 0, v2R = 0;

    // Smoothed coefficients/parameters in Q15
    int32_t sm_g = 1000;
    int32_t sm_r = 8192;
    int32_t sm_cutoff = 16384;
    int32_t sm_grit = 0;

    // Output lowpass warmth filter states
    int32_t lp_outL = 0, lp_outR = 0;

    // DC blockers to strip offset before sending to reverb/outputs
    DCBlocker dcL;
    DCBlocker dcR;

    // Compressor envelope follower state
    int32_t env = 0;

    inline int32_t filter_saturate(int32_t x) {
        // Stabilize and soft-saturate using a division-free cubic curve: y = x - x^3 / 6
        int32_t clamped_x = x;
        if (clamped_x > 32767) clamped_x = 32767;
        int32_t x3 = (((clamped_x * clamped_x) >> 15) * clamped_x) >> 15;
        // x3 * 5461 >> 15 is x^3 / 6 (since 5461/32768 ≈ 1/6)
        int32_t y = clamped_x - ((x3 * 5461) >> 15);
        return y;
    }

    void init() {
        v1L = v2L = v1R = v2R = 0;
        sm_g = 1000;
        sm_r = 8192;
        sm_cutoff = 16384;
        sm_grit = 0;
        lp_outL = lp_outR = 0;
        dcL.init();
        dcR.init();
        env = 0;
    }

    __attribute__((always_inline)) inline void process(int16_t inL, int16_t &outL, int16_t inR, int16_t &outR,
                 int32_t cutoff, int32_t resonance, int32_t grit_param,
                 int32_t cv1Warp, int32_t cv2Corruption = 0)
    {
        (void)cv1Warp;
        (void)cv2Corruption;

        // Smooth control parameters to prevent clicking/plopping on fast sweeps
        IIR_SMOOTH(sm_cutoff, cutoff, 8);
        IIR_SMOOTH(sm_grit, grit_param, 8);

        int32_t eff_cutoff = sm_cutoff;
        int32_t eff_grit = sm_grit;

        // ── Cutoff and Mode Mapping ──────────────────────────────────────────
        int32_t target_g = 300;
        if (eff_cutoff < 16384) {
            int32_t ratio = (eff_cutoff * 32767) >> 14;
            if (ratio > 32767) ratio = 32767;
            int32_t quad_part = (ratio * ratio) >> 15;
            int32_t mixed = (ratio * 14000 + quad_part * 18768) >> 15;
            target_g = 300 + mixed;
        } else {
            int32_t ratio = ((eff_cutoff - 16384) * 32767) >> 14;
            if (ratio > 32767) ratio = 32767;
            int32_t quad_part = (ratio * ratio) >> 15;
            int32_t mixed = (ratio * 14000 + quad_part * 18768) >> 15;
            target_g = 300 + mixed;
        }
        target_g = clamp_i32(target_g, 300, 22000);

        // ── Resonance Damping Mapping ────────────────────────────────────────
        // Normal range: map [0..28000] to damping [32000..4500] (highly resonant).
        // Self-oscillation: above 28000, fade damping down to 0 (infinite Q) for clean sine waves.
        int32_t target_r;
        if (resonance < 28000) {
            target_r = 32000 - ((resonance * 27500) >> 15);
        } else {
            int32_t diff = resonance - 28000; // 0..4767
            target_r = 4500 - ((diff * 4500) / 4767);
        }
        target_r = clamp_i32(target_r, 0, 32000);

        // ── Grit / Folder Mapping (Y knob) ───────────────────────────────────
        // Swept parameters (drive, fold_mix, fold_gain, fuzz_gain) are calculated continuously
        // inside the processing stage below to ensure pop-free, gradual timbre morphs.

        // Output volume compensation to keep perceived loudness stable as grit increases (scales down to 50%)
        int32_t grit_volume_scale = 32768 - (eff_grit / 2);

        // Smooth coefficients to eliminate zipper noise
        IIR_SMOOTH(sm_g, target_g, 8);
        IIR_SMOOTH(sm_r, target_r, 8);

        int32_t g = sm_g;
        int32_t r = sm_r;

        // Compensate for self-oscillation volume spike by slightly dipping the gain at high resonance
        int32_t gain_q15 = 16384 - ((32000 - r) >> 4);
        gain_q15 = clamp_i32(gain_q15, 13000, 16384);

        // ── Volume Fade at Extremes ──────────────────────────────────────────
        // ── Volume Fade at Extremes ──────────────────────────────────────────
        int32_t volume_scale = 32768; // Q15
        if (eff_cutoff < 512) {
            volume_scale = eff_cutoff << 6;
        } else if (eff_cutoff > 32255) {
            volume_scale = (32767 - eff_cutoff) << 6;
        }

        // ── 1. Drive & Distortion Stage ──────────────────────────────────────
        int16_t distL = inL;
        int16_t distR = inR;
        if (eff_grit > 0) {
            // Calculate continuous parameter sweeps based on eff_grit
            // 1. Drive sweeps from 1.0x (32768) to 3.0x (98304) to prevent severe digital noise
            int32_t drive = 32768 + (eff_grit * 2);

            // 2. Folder Mix sweeps from 0% (at 10000) to 100% (at 22000)
            int32_t fold_mix = 0;
            if (eff_grit >= 10000) {
                if (eff_grit < 22000) {
                    fold_mix = ((eff_grit - 10000) * 22370) >> 13;
                } else {
                    fold_mix = 32768;
                }
            }

            // 3. Folder Gain sweeps from 128 (0.5x) to 384 (1.5x) (moderate Buchla-style folding folds)
            int32_t fold_gain_q8 = 128;
            if (eff_grit >= 10000) {
                fold_gain_q8 = 128 + (((eff_grit - 10000) * 256) / 22767);
            }

            // 4. Fuzz Gain sweeps from 1.0x (32768) to 2.0x (65536) for clean soft-saturating overdrive
            int32_t fuzz_gain = 32768;
            if (eff_grit >= 22000) {
                int32_t diff = eff_grit - 22000;
                int32_t ratio = (diff * 24931) >> 13; // 0..32767
                int32_t ratio_sq = (ratio * ratio) >> 15; // 0..32767 (quadratic curve)
                fuzz_gain = 32768 + ratio_sq; // sweeps 32768 to 65535 (2.0x)
            }

            // Drive inputs
            int32_t drive_xL = ((int32_t)distL * drive) >> 15;
            int32_t drive_xR = ((int32_t)distR * drive) >> 15;

            // Calculate soft overdrive (Stage 1)
            int16_t odL = soft_limit_q15(drive_xL);
            int16_t odR = soft_limit_q15(drive_xR);

            // Calculate wavefolder (Stage 2)
            int32_t phaseL = (drive_xL * fold_gain_q8) >> 8;
            int32_t phaseR = (drive_xR * fold_gain_q8) >> 8;
            int16_t foldL = lookup_sine((uint16_t)phaseL);
            int16_t foldR = lookup_sine((uint16_t)phaseR);

            // Apply fuzz gain boost if we are in the fuzz region (Stage 3)
            if (eff_grit >= 22000) {
                int16_t fL = soft_limit_q15(((int32_t)foldL * fuzz_gain) >> 15);
                int16_t fR = soft_limit_q15(((int32_t)foldR * fuzz_gain) >> 15);

                int32_t fuzz_diff = eff_grit - 22000;
                if (fuzz_diff < 2048) {
                    int32_t fuzz_blend = fuzz_diff << 4; // morph range is exactly 2048 samples
                    foldL = lerp_q15(foldL, fL, fuzz_blend);
                    foldR = lerp_q15(foldR, fR, fuzz_blend);
                } else {
                    foldL = fL;
                    foldR = fR;
                }
            }

            // Linearly interpolate between soft overdrive and wavefolded/fuzzed signal
            distL = lerp_q15(odL, foldL, fold_mix);
            distR = lerp_q15(odR, foldR, fold_mix);
        }

        // ── 2. Post-Distortion Compressor Stage ──────────────────────────────
        if (eff_grit > 0) {
            int32_t absL = distL < 0 ? -distL : distL;
            int32_t absR = distR < 0 ? -distR : distR;
            int32_t peak = absL > absR ? absL : absR;
            if (peak > 32767) peak = 32767;

            // attack time ~2.5ms, release time ~40ms
            int32_t attack_shift = 5;
            int32_t release_shift = 9;
            if (peak > env) env += (peak - env) >> attack_shift;
            else env += (peak - env) >> release_shift;

            // Threshold sweeps from 32767 down to 12767 as grit increases
            int32_t thresh = 32767 - ((eff_grit * 20000) >> 15);
            // Compression slope sweeps up from 0 to 15000 (around 1.5:1 ratio)
            int32_t slope = (eff_grit * 15000) >> 15;

            int32_t gain_coef = 32768; // Q15
            if (env > thresh) {
                int32_t overshoot = env - thresh;
                int32_t gain_reduction = ((int32_t)overshoot * slope) >> 15;
                gain_coef = 32768 - gain_reduction;
                if (gain_coef < 18000) gain_coef = 18000; // max ~5.2dB gain reduction for constant volume
            }

            distL = ((int32_t)distL * gain_coef) >> 15;
            distR = ((int32_t)distR * gain_coef) >> 15;
        }

        // ── 3. SVF Filtering Stage (Sweeps the distorted sound!) ─────────────
        // Left Channel
        int32_t feedbackL = ((r * v1L) >> 15) + v2L;
        int32_t hpL = (int32_t)distL - filter_saturate(feedbackL);
        v1L       += (g * hpL) >> 15;
        v1L        = soft_limit_q15(v1L);
        int32_t lpL = v2L + ((g * v1L) >> 15);
        v2L        = lpL;
        v2L        = soft_limit_q15(v2L);

        int16_t lp16L = saturate_q15(lpL);
        int16_t hp16L = saturate_q15(hpL);
        int16_t morphedL = (eff_cutoff < 16384) ? lp16L : hp16L;

        if (eff_cutoff < 16384) {
            if (eff_cutoff > 14336) {
                int32_t dry_ratio = (eff_cutoff - 14336) << 4;
                morphedL = lerp_q15(morphedL, distL, dry_ratio);
            }
        } else {
            if (eff_cutoff < 18432) {
                int32_t dry_ratio = (18432 - eff_cutoff) << 4;
                morphedL = lerp_q15(morphedL, distL, dry_ratio);
            }
        }

        // Right Channel
        int32_t feedbackR = ((r * v1R) >> 15) + v2R;
        int32_t hpR = (int32_t)distR - filter_saturate(feedbackR);
        v1R       += (g * hpR) >> 15;
        v1R        = soft_limit_q15(v1R);
        int32_t lpR = v2R + ((g * v1R) >> 15);
        v2R        = lpR;
        v2R        = soft_limit_q15(v2R);

        int16_t lp16R = saturate_q15(lpR);
        int16_t hp16R = saturate_q15(hpR);
        int16_t morphedR = (eff_cutoff < 16384) ? lp16R : hp16R;

        if (eff_cutoff < 16384) {
            if (eff_cutoff > 14336) {
                int32_t dry_ratio = (eff_cutoff - 14336) << 4;
                morphedR = lerp_q15(morphedR, distR, dry_ratio);
            }
        } else {
            if (eff_cutoff < 18432) {
                int32_t dry_ratio = (18432 - eff_cutoff) << 4;
                morphedR = lerp_q15(morphedR, distR, dry_ratio);
            }
        }

        // ── 4. Warmth Smoothing & DC Blocker ─────────────────────────────────
        // Left Channel
        {
            // Gentle post-distortion lowpass filter to smooth out harsh high harmonics
            int32_t lp_coef = 32768 - ((eff_grit * 12000) >> 15); // sweeps from 32768 down to 20768
            lp_outL += (((int32_t)morphedL - lp_outL) * lp_coef) >> 15;
            int16_t finalL = dcL.process((int16_t)lp_outL);

            int32_t out32 = saturate_q15(((int32_t)finalL * gain_q15) >> 14);
            out32 = (out32 * grit_volume_scale) >> 15;
            outL = (out32 * volume_scale) >> 15;
        }

        // Right Channel
        {
            // Gentle post-distortion lowpass filter to smooth out harsh high harmonics
            int32_t lp_coef = 32768 - ((eff_grit * 12000) >> 15); // sweeps from 32768 down to 20768
            lp_outR += (((int32_t)morphedR - lp_outR) * lp_coef) >> 15;
            int16_t finalR = dcR.process((int16_t)lp_outR);

            int32_t out32 = saturate_q15(((int32_t)finalR * gain_q15) >> 14);
            out32 = (out32 * grit_volume_scale) >> 15;
            outR = (out32 * volume_scale) >> 15;
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

            int16_t dIn  = lerp_delay_q15(bufIn[rd1],  bufIn[rd2],  frac);
            int16_t dOut = lerp_delay_q15(bufOut[rd1], bufOut[rd2], frac);

            int32_t interm = ((int32_t)(g * in) >> 15) + dIn - ((int32_t)(dOut * g) >> 15);
            int16_t out = soft_limit_q15(interm);
            bufIn[ptr] = in;
            bufOut[ptr] = out;
            ptr = (ptr + 1) & mask;
            return out;
        }

        inline int16_t process_fixed(int16_t in) {
            uint16_t rd = (ptr - len) & mask;
            int16_t dIn  = bufIn[rd];
            int16_t dOut = bufOut[rd];

            int32_t interm = ((int32_t)(g * in) >> 15) + dIn - ((int32_t)(dOut * g) >> 15);
            int16_t out = soft_limit_q15(interm);
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

        inline int16_t read_integer(int32_t len_scale) {
            int32_t scaled_len = (len_scale * len) >> 15;
            if (scaled_len < 1) scaled_len = 1;
            if (scaled_len >= mask) scaled_len = mask - 1;
            uint16_t rd = (ptr - scaled_len) & mask;
            return buf[rd];
        }
    };

    int16_t mem[28672];
    AP apIn[4], apTankL, apTankR;
    Delay modL, d1L, d2L, modR, d1R, d2R;
    int32_t lpL = 0, lpR = 0, lpIn = 0;
    uint32_t lfo_phase1 = 0, lfo_phase2 = 0, lfo_phase3 = 0, lfo_phase4 = 0;
    int32_t lp_size_scale = 32767;

    // Decimation state for glitch effect
    uint16_t decimate_phase = 0;
    int16_t last_outL = 0;
    int16_t last_outR = 0;
    int32_t dec_lpL = 0, dec_lpR = 0;

    // Input soft-limiter: fast-attack / slow-release gain reduction to absorb
    // clipped transients and pops gracefully instead of hard-slamming the tank.
    int32_t limiter_gain = 32767; // Q15 gain applied to tank input (32767 = unity)

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
        lfo_phase1 = 0;
        lfo_phase2 = 16384 << 16;
        lfo_phase3 = 32768 << 16;
        lfo_phase4 = 49152 << 16;
        decimate_phase = 0;
        last_outL = 0;
        last_outR = 0;
        dec_lpL = 0;
        dec_lpR = 0;
        limiter_gain = 32767;
        dc_loopL.init();
        dc_loopR.init();
        lp_size_scale = 32767;
    }

    __attribute__((always_inline)) inline void process(int16_t &L, int16_t &R, int32_t mix, int32_t size_scale,
                 int32_t decay, int32_t damp, int32_t lofi_level,
                 int32_t sparkle_level, int32_t circuit_bent_level,
                 int32_t lofi_shift, int32_t lofi_frac, int32_t reverb_mode = 0,
                 bool dual_mono_mode = false) {
        if (mix < 50) {
            return;
        }
        // Modulate delay times independently using 4 slow, prime-spaced LFOs (speeds ~0.4Hz to 0.9Hz) (doubled for 24kHz)
        lfo_phase1 += 122;
        lfo_phase2 += 158;
        lfo_phase3 += 194;
        lfo_phase4 += 226;

        auto get_tri = [](uint32_t phase) -> int32_t {
            uint16_t ph = (uint16_t)(phase >> 16);
            int32_t tmp = (ph < 32768) ? ((ph << 1) - 32768) : (32767 - ((ph - 32768) << 1));
            return tmp; // [-32768, 32767]
        };

        // Jitter/modulate the size scales independently (depth = ~150 Q15 units, approx 0.45% size)
        int32_t mod1 = (get_tri(lfo_phase1) * 150) >> 15;
        int32_t mod2 = (get_tri(lfo_phase2) * 150) >> 15;
        int32_t mod3 = (get_tri(lfo_phase3) * 150) >> 15;
        int32_t mod4 = (get_tri(lfo_phase4) * 150) >> 15;

        // Smooth size scale slowly to eliminate pitch-glide howling when changing room sizes
        lp_size_scale += (size_scale - lp_size_scale) >> 11; // ~85ms time constant

        int32_t scale_d1L = lp_size_scale + mod1;
        int32_t scale_d2L = lp_size_scale + mod2;
        int32_t scale_d1R = lp_size_scale + mod3;
        int32_t scale_d2R = lp_size_scale + mod4;
        int32_t scale_modL = lp_size_scale + ((mod1 + mod2) >> 1);
        int32_t scale_modR = lp_size_scale + ((mod3 + mod4) >> 1);

        // Apply Address Jitter / Read Head Flutter in circuit-bent mode to left/right channels
        if (circuit_bent_level > 0) {
            uint32_t r = fast_rand(rand_seed);
            scale_d1L -= ((r & 0xFF) * circuit_bent_level) >> 13;
            scale_d2L -= (((r >> 8) & 0xFF) * circuit_bent_level) >> 13;
            scale_d1R -= (((r >> 16) & 0xFF) * circuit_bent_level) >> 13;
            scale_d2R -= (((r >> 24) & 0xFF) * circuit_bent_level) >> 13;
        }

        auto clamp_scale = [](int32_t s) -> int32_t {
            if (s < 3276) return 3276;
            if (s > 32767) return 32767;
            return s;
        };

        scale_d1L = clamp_scale(scale_d1L);
        scale_d2L = clamp_scale(scale_d2L);
        scale_d1R = clamp_scale(scale_d1R);
        scale_d2R = clamp_scale(scale_d2R);
        scale_modL = clamp_scale(scale_modL);
        scale_modR = clamp_scale(scale_modR);

        // Send level is scaled by mix so at low mix settings less signal feeds the tank
        int16_t monoL = dual_mono_mode ? L : (int16_t)(((int32_t)L + (int32_t)R) >> 1);
        int16_t monoR = dual_mono_mode ? R : (int16_t)(((int32_t)L + (int32_t)R) >> 1);
        {
            int32_t send = (mix < 16384) ? (mix * 2) : 32767;
            monoL = (int16_t)(((int32_t)monoL * send) >> 15);
            monoR = (int16_t)(((int32_t)monoR * send) >> 15);
        }


        // Input soft-limiter — prevents clipped transients / pops from slamming the tank
        {
            int32_t peakL = (int32_t)monoL < 0 ? -(int32_t)monoL : (int32_t)monoL;
            int32_t gain_target = 32767;
            if (peakL > 28000) {
                gain_target = (28000 * 32767) / peakL;
            }
            if (gain_target < limiter_gain) {
                limiter_gain += (gain_target - limiter_gain) >> 3;
            } else {
                limiter_gain += (gain_target - limiter_gain) >> 11;
            }
            monoL = (int16_t)(((int32_t)monoL * limiter_gain) >> 15);
            monoR = (int16_t)(((int32_t)monoR * limiter_gain) >> 15);
        }

        // ── Spring Reverb Dispersion & Drip (OP-1 Style Dual Spring Tank) ────────
        if (reverb_mode == 1) {
            // High-pass filter to cut sub-bass rumble from spring tank (< 180Hz)
            monoL = dc_loopL.process(monoL);
            monoR = dc_loopR.process(monoR);

            // Left Spring Coil Dispersion (high-density chirp)
            int32_t sp1L = ((int32_t)monoL * 24000) >> 15;
            int32_t sp2L = ((int32_t)sp1L * 20000) >> 15;
            monoL = saturate_q15(monoL + sp1L - sp2L);

            // Right Spring Coil Dispersion (prime offset chirp for wide 3D stereo drip)
            int32_t sp1R = ((int32_t)monoR * 20000) >> 15;
            int32_t sp2R = ((int32_t)sp1R * 25000) >> 15;
            monoR = saturate_q15(monoR + sp1R - sp2R);
            
            // Add asymmetric 14Hz spring coil wobble/flutter modulation
            scale_modL += (get_tri(lfo_phase1 * 3) * 450) >> 15;
            scale_modR += (get_tri(lfo_phase3 * 3 + 16384) * 450) >> 15;
        }

        // Input all-passes are kept at fixed scale to prevent pitch-glide in the diffusion network
        int16_t monoL_ap = apIn[0].process_fixed(monoL);
        monoL_ap = apIn[1].process_fixed(monoL_ap);

        int16_t monoR_ap = apIn[2].process_fixed(monoR);
        monoR_ap = apIn[3].process_fixed(monoR_ap);

        // Read tank loop outputs using independent scales
        int16_t tOutL = d2L.read_integer(scale_d2L);
        int16_t tOutR = d2R.read_integer(scale_d2R);

        // Isolate Left and Right reverb tank feedback in Dual Mono Mode
        int16_t fb_L = dual_mono_mode ? tOutL : tOutR;
        int16_t fb_R = dual_mono_mode ? tOutR : tOutL;

        // Left Tank
        int32_t iL = (int32_t)monoL_ap + (((int32_t)decay * fb_L) >> 15);
        int16_t iL_soft = soft_limit_q15(iL);
        int16_t sL = iL_soft + (int16_t)((16384 * modL.read_integer(scale_modL)) >> 15);
        modL.write(soft_limit_q15((int32_t)iL_soft - (int16_t)((16384 * sL) >> 15)));
        d1L.write(sL);
        sL = d1L.read_integer(scale_d1L);
        lpL += (((int32_t)sL - lpL) * damp) >> 15;
        lpL = soft_limit_q15(lpL);
        sL = (int16_t)lpL;
        sL = apTankL.process_fixed(sL);
        d2L.write(sL);

        // Right Tank
        int32_t iR = (int32_t)monoR_ap + (((int32_t)decay * fb_R) >> 15);
        int16_t iR_soft = soft_limit_q15(iR);
        int16_t sR = iR_soft + (int16_t)((16384 * modR.read_integer(scale_modR)) >> 15);
        modR.write(soft_limit_q15((int32_t)iR_soft - (int16_t)((16384 * sR) >> 15)));
        d1R.write(sR);
        sR = d1R.read_integer(scale_d1R);
        lpR += (((int32_t)sR - lpR) * damp) >> 15;
        lpR = soft_limit_q15(lpR);
        sR = (int16_t)lpR;
        sR = apTankR.process_fixed(sR);
        d2R.write(sR);

        int32_t wetL = sL;
        int32_t wetR = sR;

        // Apply Bitcrushing and XOR Scrambling OUTSIDE the feedback loop to keep the reverb tail natural and long
        // 1. Continuous Bitcrushing (word-length truncation) based on lofi_level
        if (lofi_level > 0) {
            int16_t q1L = (wetL >> lofi_shift) << lofi_shift;
            int16_t q2L = (wetL >> (lofi_shift + 1)) << (lofi_shift + 1);
            wetL = lerp_q15(q1L, q2L, lofi_frac);

            int16_t q1R = (wetR >> lofi_shift) << lofi_shift;
            int16_t q2R = (wetR >> (lofi_shift + 1)) << (lofi_shift + 1);
            wetR = lerp_q15(q1R, q2R, lofi_frac);
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
