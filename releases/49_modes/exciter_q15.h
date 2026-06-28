// =============================================================================
// exciter_q15.h — Fixed-point Exciter for Elements port
//
// Port of elements::Exciter to Q15 integer arithmetic.
// Generates excitation signals that drive the resonator/string models.
//
// Available models:
//   GRANULAR  — Granular sample player from flash noise texture.
//   SAMPLE    — Sample player: 9 percussion samples crossfaded from flash.
//   MALLET    — Single impulse filtered through LP. Classic percussion.
//   PLECTRUM  — Delayed snap after initial dip. Guitar-pick feel.
//   PARTICLES — Stochastic impulse cloud with evolving density.
//   FLOW      — Chaotic particle-based wind/bow noise.
//   NOISE     — Filtered white noise (resonant if parameter > 0).
//
// Sample data lives in flash via XIP (__in_flash()) — zero RAM cost.
//
// Original code: Émilie Gillet (Mutable Instruments) — MIT License
// =============================================================================

#ifndef EXCITER_Q15_H_
#define EXCITER_Q15_H_

#include <stdint.h>
#include <string.h>
#include "dsp_q15.h"
#include "svf_q15.h"
#include "resources_q15.h"
#include "samples_flash.h"

// ── Exciter Model (order matches original for meta sweep) ───────────────────
enum ExciterModelQ15 {
    EXCITER_Q15_GRANULAR  = 0,  // Granular sample player (noise texture)
    EXCITER_Q15_SAMPLE    = 1,  // Sample player (9 percussion hits)
    EXCITER_Q15_MALLET    = 2,
    EXCITER_Q15_PLECTRUM  = 3,
    EXCITER_Q15_PARTICLES = 4,
    EXCITER_Q15_FLOW      = 5,
    EXCITER_Q15_NOISE     = 6,
    EXCITER_Q15_NUM_MODELS
};

// ── Exciter ─────────────────────────────────────────────────────────────────

struct ExciterQ15 {
    ExciterModelQ15 model;
    int32_t parameter;   // Q15: model-specific parameter
    int32_t timbre;      // Q15: brightness / filter cutoff
    int32_t signature;   // Q15: random seed variation
    
    SvfQ15 lp;           // Output low-pass filter
    
    int32_t damp_state;  // Q15: damping accumulator
    int32_t damping;     // Q15: output damping (for palm mute effect)
    
    // Particle state
    int32_t particle_state;  // Q15
    int32_t particle_range;  // Q15
    uint32_t delay;
    uint32_t plectrum_delay;
    
    // Sample player state
    uint32_t phase;          // Phase accumulator for sample playback
    
    // PRNG seed (unique per exciter instance)
    uint32_t rng_seed;
    
    void Init(uint32_t seed_init) {
        model = EXCITER_Q15_MALLET;
        parameter = 0;
        timbre = 32440;  // ~0.99
        signature = 0;
        lp.Init();
        damp_state = 0;
        damping = 0;
        particle_state = 16384;  // 0.5
        particle_range = 32767;  // 1.0
        delay = 0;
        plectrum_delay = 0;
        phase = 0;
        rng_seed = seed_init;
    }
    
    /// Set model from meta parameter (continuous sweep across models).
    /// meta_q15: 0..32767 sweeps through all models.
    /// first, last: range of models to sweep.
    void SetMeta(int32_t meta_q15, ExciterModelQ15 first, ExciterModelQ15 last) {
        int32_t num_models = (int32_t)(last - first + 1);
        // Scale meta to model index + fractional parameter
        int32_t scaled = (int32_t)(((int64_t)meta_q15 * num_models) >> 15);
        int32_t model_idx = scaled;
        // The fractional part becomes the parameter
        // parameter = (meta * num_models) mod 1.0, approximately
        int32_t frac = meta_q15 * num_models - (model_idx << 15);
        if (frac < 0) frac = 0;
        if (frac > 32767) frac = 32767;
        
        model = (ExciterModelQ15)(first + model_idx);
        if ((int32_t)model > (int32_t)last) model = last;
        parameter = frac;
    }
    
    /// Get pulse amplitude for the current timbre setting.
    /// Higher timbre = brighter = lower amplitude needed for constant loudness.
    /// Uses the SVF gain compensation LUT.
    int32_t __not_in_flash_func(GetPulseAmplitude)() {
        int32_t idx = (timbre * 256) >> 15;
        if (idx > 256) idx = 256;
        return lut_approx_svf_gain_q15[idx];
    }
    
    // ── Random helper ───────────────────────────────────────────────────
    
    /// Returns random sample in range 0..32767 (Q15 unsigned)
    int32_t RandomU() {
        rng_seed = 1103515245u * rng_seed + 12345u;
        return (int32_t)(rng_seed >> 17);  // 0..32767
    }
    
    /// Returns random sample in range -16384..16383 (Q15 signed, ~±0.5)
    int32_t RandomS() {
        return RandomU() - 16384;
    }
    
    // ── Process Functions ───────────────────────────────────────────────
    
    /// Main process entry point. Generates one sample of excitation.
    /// flags: gate/edge flags (ENV_FLAG_RISING, etc.)
    /// Returns Q15 excitation signal.
    int32_t __not_in_flash_func(Process)(uint8_t flags) {
        damping = 0;
        int32_t out = 0;
        bool skip_filter = false;
        
        switch (model) {
            case EXCITER_Q15_GRANULAR:  out = ProcessGranular(flags); skip_filter = true; break;
            case EXCITER_Q15_SAMPLE:    out = ProcessSample(flags); skip_filter = true; break;
            case EXCITER_Q15_MALLET:    out = ProcessMallet(flags); break;
            case EXCITER_Q15_PLECTRUM:  out = ProcessPlectrum(flags); break;
            case EXCITER_Q15_PARTICLES: out = ProcessParticles(flags); break;
            case EXCITER_Q15_FLOW:      out = ProcessFlow(flags); break;
            case EXCITER_Q15_NOISE:     out = ProcessNoise(flags); break;
            default: break;
        }
        
        // Sample-based models have their own filtering
        if (skip_filter) return out;
        
        // Apply LP filter for synthesis models
        int32_t cutoff_idx = (timbre * 256) >> 15;
        if (cutoff_idx > 256) cutoff_idx = 256;
        
        if (model == EXCITER_Q15_NOISE) {
            // Noise model: use resonant filter (parameter controls Q)
            int32_t res_idx = (parameter * 256) >> 15;
            if (res_idx > 256) res_idx = 256;
            lp.g = lut_approx_svf_g_q14[cutoff_idx];
            lp.r = lut_approx_svf_r_q14[res_idx];
            int32_t rg = (int32_t)(((int64_t)lp.r * lp.g) >> 13);
            int32_t g2 = (int32_t)(((int64_t)lp.g * lp.g) >> 13);
            int32_t denom = 32767 + rg + g2;
            if (denom < 1) denom = 1;
            lp.h = (int32_t)((32767LL * 32767) / denom);
            if (lp.h > 32767) lp.h = 32767;
        } else {
            // Other models: non-resonant LP
            lp.g = lut_approx_svf_g_q14[cutoff_idx];
            lp.r = 32767;  // Q14 ~2.0 = low Q (no resonance)
            lp.h = lut_approx_svf_h_q15[cutoff_idx];
        }
        
        out = lp.Process(out, FILT_LP);
        return out;
    }
    
    // ── Mallet ──────────────────────────────────────────────────────────
    // Single impulse on gate rising edge, followed by silence.
    // The impulse amplitude is compensated for filter cutoff.
    // Parameter controls damping: higher = faster mute on release.
    
    int32_t __not_in_flash_func(ProcessMallet)(uint8_t flags) {
        int32_t out = 0;
        
        if (flags & ENV_FLAG_RISING) {
            damp_state = 0;
            out = GetPulseAmplitude();
        }
        
        if (!(flags & ENV_FLAG_GATE)) {
            // Exponential decay of damping scaled from 32kHz original to 24kHz (0.95^1.3333 = 0.93397)
            // In Q15: damp = 32767 - ((32767 - damp) * 30604) >> 15
            //   where 30604 ≈ 0.93397 * 32767
            damp_state = 32767 - (int32_t)(((int64_t)(32767 - damp_state) * 30604) >> 15);
        }
        
        // damping = damp_state * (1 - parameter)
        damping = mul_q15(damp_state, 32767 - parameter);
        
        return out;
    }
    
    // ── Plectrum ────────────────────────────────────────────────────────
    // On gate: small negative impulse (string being pulled).
    // After a delay controlled by parameter: large positive impulse (release).
    // Creates a guitar-pick or plucked-string feel.
    
    int32_t __not_in_flash_func(ProcessPlectrum)(uint8_t flags) {
        int32_t amplitude = GetPulseAmplitude();
        int32_t impulse = 0;
        
        if (flags & ENV_FLAG_RISING) {
            // Small negative pre-impulse (pull phase)
            // -(amplitude * (0.05 + signature * 0.2))
            int32_t pull_gain = 1638 + mul_q15(signature, 6554);  // 0.05 + sig*0.2
            impulse = -mul_q15(amplitude, pull_gain);
            
            // Delay before snap scaled from 32kHz original to 24kHz: 48 + 3072 * parameter²
            int32_t p2 = mul_q15(parameter, parameter);
            plectrum_delay = 48 + (uint32_t)((p2 * 3072) >> 15);
        }
        
        if (plectrum_delay > 0) {
            --plectrum_delay;
            if (plectrum_delay == 0) {
                impulse = amplitude;  // Snap!
            }
            // Damping builds during pre-snap scaled from 32kHz to 24kHz (0.997^1.3333 = 0.99600)
            damp_state = 32767 - (int32_t)(((int64_t)(32767 - damp_state) * 32637) >> 15);
        } else {
            // After snap: damping decays scaled from 32kHz to 24kHz (0.9^1.3333 = 0.86987)
            damp_state = (int32_t)(((int64_t)damp_state * 28503) >> 15);
        }
        
        damping = damp_state >> 1;  // * 0.5
        
        return impulse;
    }
    
    // ── Particles ───────────────────────────────────────────────────────
    // Stochastic cloud of impulses with evolving density and pitch.
    // Gate triggers initialization; impulses continue while gate is high.
    // Parameter controls decay rate of the particle cloud.
    
    int32_t __not_in_flash_func(ProcessParticles)(uint8_t flags) {
        if (flags & ENV_FLAG_RISING) {
            int32_t r = RandomU();
            // particle_state = 1.0 - 0.6 * r² (in Q15)
            int32_t r2 = mul_q15(r, r);
            particle_state = 32767 - (int32_t)(((int64_t)r2 * 19661) >> 15);
            delay = 0;
            particle_range = 32767;
        }
        
        int32_t out = 0;
        
        if (flags & ENV_FLAG_GATE) {
            if (delay == 0) {
                // Random amount for next jump: 1.05 + 0.5 * rand²
                int32_t amt_r = RandomU();
                int32_t amt_r2 = mul_q15(amt_r, amt_r);
                int32_t amount = 34406 + (amt_r2 >> 1);
                
                // Stochastic jump (70% up, 30% down)
                uint32_t coin = FastRandQ15(rng_seed);
                if (coin > 3006477107u) {
                    particle_state = mul_q15(particle_state, amount);
                    int32_t cap = particle_range + 8192;
                    if (particle_state > cap) particle_state = cap;
                } else if (coin < 1288490189u) {
                    if (amount > 0) {
                        particle_state = (int32_t)((32767LL * (int64_t)particle_state) / amount);
                    }
                    if (particle_state < 655) particle_state = 655;
                }
                
                // Compute base delay scaled from 32kHz to 24kHz (original 3600 * 0.75 / 3x density = 900)
                uint32_t base_delay = (uint32_t)((particle_state * 900) >> 15);
                
                // --- SCATTERING LOGIC ---
                // Use 'signature' to control the amount of timing jitter (scattering)
                // If signature is high, delay is randomized up to +/- 100%
                int32_t jitter_range = mul_q15(base_delay, signature);
                int32_t jitter = (int32_t)(((int64_t)RandomS() * jitter_range) >> 15);
                delay = base_delay + jitter;
                if (delay < 1) delay = 1;
                
                // Compute amplitude
                int32_t gain_inv = 32767 - particle_range;
                int32_t gain = 32767 - mul_q15(gain_inv, gain_inv);
                
                // PER-PARTICLE TIMBRE SCATTERING
                // Briefly randomize the filter cutoff for this specific particle
                int32_t p_timbre = timbre + (int32_t)(((int64_t)RandomS() * signature) >> 15);
                if (p_timbre < 1000) p_timbre = 1000;
                if (p_timbre > 32000) p_timbre = 32000;
                
                // Get pulse amp based on randomized timbre
                int32_t p_idx = (p_timbre * 256) >> 15;
                int32_t p_amp = lut_approx_svf_gain_q15[p_idx > 256 ? 256 : p_idx];
                
                out = mul_q15(mul_q15(particle_state, p_amp), gain);
                
                // Decay density scaled from 32kHz to 24kHz ((1 - e/2)^1.3333 ≈ 1 - 2/3e)
                int32_t df = 32767 - parameter;
                int32_t df2 = mul_q15(df, df);
                particle_range = mul_q15(particle_range, 32767 - ((df2 * 2) / 3));
            } else {
                --delay;
            }
        }
        
        return out >> 1;  // Attenuate dense particle cloud by 6dB to prevent volume spikes
    }
    
    // ── Flow ────────────────────────────────────────────────────────────
    // Chaotic particle-based noise generator. Good for bow/wind sounds.
    // Parameter controls turbulence: low = smooth drone, high = chaotic.
    
    int32_t __not_in_flash_func(ProcessFlow)(uint8_t flags) {
        // scale = parameter^4 (very small at low values)
        int32_t p2 = mul_q15(parameter, parameter);
        int32_t scale = mul_q15(p2, p2);
        
        // threshold = 0.0001 + scale * 0.125
        // In Q15: 3 + (scale * 4096) >> 15
        int32_t threshold = 3 + (scale >> 3);
        
        if (flags & ENV_FLAG_RISING) {
            particle_state = 16384;  // 0.5
        }
        
        int32_t sample = RandomU();  // 0..32767
        
        if (sample < threshold) {
            particle_state = -particle_state;
        }
        
        // out = particle_state + (sample - 0.5 - particle_state) * scale
        int32_t centered = sample - 16384 - particle_state;
        particle_state = particle_state + mul_q15(centered, scale);
        
        return particle_state;
    }
    
    // ── Noise ───────────────────────────────────────────────────────────
    // Simple filtered white noise. Timbre controls cutoff, parameter
    // controls resonance (Q) of the filter.
    
    int32_t __not_in_flash_func(ProcessNoise)(uint8_t flags) {
        (void)flags;
        return RandomS();  // -16384..16383
    }
    
    // ── Granular Sample Player ───────────────────────────────────────────
    // Reads from the flash-resident noise texture sample via XIP.
    // Parameter selects the playback position; timbre controls pitch.
    // Random restarts create granular texture.
    // Samples accessed directly from flash — zero RAM cost.
    
    int32_t __not_in_flash_func(ProcessGranular)(uint8_t flags) {
        (void)flags;
        
        // Restart probability: ~1% chance per sample
        const uint32_t restart_prob = 42949673u;  // ~0.01 * 2^32
        
        // Restart position: parameter selects where in the noise sample
        // parameter is Q15 (0..32767), map to sample position
        const uint32_t restart_point = (uint32_t)(parameter & 0x7FFF) << 17;
        
        // Phase increment: timbre controls speed/pitch (scaled from 32kHz original to 24kHz, * 1.3333)
        // At timbre=0.5 (16384), play at original speed (increment=131072)
        // timbre maps to -60..+12 semitones of pitch shift
        // Simplified: increment = 131072 * 2^((timbre*72/32767 - 60) / 12)
        // Use a rough exponential: timbre² * 524288 / 32767 + 4096 (scaled by 4/3)
        int32_t t = timbre;
        uint32_t phase_increment = 5461 + (uint32_t)(((int64_t)t * t * 21) >> 15);
        
        // Base pointer offset from signature (different texture regions)
        uint32_t sig_offset = (uint32_t)((signature * 8192) >> 15);
        const int16_t* base = &smp_noise_sample[sig_offset];
        
        // Read sample with linear interpolation (directly from flash!)
        uint32_t phase_int = phase >> 17;
        int32_t phase_frac = (phase >> 9) & 0xFF;  // 8-bit fraction
        
        // Bounds check against noise sample length
        if (phase_int + 1 < SMP_NOISE_LENGTH - sig_offset) {
            int32_t a = base[phase_int];
            int32_t b = base[phase_int + 1];
            int32_t out = a + (((b - a) * phase_frac) >> 8);
            
            phase += phase_increment;
            
            // Random restart for granular texture
            uint32_t coin = FastRandQ15(rng_seed);
            if (coin < restart_prob) {
                phase = restart_point;
            }
            
            return out >> 1;  // Scaled down to match regular sample player headroom
        } else {
            // Past end of sample — wrap around
            phase = restart_point;
            return 0;
        }
    }
    
    // ── Sample Player ────────────────────────────────────────────────────
    // Plays back one of 9 percussion samples from flash.
    // Parameter selects the sample (crossfades between adjacent ones).
    // Timbre controls playback speed/pitch.
    // Gate retriggers from the start.
    
    int32_t __not_in_flash_func(ProcessSample)(uint8_t flags) {
        // Map parameter (Q15) to sample index 0..8 with fractional crossfade
        // index = (1 - parameter) * 8
        int32_t index_scaled = ((32767 - parameter) * 8);
        int32_t idx1 = index_scaled >> 15;
        int32_t idx_frac = (index_scaled >> 7) & 0xFF;  // 8-bit fraction
        if (idx1 >= 8) { idx1 = 7; idx_frac = 255; }
        int32_t idx2 = idx1 + 1;
        
        // Segment boundaries from flash
        uint32_t offset1 = smp_boundaries[idx1];
        uint32_t offset2 = smp_boundaries[idx2];
        uint32_t length1 = offset2 - offset1 - 1;
        uint32_t length2 = smp_boundaries[idx2 + 1] - offset2 - 1;
        
        // Phase increment from timbre (pitch control, scaled from 32kHz to 24kHz, * 1.3333)
        // At timbre=0.5: normal speed (65536). Range: -36 to +43 semitones
        uint32_t phase_increment = 10922 + (uint32_t)(((int64_t)timbre * timbre) / 6144);
        
        if (flags & ENV_FLAG_RISING) {
            damp_state = 0;
            phase = 0;
        }
        if (!(flags & ENV_FLAG_GATE)) {
            // Release damping scaled from 32kHz to 24kHz (0.95^1.3333 = 0.93397): damp = 1 - 0.93397*(1-damp)
            damp_state = 32767 - (int32_t)(((int64_t)(32767 - damp_state) * 30604) >> 15);
        }
        
        uint32_t phase_int = phase >> 16;
        int32_t phase_frac = (phase >> 8) & 0xFF;
        
        int32_t sample1 = 0;
        int32_t sample2 = 0;
        bool advancing = false;
        
        // Read from sample 1 (from flash!)
        if (phase_int < length1) {
            const int16_t* base = &smp_sample_data[offset1 + phase_int];
            int32_t a = base[0];
            int32_t b = base[1];
            sample1 = a + (((b - a) * phase_frac) >> 8);
            advancing = true;
        }
        
        // Read from sample 2 (for crossfade)
        if (phase_int < length2) {
            const int16_t* base = &smp_sample_data[offset2 + phase_int];
            int32_t a = base[0];
            int32_t b = base[1];
            sample2 = a + (((b - a) * phase_frac) >> 8);
            advancing = true;
        }
        
        if (advancing) {
            phase += phase_increment;
        }
        
        // Crossfade between adjacent samples
        int32_t out = sample1 + (((sample2 - sample1) * idx_frac) >> 8);
        
        // Scale down (samples are full int16 range, we want Q15 headroom)
        out >>= 1;
        
        // Damping output: if parameter is high, damping affects resonator on release
        if (parameter >= 26214) {  // >= 0.8
            damping = mul_q15(damp_state, (int32_t)(((int64_t)parameter * 5) >> 15) - 4 * 32767 / 5);
        }
        
        return out;
    }
};

#endif  // EXCITER_Q15_H_
