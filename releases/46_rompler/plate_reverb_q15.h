#pragma once

#include <stdint.h>
#include <string.h>

// Full quality Freeverb implementation (8 parallel comb filters + 4 series allpass sections per channel).
// Scaled and tuned for 48kHz. All delay lengths are prime to minimize flutter echo.
class PlateReverbQ15 {
    static inline int16_t clamp16(int32_t x) {
        if (x >  32767) return  32767;
        if (x < -32768) return -32768;
        return (int16_t)x;
    }

    // Single-feedback comb filter (Schroeder)
    struct Comb {
        int16_t *buf;
        uint16_t len, ptr;
        int32_t  lp;          // one-pole LP state (damps high freq in tail)

        void init(int16_t *b, uint16_t l) {
            buf = b; len = l; ptr = 0; lp = 0;
            memset(buf, 0, len * sizeof(int16_t));
        }
        // feedback: Q15  damp: Q15
        int16_t process(int16_t in, int32_t feedback, int32_t damp) {
            int16_t out = buf[ptr];
            // LP inside the loop: lp = lp + damp*(out - lp)
            lp += ((((int32_t)out - lp) * damp) >> 15);
            int32_t fb = (lp * feedback) >> 15;
            buf[ptr] = clamp16((int32_t)in + fb);
            if (++ptr >= len) ptr = 0;
            return out;
        }
    };

    // Schroeder allpass
    struct AP {
        int16_t *buf;
        uint16_t len, ptr;

        void init(int16_t *b, uint16_t l) {
            buf = b; len = l; ptr = 0;
            memset(buf, 0, len * sizeof(int16_t));
        }
        // g fixed ~0.7 (22938 in Q15)
        int16_t process(int16_t in) {
            const int32_t g = 22938; // 0.7 * 32768
            int16_t bufOut = buf[ptr];
            int32_t v = (int32_t)in - ((g * (int32_t)bufOut) >> 15);
            buf[ptr] = clamp16(v);
            if (++ptr >= len) ptr = 0;
            return clamp16((int32_t)bufOut + ((g * v) >> 15));
        }
    };

    // Scaled delay lengths for 48kHz (scaled from Freeverb 44.1kHz values by ~1.0884)
    // Comb L: 1693, 1759, 1621, 1543, 1381, 1471, 1291, 1213
    // Comb R (offset by 25): 1721, 1787, 1657, 1579, 1423, 1499, 1321, 1237
    // AP L: 241, 367, 479, 601
    // AP R (offset by 25): 271, 397, 509, 631
    static constexpr uint16_t CL0 = 1693, CL1 = 1759, CL2 = 1621, CL3 = 1543,
                              CL4 = 1381, CL5 = 1471, CL6 = 1291, CL7 = 1213;
    static constexpr uint16_t CR0 = 1721, CR1 = 1787, CR2 = 1657, CR3 = 1579,
                              CR4 = 1423, CR5 = 1499, CR6 = 1321, CR7 = 1237;
    static constexpr uint16_t AL0 =  241, AL1 =  367, AL2 =  479, AL3 =  601;
    static constexpr uint16_t AR0 =  271, AR1 =  397, AR2 =  509, AR3 =  631;

    static constexpr uint16_t TOTAL_WORDS =
        CL0+CL1+CL2+CL3+CL4+CL5+CL6+CL7 +
        CR0+CR1+CR2+CR3+CR4+CR5+CR6+CR7 +
        AL0+AL1+AL2+AL3 + AR0+AR1+AR2+AR3; // = 27692 words (~55.4KB RAM)

    int16_t mem[TOTAL_WORDS];

    Comb combL[8], combR[8];
    AP   apL[4],   apR[4];

    // Smooth targets so knob changes don't click
    int32_t smooth_feedback; // Q15
    int32_t smooth_damp;     // Q15

public:
    void Init() {
        memset(mem, 0, sizeof(mem));
        smooth_feedback = 20000;
        smooth_damp     = 16384;

        int16_t *p = mem;
        
        combL[0].init(p, CL0); p += CL0;
        combL[1].init(p, CL1); p += CL1;
        combL[2].init(p, CL2); p += CL2;
        combL[3].init(p, CL3); p += CL3;
        combL[4].init(p, CL4); p += CL4;
        combL[5].init(p, CL5); p += CL5;
        combL[6].init(p, CL6); p += CL6;
        combL[7].init(p, CL7); p += CL7;

        combR[0].init(p, CR0); p += CR0;
        combR[1].init(p, CR1); p += CR1;
        combR[2].init(p, CR2); p += CR2;
        combR[3].init(p, CR3); p += CR3;
        combR[4].init(p, CR4); p += CR4;
        combR[5].init(p, CR5); p += CR5;
        combR[6].init(p, CR6); p += CR6;
        combR[7].init(p, CR7); p += CR7;

        apL[0].init(p, AL0); p += AL0;
        apL[1].init(p, AL1); p += AL1;
        apL[2].init(p, AL2); p += AL2;
        apL[3].init(p, AL3); p += AL3;

        apR[0].init(p, AR0); p += AR0;
        apR[1].init(p, AR1); p += AR1;
        apR[2].init(p, AR2); p += AR2;
        apR[3].init(p, AR3); p += AR3;
    }

    // outL/outR  — modified in-place (dry + wet added).
    // mix        — wet level Q15 (0..32767).
    // decay      — feedback (0..30000).
    // damp       — high-frequency damping (0..32767).
    // dual_mono  — if true, L+R are summed before processing.
    void __not_in_flash_func(Process)(
        int32_t &outL, int32_t &outR,
        int32_t mix, int32_t decay, int32_t damp,
        int32_t /*size_ratio*/, bool dual_mono = false)
    {
        // Smooth feedback and damp so knob changes are zipper-free
        smooth_feedback += ((decay      - smooth_feedback) >> 6);
        smooth_damp     += ((damp       - smooth_damp)     >> 6);

        // Clamp feedback to prevent runaway (<1.0 always)
        int32_t fb = smooth_feedback;
        if (fb > 30000) fb = 30000;
        if (fb < 0)     fb = 0;

        int32_t dp = smooth_damp;
        if (dp > 30000) dp = 30000;
        if (dp < 0)     dp = 0;

        // Input: scale down from 12-bit audio range to int16 reverb range.
        int16_t inL = clamp16(outL * 8);
        int16_t inR = dual_mono ? inL : clamp16(outR * 8);

        // Run 8 parallel combs and sum
        int32_t sumL = 0, sumR = 0;
        for (int i = 0; i < 8; i++) {
            sumL += combL[i].process(inL, fb, dp);
            sumR += combR[i].process(inR, fb, dp);
        }
        // Scale sum: 8 combs, divide by 8 to stay in int16 range
        int16_t sL = clamp16(sumL >> 3);
        int16_t sR = clamp16(sumR >> 3);

        // Four series allpass sections per channel
        sL = apL[0].process(sL);
        sL = apL[1].process(sL);
        sL = apL[2].process(sL);
        sL = apL[3].process(sL);

        sR = apR[0].process(sR);
        sR = apR[1].process(sR);
        sR = apR[2].process(sR);
        sR = apR[3].process(sR);

        // Mix wet into output ONLY if mix > 0. mix is Q15 (0..32767).
        // sL/sR are int16. Scale back down to audio range: /8
        if (mix > 0) {
            outL += (((int32_t)sL * mix) >> 18);
            outR += (((int32_t)sR * mix) >> 18);
        }
    }
};
