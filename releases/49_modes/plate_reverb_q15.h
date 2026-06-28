#pragma once

#include <stdint.h>
#include <string.h>

class PlateReverbQ15 {
    struct AP {
        int16_t *bufIn, *bufOut;
        uint16_t size, ptr, len;
        uint16_t base_len;
        int16_t g;
        void init(int16_t *b, uint16_t s, uint16_t l, int16_t gain) {
            bufIn = b;
            bufOut = b + s;
            size = s;
            base_len = l;
            len = l;
            g = gain;
            ptr = 0;
        }
        int16_t process(int16_t in) {
            uint16_t rd = ptr >= len ? ptr - len : ptr + size - len;
            int16_t dIn = bufIn[rd], dOut = bufOut[rd];
            int32_t interm = ((int32_t)(g * in) >> 15) + dIn - ((int32_t)(dOut * g) >> 15);
            if (interm > 32767) interm = 32767;
            else if (interm < -32768) interm = -32768;
            int16_t out = (int16_t)interm;
            bufIn[ptr] = in;
            bufOut[ptr] = out;
            if (++ptr >= size) ptr = 0;
            return out;
        }
    };
    
    struct Delay {
        int16_t *buf;
        uint16_t size, ptr, len;
        uint16_t base_len;
        void init(int16_t *b, uint16_t s, uint16_t l) {
            buf = b;
            size = s;
            base_len = l;
            len = l;
            ptr = 0;
        }
        void write(int16_t in) {
            buf[ptr] = in;
            if (++ptr >= size) ptr = 0;
        }
        int16_t read() { 
            uint16_t rd = ptr >= len ? ptr - len : ptr + size - len;
            return buf[rd]; 
        }
    };
    
    // Total required memory: 2*(172+130+459+336 + 2178+2888) + 813+4501+4351+1099+4651+4126 = 31631 words.
    // 32000 is perfectly safe and smaller than the previous 35840.
    int16_t mem[32000];
    AP apIn[4], apTankL, apTankR;
    Delay modL, d1L, d2L, modR, d1R, d2R;
    int32_t lpL, lpR, lpIn;
    uint32_t lfo;
    int32_t current_ratio;

public:
    void Init() {
        memset(mem, 0, sizeof(mem));
        lpL = 0;
        lpR = 0;
        lpIn = 0;
        lfo = 0;
        current_ratio = 32768;
        
        int16_t *p = mem;
        
        // Original Elements runs Reverb at 32kHz. We run at 24kHz.
        // We scale all lengths by 24/32 = 0.75 to maintain exact room density.
        // The original sizes were:
        // APs: 229, 172, 611, 447
        // Mod: 1083, 1464
        // D1: 6000, 6200
        // ApTank: 2903, 3850
        // D2: 5800, 5500
        
        apIn[0].init(p, 172, 171, 24576);  p += 172 * 2;
        apIn[1].init(p, 130, 129, 24576);  p += 130 * 2;
        apIn[2].init(p, 459, 458, 20480);  p += 459 * 2;
        apIn[3].init(p, 336, 335, 20480);  p += 336 * 2;
        
        modL.init(p, 813, 812);           p += 813;
        d1L.init(p, 4501, 4500);          p += 4501;
        apTankL.init(p, 2178, 2177, 16384); p += 2178 * 2;
        d2L.init(p, 4351, 4350);          p += 4351;
        
        modR.init(p, 1099, 1098);         p += 1099;
        d1R.init(p, 4651, 4650);          p += 4651;
        apTankR.init(p, 2888, 2887, 16384); p += 2888 * 2;
        d2R.init(p, 4126, 4125);          p += 4126;
    }
    
    void __not_in_flash_func(Process)(int32_t &outL, int32_t &outR, int32_t mix, int32_t decay, int32_t damp, int32_t size_ratio) {
        if (mix < 10) return;
        
        // Smoothly interpolate size ratio to prevent clicks and generate lush pitch-shifter sweeping effects
        current_ratio += (size_ratio - current_ratio) >> 8;
        
        for (int i = 0; i < 4; i++) {
            apIn[i].len = (apIn[i].base_len * current_ratio) >> 15;
            if (apIn[i].len < 2) apIn[i].len = 2;
        }
        apTankL.len = (apTankL.base_len * current_ratio) >> 15;
        if (apTankL.len < 2) apTankL.len = 2;
        apTankR.len = (apTankR.base_len * current_ratio) >> 15;
        if (apTankR.len < 2) apTankR.len = 2;
        
        modL.len = (modL.base_len * current_ratio) >> 15;
        if (modL.len < 2) modL.len = 2;
        d1L.len = (d1L.base_len * current_ratio) >> 15;
        if (d1L.len < 2) d1L.len = 2;
        d2L.len = (d2L.base_len * current_ratio) >> 15;
        if (d2L.len < 2) d2L.len = 2;
        
        modR.len = (modR.base_len * current_ratio) >> 15;
        if (modR.len < 2) modR.len = 2;
        d1R.len = (d1R.base_len * current_ratio) >> 15;
        if (d1R.len < 2) d1R.len = 2;
        d2R.len = (d2R.base_len * current_ratio) >> 15;
        if (d2R.len < 2) d2R.len = 2;

        int32_t mono_32 = (outL + outR) >> 1;
        if (mono_32 > 32767) mono_32 = 32767;
        else if (mono_32 < -32768) mono_32 = -32768;
        
        // Gentle pre-filter to tame harshness without killing transients.
        lpIn += ((mono_32 - lpIn) * 26000) >> 15;
        
        // Soft clip the reverb input to prevent harsh transients in the tail
        int32_t clipped = lpIn;
        if (clipped > 24000) clipped = 24000 + ((clipped - 24000) >> 2);
        else if (clipped < -24000) clipped = -24000 + ((clipped + 24000) >> 2);
        
        // Hard limit before cast
        if (clipped > 32767) clipped = 32767;
        else if (clipped < -32768) clipped = -32768;
        
        // Scale input gain to 0.2 (6554 in Q15) to prevent internal tank clipping, matching Elements exactly
        int32_t mono_scaled = (clipped * 6554) >> 15;
        int16_t mono = (int16_t)mono_scaled;
        
        for (int i = 0; i < 4; i++) {
            mono = apIn[i].process(mono);
        }
        
        lfo += 4096;
        int16_t mod = (lfo >> 16) & 0x7FFF;
        if (lfo & 0x80000000) mod = 32767 - mod;
        
        int16_t tOutL = d2L.read(), tOutR = d2R.read();
        
        int32_t iL = (int32_t)mono + (((int32_t)decay * tOutR) >> 15);
        int16_t sL = (int16_t)iL + (int16_t)((16384 * modL.read()) >> 15);
        modL.write((int16_t)iL - (int16_t)((16384 * sL) >> 15));
        d1L.write(sL);
        sL = d1L.read();
        lpL += (((int32_t)sL - lpL) * damp) >> 15;
        sL = (int16_t)lpL;
        sL = apTankL.process(sL);
        d2L.write(sL);
        
        int32_t iR = (int32_t)mono + (((int32_t)decay * tOutL) >> 15);
        int16_t sR = (int16_t)iR + (int16_t)((16384 * modR.read()) >> 15);
        modR.write((int16_t)iR - (int16_t)((16384 * sR) >> 15));
        d1R.write(sR);
        sR = d1R.read();
        lpR += (((int32_t)sR - lpR) * damp) >> 15;
        sR = (int16_t)lpR;
        sR = apTankR.process(sR);
        d2R.write(sR);
        
        int32_t wetL = sL << 2;
        int32_t wetR = sR << 2;
        outL = outL + (((wetL - outL) * mix) >> 15);
        outR = outR + (((wetR - outR) * mix) >> 15);
    }
};
