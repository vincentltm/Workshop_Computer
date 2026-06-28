#ifndef DIFFUSER_Q15_H_
#define DIFFUSER_Q15_H_

#include <stdint.h>
#include "dsp_q15.h"

// -----------------------------------------------------------------------------
// DiffuserQ15
//
// A chain of 4 all-pass filters used to "smear" the excitation signal.
// Based on Mutable Instruments Elements diffuser.
// Delays: 126, 180, 269, 444 samples.
// Total memory: 1019 samples (int16_t).
// -----------------------------------------------------------------------------

class DiffuserQ15 {
public:
    void Init() {
        for (int i = 0; i < 1019; i++) buffer[i] = 0;
        ptr1 = 0;
        ptr2 = 0;
        ptr3 = 0;
        ptr4 = 0;
    }

    int32_t Process(int32_t in) {
        const int32_t kap = 20480; // 0.625 in Q15

        // All-pass 1 (126 samples)
        int32_t out1 = AllPass(in, &buffer[0], 126, ptr1, kap);
        // All-pass 2 (180 samples)
        int32_t out2 = AllPass(out1, &buffer[126], 180, ptr2, kap);
        // All-pass 3 (269 samples)
        int32_t out3 = AllPass(out2, &buffer[126 + 180], 269, ptr3, kap);
        // All-pass 4 (444 samples)
        int32_t out4 = AllPass(out3, &buffer[126 + 180 + 269], 444, ptr4, kap);

        return out4;
    }

private:
    inline int32_t AllPass(int32_t in, int16_t* b, int length, int& ptr, int32_t g) {
        int32_t delayed = b[ptr];
        // Schroeder all-pass:
        // w[n] = x[n] + g * w[n-D]
        // y[n] = -g * w[n] + w[n-D]
        int32_t w = in + mul_q15(g, delayed);
        b[ptr] = (int16_t)sat_q15(w);
        int32_t out = delayed - mul_q15(g, w);
        
        ptr++;
        if (ptr >= length) ptr = 0;
        
        return out;
    }

    int16_t buffer[1019];
    int ptr1, ptr2, ptr3, ptr4;
};

#endif // DIFFUSER_Q15_H_
