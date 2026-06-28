#ifndef TUBE_Q15_H_
#define TUBE_Q15_H_

#include <stdint.h>
#include "dsp_q15.h"

// -----------------------------------------------------------------------------
// TubeQ15
//
// A simplified waveguide model of a tube (flute/bottle body).
// Based on Mutable Instruments Elements.
// Includes a scattering junction (reed model) and a feedback delay line.
// -----------------------------------------------------------------------------

class TubeQ15 {
public:
    void Init() {
        for (int i = 0; i < 2048; i++) delay_line[i] = 0;
        ptr = 0;
        zero_state = 0;
        pole_state = 0;
        last_freq_q15 = 0;
        cached_delay_q15 = 0;
    }

    /// Process one sample.
    /// freq_q15: normalized frequency (f/sr_dsp) in Q15
    /// envelope: excitation envelope in Q15 (0..32767)
    /// damping: resonator damping parameter in Q15
    /// timbre: blow timbre in Q15
    /// input: exciter signal (noise) in Q15
    /// Returns the resonance signal in Q15.
    int32_t Process(int32_t freq_q15, int32_t envelope, int32_t damping, int32_t timbre, int32_t input) {
        if (freq_q15 < 1) freq_q15 = 1;
        
        int32_t delay_q15;
        if (freq_q15 == last_freq_q15) {
            delay_q15 = cached_delay_q15;
        } else {
            delay_q15 = (int32_t)((32768LL << 15) / freq_q15);
            last_freq_q15 = freq_q15;
            cached_delay_q15 = delay_q15;
        }
        
        // Cap delay at buffer size
        if (delay_q15 > (2046 << 15)) delay_q15 = 2046 << 15;
        
        int32_t delay_integral = delay_q15 >> 15;
        int32_t delay_fractional = delay_q15 & 0x7FFF;

        // damping_mod = 3.6 - damping * 1.8
        // Q15: 117965 - damping * 1.8
        int32_t damping_mod = 117965 - ((int32_t)damping * 58982 >> 15); // Q15

        // lpf_coefficient = frequency * (1.0 + timbre * timbre * 256.0)
        // timbre*timbre is Q30, shift to Q15. 
        int32_t t2 = ((int64_t)timbre * timbre) >> 15; // Q15
        int32_t lpf_coeff = mul_q15(freq_q15, 32767 + (t2 << 8));
        if (lpf_coeff > 32600) lpf_coeff = 32600; // 0.995

        // Scattering junction math
        // breath = input * damping_mod + 0.8
        // 0.8 in Q15 is 26214.
        // input * damping_mod is Q15*Q15 -> Q15.
        int32_t breath = mul_q15(input, damping_mod) + 26214;

        // Read from delay line with linear interpolation
        int32_t read_ptr = ptr + delay_integral;
        int32_t a = delay_line[read_ptr & 2047];
        int32_t b = delay_line[(read_ptr + 1) & 2047];
        int32_t in = a + (((b - a) * delay_fractional) >> 15); // Q15

        // pressure_delta = -0.95 * (in * envelope + zero_state) - breath
        // -0.95 in Q15 is -31130
        int32_t capped_envelope = (envelope > 32767) ? 32767 : envelope;
        int32_t env_in = mul_q15(in, capped_envelope);
        int32_t pressure_delta = mul_q15(-31130, env_in + zero_state) - breath;
        zero_state = in;

        // reed = pressure_delta * -0.2 + 0.8
        // -0.2 in Q15 is -6554, 0.8 is 26214
        int32_t reed = mul_q15(pressure_delta, -6554) + 26214;
        
        // out = pressure_delta * reed + breath
        int32_t out = mul_q15(pressure_delta, reed) + breath;
        
        // Constrain to +/- 5.0 (clipping)
        if (out > 163835) out = 163835;
        else if (out < -163835) out = -163835;
        
        // delay_line[ptr] = out * 0.5
        delay_line[ptr] = (int16_t)(out >> 1);
        
        ptr = (ptr - 1) & 2047;

        // Low-pass filter the output
        pole_state += mul_q15(lpf_coeff, out - pole_state);
        
        // Return filtered signal scaled by envelope
        return mul_q15(envelope, pole_state);
    }

private:
    int16_t delay_line[2048];
    int ptr;
    int32_t zero_state;
    int32_t pole_state;
    int32_t last_freq_q15;
    int32_t cached_delay_q15;
};

#endif // TUBE_Q15_H_
