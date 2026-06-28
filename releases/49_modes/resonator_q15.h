// =============================================================================
// resonator_q15.h — Fixed-point Modal Resonator for Elements port
//
// Port of elements::Resonator to Q15 integer arithmetic.
// Simulates a vibrating structure using a bank of 24 resonant band-pass filters.
// =============================================================================

#ifndef RESONATOR_Q15_H_
#define RESONATOR_Q15_H_

#include <stdint.h>
#include "dsp_q15.h"
#include "svf_q15.h"
#include "resources_q15.h"

// ── Simple Delay Line (Q15, int16_t storage) ────────────────────────────────

template<int max_size>
struct DelayLineQ15 {
    int16_t line[max_size];
    uint16_t ptr;
    uint32_t delay_q15;

    void Init() {
        for (int i = 0; i < max_size; i++) line[i] = 0;
        ptr = 0;
        delay_q15 = 1 << 15;
    }

    void set_delay(uint32_t d_q15) {
        if (d_q15 < (1 << 15)) d_q15 = 1 << 15;
        if (d_q15 >= ((uint32_t)max_size << 15)) d_q15 = ((uint32_t)max_size - 1) << 15;
        delay_q15 = d_q15;
    }

    int32_t Read() const {
        return ReadHermite(delay_q15);
    }

    void Write(int32_t sample) {
        if (sample > 32767) sample = 32767;
        if (sample < -32768) sample = -32768;
        line[ptr] = (int16_t)sample;
        ptr = (ptr + 1) & (max_size - 1);
    }
    
    int32_t ReadHermite(int32_t delay_q15) const {
        uint32_t int_delay = delay_q15 >> 15;
        int32_t frac = delay_q15 & 0x7FFF;
        
        uint32_t idx1 = (ptr + max_size - int_delay) & (max_size - 1);
        uint32_t idx2 = (ptr + max_size - int_delay - 1) & (max_size - 1);
        
        int32_t y1 = line[idx1];
        int32_t y2 = line[idx2];
        
        return y1 + mul_q15(frac, y2 - y1);
    }
    
    int32_t Allpass(int32_t in, int32_t delay_q15, int32_t gain_q15) {
        int32_t read = ReadHermite(delay_q15);
        int32_t write = in + mul_q15(gain_q15, read);
        Write(write);
        return read - mul_q15(gain_q15, write);
    }
};

// ── Resonator (Q15) ──────────────────────────────────────────────────────────

static const size_t kMaxModesQ15 = 16;
static const size_t kMaxBowedModesQ15 = 4;
static const size_t kMaxDelayLineSizeQ15 = 512;

struct ResonatorQ15 {
    // Parameters (all Q15 0..32767)
    int32_t frequency_q15;  // normalized frequency f/sr, Q15 (0..32767 = 0..1.0)
    int32_t geometry_q15;   // 0..32767
    int32_t brightness_q15; // 0..32767
    int32_t damping_q15;    // 0..32767
    int32_t position_q15;   // 0..32767
    
    int32_t previous_position_q15;
    int32_t previous_geometry_q15;
    int32_t modulation_frequency_q15;
    int32_t modulation_offset_q15;
    int32_t lfo_phase_q15;
    
    int32_t bow_signal_q15;
    size_t resolution;
    
    enum Structure {
        STRUC_MODAL,
        STRUC_WIND
    } structure;
    
    // Filter bank
    SvfQ15 f_[kMaxModesQ15];
    SvfQ15 f_bow_[kMaxBowedModesQ15];
    DelayLineQ15<kMaxDelayLineSizeQ15> d_bow_[kMaxBowedModesQ15];
    
    size_t clock_divider;
    size_t num_modes_cached;  // cached from last ComputeFilters
    
    void Init();
    size_t ComputeFilters();
    
    // Process one sample (for Workshop_Computer per-sample architecture)
    void Process1(int32_t bow_strength, int32_t in, int32_t &center, int32_t &sides);
};

#endif  // RESONATOR_Q15_H_
