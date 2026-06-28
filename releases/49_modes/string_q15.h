#pragma once

#include <stdint.h>
#include "dsp_q15.h"
#include "svf_q15.h"

#include "resonator_q15.h"

// 2048 sample delay line for string
#define STRING_DELAY_SIZE 2048
#define STRETCH_DELAY_SIZE 1024

// DampingFilterQ15 removed as it was unstable.

class StringQ15 {
    uint32_t frequency_inc_;
    int32_t dispersion_q15_;
    int32_t brightness_q15_;
    int32_t damping_q15_;
    int32_t position_q15_;
    
    DelayLineQ15<STRING_DELAY_SIZE> string_;
    DelayLineQ15<STRETCH_DELAY_SIZE> stretch_;
    
    int32_t lp_state_;
    int32_t hp_state_;
    
    int32_t dc_px, dc_py; // DC blocker state
    
    int32_t dispersion_noise_;
    int32_t curved_bridge_;
    
    int32_t out_sample_[2];
    int32_t aux_sample_[2];
    
    uint32_t last_frequency_inc_;
    int32_t cached_delay_q15_;
    int32_t last_lp_coef_;
    int32_t cached_filter_delay_q15_;
    
public:
    void Init();
    void SetFrequency(uint32_t f) { frequency_inc_ = f; }
    void SetDispersion(int32_t d) { dispersion_q15_ = d; }
    void SetBrightness(int32_t b) { brightness_q15_ = b; }
    void SetDamping(int32_t d) { damping_q15_ = d; }
    void SetPosition(int32_t p) { position_q15_ = p; }
    
    void Process(int32_t in, int32_t& out, int32_t& aux);
};

