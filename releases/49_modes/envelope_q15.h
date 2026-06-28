// =============================================================================
// envelope_q15.h — Fixed-point Multistage Envelope for Elements port
//
// Port of elements::MultistageEnvelope to Q15 integer arithmetic.
// Supports AD, AR, ADSR, and looped envelope shapes.
// Used by the exciter to shape the amplitude of mallet/bow/blow signals.
//
// All values in Q15 (0..32767 = 0..1.0).
// Phase increments from lut_env_increments_q20 (Q20 format).
// Shape curves from lut_env_linear/expo/quartic_q15.
// =============================================================================

#ifndef ENVELOPE_Q15_H_
#define ENVELOPE_Q15_H_

#include <stdint.h>
#include "dsp_q15.h"
#include "resources_q15.h"

// ── Envelope Flags (match original) ─────────────────────────────────────────
#define ENV_FLAG_RISING   1
#define ENV_FLAG_FALLING  2
#define ENV_FLAG_GATE     4

// ── Envelope Shape ──────────────────────────────────────────────────────────
enum EnvShape {
    ENV_LINEAR = 0,
    ENV_EXPO   = 1,
    ENV_QUARTIC = 2
};

static const uint16_t kMaxSegments = 6;

// ── Multistage Envelope ─────────────────────────────────────────────────────

struct EnvelopeQ15 {
    int32_t level[kMaxSegments];           // Target levels per segment (Q15)
    int32_t time_param[kMaxSegments];      // Time parameter for LUT lookup (Q15)
    EnvShape shape[kMaxSegments];          // Shape curve per segment
    
    int16_t segment;                       // Current segment index
    int32_t start_value;                   // Value at start of current segment
    int32_t value;                         // Current output value (Q15)
    int32_t phase;                         // Phase within segment (Q15, 0..32767)
    
    uint16_t num_segments;
    uint16_t sustain_point;                // Segment to hold during gate
    uint16_t loop_start;
    uint16_t loop_end;
    bool hard_reset;
    
    void Init() {
        for (int i = 0; i < kMaxSegments; i++) {
            level[i] = 0;
            time_param[i] = 0;
            shape[i] = ENV_LINEAR;
        }
        segment = 0;
        start_value = 0;
        value = 0;
        phase = 0;
        num_segments = 2;
        sustain_point = 0;
        loop_start = 0;
        loop_end = 0;
        hard_reset = true;
    }
    
    /// Process one sample. flags contains gate/edge information.
    /// Returns Q15 envelope value (0..32767).
    int32_t __not_in_flash_func(Process)(uint8_t flags) {
        if (flags & ENV_FLAG_RISING) {
            start_value = (segment == num_segments || hard_reset) 
                ? level[0] : value;
            segment = 0;
            phase = 0;
        } else if ((flags & ENV_FLAG_FALLING) && sustain_point) {
            start_value = value;
            segment = sustain_point;
            phase = 0;
        } else if (phase >= 32767) {
            start_value = level[segment + 1];
            ++segment;
            phase = 0;
            if (segment == loop_end) {
                segment = loop_start;
            }
        }
        
        bool done = (segment == num_segments);
        bool sustained = sustain_point && (segment == sustain_point) 
                         && (flags & ENV_FLAG_GATE);
        
        int32_t phase_increment = 0;
        if (!sustained && !done) {
            // Look up phase increment from time parameter
            // time_param is Q15 (0..32767), table has 258 entries
            int32_t idx_q8 = (int32_t)(((int64_t)time_param[segment] * 257) >> 7);
            phase_increment = InterpolateQ16(lut_env_increments_q20, idx_q8);
            
            // Scale phase increment by 4/3 to compensate for running at 24kHz instead of 32kHz
            // Formula: (phase_increment >> 5) * 4 / 3 = (phase_increment >> 3) / 3
            phase_increment = (phase_increment >> 3) / 3;
        }
        
        // Look up shape curve
        // phase is Q15 (0..32767), shape table has 258 entries
        int32_t t_idx_q8 = (int32_t)(((int64_t)(phase & 0x7FFF) * 257) >> 7);
        const int16_t* shape_table = lut_env_shapes_q15[shape[segment]];
        int32_t t = InterpolateQ15(shape_table, t_idx_q8);
        
        phase += phase_increment;
        if (phase > 32767) phase = 32767;  // Clamp instead of wrap
        
        // Interpolate between start and target level
        int32_t target = (segment < num_segments) ? level[segment + 1] : level[num_segments];
        value = start_value + (int32_t)(((int64_t)(target - start_value) * t) >> 15);
        
        return value;
    }
    
    // ── Preset configurations ───────────────────────────────────────────
    
    /// Attack-Decay envelope (triggered, no sustain)
    void SetAD(int32_t attack_q15, int32_t decay_q15) {
        num_segments = 2;
        sustain_point = 0;
        level[0] = 0;
        level[1] = 32767;
        level[2] = 0;
        time_param[0] = attack_q15;
        time_param[1] = decay_q15;
        shape[0] = ENV_LINEAR;
        shape[1] = ENV_EXPO;
        loop_start = loop_end = 0;
    }
    
    /// Attack-Release envelope (gated, sustain at peak)
    void SetAR(int32_t attack_q15, int32_t release_q15) {
        num_segments = 2;
        sustain_point = 1;
        level[0] = 0;
        level[1] = 32767;
        level[2] = 0;
        time_param[0] = attack_q15;
        time_param[1] = release_q15;
        shape[0] = ENV_LINEAR;
        shape[1] = ENV_LINEAR;
        loop_start = loop_end = 0;
    }
    
    /// ADSR envelope
    void SetADSR(int32_t attack_q15, int32_t decay_q15, 
                 int32_t sustain_q15, int32_t release_q15) {
        num_segments = 3;
        sustain_point = 2;
        level[0] = 0;
        level[1] = 32767;
        level[2] = sustain_q15;
        level[3] = 0;
        time_param[0] = attack_q15;
        time_param[1] = decay_q15;
        time_param[2] = release_q15;
        shape[0] = ENV_QUARTIC;
        shape[1] = ENV_EXPO;
        shape[2] = ENV_EXPO;
        loop_start = loop_end = 0;
    }
    
    /// Attack-Decay-Release (no sustain, but with a middle level)
    void SetADR(int32_t attack_q15, int32_t decay_q15, 
                int32_t sustain_q15, int32_t release_q15) {
        num_segments = 3;
        sustain_point = 0;
        level[0] = 0;
        level[1] = 32767;
        level[2] = sustain_q15;
        level[3] = 0;
        time_param[0] = attack_q15;
        time_param[1] = decay_q15;
        time_param[2] = release_q15;
        shape[0] = ENV_LINEAR;
        shape[1] = ENV_LINEAR;
        shape[2] = ENV_LINEAR;
        loop_start = loop_end = 0;
    }
    
    /// Looped AD (LFO-like cycling)
    void SetADLoop(int32_t attack_q15, int32_t decay_q15) {
        num_segments = 2;
        sustain_point = 0;
        level[0] = 0;
        level[1] = 32767;
        level[2] = 0;
        time_param[0] = attack_q15;
        time_param[1] = decay_q15;
        shape[0] = ENV_LINEAR;
        shape[1] = ENV_LINEAR;
        loop_start = 0;
        loop_end = 2;
    }
};

#endif  // ENVELOPE_Q15_H_
