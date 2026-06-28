#define COMPUTERCARD_SAMPLE_RATE_DIV 1
#include "ComputerCard.h"
#include "hardware/clocks.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <stdlib.h>
#include <string.h>

// -----------------------------------------------------------------------------
// Constants and Data Structures
// -----------------------------------------------------------------------------

#define SAMPLE_RATE 48000
#define ROOM_DELAY_SIZE 32768  // 32768 samples = 0.68 seconds of delay at 48kHz

#define SAVE_FLASH_OFFSET 0x000F0000

union FlashSaveBuffer {
    struct {
        uint32_t magic;
        int32_t  stored_knob_Main[2];
        int32_t  stored_knob_X[2];
        int32_t  stored_knob_Y[2];
        int32_t  room_size_param;
        int32_t  room_delay_param;
        int32_t  room_fb_param;
        uint32_t checksum;
    } data;
    uint8_t bytes[256];
};

// Fast integer-only approximation of sine.
// Input phase ranges from 0 to 32767 representing a full circle (0 to 2*pi).
// Output is in Q15 format (-32768 to 32767).
inline int32_t fast_sin_q15(uint32_t phase) {
    phase &= 32767;
    if (phase < 16384) {
        // Quadratic curve approximation: sin(x) approx 4*x*(pi-x)/pi^2
        return (phase * (16384 - phase)) >> 11;
    } else {
        uint32_t p = phase - 16384;
        return -((int32_t)((p * (16384 - p)) >> 11));
    }
}

// Fast integer-only approximation of cosine.
inline int32_t fast_cos_q15(uint32_t phase) {
    return fast_sin_q15(phase + 8192);
}

// Fractional Delay Line using linear interpolation.
// Used for the dynamic ITD (Interaural Time Difference) path.
struct DelayLine {
    int16_t buffer[64];
    int writePtr = 0;
    
    void init() {
        memset(buffer, 0, sizeof(buffer));
        writePtr = 0;
    }
    
    void write(int16_t sample) {
        buffer[writePtr] = sample;
        writePtr = (writePtr + 1) & 63;
    }
    
    // Reads a sample with a fractional delay specified in Q16 format (delay * 65536).
    int16_t read_q16(uint32_t delay_q16) {
        uint32_t delayInt = delay_q16 >> 16;
        uint32_t delayFrac = delay_q16 & 0xFFFF; // Fractional part
        
        int i0 = (writePtr - (int)delayInt - 1) & 63;
        int i1 = (i0 + 1) & 63;
        
        int32_t s0 = buffer[i0];
        int32_t s1 = buffer[i1];
        
        // Linear interpolation: s0 + (s1 - s0) * delayFrac
        int32_t out = s0 + (((s1 - s0) * (int32_t)delayFrac) >> 16);
        return (int16_t)out;
    }
};

// First-order digital IIR filter approximating Duda's Head Shadow model.
// Represents the ILD (Interaural Level Difference) caused by head obstruction.
// Implemented using Q13 coefficients to prevent 32-bit signed overflow.
struct HeadShadowFilter {
    int32_t x1 = 0;
    int32_t y1 = 0;
    
    void init() {
        x1 = 0;
        y1 = 0;
    }
    
    int32_t process(int32_t x0, int32_t g) {
        // g: azimuth-dependent shadowing factor in Q15 (-32768 to 32767)
        // For Fs = 48kHz:
        // B0 = 8192 + (g * 7042 >> 15)
        // B1 = -5892 - (g * 7042 >> 15)
        // A1 = -5892 (constant representing 7.8kHz pole frequency)
        int32_t prod = (g * 7042) >> 15;
        int32_t B0 = 8192 + prod;
        int32_t B1 = -5892 - prod;
        int32_t A1 = -5892;
        
        // y0 = (B0 * x0 + B1 * x1 - A1 * y1) >> 13
        int32_t y0 = (B0 * x0 + B1 * x1 - A1 * y1) >> 13;
        
        // Safety clamp to prevent windup
        if (y0 > 32767) y0 = 32767;
        else if (y0 < -32768) y0 = -32768;
        
        x1 = x0;
        y1 = y0;
        return y0;
    }
};

// Dynamic 1-pole Low-Pass Filter.
// Cutoff frequency changes dynamically by modifying coefficient alpha.
struct DynamicLPF {
    int32_t y1 = 0;
    
    void init() {
        y1 = 0;
    }
    
    int32_t process(int32_t x0, int32_t alpha_q15) {
        // alpha_q15 is 0 (fully closed) to 32768 (fully open)
        int32_t y0 = y1 + ((alpha_q15 * (x0 - y1)) >> 15);
        y1 = y0;
        return y0;
    }
};

// DC Blocker filter to prevent offset build-up from feedback and op-amp interfaces.
struct DCBlock {
    int32_t px = 0;
    int32_t py = 0;
    
    void init() {
        px = 0;
        py = 0;
    }
    
    int32_t process(int32_t x) {
        int32_t y = x - px + py - (py >> 8);
        px = x;
        py = y;
        return y;
    }
};

// Dattorro Plate Reverb. Reused and adapted from grains.cpp.
// It creates a lush, diffuse late-field reverberation tail.
class PlateReverb {
  struct AP {
    int16_t *bufIn, *bufOut;
    uint16_t mask, ptr, len;
    int16_t g;
    void init(int16_t *b, uint16_t m, uint16_t l, int16_t gain) {
      bufIn = b;
      bufOut = b + m + 1;
      mask = m;
      len = l;
      g = gain;
      ptr = 0;
    }
    int16_t process(int16_t in) {
      uint16_t rd = (ptr - len) & mask;
      int16_t dIn = bufIn[rd], dOut = bufOut[rd];
      int32_t interm =
          ((int32_t)(g * in) >> 15) + dIn - ((int32_t)(dOut * g) >> 15);
      if (interm > 32767)
        interm = 32767;
      else if (interm < -32768)
        interm = -32768;
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
      len = l;
      ptr = 0;
    }
    void write(int16_t in) {
      buf[ptr] = in;
      ptr = (ptr + 1) & mask;
    }
    int16_t read() { return buf[(ptr - len) & mask]; }
  };
  int16_t mem[35840];
  AP apIn[4], apTankL, apTankR;
  Delay modL, d1L, d2L, modR, d1R, d2R;
  int32_t lpL = 0, lpR = 0, lpIn = 0;
  uint32_t lfo = 0;

public:
  PlateReverb() {
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
  }
  void __not_in_flash_func(process)(int32_t &outL, int32_t &outR, int32_t mix,
                                    int32_t decay, int32_t damp) {
    if (mix < 10)
      return;
    int16_t mono = (int16_t)((outL + outR) >> 1);
    lpIn += (((int32_t)mono - lpIn) * 16000) >> 15;
    mono = (int16_t)(lpIn >> 1);
    for (int i = 0; i < 4; i++)
      mono = apIn[i].process(mono);
    lfo += 4096;
    int16_t mod = (lfo >> 16) & 0x7FFF;
    if (lfo & 0x80000000)
      mod = 32767 - mod;
    int16_t tOutL = d2L.read(), tOutR = d2R.read();
    
    // Left channel tank feedback path
    int32_t iL = (int32_t)mono + (((int32_t)decay * tOutR) >> 15);
    if (iL > 32767) iL = 32767;
    else if (iL < -32768) iL = -32768;
    
    int32_t sL_32 = iL + (((int32_t)16384 * modL.read()) >> 15);
    if (sL_32 > 32767) sL_32 = 32767;
    else if (sL_32 < -32768) sL_32 = -32768;
    int16_t sL = (int16_t)sL_32;
    
    int32_t modL_val = iL - (((int32_t)16384 * sL) >> 15);
    if (modL_val > 32767) modL_val = 32767;
    else if (modL_val < -32768) modL_val = -32768;
    
    modL.write((int16_t)modL_val);
    d1L.write(sL);
    sL = d1L.read();
    lpL += (((int32_t)sL - lpL) * damp) >> 15;
    sL = (int16_t)lpL;
    sL = apTankL.process(sL);
    d2L.write(sL);
    
    // Right channel tank feedback path
    int32_t iR = (int32_t)mono + (((int32_t)decay * tOutL) >> 15);
    if (iR > 32767) iR = 32767;
    else if (iR < -32768) iR = -32768;
    
    int32_t sR_32 = iR + (((int32_t)16384 * modR.read()) >> 15);
    if (sR_32 > 32767) sR_32 = 32767;
    else if (sR_32 < -32768) sR_32 = -32768;
    int16_t sR = (int16_t)sR_32;
    
    int32_t modR_val = iR - (((int32_t)16384 * sR) >> 15);
    if (modR_val > 32767) modR_val = 32767;
    else if (modR_val < -32768) modR_val = -32768;
    
    modR.write((int16_t)modR_val);
    d1R.write(sR);
    sR = d1R.read();
    lpR += (((int32_t)sR - lpR) * damp) >> 15;
    sR = (int16_t)lpR;
    sR = apTankR.process(sR);
    d2R.write(sR);
    
    outL = (outL * (32767 - mix) + (int32_t)sL * mix) >> 15;
    outR = (outR * (32767 - mix) + (int32_t)sR * mix) >> 15;
  }
};

// UI Parameter Knob Lock utility.
// Locks a parameter until the physical pot matches the stored setting.
struct KnobLock {
  bool locked = true;
  int32_t ref = 0;
  void engage(int32_t v) {
    locked = true;
    ref = v;
  }
  bool update(int32_t v) {
    if (locked) {
      int32_t d = v - ref;
      if (d < 0) d = -d;
      if (d > 1638) // Unlock when the physical knob wiggles by more than 5%
        locked = false;
    }
    return !locked;
  }
};

// -----------------------------------------------------------------------------
// Core Worker Thread Prototype
// -----------------------------------------------------------------------------

void core1_worker();

// -----------------------------------------------------------------------------
// Rooms Card Class Definition
// -----------------------------------------------------------------------------

class Rooms : public ComputerCard {
public:
    // Shared position variables for Source 0 (Input 1) and Source 1 (Input 2)
    volatile int32_t target_azimuth[2] = {0, 0};      // 0 to 32767 (0 to 360 degrees)
    volatile int32_t target_distance[2] = {8000, 8000}; // 0 (closest) to 32767 (furthest)
    volatile int32_t target_width[2] = {0, 0};        // 0 (pinpoint) to 32767 (wide)
    
    volatile int32_t current_azimuth[2] = {0, 0};
    volatile int32_t current_distance[2] = {8000, 8000};
    volatile int32_t current_width[2] = {0, 0};

    // Real-time CV-modulated positions written on Core 0, read on Core 1 for LED visualizer
    volatile int32_t dynamic_azimuth[2] = {0, 0};
    volatile int32_t dynamic_distance[2] = {8000, 8000};
    volatile int32_t dynamic_width[2] = {0, 0};
    
    // Stored knob values for soft takeover locking
    int32_t stored_knob_Main[2] = {16384, 16384};
    int32_t stored_knob_X[2] = {8000, 8000};
    int32_t stored_knob_Y[2] = {16384, 16384};
    
    // Shared room/delay setup variables (configured in Switch Down)
    volatile int32_t room_size_param = 16384;  // Main knob: Reverb Size
    volatile int32_t room_delay_param = 8000;  // X knob: Delay Time (Bypassed if < 500)
    volatile int32_t room_fb_param = 0;        // Y knob: Feedback
    volatile int32_t selectedInput = 0;        // 0 = AudioIn1 (Up), 1 = AudioIn2 (Middle)
    
    // UI timers for displaying parameter bar graphs on LEDs
    volatile int32_t displayTimer = 0;
    volatile int32_t displayParamIdx = 0; // 0 = Reverb, 1 = Delay, 2 = Feedback
    
    // Interrupt-safe trigger buffering
    volatile bool trigger1Buffered = false;
    volatile bool trigger2Buffered = false;
    
    // Output trigger pulse counters (decremented in ProcessSample at 48kHz)
    volatile int32_t zero_cross_pulse_timer = 0;
    
    // Cache inputs from ProcessSample (thread-safe UI update source)
    volatile int32_t cachedKnobMain = 0;
    volatile int32_t cachedKnobX = 0;
    volatile int32_t cachedKnobY = 0;
    volatile Switch cachedSwitch = Switch::Middle;
    volatile bool cachedPulseIn2 = false;
    
    volatile int32_t room_fb_gain = 0;
    volatile int32_t refl_gain = 32768; // Delay bypass flag (0 = off, 32768 = on)
    
    // Knob locks for mode changes
    KnobLock lockMain, lockX, lockY;
    
    // DSP Objects for Source 0 (Audio Input 1)
    DynamicLPF directLPF_L0, directLPF_R0;
    DelayLine directDelayL0, directDelayR0;
    HeadShadowFilter head_shadow_L0, head_shadow_R0;
    
    // DSP Objects for Source 1 (Audio Input 2)
    DynamicLPF directLPF_L1, directLPF_R1;
    DelayLine directDelayL1, directDelayR1;
    HeadShadowFilter head_shadow_L1, head_shadow_R1;
    
    // Shared Reverb and DC blockers
    PlateReverb plate;
    DCBlock dc_L, dc_R;
    
    Rooms() {
        directLPF_L0.init(); directLPF_R0.init();
        directLPF_L1.init(); directLPF_R1.init();
        directDelayL0.init(); directDelayR0.init();
        directDelayL1.init(); directDelayR1.init();
        head_shadow_L0.init(); head_shadow_R0.init();
        head_shadow_L1.init(); head_shadow_R1.init();
        dc_L.init(); dc_R.init();
        
        lockMain.engage(0);
        lockX.engage(0);
        lockY.engage(0);
    }
    
    // Main audio sample processor. Called at 48kHz interrupt.
    void ProcessSample() override {
        // Poll pulse inputs immediately at 48kHz to detect triggers for randomization
        static uint32_t rand_seed = 12345;
        if (PulseIn1RisingEdge()) {
            rand_seed = 1664525 * rand_seed + 1013904223;
            int32_t r_kMain = rand_seed & 32767;
            int32_t raw_phase = ((r_kMain - 16384) * 5) / 6;
            if (raw_phase < 0) {
                raw_phase += 32768;
            }
            target_azimuth[0] = raw_phase & 32767;
            target_distance[0] = (rand_seed >> 15) & 32767;
            target_width[0] = (rand_seed >> 7) & 32767;
            trigger1Buffered = true;
        }
        if (PulseIn2RisingEdge()) {
            rand_seed = 1664525 * rand_seed + 1013904223;
            int32_t r_kMain = rand_seed & 32767;
            int32_t raw_phase = ((r_kMain - 16384) * 5) / 6;
            if (raw_phase < 0) {
                raw_phase += 32768;
            }
            target_azimuth[1] = raw_phase & 32767;
            target_distance[1] = (rand_seed >> 15) & 32767;
            target_width[1] = (rand_seed >> 7) & 32767;
            trigger2Buffered = true;
        }

        // Cache physical hardware inputs for Core 1 (interrupt-safe)
        cachedKnobMain = KnobVal(Knob::Main);
        cachedKnobX = KnobVal(Knob::X);
        cachedKnobY = KnobVal(Knob::Y);
        cachedSwitch = SwitchVal();
        cachedPulseIn2 = PulseIn2();
        
        // Output zero-crossing trigger pulse (decremented at 48kHz for accuracy)
        if (zero_cross_pulse_timer > 0) {
            zero_cross_pulse_timer--;
            PulseOut1(true);
        } else {
            PulseOut1(false);
        }
        
        // Pulse Out 2 is unused since LFO was removed
        PulseOut2(false);
        
        // 1. Read input audio channels and scale to 16-bit range
        int32_t dry0 = AudioIn1() << 4;
        int32_t dry1 = AudioIn2() << 4;
        
        // 2. Read CV inputs for independent source azimuth modulation with 1-pole LPF to reduce noise
        static int32_t smoothed_CV1 = 0;
        static int32_t smoothed_CV2 = 0;
        smoothed_CV1 += (CVIn1() - (smoothed_CV1 >> 4));
        smoothed_CV2 += (CVIn2() - (smoothed_CV2 >> 4));
        int32_t mod_azimuth0 = (smoothed_CV1 >> 4) * 8;  // Scale -2048..2047 to -16384..16384
        int32_t mod_azimuth1 = (smoothed_CV2 >> 4) * 8;
        
        // --- Process Source 0 (Audio Input 1) ---
        int32_t eff_azi0 = (current_azimuth[0] + mod_azimuth0) & 32767;
        int32_t eff_dist0 = current_distance[0];
        
        // Dynamic depth cues (LPFs) for Source 0
        int32_t alpha_dist0 = 32768 - ((eff_dist0 * 30000) >> 15);
        int32_t alpha_pinna0 = 32768;
        int32_t cos_val0 = fast_cos_q15(eff_azi0);
        if (cos_val0 < 0) {
            alpha_pinna0 = 32768 - (((-cos_val0) * 16384) >> 15);
        }
        
        int32_t alpha_total0 = (alpha_dist0 * alpha_pinna0) >> 15;
        if (alpha_total0 < 800) alpha_total0 = 800;
        
        int32_t direct_filtered_L0 = directLPF_L0.process(dry0, alpha_total0);
        int32_t direct_filtered_R0 = directLPF_R0.process(dry0, alpha_total0);
        
        directDelayL0.write((int16_t)direct_filtered_L0);
        directDelayR0.write((int16_t)direct_filtered_R0);
        
        int32_t sin_val0 = fast_sin_q15(eff_azi0);
        
        uint32_t delayL_q16_0 = 983040 - 30 * sin_val0;  // Delay ranges from 0 to 30 samples (in Q16)
        uint32_t delayR_q16_0 = 983040 + 30 * sin_val0;
        
        int32_t delay_sample_L0 = directDelayL0.read_q16(delayL_q16_0);
        int32_t delay_sample_R0 = directDelayR0.read_q16(delayR_q16_0);
        
        int32_t spat_L0 = head_shadow_L0.process(delay_sample_L0, -sin_val0);
        int32_t spat_R0 = head_shadow_R0.process(delay_sample_R0, sin_val0);
        
        // Decorrelation spatial widening (Gerzon shuffler)
        uint32_t delay_decorr_q16_0 = 1310720; // 20 samples delay
        int32_t decorr_L0 = directDelayL0.read_q16(delay_decorr_q16_0);
        int32_t decorr_R0 = directDelayR0.read_q16(delay_decorr_q16_0);
        
        int32_t w0 = current_width[0] >> 1; // 0 to 16384 (0% to 50% mix)
        int32_t spat_w_L0 = spat_L0 - ((decorr_R0 * w0) >> 15);
        int32_t spat_w_R0 = spat_R0 + ((decorr_L0 * w0) >> 15);
        
        int32_t g_direct0 = 32768 - ((eff_dist0 * 29491) >> 15);
        int32_t outDirectL0 = (spat_w_L0 * g_direct0) >> 15;
        int32_t outDirectR0 = (spat_w_R0 * g_direct0) >> 15;
        
        // --- Process Source 1 (Audio Input 2) ---
        int32_t eff_azi1 = (current_azimuth[1] + mod_azimuth1) & 32767;
        int32_t eff_dist1 = current_distance[1];
        
        // Dynamic depth cues (LPFs) for Source 1
        int32_t alpha_dist1 = 32768 - ((eff_dist1 * 30000) >> 15);
        int32_t alpha_pinna1 = 32768;
        int32_t cos_val1 = fast_cos_q15(eff_azi1);
        if (cos_val1 < 0) {
            alpha_pinna1 = 32768 - (((-cos_val1) * 16384) >> 15);
        }
        
        int32_t alpha_total1 = (alpha_dist1 * alpha_pinna1) >> 15;
        if (alpha_total1 < 800) alpha_total1 = 800;
        
        int32_t direct_filtered_L1 = directLPF_L1.process(dry1, alpha_total1);
        int32_t direct_filtered_R1 = directLPF_R1.process(dry1, alpha_total1);
        
        directDelayL1.write((int16_t)direct_filtered_L1);
        directDelayR1.write((int16_t)direct_filtered_R1);
        
        int32_t sin_val1 = fast_sin_q15(eff_azi1);
        
        uint32_t delayL_q16_1 = 983040 - 30 * sin_val1;
        uint32_t delayR_q16_1 = 983040 + 30 * sin_val1;
        
        int32_t delay_sample_L1 = directDelayL1.read_q16(delayL_q16_1);
        int32_t delay_sample_R1 = directDelayR1.read_q16(delayR_q16_1);
        
        int32_t spat_L1 = head_shadow_L1.process(delay_sample_L1, -sin_val1);
        int32_t spat_R1 = head_shadow_R1.process(delay_sample_R1, sin_val1);
        
        // Decorrelation spatial widening (Gerzon shuffler)
        uint32_t delay_decorr_q16_1 = 1310720; // 20 samples delay
        int32_t decorr_L1 = directDelayL1.read_q16(delay_decorr_q16_1);
        int32_t decorr_R1 = directDelayR1.read_q16(delay_decorr_q16_1);
        
        int32_t w1 = current_width[1] >> 1; // 0 to 16384 (0% to 50% mix)
        int32_t spat_w_L1 = spat_L1 - ((decorr_R1 * w1) >> 15);
        int32_t spat_w_R1 = spat_R1 + ((decorr_L1 * w1) >> 15);
        
        int32_t g_direct1 = 32768 - ((eff_dist1 * 29491) >> 15);
        int32_t outDirectL1 = (spat_w_L1 * g_direct1) >> 15;
        int32_t outDirectR1 = (spat_w_R1 * g_direct1) >> 15;
        
        // Share real-time dynamic coordinates with Core 1 (LED visualizer)
        dynamic_azimuth[0] = eff_azi0;
        dynamic_distance[0] = eff_dist0;
        dynamic_width[0] = current_width[0];
        dynamic_azimuth[1] = eff_azi1;
        dynamic_distance[1] = eff_dist1;
        dynamic_width[1] = current_width[1];
        
        // Check zero crossing of active source azimuth
        int32_t active_azi_cross = (selectedInput == 1) ? eff_azi1 : eff_azi0;
        static uint32_t last_active_azi = 0;
        uint32_t curr_active_azi = active_azi_cross;
        if ((last_active_azi > 31000 && curr_active_azi < 1000) || (last_active_azi < 1000 && curr_active_azi > 31000)) {
            zero_cross_pulse_timer = 480; // 10ms pulse
        }
        last_active_azi = curr_active_azi;
        
        // 5. Room Reflections & Wall boundary Delay Simulation
        int32_t refl_sum_L = 0;
        int32_t refl_sum_R = 0;
        int32_t feedback_val = 0;
        
        if (refl_gain > 0) {
            // Find active source azimuth and distance to calculate position
            int32_t active_azi = (selectedInput == 1) ? eff_azi1 : eff_azi0;
            int32_t active_dist = (selectedInput == 1) ? eff_dist1 : eff_dist0;

            int32_t sin_val = fast_sin_q15(active_azi);
            int32_t cos_val = fast_cos_q15(active_azi);

            // Cartesian coordinates in Q15 range [-32768, 32767]
            int32_t x_s = (sin_val * active_dist) >> 15;
            int32_t y_s = (cos_val * active_dist) >> 15;

            // Delay base time scales from 20ms (960 samples) to 400ms (19200 samples)
            int32_t T_base = 960 + ((room_delay_param * 18240) >> 15);
            
            // Dynamic delay times based on source position relative to the walls
            int32_t T_L = T_base + ((x_s * T_base) >> 16);
            int32_t T_R = T_base - ((x_s * T_base) >> 16);
            int32_t T_F = T_base - ((y_s * T_base) >> 16);
            int32_t T_B = T_base + ((y_s * T_base) >> 16);
            
            // Dynamic reflection gains based on wall proximity
            int32_t g_L = 16384 - (x_s >> 2); // 8192 to 24576
            int32_t g_R = 16384 + (x_s >> 2);
            int32_t g_F = 16384 + (y_s >> 2);
            int32_t g_B = 16384 - (y_s >> 2);

            // Left reflection (Left ear earlier, 25% opposite leak)
            int32_t tap_L_L = room_delay_read(T_L);
            int32_t tap_L_R = room_delay_read(T_L + 30);
            int32_t refl_L_L = (tap_L_L * g_L) >> 15;
            int32_t refl_L_R = (tap_L_R * (g_L >> 2)) >> 15;

            // Right reflection (Right ear earlier)
            int32_t tap_R_L = room_delay_read(T_R + 30);
            int32_t tap_R_R = room_delay_read(T_R);
            int32_t refl_R_L = (tap_R_L * (g_R >> 2)) >> 15;
            int32_t refl_R_R = (tap_R_R * g_R) >> 15;

            // Front reflection (No ITD)
            int32_t tap_F = room_delay_read(T_F);
            int32_t refl_F_L = (tap_F * g_F) >> 16;
            int32_t refl_F_R = (tap_F * g_F) >> 16;

            // Back reflection (No ITD)
            int32_t tap_B = room_delay_read(T_B);
            int32_t refl_B_L = (tap_B * g_B) >> 16;
            int32_t refl_B_R = (tap_B * g_B) >> 16;

            // Mix reflections
            refl_sum_L = refl_L_L + refl_R_L + refl_F_L + refl_B_L;
            refl_sum_R = refl_L_R + refl_R_R + refl_F_R + refl_B_R;
            
            int32_t feedback_sum = tap_L_L + tap_R_R + tap_F + tap_B;
            feedback_val = ((feedback_sum >> 2) * room_fb_gain) >> 15;
        }
        
        // Write the mix of both input signals + feedback to room delay
        int32_t input_with_fb = dry0 + dry1 + feedback_val;
        if (input_with_fb > 32767) input_with_fb = 32767;
        else if (input_with_fb < -32768) input_with_fb = -32768;
        
        room_delay_write((int16_t)input_with_fb);
        
        // 6. Combine Direct & Reflection paths, clamp to 16-bit, and apply Plate Reverb
        int32_t out_L = outDirectL0 + outDirectL1 + refl_sum_L;
        int32_t out_R = outDirectR0 + outDirectR1 + refl_sum_R;
        
        if (out_L > 32767) out_L = 32767;
        else if (out_L < -32768) out_L = -32768;
        if (out_R > 32767) out_R = 32767;
        else if (out_R < -32768) out_R = -32768;
        
        // Reverb mix scales from 10% to ~45%
        int32_t reverb_mix = 3000 + ((room_size_param * 12000) >> 15);
        // Reverb decay scales from 0.3 to 0.87 (28500) for feedback loop stability
        int32_t reverb_decay = 10000 + ((room_size_param * 18500) >> 15);
        int32_t reverb_damp = 16000; // Moderate high-frequency damping
        
        plate.process(out_L, out_R, reverb_mix, reverb_decay, reverb_damp);
        
        // 7. Apply DC blocker and scale back to hardware 12-bit range
        out_L = dc_L.process(out_L);
        out_R = dc_R.process(out_R);
        
        out_L >>= 4;
        out_R >>= 4;
        
        if (out_L > 2047) out_L = 2047;
        else if (out_L < -2048) out_L = -2048;
        if (out_R > 2047) out_R = 2047;
        else if (out_R < -2048) out_R = -2048;
        
        // 8. Expose positioning parameters of active input on CV outputs at 48kHz
        int32_t active_azi_cv = (selectedInput == 1) ? eff_azi1 : eff_azi0;
        int32_t active_dist_cv = (selectedInput == 1) ? eff_dist1 : eff_dist0;
        CVOut1((int16_t)((active_azi_cv >> 3) - 2048));
        CVOut2((int16_t)((active_dist_cv >> 3) - 2048));

        AudioOut1((int16_t)out_L);
        AudioOut2((int16_t)out_R);
    }
    
    // Helper to display continuous parameter value bar graphs on the 2x3 LEDs
    void display_bar_graph(int32_t val) {
        // val ranges from 0 to 32767
        // Each of the 6 LEDs represents a step of 32768 / 6 = 5461
        int32_t numFull = val / 5461;
        int32_t frac = val % 5461;
        
        static const int8_t ledSeq[] = {4, 5, 2, 3, 0, 1}; // Bottom-to-top layout
        for (int i = 0; i < 6; i++) {
            if (i < numFull) {
                LedBrightness(ledSeq[i], 4095);
            } else if (i == numFull) {
                LedBrightness(ledSeq[i], (frac * 4095) / 5461);
            } else {
                LedBrightness(ledSeq[i], 0);
            }
        }
    }
    
    // Member function to handle control-rate UI and positioning logic (runs on Core 1)
    void UpdateControls() {
        Switch sw = cachedSwitch;
        
        auto scaleK = [](int32_t v) {
            int32_t res = (v - 50) * 32767 / (4095 - 100);
            if (res < 0) res = 0;
            if (res > 32767) res = 32767;
            return res;
        };
        
        int32_t kMain = scaleK(cachedKnobMain);
        int32_t kX = scaleK(cachedKnobX);
        int32_t kY = scaleK(cachedKnobY);
        
        bool param_changed = false;
        
        // Boot initialization
        static bool firstRun = true;
        if (firstRun) {
            firstRun = false;

            // Load saved settings from flash if valid
            bool loaded = false;
            const FlashSaveBuffer *saved = (const FlashSaveBuffer *)(0x10000000 + SAVE_FLASH_OFFSET);
            if (saved->data.magic == 0x47524f4f) { // Magic code: "ROOM"
                uint32_t sum = 0;
                for (size_t i = 0; i < 2; i++) {
                    sum += saved->data.stored_knob_Main[i];
                    sum += saved->data.stored_knob_X[i];
                    sum += saved->data.stored_knob_Y[i];
                }
                sum += saved->data.room_size_param;
                sum += saved->data.room_delay_param;
                sum += saved->data.room_fb_param;
                if (sum == saved->data.checksum) {
                    stored_knob_Main[0] = saved->data.stored_knob_Main[0];
                    stored_knob_Main[1] = saved->data.stored_knob_Main[1];
                    stored_knob_X[0] = saved->data.stored_knob_X[0];
                    stored_knob_X[1] = saved->data.stored_knob_X[1];
                    stored_knob_Y[0] = saved->data.stored_knob_Y[0];
                    stored_knob_Y[1] = saved->data.stored_knob_Y[1];
                    room_size_param = saved->data.room_size_param;
                    room_delay_param = saved->data.room_delay_param;
                    room_fb_param = saved->data.room_fb_param;
                    loaded = true;
                }
            }

            if (!loaded) {
                // Set stored knob values to the current physical positions
                stored_knob_Main[0] = kMain; stored_knob_Main[1] = kMain;
                stored_knob_X[0] = kX; stored_knob_X[1] = kX;
                stored_knob_Y[0] = kY; stored_knob_Y[1] = kY;
                
                room_size_param = 16384;
                room_delay_param = 8000;
                room_fb_param = 0;
            }

            // Engage locks with the current physical knob positions on boot
            lockMain.engage(kMain);
            lockX.engage(kX);
            lockY.engage(kY);
            
            // Set targets to match loaded/default parameters
            for (int s = 0; s < 2; s++) {
                int32_t raw_phase = ((stored_knob_Main[s] - 16384) * 5) / 6;
                if (raw_phase < 0) {
                    raw_phase += 32768;
                }
                target_azimuth[s] = raw_phase & 32767;
                target_distance[s] = stored_knob_X[s];
                target_width[s] = stored_knob_Y[s];
                
                // Set current coordinates to target coordinates immediately to prevent glide on boot
                current_azimuth[s] = target_azimuth[s];
                current_distance[s] = target_distance[s];
                current_width[s] = target_width[s];
            }
        }
        
        // Lock knobs if a randomization trigger occurred
        if (trigger1Buffered) {
            trigger1Buffered = false;
            
            // Calculate stored knob values matching the new randomized target position
            int32_t phase = target_azimuth[0];
            int32_t angle = (phase > 16384) ? (phase - 32768) : phase;
            stored_knob_Main[0] = 16384 + (angle * 6) / 5;
            stored_knob_X[0] = target_distance[0];
            stored_knob_Y[0] = target_width[0];
            
            if (sw == Switch::Up) {
                lockMain.engage(kMain);
                lockX.engage(kX);
                lockY.engage(kY);
            }
            param_changed = true;
        }
        if (trigger2Buffered) {
            trigger2Buffered = false;
            
            // Calculate stored knob values matching the new randomized target position
            int32_t phase = target_azimuth[1];
            int32_t angle = (phase > 16384) ? (phase - 32768) : phase;
            stored_knob_Main[1] = 16384 + (angle * 6) / 5;
            stored_knob_X[1] = target_distance[1];
            stored_knob_Y[1] = target_width[1];
            
            if (sw == Switch::Middle) {
                lockMain.engage(kMain);
                lockX.engage(kX);
                lockY.engage(kY);
            }
            param_changed = true;
        }
        
        // Switch position transition lock handler.
        // Sets locking references to the physical knob positions.
        static Switch last_sw = Switch::Middle;
        static bool last_sw_inited = false;
        if (!last_sw_inited) {
            last_sw = sw;
            last_sw_inited = true;
            if (sw == Switch::Middle) {
                selectedInput = 1;
            } else {
                selectedInput = 0;
            }
        }
        if (sw != last_sw) {
            // Engage locks with physical knob positions
            lockMain.engage(kMain);
            lockX.engage(kX);
            lockY.engage(kY);
            if (sw == Switch::Up) {
                selectedInput = 0; // Audio Input 1
            } else if (sw == Switch::Middle) {
                selectedInput = 1; // Audio Input 2
            }
            last_sw = sw;
        }
        
        // Update active targets based on current Switch position
        if (sw == Switch::Up) {
            if (lockMain.update(kMain)) {
                // Map physical knob rotation (0..32767) to azimuth range -150 to +150 degrees
                // Center (16384) represents 0 degrees (straight ahead).
                int32_t raw_phase = ((kMain - 16384) * 5) / 6;
                if (raw_phase < 0) {
                    raw_phase += 32768;
                }
                int32_t new_azi = raw_phase & 32767;
                if (target_azimuth[0] != new_azi) {
                    target_azimuth[0] = new_azi;
                    stored_knob_Main[0] = kMain; // Save the updated physical knob value!
                    param_changed = true;
                }
            }
            if (lockX.update(kX)) {
                if (target_distance[0] != kX) {
                    target_distance[0] = kX;
                    stored_knob_X[0] = kX;
                    param_changed = true;
                }
            }
            if (lockY.update(kY)) {
                if (target_width[0] != kY) {
                    target_width[0] = kY;
                    stored_knob_Y[0] = kY;
                    param_changed = true;
                }
            }
        } else if (sw == Switch::Middle) {
            if (lockMain.update(kMain)) {
                int32_t raw_phase = ((kMain - 16384) * 5) / 6;
                if (raw_phase < 0) {
                    raw_phase += 32768;
                }
                int32_t new_azi = raw_phase & 32767;
                if (target_azimuth[1] != new_azi) {
                    target_azimuth[1] = new_azi;
                    stored_knob_Main[1] = kMain; // Save the updated physical knob value!
                    param_changed = true;
                }
            }
            if (lockX.update(kX)) {
                if (target_distance[1] != kX) {
                    target_distance[1] = kX;
                    stored_knob_X[1] = kX;
                    param_changed = true;
                }
            }
            if (lockY.update(kY)) {
                if (target_width[1] != kY) {
                    target_width[1] = kY;
                    stored_knob_Y[1] = kY;
                    param_changed = true;
                }
            }
        } else { // Switch::Down (Room Setup)
            // If the physical knob moves, we trigger the bar graph of the stored parameter
            // to show the user the target value they need to match!
            static int32_t last_kMain = 0, last_kX = 0, last_kY = 0;
            if (kMain != last_kMain) {
                displayTimer = 750;
                displayParamIdx = 0;
                last_kMain = kMain;
            }
            if (kX != last_kX) {
                displayTimer = 750;
                displayParamIdx = 1;
                last_kX = kX;
            }
            if (kY != last_kY) {
                displayTimer = 750;
                displayParamIdx = 2;
                last_kY = kY;
            }

            if (lockMain.update(kMain)) {
                if (room_size_param != kMain) {
                    room_size_param = kMain;
                    param_changed = true;
                }
            }
            if (lockX.update(kX)) {
                if (room_delay_param != kX) {
                    room_delay_param = kX;
                    param_changed = true;
                }
            }
            if (lockY.update(kY)) {
                if (room_fb_param != kY) {
                    room_fb_param = kY;
                    param_changed = true;
                }
            }
        }
        
        // Delay bypass evaluation
        if (room_delay_param < 500) {
            refl_gain = 0; // Turn room boundary delay completely OFF
        } else {
            refl_gain = 32768;
        }
        
        // Feedback gain scales with Y knob
        room_fb_gain = (room_fb_param * 28000) >> 15; // Max feedback 0.85
        
        // Smooth positioning variables to prevent clicks/abrupt jumps
        for (int s = 0; s < 2; s++) {
            int32_t diff = target_azimuth[s] - current_azimuth[s];
            if (diff > 16384) diff -= 32768;
            else if (diff < -16384) diff += 32768;
            current_azimuth[s] = (current_azimuth[s] + (diff >> 5)) & 32767; // 30ms time constant
            
            current_distance[s] += (target_distance[s] - current_distance[s]) >> 5;
            current_width[s] += (target_width[s] - current_width[s]) >> 5;
        }

        // Non-blocking Flash Auto-Save
        static bool save_dirty = false;
        static uint32_t save_cooldown_timer = 0;
        
        if (param_changed) {
            save_dirty = true;
            save_cooldown_timer = 1250; // 2.5 seconds at 500Hz
        }
        
        if (save_dirty) {
            if (save_cooldown_timer > 0) {
                save_cooldown_timer--;
            } else {
                save_dirty = false;
                
                union FlashSaveBuffer buf;
                memset(buf.bytes, 0, 256);
                buf.data.magic = 0x47524f4f; // "ROOM"
                buf.data.stored_knob_Main[0] = stored_knob_Main[0];
                buf.data.stored_knob_Main[1] = stored_knob_Main[1];
                buf.data.stored_knob_X[0] = stored_knob_X[0];
                buf.data.stored_knob_X[1] = stored_knob_X[1];
                buf.data.stored_knob_Y[0] = stored_knob_Y[0];
                buf.data.stored_knob_Y[1] = stored_knob_Y[1];
                buf.data.room_size_param = room_size_param;
                buf.data.room_delay_param = room_delay_param;
                buf.data.room_fb_param = room_fb_param;
                
                uint32_t sum = 0;
                for (size_t i = 0; i < 2; i++) {
                    sum += stored_knob_Main[i];
                    sum += stored_knob_X[i];
                    sum += stored_knob_Y[i];
                }
                sum += room_size_param;
                sum += room_delay_param;
                sum += room_fb_param;
                buf.data.checksum = sum;
                
                uint32_t ints = save_and_disable_interrupts();
                flash_range_erase(SAVE_FLASH_OFFSET, FLASH_SECTOR_SIZE);
                flash_range_program(SAVE_FLASH_OFFSET, buf.bytes, 256);
                restore_interrupts(ints);
            }
        }
    }
    
    // Member function to handle LED lighting updates (runs on Core 1)
    void UpdateLEDs() {
        if (displayTimer > 0) {
            displayTimer--;
            int32_t val = 0;
            if (displayParamIdx == 0) val = room_size_param;
            else if (displayParamIdx == 1) val = room_delay_param;
            else val = room_fb_param;
            
            display_bar_graph(val);
        } else {
            // Directional visualizer projection for Source 0 and Source 1
            int32_t intensity0 = 0;
            int32_t intensity1 = 0;

            if (cachedSwitch == Switch::Up || cachedSwitch == Switch::Down) {
                int32_t dist0 = dynamic_distance[0];
                intensity0 = 4095 - (dist0 >> 3); // 4095 to 0
                if (intensity0 < 0) intensity0 = 0;
            }
            if (cachedSwitch == Switch::Middle || cachedSwitch == Switch::Down) {
                int32_t dist1 = dynamic_distance[1];
                intensity1 = 4095 - (dist1 >> 3);
                if (intensity1 < 0) intensity1 = 0;
            }

            int32_t azi0 = dynamic_azimuth[0];
            int32_t azi1 = dynamic_azimuth[1];

            // Target angles for the 6 LEDs in Q15 circle range 0..32767
            // 0: Top-L (-45 deg) = 28672
            // 1: Top-R (+45 deg) = 4096
            // 2: Mid-L (-90 deg) = 24576
            // 3: Mid-R (+90 deg) = 8192
            // 4: Bot-L (-135 deg) = 20480
            // 5: Bot-R (+135 deg) = 12288
            static const int32_t led_angles[6] = { 28672, 4096, 24576, 8192, 20480, 12288 };

            int32_t leds[6] = {0, 0, 0, 0, 0, 0};

            int32_t th0 = 8192 + ((dynamic_width[0] * 8192) >> 15);
            int32_t th1 = 8192 + ((dynamic_width[1] * 8192) >> 15);

            for (int i = 0; i < 6; i++) {
                // Source 0 weight (window scales with width setting)
                int32_t diff0 = led_angles[i] - azi0;
                if (diff0 > 16384) diff0 -= 32768;
                else if (diff0 < -16384) diff0 += 32768;
                if (diff0 < 0) diff0 = -diff0;

                int32_t w0 = 0;
                if (diff0 < th0) {
                    w0 = 32768 - (diff0 * 32768) / th0;
                }

                // Source 1 weight (window scales with width setting)
                int32_t diff1 = led_angles[i] - azi1;
                if (diff1 > 16384) diff1 -= 32768;
                else if (diff1 < -16384) diff1 += 32768;
                if (diff1 < 0) diff1 = -diff1;

                int32_t w1 = 0;
                if (diff1 < th1) {
                    w1 = 32768 - (diff1 * 32768) / th1;
                }

                int32_t b0 = (w0 * intensity0) >> 15;
                int32_t b1 = (w1 * intensity1) >> 15;

                leds[i] = b0 + b1;
            }
            
            for (int i = 0; i < 6; i++) {
                int32_t brightness = leds[i];
                if (brightness > 4095) brightness = 4095;
                if (brightness < 0) brightness = 0;
                
                // Overlay a breathing pulse pattern if the user is in Setup Mode (Switch Down)
                if (cachedSwitch == Switch::Down) {
                    static uint32_t breath = 0;
                    breath++;
                    int32_t pulse = fast_sin_q15((breath >> 2) & 32767) >> 6; // range [-512, 511]
                    brightness = (brightness * (3584 + pulse)) >> 12;
                }
                
                LedBrightness(i, (uint16_t)brightness);
            }
        }
    }

private:
    // Room boundary delay buffer
    int16_t roomDelayBuffer[ROOM_DELAY_SIZE] __attribute__((aligned(4)));
    int32_t roomDelayWritePtr = 0;
    
    void room_delay_write(int16_t sample) {
        roomDelayBuffer[roomDelayWritePtr] = sample;
        roomDelayWritePtr = (roomDelayWritePtr + 1) & (ROOM_DELAY_SIZE - 1);
    }
    
    int16_t room_delay_read(int32_t delaySamples) {
        int32_t readPtr = (roomDelayWritePtr - delaySamples) & (ROOM_DELAY_SIZE - 1);
        return roomDelayBuffer[readPtr];
    }
};

// -----------------------------------------------------------------------------
// Core 1 Worker (Control Thread Loop)
// -----------------------------------------------------------------------------

void core1_worker() {
    // Wait for multicore launch handshake
    while (multicore_fifo_rvalid()) {
        multicore_fifo_pop_blocking();
    }
    
    while (1) {
        // Run loop at 500Hz
        sleep_ms(2);
        
        Rooms *card = (Rooms *)ComputerCard::ThisPtr();
        if (!card) continue;
        
        card->UpdateControls();
        card->UpdateLEDs();
    }
}

// -----------------------------------------------------------------------------
// Main Application Entry Point
// -----------------------------------------------------------------------------

int main() {
    // Overclock slightly and boost voltage for DSP performance safety
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    sleep_ms(10);
    set_sys_clock_khz(240000, true);
    
    stdio_init_all();
    
    // Initialize class instance and start core 1 control thread
    static Rooms rooms;
    rooms.EnableNormalisationProbe();
    multicore_launch_core1(core1_worker);
    
    // Run the main audio DSP core0 loop (blocking)
    rooms.Run();
}
