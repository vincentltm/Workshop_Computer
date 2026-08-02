#include "ComputerCard.h"
#include "pico/stdlib.h"
#include <stdlib.h>
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/adc.h"
#include "hardware/vreg.h"
#include <string.h>
#include <stdio.h>
#include "dsp_q15.h"

#define NUM_PAGES 6
#define FLASH_SAMPLE_ADDR 0x10100000
#define SAMPLES_MAGIC 0x6772616E

// --- Flash structures ---
struct FlashHeader {
  uint32_t magic;
  uint32_t count;
};
struct FlashSampleEntry {
  uint32_t offset;
  uint32_t length;
  char name[24];
};

// --- Multi-core Handshake Flags ---
#define FLAG_TRIGGER_1   (1 << 0)
#define FLAG_TRIGGER_2   (1 << 1)
#define FLAG_TRIGGER_3   (1 << 2)
#define FLAG_TRIGGER_4   (1 << 3)
#define FLAG_VEL_1(v)    (((v) & 0xFF) << 4)
#define FLAG_VEL_2(v)    (((v) & 0xFF) << 12)
#define FLAG_VEL_3(v)    (((v) & 0xFF) << 20)
#define FLAG_EXT_CLK     (1 << 28)
#define FLAG_EXT_RUN     (1 << 29)

// --- Shared Volatile State (atomic reads/writes) ---
struct VoiceParams {
  volatile int32_t pitch;   // 0..4095
  volatile int32_t decay;   // 0..4095
  volatile int32_t timbre;  // 0..4095
  volatile int32_t level;   // 0..4095
  volatile int32_t model;   // 0..15 (Engine select)
};
struct SeqParams {
  volatile int32_t steps;   // 0..16
  volatile int32_t fills;   // 0..steps
  volatile int32_t rotate;  // 0..15
};

static VoiceParams g_voice[4];
static SeqParams g_seq[4];
static volatile bool pause_core1 = false;
static volatile bool core1_is_paused = false;

static volatile int32_t g_bpm = 120;
static volatile int32_t g_swing = 0;
static volatile int32_t g_humanize = 0;
static volatile int32_t g_fills = 0;
static volatile int32_t g_chaos = 0;
static volatile int32_t g_mutation = 0;

static volatile int32_t g_delay_time = 10000;
static volatile int32_t g_delay_mix = 0;
static volatile int32_t g_reverb_mix = 0;
static volatile int32_t g_reverb_size = 16384;
static volatile int32_t g_master_filter = 2048;
static volatile int32_t g_master_drive = 0;
static volatile int32_t g_master_volume = 4095;
static volatile int32_t g_delay_feedback = 0;
static volatile uint32_t g_live_pulse_mask = 0;
static volatile bool g_trigger_occurred[4] = {false, false, false, false};

// --- Sample Lists (Categorized on boot) ---
static int32_t g_num_flash_samples = 0;
static int32_t g_slot_samples[4][32]; // Sample indices per slot
static int32_t g_slot_sample_count[4] = {0, 0, 0, 0};

// --- Q15 State Variable Filter (SVF) ---
struct SVF_Q15 {
  int32_t low, band, high;
  int32_t f, q; // Q14 coefficients
  
  void init() {
    low = band = high = 0;
    f = 1000;
    q = 16384;
  }
  
  void set(int32_t cutoff_q15, int32_t resonance_q15) {
    // Approx f coefficient: f = cutoff_q15 * 6000 >> 15 (max f ~ 6000 in Q14 = 0.36)
    f = (cutoff_q15 * 6000) >> 15;
    if (f < 10) f = 10;
    
    // resonance_q15 (0..32767) -> q (Q14 damp coefficient)
    // damp = 2 - 2 * resonance. resonance=32767 -> damp=0 (high Q). resonance=0 -> damp=2 (32768 in Q14).
    q = (32767 - resonance_q15) >> 1;
    if (q < 160) q = 160; // prevent overflow self-oscillation
  }
  
  void process(int32_t in) {
    // high = in - bp * q - lp
    high = in - ((band * q) >> 14) - low;
    band += (high * f) >> 14;
    low += (band * f) >> 14;
    
    // Saturation to prevent roll-over overflow
    high = sat_q15(high);
    band = sat_q15(band);
    low = sat_q15(low);
  }
};

// --- Q15 Pink Noise Generator ---
struct PinkNoise_Q15 {
  int32_t b0, b1, b2;
  uint32_t seed;
  
  void init() { b0 = b1 = b2 = 0; seed = 12345; }
  
  int32_t process() {
    int32_t white = RandomQ15(seed); // ±16384
    b0 = OnePoleQ15(b0, white, 32691); // 0.99765 in Q15 = 32691
    b1 = OnePoleQ15(b1, white, 31555); // 0.963 in Q15 = 31555
    b2 = OnePoleQ15(b2, white, 18677); // 0.57 in Q15 = 18677
    
    int32_t out = (b0 + b1 + b2 + (white >> 2)) >> 2;
    return sat_q15(out * 3);
  }
};

// --- Q15 Reverb Comb & Allpass (Freeverb) ---
struct FVComb_Q15 {
  int16_t *buf; // 16-bit to save 50% RAM
  int size, ptr;
  int32_t feedback, filter_state;
  
  void init(int16_t *b, int sz) {
    buf = b; size = sz; ptr = 0; filter_state = 0; feedback = 27525; // 0.84 in Q15 = 27525
    memset(buf, 0, size * sizeof(int16_t));
  }
  
  int32_t process(int32_t in, int32_t damp) {
    int32_t out = (int32_t)buf[ptr];
    filter_state = OnePoleQ15(out, filter_state, damp);
    int32_t val = in + mul_q15(filter_state, feedback);
    buf[ptr] = (int16_t)sat_q15(val);
    if (++ptr >= size) ptr = 0;
    return out;
  }
};

struct FVAllpass_Q15 {
  int16_t *buf;
  int size, ptr;
  
  void init(int16_t *b, int sz) {
    buf = b; size = sz; ptr = 0;
    memset(buf, 0, size * sizeof(int16_t));
  }
  
  int32_t process(int32_t in) {
    int32_t buf_out = (int32_t)buf[ptr];
    int32_t out = -in + buf_out;
    buf[ptr] = (int16_t)sat_q15(in + (buf_out >> 1));
    if (++ptr >= size) ptr = 0;
    return out;
  }
};

// --- Core 1 DSP Engine Buffers ---
static FVComb_Q15 rv_comb[8];
static FVAllpass_Q15 rv_ap[4];
static int16_t rv_comb_buf[8][1620];
static int16_t rv_ap_buf[4][560];

// Delay buffer (approx 0.34s at 48kHz, optimized RAM footprint)
#define DELAY_SIZE 16384
static int16_t g_delay_buf[DELAY_SIZE];
static int g_delay_ptr = 0;
static int32_t g_slewed_delay_q8 = 1000 << 8;

// Waveguide hihat buffers
#define HH_COMB_MAX 64
static int16_t hh_comb[6][HH_COMB_MAX];
static int hh_ptr = 0;
static int hh_len[6] = { 17, 23, 29, 37, 41, 47 };

// Tone/Noise Filter for Snare
static SVF_Q15 sn_filter;
static PinkNoise_Q15 pink_noise_gen;

// --- Voice Synthesis Engine ---
struct Voice {
  int slot;
  uint32_t phase, click_phase;
  int32_t env, pitch_env1, pitch_env2;
  
  // Sample Playback states
  int32_t playhead_q8;
  int32_t playhead_inc_q8;
  const int16_t *flash_ptr;
  uint32_t sample_len;
  int32_t vel; // Q15
  
  // FM Operator states
  uint32_t fm_carrier_phase, fm_modulator_phase;
  
  // Resonator block SVF
  SVF_Q15 resonant_filter;
  uint32_t seed;
  
  void init(int s) {
    slot = s; phase = click_phase = fm_carrier_phase = fm_modulator_phase = 0;
    env = pitch_env1 = pitch_env2 = 0;
    playhead_q8 = -256; playhead_inc_q8 = 0; flash_ptr = nullptr; sample_len = 0;
    vel = 32767; seed = 12345 + s;
    resonant_filter.init();
  }
  
  void trigger(int32_t velocity_q15) {
    vel = velocity_q15;
    env = 32767;
    phase = 0;
    click_phase = 0;
    
    int model = g_voice[slot].model;
    
    // Sample Playback triggering
    if (model >= 3) {
      int sample_idx_in_slot = model - 3;
      if (sample_idx_in_slot < g_slot_sample_count[slot]) {
        int global_idx = g_slot_samples[slot][sample_idx_in_slot];
        FlashSampleEntry *entries = (FlashSampleEntry *)(FLASH_SAMPLE_ADDR + 8);
        sample_len = entries[global_idx].length;
        flash_ptr = (const int16_t *)(FLASH_SAMPLE_ADDR + entries[global_idx].offset);
        playhead_q8 = 0;
        
        // Pitch mapping for samples: default is 1.0 (2048).
        // 2048 translates to 256 in Q8 step.
        playhead_inc_q8 = (g_voice[slot].pitch * 256) / 2048;
        if (playhead_inc_q8 < 20) playhead_inc_q8 = 20;
      } else {
        playhead_q8 = -256; // invalid sample
      }
    } else {
      // Synth models
      pitch_env1 = 32767;
      pitch_env2 = 32767;
      fm_carrier_phase = 0;
      fm_modulator_phase = 0;
    }
  }
  
  int32_t process() {
    int model = g_voice[slot].model;
    int32_t out = 0;
    
    int32_t decay_knob = g_voice[slot].decay;   // 0..4095
    int32_t timbre_knob = g_voice[slot].timbre; // 0..4095
    int32_t pitch_knob = g_voice[slot].pitch;   // 0..4095
    
    // --- SAMPLE PLAYBACK ---
    if (model >= 3) {
      if (playhead_q8 >= 0 && flash_ptr && (playhead_q8 >> 8) < (int)(sample_len - 1) && env > 30) {
        int idx = playhead_q8 >> 8;
        int32_t frac = playhead_q8 & 0xFF;
        
        int32_t s = (int32_t)flash_ptr[idx] + ((((int32_t)flash_ptr[idx+1] - (int32_t)flash_ptr[idx]) * frac) >> 8);
        
        // Simple Timbre dynamic lowpass filter on samples (OnePole)
        static int32_t lp_state = 0;
        int32_t lp_coeff = 1600 + (timbre_knob * 28000 >> 12); // Q15 coefficient
        lp_state = OnePoleQ15(lp_state, s, lp_coeff);
        s = lp_state;
        
        out = mul_q15(s, env);
        out = mul_q15(out, vel);
        
        playhead_q8 += playhead_inc_q8;
        
        // Decay mapping for sample length gating (0.999 to 0.99995 in Q15)
        int32_t dec = 32735 + (decay_knob * 30 >> 12);
        env = mul_q15(env, dec);
      } else {
        playhead_q8 = -256;
      }
      return out;
    }
    
    // --- SYNTHESIS MODELS ---
    if (env <= 30) return 0;
    
    if (slot == 0) { // --- KICK DRUM ENGINE ---
      int32_t dec = 32440 + (decay_knob * 310 >> 12); // up to 32750 decay in Q15
      env = mul_q15(env, dec);
      pitch_env1 = mul_q15(pitch_env1, 32170); // attack sweep decay (0.982)
      pitch_env2 = mul_q15(pitch_env2, 32670); // body sweep decay (0.997)
      
      if (model == 0) { // Synth Kick (808/909 Sweep)
        // MIDI Pitch maps raw pitch to MIDI: 32 (A0) to 60 (C3)
        int32_t base_midi_q8 = (32 << 8) + ((pitch_knob * 28) << 8 >> 12);
        int32_t pitch_bend = (pitch_env1 * 30) + (pitch_env2 * 8); // pitch bend in Q8
        int32_t pitch_midi_q8 = base_midi_q8 + (pitch_bend >> 8);
        
        phase += MidiToIncrementU32(pitch_midi_q8);
        int32_t body = SineQ15_U32(phase);
        
        // Transient Click
        click_phase += 268435456u; // 3000Hz frequency increment (3000 * 2^32 / 48000)
        int32_t click = SineQ15_U32(click_phase);
        click = mul_q15(click, pitch_env1);
        click = mul_q15(click, 3270 + (timbre_knob * 13100 >> 12));
        
        out = mul_q15(body, env) + click;
        // Saturation drive
        int32_t drive = 39300 + (timbre_knob * 65500 >> 12); // Q15 gain
        out = SoftClipQ15(mul_q15(out, drive));
      } else if (model == 1) { // Resonant Kick (Physical Skin)
        int32_t base_cutoff = 1200 + (pitch_knob * 1600 >> 12); // Q15 freq coefficient
        resonant_filter.set(base_cutoff, (1600 + (4095 - decay_knob) * 4900) >> 12);
        
        int32_t impulse = (pitch_env1 > 26214) ? (32767 - (timbre_knob * 16384 >> 12)) : 0;
        resonant_filter.process(impulse);
        out = resonant_filter.band * 9 >> 1; // 4.5x scale
        out = mul_q15(out, env);
      } else { // Industrial FM Kick
        int32_t c_base_midi = (29 << 8) + ((pitch_knob * 20) << 8 >> 12);
        int32_t mod_pitch = c_base_midi + (pitch_env1 * 25 >> 8);
        
        fm_modulator_phase += MidiToIncrementU32(c_base_midi + (12 << 8)); // 2x frequency ratio
        int32_t mod = SineQ15_U32(fm_modulator_phase);
        mod = mul_q15(mod, timbre_knob << 3); // FM index
        mod = mul_q15(mod, pitch_env1);
        
        fm_carrier_phase += MidiToIncrementU32(mod_pitch + (mod >> 4));
        out = SineQ15_U32(fm_carrier_phase);
        out = mul_q15(out, env);
        out = SoftLimitQ15(out << 1);
      }
      
    } else if (slot == 1) { // --- SNARE DRUM ENGINE ---
      int32_t dec = 32276 + (decay_knob * 480 >> 12); // decay Q15
      env = mul_q15(env, dec);
      pitch_env1 = mul_q15(pitch_env1, 32276); // 0.985
      
      int32_t noise = RandomQ15(seed);
      
      if (model == 0) { // Synth Snare (Noise SVF + Shell Tone)
        int32_t tone_midi = (50 << 8) + ((pitch_knob * 24) << 8 >> 12) + (pitch_env1 * 10 >> 8);
        phase += MidiToIncrementU32(tone_midi);
        int32_t shell = mul_q15(SineQ15_U32(phase) * 7 >> 3, pitch_env1);
        
        sn_filter.set(4000 + (timbre_knob * 12000 >> 12), 13107);
        sn_filter.process(noise);
        int32_t filt_noise = sn_filter.band * 5 >> 1;
        filt_noise = mul_q15(filt_noise, env);
        
        out = shell + filt_noise;
      } else if (model == 1) { // Jazz Brush Snare (Resonant Noise Friction)
        sn_filter.set(3000 + (pitch_knob * 8000 >> 12), 3276 + (timbre_knob * 16384 >> 12));
        int32_t friction = pink_noise_gen.process();
        friction = mul_q15(friction, env);
        sn_filter.process(friction);
        out = sn_filter.high * 3;
      } else { // Chiptune Snare (NES/SID 8-Bit decimated noise)
        static int32_t dec_hold = 0;
        static int dec_cnt = 0;
        int target_dec = 2 + ((4095 - pitch_knob) * 64 >> 12);
        if (++dec_cnt >= target_dec) {
          dec_cnt = 0;
          dec_hold = noise;
        }
        
        // Timbre controls bit-depth reduction (crusher)
        int32_t steps = 2 + (timbre_knob * 14 >> 12);
        int32_t crushed = (dec_hold / (32768 / steps)) * (32768 / steps);
        
        out = mul_q15(crushed * 7 >> 3, env);
      }
      
    } else if (slot == 2) { // --- HIHAT / METAL ENGINE ---
      int32_t dec = 31129 + (decay_knob * 1630 >> 12); // decay Q15
      env = mul_q15(env, dec);
      
      if (model == 0) { // Synth Hihat (808 Multi-oscillator mix)
        int32_t metal = 0;
        static uint32_t phases[6] = {0};
        int32_t freqs[6] = { 205, 369, 562, 716, 830, 1110 };
        int32_t pitch_scale = 16384 + (pitch_knob * 81920 >> 12); // Q15 freq scale (0.5..3.0)
        for (int i = 0; i < 6; i++) {
          // Increment phase using integer multiplier
          phases[i] += (uint32_t)(((int64_t)freqs[i] * pitch_scale * 89478) >> 15);
          metal += (phases[i] < 0x80000000u) ? 5461 : -5461; // ±0.166 in Q15 = ±5461
        }
        sn_filter.set(16000 + (timbre_knob * 16000 >> 12), 6553);
        sn_filter.process(metal);
        out = mul_q15(sn_filter.high * 5 >> 1, env);
      } else if (model == 1) { // Waveguide Chime (Comb Feedback Clang)
        int32_t exciter = mul_q15(RandomQ15(seed), env);
        
        // Read parallel comb lines
        int32_t sum = 0;
        int32_t fb = 24576 + (timbre_knob * 7500 >> 12); // Q15 feedback (0.75..0.98)
        int delay_offs = (4095 - pitch_knob) * 30 >> 12;
        for (int i = 0; i < 6; i++) {
          int cur_len = hh_len[i] + delay_offs;
          if (cur_len < 4) cur_len = 4;
          if (cur_len > HH_COMB_MAX - 1) cur_len = HH_COMB_MAX - 1;
          
          int32_t delayed = (int32_t)hh_comb[i][(hh_ptr + HH_COMB_MAX - cur_len) & (HH_COMB_MAX - 1)];
          hh_comb[i][hh_ptr] = (int16_t)sat_q15(exciter + mul_q15(delayed, fb));
          sum += delayed;
        }
        hh_ptr = (hh_ptr + 1) & (HH_COMB_MAX - 1);
        out = sum * 5461 >> 15; // sum * 0.166
      } else { // Shaker (Noise bandpass)
        int32_t w_noise = RandomQ15(seed);
        int32_t bp_freq = 8000 + (pitch_knob * 22000 >> 12);
        sn_filter.set(bp_freq, 13107 - (timbre_knob * 11468 >> 12));
        sn_filter.process(w_noise);
        out = mul_q15(sn_filter.band * 7 >> 1, env);
      }
      
    } else { // --- TOM / PERCUSSION ENGINE ---
      int32_t dec = 31784 + (decay_knob * 978 >> 12);
      env = mul_q15(env, dec);
      pitch_env1 = mul_q15(pitch_env1, 32276); // 0.985
      
      if (model == 0) { // FM Tom
        int32_t c_base_midi = (40 << 8) + ((pitch_knob * 36) << 8 >> 12);
        int32_t mod_pitch = c_base_midi + (pitch_env1 * 10 >> 8);
        
        fm_modulator_phase += MidiToIncrementU32(c_base_midi + (7 << 8)); // 1.5x frequency ratio
        int32_t mod = SineQ15_U32(fm_modulator_phase);
        mod = mul_q15(mod, timbre_knob << 3);
        mod = mul_q15(mod, pitch_env1);
        
        fm_carrier_phase += MidiToIncrementU32(mod_pitch + (mod >> 4));
        out = mul_q15(SineQ15_U32(fm_carrier_phase), env);
      } else if (model == 1) { // Resonant Wood Block / Claves
        int32_t base_freq = 4000 + (pitch_knob * 12000 >> 12);
        resonant_filter.set(base_freq, 655 + ((4095 - timbre_knob) * 4915 >> 12));
        int32_t impulse = (pitch_env1 > 26214) ? 32767 : 0;
        resonant_filter.process(impulse);
        out = mul_q15(resonant_filter.band * 6, env);
      } else { // Classic cowbell
        int32_t pitch_val = 540 * (16384 + (pitch_knob * 49152 >> 12)) >> 15;
        static uint32_t phase1 = 0, phase2 = 0;
        phase1 += (uint32_t)((int64_t)pitch_val * 89478);
        phase2 += (uint32_t)((int64_t)pitch_val * 132427);
        int32_t sum = ((phase1 < 0x80000000u) ? 16384 : -16384) + ((phase2 < 0x80000000u) ? 16384 : -16384);
        
        sn_filter.set(5000 + (timbre_knob * 15000 >> 12), 6553);
        sn_filter.process(sum);
        out = mul_q15(sn_filter.band * 7 >> 1, env);
      }
    }
    
    return mul_q15(out, vel);
  }
};

static Voice voices[4];

// --- Internal Sequencer Engine Core ---
static uint16_t turing_register = 0xACE1; // 16-bit master seed
static int32_t seq_step_index = 0;
static int32_t seq_accum = 0;
static int32_t trigger_pulses[4] = {0, 0, 0, 0}; // pulse width timers

static inline bool calculate_euclidean(int step, int steps, int fills, int rotate) {
  if (steps <= 0) return false;
  if (fills <= 0) return false;
  if (fills > steps) fills = steps;
  
  int adjusted_step = (step + rotate) % steps;
  int num = adjusted_step * fills;
  int rem = num % steps;
  return (rem < fills);
}

void __not_in_flash_func(core1_worker)() {
  for (int i = 0; i < 4; i++) {
    voices[i].init(i);
  }
  sn_filter.init();
  pink_noise_gen.init();
  
  // Reverb setup
  int rv_cb_sizes[8] = { 1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617 };
  int rv_ap_sizes[4] = { 225, 341, 441, 556 };
  for (int i = 0; i < 8; i++) rv_comb[i].init(rv_comb_buf[i], rv_cb_sizes[i]);
  for (int i = 0; i < 4; i++) rv_ap[i].init(rv_ap_buf[i], rv_ap_sizes[i]);
  
  memset(g_delay_buf, 0, sizeof(g_delay_buf));
  
  while (1) {
    if (pause_core1) {
      core1_is_paused = true;
      while (pause_core1) {
        tight_loop_contents();
      }
      core1_is_paused = false;
    }
    // Block waiting for Core 0 UI/Trigger flags
    uint32_t flags = multicore_fifo_pop_blocking();
    
    // Check external inputs trigger triggers
    if (flags & FLAG_TRIGGER_1) {
      int32_t vel = ((flags >> 4) & 0xFF) * 128; // scale 0..255 to 0..32640 in Q15
      voices[0].trigger(vel);
      trigger_pulses[0] = 480; // 10ms gate pulse
    }
    if (flags & FLAG_TRIGGER_2) {
      int32_t vel = ((flags >> 12) & 0xFF) * 128;
      voices[1].trigger(vel);
      trigger_pulses[1] = 480;
    }
    if (flags & FLAG_TRIGGER_3) {
      int32_t vel = ((flags >> 20) & 0xFF) * 128;
      voices[2].trigger(vel);
      trigger_pulses[2] = 480;
    }
    if (flags & FLAG_TRIGGER_4) {
      if (flags & (1 << 30)) {
        voices[3].trigger(32767);
        trigger_pulses[3] = 480;
      }
    }
    
    // --- INTERNAL SEQUENCER STEP CALCULATIONS ---
    bool is_ext_clock = (flags & FLAG_EXT_CLK) != 0;
    bool is_ext_run = (flags & FLAG_EXT_RUN) != 0;
    bool sequencer_trigger = false;
    
    if (is_ext_clock) {
      if (flags & (1 << 31)) {
        sequencer_trigger = true;
      }
    } else {
      // 16th notes period: samples = 60 * 48000 / (bpm * 4) = 720000 / bpm
      int32_t step_samples = 720000 / g_bpm;
      seq_accum += 1;
      if (seq_accum >= step_samples) {
        seq_accum = 0;
        sequencer_trigger = true;
      }
    }
    
    if (sequencer_trigger && (!is_ext_clock || is_ext_run)) {
      // Shift Turing Register
      uint16_t last_bit = turing_register & 1;
      turing_register >>= 1;
      
      // Evolve register with mutation probability
      int32_t roll = RandomQ15(voices[0].seed) & 0x7FFF; // 0..32767
      if (roll < g_mutation * 8) {
        last_bit ^= 1; // Flip
      }
      turing_register |= (last_bit << 15);
      
      // Determine Euclidean triggers for each track
      for (int i = 0; i < 4; i++) {
        int steps = g_seq[i].steps;
        if (steps > 0) {
          int fills = g_seq[i].fills;
          
          if (g_fills > 50) {
            fills += (g_fills * (steps - fills)) >> 12;
          }
          
          bool hit = calculate_euclidean(seq_step_index, steps, fills, g_seq[i].rotate);
          
          int32_t c_roll = RandomQ15(voices[i].seed) & 0x7FFF;
          if (c_roll < g_chaos * 8) {
            bool turing_bit = (turing_register & (1 << i)) != 0;
            hit ^= turing_bit;
          }
          
          if (hit) {
            voices[i].trigger(32767);
            trigger_pulses[i] = 480;
          }
        }
      }
      
      seq_step_index = (seq_step_index + 1) % 16;
    }
    
    // Decrement trigger output pulse timers
    for (int i = 0; i < 4; i++) {
      if (trigger_pulses[i] > 0) trigger_pulses[i]--;
    }
    
    // --- SYNTHESIZE VOICES ---
    int32_t s1 = voices[0].process();
    int32_t s2 = voices[1].process();
    int32_t s3 = voices[2].process();
    int32_t s4 = voices[3].process();
    
    // Apply Mixer volume bias
    // Voice 1 & 2 bias: Page 4 Main (0..4095)
    int32_t v1_v2_bias_q15 = g_voice[0].level << 3; // 0..32760
    int32_t v1_vol = 32768 - v1_v2_bias_q15;
    int32_t v2_vol = v1_v2_bias_q15;
    
    // Voice 3 & 4 bias: Page 5 Main
    int32_t v3_v4_bias_q15 = g_voice[2].level << 3;
    int32_t v3_vol = 32768 - v3_v4_bias_q15;
    int32_t v4_vol = v3_v4_bias_q15;
    
    int32_t mix = mul_q15(s1, v1_vol) + mul_q15(s2, v2_vol) + mul_q15(s3, v3_vol) + mul_q15(s4, v4_vol);
    mix = sat_q15(mix);
    
    // --- TAPE DELAY ---
    int32_t delay_target_q8 = (g_delay_time * DELAY_SIZE) >> 4; // Q8 delay target
    if (delay_target_q8 < (200 << 8)) delay_target_q8 = 200 << 8;
    if (delay_target_q8 > ((DELAY_SIZE - 10) << 8)) delay_target_q8 = (DELAY_SIZE - 10) << 8;
    g_slewed_delay_q8 += (delay_target_q8 - g_slewed_delay_q8) >> 10;
    
    int32_t d_idx = g_slewed_delay_q8 >> 8;
    int32_t d_frac = g_slewed_delay_q8 & 0xFF;
    int32_t d_read_ptr = (g_delay_ptr + DELAY_SIZE - d_idx) & (DELAY_SIZE - 1);
    
    int32_t a_val = (int32_t)g_delay_buf[d_read_ptr];
    int32_t b_val = (int32_t)g_delay_buf[(d_read_ptr + 1) & (DELAY_SIZE - 1)];
    int32_t d_out = a_val + (((b_val - a_val) * d_frac) >> 8);
    
    // Delay lowpass feedback damp
    static int32_t d_lp = 0;
    d_lp = OnePoleQ15(d_lp, d_out, 11468); // ~0.35 coeff
    
    int32_t d_fb = g_delay_feedback << 3; // feedback gain Q15
    int32_t fb_sig = mul_q15(d_lp, d_fb);
    g_delay_buf[g_delay_ptr] = (int16_t)sat_q15(mix + fb_sig);
    g_delay_ptr = (g_delay_ptr + 1) & (DELAY_SIZE - 1);
    
    int32_t d_mix_coeff = g_delay_mix << 3;
    int32_t wet_delay = mix + mul_q15(d_out, d_mix_coeff);
    
    // --- ROOM REVERB (Freeverb Schroeder-Moorer) ---
    int32_t wet_rv = wet_delay;
    int32_t rv_mix_pct = g_reverb_mix << 3; // Q15
    if (rv_mix_pct > 300) {
      int32_t rv_damp = 11468;
      int32_t rv_room = 19660 + (g_reverb_size * 11468 >> 12); // Q15 decay (0.6..0.95)
      
      // Parallel Comb filters
      int32_t cb_sum = 0;
      for (int i = 0; i < 8; i++) {
        // Temporarily adjust feedback coefficient
        int32_t old_fb = rv_comb[i].feedback;
        rv_comb[i].feedback = rv_room;
        cb_sum += rv_comb[i].process(wet_delay, rv_damp);
        rv_comb[i].feedback = old_fb;
      }
      
      // Series Allpass filters
      int32_t ap_sum = cb_sum >> 3; // scale comb outputs
      for (int i = 0; i < 4; i++) {
        ap_sum = rv_ap[i].process(ap_sum);
      }
      
      wet_rv = wet_delay + mul_q15(ap_sum * 4, rv_mix_pct);
    }
    
    int32_t out = sat_q15(wet_rv);
    
    // --- MASTER COMPRESSOR & SATURATION ---
    // Diode Compressor envelope tracker (OnePole)
    static int32_t comp_env = 0;
    int32_t rect = abs_q15(out);
    int32_t comp_coeff = (rect > comp_env) ? 1638 : 32; // fast attack (0.05), slow release (0.001)
    comp_env = OnePoleQ15(comp_env, rect, comp_coeff);
    
    int32_t comp_thresh = 4915; // 0.15 in Q15 = 4915
    int32_t gain = 32767;
    if (comp_env > comp_thresh) {
      // 4:1 compression ratio
      gain = comp_thresh + ((comp_env - comp_thresh) >> 2);
      gain = (gain << 15) / comp_env; // Q15 gain multiplier
      if (gain > 32767) gain = 32767;
    }
    out = mul_q15(out, gain);
    
    // DJ Filter SVF
    static SVF_Q15 master_svf;
    static bool svf_init = false;
    if (!svf_init) { master_svf.init(); svf_init = true; }
    
    int32_t dj = g_master_filter << 3; // Q15
    int32_t final_filtered = out;
    if (dj < 15000) { // Lowpass sweep
      int32_t lp_cutoff = 1000 + (dj * 20000 >> 15);
      master_svf.set(lp_cutoff, 22937); // damp ~0.7
      master_svf.process(out);
      final_filtered = master_svf.low;
    } else if (dj > 17700) { // Highpass sweep
      int32_t hp_cutoff = 800 + ((dj - 17700) * 16000 >> 15);
      master_svf.set(hp_cutoff, 22937);
      master_svf.process(out);
      final_filtered = master_svf.high;
    }
    
    // Saturation & Master Volume
    int32_t drive_gain = 32768 + (g_master_drive * 81920 >> 12); // Q15 drive gain (1.0..3.5)
    int32_t final_out = SoftClipQ15(mul_q15(final_filtered, drive_gain));
    
    // Master Volume scale
    int32_t vol_coeff = g_master_volume << 3; // Q15
    final_out = mul_q15(final_out, vol_coeff);
    
    // Scale output to 16-bit signed
    int16_t outL = (int16_t)final_out;
    int16_t outR = outL;
    
    // Push output samples to Core 0 FIFO
    multicore_fifo_push_blocking((uint32_t)outL);
    multicore_fifo_push_blocking((uint32_t)outR);
    
    // Push trigger pulse states (packed) for external jacks
    uint32_t pulse_mask = 0;
    for (int i = 0; i < 4; i++) {
      if (trigger_pulses[i] > 0) {
        pulse_mask |= (1 << i);
      }
    }
    multicore_fifo_push_blocking(pulse_mask);
  }
}

// --- Card Class Class ---
class DrumsCard : public ComputerCard {
  int currentPage = 0;
  Switch lastSwitch = Switch::Middle;
  
  // Debouncing / Flick Gestures
  uint32_t sw_down_timer = 0;
  bool sw_down_handled = false;
  bool in_preset_menu = false;
  int preset_selected_slot = 0;
  
  // Knob locks
  int32_t lock_ref_main = 0, lock_ref_x = 0, lock_ref_y = 0;
  bool lock_main = true, lock_x = true, lock_y = true;
  
  // Led Feedback override
  uint32_t feedback_timer = 0;
  uint8_t feedback_val = 0;
  bool feedback_binary = false;
  uint32_t page_blink_timer_ = 0;
  uint16_t led_decay[6] = {0, 0, 0, 0, 0, 0};
  
  // Trigger Input peak trackers
  struct PeakDetector {
    int32_t last_val = 0;
    int32_t max_peak = 0;
    int32_t delay_timer = 0;
    int32_t holdoff = 0;
    
    bool process_audio(int32_t sample, uint8_t &velocity) {
      if (holdoff > 0) { holdoff--; return false; }
      int32_t rect = abs(sample);
      if (rect > 200 && rect > last_val + 100 && delay_timer == 0) {
        delay_timer = 48; // track peak for 1ms
        max_peak = rect;
        return false;
      }
      if (delay_timer > 0) {
        if (rect > max_peak) max_peak = rect;
        delay_timer--;
        if (delay_timer == 0) {
          velocity = (max_peak > 2047) ? 255 : (uint8_t)((max_peak * 255) / 2048);
          holdoff = 480; // 10ms holdoff
          return true;
        }
      }
      last_val = rect;
      return false;
    }
    
    bool process_cv(int32_t sample, uint8_t &velocity) {
      if (holdoff > 0) { holdoff--; return false; }
      int32_t diff = sample - last_val;
      if (diff > 250 && delay_timer == 0) {
        delay_timer = 48;
        max_peak = sample;
        return false;
      }
      if (delay_timer > 0) {
        if (sample > max_peak) max_peak = sample;
        delay_timer--;
        if (delay_timer == 0) {
          velocity = (max_peak > 2047) ? 255 : (uint8_t)((max_peak * 255) / 2048);
          holdoff = 480;
          return true;
        }
      }
      last_val = sample;
      return false;
    }
  };
  
  PeakDetector trigger_detectors[4];

public:
  DrumsCard() {
    // Apply default parameters
    for (int i = 0; i < 4; i++) {
      g_voice[i].pitch = 2048;
      g_voice[i].decay = 1024;
      g_voice[i].timbre = 1000;
      g_voice[i].level = 2048; // Mixer bias at center
      g_voice[i].model = 0;   // Default Synth engine
      
      g_seq[i].steps = 0;     // Sequencer off by default
      g_seq[i].fills = 0;
      g_seq[i].rotate = 0;
    }
  }
  
  void init_flash_samples() {
    FlashHeader *header = (FlashHeader *)FLASH_SAMPLE_ADDR;
    if (header->magic == SAMPLES_MAGIC) {
      g_num_flash_samples = header->count;
      if (g_num_flash_samples > 28) g_num_flash_samples = 28;
      
      FlashSampleEntry *entries = (FlashSampleEntry *)(FLASH_SAMPLE_ADDR + 8);
      for (int i = 0; i < g_num_flash_samples; i++) {
        char name_lower[25];
        memset(name_lower, 0, sizeof(name_lower));
        for (int j = 0; j < 24; j++) {
          char c = entries[i].name[j];
          if (c == '\0') break;
          name_lower[j] = (c >= 'A' && c <= 'Z') ? (c + 32) : c;
        }
        
        // Dynamically classify into Slot arrays
        if (strstr(name_lower, "kick") || strstr(name_lower, "k_")) {
          if (g_slot_sample_count[0] < 12) {
            g_slot_samples[0][g_slot_sample_count[0]++] = i;
          }
        } else if (strstr(name_lower, "snare") || strstr(name_lower, "s_")) {
          if (g_slot_sample_count[1] < 12) {
            g_slot_samples[1][g_slot_sample_count[1]++] = i;
          }
        } else if (strstr(name_lower, "hat") || strstr(name_lower, "hihat") || strstr(name_lower, "h_")) {
          if (g_slot_sample_count[2] < 12) {
            g_slot_samples[2][g_slot_sample_count[2]++] = i;
          }
        } else {
          // Tom / Perc / cowbell
          if (g_slot_sample_count[3] < 12) {
            g_slot_samples[3][g_slot_sample_count[3]++] = i;
          }
        }
      }
    }
  }
  
  void engage_locks() {
    lock_main = lock_x = lock_y = true;
    lock_ref_main = KnobVal(Knob::Main);
    lock_ref_x = KnobVal(Knob::X);
    lock_ref_y = KnobVal(Knob::Y);
  }
  
  void check_knob_pickup(int32_t raw_main, int32_t raw_x, int32_t raw_y,
                         int32_t &out_main, int32_t &out_x, int32_t &out_y) {
    if (lock_main) {
      if (abs(raw_main - lock_ref_main) > 150) lock_main = false;
    }
    if (lock_x) {
      if (abs(raw_x - lock_ref_x) > 150) lock_x = false;
    }
    if (lock_y) {
      if (abs(raw_y - lock_ref_y) > 150) lock_y = false;
    }
    out_main = lock_main ? lock_ref_main : raw_main;
    out_x = lock_x ? lock_ref_x : raw_x;
    out_y = lock_y ? lock_ref_y : raw_y;
  }
  
  void trigger_feedback_binary(uint8_t val) {
    feedback_timer = 1200; // 1.2 seconds in millisecond loops
    feedback_val = val;
    feedback_binary = true;
  }
  
  void trigger_feedback_bar(uint8_t val) {
    feedback_timer = 1200;
    feedback_val = val;
    feedback_binary = false;
  }
  
  void render_feedback_leds() {
    if (feedback_binary) {
      for (int i = 0; i < 6; i++) {
        LedOn(i, (feedback_val & (1 << i)) != 0);
      }
    } else {
      int full_leds = (feedback_val * 6) / 255;
      for (int i = 0; i < 6; i++) {
        LedOn(i, i < full_leds);
      }
    }
  }

  void save_preset(int slot) {
    uint32_t flash_offset = 0xF0000 + slot * 4096;
    
    // Prepare settings array
    uint32_t save_buf[64];
    memset(save_buf, 0, sizeof(save_buf));
    save_buf[0] = 0xAA551122; // Magic
    
    int idx = 1;
    for (int i = 0; i < 4; i++) {
      save_buf[idx++] = g_voice[i].pitch;
      save_buf[idx++] = g_voice[i].decay;
      save_buf[idx++] = g_voice[i].timbre;
      save_buf[idx++] = g_voice[i].level;
      save_buf[idx++] = g_voice[i].model;
      
      save_buf[idx++] = g_seq[i].steps;
      save_buf[idx++] = g_seq[i].fills;
      save_buf[idx++] = g_seq[i].rotate;
    }
    save_buf[idx++] = g_bpm;
    save_buf[idx++] = g_swing;
    
    pause_core1 = true;
    while (!core1_is_paused) {
      if (multicore_fifo_rvalid()) {
        multicore_fifo_pop_blocking();
      }
      tight_loop_contents();
    }
    
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(flash_offset, 4096);
    flash_range_program(flash_offset, (const uint8_t *)save_buf, sizeof(save_buf));
    restore_interrupts(ints);
    
    pause_core1 = false;
  }
  
  void load_preset(int slot) {
    uint32_t flash_offset = 0xF0000 + slot * 4096;
    const uint32_t *flash_ptr = (const uint32_t *)(XIP_BASE + flash_offset);
    if (flash_ptr[0] == 0xAA551122) {
      int idx = 1;
      for (int i = 0; i < 4; i++) {
        g_voice[i].pitch = flash_ptr[idx++];
        g_voice[i].decay = flash_ptr[idx++];
        g_voice[i].timbre = flash_ptr[idx++];
        g_voice[i].level = flash_ptr[idx++];
        
        int32_t loaded_model = flash_ptr[idx++];
        if (loaded_model < 0) loaded_model = 0;
        int max_models = 3 + g_slot_sample_count[i];
        if (loaded_model >= max_models) loaded_model = max_models - 1;
        g_voice[i].model = loaded_model;
        
        g_seq[i].steps = flash_ptr[idx++];
        g_seq[i].fills = flash_ptr[idx++];
        g_seq[i].rotate = flash_ptr[idx++];
      }
      g_bpm = flash_ptr[idx++];
      g_swing = flash_ptr[idx++];
    }
  }

  void BackgroundLoop() override {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    static uint32_t last_ui_time = 0;
    if (now_ms - last_ui_time < 1) {
      return; // only run once per millisecond
    }
    last_ui_time = now_ms;
    
    if (page_blink_timer_ > 0) {
      page_blink_timer_--;
    }
    
    tick_ui_once(now_ms);
  }

  void tick_ui_once(uint32_t now_ms) {
    // --- READ PHYSICAL SWITCH POSITION ---
    Switch sw = SwitchVal();
    
    // --- GESTURE DETECTORS ---
    if (sw == Switch::Down) {
      sw_down_timer++;
      
      // Preset menu hold (1.5 seconds = 1500 ms)
      if (sw_down_timer > 1500 && !sw_down_handled) {
        in_preset_menu = true;
        sw_down_handled = true;
      }
      
      if (in_preset_menu) {
        // Scroll through slot 1..6 on LEDs
        int select = (KnobVal(Knob::Main) * 6) >> 12;
        if (select > 5) select = 5;
        preset_selected_slot = select;
        
        // Show flashing slot LED
        for (int i = 0; i < 6; i++) {
          if (i == preset_selected_slot) {
            bool flash = (now_ms % 300) < 150; // 3.3Hz flash
            LedOn(i, flash);
          } else {
            LedOff(i);
          }
        }
      }
    } else {
      if (sw_down_timer > 0) { // released after being pressed (analog filter provides hardware debouncing)
        if (in_preset_menu) {
          // Determine save vs load based on X knob: left half = Load, right half = Save
          if (KnobVal(Knob::X) < 2048) {
            load_preset(preset_selected_slot);
          } else {
            save_preset(preset_selected_slot);
          }
          in_preset_menu = false;
          // Page change cycle
          int num_pages = 6;
          int old_page = currentPage;
          if (sw_down_timer < 350) { // Tap (<350ms): cycle forward
            currentPage = (currentPage + 1) % num_pages;
          } else { // Hold: cycle backwards
            currentPage = (currentPage + num_pages - 1) % num_pages;
          }
          if (currentPage != old_page) {
            page_blink_timer_ = 250;
          }
          engage_locks();
        }
      }
      
      sw_down_timer = 0;
      sw_down_handled = false;
      lastSwitch = sw;
    }
    
    if (in_preset_menu) return; // Keep preset display active
    
    // Reset page limits when switch changes mode
    static Switch last_sw_mode = Switch::Middle;
    if (sw != Switch::Down && sw != last_sw_mode) {
      page_blink_timer_ = 250;
      engage_locks();
      last_sw_mode = sw;
    }
    
    // --- KNOB VALUE LOCK/PICKUP ---
    int32_t val_main, val_x, val_y;
    check_knob_pickup(KnobVal(Knob::Main), KnobVal(Knob::X), KnobVal(Knob::Y),
                      val_main, val_x, val_y);
                      
    // --- UPDATE STATE VARIABLES FROM INTERFACE ---
    if (sw == Switch::Up) { // ================= SOUND EDIT MENU =================
      if (currentPage < 4) { // Page 0..3: Sound Edit Voice 1..4
        int ch = currentPage;
        
        // Pitch
        if (!lock_main && g_voice[ch].pitch != val_main) {
          g_voice[ch].pitch = val_main;
          trigger_feedback_bar(val_main >> 4);
        }
        // Decay
        if (!lock_x && g_voice[ch].decay != val_x) {
          g_voice[ch].decay = val_x;
          trigger_feedback_bar(val_x >> 4);
        }
        // Timbre
        if (!lock_y && g_voice[ch].timbre != val_y) {
          g_voice[ch].timbre = val_y;
          trigger_feedback_bar(val_y >> 4);
        }
      } else if (currentPage == 4) { // Page 4: Mixer V1 & V2
        // Main: V1/V2 volume mix bias
        if (!lock_main && g_voice[0].level != val_main) {
          g_voice[0].level = val_main;
          trigger_feedback_bar(val_main >> 4);
        }
        
        // X: Voice 1 Model Select (3 synth engines + user samples count)
        int v1_models = 3 + g_slot_sample_count[0];
        int v1_select = (val_x * v1_models) >> 12;
        if (v1_select >= v1_models) v1_select = v1_models - 1;
        if (!lock_x && g_voice[0].model != v1_select) {
          g_voice[0].model = v1_select;
          trigger_feedback_binary(v1_select);
        }
        
        // Y: Voice 2 Model Select
        int v2_models = 3 + g_slot_sample_count[1];
        int v2_select = (val_y * v2_models) >> 12;
        if (v2_select >= v2_models) v2_select = v2_models - 1;
        if (!lock_y && g_voice[1].model != v2_select) {
          g_voice[1].model = v2_select;
          trigger_feedback_binary(v2_select);
        }
      } else if (currentPage == 5) { // Page 5: Mixer V3 & V4
        // Main: V3/V4 volume mix bias
        if (!lock_main && g_voice[2].level != val_main) {
          g_voice[2].level = val_main;
          trigger_feedback_bar(val_main >> 4);
        }
        
        // X: Voice 3 Model Select
        int v3_models = 3 + g_slot_sample_count[2];
        int v3_select = (val_x * v3_models) >> 12;
        if (v3_select >= v3_models) v3_select = v3_models - 1;
        if (!lock_x && g_voice[2].model != v3_select) {
          g_voice[2].model = v3_select;
          trigger_feedback_binary(v3_select);
        }
        
        // Y: Voice 4 Model Select
        int v4_models = 3 + g_slot_sample_count[3];
        int v4_select = (val_y * v4_models) >> 12;
        if (v4_select >= v4_models) v4_select = v4_models - 1;
        if (!lock_y && g_voice[3].model != v4_select) {
          g_voice[3].model = v4_select;
          trigger_feedback_binary(v4_select);
        }
      }
      
    } else if (sw == Switch::Middle) { // ================= SEQUENCER & GLOBALS MENU =================
      if (currentPage < 4) { // Page 0..3: Channel Euclidean Sequencer
        int ch = currentPage;
        
        // Steps
        int steps = (val_main * 17) >> 12; // 0..16
        if (steps > 16) steps = 16;
        if (!lock_main && g_seq[ch].steps != steps) {
          g_seq[ch].steps = steps;
          trigger_feedback_binary(steps);
        }
        // Fills
        int max_fills = steps > 0 ? steps : 16;
        int fills = (val_x * (max_fills + 1)) >> 12;
        if (fills > max_fills) fills = max_fills;
        if (!lock_x && g_seq[ch].fills != fills) {
          g_seq[ch].fills = fills;
          trigger_feedback_binary(fills);
        }
        // Rotate
        int rotate = (val_y * 16) >> 12;
        if (!lock_y && g_seq[ch].rotate != rotate) {
          g_seq[ch].rotate = rotate;
          trigger_feedback_binary(rotate);
        }
      } else if (currentPage == 4) { // Page 4: Master Filter & FX
        // Main: Master DJ Filter
        if (!lock_main && g_master_filter != val_main) {
          g_master_filter = val_main;
          trigger_feedback_bar(val_main >> 4);
        }
        // X: Delay Mix & Feedback
        if (!lock_x && g_delay_mix != val_x) {
          g_delay_mix = val_x;
          g_delay_feedback = val_x;
          trigger_feedback_bar(val_x >> 4);
        }
        // Y: Reverb Mix & Size
        if (!lock_y && g_reverb_mix != val_y) {
          g_reverb_mix = val_y;
          g_reverb_size = val_y;
          trigger_feedback_bar(val_y >> 4);
        }
      } else if (currentPage == 5) { // Page 5: Global Clock & Chaos
        // Main: BPM
        int bpm_val = 40 + ((val_main * 200) >> 12); // 40..240 BPM
        if (!lock_main && g_bpm != bpm_val) {
          g_bpm = bpm_val;
          trigger_feedback_bar(val_main >> 4);
        }
        // X: Swing & Fills Macro
        if (!lock_x && g_swing != val_x / 80) {
          g_swing = val_x / 80;
          g_fills = (val_x > 2500) ? ((val_x - 2500) * 5) >> 1 : 0;
          trigger_feedback_bar(val_x >> 4);
        }
        // Y: Turing Mutation & Chaos
        if (!lock_y && g_mutation != val_y) {
          g_mutation = val_y;
          g_chaos = val_y;
          trigger_feedback_bar(val_y >> 4);
        }
      }
    }
    
    // Update trigger LED decay envelopes
    uint32_t pulses = g_live_pulse_mask;
    for (int i = 0; i < 4; i++) {
      if (g_trigger_occurred[i]) {
        led_decay[i] = 4095;
        g_trigger_occurred[i] = false;
      } else {
        led_decay[i] = (led_decay[i] * 240) >> 8; // decay to baseline (0.9375 coeff)
      }
    }
    // LED 4: Clock tick trigger decay
    if (pulses & 0xF) {
      led_decay[4] = 4095;
    } else {
      led_decay[4] = (led_decay[4] * 240) >> 8;
    }
    led_decay[5] = (led_decay[5] * 240) >> 8;

    // --- LED DISPLAY CONTROLLER ---
    if (page_blink_timer_ > 0) {
      // Snappy double-blink page indicator (Clockworks-style)
      uint32_t t = 250 - page_blink_timer_; // 0..250ms
      bool flash_on = (t < 50) || (t >= 100 && t < 150);
      for (int i = 0; i < 6; i++) {
        LedBrightness(i, flash_on ? 4095 : 0);
      }
    } else if (feedback_timer > 0) {
      // Temporary parameter feedback (bar or binary)
      feedback_timer--;
      if (feedback_binary) {
        for (int i = 0; i < 6; i++) {
          LedBrightness(i, (feedback_val & (1 << i)) ? 4095 : 0);
        }
      } else {
        int full_leds = (feedback_val * 6) / 255;
        for (int i = 0; i < 6; i++) {
          LedBrightness(i, (i < full_leds) ? 4095 : 0);
        }
      }
    } else if (in_preset_menu) {
      // Flashing preset slot indicator
      for (int i = 0; i < 6; i++) {
        if (i == preset_selected_slot) {
          bool flash = (now_ms % 300) < 150;
          LedBrightness(i, flash ? 4095 : 0);
        } else {
          LedBrightness(i, 0);
        }
      }
    } else {
      // Live Performance & Page Indicator (Clockworks-style combined)
      Switch sw_mode = SwitchVal();
      
      for (int i = 0; i < 6; i++) {
        uint16_t led_val = led_decay[i];
        
        if (i < 4) {
          // Show active page with a distinct solid glow, other pages stay dim
          if (currentPage == i) {
            if (led_val < 1500) led_val = 1500; // active page baseline glow
          } else {
            if (led_val < 150) led_val = 150;   // non-active baseline glow
          }
        } else if (i == 4) {
          // LED 4: Glow if active page, flash on trigger
          if (currentPage == 4) {
            if (led_val < 1500) led_val = 1500;
          } else {
            if (led_val < 150) led_val = 150;
          }
        } else { // i == 5
          // LED 5: Glow if active page, show mode (UP = Sound Edit, MIDDLE = Sequencer)
          if (currentPage == 5) {
            if (led_val < 1500) led_val = 1500;
          } else {
            uint16_t mode_glow = (sw_mode == Switch::Up) ? 400 : 0;
            if (led_val < mode_glow) led_val = mode_glow;
          }
        }
        
        LedBrightness(i, led_val);
      }
    }
  }

  void __not_in_flash_func(ProcessSample)() override {
    // --- VELOCITY PEAK & ATTACK DETECTION ---
    uint8_t vel1 = 255, vel2 = 255, vel3 = 255, vel4 = 255;
    bool trig1 = trigger_detectors[0].process_audio(AudioIn1(), vel1);
    bool trig2 = trigger_detectors[1].process_audio(AudioIn2(), vel2);
    bool trig3 = trigger_detectors[2].process_cv(CVIn1(), vel3);
    bool trig4 = trigger_detectors[3].process_cv(CVIn2(), vel4);
    
    // Check external Clock (Pulse In 1)
    bool ext_clk_rising = PulseIn1RisingEdge();
    bool run_gate = Disconnected(Input::Pulse2) || PulseIn2();
    
    // Pack triggers/velocity flags and set volatile trigger markers for LPF decay LEDs
    uint32_t flags = 0;
    if (trig1) { flags |= FLAG_TRIGGER_1 | FLAG_VEL_1(vel1); g_trigger_occurred[0] = true; }
    if (trig2) { flags |= FLAG_TRIGGER_2 | FLAG_VEL_2(vel2); g_trigger_occurred[1] = true; }
    if (trig3) { flags |= FLAG_TRIGGER_3 | FLAG_VEL_3(vel3); g_trigger_occurred[2] = true; }
    if (trig4) { flags |= (1 << 30); g_trigger_occurred[3] = true; }
    
    if (Connected(Input::Pulse1)) flags |= FLAG_EXT_CLK;
    if (run_gate) flags |= FLAG_EXT_RUN;
    if (ext_clk_rising) flags |= (1 << 31); // Clock trigger
    
    // --- SYNCHRONOUS HANDSHAKE WITH CORE 1 ---
    // Push triggers and parameters to Core 1 (BLOCKING)
    multicore_fifo_push_blocking(flags);
    
    // Pop audio from Core 1 (BLOCKING)
    int16_t last_sampleL = (int16_t)multicore_fifo_pop_blocking();
    int16_t last_sampleR = (int16_t)multicore_fifo_pop_blocking();
    uint32_t last_pulse_mask = multicore_fifo_pop_blocking();
    
    // Write out stereo samples
    AudioOut1(last_sampleL);
    AudioOut2(last_sampleR);
    
    // Drive dedicated trigger output jacks (Pulse 1, Pulse 2, CV 1, CV 2)
    PulseOut1(last_pulse_mask & (1 << 0));
    PulseOut2(last_pulse_mask & (1 << 1));
    CVOut1((last_pulse_mask & (1 << 2)) ? 2047 : -2048);
    CVOut2((last_pulse_mask & (1 << 3)) ? 2047 : -2048);
    
    g_live_pulse_mask = last_pulse_mask;
  }
};

int main() {
  vreg_set_voltage(VREG_VOLTAGE_1_25);
  sleep_ms(10);
  set_sys_clock_khz(240000, true);
  stdio_init_all();
  
  static DrumsCard card;
  card.init_flash_samples();
  card.EnableNormalisationProbe();
  
  multicore_launch_core1(core1_worker);
  sleep_ms(20); // Wait for Core 1 to initialize and enter its blocking state
  
  // Clear any boot FIFO junk
  while (multicore_fifo_rvalid()) {
    multicore_fifo_pop_blocking();
  }
  
  card.Run();
}
