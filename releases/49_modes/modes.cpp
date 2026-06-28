// =============================================================================
//  49_modal — Mutable Instruments Elements port for Workshop Computer
//
//  A modal synthesis voice inspired by Émilie Gillet's Elements module.
//  Ported from the mi-UGens SuperCollider adaptation by Volker Böhm.
//
//  Original code: Émilie Gillet (Mutable Instruments) — MIT License
//  SC port: Volker Böhm — GPL3
//  Workshop Computer port: Vincent Maurer — GPL3
//
//  Target: RP2040 (Cortex-M0+, no FPU, 264KB RAM)
//  All DSP uses Q15 fixed-point arithmetic (int32_t with >>15 scaling).
//  Audio runs at 48kHz on Core 0, DSP engine at 24kHz on Core 1.
// =============================================================================

// ── ComputerCard setup ──────────────────────────────────────────────────────
// Use 144kHz system clock for reduced ADC tonal artifacts.
// ProcessSample runs at 48kHz; we decimate to 24kHz for the DSP engine.
#define COMPUTERCARD_SAMPLE_RATE_DIV 1
#include "ComputerCard.h"

#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <cstdlib>
#include <string.h>

#include "tusb.h"
#include "usb_midi_host.h"
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <pico/flash.h>
#include <pico/unique_id.h>
#include <stdio.h>

// ── Fixed-point DSP infrastructure ──────────────────────────────────────────
#include "diffuser_q15.h"
#include "dsp_q15.h"
#include "envelope_q15.h"
#include "exciter_q15.h"
#include "plate_reverb_q15.h"
#include "resonator_q15.h"
#include "string_q15.h"
#include "svf_q15.h"
#include "tube_q15.h"

// ── Constants ───────────────────────────────────────────────────────────────

#define SAMPLE_RATE 48000
#define DSP_RATE 24000       // Effective DSP rate (every 2nd sample)
#define NUM_PAGES 6          // Number of parameter pages
#define BOOT_SILENCE_MS 1000 // Startup mute duration in samples
#define BOOT_SILENCE_SAMP (SAMPLE_RATE * BOOT_SILENCE_MS / 1000)

// ── Fixed-point helpers ─────────────────────────────────────────────────────
// mul_q15() and other core Q15 ops are in dsp_q15.h.
// Knob scaling is project-specific:

/// Scale a raw knob value (0–4095) to Q15 (0–32767), with dead zones at edges.
/// The pots don't reliably reach 0, so we remap ~14–4095 → 0–32767.
static inline int32_t knob_to_q15(int32_t raw) {
  int32_t v = (raw - 50) * 32767 / (4095 - 100);
  if (v < 0)
    v = 0;
  if (v > 32767)
    v = 32767;
  return v;
}

// ── DC Blocker ──────────────────────────────────────────────────────────────
// Simple first-order high-pass filter to remove DC offset from audio signals.

static inline int32_t dc_block(int32_t x, int32_t &px, int32_t &py) {
  int32_t py_div = (py > 0) ? ((py + 255) >> 8) : ((py < 0) ? ((py - 255) >> 8) : 0);
  int32_t y = x - px + py - py_div;
  px = x;
  py = y;
  return y;
}

// ── PRNG ────────────────────────────────────────────────────────────────────
// Fast linear congruential generator, one seed per core to avoid contention.

static uint32_t rand_seed0 = 12345, rand_seed1 = 67890;

uint32_t __not_in_flash_func(fast_rand)(int core) {
  uint32_t &s = (core == 0) ? rand_seed0 : rand_seed1;
  s = (1103515245u * s + 12345u);
  return s;
}

// ── Knob Lock ───────────────────────────────────────────────────────────────
// When switching pages, knobs are "locked" to their last value. They only
// start controlling the new page's parameter once the user moves them past
// a threshold (~40% of range), preventing parameter jumps on page change.

struct KnobLock {
  bool locked = true;
  int32_t ref = 0;

  /// Lock the knob at its current value (called on page change)
  void engage(int32_t v) {
    locked = true;
    ref = v;
  }

  /// Returns true if the knob is "unlocked" and should update the parameter.
  /// Unlocks when the knob moves >1638 units (~5% of Q15 range) from ref.
  bool update(int32_t v) {
    if (locked) {
      int32_t d = v - ref;
      if (d < 0)
        d = -d;
      if (d > 1638)
        locked = false;
    }
    return !locked;
  }

  /// Force the knob back into a locked state (used when CC takes over)
  void relock(int32_t v) {
    locked = true;
    ref = v;
  }
};

// ── Page Parameter Storage ──────────────────────────────────────────────────
// Each page stores 3 parameters (one per knob: Main, X, Y) in Q15 range.
// Pages are logically grouped to match Elements' control layout.
//
// Page 0: STRIKE         — Level / Timbre / Model (Meta)
// Page 1: BLOW           — Level / Timbre / Texture (Meta)
// Page 2: BOW & ENV      — Bow Level / Bow Timbre / Env Shape
// Page 3: RESONATOR 1    — Geometry / Brightness / Damping
// Page 4: RESONATOR 2    — Position / Space / [Unused]
// Page 5: PERFORMANCE    — Pitch Coarse / Fine Tune / Exciter Strength

struct PageParams {
  volatile int32_t pMain, pX, pY;
};

// Track what we last sent over MIDI to avoid flooding and loops
struct MIDIState {
  int32_t pMain, pX, pY;
};

// ── Resonator Model ─────────────────────────────────────────────────────────
enum ResonatorModel {
  MODEL_MODAL = 0,        // Bank of bandpass filters (SVF) simulating modes
  MODEL_STRING = 1,       // Single Karplus-Strong string
  MODEL_STRINGS_POLY = 2, // 3-voice polyphonic strings (round-robin voicing)
  MODEL_CHORDS = 3,       // Chord of 3 strings (triggered together)
  MODEL_COUNT = 4
};

// ── Core 1 Communication ────────────────────────────────────────────────────
// We use the RP2040 multicore FIFO to send parameters from Core 0 (audio IRQ)
// to Core 1 (DSP engine) and receive processed audio samples back.
//
// Protocol (every 2nd ProcessSample call):
//   Core 0 → Core 1:  word 0 = trigger/gate flags
//   Core 1 → Core 0:  word 0 = left sample (Q15)
//                      word 1 = right sample (Q15)

// Trigger flags packed into the FIFO word
#define FIFO_FLAG_ACTIVE 0x00000001    // "I'm sending you work"
#define FIFO_FLAG_GATE 0x00000002      // Gate is high (Pulse In 1)
#define FIFO_FLAG_RISING 0x00000004    // Gate just went high
#define FIFO_FLAG_FALLING 0x00000008   // Gate just went low
#define FIFO_FLAG_MODEL_CHG 0x00000010 // Resonator model changed

// ── Shared State (volatile for cross-core access) ───────────────────────────

// Parameters are written by Core 0 and read by Core 1.
// Cross-core access is safe because writes are atomic at the word level
// and we only write from Core 0 / read from Core 1.
PageParams params[NUM_PAGES];
MIDIState last_sent[NUM_PAGES];
volatile int32_t cv1_pitch_q8 = 0;  // V/Oct pitch from CV In 1 (Q8)
volatile int32_t cv2_strength = 0;  // Strength from CV In 2 (Q15)
volatile int32_t audio_in1_q15 = 0; // Blow external input (Q15)
volatile int32_t audio_in2_q15 = 0; // Strike external input (Q15)
volatile int32_t currentModel = MODEL_MODAL;
static volatile int32_t poly_pitch_q8[4] = {6144, 6144, 6144, 6144};
static volatile int poly_voice = 0;

// ── MIDI State ──────────────────────────────────────────────────────────────
volatile int32_t midi_pitch_q8 = 0; // MIDI note pitch (Q8)
volatile bool midi_trigger = false; // MIDI note-on trigger
volatile int32_t midi_velocity_q15 = 25600; // MIDI velocity scaled to Q15 (Default 100)
volatile bool midi_gate = false;    // MIDI gate state
uint8_t midi_dev_addr = 0;          // Current MIDI device address
bool isUSBMIDIHost = false;         // Are we acting as a host?
ComputerCard::USBPowerState_t usb_power_state = ComputerCard::Unsupported;

// ── Preset System ───────────────────────────────────────────────────────────
#define FLASH_PRESET_OFFSET (1536 * 1024) // 1.5MB offset (plenty of room)
#define PRESET_COUNT 16
#define PRESET_MAGIC 0x4D4F4442 // "MODB" - forced update to rewrite new presets

struct Preset {
  uint32_t magic;
  uint32_t model;
  uint32_t sequence;   // 16-step bitmask
  uint8_t pitches[16]; // 16 pitches
  int32_t bpm_val;
  int32_t density_val;
  int32_t map_val;
  uint8_t scale;
  uint8_t root;
  PageParams pages[NUM_PAGES];
  uint32_t reserved[16];
};

#define GLOBAL_SETTINGS_OFFSET 4000
#define GLOBAL_SETTINGS_MAGIC 0x474C4F42 // "GLOB"

struct GlobalSettings {
  uint32_t magic;
  uint8_t midi_channel; // 0=Omni, 1-16
  uint8_t reserved[3];
};

static uint8_t runtime_midi_channel = 0; // 0=Omni
volatile bool web_ui_connected = false;

enum UIState { STATE_NORMAL, STATE_LOAD_MENU, STATE_SAVE_MENU, STATE_GEN_SEQ };

volatile UIState currentUIState = STATE_NORMAL;

// ── Generative Sequencer State ──────────────────────────────────────────────
static int gen_step = 0;
static uint32_t gen_sequence = 0xAAAA; // 16-step bitmask
static uint8_t gen_pitches[16];
static int32_t gen_cv2_val = 0;
static bool gen_gate = false;
static bool gen_gate_pending = false;
static bool gen_pulse_clk = false;
static int32_t gen_melody_q8 = 0;
static volatile bool audio1_connected_flag = false;
static int32_t seq_bpm_val = 16384;
static int32_t seq_density_val = 16384;
static int32_t seq_map_val = 0;
static int32_t last_seq_dens = -1, last_seq_bpm = -1, last_seq_ent = -1;
static uint32_t last_cc_tx_time[128] = {0};
static uint8_t currentScale =
    0; // 0=PentMinor, 1=PentMajor, 2=Minor, 3=Major, 4=Chromatic
static uint8_t currentRoot = 0; // 0=C, 1=C#, etc.
int32_t presetMenuSlot = 0;

// ── MIDI Clock Sync & Host Parser State ─────────────────────────────────────
extern "C" void handle_midi_message(uint8_t *packet, int size);
volatile uint32_t last_midi_clock_ms = 0;
volatile int midi_clock_ticks = 0;
volatile bool midi_clock_step_trigger = false;
volatile bool midi_clock_reset_pending = false;

static uint8_t host_midi_status = 0;
static uint8_t host_midi_buf[3];
static int host_midi_len = 0;
static int host_midi_expected = 0;
static bool host_in_sysex = false;

int get_midi_message_length(uint8_t status) {
  uint8_t type = status & 0xF0;
  if (type >= 0x80 && type < 0xF0) {
    switch (type) {
      case 0x80: case 0x90: case 0xA0: case 0xB0: case 0xE0:
        return 3;
      case 0xC0: case 0xD0:
        return 2;
    }
  } else if (status >= 0xF0 && status < 0xF8) {
    switch (status) {
      case 0xF1: case 0xF3:
        return 2;
      case 0xF2:
        return 3;
      case 0xF0:
        return -1; // SysEx variable length
      default:
        return 1;
    }
  }
  return 0;
}

void handle_midi_realtime(uint8_t status) {
  if (status == 0xF8) {
    last_midi_clock_ms = to_ms_since_boot(get_absolute_time());
    midi_clock_ticks++;

    // Dynamic MIDI clock multiplier/divider based on Sequencer X Knob (seq_bpm_val)
    int ticks_per_step = 6; // default 1/16 note (6 ticks)
    if (seq_bpm_val < 5461) {
      ticks_per_step = 24; // 1/4 notes
    } else if (seq_bpm_val < 10922) {
      ticks_per_step = 12; // 1/8 notes
    } else if (seq_bpm_val < 21845) {
      ticks_per_step = 6;  // 1/16 notes
    } else if (seq_bpm_val < 27306) {
      ticks_per_step = 3;  // 1/32 notes
    } else {
      ticks_per_step = 2;  // 1/48 notes (triplets)
    }

    if (midi_clock_ticks >= ticks_per_step) {
      midi_clock_ticks = 0;
      midi_clock_step_trigger = true;
    }
  } else if (status == 0xFA || status == 0xFB) {
    midi_clock_ticks = 999; // Force next tick to trigger step 0 immediately
    midi_clock_reset_pending = true;
  }
}

void parse_host_midi_byte(uint8_t b) {
  if (b >= 0xF8) {
    handle_midi_realtime(b);
    return;
  }
  if (b >= 0x80) {
    host_in_sysex = (b == 0xF0);
    host_midi_status = b;
    host_midi_len = 0;
    host_midi_expected = get_midi_message_length(b);
    if (host_midi_expected == 1) {
      uint8_t msg[1] = {b};
      handle_midi_message(msg, 1);
      host_midi_status = 0;
    } else if (host_midi_expected == 2 || host_midi_expected == 3) {
      host_midi_buf[0] = b;
      host_midi_len = 1;
    }
    return;
  }
  if (host_in_sysex) return;
  if (host_midi_status != 0) {
    if (host_midi_len < host_midi_expected) {
      host_midi_buf[host_midi_len++] = b;
      if (host_midi_len == host_midi_expected) {
        handle_midi_message(host_midi_buf, host_midi_expected);
        host_midi_len = 1; // Running status
      }
    }
  }
}

// MIDI CC mappings for each page [Page][Knob]
static const uint8_t PageCCs[6][3] = {
    {34, 35, 36}, // Page 0: Strike
    {40, 41, 42}, // Page 1: Bow & Envelope
    {37, 38, 39}, // Page 2: Blow
    {43, 44, 45}, // Page 3: Resonator 1
    {46, 47, 48}, // Page 4: Resonator 2 (Pos, Space, Reverb)
    {49, 50, 51}  // Page 5: Perf (Coarse, Fine, Strength)
};

// ── Core 1 DSP Engine ──────────────────────────────────────────────────────
// ── Core 1 DSP Engine ──────────────────────────────────────────────────────
// This function runs on Core 1 in an infinite loop. It waits for trigger
// words from Core 0, processes one sample of DSP, and sends audio back.
//
// Signal flow: Gate → Envelope → Exciters → Resonator → Stereo Output

// Core 1 persistent state — lives outside the function to avoid stack pressure
static ExciterQ15 bow_exciter;
static ExciterQ15 blow_exciter;
static ExciterQ15 strike_exciter;
static EnvelopeQ15 envelope;
static ResonatorQ15 resonator;
static StringQ15 strings[4];
static DiffuserQ15 diffuser;
static TubeQ15 tube;
static PlateReverbQ15 reverb; // Moved to Core 0 context logically
static OnePoleFilterQ15 excitation_filter;

static int32_t strum_delay_buffer[512];
static uint16_t strum_delay_ptr = 0;

static int32_t dc_ex_x = 0;
static int32_t dc_ex_y = 0;

// Smoothed parameters for zipper-free interpolation
static int32_t smooth_strength = 0;
static int32_t smooth_env_value = 0;

volatile bool pause_core1 = false;
volatile bool core1_is_paused = false;

static void __not_in_flash_func(core1_dsp_loop)() {

  // Initialize exciters with different PRNG seeds
  bow_exciter.Init(12345);
  blow_exciter.Init(67890);
  strike_exciter.Init(24680);
  envelope.Init();
  resonator.Init();
  for (int i = 0; i < 4; i++)
    strings[i].Init();
  diffuser.Init();
  tube.Init();
  excitation_filter.Init();

  memset(strum_delay_buffer, 0, sizeof(strum_delay_buffer));
  strum_delay_ptr = 0;

  // Set default exciter models
  bow_exciter.model = EXCITER_Q15_FLOW;
  bow_exciter.parameter = 22938; // ~0.7
  bow_exciter.timbre = 16384;    // ~0.5

  blow_exciter.model = EXCITER_Q15_NOISE;

  strike_exciter.model = EXCITER_Q15_MALLET;

  uint32_t prng_seed = 0x12345678;
  int32_t random_pitch = 0;
  int32_t random_timbre = 0;

  while (1) {
    if (pause_core1) {
      core1_is_paused = true;
      while (pause_core1) {
        tight_loop_contents();
      }
      core1_is_paused = false;
    }

    if (!multicore_fifo_rvalid()) {
      tight_loop_contents();
      continue;
    }

    uint32_t flags = multicore_fifo_pop_blocking();
    if (!(flags & FIFO_FLAG_ACTIVE)) {
      multicore_fifo_push_blocking(0);
      multicore_fifo_push_blocking(0);
      continue;
    }

    // Simple PRNG for humanization
    auto get_rand = [&]() {
      prng_seed = prng_seed * 1103515245 + 12345;
      return (int32_t)((prng_seed >> 16) & 0x7FFF);
    };

    // ── Read Parameters ─────────────────────────────────────────────
    // Page 0: Strike
    int32_t strike_level = params[0].pMain;
    int32_t strike_timbre = params[0].pX;
    int32_t strike_meta = params[0].pY;

    // Page 1: Bow & Envelope
    int32_t bow_level = params[1].pMain;
    int32_t bow_timbre = params[1].pX;
    int32_t env_shape = params[1].pY;

    // Page 2: Blow
    int32_t blow_level = params[2].pMain;
    int32_t blow_timbre = params[2].pX;
    int32_t blow_meta = params[2].pY;

    // Page 3: Resonator Core
    int32_t geometry = params[3].pMain;
    int32_t brightness = params[3].pX;
    int32_t damping = params[3].pY;

    // Page 4: Resonator Space
    int32_t position = params[4].pMain;

    // Page 5: Performance
    int32_t pitch_coarse = params[5].pMain;
    int32_t fine_tune = params[5].pX;
    int32_t strength = params[5].pY;

    // External inputs
    int32_t ext_audio = audio_in1_q15;  // Mono audio input for excitation
    int32_t cv_damping = audio_in2_q15; // Use Audio 2 as CV for Damping
    int32_t pitch_cv = cv1_pitch_q8;
    int32_t cv_bright =
        cv2_strength; // Use CV 2 as CV for Brightness modulation

    // Gate state
    bool gate = (flags & FIFO_FLAG_GATE) != 0;
    bool rising = (flags & FIFO_FLAG_RISING) != 0;
    bool falling = (flags & FIFO_FLAG_FALLING) != 0;

    // On every trigger, generate slight deviations for more organic sound
    if (rising) {
      // Pitch: ±8 units (approx ±3 cents)
      random_pitch = (get_rand() & 0x1F) - 16;
      // Timbre: ±400 units (approx 1.2%)
      random_timbre = (get_rand() & 0x3FF) - 512;
    }

    // Suppress unused warnings for parameters not yet wired

    // ── Build gate flags for exciters/envelope ──────────────────────
    uint8_t env_flags = 0;
    if (gate)
      env_flags |= ENV_FLAG_GATE;
    if (rising)
      env_flags |= ENV_FLAG_RISING;
    if (falling)
      env_flags |= ENV_FLAG_FALLING;

    // ── Configure Envelope ──────────────────────────────────────────
    // env_shape (Q15): 0..13107 = AD, 13107..19660 = ADSR, 19660..32767 = AR
    // Maps the original Elements envelope shape parameter
    if (env_shape < 13107) {
      // Short AD shapes (0..0.4): attack + decay, no sustain
      // a = shape*0.75 + 0.15 → Q15: shape*24576/32767 + 4915
      int32_t a = mul_q15(env_shape, 24576) + 4915;
      int32_t dr = mul_q15(a, 29491) << 1; // a * 1.8
      if (dr > 32767)
        dr = 32767;
      envelope.SetADR(a, dr, 0, dr);
    } else if (env_shape < 19660) {
      // ADSR with increasing sustain (0.4..0.6)
      int32_t s = (env_shape - 13107) * 5; // scale to 0..32767
      if (s > 32767)
        s = 32767;
      envelope.SetADSR(14746, 26542, s, 26542); // a=0.45, dr=0.81
    } else {
      // Long AR shapes (0.6..1.0): full sustain
      int32_t a = mul_q15(32767 - env_shape, 24576) + 4915;
      int32_t dr = mul_q15(a, 29491) << 1;
      if (dr > 32767)
        dr = 32767;
      envelope.SetADSR(a, dr, 32767, dr);
    }

    // Process envelope
    int32_t env_value = envelope.Process(env_flags);

    // Smooth envelope to avoid zipper noise
    smooth_env_value += (env_value - smooth_env_value) >> 3;

    // ── CV Modulations ──────────────────────────────────────────────
    // Map CV2 to Brightness and Audio2 to Damping
    // Shift right by 1 to make the modulation depth sensible (50%)
    int32_t total_brightness = brightness + (cv_bright >> 1) + random_timbre;
    if (total_brightness < 0)
      total_brightness = 0;
    if (total_brightness > 32767)
      total_brightness = 32767;

    int32_t total_damping_cv = damping + (cv_damping >> 1);
    if (total_damping_cv < 0)
      total_damping_cv = 0;
    if (total_damping_cv > 32767)
      total_damping_cv = 32767;

    // Modulate position by Audio 2 (cv_damping) up to 50% depth
    int32_t total_position = position + (cv_damping >> 1);
    if (total_position < 0)
      total_position = 0;
    if (total_position > 32767)
      total_position = 32767;

    // Modulate exciter timbres by CV 2 (cv_bright) up to 50% depth
    int32_t cv2_timbre_mod = cv_bright >> 1;
    
    int32_t total_strike_timbre = strike_timbre + cv2_timbre_mod;
    if (total_strike_timbre < 0) total_strike_timbre = 0;
    if (total_strike_timbre > 32767) total_strike_timbre = 32767;

    int32_t total_blow_timbre = blow_timbre + cv2_timbre_mod;
    if (total_blow_timbre < 0) total_blow_timbre = 0;
    if (total_blow_timbre > 32767) total_blow_timbre = 32767;

    int32_t total_bow_timbre = bow_timbre + cv2_timbre_mod;
    if (total_bow_timbre < 0) total_bow_timbre = 0;
    if (total_bow_timbre > 32767) total_bow_timbre = 32767;

    // Modulate geometry by Audio 2 (cv_damping) up to 50% depth
    int32_t total_geometry = geometry + (cv_damping >> 1);
    if (total_geometry < 0)
      total_geometry = 0;
    if (total_geometry > 32767)
      total_geometry = 32767;

    // ── Configure Resonator / String ────────────────────────────────

    resonator.geometry_q15 = total_geometry;
    resonator.brightness_q15 = total_brightness;
    resonator.position_q15 = total_position;

    int32_t max_active_strings = (currentModel == MODEL_STRINGS_POLY || currentModel == MODEL_CHORDS) ? 4 : 1;
    for (int i = 0; i < max_active_strings; i++) {
      strings[i].SetDispersion(total_geometry);
      strings[i].SetBrightness(total_brightness);
      strings[i].SetPosition(total_position);
    }

    // Model toggle: changes the number of active modes for Modal
    int32_t model = currentModel;
    if (model == MODEL_MODAL) {
      resonator.resolution = kMaxModesQ15;
      resonator.structure = ResonatorQ15::STRUC_MODAL;
    } else if (model == MODEL_STRING) {
      resonator.resolution = 6;
    } else {
      resonator.resolution = 12;
    }

    // Stereo modulation LFO: ~0.5Hz / 24kHz = 2.08e-5
    // In Q15: 0.5/24000 * 32768 ≈ 0.68 → round to 1
    resonator.modulation_frequency_q15 = 1;
    resonator.modulation_offset_q15 = 3277; // 0.1 in Q15

    // ── Pitch Mapping ────────────────────────────────────────────────
    // pitch_coarse (Q15: 0..32767) → MIDI 24..96 (72 semitones)
    // Center (16384) = MIDI 60 (C4 = 261.6Hz) — musically useful range
    // In Q8: MIDI 60 = 15360.
    // Range: 24..96 (C1..C7) — good for both bass drones and bells
    // Mapping: midi = 24 + (pitch_coarse * 72 / 32767)
    //        = 24*256 + (pitch_coarse * 72*256 / 32767)
    //        ≈ 6144 + (pitch_coarse * 18432 / 32767)
    //        ≈ 6144 + (pitch_coarse * 18432 >> 15)
    int32_t midi_q8 = 6144 + (int32_t)(((int64_t)pitch_coarse * 18432) >> 15);

    // Fine tune: ±2 semitones (Page 5 X knob)
    // fine_tune 0..32767 → -2..+2 semitones → -512..+512 in Q8
    midi_q8 += ((fine_tune - 16384) >> 5);

    // CV1 V/Oct pitch tracking
    // pitch_cv is already scaled correctly to Q8 (1V = 1 octave = 3072)
    midi_q8 += pitch_cv;



    // Clamp to valid MIDI range
    if (midi_q8 < 0)
      midi_q8 = 0; // MIDI 0 (~8Hz)
    if (midi_q8 > 30720)
      midi_q8 = 30720; // MIDI 120 (C9)

    // Polyphonic round-robin pitch capture on trigger rising edge
    if (rising && currentModel == MODEL_STRINGS_POLY) {
      poly_voice = (poly_voice + 1) % 4;
      poly_pitch_q8[poly_voice] = midi_q8;
    }

    // Convert MIDI pitch to normalized frequency via phase increment.
    // MidiToIncrementU32() internally adds +38<<8 to the index,
    // so we pass the raw MIDI Q8 value directly — no external offset.
    uint32_t inc = MidiToIncrementU32(midi_q8);
    // MidiToIncrementU32 returns inc = (f / 32000) * 2^32.
    // We need f_norm = f / sr_dsp, where sr_dsp = 24kHz, in Q15.
    //   f_norm_q15 = (f / 24000) * 32768
    //              = (f / 32000) * (32000 / 24000) * 32768
    //              = (f / 32000) * (4 / 3) * 32768
    //              = (inc / 2^32) * 43690.66
    //              = (inc * 43691) >> 32
    int32_t freq_q15 = (int32_t)(((uint64_t)inc * 43691) >> 32);
    if (freq_q15 > 16056)
      freq_q15 = 16056; // cap at 0.49 Nyquist
    if (freq_q15 < 1)
      freq_q15 = 1;
    resonator.frequency_q15 = freq_q15;

    // Apply frequencies to strings, including chord offsets
    // chords_table offsets are in semitones. 1 semitone = 256 in Q8.
    int32_t num_strings = (currentModel == MODEL_STRINGS_POLY || currentModel == MODEL_CHORDS) ? 4 : 1;
    for (int i = 0; i < num_strings; i++) {
      int32_t string_midi = midi_q8;
      if (currentModel == MODEL_STRINGS_POLY) {
        string_midi = poly_pitch_q8[i];
      } else if (currentModel == MODEL_CHORDS) {
        // For discrete chord selection, use kMain (raw-ish) to avoid sluggish
        // jumps but keep geometry (smoothed) for continuous SetDispersion
        // below.
        int32_t chord_idx = (total_geometry * 11) >> 15;
        if (chord_idx > 10)
          chord_idx = 10;

        static const int16_t chord_offsets[11][4] = {
            {-12 * 256, 0, 3, 12 * 256},             // Octaves
            {-12 * 256, 3 * 256, 7 * 256, 10 * 256}, // Minor 7th
            {-12 * 256, 3 * 256, 7 * 256, 12 * 256}, // Minor
            {-12 * 256, 3 * 256, 7 * 256, 14 * 256}, // Minor 9th
            {-12 * 256, 3 * 256, 7 * 256, 17 * 256}, // Minor 11th
            {-12 * 256, 7 * 256, 12 * 256, 19 * 256}, // Fifth (Power Chord stack)
            {-12 * 256, 4 * 256, 7 * 256, 17 * 256}, // Major 11th
            {-12 * 256, 4 * 256, 7 * 256, 14 * 256}, // Major 9th
            {-12 * 256, 4 * 256, 7 * 256, 12 * 256}, // Major
            {-12 * 256, 4 * 256, 7 * 256, 11 * 256}, // Major 7th
            {-12 * 256, 5 * 256, 7 * 256, 12 * 256}  // Sus4
        };
        string_midi += chord_offsets[chord_idx][i];
      }
      if (string_midi < 0)
        string_midi = 0;
      if (string_midi > 30720)
        string_midi = 30720;
      // MidiToIncrementU32 handles the LUT offset internally
      uint32_t s_inc = MidiToIncrementU32(string_midi);
      strings[i].SetFrequency(s_inc);
    }

    // ── Configure Exciters ──────────────────────────────────────────

    // Brightness factor: resonator brightness modulates exciter timbre
    // brightness_factor = 0.4 + 0.6 * brightness → Q15: 13107 + brightness *
    // 0.6
    int32_t brightness_factor = 13107 + mul_q15(brightness, 19661);

    // Bow: Flow model, timbre controlled by total_bow_timbre * brightness
    bow_exciter.timbre = mul_q15(total_bow_timbre, brightness_factor);
    bow_exciter.model = EXCITER_Q15_FLOW;
    // Turbulence driven by bow timbre: soft timbre = smooth bowing, bright timbre = scratchy
    // Range 0.2..1.0 maps from silky-smooth to coarse raspy scrape (matches Elements)
    bow_exciter.parameter = 6554 + mul_q15(total_bow_timbre, 26214);

    // Blow: Granular sample player (matching original)
    blow_exciter.parameter = blow_meta;
    blow_exciter.timbre = total_blow_timbre;
    blow_exciter.signature =
        blow_meta; // Tie signature to meta for texture variation
    blow_exciter.model = EXCITER_Q15_GRANULAR;

    // Strike: Use meta to select model (Sample→Mallet→Plectrum→Particles)
    // strike_meta <= 0.4: scale to 0..0.25 range (sample player region)
    // strike_meta > 0.4: scale to 0.25..1.0 range (synth models)
    int32_t adjusted_meta;
    if (strike_meta <= 13107) {
      adjusted_meta = mul_q15(strike_meta, 20480); // * 0.625
    } else {
      adjusted_meta = mul_q15(strike_meta, 40960) - 8192; // * 1.25 - 0.25
    }
    if (adjusted_meta < 0)
      adjusted_meta = 0;
    if (adjusted_meta > 32767)
      adjusted_meta = 32767;
    strike_exciter.SetMeta(adjusted_meta, EXCITER_Q15_SAMPLE,
                           EXCITER_Q15_PARTICLES);
    strike_exciter.timbre = total_strike_timbre;
    strike_exciter.signature = strike_meta; // Tie signature to meta

    // ── Process Exciters ────────────────────────────────────────────

    int32_t bow_out = bow_exciter.Process(env_flags);
    int32_t blow_out = blow_exciter.Process(env_flags);
    int32_t strike_out = strike_exciter.Process(env_flags);

    // ── Smooth Strength ─────────────────────────────────────────────
    // Strength from knob + CV2
    int32_t total_strength = mul_q15(strength, midi_velocity_q15) + cv2_strength;
    if (total_strength < 0)
      total_strength = 0;
    if (total_strength > 32767)
      total_strength = 32767;
    smooth_strength += (total_strength - smooth_strength) >> 4;

    // Accent gain from strength
    int32_t accent = AccentGainQ14(smooth_strength);

    // ── Mix Excitation ──────────────────────────────────────────────
    // Replicate the original mix logic from voice.cc:
    //
    // blow: level < 1.0 → blow * 0.4, level ≥ 1.0 → 0.4
    //       (tube level for values > 1.0, but tube is omitted for now)
    // strike: level < 1.0 → strike * level * 1.5, level ≥ 1.0 → strike * 1.5
    //         bleed: level > 1.0 → raw strike bleeds to output
    // bow: bow * bow_level * envelope * accent * 0.125

    int32_t e = mul_q15_q14(smooth_env_value, accent);

    // Bow contribution: bow * bow_level * e * 1.8 (reduced from 2.5)
    int32_t bow_mix = mul_q15(mul_q15(bow_out, bow_level), e);
    bow_mix = (bow_mix * 9) >> 2;

    // Blow contribution: blow * blow_level_scaled * e + tube body
    // blow_level < 0.5 (16384): scale to 0..1.0 for noise level
    // blow_level >= 0.5: noise level stays at 0.4, and tube level increases
    int32_t blow_noise_lvl;
    int32_t tube_amt = 0;
    if (blow_level < 16384) {
      blow_noise_lvl = blow_level << 1; // 0..32767
      tube_amt = 0;
    } else {
      blow_noise_lvl = 13107;               // 0.4 fixed
      tube_amt = (blow_level - 16384) << 1; // 0..32767
    }

    int32_t b_noise = mul_q15(blow_out, blow_noise_lvl);
    b_noise = mul_q15(b_noise, e);

    // Process Tube (Flute Body)
    // Gain is reduced (tube_amt >> 3) to keep the Blow section from
    // overpowering
    int32_t tube_out =
        tube.Process(freq_q15, smooth_env_value, damping, total_blow_timbre, b_noise);
    int32_t b_mix = b_noise + mul_q15(tube_out, tube_amt >> 3);

    // Strike contribution: strike * accent * strike_level_scaled + external
    // strike_level_scaled = min(strike_level * 1.25, 1.0) * 0.9 (reduced
    // from 1.1)
    int32_t strike_lvl_adj = mul_q15(strike_level, 40960); // * 1.25
    if (strike_lvl_adj > 32767)
      strike_lvl_adj = 32767;
    int32_t strike_scaled = mul_q15(strike_lvl_adj, 29491); // * 0.9
    int32_t strike_mix = mul_q15(mul_q15_q14(strike_out, accent), strike_scaled);

    // ── Position-Dependent Exciter Bleed ───────────────────────────
    // In a real physical instrument, you hear direct transient impact and friction noise
    // (strike hammer, bow rasp, blow air) mixed with the resonance, depending on pickup position.
    // We scale this raw exciter bleed quadratically with the Position knob.
    int32_t raw_exciter_bleed = 0;
    
    // 1. Strike bleed: raw hammer/plectrum transient
    if (strike_level > 20000) { 
      raw_exciter_bleed += mul_q15(strike_out, (strike_level - 20000) << 1);
    }
    // 2. Bow bleed: direct bow scraping raspiness
    raw_exciter_bleed += mul_q15(bow_mix, 8000);
    // 3. Blow bleed: direct blowing breath noise
    raw_exciter_bleed += mul_q15(b_mix, 6000);

    // Scale raw bleed by position (quadratic curve: 0% at position=0, up to 100% at position=1.0)
    int32_t bleed_gain = mul_q15(total_position, total_position);
    int32_t active_bleed = mul_q15(raw_exciter_bleed, bleed_gain);

    // Sum all components that are always diffused (Blow, External)
    // Scale external exciter by blow_level to allow volume control
    int32_t to_diffuse = b_mix + mul_q15(ext_audio, blow_level);

    // Split strike component: diffusion depends on Strike Timbre
    // Low Timbre = soft mallet (diffused), High Timbre = hard stick (direct)
    // fade = 0 (low) -> strike_to_diffuse = strike_mix
    // fade = 32767 (high) -> strike_to_diffuse = 0
    int32_t strike_to_diffuse = strike_mix - mul_q15(strike_mix, total_strike_timbre);
    int32_t strike_direct = strike_mix - strike_to_diffuse;

    to_diffuse += strike_to_diffuse;

    // Process the diffuser stage (advances pointers once)
    int32_t diffused_excitation = diffuser.Process(to_diffuse);

    // Sum all parts: Bow (direct) + Diffused (Blow/Ext/SoftStrike) + Direct
    // Strike
    int32_t excitation = bow_mix + diffused_excitation + strike_direct;

    if (smooth_env_value < 20 && !audio1_connected_flag) {
      excitation = 0;
    }

    // ── Exciter Gain Staging ────────────────────────────────────────
    // Apply soft limiting to the combined excitation signal to prevent
    // the resonator from blowing up / clipping internally.
    excitation = SoftLimitQ15(excitation);

    // DC block the excitation signal (critical for strings because strike is a
    // positive impulse)
    excitation = DCBlockQ15(excitation, dc_ex_x, dc_ex_y);

    // Apply brightness-controlled low-pass filter on the excitation signal (all models).
    // This replicates the original Elements excitation filter: brightness knob simultaneously
    // controls both the resonator Q loss AND the colour of the excitation going in.
    // Low brightness = warm/wooden attack; high brightness = bright/metallic click.
    {
      int32_t ex_g = total_brightness >> 1; // Map 0..32767 → 0..16383 (Q14)
      if (ex_g < 150) ex_g = 150;           // Keep a warm fundamental bottom end
      excitation_filter.SetG(ex_g);
      excitation = excitation_filter.ProcessLP(excitation);
    }

    // ── Damping from Exciters ───────────────────────────────────────
    // Strike exciter provides damping feedback (palm mute effect):
    // Soft mallets (low adjusted_meta) = high inherent damping (short ring).
    // Hard sticks (high adjusted_meta) = low inherent damping (long sustain).
    // This gives the Strike meta knob a physical feel: mallets naturally mute, sticks ring.
    int32_t final_damping = total_damping_cv;
    int32_t derived_strike_damping = 32767 - adjusted_meta; // soft=high damp, hard=low damp
    final_damping -= mul_q15(derived_strike_damping, strike_lvl_adj) >> 3;
    // Bow damping: when bow is not pressed, it damps
    int32_t bow_strength_inv = 32767 - mul_q15(bow_level, smooth_env_value);
    final_damping -= mul_q15(bow_strength_inv, bow_level) >> 4;
    if (final_damping < 0)
      final_damping = 0;

    // ── Process Resonator / String ──────────────────────────────────
    // ── Output Stage ───────────────────────────────────────────────
    // We now send the raw Center and Side signals to Core 0.
    // Core 0 will handle the Stereo Widener (spread) and Delay (reverb).
    // This offloads significant CPU from Core 1.

    int32_t final_center = 0;
    int32_t final_sides = 0;

    if (currentModel == MODEL_MODAL) {
      int32_t res_center = 0;
      int32_t res_sides = 0;
      int32_t bow_strength_q15 = mul_q15(bow_level, smooth_env_value);

      resonator.damping_q15 = final_damping;
      resonator.Process1(bow_strength_q15, excitation, res_center, res_sides);

      final_center = res_center + active_bleed;
      final_sides = res_sides;
    } else {
      // String models
      
      // Write current excitation to the strum delay circular buffer
      strum_delay_buffer[strum_delay_ptr] = excitation;

      int32_t num_strings = (currentModel == MODEL_STRINGS_POLY || currentModel == MODEL_CHORDS) ? 4 : 1;
      // Chord table matching Rings' original 4-string voicing.
      // Values in Q8 semitones (1 semitone = 256)
      static const int16_t chord_offsets[11][4] = {
          {-12 * 256, 0, 3, 12 * 256},             // Octaves
          {-12 * 256, 3 * 256, 7 * 256, 10 * 256}, // Minor 7th
          {-12 * 256, 3 * 256, 7 * 256, 12 * 256}, // Minor
          {-12 * 256, 3 * 256, 7 * 256, 14 * 256}, // Minor 9th
          {-12 * 256, 3 * 256, 7 * 256, 17 * 256}, // Minor 11th
          {-12 * 256, 7 * 256, 12 * 256, 19 * 256}, // Fifth (Power Chord stack)
          {-12 * 256, 4 * 256, 7 * 256, 17 * 256}, // Major 11th
          {-12 * 256, 4 * 256, 7 * 256, 14 * 256}, // Major 9th
          {-12 * 256, 4 * 256, 7 * 256, 12 * 256}, // Major
          {-12 * 256, 4 * 256, 7 * 256, 11 * 256}, // Major 7th
          {-12 * 256, 5 * 256, 7 * 256, 12 * 256}  // Sus4
      };

      int32_t s_center = 0;
      int32_t s_sides = 0;
      int32_t sympathetic_bus = 0;

      for (int i = 0; i < num_strings; i++) {
        int32_t string_midi = midi_q8;
        if (currentModel == MODEL_STRINGS_POLY) {
          string_midi = poly_pitch_q8[i];
        } else if (currentModel == MODEL_CHORDS) {
          int32_t chord_idx = (total_geometry * 11) >> 15;
          if (chord_idx > 10)
            chord_idx = 10;
          string_midi += chord_offsets[chord_idx][i];
        }

        if (string_midi < 0)
          string_midi = 0;
        if (string_midi > 30720)
          string_midi = 30720;

        uint32_t s_inc = MidiToIncrementU32(string_midi);
        strings[i].SetFrequency(s_inc);

        // Sympathetic strings ring longer and are brighter in chords mode
        if (i > 0 && currentModel == MODEL_CHORDS) {
          // Scale progressively and make it proportional to final_damping so turning damping knob down works!
          int32_t string_index_q15 = (i == 1) ? 10922 : ((i == 2) ? 21845 : 32767);
          int32_t sym_damping = final_damping + mul_q15(string_index_q15, mul_q15(32767 - final_damping, 22938));
          strings[i].SetDamping(sym_damping);

          // Rings' boosted brightness for sympathetic strings:
          int32_t sym_brightness = total_brightness;
          sym_brightness = 2 * sym_brightness - mul_q15(sym_brightness, sym_brightness);
          sym_brightness = 2 * sym_brightness - mul_q15(sym_brightness, sym_brightness);
          if (sym_brightness > 32767) sym_brightness = 32767;
          strings[i].SetBrightness(sym_brightness);
        } else {
          strings[i].SetDamping(final_damping);
          strings[i].SetBrightness(total_brightness);
        }

        strings[i].SetPosition(total_position);
        strings[i].SetDispersion(total_geometry);

        int32_t c = 0, s = 0;
        int32_t current_in = 0;

        if (currentModel == MODEL_STRINGS_POLY) {
          // Route excitation ONLY to the active polyphonic voice string
          current_in = (i == poly_voice) ? excitation : 0;
        } else if (currentModel == MODEL_CHORDS) {
          // Excite directly with 100-sample (4.1ms) stagger delay per string
          int32_t delayed_exc = strum_delay_buffer[(strum_delay_ptr + 512 - i * 100) & 511];
          if (i == 0) {
            // String 0 is excited directly (0.5x gain to prevent clipping)
            current_in = mul_q15(delayed_exc, 16384);
          } else {
            // Strings 1, 2 & 3 are excited directly with delay AND receive sympathetic coupling!
            current_in = mul_q15(delayed_exc, 16384) + (sympathetic_bus >> 4);
          }
        } else {
          // Monophonic String
          current_in = excitation;
        }

        strings[i].Process(current_in, c, s);

        if (currentModel == MODEL_CHORDS && i == 0) {
          // Use output difference for sympathetic excitation
          sympathetic_bus = c - s;
        }

        // Fixed spatial panning across Mid-Side (scaled by Space/Spread knob on Core 0)
        int32_t pan = 0;
        if (num_strings == 4) {
          if (i == 0) pan = -32767;
          else if (i == 1) pan = -10922; // -1/3 of Q15
          else if (i == 2) pan = 10922;  // 1/3 of Q15
          else if (i == 3) pan = 32767;
        }

        s_center += c;
        s_sides += mul_q15(c, pan) + (s >> 1);
      }

      // Advance strum delay pointer
      strum_delay_ptr = (strum_delay_ptr + 1) & 511;

      final_center = s_center + active_bleed;
      final_sides = s_sides;
    }

    // ── Send Results Back (Center/Sides) ────────────────────────────
    multicore_fifo_push_blocking((uint32_t)final_center);
    multicore_fifo_push_blocking((uint32_t)final_sides);
  }
}

// ── Main Application Forward Declaration ────────────────────────────────────
class Modal;

// ── MIDI Callbacks ──────────────────────────────────────────────────────────

// Forward declarations for MIDI callbacks used by the USB host driver
extern "C" {
void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep,
                       uint8_t num_cables_rx, uint16_t num_cables_tx);
void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance);
void handle_midi_message(uint8_t *packet, int size);
void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets);
}

// ── Preset Management Logic ─────────────────────────────────────────────────

void load_global_settings() {
  const GlobalSettings *g = (const GlobalSettings *)(XIP_BASE + FLASH_PRESET_OFFSET + GLOBAL_SETTINGS_OFFSET);
  if (g->magic == GLOBAL_SETTINGS_MAGIC) {
    runtime_midi_channel = g->midi_channel;
  } else {
    runtime_midi_channel = 0; // Default to Omni
  }
}

void __not_in_flash_func(save_global_settings)() {
  static uint8_t flash_buffer[4096];
  
  pause_core1 = true;
  while (!core1_is_paused) {
    if (multicore_fifo_rvalid()) {
      multicore_fifo_pop_blocking();
    }
    tight_loop_contents();
  }
  
  uint32_t ints = save_and_disable_interrupts();
  
  memcpy(flash_buffer, (const uint8_t *)(XIP_BASE + FLASH_PRESET_OFFSET), 4096);
  
  GlobalSettings g;
  g.magic = GLOBAL_SETTINGS_MAGIC;
  g.midi_channel = runtime_midi_channel;
  memcpy(flash_buffer + GLOBAL_SETTINGS_OFFSET, &g, sizeof(GlobalSettings));
  
  flash_range_erase(FLASH_PRESET_OFFSET, 4096);
  flash_range_program(FLASH_PRESET_OFFSET, flash_buffer, 4096);
  
  restore_interrupts(ints);
  
  pause_core1 = false;
}

void __not_in_flash_func(save_preset)(int slot, const Preset *p_src = nullptr) {
  if (slot < 0 || slot >= PRESET_COUNT)
    return;

  Preset p;
  p.magic = PRESET_MAGIC;
  if (p_src) {
    p = *p_src;
    p.magic = PRESET_MAGIC;
  } else {
    p.model = (uint32_t)currentModel;
    p.sequence = gen_sequence;
    memcpy(p.pitches, gen_pitches, 16);
    p.bpm_val = seq_bpm_val;
    p.density_val = seq_density_val;
    p.map_val = seq_map_val;
    p.scale = currentScale;
    p.root = currentRoot;
    for (int i = 0; i < NUM_PAGES; i++) {
      p.pages[i] = params[i];
    }
  }

  // Static buffer to avoid stack overflow (RP2040 stack is small)
  static uint8_t flash_buffer[4096];

  pause_core1 = true;
  while (!core1_is_paused) {
    if (multicore_fifo_rvalid()) {
      multicore_fifo_pop_blocking();
    }
    tight_loop_contents();
  }
  
  uint32_t ints = save_and_disable_interrupts();

  // Copy existing presets to RAM buffer
  memcpy(flash_buffer, (const uint8_t *)(XIP_BASE + FLASH_PRESET_OFFSET), 4096);
  // Update the specific slot in our RAM buffer
  memcpy(flash_buffer + (slot * sizeof(Preset)), &p, sizeof(Preset));

  // Erase and reprogram the sector
  flash_range_erase(FLASH_PRESET_OFFSET, 4096);
  flash_range_program(FLASH_PRESET_OFFSET, flash_buffer, 4096);

  restore_interrupts(ints);
  
  pause_core1 = false;
}

void load_preset(int slot) {
  if (slot < 0 || slot >= PRESET_COUNT)
    return;

  const Preset *p = (const Preset *)(XIP_BASE + FLASH_PRESET_OFFSET +
                                     (slot * sizeof(Preset)));

  if (p->magic != PRESET_MAGIC)
    return;

  currentModel = (int32_t)p->model;

  // Sequencer Load
  gen_sequence = p->sequence;
  memcpy(gen_pitches, p->pitches, 16);
  seq_bpm_val = p->bpm_val;
  seq_density_val = p->density_val;
  seq_map_val = p->map_val;
  currentScale = p->scale;
  currentRoot = p->root;

  for (int i = 0; i < NUM_PAGES; i++) {
    params[i] = p->pages[i];
  }
}

// ── Factory Defaults ────────────────────────────────────────────────────────

static const Preset factory_presets[16] = {
    {// Slot 0: Wooden Marimba (Warm, organic percussive bank)
     PRESET_MAGIC,
     (uint32_t)MODEL_MODAL,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     16384, // BPM
     0,     // Arp Mode: Up Arp
     16384, // Y knob: Loop=8, Syncopated Euclidean
     0,     // Scale: Pentatonic Minor
     0,     // Root: C
     {{32767, 12000, 16384}, // Page 0: Strike (Level, Timbre, Meta)
      {0, 0, 0},             // Page 1: Blow
      {0, 0, 8192},          // Page 2: Bow (Level, Timbre, Shape=AD envelope)
      {5000, 10000, 12000},  // Page 3: Resonator 1 (Geometry, Brightness, Damping)
      {16384, 8000, 12000},  // Page 4: Resonator 2 (Position, Reverb, Room Size)
      {16384, 16384, 24000}},// Page 5: Performance (Pitch Coarse, Fine, Strength)
     {0}},
    {// Slot 1: Celestial Glass (Bright, shimmering glass vibraphone)
     PRESET_MAGIC,
     (uint32_t)MODEL_MODAL,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     12000, // BPM
     21845, // Arp Mode: Random/Generative (seq_density_val / 6 = 4)
     24576, // Y knob: Loop=12, Wild Rolls/Ratchets
     1,     // Scale: Pentatonic Major
     5,     // Root: F
     {{28000, 28000, 20000}, // Page 0: Strike
      {0, 0, 0},             // Page 1: Blow
      {0, 0, 12000},         // Page 2: Bow
      {28000, 24000, 24000}, // Page 3: Resonator 1
      {24000, 22000, 28000}, // Page 4: Resonator 2 (Massive ambient Reverb)
      {16384, 16384, 28000}},// Page 5: Performance
     {0}},
    {// Slot 2: Industrial Metal (Clangy plates, spring echo, stutters)
     PRESET_MAGIC,
     (uint32_t)MODEL_MODAL,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     22000, // BPM
     27300, // Arp Mode: Pattern/Trance Leap (seq_density_val / 6 = 5)
     12000, // Y knob: Loop=8, Syncopated
     4,     // Scale: Chromatic
     2,     // Root: D
     {{32767, 32767, 28000}, // Page 0: Strike
      {8000, 16384, 16384},  // Page 1: Blow (Noisy metallic air)
      {0, 0, 4000},          // Page 2: Bow
      {32767, 16384, 8000},  // Page 3: Resonator 1
      {8000, 6000, 8000},    // Page 4: Resonator 2
      {16384, 16384, 32767}},// Page 5: Performance
     {0}},
    {// Slot 3: Shimmering Harp (Plucked chord string ensemble)
     PRESET_MAGIC,
     (uint32_t)MODEL_CHORDS,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     14000, // BPM
     10922, // Arp Mode: Pendulum (seq_density_val / 6 = 2)
     28000, // Y knob: Loop=16, Level 3 rolls
     3,     // Scale: Major
     0,     // Root: C
     {{28000, 12000, 0},     // Page 0: Strike
      {0, 0, 0},             // Page 1: Blow
      {0, 0, 8192},          // Page 2: Bow
      {16384, 18000, 20000}, // Page 3: Resonator 1
      {28000, 20000, 24000}, // Page 4: Resonator 2
      {16384, 16384, 24000}},// Page 5: Performance
     {0}},
    {// Slot 4: Ambient Wind Flute (Hollow woodwind swells, cathedral reverb)
     PRESET_MAGIC,
     (uint32_t)MODEL_MODAL,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     8000,  // BPM
     16384, // Arp Mode: Pedal Point (seq_density_val / 6 = 3)
     6000,  // Y knob: Loop=4, Syncopated Breath
     2,     // Scale: Natural Minor
     9,     // Root: A
     {{0, 0, 0},             // Page 0: Strike
      {32767, 16000, 12000}, // Page 1: Blow (Continuous rich blowing breath)
      {8000, 16384, 24000},  // Page 2: Bow (Friction bow scrape + slow swell shape)
      {24000, 12000, 16384}, // Page 3: Resonator 1
      {16384, 25000, 30000}, // Page 4: Resonator 2 (Huge wash)
      {16384, 16384, 20000}},// Page 5: Performance
     {0}},
    {// Slot 5: Deep Rubber Bass ( Karplus-Strong low, punchy plucked bass)
     PRESET_MAGIC,
     (uint32_t)MODEL_STRING,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     18000, // BPM
     5461,  // Arp Mode: Down Arp (seq_density_val / 6 = 1)
     2000,  // Y knob: Loop=4, Steady
     0,     // Scale: Pentatonic Minor
     7,     // Root: G
     {{32767, 4000, 0},     // Page 0: Strike (Deep finger pluck)
      {0, 0, 0},             // Page 1: Blow
      {0, 0, 6000},          // Page 2: Bow (Punchy decay shape)
      {4000, 6000, 10000},   // Page 3: Resonator 1 (Thick rubber geometry)
      {8000, 4000, 5000},    // Page 4: Resonator 2 (Tight dry focused mono)
      {4096, 16384, 32767}}, // Page 5: Performance (Pitched 2 octaves down!)
     {0}},
    {// Slot 6: Bowed Violin (Rich woodbody swelling strings)
     PRESET_MAGIC,
     (uint32_t)MODEL_CHORDS,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     10000, // BPM
     0,     // Arp Mode: Up Arp
     14000, // Y knob: Loop=8, Syncopated
     2,     // Scale: Natural Minor
     4,     // Root: E
     {{0, 0, 0},             // Page 0: Strike
      {0, 0, 0},             // Page 1: Blow
      {32767, 28000, 28000}, // Page 2: Bow (Heavy continuous bow swell)
      {20000, 16384, 24000}, // Page 3: Resonator 1
      {28000, 24000, 26000}, // Page 4: Resonator 2 (Wide stereo chorus space)
      {16384, 16384, 28000}},// Page 5: Performance
     {0}},
    {// Slot 7: Hollow Clay Bottle (Hand slap, air puffs and cave echo)
     PRESET_MAGIC,
     (uint32_t)MODEL_MODAL,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     15000, // BPM
     21845, // Arp Mode: Random/Generative (seq_density_val / 6 = 4)
     16000, // Y knob: Loop=8, Level 2
     0,     // Scale: Pentatonic Minor
     11,    // Root: B
     {{24000, 8000, 16384},  // Page 0: Strike (Hand slap over opening)
      {28000, 12000, 16384}, // Page 1: Blow (Air puff across neck)
      {0, 0, 8192},          // Page 2: Bow
      {10000, 8000, 12000},  // Page 3: Resonator 1 (Hollow jug geometry)
      {16384, 16384, 20000}, // Page 4: Resonator 2 (Echoey cave room)
      {16384, 16384, 24000}},// Page 5: Performance
     {0}},
    {// Slot 8: Shimmering Bells (SVF Resonator high crystal chime)
     PRESET_MAGIC,
     (uint32_t)MODEL_MODAL,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     13000, // BPM
     0,     // Arp Mode: Up
     6000,  // Y knob: Loop=4, Syncopated
     1,     // Scale: Pentatonic Major
     7,     // Root: G
     {{32767, 20000, 16384}, // Page 0: Strike
      {0, 0, 0},             // Page 1: Blow
      {0, 0, 8192},          // Page 2: Bow
      {12000, 22000, 26000}, // Page 3: Resonator 1 (High bell sustain)
      {16384, 18000, 20000}, // Page 4: Resonator 2
      {20480, 16384, 24000}},// Page 5: Performance (Pitched 1 octave up!)
     {0}},
    {// Slot 9: Bowed Metal Sheet (Friction scrape on resonant metal SVF)
     PRESET_MAGIC,
     (uint32_t)MODEL_MODAL,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     9000,  // BPM
     21845, // Arp Mode: Random/Generative
     14000, // Y knob: Loop=8
     2,     // Scale: Natural Minor
     2,     // Root: D
     {{0, 0, 0},             // Page 0: Strike
      {4000, 16384, 16384},  // Page 1: Blow (Noisy hiss)
      {32767, 20000, 24000}, // Page 2: Bow (Friction scrape)
      {30000, 14000, 28000}, // Page 3: Resonator 1 (Massive plate sheet)
      {28000, 20000, 24000}, // Page 4: Resonator 2
      {16384, 16384, 28000}},// Page 5: Performance
     {0}},
    {// Slot 10: Organic Wood Block (Super damp, tight woody bar)
     PRESET_MAGIC,
     (uint32_t)MODEL_MODAL,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     17000, // BPM
     27300, // Arp Mode: Trance Leap Motif
     28000, // Y knob: Loop=16, Rolls
     0,     // Scale: Pentatonic Minor
     0,     // Root: C
     {{32767, 12000, 8000},  // Page 0: Strike
      {0, 0, 0},             // Page 1: Blow
      {0, 0, 4000},          // Page 2: Bow
      {2000, 8000, 6000},    // Page 3: Resonator 1 (Extremely short decay)
      {16384, 3000, 4000},   // Page 4: Resonator 2 (Mono focused tight room)
      {16384, 16384, 28000}},// Page 5: Performance
     {0}},
    {// Slot 11: Electric String Pluck (Single string with steel bite)
     PRESET_MAGIC,
     (uint32_t)MODEL_STRING,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     15000, // BPM
     16384, // Arp Mode: Pedal Point
     16000, // Y knob: Loop=8
     3,     // Scale: Major
     5,     // Root: F
     {{32767, 28000, 0},     // Page 0: Strike (Pick pluck)
      {0, 0, 0},             // Page 1: Blow
      {0, 0, 8192},          // Page 2: Bow
      {16384, 24000, 16384}, // Page 3: Resonator 1 (Steel string geometry)
      {16384, 12000, 16000}, // Page 4: Resonator 2
      {16384, 16384, 32767}},// Page 5: Performance
     {0}},
    {// Slot 12: Atmospheric Wind Pad (Bore breathy swells, huge space)
     PRESET_MAGIC,
     (uint32_t)MODEL_MODAL,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     8000,  // BPM
     10922, // Arp Mode: Pendulum
     24000, // Y knob: Loop=12, Ratchets
     2,     // Scale: Natural Minor
     7,     // Root: G
     {{0, 0, 0},             // Page 0: Strike
      {28000, 12000, 16384}, // Page 1: Blow
      {12000, 16384, 30000}, // Page 2: Bow (Wind pad slow envelope shape)
      {18000, 10000, 20000}, // Page 3: Resonator 1
      {28000, 28000, 32000}, // Page 4: Resonator 2 (Oceanic hall wash)
      {16384, 16384, 24000}},// Page 5: Performance
     {0}},
    {// Slot 13: Space Drone (Multi-strings bowed continuously)
     PRESET_MAGIC,
     (uint32_t)MODEL_CHORDS,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     7000,  // BPM
     21845, // Arp Mode: Random/Generative
     6000,  // Y knob: Loop=4
     2,     // Scale: Natural Minor
     0,     // Root: C
     {{0, 0, 0},             // Page 0: Strike
      {0, 0, 0},             // Page 1: Blow
      {32767, 24000, 32767}, // Page 2: Bow (Continuous violin bow scrape drone)
      {28000, 16384, 30000}, // Page 3: Resonator 1 (Infinite string sustain)
      {28000, 26000, 28000}, // Page 4: Resonator 2 (Cathedral space)
      {12288, 16384, 28000}},// Page 5: Performance (Pitched 1/2 octave down)
     {0}},
    {// Slot 14: Toy Glockenspiel (SVF high metallic chime)
     PRESET_MAGIC,
     (uint32_t)MODEL_MODAL,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     14000, // BPM
     0,     // Arp Mode: Up
     16000, // Y knob
     1,     // Scale: Pentatonic Major
     0,     // Root: C
     {{32767, 28000, 20000}, // Page 0: Strike
      {0, 0, 0},             // Page 1: Blow
      {0, 0, 6000},          // Page 2: Bow
      {8000, 28000, 16384},  // Page 3: Resonator 1
      {16384, 16000, 18000}, // Page 4: Resonator 2
      {24576, 16384, 28000}},// Page 5: Performance (Pitched 2 octaves up!)
     {0}},
    {// Slot 15: Flutey Bottle Percussion (Bore percussive puff/slap combo)
     PRESET_MAGIC,
     (uint32_t)MODEL_MODAL,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     16000, // BPM
     27300, // Arp Mode: Trance Leap Motif
     28000, // Y knob: Loop=16, Rolls
     0,     // Scale: Pentatonic Minor
     2,     // Root: D
     {{28000, 10000, 16384}, // Page 0: Strike
      {24000, 12000, 16384}, // Page 1: Blow
      {0, 0, 8192},          // Page 2: Bow
      {6000, 12000, 14000},  // Page 3: Resonator 1
      {16384, 14000, 16000}, // Page 4: Resonator 2
      {16384, 16384, 24000}},// Page 5: Performance
     {0}}};

void init_factory_presets() {
  const Preset *p = (const Preset *)(XIP_BASE + FLASH_PRESET_OFFSET);
  if (p->magic != PRESET_MAGIC) {
    // Flash is empty, write factory defaults
    uint8_t buffer[4096];
    memset(buffer, 0, 4096);
    memcpy(buffer, factory_presets, sizeof(factory_presets));

    GlobalSettings g;
    g.magic = GLOBAL_SETTINGS_MAGIC;
    g.midi_channel = 0; // Default to Omni
    memcpy(buffer + GLOBAL_SETTINGS_OFFSET, &g, sizeof(GlobalSettings));

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_PRESET_OFFSET, 4096);
    flash_range_program(FLASH_PRESET_OFFSET, buffer, 4096);
    restore_interrupts(ints);
  }
}

class Modal : public ComputerCard {
public:
  // ── UI State ────────────────────────────────────────────────────────
  int currentPage = 0;             // Currently active parameter page (0–5)
  KnobLock lockMain, lockX, lockY; // Knob locks for page switching
  int32_t smoothMain = 0, smoothX = 0, smoothY = 0; // Smoothed knob values

  // ── Switch State ────────────────────────────────────────────────────
  uint32_t switchDownTimer = 0; // How long switch has been held down
  bool switchHandled = false;   // Has the current press been handled?
  uint32_t switchUpTimer = 0;
  bool switchHandledUp = false;
  bool switchInit = false;            // Has switch been initialized?
  bool lastSwitchUp = false;          // Previous state of switch-up
  Switch lastSwitch = Switch::Middle; // Debounced switch state
  uint32_t debounceTimer = 0;         // Switch debounce counter

  // ── Gate State ──────────────────────────────────────────────────────
  bool previousGate = false;    // Gate state from last DSP cycle
  bool triggerBuffered = false; // Buffered trigger from Pulse In 1

  // ── Audio State ─────────────────────────────────────────────────────
  int32_t dspOutL = 0, dspOutR = 0;         // Latest output from Core 1
  int gPhase = 0;                           // Phase counter for 2:1 decimation
  uint32_t bootSilence = BOOT_SILENCE_SAMP; // Startup mute countdown

  // ── CV Smoothing ────────────────────────────────────────────────────
  int32_t cv1_acc = 0; // Smoothed CV1 accumulator
  int32_t cv1_fast_acc = 0; // Fast CV1 accumulator for S&H settling
  int sh_settling_timer = 0; // Settling counter (96 samples = 2ms at 48kHz)
  bool sh_pending = false;   // Flag indicating a CV settling cycle is active
  bool delayed_gate = false; // Delayed gate state to align with settled CV
  int32_t last_auto_pitch_q8 = -99999; // Rings-style: last pitch for auto-strum detection
  int32_t cv2_acc = 0; // Smoothed CV2 accumulator

  // ── DC Blockers ─────────────────────────────────────────────────────
  int32_t dc_bxL = 0, dc_byL = 0, dc_bxR = 0, dc_byR = 0; // Input
  int32_t dc_oxL = 0, dc_oyL = 0, dc_oxR = 0, dc_oyR = 0; // Output

  // ── Reverb Parameter Smoothing ───────────────────────────────────
  // Slow IIR smoothing prevents zipper noise when the reverb wet/dry or
  // room size is changed. >> 7 gives ~128-sample (5ms) attack at 24kHz.
  int32_t smooth_reverb_amt = 0;
  int32_t smooth_rev_decay  = 2000;
  int32_t smooth_size_ratio = 6554;

  // ── Page Display Timer ──────────────────────────────────────────────
  uint32_t pageDisplayTimer = 0; // Countdown for page display LED flash
  uint32_t saveDisplayTimer = 0; // Countdown for save confirm LED flash
  static const uint32_t PAGE_DISPLAY_DURATION = 24000; // ~500ms at 48kHz

  // ────────────────────────────────────────────────────────────────────
  //  Constructor — Initialize all parameters to musical defaults
  // ────────────────────────────────────────────────────────────────────

  Modal() {
    // Page 0: Strike (Level, Timbre, Meta) — Mallet, mid timbre, 50% level
    params[0] = {16384, 16384, 16384};

    // Page 1: Bow & Env (Level, Timbre, Shape) — AD envelope, off by default
    params[1] = {0, 16384, 8192};

    // Page 2: Blow (Level, Timbre, Meta) — Off by default
    params[2] = {0, 16384, 16384};

    // Page 3: Resonator Core (Geometry, Brightness, Damping)
    // Matching original Elements defaults: geometry=0.2, brightness=0.5,
    // damping=0.25
    params[3] = {6554, 16384, 8192};

    // Page 4: Resonator Space (Position, Space)
    params[4] = {9830, 8192, 0}; // Position=0.3, Space=0.25 (light reverb)

    // Page 5: Performance (Pitch Coarse, Fine Tune, Strength)
    params[5] = {16384, 16384, 16384};

    // Initialize reverb state
    reverb.Init();

    // Initialize sequencer pitches (Pentatonic Minor)
    for (int i = 0; i < 16; i++) {
      static const uint8_t scale[] = {0, 3, 5, 7, 10};
      gen_pitches[i] = 48 + scale[i % 5];
    }
  }

  // ────────────────────────────────────────────────────────────────────
  //  InitUSB — Initialize TinyUSB based on hardware power state
  // ────────────────────────────────────────────────────────────────────

  void InitUSB() {
    // Wait for USB power state to settle
    sleep_ms(300);
    usb_power_state = USBPowerState();

    // Visual feedback:
    // LED 0 flashes for Host mode, LED 1 for Device mode
    if (usb_power_state == ComputerCard::DFP) {
      isUSBMIDIHost = true;
      LedOn(0);
      tuh_init(TUH_OPT_RHPORT);
    } else {
      isUSBMIDIHost = false;
      LedOn(1);
      tud_init(TUD_OPT_RHPORT);
    }
    sleep_ms(200);
    LedOff(0);
    LedOff(1);
  }

  // ────────────────────────────────────────────────────────────────────
  //  BackgroundLoop — runs on Core 0 when not in ProcessSample ISR
  //  Used for slow UI updates (LED display, flash operations, etc.)
  //  ~5000 iterations per meaningful update (matches grains pattern).
  // ────────────────────────────────────────────────────────────────────

  void BackgroundLoop() override {
    // ── USB / MIDI Tasks ────────────────────────────────────────────
    if (isUSBMIDIHost) {
      tuh_task();
    } else {
      tud_task();
      while (tud_midi_available()) {
        uint8_t packet[4];
        tud_midi_packet_read(packet);
        if (packet[1] >= 0xF8) {
          handle_midi_realtime(packet[1]);
        } else {
          handle_midi_message(packet + 1, 3);
        }
      }

      // ── MIDI Feedback Sync ──────────────────────────────────────
      // Send current knob values back to UI, but debounced and only if changed.
      // Check one page per loop to save CPU.
      static int syncPage = 0;
      static int32_t last_sent_model = -1;
      syncPage = (syncPage + 1) % 6;

      // Sync Model via CC 102
      if (web_ui_connected && currentModel != last_sent_model) {
        last_sent_model = currentModel;
        if (tud_midi_mounted()) {
          uint8_t packet[4] = {0x0B, 0xB0, 102, (uint8_t)currentModel};
          tud_midi_packet_write(packet);
        }
      }

      auto syncKnob = [&](int32_t current, int32_t &last, uint8_t cc) {
        // Only send if the difference is significant (>1 CC step) to avoid
        // jitter
        int32_t diff = (current > last) ? (current - last) : (last - current);
        if (diff > 256) {
          last = current;
          if (web_ui_connected && tud_midi_mounted()) {
            uint8_t packet[4] = {0x0B, 0xB0, cc, (uint8_t)(current >> 8)};
            tud_midi_packet_write(packet);
            last_cc_tx_time[cc & 0x7F] = to_ms_since_boot(get_absolute_time());
          }
        }
      };

      syncKnob(params[syncPage].pMain, last_sent[syncPage].pMain,
               PageCCs[syncPage][0]);
      syncKnob(params[syncPage].pX, last_sent[syncPage].pX,
               PageCCs[syncPage][1]);
      syncKnob(params[syncPage].pY, last_sent[syncPage].pY,
               PageCCs[syncPage][2]);

      // Sync Sequencer Params
      auto syncSeq = [&](int32_t &current, int32_t &last, uint8_t cc) {
        // Apply a 2.5% dead-zone (800 units in Q15)
        if (abs(current - last) > 800) {
          last = current;
          if (web_ui_connected && tud_midi_mounted()) {
            uint8_t packet[4] = {0x0B, 0xB0, cc, (uint8_t)(current >> 8)};
            tud_midi_packet_write(packet);
            last_cc_tx_time[cc & 0x7F] = to_ms_since_boot(get_absolute_time());
          }
        }
      };
      syncSeq(seq_density_val, last_seq_dens, 103);
      syncSeq(seq_bpm_val, last_seq_bpm, 104);
      syncSeq(seq_map_val, last_seq_ent, 105);

      // Sync Sequence Pattern
      static uint32_t last_gen_sequence = 0;
      if (gen_sequence != last_gen_sequence) {
        last_gen_sequence = gen_sequence;
        if (web_ui_connected && tud_midi_mounted()) {
          uint8_t p1[4] = {0x0B, 0xB0, 107, (uint8_t)(gen_sequence & 0x7F)};
          uint8_t p2[4] = {0x0B, 0xB0, 108,
                           (uint8_t)((gen_sequence >> 7) & 0x7F)};
          uint8_t p3[4] = {0x0B, 0xB0, 110,
                           (uint8_t)((gen_sequence >> 14) & 0x03)};
          tud_midi_packet_write(p1);
          tud_midi_packet_write(p2);
          tud_midi_packet_write(p3);
        }
      }

      // ── Generative Sequencer / Arpeggiator Logic ────────────────────
      // Runs every background loop (~1ms) at all times
      static uint32_t last_gen_ms = 0;
      static bool gen_ratchet_active = false;
      uint32_t now = to_ms_since_boot(get_absolute_time());

      static uint32_t last_pulse2_time = 0;
      static bool pulse2_step_trigger = false;
      static bool last_pulse2 = false;

      bool p2 = PulseIn2();
      if (p2 && !last_pulse2) {
        last_pulse2_time = now;
        pulse2_step_trigger = true;
      }
      last_pulse2 = p2;

      int bpm = 15 + ((seq_bpm_val * 165) >> 15); // Slowed down BPM range: 15 to 180 BPM
      uint32_t step_ms = (60000 / bpm) / 4; // 16th notes

      static uint32_t last_step_trigger_ms = 0;
      static uint32_t measured_step_ms = 250;

      bool pulse2_active = (now - last_pulse2_time < 1500);
      bool clock_active = (now - last_midi_clock_ms < 1000);
      bool trigger_step = false;

      if (pulse2_active) {
        if (pulse2_step_trigger) {
          pulse2_step_trigger = false;
          trigger_step = true;
          
          uint32_t diff = now - last_step_trigger_ms;
          if (diff > 10 && diff < 2000) {
            measured_step_ms = diff;
          }
          last_step_trigger_ms = now;
          last_gen_ms = now;
        }
      } else if (clock_active) {
        if (midi_clock_step_trigger) {
          midi_clock_step_trigger = false;
          trigger_step = true;
          
          uint32_t diff = now - last_step_trigger_ms;
          if (diff > 10 && diff < 2000) {
            measured_step_ms = diff;
          }
          last_step_trigger_ms = now;
          last_gen_ms = now; // keep internal clock aligned
        }
      } else {
        if (now - last_gen_ms >= step_ms) {
          trigger_step = true;
          measured_step_ms = step_ms;
          last_step_trigger_ms = now;
          last_gen_ms = now;
        }
      }

      if (trigger_step) {
        // Loop Length determined by seq_map_val (Y knob)
        int32_t loop_len = 16;
        if (seq_map_val < 8192) {
          loop_len = 4;
        } else if (seq_map_val < 16384) {
          loop_len = 8;
        } else if (seq_map_val < 24576) {
          loop_len = 12;
        } else {
          loop_len = 16;
        }

        if (clock_active && midi_clock_reset_pending) {
          midi_clock_reset_pending = false;
          gen_step = -1; // Reset to start of loop
        }

        gen_step = (gen_step + 1) % loop_len;
        gen_pulse_clk = true; // Clock pulse start

        // Sync step to Web UI
        if (web_ui_connected && tud_midi_mounted()) {
          uint8_t packet[4] = {0x0B, 0xB0, 106, (uint8_t)gen_step};
          tud_midi_packet_write(packet);
        }

        // Map to Scale/Root
        static const uint8_t scales[5][12] = {
            {0, 3, 5, 7, 10, 12, 15, 17, 19, 22, 24, 27}, // Pent Minor
            {0, 2, 4, 7, 9, 12, 14, 16, 19, 21, 24, 26},  // Pent Major
            {0, 2, 3, 5, 7, 8, 10, 12, 14, 15, 17, 19},   // Natural Minor
            {0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 17, 19},   // Major
            {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}        // Chromatic
        };
        uint8_t scale_idx = currentScale % 5;

        // Dynamic chord/scale notes (6 musical voicing steps)
        int32_t notes[6];
        notes[0] = scales[scale_idx][0];  // Root
        notes[1] = scales[scale_idx][1];  // 3rd / 2nd
        notes[2] = scales[scale_idx][3];  // 5th
        notes[3] = scales[scale_idx][5];  // Octave
        notes[4] = scales[scale_idx][7];  // 10th / 9th
        notes[5] = scales[scale_idx][10]; // 12th / Double Octave

        // Arp Mode selection based on Main knob (seq_density_val)
        int arp_mode = (seq_density_val * 6) >> 15;
        if (arp_mode > 5) arp_mode = 5;

        int32_t p = notes[0];
        switch (arp_mode) {
          case 0: // Up
            p = notes[gen_step % 6];
            break;
          case 1: // Down
            p = notes[5 - (gen_step % 6)];
            break;
          case 2: // Pendulum (Up-Down)
            {
              int32_t cycle = gen_step % 10;
              p = notes[(cycle < 5) ? cycle : (10 - cycle)];
            }
            break;
          case 3: // Alternating Pedal Point (Root -> Note -> Root -> Note)
            p = (gen_step % 2 == 0) ? notes[0] : notes[1 + ((gen_step >> 1) % 5)];
            break;
          case 4: // Random / Generative
            p = notes[rand() % 6];
            break;
          case 5: // Pattern/Trance Leap Motif
            {
              static const uint8_t tr_pattern[16] = {0, 3, 1, 4, 2, 5, 3, 0, 4, 1, 5, 2, 0, 3, 1, 5};
              p = notes[tr_pattern[gen_step % 16] % 6];
            }
            break;
        }

        int32_t target_note = p + 60; // Base C4

        // Rhythmic density / patterns based on Y knob (seq_map_val)
        int32_t frac_y = seq_map_val % 8192;
        bool gate_on = false;
        gen_ratchet_active = false;

        if (frac_y < 2730) {
          // Level 1: Steady, all gates ON
          gate_on = true;
        } else if (frac_y < 5460) {
          // Level 2: Syncopated Euclidean rhythms
          if (loop_len == 4) {
            static const bool pat4[4] = {true, true, false, true};
            gate_on = pat4[gen_step % 4];
          } else if (loop_len == 8) {
            static const bool pat8[8] = {true, false, true, true, false, true, true, false};
            gate_on = pat8[gen_step % 8];
          } else if (loop_len == 12) {
            static const bool pat12[12] = {true, false, true, false, true, true, false, true, false, true, true, false};
            gate_on = pat12[gen_step % 12];
          } else {
            static const bool pat16[16] = {true, false, false, true, false, false, true, false, false, true, false, false, true, true, false, false};
            gate_on = pat16[gen_step % 16];
          }
        } else {
          // Level 3: Tasteful dynamically scaled rolls (ratchets) and skipped steps
          int32_t intensity = (frac_y - 5460) * 100 / 2732; // 0 to 100%
          int32_t skip_chance = 3 + (intensity * 7) / 100;      // 3% to 10%
          int32_t ratchet_chance = 5 + (intensity * 15) / 100;  // 5% to 20%

          int32_t roll_roll = rand() % 100;
          if (roll_roll < skip_chance) {
            gate_on = false; // Skip step tastefully
          } else if (roll_roll < (skip_chance + ratchet_chance)) {
            // Only ratchet on even steps to keep the groove structured and musical
            if (gen_step % 2 == 0) {
              gate_on = true;
              gen_ratchet_active = true; // Double-trigger ratchet roll!
            } else {
              gate_on = true;
            }
          } else {
            gate_on = true; // Normal step
          }
        }

        gen_sequence = (gen_sequence & ~(1 << gen_step)) | (gate_on ? (1 << gen_step) : 0);

        int32_t octave = (target_note / 12) - 4;
        int32_t note_in_octave = target_note % 12;

        // Find closest note in scale
        int32_t best_dist = 100;
        int32_t best_note = 0;
        for (int i = 0; i < 12; i++) {
          int32_t dist = abs(note_in_octave - scales[scale_idx][i]);
          if (dist < best_dist) {
            best_dist = dist;
            best_note = scales[scale_idx][i];
          }
        }
        gen_pitches[gen_step] = 48 + currentRoot + (octave * 12) + best_note;

        // Trigger Step
        bool stepActive = (gen_sequence & (1 << gen_step));
        if (stepActive) {
          gen_melody_q8 = (gen_pitches[gen_step] - 60) * 256;
          gen_gate_pending = true; // Prepare to fire gate after CV settles
          gen_gate = false;        // Ensure gate is low during CV transition
        } else {
          gen_gate = false;
          gen_gate_pending = false;
        }
      } else if (gen_gate_pending && (now - last_gen_ms >= 2)) {
        // 2ms have passed since CV update — fire the gate
        gen_gate = true;
        gen_gate_pending = false;
      } else if (gen_ratchet_active) {
        // Double-trigger ratchet (elapsed time checkpoints)
        uint32_t elapsed = now - last_gen_ms;
        if (elapsed >= (measured_step_ms * 3) / 4) {
          gen_gate = false;
        } else if (elapsed >= measured_step_ms / 2) {
          gen_gate = true;
        } else if (elapsed >= measured_step_ms / 4) {
          gen_gate = false;
        }
      } else if (gen_gate && (now - last_gen_ms >= (measured_step_ms / 2))) {
        // 50% duty cycle reached — end the gate
        gen_gate = false;
      } else if (now - last_gen_ms > 20) {
        gen_pulse_clk = false; // Short clock pulse
      }
      
      // ── External Sync & Randomization ─────────────────────────────
      static bool last_pulse1 = false;
      bool p1 = PulseIn1();

      if (p1 && !last_pulse1) {
        // Pulse 1 In: Randomize CV 2
        gen_cv2_val = (rand() % 4096) - 2048;
      }
      last_pulse1 = p1;
    }

    static uint32_t loopCount = 0;
    if (++loopCount < 1000)
      return;
    loopCount = 0;

    // ── LED Rendering ───────────────────────────────────────────────
    if (currentUIState == STATE_LOAD_MENU ||
        currentUIState == STATE_SAVE_MENU) {
      // Preset Menu Visuals:
      // LEDs 0-3 (top 4) show slot index in binary (0-15)
      // LED 4 (bottom left) is Load Indicator
      // LED 5 (bottom right) is Save Indicator
      for (int i = 0; i < 6; i++) {
        if (i < 4) {
          // Top 4 show binary slot index (0 to 15)
          bool on = (presetMenuSlot & (1 << i));
          LedOn(i, on);
        } else if (i == 4) {
          // Bottom-Left LED represents LOAD
          if (currentUIState == STATE_LOAD_MENU) {
            LedOn(4);
          } else {
            LedOff(4);
          }
        } else if (i == 5) {
          // Bottom-Right LED represents SAVE
          if (currentUIState == STATE_SAVE_MENU) {
            LedOn(5);
          } else {
            LedOff(5);
          }
        }
      }
    } else if (currentUIState == STATE_GEN_SEQ) {
      // Sequencer View: 6 LEDs representing the 16 steps
      // 1. Background bar shows Density
      int density_leds = (seq_density_val * 6) >> 15;
      // 2. Traveling dot shows 16-step position scaled to 6 LEDs
      int pos_led = (gen_step * 6) / 16;

      for (int i = 0; i < 6; i++) {
        int br = 0;
        if (i < density_leds)
          br = 150; // Dim background for density
        if (i == pos_led)
          br = 4095; // Bright dot for position
        LedBrightness(i, br);
      }
    } else if (saveDisplayTimer > 0) {
      // Save confirmed: flash all LEDs
      for (int i = 0; i < 6; i++) {
        LedOn(i);
      }
    } else if (pageDisplayTimer > 0) {
      // Page just changed — show current page prominently
      static const int page_to_led_map[6] = {0, 2, 4, 1, 3, 5};
      int activeLed = page_to_led_map[currentPage];
      for (int i = 0; i < 6; i++) {
        LedOn(i, i == activeLed);
      }
    } else {
      // Normal Operation: Display current page + Gate pulses
      static const int page_to_led_map[6] = {0, 2, 4, 1, 3, 5};
      int activeLed = page_to_led_map[currentPage];
      for (int i = 0; i < 6; i++) {
        int32_t b = (i == activeLed) ? 1800 : 0;

        // Pulse LED 4 on gate activity
        if (i == 4 && previousGate)
          b += 2000;

        if (b > 4095)
          b = 4095;
        if (b > 0)
          LedBrightness(i, b);
        else
          LedOff(i);
      }
    }
  }

  // ────────────────────────────────────────────────────────────────────
  //  ProcessSample — runs at 48kHz in interrupt context
  //  MUST complete within ~20μs. No allocations, no blocking.
  // ────────────────────────────────────────────────────────────────────

  void __not_in_flash_func(ProcessSample)() override {
    audio1_connected_flag = Connected(ComputerCard::Audio1);
    // ── Boot Silence ────────────────────────────────────────────────
    // Mute outputs for the first second to let hardware settle.
    // Show a chasing LED animation during boot.
    if (bootSilence > 0) {
      bootSilence--;
      AudioOut1(0);
      AudioOut2(0);

      int chaseIdx = (BOOT_SILENCE_SAMP - bootSilence) / 4000;
      for (int i = 0; i < 6; i++) {
        LedOn(i, i == (chaseIdx % 6));
      }
      if (bootSilence == 0) {
        for (int i = 0; i < 6; i++)
          LedOff(i);
      }
      return;
    }

    // ── Poll Triggers ───────────────────────────────────────────────
    // Check pulse inputs at full 48kHz rate to avoid missing triggers.
    if (PulseIn1RisingEdge()) {
      midi_velocity_q15 = 25600; // Default to velocity 100 for physical triggers
      if (!Connected(ComputerCard::Pulse1)) {
        triggerBuffered = true;
      }
    }
    if (midi_trigger) {
      if (!Connected(ComputerCard::Pulse1)) {
        triggerBuffered = true;
      }
    }

    // ── Read Raw Inputs ─────────────────────────────────────────────
    Switch sw = SwitchVal();
    int32_t rawMain = KnobVal(Knob::Main);
    int32_t rawX = KnobVal(Knob::X);
    int32_t rawY = KnobVal(Knob::Y);

    // Scale knobs to Q15 range
    int32_t kMain = knob_to_q15(rawMain);
    int32_t kX = knob_to_q15(rawX);
    int32_t kY = knob_to_q15(rawY);

    // Smooth knobs with heavy IIR for sequencer (>>5) and
    // responsive IIR for normal play (>>3)
    int32_t shift = (currentUIState == STATE_GEN_SEQ) ? 5 : 3;
    smoothMain += (kMain - smoothMain) >> shift;
    smoothX += (kX - smoothX) >> shift;
    smoothY += (kY - smoothY) >> shift;

    // CV1 for V/Oct pitch tracking
    int32_t cv1_raw = Connected(ComputerCard::CV1) ? CVIn1() : 0;
    cv1_acc = cv1_acc - (cv1_acc >> 8) + cv1_raw;
    int32_t cv1_smoothed = cv1_acc >> 8;

    cv1_fast_acc = cv1_fast_acc - (cv1_fast_acc >> 3) + cv1_raw;
    int32_t cv1_fast = cv1_fast_acc >> 3;

    // Sample & Hold logic triggered by Pulse In 1 or MIDI Note On
    static bool last_p1_sh = false;
    bool p1_now = PulseIn1();
    bool midi_trig_local = midi_trigger;
    bool pulse1_connected = Connected(ComputerCard::Pulse1);

    if (pulse1_connected) {
      // S&H and Quantization active
      if (p1_now && !last_p1_sh) {
        sh_settling_timer = 96; // 2ms at 48kHz
        sh_pending = true;
      }

      if (sh_pending) {
        if (sh_settling_timer > 0) {
          sh_settling_timer--;
        }
        if (sh_settling_timer == 0) {
          int32_t raw_pitch = Connected(ComputerCard::CV1) ? (cv1_fast * 9) : 0;
          cv1_pitch_q8 = ((raw_pitch + 128) & ~0xFF) + midi_pitch_q8;
          delayed_gate = true;
          triggerBuffered = true;
          sh_pending = false;
        }
      }

      if (midi_trig_local) {
        int32_t raw_pitch = Connected(ComputerCard::CV1) ? (cv1_fast * 9) : 0;
        cv1_pitch_q8 = ((raw_pitch + 128) & ~0xFF) + midi_pitch_q8;
        triggerBuffered = true;
        midi_trigger = false;
        sh_pending = false; // Cancel any pending physical S&H
      }

      if (!p1_now) {
        delayed_gate = false;
        sh_pending = false;
      }
    } else {
      // Continuous tracking, no quantization
      int32_t raw_pitch = Connected(ComputerCard::CV1) ? (cv1_smoothed * 9) : 0;
      cv1_pitch_q8 = raw_pitch + midi_pitch_q8;
      if (midi_trig_local)
        midi_trigger = false;
      delayed_gate = false;
      sh_pending = false;

      // ── Rings-style auto-strum on V/Oct pitch change ─────────────────
      // When no Gate/Pulse is connected, strum the resonator whenever the
      // pitch changes by ≥ 0.5 semitone. Exactly how Rings works standalone.
      if (Connected(ComputerCard::CV1) && !midi_trig_local) {
        int32_t snapped = (cv1_pitch_q8 + 128) & ~0xFF; // round to semitone
        if (abs(snapped - last_auto_pitch_q8) > 128) {  // 0.5 semitone hysteresis
          last_auto_pitch_q8 = snapped;
          triggerBuffered = true;
        }
      }
    }
    last_p1_sh = p1_now;

    // CV2 for strength/accent modulation
    int32_t cv2_raw = Connected(ComputerCard::CV2) ? CVIn2() : 0;
    cv2_acc = cv2_acc - (cv2_acc >> 4) + cv2_raw;
    cv2_strength = (cv2_acc >> 4) << 3; // Scale 12-bit to ~Q15

    // Buffer external audio inputs for Core 1 (scale 12-bit to Q15)
    audio_in1_q15 = AudioIn1() << 4; // ±2047 → ±32752
    audio_in2_q15 = AudioIn2() << 4;



    // ── Switch Debouncing & Page Navigation ─────────────────────────
    // Debounce the switch with a 500-sample (~10ms) window.
    Switch effectiveSwitch = sw;
    if (sw != lastSwitch) {
      if (++debounceTimer < 500) {
        effectiveSwitch = lastSwitch;
      } else {
        lastSwitch = sw;
        debounceTimer = 0;
      }
    } else {
      debounceTimer = 0;
    }

    // ── Switch Down: Page Cycling & Preset Menu ───────────────────────
    // Tap = next page. Hold (2s) = Preset Menu (X Left = Load, X Right = Save).
    static Preset original_sound_state;
    if (effectiveSwitch == Switch::Down) {
      switchDownTimer++;
      if (switchDownTimer > 30000 && currentUIState == STATE_NORMAL) {
        // Capture active state to original_sound_state
        original_sound_state.model = currentModel;
        original_sound_state.sequence = gen_sequence;
        memcpy(original_sound_state.pitches, gen_pitches, 16);
        original_sound_state.bpm_val = seq_bpm_val;
        original_sound_state.density_val = seq_density_val;
        original_sound_state.map_val = seq_map_val;
        original_sound_state.scale = currentScale;
        original_sound_state.root = currentRoot;
        for (int i = 0; i < NUM_PAGES; i++) {
          original_sound_state.pages[i] = params[i];
        }

        // Set state dynamically
        if (smoothX < 16384) {
          currentUIState = STATE_LOAD_MENU;
          // Immediately preview the selected preset!
          load_preset(presetMenuSlot);
        } else {
          currentUIState = STATE_SAVE_MENU;
        }

        lockMain.relock(smoothMain);
        lockX.relock(smoothX);
        lockY.relock(smoothY);
      }
    } else {
      if (switchDownTimer > 500) {
        if (currentUIState == STATE_LOAD_MENU) {
          // Preset is already loaded/previewed! Just confirm it.
          currentUIState = STATE_NORMAL;
          lockMain.relock(smoothMain);
          lockX.relock(smoothX);
          lockY.relock(smoothY);
        } else if (currentUIState == STATE_SAVE_MENU) {
          // Save the captured original sound state into the selected slot!
          save_preset(presetMenuSlot, &original_sound_state);
          currentUIState = STATE_NORMAL;
          // Restore captured state so they can continue playing it
          currentModel = original_sound_state.model;
          gen_sequence = original_sound_state.sequence;
          memcpy(gen_pitches, original_sound_state.pitches, 16);
          seq_bpm_val = original_sound_state.bpm_val;
          seq_density_val = original_sound_state.density_val;
          seq_map_val = original_sound_state.map_val;
          currentScale = original_sound_state.scale;
          currentRoot = original_sound_state.root;
          for (int i = 0; i < NUM_PAGES; i++) {
            params[i] = original_sound_state.pages[i];
          }
          lockMain.relock(smoothMain);
          lockX.relock(smoothX);
          lockY.relock(smoothY);

          // Visual feedback for save: flash all LEDs without blocking
          saveDisplayTimer = 4800; // ~100ms at 48kHz
        } else if (!switchHandled) {
          if (switchDownTimer < 15000) {
            // Short tap: Next Page
            currentPage = (currentPage + 1) % NUM_PAGES;
          } else if (switchDownTimer < 30000) {
            // Long tap: Previous Page
            currentPage = (currentPage + NUM_PAGES - 1) % NUM_PAGES;
          }
          pageDisplayTimer = PAGE_DISPLAY_DURATION;
          lockMain.relock(smoothMain);
          lockX.relock(smoothX);
          lockY.relock(smoothY);
        }
        switchHandled = true;
      }
      if (effectiveSwitch == Switch::Middle) {
        switchDownTimer = 0;
        switchHandled = false;
      }
    }

    // ── Switch Up: Model Toggle & Generative Menu ───────────────────
    // Tap = cycle models. Hold (2s) = Generative Sequencer Menu.
    if (effectiveSwitch == Switch::Up) {
      switchUpTimer++;
      if (switchUpTimer > 30000 && currentUIState == STATE_NORMAL) {
        currentUIState = STATE_GEN_SEQ;
        lockMain.relock(smoothMain);
        lockX.relock(smoothX);
        lockY.relock(smoothY);
      }
    } else {
      if (switchUpTimer > 500) {
        if (currentUIState == STATE_GEN_SEQ) {
          currentUIState = STATE_NORMAL;
          lockMain.relock(smoothMain);
          lockX.relock(smoothX);
          lockY.relock(smoothY);
        } else if (!switchHandledUp) {
          currentModel = (currentModel + 1) % MODEL_COUNT;
        }
        switchHandledUp = true;
      }
      if (effectiveSwitch == Switch::Middle) {
        switchUpTimer = 0;
        switchHandledUp = false;
      }
    }

    // ── Generative Sequencer / Preset Menu Controls ──────────────────
    if (currentUIState == STATE_GEN_SEQ) {
      if (lockMain.update(smoothMain))
        seq_density_val = smoothMain;
      if (lockX.update(smoothX))
        seq_bpm_val = smoothX;
      if (lockY.update(smoothY))
        seq_map_val = smoothY;
    } else if (currentUIState == STATE_LOAD_MENU ||
               currentUIState == STATE_SAVE_MENU) {
      // Dynamic LOAD/SAVE toggling based on X knob while in the menu
      if (smoothX < 16384 && currentUIState == STATE_SAVE_MENU) {
        currentUIState = STATE_LOAD_MENU;
        // Load preview
        load_preset(presetMenuSlot);
        lockMain.relock(smoothMain);
        lockX.relock(smoothX);
        lockY.relock(smoothY);
      } else if (smoothX >= 16384 && currentUIState == STATE_LOAD_MENU) {
        currentUIState = STATE_SAVE_MENU;
        // Restore original state so they don't overwrite with preset preview
        currentModel = original_sound_state.model;
        gen_sequence = original_sound_state.sequence;
        memcpy(gen_pitches, original_sound_state.pitches, 16);
        seq_bpm_val = original_sound_state.bpm_val;
        seq_density_val = original_sound_state.density_val;
        seq_map_val = original_sound_state.map_val;
        currentScale = original_sound_state.scale;
        currentRoot = original_sound_state.root;
        for (int i = 0; i < NUM_PAGES; i++) {
          params[i] = original_sound_state.pages[i];
        }
        lockMain.relock(smoothMain);
        lockX.relock(smoothX);
        lockY.relock(smoothY);
      }

      // Use Main knob to select slot 0-15
      int32_t newSlot = (smoothMain >> 11); // Q15 -> 0..15
      if (newSlot > 15)
        newSlot = 15;
      if (newSlot < 0)
        newSlot = 0;

      if (newSlot != presetMenuSlot) {
        presetMenuSlot = newSlot;
        if (currentUIState == STATE_LOAD_MENU) {
          load_preset(presetMenuSlot);
          lockMain.relock(smoothMain);
          lockX.relock(smoothX);
          lockY.relock(smoothY);
        }
      }
    }

    // Pulse In 2 functionality moved to Sequencer Sync

    // ── Page Display Timer ──────────────────────────────────────────
    if (pageDisplayTimer > 0)
      pageDisplayTimer--;
    if (saveDisplayTimer > 0)
      saveDisplayTimer--;

    // Only update parameters if we are in normal operation mode
    if (currentUIState == STATE_NORMAL) {
      if (lockMain.update(smoothMain))
        params[currentPage].pMain = smoothMain;
      if (lockX.update(smoothX))
        params[currentPage].pX = smoothX;
      if (lockY.update(smoothY))
        params[currentPage].pY = smoothY;
    }

    // ── DSP Engine Communication (24kHz) ────────────────────────────
    // Every 2nd sample, exchange data with Core 1 via multicore FIFO.
    // This effectively runs the DSP engine at 24kHz while maintaining
    // the 48kHz sample rate for input polling and output.

    if (++gPhase >= 2) {
      gPhase = 0;

      // Build gate flags
      uint32_t flags = FIFO_FLAG_ACTIVE;
      bool gateNow = (pulse1_connected ? delayed_gate : PulseIn1()) || triggerBuffered || midi_gate;

      if (gateNow)
        flags |= FIFO_FLAG_GATE;
      if (gateNow && !previousGate)
        flags |= FIFO_FLAG_RISING;
      if (!gateNow && previousGate)
        flags |= FIFO_FLAG_FALLING;

      previousGate = gateNow;
      triggerBuffered = false;

      // 1. Receive processed audio from Core 1 (from the PREVIOUS 24kHz period)
      // By popping before pushing, we don't force Core 1 to finish within
      // this 20.8us ISR. It gets a full 41.6us to compute. If it's not ready,
      // we reuse the old output (preventing a hard drop).
      if (multicore_fifo_rvalid()) {
        int32_t center = (int32_t)multicore_fifo_pop_blocking();
        int32_t sides = (int32_t)multicore_fifo_pop_blocking();

        // ── Stereo Widener (Mid-Side Spread) ────────────────────
        // Space parameter (Page 4 X) controls the stereo width.
        int32_t space = params[4].pX;
        int32_t space_adj = (space > 3277) ? (space - 3277) : 0;
        int32_t spread = space_adj;
        if (spread > 22938)
          spread = 22938; // Max spread 0.7

        int32_t side_signal = mul_q15(sides, spread);
        int32_t outL = center + side_signal;
        int32_t outR = center - side_signal;

        // ── Soft Limiter ────────────────────────────────────────
        // Applied before reverb to prevent harsh resonance peaks.
        outL = SoftLimitQ15(outL);
        outR = SoftLimitQ15(outR);

        // ── Stereo Delay / Reverb ───────────────────────────────
        // Reverb parameter (Page 4 Y) controls mix level, room decay, and size.
        // All three are smoothed via IIR (>> 7, ~5ms) before passing to the reverb
        // engine, eliminating zipper noise when turning the reverb knob.
        int32_t reverb_amt = params[4].pY;
        int32_t rev_decay  = 2000 + mul_q15(reverb_amt, 30440);
        int32_t rev_damp   = 16384;
        int32_t size_ratio = 6554 + mul_q15(reverb_amt, 26214); // 0.2..1.0

        smooth_reverb_amt += (reverb_amt - smooth_reverb_amt) >> 7;
        smooth_rev_decay  += (rev_decay  - smooth_rev_decay)  >> 7;
        smooth_size_ratio += (size_ratio - smooth_size_ratio) >> 7;

        reverb.Process(outL, outR, smooth_reverb_amt, smooth_rev_decay, rev_damp, smooth_size_ratio);

        // ── Soft Clip ───────────────────────────────────────────
        outL = SoftClipQ15(outL);
        outR = SoftClipQ15(outR);

        // ── DC Block ────────────────────────────────────────────
        dspOutL = dc_block(outL, dc_oxL, dc_oyL);
        dspOutR = dc_block(outR, dc_oxR, dc_oyR);
      }

      // 2. Send work to Core 1 to start processing the NEXT period
      multicore_fifo_push_blocking(flags);
    }

    // ── Audio Output ────────────────────────────────────────────────
    // Scale from Q15 (±32767) to 12-bit DAC range (±2047) and clamp.

    int32_t outL = dspOutL >> 4;
    int32_t outR = dspOutR >> 4;

    if (outL > 2047)
      outL = 2047;
    if (outL < -2048)
      outL = -2048;
    if (outR > 2047)
      outR = 2047;
    if (outR < -2048)
      outR = -2048;

    AudioOut1((int16_t)outL);
    AudioOut2((int16_t)outR);

    // ── CV Outputs ──────────────────────────────────────────────────
    // Always output the current melody/mod, but they only update
    // in the background loop (with a 2ms lead time before gen_gate).
    int32_t melody_dac = (gen_melody_q8 * 410) / 3072;
    CVOut1(melody_dac);
    CVOut2(gen_cv2_val);

    // ── Pulse Outputs ───────────────────────────────────────────────
    PulseOut1(gen_gate);
    PulseOut2(gen_pulse_clk);
  }
};

int main() {
  // ── System Overclock ────────────────────────────────────────────────
  // 240MHz for maximum DSP headroom
  vreg_set_voltage(VREG_VOLTAGE_1_25);
  sleep_ms(10);
  set_sys_clock_khz(240000, true);

  // Create the application
  Modal *modal = new Modal();
  modal->EnableNormalisationProbe();

  // ── USB / MIDI Initialization ───────────────────────────────────────
  modal->InitUSB();

  // Initialize factory presets if flash is empty
  init_factory_presets();
  
  // Load global settings
  load_global_settings();

  // Launch Core 1 DSP engine
  multicore_launch_core1(core1_dsp_loop);

  // Run the main application (never returns)
  modal->Run();
}

// ── MIDI Callback Implementations ───────────────────────────────────────────

extern "C" {
void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep,
                       uint8_t num_cables_rx, uint16_t num_cables_tx) {
  (void)in_ep;
  (void)out_ep;
  (void)num_cables_rx;
  (void)num_cables_tx;
  if (midi_dev_addr == 0)
    midi_dev_addr = dev_addr;
}

void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance) {
  (void)instance;
  if (dev_addr == midi_dev_addr)
    midi_dev_addr = 0;
}

void handle_midi_message(uint8_t *packet, int size) {
  if (size < 3)
    return;
  uint8_t status = packet[0];
  uint8_t type = status & 0xF0;

  if (runtime_midi_channel > 0 && status < 0xF0) {
    bool is_note = (type == 0x90 || type == 0x80);
    if (is_note && (status & 0x0F) != (runtime_midi_channel - 1)) {
      return;
    }
    if (type == 0xB0) {
      uint8_t cc = packet[1];
      bool is_page_or_sys_cc = false;
      if (cc >= 102 && cc <= 119) {
        is_page_or_sys_cc = true;
      } else {
        for (int p = 0; p < NUM_PAGES; p++) {
          for (int k = 0; k < 3; k++) {
            if (PageCCs[p][k] == cc) {
              is_page_or_sys_cc = true;
              break;
            }
          }
          if (is_page_or_sys_cc) break;
        }
      }
      if ((status & 0x0F) != (runtime_midi_channel - 1)) {
        if (!((status & 0x0F) == 0 && is_page_or_sys_cc)) {
          return;
        }
      }
    }
  }

  Modal *app = (Modal *)Modal::ThisPtr();

  if (type == 0x90 && packet[2] > 0) { // Note On
    midi_pitch_q8 = (packet[1] - 60) * 256;
    midi_velocity_q15 = (packet[2] << 8); // Scale 0-127 to 0-32512
    midi_gate = true;
    midi_trigger = true;
    printf("MIDI Note On: %d\n", packet[1]);
  } else if (type == 0x80 || (type == 0x90 && packet[2] == 0)) { // Note Off
    if (midi_pitch_q8 == (packet[1] - 60) * 256) {
      midi_gate = false;
    }
  } else if (type == 0xB0) { // CC
    uint8_t cc = packet[1];
    int32_t val = packet[2] << 8;

    // Hardware priority: ignore incoming CCs for 500ms after we just sent one
    // to prevent MIDI echo loops from fighting the physical knobs.
    if (to_ms_since_boot(get_absolute_time()) - last_cc_tx_time[cc & 0x7F] <
        500) {
      return;
    }

    int targetPage = -1;
    int targetKnob = -1;

    // Scan PageCCs dynamically to map knob control CCs
    for (int p = 0; p < NUM_PAGES; p++) {
      for (int k = 0; k < 3; k++) {
        if (PageCCs[p][k] == cc) {
          targetPage = p;
          targetKnob = k;
          break;
        }
      }
      if (targetPage >= 0) break;
    }

    switch (cc) {
    case 102: // Model Switch
      currentModel = packet[2] % MODEL_COUNT;
      printf("Model Change: %ld\n", currentModel);
      break;

    // ── Remote Preset Management ─────────────────────────────
    case 114: // Set MIDI Channel
      runtime_midi_channel = packet[2] % 17;
      save_global_settings();
      break;
    case 118: // Save to Slot X
      save_preset(packet[2] % PRESET_COUNT);
      break;
    case 119: // Load from Slot X
      load_preset(packet[2] % PRESET_COUNT);
      app->lockMain.relock(app->smoothMain);
      app->lockX.relock(app->smoothX);
      app->lockY.relock(app->smoothY);
      if (tud_midi_mounted()) {
        // Sync current sequence pattern
        uint8_t p1[4] = {0x0B, 0xB0, 107, (uint8_t)(gen_sequence & 0x7F)};
        uint8_t p2[4] = {0x0B, 0xB0, 108,
                         (uint8_t)((gen_sequence >> 7) & 0x7F)};
        uint8_t p3[4] = {0x0B, 0xB0, 110,
                         (uint8_t)((gen_sequence >> 14) & 0x03)};
        tud_midi_packet_write(p1);
        tud_midi_packet_write(p2);
        tud_midi_packet_write(p3);
        // Also sync Scale/Root
        uint8_t p4[4] = {0x0B, 0xB0, 112, (uint8_t)currentRoot};
        uint8_t p5[4] = {0x0B, 0xB0, 113, (uint8_t)currentScale};
        tud_midi_packet_write(p4);
        tud_midi_packet_write(p5);

        // Sync Model
        uint8_t pm[4] = {0x0B, 0xB0, 102, (uint8_t)currentModel};
        tud_midi_packet_write(pm);

        // Sync Sequencer Params
        uint8_t sd[4] = {0x0B, 0xB0, 103, (uint8_t)(seq_density_val >> 8)};
        uint8_t sb[4] = {0x0B, 0xB0, 104, (uint8_t)(seq_bpm_val >> 8)};
        uint8_t sm[4] = {0x0B, 0xB0, 105, (uint8_t)(seq_map_val >> 8)};
        tud_midi_packet_write(sd);
        tud_midi_packet_write(sb);
        tud_midi_packet_write(sm);

        // Sync all parameter pages
        for (int page = 0; page < NUM_PAGES; page++) {
          uint8_t ccM = PageCCs[page][0];
          uint8_t ccX = PageCCs[page][1];
          uint8_t ccY = PageCCs[page][2];
          uint8_t pkt1[4] = {0x0B, 0xB0, ccM,
                             (uint8_t)(params[page].pMain >> 8)};
          uint8_t pkt2[4] = {0x0B, 0xB0, ccX, (uint8_t)(params[page].pX >> 8)};
          uint8_t pkt3[4] = {0x0B, 0xB0, ccY, (uint8_t)(params[page].pY >> 8)};
          tud_midi_packet_write(pkt1);
          tud_midi_packet_write(pkt2);
          tud_midi_packet_write(pkt3);
        }
      }
      break;
    case 103:
      if (packet[2] != (last_seq_dens >> 8))
        seq_density_val = (packet[2] << 8);
      break;
    case 104:
      if (packet[2] != (last_seq_bpm >> 8))
        seq_bpm_val = (packet[2] << 8);
      break;
    case 105:
      if (packet[2] != (last_seq_ent >> 8))
        seq_map_val = (packet[2] << 8);
      break;

    case 109: // Toggle Step
      gen_sequence ^= (1 << (packet[2] % 16));
      break;

    case 111: // Request Pattern & Parameter Sync
      web_ui_connected = true;
      if (tud_midi_mounted()) {
        // Sync current sequence pattern
        uint8_t p1[4] = {0x0B, 0xB0, 107, (uint8_t)(gen_sequence & 0x7F)};
        uint8_t p2[4] = {0x0B, 0xB0, 108,
                         (uint8_t)((gen_sequence >> 7) & 0x7F)};
        uint8_t p3[4] = {0x0B, 0xB0, 110,
                         (uint8_t)((gen_sequence >> 14) & 0x03)};
        tud_midi_packet_write(p1);
        tud_midi_packet_write(p2);
        tud_midi_packet_write(p3);
        
        // Sync MIDI Channel
        uint8_t pc[4] = {0x0B, 0xB0, 114, runtime_midi_channel};
        tud_midi_packet_write(pc);

        // Also sync Scale/Root
        uint8_t p4[4] = {0x0B, 0xB0, 112, currentRoot};
        uint8_t p5[4] = {0x0B, 0xB0, 113, currentScale};
        tud_midi_packet_write(p4);
        tud_midi_packet_write(p5);

        // Sync Model
        uint8_t pm[4] = {0x0B, 0xB0, 102, (uint8_t)currentModel};
        tud_midi_packet_write(pm);

        // Sync Sequencer Params
        uint8_t sd[4] = {0x0B, 0xB0, 103, (uint8_t)(seq_density_val >> 8)};
        uint8_t sb[4] = {0x0B, 0xB0, 104, (uint8_t)(seq_bpm_val >> 8)};
        uint8_t sm[4] = {0x0B, 0xB0, 105, (uint8_t)(seq_map_val >> 8)};
        tud_midi_packet_write(sd);
        tud_midi_packet_write(sb);
        tud_midi_packet_write(sm);

        // Sync all parameter pages
        for (int page = 0; page < NUM_PAGES; page++) {
          uint8_t ccM = PageCCs[page][0];
          uint8_t ccX = PageCCs[page][1];
          uint8_t ccY = PageCCs[page][2];
          uint8_t pkt1[4] = {0x0B, 0xB0, ccM,
                             (uint8_t)(params[page].pMain >> 8)};
          uint8_t pkt2[4] = {0x0B, 0xB0, ccX, (uint8_t)(params[page].pX >> 8)};
          uint8_t pkt3[4] = {0x0B, 0xB0, ccY, (uint8_t)(params[page].pY >> 8)};
          tud_midi_packet_write(pkt1);
          tud_midi_packet_write(pkt2);
          tud_midi_packet_write(pkt3);
        }
      }
      break;

    case 112:
      currentRoot = packet[2] % 12;
      break;
    case 113:
      currentScale = packet[2] % 5;
      break;
    }

    if (targetPage != -1) {
      // Ignore the CC if it's identical to the one we just sent out (MIDI
      // echo/loopback prevention). This prevents the hardware knobs from
      // snapping/locking when turning them while connected to Web UI.
      uint8_t last_cc = 0;
      if (targetKnob == 0)
        last_cc = last_sent[targetPage].pMain >> 8;
      else if (targetKnob == 1)
        last_cc = last_sent[targetPage].pX >> 8;
      else if (targetKnob == 2)
        last_cc = last_sent[targetPage].pY >> 8;

      if (packet[2] != last_cc) {
        if (targetKnob == 0) {
          params[targetPage].pMain = val;
          last_sent[targetPage].pMain = val;
        } else if (targetKnob == 1) {
          params[targetPage].pX = val;
          last_sent[targetPage].pX = val;
        } else if (targetKnob == 2) {
          params[targetPage].pY = val;
          last_sent[targetPage].pY = val;
        }

        if (app && targetPage == app->currentPage) {
          if (targetKnob == 0)
            app->lockMain.relock(app->smoothMain);
          else if (targetKnob == 1)
            app->lockX.relock(app->smoothX);
          else if (targetKnob == 2)
            app->lockY.relock(app->smoothY);
        }
      }
    }
  }
}

void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets) {
  if (midi_dev_addr != dev_addr || num_packets == 0)
    return;
  uint8_t cable_num;
  uint8_t buffer[48];
  while (true) {
    int32_t bytesRead =
        tuh_midi_stream_read(dev_addr, &cable_num, buffer, sizeof(buffer));
    if (bytesRead <= 0)
      break;
    for (int i = 0; i < bytesRead; i++) {
      parse_host_midi_byte(buffer[i]);
    }
  }
}

void tuh_midi_tx_cb(uint8_t dev_addr) { (void)dev_addr; }

// USB Device callbacks to automatically reset web_ui_connected on disconnect
void tud_mount_cb(void) {}

void tud_umount_cb(void) {
  web_ui_connected = false;
}

void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
  web_ui_connected = false;
}
}
