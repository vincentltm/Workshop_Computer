// Enable 48kHz audio sample rate in ComputerCard. This must be defined before
// including the header-only ComputerCard implementation.
#define COMPUTERCARD_SAMPLE_RATE_DIV 1
#define SAMPLE_PLAYER_RATE 48000u

#include "ComputerCard.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// USB MIDI Headers
#include "plate_reverb_q15.h"
#include "tusb.h"
#include "usb_midi_host.h"
#include "pico/util/queue.h"

struct AudioFrame {
  int32_t L;
  int32_t R;
};

static queue_t core1_to_core0_queue;
static queue_t core0_to_core1_queue;

static int8_t active_transpositions[16][128];

struct SequencerRatchet {
  bool active;
  uint8_t note;
  uint8_t velocity;
  uint16_t bank;
  uint8_t preset;
  uint8_t channel;
  uint8_t remaining_repeats;
  uint8_t repeat_rate_ticks;
  uint8_t tick_counter;
};
static SequencerRatchet sequencer_ratchets[8];

// SoundFont 2 memory-mapped base address (1MB offset in flash)
#define SF2_BASE_ADDRESS 0x10100000
#ifndef ROMPLER_SF2_MAX_BYTES
#define ROMPLER_SF2_MAX_BYTES (1 * 1024 * 1024)
#endif
// Default 2MB Pico flash layout: firmware in the first 1MB, SF2 data in the
// second. A larger RIFF size here can be stale data from a 16MB image and must
// not be scanned on 2MB cards, or boot can hang before USB/LED initialization
// finishes.
#define SF2_MAX_BYTES ROMPLER_SF2_MAX_BYTES
#define SF2_PARSE_WATCHDOG_MAGIC 0x53463250u
#define AUDIO_EVT_NOTE_ON 0x01u
#define AUDIO_EVT_NOTE_OFF 0x02u
#define AUDIO_EVT_ALL_NOTES_OFF 0x03u

// Packed SoundFont 2 Structures
struct sfPresetHeader {
  char achPresetName[20];
  uint16_t wPreset;
  uint16_t wBank;
  uint16_t wPresetBagNDX;
  uint32_t dwLibrary;
  uint32_t dwGenre;
  uint32_t dwMorphology;
} __attribute__((packed));

struct sfBag {
  uint16_t wGenNDX;
  uint16_t wModNDX;
} __attribute__((packed));

struct sfGenList {
  uint16_t sfGenOper;
  uint16_t genAmount;
} __attribute__((packed));

struct sfInst {
  char achInstName[20];
  uint16_t wInstBagNDX;
} __attribute__((packed));

struct sfSample {
  char achSampleName[20];
  uint32_t dwStart;
  uint32_t dwEnd;
  uint32_t dwStartloop;
  uint32_t dwEndloop;
  uint32_t dwSampleRate;
  uint8_t byOriginalPitch;
  int8_t chPitchCorrection;
  uint16_t wSampleLink;
  uint16_t sfSampleType;
} __attribute__((packed));

// Lightweight Indexes in Flash (Packed)
struct __attribute__((packed)) PresetsIndex {
  uint16_t bank;
  uint16_t preset;
  char name[20];
  uint16_t bag_start;
  uint16_t bag_end;
};

struct __attribute__((packed)) SamplesIndex {
  const int16_t *sample_data;
  uint32_t start;
  uint32_t end;
  uint32_t loop_start;
  uint32_t loop_end;
  uint32_t sample_rate;
  uint8_t original_pitch;
  int8_t pitch_correction;
  uint8_t pad[2]; // Pad to 28 bytes for 32-bit alignment
};

// Global Parser References
const uint8_t *sf2_base = (const uint8_t *)SF2_BASE_ADDRESS;
const uint8_t *smpl_data_ptr = nullptr;
const sfPresetHeader *phdr_ptr = nullptr;
uint32_t phdr_count = 0;
const sfBag *pbag_ptr = nullptr;
uint32_t pbag_count = 0;
const sfGenList *pgen_ptr = nullptr;
uint32_t pgen_count = 0;
const sfInst *inst_ptr = nullptr;
uint32_t inst_count = 0;
const sfBag *ibag_ptr = nullptr;
uint32_t ibag_count = 0;
const sfGenList *igen_ptr = nullptr;
uint32_t igen_count = 0;
const sfSample *shdr_ptr = nullptr;
uint32_t shdr_count = 0;
uint32_t smpl_sample_count = 0;

const PresetsIndex *presets_index = nullptr;
uint32_t indexed_presets_count = 0;
bool isUSBMIDIHost = false;
void tick_midi_sequencer();

const SamplesIndex *samples_index = nullptr;
uint32_t indexed_samples_count = 0;
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
      if (d < 0)
        d = -d;
      if (d > 200) { // ~5% of 12-bit range
        locked = false;
      }
    }
    return !locked;
  }

  void relock(int32_t v) {
    locked = true;
    ref = v;
  }
};

// Global Reverb Instance (lightweight Schroeder — ~4KB RAM, fast enough to not overrun Core 1)
PlateReverbQ15 reverb;

// Synthesis Parameters (Q15 range 0..32767)
volatile int32_t param_pitch = 16384; // Default 0 offset (middle)
volatile int32_t param_cutoff =
    16384;                         // Default center (bypass zone, no LP or HP)
volatile int32_t param_res = 0;    // Default no resonance / vibrato
volatile int32_t param_attack = 0; // Default fast attack
volatile int32_t param_decay = 24576;   // Default medium decay
volatile int32_t param_release = 24576; // Default medium release
volatile int32_t param_reverb_mix = 0;  // Default dry

volatile int32_t page_cutoff = 10000;
volatile int32_t page_res = 0;
volatile int32_t page_attack = -12000;
volatile int32_t page_decay = -4500;
volatile int32_t page_release = -4500;
volatile int32_t page_coarse = 0;
volatile int32_t page_fine = 0;
volatile int32_t page_reverb_mix = 0;
volatile int32_t global_env_attack_step = 1431655744;
volatile int32_t global_env_release_step = 19346688;
volatile int32_t global_drum_env_release_step = 19346688;
volatile int32_t midi_mod_wheel = 0;
struct ChannelRouting {
  int32_t target;              // 0 = Synth 1 (Audio 1), 1 = Synth 2 / Drums (Audio 2), 2 = CV1/Pulse1, 3 = CV2/Pulse2, 4 = Off, 5 = Synth 1 + CV1, 6 = Synth 2 + CV2
  int16_t instrument_override; // -1 = No override, >=0 = preset index override in presets_index
  int16_t volume_trim;         // -12 to +6 (decibels), default 0
};

struct PerformanceConfig {
  uint32_t stereo_mode;        // 0 = Stereo, 1 = Dual Mono
  uint32_t cv1_mode;           // 0 = Gate, 1 = Trigger
  uint32_t cv2_mode;           // 0 = Gate, 1 = Trigger
  uint32_t loop_mode;          // 0 = Loop, 1 = Play Once & Advance
  ChannelRouting channels[16]; // Routing per MIDI channel (0..15)
  uint8_t cc_maps[8];          // Custom MIDI CC maps: 0=Cutoff, 1=ReverbMix, 2=Res/Glitch, 3=Attack, 4=Decay/Release, 5..7=Reserved
};

extern volatile PerformanceConfig global_config;
volatile PerformanceConfig global_config;
volatile PerformanceConfig normal_mode_config;

// Monophonic CV/Gate Out Note Trackers
struct CVGateTracker {
  uint8_t notes[8];
  uint8_t count = 0;

  void push(uint8_t note) {
    remove(note);
    if (count < 8) {
      notes[count++] = note;
    }
  }

  void remove(uint8_t note) {
    for (int i = 0; i < count; i++) {
      if (notes[i] == note) {
        for (int j = i; j < count - 1; j++) {
          notes[j] = notes[j + 1];
        }
        count--;
        break;
      }
    }
  }

  void clear() {
    count = 0;
  }

  int32_t get_active_note() const {
    if (count > 0) {
      return notes[count - 1];
    }
    return -1;
  }
};

extern CVGateTracker cv1_tracker;
extern CVGateTracker cv2_tracker;
CVGateTracker cv1_tracker;
CVGateTracker cv2_tracker;

volatile int32_t cv1_out_note = -1;
volatile uint8_t cv1_out_channel = 0;
volatile bool cv1_out_gate = false;
volatile int32_t cv2_out_note = -1;
volatile uint8_t cv2_out_channel = 0;
volatile bool cv2_out_gate = false;

// Performance parameters
volatile int32_t param_ducking = 16384;             // Default 50% sidechain ducking depth
volatile int32_t param_velocity_scaling = 100; // Knob Y in Switch Middle: 0..200 (100 is unity)
volatile int32_t param_mutate = 0;             // Knob Y in Switch Down + CV2: 0..32767
volatile int32_t param_drum_release = 24576;    // Default 24576 (~650ms decay)
volatile int32_t param_freeze_detune = 0;        // Freeze Y knob: loop glitch probability (0..32767, unipolar)
volatile int32_t stutter_volume = 32767;        // Dynamic volume of the stutter loop (0..32767)
volatile int32_t selected_pattern_knob_idx = 0; // Tracks the pattern selected by knob on Down page
volatile bool clear_voices_request = false;     // Trigger to instantly silence all voices on Core 1

// Beat loop freeze variables
volatile bool freeze_active = false;
volatile uint32_t midi_beat_start_tick = 0;

struct TrackBeatState {
  const uint8_t *ptr = nullptr;
  uint32_t ticks_to_next = 0;
  uint8_t running_status = 0;
  bool active = false;
};
extern TrackBeatState midi_track_beat_states[32];
TrackBeatState midi_track_beat_states[32];
static TrackBeatState midi_track_freeze_states[32];
static uint32_t midi_freeze_start_tick = 0;
static TrackBeatState midi_track_bar_states[32];
static uint32_t midi_bar_start_tick = 0;
volatile int32_t glitch_stutter_progress_q15 = 0;

volatile int32_t param_glitch_mangle = 0;  // Y knob:    mangle intensity / mode selection
volatile int32_t param_stutter = 2048;      // Main knob: stutter speed / division (0..4095)
volatile int32_t param_scrub = 0;           // X knob:    scrub position (0..4095)

volatile bool pulse1_trigger_occurred = false;
volatile bool pulse2_trigger_occurred = false;
volatile uint32_t midi_next_boundary_tick = 0;
volatile uint64_t midi_last_pulse1_time = 0;
volatile bool mclk_trigger_occurred = false;
volatile uint64_t last_midi_clock_time_us = 0;
volatile int32_t glitch_speed_factor_q15 = 32768;

// Beat boundary pattern switching
volatile int32_t pending_midi_idx = 0;

// Internal sidechain envelope (driven by channel 9 drum triggers)
volatile int32_t internal_sc_env = 0;

volatile bool midi_playback_mode = false;
volatile int32_t midi_transpose = 0;
volatile uint32_t boot_sample_counter = 48000;

// Debounced connection state to prevent ADC normalisation probe phase jitter from causing false triggers
volatile bool debounced_connected[6] = {false, false, false, false, false, false};
static int32_t connection_debounce_counter[6] = {0, 0, 0, 0, 0, 0};
volatile uint32_t channel_bend_mult_q16[16] = {
  65536, 65536, 65536, 65536, 65536, 65536, 65536, 65536,
  65536, 65536, 65536, 65536, 65536, 65536, 65536, 65536
};

static const uint32_t db_to_gain_q15[19] = {
  8230,  // -12 dB
  9235,  // -11 dB
  10362, // -10 dB
  11627, // -9 dB
  13045, // -8 dB
  14636, // -7 dB
  16422, // -6 dB
  18426, // -5 dB
  20675, // -4 dB
  23197, // -3 dB
  26029, // -2 dB
  29205, // -1 dB
  32768, //  0 dB (Index 12)
  36766, // +1 dB
  41252, // +2 dB
  46287, // +3 dB
  51933, // +4 dB
  58269, // +5 dB
  65381  // +6 dB
};


// Multi-timbral Channel Presets & Banks
volatile uint16_t channel_banks[16];
volatile uint16_t channel_presets[16];

// Mode and Selection State (Switch Down Knob Y Mapping)
volatile int32_t current_y_mode =
    0; // 0: Duophony, 1: Dual Preset, 2: MIDI Playback
volatile int32_t selectedPresetB = 0; // Flat preset index for Preset B
volatile int32_t selectedMidiIdx = 0; // Selected MIDI file index
uint16_t master_bank2 = 0;
uint16_t master_preset2 = 0;
int selectedPreset = 0;
int activeBank = 0; // 0: Melodic, 128: Drum Bank
uint16_t master_bank = 0;
uint16_t master_preset = 0;
uint8_t midi_bank_msb = 0;
uint8_t midi_bank_lsb = 0;
volatile int32_t active_scale = 0;

// MIDI Storage & Playback
struct MidiEntry {
  char name[24];
  const uint8_t *data;
  uint32_t size;
  uint32_t stereo_mode; // 0 = Stereo, 1 = Dual Mono
  uint32_t cv1_mode;    // 0 = Gate, 1 = Trigger
  uint32_t cv2_mode;    // 0 = Gate, 1 = Trigger
  uint32_t loop_mode;   // 0 = Loop, 1 = Play Once & Advance
  ChannelRouting channels[16];
};
#define MAX_MIDI_FILES 16
MidiEntry midi_files[MAX_MIDI_FILES];
uint32_t num_midi_files = 0;
#define MIDI_BASE_ADDRESS (0x10000000 + 500 * 1024) // 500KB flash offset

#define FLASH_SETTINGS_OFFSET 0x000FF000
#define FLASH_SETTINGS_XIP_ADDR (0x10000000 + FLASH_SETTINGS_OFFSET)
#define SETTINGS_MAGIC 0x5346324D // 'SF2M'

struct PersistentSettings {
  uint32_t magic;
  uint32_t mode; // 0 = Normal, 1 = MIDI Playback
};

void save_persistent_settings(bool is_midi) {
  PersistentSettings settings;
  settings.magic = SETTINGS_MAGIC;
  settings.mode = is_midi ? 1 : 0;

  uint8_t buffer[256];
  memset(buffer, 0, sizeof(buffer));
  memcpy(buffer, &settings, sizeof(settings));

  // Lock out Core 1 to prevent it from executing code in flash
  multicore_lockout_start_blocking();

  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(FLASH_SETTINGS_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(FLASH_SETTINGS_OFFSET, buffer, sizeof(buffer));
  restore_interrupts(ints);

  multicore_lockout_end_blocking();
}

void load_persistent_settings() {
  const PersistentSettings *settings = (const PersistentSettings *)FLASH_SETTINGS_XIP_ADDR;
  if (settings->magic == SETTINGS_MAGIC) {
    if (settings->mode == 1 && num_midi_files > 0) {
      midi_playback_mode = true;
    } else {
      midi_playback_mode = false;
    }
  } else {
    // No saved settings (fresh flash): auto-enable MIDI playback if files present
    midi_playback_mode = (num_midi_files > 0);
  }
}

volatile bool midi_playing = false;
volatile uint32_t midi_tempo_us = 500000; // 120 BPM
uint32_t midi_tick_accumulator = 0;
uint64_t last_seq_time_us = 0;
uint16_t midi_division = 96;
uint32_t midi_sequence_ticks = 0;
uint32_t midi_last_beat_counter = 0;

volatile bool mute_midi_events = false;

#define MAX_LOOP_EVENTS 1024
struct LoopEvent {
  bool active = false;
  uint16_t tick = 0;       // 0 to 1535 (4 bars)
  uint8_t type = 0;        // AUDIO_EVT_NOTE_ON, AUDIO_EVT_NOTE_OFF
  uint8_t note = 0;
  uint8_t velocity = 0;
  uint8_t channel = 0;
  uint16_t bank = 0;
  uint16_t preset = 0;
};
LoopEvent tape_loop_buffer[MAX_LOOP_EVENTS];
volatile uint32_t elapsed_ticks = 0;

void record_tape_event(uint8_t type, uint8_t note, uint8_t velocity, uint8_t channel) {
  if (freeze_active) return;
  uint32_t len = 16 * midi_division;
  uint32_t t = midi_sequence_ticks % len;
  
  int slot = -1;
  for (int i = 0; i < MAX_LOOP_EVENTS; i++) {
    if (!tape_loop_buffer[i].active) {
      slot = i;
      break;
    }
  }
  if (slot == -1) {
    static uint32_t rng = 12345;
    rng = rng * 1103515245 + 12345;
    slot = (rng / 65536) % MAX_LOOP_EVENTS;
  }
  
  tape_loop_buffer[slot].active = true;
  tape_loop_buffer[slot].tick = t;
  tape_loop_buffer[slot].type = type;
  tape_loop_buffer[slot].note = note;
  tape_loop_buffer[slot].velocity = velocity;
  tape_loop_buffer[slot].channel = channel;
  tape_loop_buffer[slot].bank = channel_banks[channel];
  tape_loop_buffer[slot].preset = channel_presets[channel];
}

void erase_tape_ticks(uint32_t start_tick, uint32_t ticks) {
  uint32_t len = 16 * midi_division;
  for (uint32_t i = 0; i < ticks; i++) {
    uint32_t t = (start_tick + 1 + i) % len;
    for (int e = 0; e < MAX_LOOP_EVENTS; e++) {
      if (tape_loop_buffer[e].active && tape_loop_buffer[e].tick == t) {
        tape_loop_buffer[e].active = false;
      }
    }
  }
}

struct TrackState {
  const uint8_t *ptr = nullptr;
  const uint8_t *end = nullptr;
  uint32_t ticks_to_next = 0;
  uint8_t running_status = 0;
  bool active = false;
} midi_tracks[32];
uint32_t midi_num_tracks = 0;

// Output beat clock timer (Pulse 2 Out)
volatile int32_t pulse2_out_timer = 0;
volatile int32_t cv1_trigger_timer = 0;
volatile int32_t cv2_trigger_timer = 0;

// Boot Silence and CV Override State
volatile int32_t bootSilence = 48000;
volatile int32_t cv2_override_acc = 0;

// Knob Locks
KnobLock lockMain;
KnobLock lockX;
KnobLock lockY;

enum RequestType { REQ_NOTE_ON, REQ_NOTE_OFF };

struct NoteRequest {
  RequestType type;
  uint8_t note;
  uint8_t velocity;
  uint16_t bank;
  uint16_t preset;
  uint8_t channel;
  bool apply_pitch_knob;
};

#define REQUEST_QUEUE_SIZE 32
NoteRequest request_queue[REQUEST_QUEUE_SIZE];
volatile uint32_t request_head = 0;
volatile uint32_t request_tail = 0;

struct TriggeredVoiceEvent {
  const int16_t *sample_data;
  uint32_t sample_end;
  uint32_t loop_start;
  uint32_t loop_end;
  bool is_looped;

  uint64_t phase_increment;
  int32_t env_attack_step;
  int32_t env_decay_step;
  int32_t env_release_step;
  int32_t env_sustain;

  int32_t mod_env_attack_step;
  int32_t mod_env_decay_step;
  int32_t mod_env_release_step;
  int32_t mod_env_sustain;
  int16_t mod_env_to_pitch;

  int32_t vol_atten_q15;
  int32_t filter_fc_base;
  int16_t scale_tuning;
};

enum EventType { EVT_NOTE_ON, EVT_NOTE_OFF, EVT_ALL_NOTES_OFF };

struct AudioEvent {
  EventType type;
  uint8_t channel;
  uint8_t note;
  uint8_t velocity;
  TriggeredVoiceEvent voice_data;
};

#define AUDIO_EVENT_QUEUE_SIZE 64
AudioEvent audio_event_queue[AUDIO_EVENT_QUEUE_SIZE];
volatile uint32_t audio_event_head = 0;
volatile uint32_t audio_event_tail = 0;

void push_note_on_event(uint8_t note, uint8_t velocity, uint16_t bank,
                        uint16_t preset, uint8_t channel,
                        bool apply_pitch_knob);
void push_note_off_event(uint8_t note, uint8_t channel);
void push_all_notes_off_event(uint8_t channel);
void start_resolved_voice_from_event(const TriggeredVoiceEvent &params,
                                     uint8_t note, uint8_t velocity,
                                     uint8_t channel);
void trigger_note_off_channel(uint8_t note, uint8_t channel);

static inline void queue_audio_event(uint8_t type, uint8_t note,
                                     uint8_t velocity = 0,
                                     uint8_t channel = 0) {
  if (mute_midi_events) return;
  if (type == AUDIO_EVT_NOTE_ON) {
    uint16_t bank = channel_banks[channel];
    uint16_t preset = channel_presets[channel];
    push_note_on_event(note, velocity, bank, preset, channel, false);
  } else if (type == AUDIO_EVT_NOTE_OFF) {
    push_note_off_event(note, channel);
  } else if (type == AUDIO_EVT_ALL_NOTES_OFF) {
    uint8_t ch = (note == 0xFF || channel == 0xFF) ? 0xFF : ((channel != 0) ? channel : note);
    push_all_notes_off_event(ch);
  }
}

// Synthesis Voice State
enum VoiceState {
  VOICE_OFF,
  VOICE_ATTACK,
  VOICE_DECAY,
  VOICE_SUSTAIN,
  VOICE_RELEASE
};

struct SynthVoice {
  VoiceState state = VOICE_OFF;
  int16_t note = -1;
  int16_t midi_note = -1;
  uint8_t channel = 0;
  bool key_down = false;
  uint64_t phase = 0;           // Q32.32
  uint64_t phase_increment = 0; // Q32.32
  uint16_t velocity = 100;

  // Volume envelope
  int32_t env_val = 0;          // Q31
  int32_t env_sustain = 0;      // Q31
  int32_t env_attack_step = 0;  // Q31 step per sample
  int32_t env_decay_step = 0;   // Q31 step per sample
  int32_t env_release_step = 0; // Q31 step per sample

  // SF2 Modulation envelope (drives pitch sweep on attack)
  int32_t mod_env_val = 0;          // Q31 current level
  int32_t mod_env_attack_step = 0;  // Q31 step per sample
  int32_t mod_env_decay_step = 0;   // Q31 step per sample
  int32_t mod_env_release_step = 0; // Q31 step per sample
  int32_t mod_env_sustain = 0;      // Q31 sustain level
  int16_t mod_env_to_pitch = 0;     // cents at peak (gen17)
  VoiceState mod_env_state = VOICE_ATTACK;

  // Per-zone SF2 tonal parameters
  int32_t vol_atten_q15 =
      32767; // Q15 amplitude scale from gen48 (initial attenuation)
  int32_t filter_fc_base = 45500; // base filter_f from zone gen8 (fully open)
  int16_t scale_tuning = 100;     // cents per semitone, gen56

  // Sample metadata
  const int16_t *sample_data = nullptr;
  uint32_t sample_start = 0;
  uint32_t sample_end = 0;
  uint32_t loop_start = 0;
  uint32_t loop_end = 0;
  bool is_looped = false;

  // Stable one-pole lowpass state (Chamberlin SVF)
  int32_t filter_low = 0;
  int32_t filter_band = 0;
  int32_t filter_f = 10000; // Q16 coefficient (updated each sample)
  int32_t filter_q = 65536; // Q16 coefficient

  uint32_t age = 0;
  // Cached modulated phase increment — only recomputed when mod_env changes
  // Avoids 64-bit multiply every sample during steady sustain
  uint64_t cached_mod_increment = 0;
  bool mod_increment_dirty = true;

  // Performance Glitch parameters per voice
  uint32_t micro_loop_start = 0;
  uint32_t micro_loop_size = 0;
  // Wrong-sample: stable alternate sample index picked once per note
  int32_t glitch_wrong_idx = -1;
  uint8_t glitch_state = 0;     // 0 = NONE, 1 = REVERSE, 2 = MICRO_LOOP, 3 = XOR, 4 = WRONG_SAMPLE
  int32_t glitch_timer = 0;
  int32_t glitch_duration = 0;
  int32_t glitch_mangle_held = 0;
};

#define MAX_VOICES 8
SynthVoice voices[MAX_VOICES];
uint32_t global_voice_age = 0;

struct MasterFilter {
  int32_t low_L = 0;
  int32_t band_L = 0;
  int32_t low_R = 0;
  int32_t band_R = 0;
};
static MasterFilter lpf_filter;
static MasterFilter hpf_filter;

// Resolved Patch parameters for triggering notes
struct ResolvedParams {
  const int16_t *sample_data = nullptr;
  uint32_t start = 0;
  uint32_t end = 0;
  uint32_t loop_start = 0;
  uint32_t loop_end = 0;
  bool is_looped = false;
  uint32_t sample_rate = 44100;
  uint8_t root_key = 60;
  int8_t pitch_correction = 0;
  int16_t coarse_tune = 0;
  int16_t fine_tune = 0;
  int16_t scale_tuning = 100; // cents per semitone (gen56, default 100)

  // SF2 Volume Envelope (timecents)
  int16_t vol_attack = -12000;
  int16_t vol_decay = -12000;
  int16_t vol_sustain = 0; // centibels attenuation at sustain
  int16_t vol_release = -12000;

  // SF2 Modulation Envelope (timecents, drives pitch via gen17)
  int16_t mod_attack = -12000;
  int16_t mod_decay = -12000;
  int16_t mod_sustain = 0; // 0..1000 centibels
  int16_t mod_release = -12000;
  int16_t mod_env_to_pitch = 0; // cents at mod env peak (gen17)

  // SF2 per-zone tonal parameters
  int16_t initial_attenuation = 0; // centibels (gen48), 0 = full vol
  int16_t filter_cutoff = 13500;   // absCents (gen8), 13500 ≈ fully open
};

// Fast Fixed-point Exponentiation Approximations
uint32_t timecents_to_ms(int16_t timecents) {
  if (timecents <= -12000)
    return 1;
  if (timecents >= 8000)
    return 100000;

  int32_t I = timecents / 1200;
  int32_t F = timecents % 1200;
  if (F < 0) {
    F += 1200;
    I -= 1;
  }

  int64_t f = F;
  int64_t f2 = (f * f) / 1200;
  int64_t f3 = (f2 * f) / 1200;

  int32_t q_frac =
      65536 + (45426 * f) / 1200 + (15743 * f2) / 1200 + (3640 * f3) / 1200;
  int32_t ms = (q_frac * 1000) >> 16;
  if (I >= 0) {
    ms <<= I;
  } else {
    ms >>= (-I);
  }
  if (ms < 1)
    ms = 1;
  return ms;
}

int32_t sustain_centibels_to_q31(int16_t cb) {
  if (cb <= 0)
    return 2147483647;
  if (cb >= 960)
    return 0;

  int32_t power_q16 = cb * 1089;
  int32_t I = power_q16 >> 16;
  int32_t F = power_q16 & 0xFFFF;

  int64_t f = F;
  int64_t f2 = (f * f) >> 16;
  int64_t f3 = (f2 * f) >> 16;

  int32_t q_frac = 1073741824 - (744261118 * f >> 16) + (257945828 * f2 >> 16) -
                   (59648937 * f3 >> 16);
  int32_t val = (q_frac << 1) >> I;
  return val;
}
extern "C" void handle_midi_message(uint8_t *packet, int size);

uint32_t cents_to_pitch_multiplier_q16(int32_t cents) {
  int32_t I = cents / 1200;
  int32_t F = cents % 1200;
  if (F < 0) {
    F += 1200;
    I -= 1;
  }

  int64_t f = F;
  int64_t f2 = (f * f) / 1200;
  int64_t f3 = (f2 * f) / 1200;

  int32_t q_frac =
      65536 + (45426 * f) / 1200 + (15743 * f2) / 1200 + (3640 * f3) / 1200;
  if (I >= 0) {
    return q_frac << I;
  } else {
    return q_frac >> (-I);
  }
}

uint32_t read_vlq(const uint8_t *&ptr) {
  uint32_t val = 0;
  uint8_t b;
  do {
    b = *ptr++;
    val = (val << 7) | (b & 0x7F);
  } while (b & 0x80);
  return val;
}

void scan_midi_files() {
  num_midi_files = 0;

  // Initialize sensible defaults for global performance config:
  global_config.stereo_mode = 0; // Stereo default
  global_config.cv1_mode = 0;
  global_config.cv2_mode = 0;
  global_config.loop_mode = 0;
  normal_mode_config.stereo_mode = 0;
  normal_mode_config.cv1_mode = 0;
  normal_mode_config.cv2_mode = 0;
  normal_mode_config.loop_mode = 0;

  for (int i = 0; i < 8; i++) {
    if (i == 0) {
      global_config.cc_maps[i] = 74; // Cutoff
      normal_mode_config.cc_maps[i] = 74;
    } else if (i == 1) {
      global_config.cc_maps[i] = 39; // Reverb Mix
      normal_mode_config.cc_maps[i] = 39;
    } else if (i == 2) {
      global_config.cc_maps[i] = 71; // Glitch Mangle / Resonance
      normal_mode_config.cc_maps[i] = 71;
    } else if (i == 3) {
      global_config.cc_maps[i] = 36; // Attack
      normal_mode_config.cc_maps[i] = 36;
    } else if (i == 4) {
      global_config.cc_maps[i] = 37; // Decay/Release
      normal_mode_config.cc_maps[i] = 37;
    } else {
      global_config.cc_maps[i] = 0;
      normal_mode_config.cc_maps[i] = 0;
    }
  }

  for (int i = 0; i < 16; i++) {
    if (i == 0) {
      global_config.channels[i].target = 2; // CV/Gate 1
    } else if (i == 1) {
      global_config.channels[i].target = 3; // CV/Gate 2
    } else if (i == 9) {
      global_config.channels[i].target = 1; // Synth 2 / Drums
    } else {
      global_config.channels[i].target = 0; // Synth 1
    }
    global_config.channels[i].instrument_override = -1; // No override
    global_config.channels[i].volume_trim = 0;           // Default 0 dB
    normal_mode_config.channels[i].target = global_config.channels[i].target;
    normal_mode_config.channels[i].instrument_override = global_config.channels[i].instrument_override;
    normal_mode_config.channels[i].volume_trim = global_config.channels[i].volume_trim;
  }

  const uint8_t *mids_base = (const uint8_t *)MIDI_BASE_ADDRESS;
  if (memcmp(mids_base, "MIDS", 4) == 0) {
    uint32_t flash_count = *(const uint32_t *)(mids_base + 4);
    if (flash_count < 16) {
      struct FlashMidiHeader {
        char name[24];
        uint32_t offset;
        uint32_t size;
        uint32_t stereo_mode;
        uint32_t cv1_mode;
        uint32_t cv2_mode;
        uint32_t loop_mode;
        ChannelRouting channels[16];
      } __attribute__((packed));

      uint32_t header_offset = 8;

      // Check for CONF block right after num_midi_files (offset 8)
      if (memcmp(mids_base + 8, "CONF", 4) == 0) {
        normal_mode_config.stereo_mode = *(const uint32_t *)(mids_base + 12);
        const ChannelRouting *flash_channels = (const ChannelRouting *)(mids_base + 16);
        for (int i = 0; i < 16; i++) {
          normal_mode_config.channels[i].target = flash_channels[i].target;
          normal_mode_config.channels[i].instrument_override = flash_channels[i].instrument_override;
          int16_t trim = flash_channels[i].volume_trim;
          if (trim < -12 || trim > 6) {
            normal_mode_config.channels[i].volume_trim = 0;
          } else {
            normal_mode_config.channels[i].volume_trim = trim;
          }
        }
        normal_mode_config.cv1_mode = *(const uint32_t *)(mids_base + 144);
        normal_mode_config.cv2_mode = *(const uint32_t *)(mids_base + 148);
        normal_mode_config.loop_mode = *(const uint32_t *)(mids_base + 152);

        // Read custom CC mapping bytes
        for (int i = 0; i < 8; i++) {
          uint8_t val = mids_base[156 + i];
          if (val == 0xFF || (val == 0 && i < 5)) {
            // Use defaults
            if (i == 0) normal_mode_config.cc_maps[i] = 74; // Cutoff
            else if (i == 1) normal_mode_config.cc_maps[i] = 39; // Reverb Mix
            else if (i == 2) normal_mode_config.cc_maps[i] = 71; // Glitch Mangle / Resonance
            else if (i == 3) normal_mode_config.cc_maps[i] = 36; // Attack
            else if (i == 4) normal_mode_config.cc_maps[i] = 37; // Decay/Release
            else normal_mode_config.cc_maps[i] = 0;
          } else {
            normal_mode_config.cc_maps[i] = val;
          }
        }

        header_offset = 8 + 156; // CONF block is 156 bytes, so 8 + 156 = 164
      }

      const FlashMidiHeader *hdrs = (const FlashMidiHeader *)(mids_base + header_offset);
      for (uint32_t i = 0; i < flash_count; i++) {
        if (num_midi_files >= MAX_MIDI_FILES)
          break;
        if (hdrs[i].offset < 1024 * 1024 && hdrs[i].size < 1024 * 1024) {
          memcpy(midi_files[num_midi_files].name, hdrs[i].name, 24);
          midi_files[num_midi_files].data = mids_base + hdrs[i].offset;
          midi_files[num_midi_files].size = hdrs[i].size;
          midi_files[num_midi_files].stereo_mode = hdrs[i].stereo_mode;
          midi_files[num_midi_files].cv1_mode = hdrs[i].cv1_mode;
          midi_files[num_midi_files].cv2_mode = hdrs[i].cv2_mode;
          midi_files[num_midi_files].loop_mode = hdrs[i].loop_mode;
          for (int c = 0; c < 16; c++) {
            midi_files[num_midi_files].channels[c].target = hdrs[i].channels[c].target;
            midi_files[num_midi_files].channels[c].instrument_override = hdrs[i].channels[c].instrument_override;
            int16_t trim = hdrs[i].channels[c].volume_trim;
            if (trim < -12 || trim > 6) {
              midi_files[num_midi_files].channels[c].volume_trim = 0;
            } else {
              midi_files[num_midi_files].channels[c].volume_trim = trim;
            }
          }
          num_midi_files++;
        }
      }
    }
  }

  // 2. Fallback to built-in MIDI arpeggio
  if (num_midi_files == 0) {
    // Removed modal test fallback
  }
}

void apply_midi_mode_configuration() {
  if (midi_playback_mode && num_midi_files > 0 && selectedMidiIdx >= 0 && selectedMidiIdx < (int32_t)num_midi_files) {
    global_config.stereo_mode = midi_files[selectedMidiIdx].stereo_mode;
    global_config.cv1_mode = midi_files[selectedMidiIdx].cv1_mode;
    global_config.cv2_mode = midi_files[selectedMidiIdx].cv2_mode;
    global_config.loop_mode = midi_files[selectedMidiIdx].loop_mode;
    for (int i = 0; i < 16; i++) {
      global_config.channels[i].target = midi_files[selectedMidiIdx].channels[i].target;
      global_config.channels[i].instrument_override = midi_files[selectedMidiIdx].channels[i].instrument_override;
      global_config.channels[i].volume_trim = midi_files[selectedMidiIdx].channels[i].volume_trim;
    }
    for (int i = 0; i < 8; i++) {
      global_config.cc_maps[i] = normal_mode_config.cc_maps[i];
    }
  }
}

void apply_mode_configuration() {
  if (midi_playback_mode) {
    apply_midi_mode_configuration();
  } else {
    global_config.stereo_mode = normal_mode_config.stereo_mode;
    global_config.cv1_mode = normal_mode_config.cv1_mode;
    global_config.cv2_mode = normal_mode_config.cv2_mode;
    global_config.loop_mode = normal_mode_config.loop_mode;
    for (int i = 0; i < 16; i++) {
      global_config.channels[i].target = normal_mode_config.channels[i].target;
      global_config.channels[i].instrument_override = normal_mode_config.channels[i].instrument_override;
      global_config.channels[i].volume_trim = normal_mode_config.channels[i].volume_trim;
    }
    for (int i = 0; i < 8; i++) {
      global_config.cc_maps[i] = normal_mode_config.cc_maps[i];
    }
  }
}

void stop_all_notes_and_gates();

void start_midi_playback(const uint8_t *data, uint32_t size) {
  if (!data || size < 14)
    return;
  if (memcmp(data, "MThd", 4) != 0)
    return;

  apply_midi_mode_configuration();
  stop_all_notes_and_gates();

  // Clear tape loop buffer on playback start/switch
  for (int i = 0; i < MAX_LOOP_EVENTS; i++) {
    tape_loop_buffer[i].active = false;
  }

  // Reset channel presets and banks, default channel 9 to drum bank 128
  for (int i = 0; i < 16; i++) {
    if (i == 9) {
      channel_banks[i] = 128;
      channel_presets[i] = 0;
    } else {
      channel_banks[i] = 0;
      channel_presets[i] = 0;
    }
  }

  midi_sequence_ticks = 0;
  midi_last_beat_counter = 0;
  midi_next_boundary_tick = 0;
  pulse1_trigger_occurred = false;
  pulse2_trigger_occurred = false;
  midi_last_pulse1_time = 0;
  mclk_trigger_occurred = false;
  last_midi_clock_time_us = 0;

  uint32_t header_len =
      (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
  if (header_len < 6)
    return;

  uint16_t format = (data[8] << 8) | data[9];
  if (format > 1)
    return; // Only format 0 or 1 supported

  uint16_t num_tracks = (data[10] << 8) | data[11];
  if (num_tracks > 32)
    num_tracks = 32;
  midi_num_tracks = num_tracks;

  uint16_t div = (data[12] << 8) | data[13];
  if (div == 0)
    return;
  midi_division = div;

  uint32_t pos = 8 + header_len;
  for (uint32_t t = 0; t < midi_num_tracks; t++) {
    if (pos + 8 > size) {
      midi_num_tracks = t;
      break;
    }
    if (memcmp(data + pos, "MTrk", 4) != 0) {
      bool found = false;
      while (pos + 8 <= size) {
        if (memcmp(data + pos, "MTrk", 4) == 0) {
          found = true;
          break;
        }
        pos++;
      }
      if (!found) {
        midi_num_tracks = t;
        break;
      }
    }
    uint32_t trk_len = (data[pos + 4] << 24) | (data[pos + 5] << 16) |
                       (data[pos + 6] << 8) | data[pos + 7];
    pos += 8;
    if (pos + trk_len > size) {
      trk_len = size - pos;
    }

    midi_tracks[t].ptr = data + pos;
    midi_tracks[t].end = data + pos + trk_len;
    midi_tracks[t].active = trk_len > 0;
    midi_tracks[t].running_status = 0;
    if (midi_tracks[t].active) {
      midi_tracks[t].ticks_to_next = read_vlq(midi_tracks[t].ptr);
    } else {
      midi_tracks[t].ticks_to_next = 0;
    }

    pos += trk_len;
  }

  midi_tick_accumulator = 0;
  last_seq_time_us = time_us_64();
  if (midi_tempo_us == 0) {
    midi_tempo_us = 500000; // 120 BPM
  }
  midi_playing = true;
  pending_midi_idx = selectedMidiIdx;
  selected_pattern_knob_idx = selectedMidiIdx;

  cv1_tracker.clear();
  cv2_tracker.clear();
  cv1_out_gate = false;
  cv2_out_gate = false;

  // Initialize beat 0 state in midi_track_beat_states
  midi_beat_start_tick = 0;
  for (uint32_t t = 0; t < midi_num_tracks; t++) {
    midi_track_beat_states[t].ptr = midi_tracks[t].ptr;
    midi_track_beat_states[t].ticks_to_next = midi_tracks[t].ticks_to_next;
    midi_track_beat_states[t].running_status = midi_tracks[t].running_status;
    midi_track_beat_states[t].active = midi_tracks[t].active;
  }

  // Initialize bar 0 state in midi_track_bar_states
  midi_bar_start_tick = 0;
  for (uint32_t t = 0; t < midi_num_tracks; t++) {
    midi_track_bar_states[t].ptr = midi_tracks[t].ptr;
    midi_track_bar_states[t].ticks_to_next = midi_tracks[t].ticks_to_next;
    midi_track_bar_states[t].running_status = midi_tracks[t].running_status;
    midi_track_bar_states[t].active = midi_tracks[t].active;
  }
}

void process_next_midi_event(uint32_t t) {
  TrackState &ts = midi_tracks[t];
  if (ts.ptr >= ts.end) {
    ts.active = false;
    return;
  }

  uint8_t status = *ts.ptr++;
  if (status < 0x80) {
    ts.ptr--;
    status = ts.running_status;
  } else if (status < 0xF0) {
    ts.running_status = status;
  }

  uint8_t event_type = status & 0xF0;
  uint8_t channel = status & 0x0F;

  if (status < 0xF0) {
    if (event_type == 0x90) {
      uint8_t note = *ts.ptr++;
      uint8_t vel = *ts.ptr++;
      if (vel > 0) {
        queue_audio_event(AUDIO_EVT_NOTE_ON, note, vel, channel);
      } else {
        queue_audio_event(AUDIO_EVT_NOTE_OFF, note, 0, channel);
      }
    } else if (event_type == 0x80) {
      uint8_t note = *ts.ptr++;
      uint8_t vel = *ts.ptr++;
      (void)vel;
      queue_audio_event(AUDIO_EVT_NOTE_OFF, note, 0, channel);
    } else if (event_type == 0xB0) {
      uint8_t cc = *ts.ptr++;
      uint8_t val = *ts.ptr++;
      uint8_t packet[3] = {status, cc, val};
      handle_midi_message(packet, 3);
    } else if (event_type == 0xC0) {
      uint8_t program = *ts.ptr++;
      uint8_t packet[2] = {status, program};
      handle_midi_message(packet, 2);
    } else if (event_type == 0xA0) {
      ts.ptr += 2;
    } else if (event_type == 0xD0) {
      ts.ptr += 1;
    } else if (event_type == 0xE0) {
      uint8_t lsb = *ts.ptr++;
      uint8_t msb = *ts.ptr++;
      uint8_t packet[3] = {status, lsb, msb};
      handle_midi_message(packet, 3);
    }
  } else if (status == 0xFF) {
    uint8_t meta_type = *ts.ptr++;
    uint32_t len = read_vlq(ts.ptr);
    const uint8_t *meta_data = ts.ptr;
    ts.ptr += len;

    if (meta_type == 0x2F) {
      ts.active = false;
      return;
    } else if (meta_type == 0x51 && len == 3) {
      uint32_t tempo =
          (meta_data[0] << 16) | (meta_data[1] << 8) | meta_data[2];
      if (!debounced_connected[ComputerCard::Input::Pulse2]) {
        midi_tempo_us = tempo;
      }
    }
  } else if (status == 0xF0 || status == 0xF7) {
    uint32_t len = read_vlq(ts.ptr);
    ts.ptr += len;
  }

  if (ts.ptr < ts.end) {
    ts.ticks_to_next = read_vlq(ts.ptr);
  } else {
    ts.active = false;
  }
}

uint32_t get_stutter_length() {
  uint32_t base_div = midi_division;
  if (base_div == 0) base_div = 96;

  int32_t raw_main = param_stutter;
  if (raw_main < 2048) {
    // Tape stop mode: loop the full 1 bar
    return 4 * base_div;
  }

  // Stutter mode: subdivide from 1 bar down to 1/64 note
  int32_t val = raw_main - 2048; // 0..2047
  int32_t div_idx = (val * 7) >> 11; // 0..6
  if (div_idx < 0) div_idx = 0;
  if (div_idx > 6) div_idx = 6;

  uint32_t len = base_div;
  switch (div_idx) {
    case 0: len = 4 * base_div;   // 1 bar
            break;
    case 1: len = 2 * base_div;   // 1/2 note
            break;
    case 2: len = base_div;       // 1/4 note (1 beat)
            break;
    case 3: len = base_div / 2;   // 1/8 note
            break;
    case 4: len = base_div / 4;   // 1/16 note
            break;
    case 5: len = base_div / 8;   // 1/32 note
            break;
    case 6: len = base_div / 16;  // 1/64 note
            break;
  }
  if (len < 1) len = 1;
  return len;
}

void step_midi_sequencer_ticks(uint32_t ticks_to_step) {
  if (!midi_playing)
    return;

  for (uint32_t step = 0; step < ticks_to_step; step++) {
    bool any_active = false;
    for (uint32_t t = 0; t < midi_num_tracks; t++) {
      if (midi_tracks[t].active) {
        any_active = true;
        if (midi_tracks[t].ticks_to_next > 0) {
          midi_tracks[t].ticks_to_next--;
        }
        while (midi_tracks[t].active && midi_tracks[t].ticks_to_next == 0) {
          process_next_midi_event(t);
        }
      }
    }

    midi_sequence_ticks++;
    
    // Check if beat boundary was crossed
    if (midi_sequence_ticks % midi_division == 0) {
      pulse2_out_timer = 480; // Output beat pulse
      midi_last_beat_counter = midi_sequence_ticks / midi_division;

      // 1. Bar-aligned Pattern Switching
      if (midi_sequence_ticks % (4 * midi_division) == 0) {
        if (pending_midi_idx != selectedMidiIdx && pending_midi_idx >= 0 && pending_midi_idx < (int32_t)num_midi_files) {
          selectedMidiIdx = pending_midi_idx;
          queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0xFF);
          start_midi_playback(midi_files[selectedMidiIdx].data, midi_files[selectedMidiIdx].size);
          // Automatically unfreeze after the bar finishes!
          if (freeze_active) {
            freeze_active = false;
          }
          return; // Pointers reset, stop this step loop
        }
        // Save bar states only when NOT frozen so that when we DO freeze, we can loop from the bar boundary
        if (!freeze_active) {
          for (uint32_t t = 0; t < midi_num_tracks; t++) {
            midi_track_bar_states[t].ptr = midi_tracks[t].ptr;
            midi_track_bar_states[t].ticks_to_next = midi_tracks[t].ticks_to_next;
            midi_track_bar_states[t].running_status = midi_tracks[t].running_status;
            midi_track_bar_states[t].active = midi_tracks[t].active;
          }
          midi_bar_start_tick = midi_sequence_ticks;
        }
      }

      // Save beat states only when NOT frozen so that when we DO freeze, we loop the correct beat
      if (!freeze_active) {
        // Save track states at start of next beat
        for (uint32_t t = 0; t < midi_num_tracks; t++) {
          midi_track_beat_states[t].ptr = midi_tracks[t].ptr;
          midi_track_beat_states[t].ticks_to_next = midi_tracks[t].ticks_to_next;
          midi_track_beat_states[t].running_status = midi_track_beat_states[t].running_status;
          midi_track_beat_states[t].active = midi_track_beat_states[t].active;
        }
        midi_beat_start_tick = midi_sequence_ticks;
      }
    }

    // 2. Stutter Freeze Loop (checked on every tick)
    if (freeze_active) {
      uint32_t freeze_len = get_stutter_length();
      uint32_t elapsed = midi_sequence_ticks - midi_freeze_start_tick;
      if (elapsed >= freeze_len) {
        // Loop back to start of freeze
        for (uint32_t t = 0; t < midi_num_tracks; t++) {
          midi_tracks[t].ptr = midi_track_freeze_states[t].ptr;
          midi_tracks[t].ticks_to_next = midi_track_freeze_states[t].ticks_to_next;
          midi_tracks[t].running_status = midi_track_freeze_states[t].running_status;
          midi_tracks[t].active = midi_track_freeze_states[t].active;
        }
        midi_sequence_ticks = midi_freeze_start_tick;
        queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0xFF);
      }
    }

    if (!any_active) {
      // Kill any held voices / CV gates before restarting or stopping
      push_all_notes_off_event(0xFF);
      if (midi_playback_mode && num_midi_files > 0) {
        if (global_config.loop_mode == 1) {
          // Play-once-and-advance: prime next clip (wrapping), stop
          selectedMidiIdx = (selectedMidiIdx + 1) % (int32_t)num_midi_files;
          pending_midi_idx = selectedMidiIdx;
          start_midi_playback(midi_files[selectedMidiIdx].data,
                              midi_files[selectedMidiIdx].size);
          // apply the new file's config then stop
          apply_midi_mode_configuration();
          midi_playing = false;
        } else {
          // Loop mode: restart current clip (or switch to pending if queued)
          if (pending_midi_idx != selectedMidiIdx && pending_midi_idx >= 0 && pending_midi_idx < (int32_t)num_midi_files) {
            selectedMidiIdx = pending_midi_idx;
          }
          start_midi_playback(midi_files[selectedMidiIdx].data,
                              midi_files[selectedMidiIdx].size);
        }
      } else {
        midi_playing = false;
      }
      break;
    }
  }
}

// Moved below SamplesApp declaration

// SoundFont RIFF Boot Scanner – uses proper hierarchical RIFF tree traversal.
// A naive byte-scan is unsafe: PCM audio data (inside the 'smpl' chunk) is
// millions of bytes of essentially random int16 values and will almost
// certainly contain byte sequences matching every chunk tag ('phdr', 'pbag',
// 'shdr' …), silently overwriting all pointers with garbage flash addresses.
void parse_sf2() {
  // Force clean bail out if flash is completely unprogrammed
  if (*(const uint32_t *)sf2_base == 0xFFFFFFFF ||
      *(const uint32_t *)sf2_base == 0x00000000) {
    return;
  }

  // Check if static index is present
  struct StaticIndexHeader {
    char magic[4];
    uint32_t presets_offset;
    uint32_t presets_count;
    uint32_t samples_offset;
    uint32_t samples_count;
    uint32_t drum_presets_offset;
    uint32_t drum_presets_count;
    uint32_t unique_banks_offset;
    uint32_t unique_banks_count;
    uint32_t phdr_offset;
    uint32_t phdr_count;
    uint32_t pbag_offset;
    uint32_t pbag_count;
    uint32_t pgen_offset;
    uint32_t pgen_count;
    uint32_t inst_offset;
    uint32_t inst_count;
    uint32_t ibag_offset;
    uint32_t ibag_count;
    uint32_t igen_offset;
    uint32_t igen_count;
    uint32_t shdr_offset;
    uint32_t shdr_count;
    uint32_t sf2_file_offset;
  } __attribute__((packed));

  const StaticIndexHeader *hdr = (const StaticIndexHeader *)sf2_base;
  if (memcmp(hdr->magic, "SF2I", 4) == 0) {
    presets_index = (const PresetsIndex *)(sf2_base + hdr->presets_offset);
    indexed_presets_count = hdr->presets_count;
    samples_index = (const SamplesIndex *)(sf2_base + hdr->samples_offset);
    indexed_samples_count = hdr->samples_count;

    // Direct pointer mappings (zero boot overhead, no RIFF scanning!)
    phdr_ptr = (const sfPresetHeader *)(sf2_base + hdr->phdr_offset);
    phdr_count = hdr->phdr_count;
    pbag_ptr = (const sfBag *)(sf2_base + hdr->pbag_offset);
    pbag_count = hdr->pbag_count;
    pgen_ptr = (const sfGenList *)(sf2_base + hdr->pgen_offset);
    pgen_count = hdr->pgen_count;
    inst_ptr = (const sfInst *)(sf2_base + hdr->inst_offset);
    inst_count = hdr->inst_count;
    ibag_ptr = (const sfBag *)(sf2_base + hdr->ibag_offset);
    ibag_count = hdr->ibag_count;
    igen_ptr = (const sfGenList *)(sf2_base + hdr->igen_offset);
    igen_count = hdr->igen_count;
    shdr_ptr = (const sfSample *)(sf2_base + hdr->shdr_offset);
    shdr_count = hdr->shdr_count;

    // Shift sf2_base to start of raw SoundFont binary
    sf2_base = sf2_base + hdr->sf2_file_offset;
  } else {
    presets_index = nullptr;
    indexed_presets_count = 0;
    samples_index = nullptr;
    indexed_samples_count = 0;
  }
}


static inline int16_t sf2_s16(uint16_t v) { return (int16_t)v; }

static bool fill_resolved_sample(int sample_idx, int16_t coarse_tune,
                                 int16_t fine_tune, int16_t scale_tuning,
                                 int16_t vol_attack, int16_t vol_decay,
                                 int16_t vol_sustain, int16_t vol_release,
                                 int16_t mod_attack, int16_t mod_decay,
                                 int16_t mod_sustain, int16_t mod_release,
                                 int16_t mod_env_to_pitch,
                                 int16_t initial_attenuation,
                                 int16_t filter_cutoff, int16_t overriding_root,
                                 int16_t sample_modes, ResolvedParams &out) {
  if (sample_idx < 0 || sample_idx >= (int)indexed_samples_count)
    return false;

  SamplesIndex s_info = samples_index[sample_idx];
  if (!s_info.sample_data || s_info.end <= s_info.start)
    return false;

  const uint32_t sample_len = s_info.end - s_info.start;
  uint32_t loop_start = 0;
  uint32_t loop_end = sample_len;
  bool valid_loop = false;

  if (s_info.loop_start >= s_info.start && s_info.loop_end <= s_info.end &&
      s_info.loop_end > s_info.loop_start + 2) {
    loop_start = s_info.loop_start - s_info.start;
    loop_end = s_info.loop_end - s_info.start;
    valid_loop = loop_end <= sample_len;
  }

  out.sample_data = s_info.sample_data;
  out.start = s_info.start;
  out.end = s_info.end;
  out.loop_start = loop_start;
  out.loop_end = loop_end;
  out.is_looped = (sample_modes == 1 || sample_modes == 3) && valid_loop;
  out.sample_rate = s_info.sample_rate;
  out.root_key =
      (overriding_root != -1) ? overriding_root : s_info.original_pitch;
  out.pitch_correction = s_info.pitch_correction;
  out.coarse_tune = coarse_tune;
  out.fine_tune = fine_tune;
  out.scale_tuning =
      (scale_tuning >= 0 && scale_tuning <= 1200) ? scale_tuning : 100;
  out.vol_attack = vol_attack;
  out.vol_decay = vol_decay;
  out.vol_sustain = vol_sustain;
  out.vol_release = vol_release;
  out.mod_attack = mod_attack;
  out.mod_decay = mod_decay;
  out.mod_sustain = mod_sustain;
  out.mod_release = mod_release;
  out.mod_env_to_pitch = mod_env_to_pitch;
  out.initial_attenuation =
      (initial_attenuation >= 0) ? initial_attenuation : 0;
  out.filter_cutoff =
      (filter_cutoff >= 1500 && filter_cutoff <= 13500) ? filter_cutoff : 13500;
  return true;
}

// SF2 Zone Resolution Logic. This mirrors the web preview: collect every
// matching sample zone for the selected bank/program, rather than stopping at
// the first match.
int collect_note_zones(uint16_t bank, uint16_t preset, uint8_t key, uint8_t vel,
                       ResolvedParams *out, int max_out) {
  if (!phdr_ptr || !pbag_ptr || !pgen_ptr || !inst_ptr || !ibag_ptr ||
      !igen_ptr || !shdr_ptr)
    return 0;

  // Find active preset with robust fallback
  int p_idx = -1;

  // 1. Try exact match
  for (uint32_t i = 0; i < indexed_presets_count; i++) {
    if (presets_index[i].bank == bank && presets_index[i].preset == preset) {
      p_idx = i;
      break;
    }
  }

  // 2. Fallback for melodic presets: if bank is not found, try bank 0
  if (p_idx == -1 && bank != 128) {
    for (uint32_t i = 0; i < indexed_presets_count; i++) {
      if (presets_index[i].bank == 0 && presets_index[i].preset == preset) {
        p_idx = i;
        break;
      }
    }
  }

  // 3. Fallback for drum presets: if bank is 128 but preset is not found, try
  // first drum preset
  if (p_idx == -1 && bank == 128) {
    for (uint32_t i = 0; i < indexed_presets_count; i++) {
      if (presets_index[i].bank == 128) {
        p_idx = i;
        break;
      }
    }
  }

  // 4. Fallback to any preset with the same program number
  if (p_idx == -1) {
    for (uint32_t i = 0; i < indexed_presets_count; i++) {
      if (presets_index[i].preset == preset) {
        p_idx = i;
        break;
      }
    }
  }

  // 5. Hard fallback to the very first indexed preset (preset 0)
  if (p_idx == -1 && indexed_presets_count > 0) {
    p_idx = 0;
  }

  if (p_idx == -1)
    return 0;

  PresetsIndex p_info = presets_index[p_idx];
  int16_t global_preset_gens[64];
  bool has_global_preset_gens[64];
  for (int i = 0; i < 64; i++) {
    global_preset_gens[i] = 0;
    has_global_preset_gens[i] = false;
  }
  int zone_count = 0;

  // Preset Zone Scan
  for (uint32_t z = p_info.bag_start; z < p_info.bag_end; z++) {
    if (z + 1 >= pbag_count)
      return zone_count;
    uint16_t gen_start = pbag_ptr[z].wGenNDX;
    uint16_t gen_end = pbag_ptr[z + 1].wGenNDX;
    if (gen_start > gen_end || gen_end > pgen_count)
      return zone_count;

    int local_inst = -1;
    int preset_key_range = -1;
    int preset_vel_range = -1;

    for (uint16_t g = gen_start; g < gen_end; g++) {
      uint16_t oper = pgen_ptr[g].sfGenOper;
      uint16_t amt = pgen_ptr[g].genAmount;

      if (oper == 43) { // keyRange
        preset_key_range = amt;
      } else if (oper == 44) { // velRange
        preset_vel_range = amt;
      } else if (oper == 41) { // instrument
        local_inst = amt;
      } else if (oper < 64) {
        global_preset_gens[oper] = sf2_s16(amt);
        has_global_preset_gens[oper] = true;
      }
    }

    if (local_inst == -1)
      continue;
    if (local_inst + 1 >= (int)inst_count)
      continue;

    uint16_t ibag_start = inst_ptr[local_inst].wInstBagNDX;
    uint16_t ibag_end = inst_ptr[local_inst + 1].wInstBagNDX;
    int16_t global_inst_gens[64];
    bool has_global_inst_gens[64];
    for (int i = 0; i < 64; i++) {
      global_inst_gens[i] = 0;
      has_global_inst_gens[i] = false;
    }

    for (uint32_t iz = ibag_start; iz < ibag_end; iz++) {
      if (iz + 1 >= ibag_count)
        return zone_count;
      uint16_t igen_start = ibag_ptr[iz].wGenNDX;
      uint16_t igen_end = ibag_ptr[iz + 1].wGenNDX;
      if (igen_start > igen_end || igen_end > igen_count)
        return zone_count;

      int sample_idx = -1;
      int inst_key_range = -1;
      int inst_vel_range = -1;
      int16_t local_gens[64];
      bool has_local_gens[64];
      for (int i = 0; i < 64; i++) {
        local_gens[i] = 0;
        has_local_gens[i] = false;
      }

      for (uint16_t g = igen_start; g < igen_end; g++) {
        uint16_t oper = igen_ptr[g].sfGenOper;
        uint16_t amt = igen_ptr[g].genAmount;

        if (oper == 43) {
          inst_key_range = amt;
        } else if (oper == 44) {
          inst_vel_range = amt;
        } else if (oper == 53) {
          sample_idx = amt;
        } else if (oper < 64) {
          local_gens[oper] = sf2_s16(amt);
          has_local_gens[oper] = true;
        }
      }

      if (sample_idx == -1) {
        for (int i = 0; i < 64; i++) {
          if (has_local_gens[i]) {
            global_inst_gens[i] = local_gens[i];
            has_global_inst_gens[i] = true;
          }
        }
        continue;
      }

      int key_range = inst_key_range;
      if (key_range < 0)
        key_range =
            has_global_inst_gens[43] ? global_inst_gens[43] : preset_key_range;
      if (key_range < 0)
        key_range = has_global_preset_gens[43] ? global_preset_gens[43] : -1;
      if (key_range >= 0) {
        uint8_t low = key_range & 0xFF;
        uint8_t high = (key_range >> 8) & 0xFF;
        if (key < low || key > high)
          continue;
      }

      int vel_range = inst_vel_range;
      if (vel_range < 0)
        vel_range =
            has_global_inst_gens[44] ? global_inst_gens[44] : preset_vel_range;
      if (vel_range < 0)
        vel_range = has_global_preset_gens[44] ? global_preset_gens[44] : -1;
      if (vel_range >= 0) {
        uint8_t low = vel_range & 0xFF;
        uint8_t high = (vel_range >> 8) & 0xFF;
        if (vel < low || vel > high)
          continue;
      }

// Helper macro: resolve gen with local → global_inst → global_preset fallback
#define GEN(id, def)                                                           \
  (has_local_gens[id]                                                          \
       ? local_gens[id]                                                        \
       : (has_global_inst_gens[id]                                             \
              ? global_inst_gens[id]                                           \
              : (has_global_preset_gens[id] ? global_preset_gens[id]           \
                                            : (int16_t)(def))))

      int16_t coarse = GEN(51, 0);
      int16_t fine = GEN(52, 0);
      int16_t root = GEN(58, -1);
      int16_t loop = GEN(54, 0);
      int16_t scale_t = GEN(56, 100);

      // SF2 Volume envelope (gens 33-38:
      // delay/attack/hold/decay/sustain/release)
      int16_t v_attack = GEN(34, -12000);  // attackVolEnv
      int16_t v_hold = GEN(35, -12000);    // holdVolEnv (unused in AHR)
      int16_t v_decay = GEN(36, -12000);   // decayVolEnv
      int16_t v_sustain = GEN(37, 0);      // sustainVolEnv (centibels)
      int16_t v_release = GEN(38, -12000); // releaseVolEnv
      (void)v_hold;

      // SF2 Modulation envelope (gens 21-26:
      // delay/attack/hold/decay/sustain/release)
      int16_t m_attack = GEN(22, -12000);  // attackModEnv
      int16_t m_hold = GEN(23, -12000);    // holdModEnv (unused)
      int16_t m_decay = GEN(24, -12000);   // decayModEnv
      int16_t m_sustain = GEN(25, 0);      // sustainModEnv
      int16_t m_release = GEN(26, -12000); // releaseModEnv
      int16_t m_to_pitch = GEN(27, 0);     // modEnvToPitch cents
      (void)m_hold;

      // SF2 per-zone tonal
      int16_t atten = GEN(48, 0); // initialAttenuation (centibels)
      int16_t flt_fc =
          GEN(8, 13500); // initialFilterFc (absCents, default open)

#undef GEN

      if (zone_count < max_out &&
          fill_resolved_sample(sample_idx, coarse, fine, scale_t, v_attack,
                               v_decay, v_sustain, v_release, m_attack, m_decay,
                               m_sustain, m_release, m_to_pitch, atten, flt_fc,
                               root, loop, out[zone_count])) {
        zone_count++;
      }
    }
  }
  return zone_count;
}

bool resolve_note(uint16_t bank, uint16_t preset, uint8_t key, uint8_t vel,
                  ResolvedParams &out) {
  return collect_note_zones(bank, preset, key, vel, &out, 1) > 0;
}

// Minimal test UI: down-click advances preset, up-click advances bank.

static const uint8_t scale_notes_count[5] = {12, 7, 7, 5, 5};
static const uint8_t scale_intervals[5][12] = {
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, // Chromatic
    {0, 2, 4, 5, 7, 9, 11},                 // Major
    {0, 2, 3, 5, 7, 8, 10},                 // Minor
    {0, 2, 4, 7, 9},                        // Major Pentatonic
    {0, 3, 5, 7, 10}                        // Minor Pentatonic
};

static inline int32_t quantize_to_scale(int32_t semitones, int32_t scale_idx) {
  if (scale_idx <= 0 || scale_idx >= 5) {
    return semitones; // Chromatic
  }

  int32_t octave = semitones / 12;
  int32_t note_in_octave = semitones % 12;
  if (note_in_octave < 0) {
    note_in_octave += 12;
    octave -= 1;
  }

  // Find closest interval in the scale
  const uint8_t *intervals = scale_intervals[scale_idx];
  uint8_t count = scale_notes_count[scale_idx];
  int32_t best_note = intervals[0];
  int32_t min_diff = 100;

  for (uint8_t i = 0; i < count; i++) {
    int32_t diff = note_in_octave - intervals[i];
    if (diff < 0)
      diff = -diff;
    if (diff < min_diff) {
      min_diff = diff;
      best_note = intervals[i];
    }
  }

  return octave * 12 + best_note;
}

// Knobs tracking
int32_t smoothMain = 0;
int32_t smoothX = 0;
int32_t smoothY = 0;

// High-speed DC-coupled CV modulators (scaled to Q15)
volatile int32_t audio_in1_q15 = 0;
volatile int32_t audio_in2_q15 = 0;

// Envelope modifier CV (patched into CV2 when Gate 2 is disconnected)
volatile int32_t cv2_decay_mod = 0;

extern "C" {
void handle_midi_message(uint8_t *packet, int size);
}

static uint32_t core0_rand_seed = 54321;
static inline uint32_t core0_rand() {
  core0_rand_seed = core0_rand_seed * 1103515245 + 12345;
  return (core0_rand_seed / 65536) % 32768;
}

static uint32_t rev_glitch_samples = 0;
static bool rev_gate_mute = false;
static bool rev_feedback_max = false;

// Hardware System Class
class RomplerApp : public ComputerCard {
public:
  // Public wrappers for protected ComputerCard methods called in main()
  bool PulseIn(int i) { return ComputerCard::PulseIn(i); }
  int16_t CVIn(int i) { return ComputerCard::CVIn(i); }
  int16_t CVIn1() { return ComputerCard::CVIn1(); }
  int16_t CVIn2() { return ComputerCard::CVIn2(); }
  void PulseOut1(bool val) { ComputerCard::PulseOut1(val); }
  void PulseOut2(bool val) { ComputerCard::PulseOut2(val); }
  bool Connected(Input i) { return debounced_connected[i]; }
  bool Disconnected(Input i) { return !debounced_connected[i]; }
  int32_t KnobVal(Knob ind) { return ComputerCard::KnobVal(ind); }
  Switch SwitchVal() { return ComputerCard::SwitchVal(); }
  void BootLed(int i, bool on) { LedOn(i, on); }

  void InitUSB() {
    sleep_ms(300);
    auto usb_power_state = USBPowerState();
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

  void __not_in_flash_func(service_audio_queue)() {
    if (get_core_num() == 1)
      return;

    AudioFrame frame;
    while (queue_try_remove(&core1_to_core0_queue, &frame)) {
      // 1. Reverb processing
      int32_t rev_out_L = frame.L;
      int32_t rev_out_R = frame.R;
      if (page_reverb_mix > 0) {
        int32_t rev_mix   = page_reverb_mix;
        int32_t rev_decay = 16000 + ((page_reverb_mix * 12000) >> 15);
        int32_t rev_damp  = 22000;

        // Glitch the reverb in the last third of the glitch knob
        if (param_glitch_mangle >= 23211) {
          uint32_t samples_per_beat = (midi_tempo_us * 48) / 1000;
          if (samples_per_beat == 0) samples_per_beat = 24000;
          uint32_t step_samples = samples_per_beat / 8; // 32nd note grid
          if (step_samples == 0) step_samples = 3000;

          rev_glitch_samples++;
          if (rev_glitch_samples >= step_samples) {
            rev_glitch_samples = 0;
            int32_t last_third_intensity = (param_glitch_mangle - 23211) * 100 / (32767 - 23211);
            
            // 30% to 70% probability of muting/gating the reverb depending on intensity
            uint32_t mute_prob = 20 + (last_third_intensity * 50 / 100);
            if ((core0_rand() % 100) < mute_prob) {
              rev_gate_mute = true;
            } else {
              rev_gate_mute = false;
            }

            // 2% to 10% probability of clearing reverb buffer (Init)
            uint32_t clear_prob = 2 + (last_third_intensity * 8 / 100);
            if ((core0_rand() % 100) < clear_prob) {
              reverb.Init();
            }

            // 10% to 30% probability of infinite reverb feedback (decay = 30000)
            uint32_t fb_prob = 10 + (last_third_intensity * 20 / 100);
            if ((core0_rand() % 100) < fb_prob) {
              rev_feedback_max = true;
            } else {
              rev_feedback_max = false;
            }
          }

          if (rev_gate_mute) {
            rev_mix = 0;
          }
          if (rev_feedback_max) {
            rev_decay = 30000;
          }
        } else {
          rev_glitch_samples = 0;
          rev_gate_mute = false;
          rev_feedback_max = false;
        }

        reverb.Process(rev_out_L, rev_out_R, rev_mix, rev_decay, rev_damp, 0,
                       false);
      }
      frame.L = rev_out_L;
      frame.R = rev_out_R;

      // 2. Filter processing (filters both the dry signal and the reverb tail/glitches)
      process_master_filter(frame.L, frame.R);

      // Push back to Core 1
      queue_try_add(&core0_to_core1_queue, &frame);
    }
  }

  // Core 0 Background Loop (USB Tasks, UI rendering, preset menu down-flicks)
  void BackgroundLoop() override {
    // Prevent concurrent execution of the background/UI logic on Core 1
    if (get_core_num() == 1)
      return;

    // Boot switch toggle check on Core 0
    static int boot_switch_down_count = 0;
    static bool boot_switch_checked = false;
    if (bootSilence > 0) {
      if (bootSilence < 43200 && bootSilence > 4800) {
        if (SwitchVal() == Switch::Down) {
          boot_switch_down_count++;
        }
      }
      if (bootSilence <= 4800 && !boot_switch_checked) {
        boot_switch_checked = true;
        if (boot_switch_down_count > 100) {
          bool next_mode = !midi_playback_mode;
          if (next_mode && num_midi_files == 0) {
            next_mode = false;
          }
          if (next_mode != midi_playback_mode) {
            midi_playback_mode = next_mode;
            apply_mode_configuration();
            save_persistent_settings(midi_playback_mode);
          }
        }
      }
    }

    // Process physical CV/Gate trigger requests from Core 1
    while (request_tail != request_head) {
      NoteRequest req = request_queue[request_tail];
      request_tail = (request_tail + 1) % REQUEST_QUEUE_SIZE;

      if (req.type == REQ_NOTE_ON) {
        push_note_on_event(req.note, req.velocity, req.bank, req.preset,
                           req.channel, req.apply_pitch_knob);
      } else if (req.type == REQ_NOTE_OFF) {
        push_note_off_event(req.note, req.channel);
      }
    }

    if (isUSBMIDIHost) {
      tuh_task();
    } else {
      tud_task();
      while (tud_midi_available()) {
        uint8_t packet[4];
        tud_midi_packet_read(packet);
        handle_midi_message(packet + 1, 3);
      }
    }

    // Tick the MIDI sequencer (only in dedicated MIDI playback mode)
    if (midi_playback_mode) {
      if (pulse2_trigger_occurred) {
        pulse2_trigger_occurred = false;
        queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0xFF);
        start_midi_playback(midi_files[selectedMidiIdx].data, midi_files[selectedMidiIdx].size);
      }

      if (pulse1_trigger_occurred) {
        pulse1_trigger_occurred = false;
        if (Connected(Input::Pulse1)) {
          uint64_t now_us = time_us_64();
          if (midi_last_pulse1_time > 0) {
            uint64_t dt = now_us - midi_last_pulse1_time;
            if (dt < 2000000) { // Only update tempo if pulse is under 2 seconds (BPM > 30)
              uint32_t target_tempo = dt * 4;
              if (midi_tempo_us == 0) {
                midi_tempo_us = target_tempo;
              } else {
                midi_tempo_us = midi_tempo_us + ((target_tempo - midi_tempo_us) >> 3);
              }
            }
          }
          midi_last_pulse1_time = now_us;

          // First, align sequencer to the current boundary (catch up if behind)
          if (midi_sequence_ticks < midi_next_boundary_tick) {
            uint32_t ticks_to_step = midi_next_boundary_tick - midi_sequence_ticks;
            step_midi_sequencer_ticks(ticks_to_step);
          }
          
          // Now, set the boundary to the next 16th note
          uint32_t step_ticks = midi_division / 4;
          if (step_ticks < 1) step_ticks = 1;
          
          if (midi_next_boundary_tick <= midi_sequence_ticks) {
            midi_next_boundary_tick = ((midi_sequence_ticks / step_ticks) + 1) * step_ticks;
          } else {
            midi_next_boundary_tick += step_ticks;
          }
        }
      }
    }

    if (mclk_trigger_occurred) {
      mclk_trigger_occurred = false;
      if (!midi_playback_mode || Disconnected(Input::Pulse1)) {
        static uint64_t last_mclk_time = 0;
        uint64_t now_us = time_us_64();
        if (last_mclk_time > 0) {
          uint64_t dt = now_us - last_mclk_time;
          if (dt < 1000000) { // Only update tempo if pulse is under 1 second (BPM > 60)
            uint32_t target_tempo = dt * 24;
            if (midi_tempo_us == 0) {
              midi_tempo_us = target_tempo;
            } else {
              midi_tempo_us = midi_tempo_us + ((target_tempo - midi_tempo_us) >> 3);
            }
          }
        }
        last_mclk_time = now_us;

        // First, align sequencer to the current boundary (catch up if behind)
        if (midi_sequence_ticks < midi_next_boundary_tick) {
          uint32_t ticks_to_step = midi_next_boundary_tick - midi_sequence_ticks;
          if (midi_playback_mode) {
            step_midi_sequencer_ticks(ticks_to_step);
          } else {
            // Just advance ticks and boundaries in Normal Mode
            uint32_t len = 16 * midi_division;
            for (uint32_t step = 0; step < ticks_to_step; step++) {
              midi_sequence_ticks = (midi_sequence_ticks + 1) % len;
              if (midi_sequence_ticks % midi_division == 0) {
                midi_last_beat_counter = midi_sequence_ticks / midi_division;
                if (midi_sequence_ticks % (4 * midi_division) == 0) {
                  if (!freeze_active) {
                    midi_bar_start_tick = midi_sequence_ticks;
                  }
                }
                if (!freeze_active) {
                  midi_beat_start_tick = midi_sequence_ticks;
                }
              }
            }
          }
        }
        
        // Now, set the boundary to the next 24 PPQN step
        uint32_t step_ticks = midi_division / 24;
        if (step_ticks < 1) step_ticks = 1;
        
        if (midi_next_boundary_tick <= midi_sequence_ticks) {
          midi_next_boundary_tick = ((midi_sequence_ticks / step_ticks) + 1) * step_ticks;
        } else {
          midi_next_boundary_tick += step_ticks;
        }
      }
    }

    // Tick the MIDI sequencer/clock continuously for tape loop recording and playback in all modes
    tick_midi_sequencer();

    if (bootSilence > 0) {
      lockMain.engage(KnobVal(Knob::Main));
      lockX.engage(KnobVal(Knob::X));
      lockY.engage(KnobVal(Knob::Y));
      return;
    }

    static uint32_t last_ui_tick = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_ui_tick < 15)
      return; // 60Hz update rate
    last_ui_tick = now;

    static bool last_freeze_active = false;
    if (freeze_active != last_freeze_active) {
      last_freeze_active = freeze_active;
      lockMain.engage(KnobVal(Knob::Main));
      lockX.engage(KnobVal(Knob::X));
      lockY.engage(KnobVal(Knob::Y));
      if (!freeze_active) {
        glitch_speed_factor_q15 = 32768;
        param_glitch_mangle = 0;
        param_freeze_detune = 0; // reset to 0 (no glitch)
        memset(active_transpositions, 0, sizeof(active_transpositions));
        memset(sequencer_ratchets, 0, sizeof(sequencer_ratchets));
      }
    }

    static bool first_run = true;
    if (first_run) {
      lockMain.engage(KnobVal(Knob::Main));
      lockX.engage(KnobVal(Knob::X));
      lockY.engage(KnobVal(Knob::Y));
      // Initialize channel bank/presets default, default channel 9 to drum bank
      // 128
      for (int i = 0; i < 16; i++) {
        if (i == 9) {
          channel_banks[i] = 128;
          channel_presets[i] = 0;
        } else {
          channel_banks[i] = master_bank;
          channel_presets[i] = master_preset;
        }
      }
      first_run = false;
    }

    static Switch last_sw = Switch::Middle;
    static uint32_t sw_press_time = 0;
    static bool unfrozen_on_press = false;
    Switch sw = SwitchVal();

    if (sw != last_sw) {
      uint32_t now_ms = to_ms_since_boot(get_absolute_time());
      if (last_sw == Switch::Middle) {
        // Transition Middle -> Up or Middle -> Down
        sw_press_time = now_ms;
        if (sw == Switch::Down) {
          unfrozen_on_press = false;
          if (freeze_active) {
            // Instant Unfreeze!
            freeze_active = false;
            unfrozen_on_press = true;
            if (midi_playback_mode) {
              // Unfreeze alignment: restore track states to freeze start
              for (uint32_t t = 0; t < midi_num_tracks; t++) {
                midi_tracks[t].ptr = midi_track_freeze_states[t].ptr;
                midi_tracks[t].ticks_to_next = midi_track_freeze_states[t].ticks_to_next;
                midi_tracks[t].running_status = midi_track_freeze_states[t].running_status;
                midi_tracks[t].active = midi_track_freeze_states[t].active;
              }
              midi_sequence_ticks = midi_freeze_start_tick;
            } else {
              // Normal Mode unfreeze
              midi_sequence_ticks = midi_freeze_start_tick;
            }
            elapsed_ticks = 0;
            queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0xFF);
            stutter_volume = 32767;
            param_freeze_detune = 0; // reset to 0 (no glitch)
            memset(active_transpositions, 0, sizeof(active_transpositions));
            memset(sequencer_ratchets, 0, sizeof(sequencer_ratchets));
          }
        }
      } else if (sw == Switch::Middle) {
        // Transition Up -> Middle or Down -> Middle
        uint32_t duration = now_ms - sw_press_time;

        if (last_sw == Switch::Down) {
          // Long Down release: commit queued pattern selection
          if (midi_playback_mode && duration >= 500) {
            pending_midi_idx = selected_pattern_knob_idx;
          } else if (duration < 500 && !unfrozen_on_press) {
            // Short Down release (not an unfreeze): Toggle Freeze ON
            freeze_active = true;
            if (midi_playback_mode) {
              // Freeze instantly on the current active state
              for (uint32_t t = 0; t < midi_num_tracks; t++) {
                midi_track_freeze_states[t].ptr = midi_tracks[t].ptr;
                midi_track_freeze_states[t].ticks_to_next = midi_tracks[t].ticks_to_next;
                midi_track_freeze_states[t].running_status = midi_tracks[t].running_status;
                midi_track_freeze_states[t].active = midi_tracks[t].active;
              }
              midi_freeze_start_tick = midi_sequence_ticks;
            } else {
              // Normal Mode freeze
              midi_freeze_start_tick = midi_sequence_ticks;
            }

            param_stutter = 2048; // center position: 1-bar loop
            param_freeze_detune = 0; // reset to 0 (no glitch)
            uint32_t freeze_len = get_stutter_length();
            uint32_t len = 16 * midi_division;
            uint32_t start_tick = (midi_sequence_ticks + len - freeze_len) % len;
            param_scrub = (start_tick * 4095) / len;

            lockMain.engage(KnobVal(Knob::Main));
            lockX.engage(KnobVal(Knob::X));
            lockY.engage(KnobVal(Knob::Y));

            elapsed_ticks = 0;
            stutter_volume = 32767;
            queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0xFF);
          }
        } else if (last_sw == Switch::Up) {
          // Up release (short flick): Toggle Play/Stop (only in MIDI mode)
          if (midi_playback_mode && duration < 500) {
            if (midi_playing) {
              midi_playing = false;
              stop_all_notes_and_gates();
            } else {
              start_midi_playback(midi_files[selectedMidiIdx].data, midi_files[selectedMidiIdx].size);
            }
          }
        }
        unfrozen_on_press = false;
      }

      // Engage locks on page transition to prevent value jumps
      lockMain.engage(KnobVal(Knob::Main));
      lockX.engage(KnobVal(Knob::X));
      lockY.engage(KnobVal(Knob::Y));
      last_sw = sw;
    }

    // Confirmation animation for playback modes
    static bool confirmed_mode = false;
    if (bootSilence == 0 && !confirmed_mode) {
      // Confirm mode by flashing LEDs
      for (int flash = 0; flash < 3; flash++) {
        if (midi_playback_mode) {
          LedOn(4);
          LedOn(5);
        } else {
          LedOn(0);
          LedOn(1);
        }
        sleep_ms(60);
        if (midi_playback_mode) {
          LedOff(4);
          LedOff(5);
        } else {
          LedOff(0);
          LedOff(1);
        }
        sleep_ms(60);
      }
      confirmed_mode = true;

      // Auto-start MIDI playback on the first midi file if available
      if (midi_playback_mode && num_midi_files > 0) {
        start_midi_playback(midi_files[selectedMidiIdx].data,
                            midi_files[selectedMidiIdx].size);
      }
    }

    int current_preset_idx = 0;
    // Find current selectedPreset's flat index in presets_index list to display
    // on LEDs
    for (uint32_t i = 0; i < indexed_presets_count; i++) {
      if (presets_index[i].preset == selectedPreset &&
          presets_index[i].bank == activeBank) {
        current_preset_idx = i;
        break;
      }
    }

    if (midi_playback_mode) {
      int32_t raw_main = KnobVal(Knob::Main);
      int32_t raw_x = KnobVal(Knob::X);
      int32_t raw_y = KnobVal(Knob::Y);

      if (sw == Switch::Down) {
        // Switch Down Page: Main = Pattern Select (pre-selected), X = Tempo, Y = Mutate
        if (lockMain.update(raw_main) && num_midi_files > 0) {
          int32_t idx = (raw_main * num_midi_files) / 4096;
          if (idx >= (int32_t)num_midi_files)
            idx = num_midi_files - 1;
          if (idx < 0)
            idx = 0;
          selected_pattern_knob_idx = idx;
        }
        if (lockX.update(raw_x)) {
          // Tempo from 40 to 280 BPM
          uint32_t bpm = 40 + (raw_x * 240) / 4095;
          if (Disconnected(Input::Pulse1)) {
            midi_tempo_us = 60000000 / bpm;
          }
        }
        if (lockY.update(raw_y)) {
          // Mutate value: 0..32767
          param_mutate = (raw_y * 32767) / 4095;
        }
      } else if (sw == Switch::Up) {
        // Switch Up Page: Main = Filter Cutoff, X = Glitch Mangle, Y = Reverb
        if (lockMain.update(raw_main)) {
          param_cutoff = (raw_main * 32767) / 4095;
        }
        if (lockX.update(raw_x)) {
          param_glitch_mangle = (raw_x * 32767) / 4095;
        }
        if (lockY.update(raw_y)) {
          param_reverb_mix = (raw_y * 32767) / 4095;
        }
      } else {
        // Switch Middle Page: Main = Transpose, X = Decay 1, Y = Decay 2 (when not frozen)
        // If frozen: Main = Stutter Division, X = Scrub, Y = Feedback
        if (freeze_active) {
          // Main controls speed/stutter, X controls scrub, Y controls feedback
          if (lockMain.update(raw_main)) {
            param_stutter = raw_main;
          }
          if (lockX.update(raw_x)) {
            param_scrub = raw_x;
          }
          if (lockY.update(raw_y)) {
            param_freeze_detune = (raw_y * 32767) / 4095; // 0..32767, unipolar
          }
        } else {
          if (lockMain.update(raw_main)) {
            midi_transpose = -24 + (raw_main * 48) / 4095;
          }
          if (lockX.update(raw_x)) {
            param_release = (raw_x * 32767) / 4095;
            param_attack = 0; // fast attack for performance!
          }
          if (lockY.update(raw_y)) {
            param_drum_release = (raw_y * 32767) / 4095;
          }
        }
      }

      // Precompute dynamic values from parameters
      page_cutoff = -12000 + (param_cutoff * 22000 / 32767);
      page_res = 0; // resonance fixed to 0 in MIDI performance mode to avoid explosions / keep it simple
      page_attack = -12000 + (param_attack * 15000 / 32767);
      page_decay = -12000 + (param_release * 15000 / 32767); // decay follows release
      page_release = -12000 + (param_release * 15000 / 32767);
      page_coarse = midi_transpose;
      page_reverb_mix = param_reverb_mix;

      uint32_t attack_ms = timecents_to_ms(page_attack);
      global_env_attack_step = (2147483647LL * 32) / (48 * attack_ms);
      if (global_env_attack_step < 1)
        global_env_attack_step = 1;

      uint32_t release_ms = timecents_to_ms(page_release);
      global_env_release_step = (2147483647LL * 32) / (48 * release_ms);
      if (global_env_release_step < 1)
        global_env_release_step = 1;

      // Drum decay calculation
      int32_t page_drum_release = -12000 + (param_drum_release * 15000 / 32767);
      uint32_t drum_release_ms = timecents_to_ms(page_drum_release);
      global_drum_env_release_step = (2147483647LL * 32) / (48 * drum_release_ms);
      if (global_drum_env_release_step < 1)
        global_drum_env_release_step = 1;

      // Update LEDs in MIDI Playback Mode
      if (bootSilence == 0) {
        if (sw == Switch::Down) {
          // Display selected pattern preview in binary
          int32_t active_idx = selected_pattern_knob_idx;
          for (int i = 0; i < 4; i++) {
            LedOn(i, ((active_idx + 1) >> i) & 1);
          }
          LedOn(4, midi_playing);
          LedOn(5, freeze_active);
        } else if (sw == Switch::Middle && freeze_active) {
          int32_t raw_main = KnobVal(Knob::Main);
          int32_t div_idx = 0;
          if (raw_main >= 2048) {
            int32_t val = raw_main - 2048;
            div_idx = (val * 7) >> 11;
          }
          if (div_idx < 0) div_idx = 0;
          if (div_idx > 6) div_idx = 6;
          for (int i = 0; i < 3; i++) {
            LedOn(i, (div_idx >> i) & 1);
          }
          LedOff(3);
          
          bool tempo_on = midi_playing && ((midi_sequence_ticks % midi_division) < (midi_division / 4));
          LedOn(4, tempo_on);
          
          bool led5_on = (now % 200 < 100);
          LedOn(5, led5_on);
        } else {
          // Switch Up and Switch Middle (when not frozen) use the performance gate & status display
          int active_t0 = 0;
          int active_t1 = 0;
          for (int i = 0; i < MAX_VOICES; i++) {
            if (voices[i].state != VOICE_OFF &&
                voices[i].state != VOICE_RELEASE) {
              int32_t ch_target = global_config.channels[voices[i].channel].target;
              if (ch_target == 0 || ch_target == 5) {
                active_t0 = 1;
              }
              if (ch_target == 1 || ch_target == 6 || voices[i].channel == 9) {
                active_t1 = 1;
              }
            }
          }

          // LED 0: Internal Sequencer Gate/Activity
          LedOn(0, active_t0);

          // LED 1: Internal Drums Gate/Activity
          LedOn(1, active_t1);

          // LED 2: Pulse 1 Out Gate
          LedOn(2, cv1_out_gate);

          // LED 3: Pulse 2 Out Gate
          LedOn(3, cv2_out_gate);

          // LED 4: Tempo LED (flashes at beat boundaries)
          bool tempo_on = midi_playing && ((midi_sequence_ticks % midi_division) < (midi_division / 4));
          LedOn(4, tempo_on);

          // LED 5: Stopped (OFF), Playing (ON), Freeze (slow blink), Play-Once (double-blink), Queued (fast blink)
          bool led5_on = false;
          if (midi_playing) {
            if (freeze_active) {
              led5_on = (now % 200 < 100);
            } else if (global_config.loop_mode == 1) {
              uint32_t phase = now % 1000;
              led5_on = (phase < 100) || (phase >= 200 && phase < 300);
            } else if (pending_midi_idx != selectedMidiIdx) {
              led5_on = (now % 100 < 50);
            } else {
              led5_on = true;
            }
          } else {
            if (global_config.loop_mode == 1) {
              led5_on = (now % 600 < 100); // sparse pulse: play-once armed, stopped
            } else if (pending_midi_idx != selectedMidiIdx) {
              led5_on = (now % 400 < 200);
            }
          }
          LedOn(5, led5_on);
        }
      }
    } else {
      // NORMAL MODE
      if (sw == Switch::Down) {
        int32_t raw_main = KnobVal(Knob::Main);
        int32_t raw_x = KnobVal(Knob::X);
        int32_t raw_y = KnobVal(Knob::Y);

        if (lockMain.update(raw_main) && indexed_presets_count > 0) {
          int preset_idx = (raw_main * indexed_presets_count) / 4096;
          if (preset_idx >= (int)indexed_presets_count)
            preset_idx = indexed_presets_count - 1;
          if (preset_idx < 0)
            preset_idx = 0;
          if (presets_index[preset_idx].preset != selectedPreset ||
              presets_index[preset_idx].bank != activeBank) {
            selectedPreset = presets_index[preset_idx].preset;
            activeBank = presets_index[preset_idx].bank;
            master_preset = selectedPreset;
            master_bank = activeBank;
            channel_banks[0] = master_bank;
            channel_presets[0] = master_preset;
            queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0);
          }
          current_preset_idx = preset_idx;
        }

        if (lockX.update(raw_x)) {
          // 5 scales (0: Chromatic, 1: Major, 2: Minor, 3: Major Pentatonic, 4:
          // Minor Pentatonic)
          int32_t scale_idx = (raw_x * 5) / 4096;
          if (scale_idx >= 5)
            scale_idx = 4;
          if (scale_idx < 0)
            scale_idx = 0;
          active_scale = scale_idx;
        }

        if (lockY.update(raw_y) && indexed_presets_count > 0) {
          int32_t new_y_mode = 0;
          int32_t new_preset_b = 0;

          if (raw_y < 410) {
            // 1. Duophony (first 10%)
            new_y_mode = 0;
          } else {
            // 2. Dual Preset B (remaining 90%)
            new_y_mode = 1;
            int32_t range_len = 4096 - 410;
            int32_t idx =
                ((raw_y - 410) * (int32_t)indexed_presets_count) / range_len;
            if (idx >= (int32_t)indexed_presets_count)
              idx = (int32_t)indexed_presets_count - 1;
            if (idx < 0)
              idx = 0;
            new_preset_b = idx;
          }

          bool changed = false;
          if (new_y_mode != current_y_mode) {
            current_y_mode = new_y_mode;
            changed = true;
          }
          if (current_y_mode == 1 && new_preset_b != selectedPresetB) {
            selectedPresetB = new_preset_b;
            changed = true;
          }

          if (changed) {
            queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0);

            if (current_y_mode == 1) {
              if (presets_index != nullptr) {
                master_bank2 = presets_index[selectedPresetB].bank;
                master_preset2 = presets_index[selectedPresetB].preset;
              } else {
                master_bank2 = 0;
                master_preset2 = 0;
              }
              channel_banks[1] = master_bank2;
              channel_presets[1] = master_preset2;
            }
          }
        }
      } else {
        // Switch Up or Switch Middle (Normal page mapping)
        int32_t raw_main = KnobVal(Knob::Main);
        int32_t raw_x = KnobVal(Knob::X);
        int32_t raw_y = KnobVal(Knob::Y);

        if (freeze_active && sw == Switch::Middle) {
          // Main controls speed/stutter, X controls scrub, Y controls feedback
          if (lockMain.update(raw_main)) {
            param_stutter = raw_main;
          }
          if (lockX.update(raw_x)) {
            param_scrub = raw_x;
          }
          if (lockY.update(raw_y)) {
            param_freeze_detune = (raw_y * 32767) / 4095; // 0..32767, unipolar
          }
        } else {
          if (sw == Switch::Middle) {
            // Page 2: Middle (Pitch, Decay 1, Decay 2)
            if (lockMain.update(raw_main)) {
              param_pitch = (raw_main * 32767) / 4095;
            }
            if (lockX.update(raw_x)) {
              param_release = (raw_x * 32767) / 4095;
              param_attack = 0; // fast attack
            }
            if (lockY.update(raw_y)) {
              param_drum_release = (raw_y * 32767) / 4095;
            }
          } else {
            // Page 1: Up (Cutoff, Glitch Mangle, Reverb Mix)
            if (lockMain.update(raw_main)) {
              param_cutoff = (raw_main * 32767) / 4095;
            }
            if (lockX.update(raw_x)) {
              param_glitch_mangle = (raw_x * 32767) / 4095;
            }
            if (lockY.update(raw_y)) {
              param_reverb_mix = (raw_y * 32767) / 4095;
            }
          }
        }
      }

      // Precompute dynamic values from parameters
      page_cutoff = -12000 + (param_cutoff * 22000 / 32767);
      page_res = 0; // resonance fixed to 0 to align with MIDI mode
      page_attack = -12000 + (param_attack * 15000 / 32767);
      page_decay = -12000 + (param_release * 15000 / 32767); // decay follows release
      page_release = -12000 + (param_release * 15000 / 32767);
      page_coarse = -36 + (param_pitch * 72) / 32767;
      page_reverb_mix = param_reverb_mix;

      uint32_t attack_ms = timecents_to_ms(page_attack);
      global_env_attack_step = (2147483647LL * 32) / (48 * attack_ms);
      if (global_env_attack_step < 1)
        global_env_attack_step = 1;

      uint32_t release_ms = timecents_to_ms(page_release);
      global_env_release_step = (2147483647LL * 32) / (48 * release_ms);
      if (global_env_release_step < 1)
        global_env_release_step = 1;

      // Drum decay calculation
      int32_t page_drum_release = -12000 + (param_drum_release * 15000 / 32767);
      uint32_t drum_release_ms = timecents_to_ms(page_drum_release);
      global_drum_env_release_step = (2147483647LL * 32) / (48 * drum_release_ms);
      if (global_drum_env_release_step < 1)
        global_drum_env_release_step = 1;

      // Normal mode LEDs output
      if (bootSilence == 0) {
        if (sw == Switch::Middle && freeze_active) {
          // Display the stutter division index in binary on LEDs 0..2
          int32_t raw_main = KnobVal(Knob::Main);
          int32_t div_idx = 0;
          if (raw_main >= 2048) {
            int32_t val = raw_main - 2048;
            div_idx = (val * 7) >> 11;
          }
          if (div_idx < 0) div_idx = 0;
          if (div_idx > 6) div_idx = 6;
          for (int i = 0; i < 3; i++) {
            LedOn(i, (div_idx >> i) & 1);
          }
          LedOff(3);
          LedOn(4, 0);
          LedOn(5, 1); // Freeze status indicator
        } else if (sw == Switch::Middle) {
          // LEDs 0..3: Show active note pitch (newest ch0 voice), mapped 0..11
          // semitones around C LEDs 4..5: Active gate indicators for channel 0
          // and channel 1
          int32_t newest_note = -1;
          uint32_t newest_age = 0;
          int active_ch0 = 0, active_ch1 = 0;
          for (int i = 0; i < MAX_VOICES; i++) {
            if (voices[i].state != VOICE_OFF &&
                voices[i].state != VOICE_RELEASE) {
              if (voices[i].channel == 0) {
                active_ch0 = 1;
                if (voices[i].age > newest_age) {
                  newest_age = voices[i].age;
                  newest_note = voices[i].midi_note;
                }
              } else if (voices[i].channel == 1) {
                active_ch1 = 1;
              }
            }
          }
          // LEDs 0-3: pitch class of newest active ch0 note (semitone within
          // octave, displayed in 4-bit binary 0-11)
          int32_t pitch_class =
              (newest_note >= 0) ? ((newest_note + page_coarse) % 12) : 0;
          if (pitch_class < 0)
            pitch_class += 12;
          for (int i = 0; i < 4; i++) {
            LedOn(i, (pitch_class >> i) & 1);
          }
          LedOn(4, active_ch0);
          LedOn(5, active_ch1);
        } else {
          if (sw == Switch::Down && current_y_mode == 1) {
            for (int i = 0; i < 6; i++) {
              LedOn(i, (selectedPresetB >> i) & 1);
            }
          } else {
            for (int i = 0; i < 6; i++) {
              LedOn(i, (current_preset_idx >> i) & 1);
            }
          }
        }
      }
    }

    // --- Preset CV Offset logic ---
    if (!midi_playback_mode) {
      if (Disconnected(Input::Pulse2) && Connected(Input::Audio2) &&
          sw != Switch::Down) {
        int32_t raw_cv = AudioIn2();
        int32_t offset = raw_cv / 128; // scale -2048..2047 to -16..15
        int32_t target_idx = current_preset_idx + offset;
        if (target_idx < 0)
          target_idx = 0;
        if (target_idx >= (int)indexed_presets_count)
          target_idx = indexed_presets_count - 1;

        if (presets_index[target_idx].preset != master_preset ||
            presets_index[target_idx].bank != master_bank) {
          master_bank = presets_index[target_idx].bank;
          master_preset = presets_index[target_idx].preset;
          channel_banks[0] = master_bank;
          channel_presets[0] = master_preset;
          queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0);
        }
      } else {
        // If Audio 2 is disconnected, or we are in Switch::Down (manual
        // select), or Pulse 2 is connected, follow the manual selection
        // (selectedPreset, activeBank)
        if (master_preset != selectedPreset || master_bank != activeBank) {
          master_preset = selectedPreset;
          master_bank = activeBank;
          channel_banks[0] = master_bank;
          channel_presets[0] = master_preset;
          queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0);
        }
      }
    }
  }

  static inline int32_t quantize_semitone(int32_t raw_pitch_q8) {
    int32_t rounded;
    if (raw_pitch_q8 >= 0) {
      rounded = (raw_pitch_q8 + 128) >> 8;
    } else {
      rounded = (raw_pitch_q8 - 128) >> 8;
    }
    return rounded;
  }

  void __not_in_flash_func(process_master_filter)(int32_t &mixed_L,
                                                  int32_t &mixed_R) {
    int32_t cutoff_val = param_cutoff;

    // Smooth cutoff_val with a one-pole to prevent zipper noise and sudden SVF
    // state jumps
    static int32_t smooth_cutoff = 16384; // starts at centre (bypass)
    smooth_cutoff +=
        (cutoff_val - smooth_cutoff) >> 5; // ~32-sample slew (sub-ms)

    // Calculate wet_gain for crossfading
    int32_t lpf_wet_gain = 0;
    int32_t hpf_wet_gain = 0;

    if (smooth_cutoff < 13000) {
      lpf_wet_gain = 32767;
    } else if (smooth_cutoff >= 13000 && smooth_cutoff < 15500) {
      // Fade down from 32767 to 0
      lpf_wet_gain = 32767 - ((smooth_cutoff - 13000) * 32767) / 2500;
    }

    if (smooth_cutoff > 17200 && smooth_cutoff <= 19700) {
      // Fade up from 0 to 32767
      hpf_wet_gain = ((smooth_cutoff - 17200) * 32767) / 2500;
    } else if (smooth_cutoff > 19700) {
      hpf_wet_gain = 32767;
    }

    // Calculate filter coefficient f and damping q
    // LPF mode: map 0..15500 to f_coeff [100 .. 20000]
    int32_t val_lpf = (smooth_cutoff > 15500) ? 15500 : smooth_cutoff;
    int32_t f_coeff_lpf = 100 + (val_lpf * 19900) / 15500;

    // HPF mode: map 17200..32767 to f_coeff [60 .. 18000]
    int32_t val_hpf = (smooth_cutoff < 17200) ? 17200 : smooth_cutoff;
    int32_t f_coeff_hpf = 60 + ((val_hpf - 17200) * 17940) / (32767 - 17200);

    // Hard caps: Chamberlin SVF stable limit at 4x oversampling (f_oversampled < ~8000)
    if (f_coeff_lpf > 28000)
      f_coeff_lpf = 28000;
    if (f_coeff_hpf > 28000)
      f_coeff_hpf = 28000;

    // Damping: maps param_res (0..32767) to damping coefficient (Q16)
    int32_t q_coeff = 131072 - (param_res * (131072 - 8192)) / 32767;
    if (q_coeff < 8192)
      q_coeff = 8192;

    // For 4x oversampling, we divide the frequency coefficient f_coeff by 4.
    int32_t f_oversampled_lpf = f_coeff_lpf >> 2;
    if (f_oversampled_lpf > 7000)
      f_oversampled_lpf = 7000; // Hard SVF stability cap

    int32_t f_oversampled_hpf = f_coeff_hpf >> 2;
    if (f_oversampled_hpf > 7000)
      f_oversampled_hpf = 7000; // Hard SVF stability cap

    // Process LPF channels
    int32_t input_L = mixed_L << 8;
    int32_t input_R = mixed_R << 8;

    for (int step = 0; step < 4; step++) {
      lpf_filter.low_L += (f_oversampled_lpf * (lpf_filter.band_L >> 8)) >> 8;
      if (lpf_filter.low_L > 134217728)
        lpf_filter.low_L = 134217728;
      else if (lpf_filter.low_L < -134217728)
        lpf_filter.low_L = -134217728;

      int32_t high_L = input_L - lpf_filter.low_L -
                       ((q_coeff * (lpf_filter.band_L >> 8)) >> 8);
      lpf_filter.band_L += (f_oversampled_lpf * (high_L >> 8)) >> 8;
      if (lpf_filter.band_L > 134217728)
        lpf_filter.band_L = 134217728;
      else if (lpf_filter.band_L < -134217728)
        lpf_filter.band_L = -134217728;

      lpf_filter.low_R += (f_oversampled_lpf * (lpf_filter.band_R >> 8)) >> 8;
      if (lpf_filter.low_R > 134217728)
        lpf_filter.low_R = 134217728;
      else if (lpf_filter.low_R < -134217728)
        lpf_filter.low_R = -134217728;

      int32_t high_R = input_R - lpf_filter.low_R -
                       ((q_coeff * (lpf_filter.band_R >> 8)) >> 8);
      lpf_filter.band_R += (f_oversampled_lpf * (high_R >> 8)) >> 8;
      if (lpf_filter.band_R > 134217728)
        lpf_filter.band_R = 134217728;
      else if (lpf_filter.band_R < -134217728)
        lpf_filter.band_R = -134217728;
    }

    // Process HPF channels
    for (int step = 0; step < 4; step++) {
      hpf_filter.low_L += (f_oversampled_hpf * (hpf_filter.band_L >> 8)) >> 8;
      if (hpf_filter.low_L > 134217728)
        hpf_filter.low_L = 134217728;
      else if (hpf_filter.low_L < -134217728)
        hpf_filter.low_L = -134217728;

      int32_t high_L = input_L - hpf_filter.low_L -
                       ((q_coeff * (hpf_filter.band_L >> 8)) >> 8);
      hpf_filter.band_L += (f_oversampled_hpf * (high_L >> 8)) >> 8;
      if (hpf_filter.band_L > 134217728)
        hpf_filter.band_L = 134217728;
      else if (hpf_filter.band_L < -134217728)
        hpf_filter.band_L = -134217728;

      hpf_filter.low_R += (f_oversampled_hpf * (hpf_filter.band_R >> 8)) >> 8;
      if (hpf_filter.low_R > 134217728)
        hpf_filter.low_R = 134217728;
      else if (hpf_filter.low_R < -134217728)
        hpf_filter.low_R = -134217728;

      int32_t high_R = input_R - hpf_filter.low_R -
                       ((q_coeff * (hpf_filter.band_R >> 8)) >> 8);
      hpf_filter.band_R += (f_oversampled_hpf * (high_R >> 8)) >> 8;
      if (hpf_filter.band_R > 134217728)
        hpf_filter.band_R = 134217728;
      else if (hpf_filter.band_R < -134217728)
        hpf_filter.band_R = -134217728;
    }

    // Resolve filtered outputs
    int32_t filtered_lpf_L = lpf_filter.low_L >> 8;
    int32_t filtered_lpf_R = lpf_filter.low_R >> 8;

    int32_t out_hpf_L = input_L - hpf_filter.low_L -
                    ((q_coeff * (hpf_filter.band_L >> 8)) >> 8);
    int32_t out_hpf_R = input_R - hpf_filter.low_R -
                    ((q_coeff * (hpf_filter.band_R >> 8)) >> 8);
    int32_t filtered_hpf_L = out_hpf_L >> 8;
    int32_t filtered_hpf_R = out_hpf_R >> 8;

    // Smoothly crossfade between input (dry) and filtered (wet)
    mixed_L = mixed_L + (((filtered_lpf_L - mixed_L) * lpf_wet_gain) >> 15)
                      + (((filtered_hpf_L - mixed_L) * hpf_wet_gain) >> 15);
    mixed_R = mixed_R + (((filtered_lpf_R - mixed_R) * lpf_wet_gain) >> 15)
                      + (((filtered_hpf_R - mixed_R) * hpf_wet_gain) >> 15);
  }

  static inline int32_t get_sine_q15(uint8_t phase_8bit) {
    static const int16_t sin_table_q15[65] = {
        0, 804, 1608, 2410, 3212, 4011, 4808, 5602,
        6393, 7179, 7962, 8739, 9512, 10278, 11039, 11793,
        12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530,
        18204, 18868, 19520, 20160, 20787, 21403, 22005, 22594,
        23170, 23731, 24278, 24811, 25329, 25832, 26320, 26792,
        27248, 27688, 28113, 28520, 28912, 29287, 29645, 29986,
        30310, 30616, 30905, 31176, 31429, 31664, 31880, 32078,
        32257, 32418, 32560, 32683, 32728, 32754, 32762, 32767,
        32767
    };
    uint8_t idx = phase_8bit & 0x7F; // map to 0..127
    if (idx > 64) {
      idx = 128 - idx;
    }
    int32_t val = sin_table_q15[idx];
    if (phase_8bit >= 128) {
      return -val;
    }
    return val;
  }

  // Core 1 Audio DSP Callback (Runs strictly at 48kHz, optimized integer-only
  // DSP)
  void __not_in_flash_func(ProcessSample)() override {
    // ── Jack connection debouncing (48kHz) ──
    for (int i = 0; i < 6; i++) {
      bool raw_conn = ComputerCard::Connected((ComputerCard::Input)i);
      if (raw_conn) {
        if (connection_debounce_counter[i] < 480) { // 10ms debounce at 48kHz
          connection_debounce_counter[i]++;
          if (connection_debounce_counter[i] == 480) {
            debounced_connected[i] = true;
          }
        }
      } else {
        if (connection_debounce_counter[i] > 0) {
          connection_debounce_counter[i]--;
          if (connection_debounce_counter[i] == 0) {
            debounced_connected[i] = false;
          }
        }
      }
    }

    static int32_t gate_vol = 32767;
    static uint8_t sub_sample_counter = 0;
    sub_sample_counter++;
    bool update_envelopes = (sub_sample_counter >= 32);
    if (update_envelopes) {
      sub_sample_counter = 0;
    }

    auto clamp_audio = [](int32_t x) -> int16_t {
      if (x > 2047)
        return 2047;
      if (x < -2048)
        return -2048;
      return (int16_t)x;
    };

    // ── Boot Silence / Startup Animation logic ──
    if (bootSilence > 0) {
      bootSilence--;
      AudioOut1(0);
      AudioOut2(0);

      // Run circular chasing animation on LEDs 0..5
      int chaseIdx = (48000 - bootSilence) / 4000; // 0 to 11
      for (int i = 0; i < 6; i++) {
        LedOn(i, (i == (chaseIdx % 6)) || midi_playback_mode);
      }

      // Startup switch check is handled on Core 0 now

      if (bootSilence == 0) {
        for (int i = 0; i < 6; i++) {
          LedOff(i);
        }
      }
      return;
    }

    // ── Pulse 1 (16th note clock trigger) in MIDI Playback Mode ──
    static bool last_pulse1_val = false;
    if (midi_playback_mode && Connected(Input::Pulse1)) {
      bool p1_now = PulseIn1();
      if (p1_now && !last_pulse1_val) {
        pulse1_trigger_occurred = true;
      }
      last_pulse1_val = p1_now;
    }

    // ── Pulse 2 (Reset trigger) in MIDI Playback Mode ──
    static bool last_pulse2_reset_val = false;
    if (midi_playback_mode && Connected(Input::Pulse2)) {
      bool p2_now = PulseIn2();
      if (p2_now && !last_pulse2_reset_val) {
        pulse2_trigger_occurred = true;
      }
      last_pulse2_reset_val = p2_now;
    }

    // ── CV / Gate / Trigger Tracking (Core 1) ──
    static int32_t cv1_acc = 0;
    static int32_t cv2_acc = 0;

    static bool last_pulse1 = false;
    static bool last_pulse2 = false;

    static int32_t gate1_active_note = -1;
    static int32_t gate2_active_note = -1;

    if (!midi_playback_mode) {
      // Read and smooth CV inputs continuously
      int32_t cv1_raw = Connected(Input::CV1) ? CVIn1() : 0;
      cv1_acc = cv1_acc - (cv1_acc >> 8) + cv1_raw;
      int32_t cv1_smoothed = cv1_acc >> 8;

      bool p1_now = PulseIn1();
      bool p2_now = PulseIn2();

      if (!freeze_active) {
        if (p1_now && !last_pulse1) {
          // Gate 1 Rising Edge
          int32_t raw_pitch =
              Connected(Input::CV1) ? ((cv1_smoothed * 15) >> 1) : 0;
          int32_t semitone_offset = quantize_semitone(raw_pitch);
          int32_t note;
          if (master_bank == 128) {
            // Drum bank: 0V CV = note 36 (Kick), no scale quantization
            note = 36 + semitone_offset;
          } else {
            int32_t chromatic_note = 60 + semitone_offset;
            note = quantize_to_scale(chromatic_note, active_scale);
          }
          if (note < 0)
            note = 0;
          if (note > 127)
            note = 127;

          int32_t velocity = 100;
          if (Connected(Input::Audio1)) {
            int32_t raw_vel = AudioIn1();
            if (raw_vel < 0)
              raw_vel = -raw_vel;
            velocity = (raw_vel * 127) / 2048;
            if (velocity < 10)
              velocity = 10;
            if (velocity > 127)
              velocity = 127;
          }

          uint32_t next_head = (request_head + 1) % REQUEST_QUEUE_SIZE;
          if (next_head != request_tail) {
            request_queue[request_head] = {
                REQ_NOTE_ON, (uint8_t)note, (uint8_t)velocity,
                master_bank, master_preset, 0,
                true};
            request_head = next_head;
          }
          gate1_active_note = note;
        } else if (!p1_now && last_pulse1) {
          // Gate 1 Falling Edge
          if (gate1_active_note != -1) {
            uint32_t next_head = (request_head + 1) % REQUEST_QUEUE_SIZE;
            if (next_head != request_tail) {
              request_queue[request_head] = {
                  REQ_NOTE_OFF, (uint8_t)gate1_active_note, 0, 0, 0, 0, false};
              request_head = next_head;
            }
            gate1_active_note = -1;
          }
        }
      } else {
        if (gate1_active_note != -1) {
          uint32_t next_head = (request_head + 1) % REQUEST_QUEUE_SIZE;
          if (next_head != request_tail) {
            request_queue[request_head] = {
                REQ_NOTE_OFF, (uint8_t)gate1_active_note, 0, 0, 0, 0, false};
            request_head = next_head;
          }
          gate1_active_note = -1;
        }
      }
      last_pulse1 = p1_now;



      // If Pulse 2 is NOT connected (and Pulse 1 is connected), handle
      // overrides:
      if (Disconnected(Input::Pulse2)) {
        if (gate2_active_note != -1) {
          trigger_note_off_channel(gate2_active_note, 0);
          gate2_active_note = -1;
        }
        last_pulse2 = false;
      } else if (Connected(Input::Pulse1)) {
        // Pulse 2 is connected AND Pulse 1 is connected -> Duophonic / Dual
        // preset mode (CV 2 / Audio 2)
        int32_t cv2_raw = Connected(Input::CV2) ? CVIn2() : 0;
        cv2_acc = cv2_acc - (cv2_acc >> 8) + cv2_raw;
        int32_t cv2_smoothed = cv2_acc >> 8;

        if (!freeze_active) {
          if (p2_now && !last_pulse2) {
            // Gate 2 Rising Edge
            int32_t raw_pitch =
                Connected(Input::CV2) ? ((cv2_smoothed * 15) >> 1) : 0;
            int32_t semitone_offset = quantize_semitone(raw_pitch);
            int32_t note;
            uint16_t active_g2_bank =
                (current_y_mode == 1) ? master_bank2 : master_bank;
            if (active_g2_bank == 128) {
              // Drum bank: 0V CV = note 36 (Kick), no scale quantization
              note = 36 + semitone_offset;
            } else {
              int32_t chromatic_note = 60 + semitone_offset;
              note = quantize_to_scale(chromatic_note, active_scale);
            }
            if (note < 0)
              note = 0;
            if (note > 127)
              note = 127;

            int32_t velocity = 100;
            if (Connected(Input::Audio2)) {
              int32_t raw_vel = AudioIn2();
              if (raw_vel < 0)
                raw_vel = -raw_vel;
              velocity = (raw_vel * 127) / 2048;
              if (velocity < 10)
                velocity = 10;
              if (velocity > 127)
                velocity = 127;
            }

            // Push note-on request to request_queue
            uint32_t next_head = (request_head + 1) % REQUEST_QUEUE_SIZE;
            if (next_head != request_tail) {
              if (current_y_mode == 1) {
                request_queue[request_head] = {
                    REQ_NOTE_ON,  (uint8_t)note,  (uint8_t)velocity,
                    master_bank2, master_preset2, 1,
                    true};
              } else {
                request_queue[request_head] = {
                    REQ_NOTE_ON, (uint8_t)note, (uint8_t)velocity,
                    master_bank, master_preset, 0,
                    true};
              }
              request_head = next_head;
            }
            gate2_active_note = note;
          } else if (!p2_now && last_pulse2) {
            // Gate 2 Falling Edge
            if (gate2_active_note != -1) {
              uint32_t next_head = (request_head + 1) % REQUEST_QUEUE_SIZE;
              if (next_head != request_tail) {
                if (current_y_mode == 1) {
                  request_queue[request_head] = {
                      REQ_NOTE_OFF, (uint8_t)gate2_active_note, 0, 0, 0, 1,
                      false};
                } else {
                  request_queue[request_head] = {
                      REQ_NOTE_OFF, (uint8_t)gate2_active_note, 0, 0, 0, 0,
                      false};
                }
                request_head = next_head;
              }
              gate2_active_note = -1;
            }
          }
        } else {
          if (gate2_active_note != -1) {
            uint32_t next_head = (request_head + 1) % REQUEST_QUEUE_SIZE;
            if (next_head != request_tail) {
              if (current_y_mode == 1) {
                request_queue[request_head] = {
                    REQ_NOTE_OFF, (uint8_t)gate2_active_note, 0, 0, 0, 1,
                    false};
              } else {
                request_queue[request_head] = {
                    REQ_NOTE_OFF, (uint8_t)gate2_active_note, 0, 0, 0, 0,
                    false};
              }
              request_head = next_head;
            }
            gate2_active_note = -1;
          }
        }
        last_pulse2 = p2_now;
      }
    }

    if (clear_voices_request) {
      audio_event_tail = audio_event_head;
      for (int i = 0; i < MAX_VOICES; i++) {
        voices[i].key_down = false;
        voices[i].state = VOICE_OFF;
        voices[i].env_val = 0;
      }
      clear_voices_request = false;
    }

    // ── Drain Audio Event Queue (Core 1) ──
    while (audio_event_tail != audio_event_head) {
      AudioEvent evt = audio_event_queue[audio_event_tail];
      audio_event_tail = (audio_event_tail + 1) % AUDIO_EVENT_QUEUE_SIZE;

      if (evt.type == EVT_NOTE_ON) {
        start_resolved_voice_from_event(evt.voice_data, evt.note, evt.velocity,
                                         evt.channel);
        // Channel 9 trigger is no longer used for sidechain ducking
      } else if (evt.type == EVT_NOTE_OFF) {
        trigger_note_off_channel(evt.note, evt.channel);
      } else if (evt.type == EVT_ALL_NOTES_OFF) {
        for (int i = 0; i < MAX_VOICES; i++) {
          if (evt.channel == 0xFF || voices[i].channel == evt.channel) {
            voices[i].key_down = false;
            voices[i].state = VOICE_OFF;
            voices[i].env_val = 0;
          }
        }
      }
    }

    // --- Sidechain Compressor Gain Calculation (only in MIDI playback mode and Audio1 connected) ---
    int32_t duck_gain = 32767; // Q15
    if (midi_playback_mode && Connected(Input::Audio1)) {
      static int32_t dc_in1_x_prev = 0;
      static int32_t dc_in1_y_prev = 0;
      int32_t raw_sc = AudioIn1();
      // DC blocker: y[n] = x[n] - x[n-1] + 0.995 * y[n-1]
      int32_t sc_val = raw_sc - dc_in1_x_prev + ((32604 * dc_in1_y_prev) >> 15);
      dc_in1_x_prev = raw_sc;
      dc_in1_y_prev = sc_val;

      if (sc_val < 0) sc_val = -sc_val;
      static int32_t sc_env_follower = 0;
      if (sc_val > sc_env_follower) {
        sc_env_follower += (sc_val - sc_env_follower) >> 3; // fast attack
      } else {
        sc_env_follower += (sc_val - sc_env_follower) >> 9; // medium release
      }
      int32_t sc_env = sc_env_follower << 4; // scale 0..2048 to 0..32768
      if (sc_env > 32767) sc_env = 32767;

      if (param_ducking > 0) {
        duck_gain = 32767 - ((sc_env * param_ducking) >> 15);
        if (duck_gain < 0) duck_gain = 0;
      }
    }

    // Compute tape-stop speed factor when frozen
    // Main knob left half: tape stop (0=silence, center=normal speed)
    // Main knob right half: stutter division
    // Y knob: pitch detune of frozen loop (center=no detune, left=down, right=up)
    int32_t speed_factor_q15 = 32768; // 1.0 in Q15
    static int32_t current_speed = 32768;
    static int32_t freeze_detune_factor_q15 = 32768; // 1.0 = no detune
    if (freeze_active) {
      int32_t raw_main = param_stutter;
      if (raw_main < 2048) {
        // Tape-stop: smooth speed ramp from 0 to full based on main knob
        int32_t target_speed = (raw_main * 32768) / 2048;
        // Smooth with a gentle coeff so it feels like tape
        current_speed = current_speed + ((target_speed - current_speed) >> 5);
        speed_factor_q15 = current_speed;
      } else {
        current_speed = 32768;
        speed_factor_q15 = 32768;
      }
      // Y knob: detune the frozen loop pitch (disabled: Y knob is dedicated to unipolar sequencer glitches)
      int32_t detune_q15 = 32768;
      // Smooth the detune to avoid zipper noise
      freeze_detune_factor_q15 += ((detune_q15 - freeze_detune_factor_q15) >> 7);
    } else {
      current_speed = 32768;
      speed_factor_q15 = 32768;
      freeze_detune_factor_q15 = 32768;
    }
    glitch_speed_factor_q15 = speed_factor_q15;

    // ── Voice Mix loop ──
    int32_t synth_L = 0;
    int32_t synth_R = 0;
    int32_t drums_L = 0;
    int32_t drums_R = 0;



    // Detect CV2 glitch trigger/gate edge (run once per sample frame)
    static bool last_cv2_gate = false;
    bool cv2_gate = false;
    if (Connected(Input::CV2) && (midi_playback_mode || Disconnected(Input::Pulse2))) {
      cv2_gate = (CVIn2() > 800);
    }
    bool cv2_trigger = (cv2_gate && !last_cv2_gate);
    last_cv2_gate = cv2_gate;

    for (int i = 0; i < MAX_VOICES; i++) {
      SynthVoice &v = voices[i];
      VoiceState voice_state = v.state;
      if (voice_state == VOICE_OFF)
        continue;

      // Localize frequently used member variables in registers/local memory
      const int16_t *s_data = v.sample_data;
      uint64_t phase = v.phase;
      bool is_looped = v.is_looped;
      uint32_t loop_start = v.loop_start;
      uint32_t loop_end = v.loop_end;
      uint32_t sample_end = v.sample_end;

      // Calculate modulated phase increment based on active mangling mode
      // Calculate effective glitch mangle amount
      int32_t effective_glitch_mangle = param_glitch_mangle;
      if (cv2_gate && effective_glitch_mangle < 4096) {
        effective_glitch_mangle = 4096;
      }

      // If CV2 trigger edge is detected, instantly reset the glitch timer to force a new glitch block
      if (cv2_trigger) {
        v.glitch_timer = 0;
        v.glitch_duration = 0;
        v.glitch_state = 0;
      }

      // Binary probability circuit-bent glitch system:
      if (effective_glitch_mangle >= 4096) {
        v.glitch_timer++;
        if (v.glitch_timer >= v.glitch_duration) {
          v.glitch_timer = 0;
          v.glitch_mangle_held = effective_glitch_mangle; // Sample & Hold CV here

          // Calculate samples per beat based on active tempo
          int32_t samples_per_beat = 12000; // default 120 BPM @ 24kHz
          if (midi_tempo_us > 0) {
            samples_per_beat = (midi_tempo_us * 24) / 1000;
          }
          int32_t step_32nd = samples_per_beat / 8;
          if (step_32nd < 50) step_32nd = 50;

          // Islands of stability: probability of entering clean state decreases as glitch percentage increases
          // At bottom of glitch range: high clean probability (up to 70%). At top: 0%.
          int32_t island_prob = 70 - ((v.glitch_mangle_held - 4096) * 70) / (32767 - 4096);
          if (cv2_gate) island_prob = 0; // force glitching, no clean islands
          
          bool go_to_island = false;
          if (v.glitch_state != 0 && (rand() % 100) < island_prob) {
            go_to_island = true;
          }

          if (go_to_island) {
            v.glitch_state = 0; // guaranteed clean island
            v.glitch_duration = (1 + (rand() % 3)) * samples_per_beat; // 1 to 3 full beats
          } else {
            // Get glitch probability (0% to 100%)
            int32_t prob = (v.glitch_mangle_held - 4096) * 100 / (32767 - 4096);
            if (cv2_gate) prob = 100; // force glitching
            if ((rand() % 100) < prob) {
              // Decide glitch type and duration based on knob zones
              int32_t r_type = rand() % 100;
              if (v.glitch_mangle_held < 13654) {
                // First Third: Microglitches/stutters. Mostly Micro-loops (80%), Reverse (10%), XOR (10%)
                if (r_type < 80) {
                  v.glitch_state = 2; // MICRO_LOOP
                } else if (r_type < 90) {
                  v.glitch_state = 1; // REVERSE
                } else {
                  v.glitch_state = 3; // XOR
                }
              } else if (v.glitch_mangle_held < 23211) {
                // Middle: Glitch textures. Micro-loops (50%), XOR (30%), Reverse (20%)
                if (r_type < 50) {
                  v.glitch_state = 2; // MICRO_LOOP
                } else if (r_type < 80) {
                  v.glitch_state = 3; // XOR
                } else {
                  v.glitch_state = 1; // REVERSE
                }
              } else {
                // Right: Last third. If we are in the top 20% (>= 26214), allow wrong sample
                if (v.glitch_mangle_held >= 26214) {
                  // Top 20%: XOR (40%), Wrong Sample (30%), Micro-loops (20%), Reverse (10%)
                  if (r_type < 40) {
                    v.glitch_state = 3; // XOR
                  } else if (r_type < 70) {
                    v.glitch_state = 4; // WRONG_SAMPLE
                  } else if (r_type < 90) {
                    v.glitch_state = 2; // MICRO_LOOP
                  } else {
                    v.glitch_state = 1; // REVERSE
                  }
                } else {
                  // Last third but below top 20%: XOR (50%), Micro-loops (30%), Reverse (20%)
                  if (r_type < 50) {
                    v.glitch_state = 3; // XOR
                  } else if (r_type < 80) {
                    v.glitch_state = 2; // MICRO_LOOP
                  } else {
                    v.glitch_state = 1; // REVERSE
                  }
                }
              }

              // Determine duration: 20% chance of a longer, musical glitch (1, 2, or 4 beats / 1 bar)
              if (rand() % 5 == 0) {
                v.glitch_duration = (1 << (rand() % 3)) * samples_per_beat; // 1, 2, or 4 beats
              } else {
                // Quick erratic durations (zone-based)
                if (v.glitch_mangle_held < 13654) {
                  v.glitch_duration = ((rand() % 2 == 0) ? 2 : 4) * step_32nd; // 16th or 8th note stutters
                } else if (v.glitch_mangle_held < 23211) {
                  v.glitch_duration = (1 + (rand() % 3)) * step_32nd; // 32nd, 16th or dotted 16th
                } else {
                  v.glitch_duration = 1 + (rand() % 4) * (step_32nd / 2); // extremely fast variable rates
                }
              }
            } else {
              v.glitch_state = 0; // clean chunk
              v.glitch_duration = ((rand() % 2 == 0) ? 2 : 4) * step_32nd; // 16th or 8th note duration
            }
          }
        }
      } else {
        v.glitch_state = 0; // GLITCH_NONE
        v.glitch_timer = 0;
        v.glitch_duration = 0;
        v.glitch_wrong_idx = -1;
      }

      bool play_reverse = (v.glitch_state == 1);
      bool play_micro_loop = (v.glitch_state == 2);
      bool do_scrambling = (v.glitch_state == 3);
      bool do_wrong_sample = (v.glitch_state == 4);

      // Wrong-sample rompler glitch: reads from a stable alternate sample in the soundfont index.
      if (do_wrong_sample && indexed_samples_count > 1) {
        if (v.glitch_wrong_idx < 0) {
          v.glitch_wrong_idx = (int32_t)(((uint32_t)v.note * 73 + (uint32_t)v.velocity * 11) % indexed_samples_count);
          if (indexed_samples_count > 1 && samples_index[v.glitch_wrong_idx].sample_data == v.sample_data) {
            v.glitch_wrong_idx = (v.glitch_wrong_idx + 1) % indexed_samples_count;
          }
        }
        const SamplesIndex &ws = samples_index[(uint32_t)v.glitch_wrong_idx];
        if (ws.sample_data != nullptr && ws.end > ws.start) {
          s_data = ws.sample_data;
          sample_end = ws.end - ws.start;
          bool valid_loop = (ws.loop_start >= ws.start && ws.loop_end <= ws.end && ws.loop_end > ws.loop_start + 2);
          if (valid_loop) {
            loop_start = ws.loop_start - ws.start;
            loop_end = ws.loop_end - ws.start;
            is_looped = true;
          } else {
            loop_start = 0;
            loop_end = sample_end;
            is_looped = false;
          }
        }
      } else if (!do_wrong_sample) {
        v.glitch_wrong_idx = -1;
      }

      // Loop boundary overrides for granular micro-loop
      uint32_t loop_start_val = loop_start;
      uint32_t loop_end_val = loop_end;
      bool is_looped_val = is_looped;

      if (play_micro_loop) {
        // Grain size: large (8192 samples = ~170ms) down to (2048 = ~43ms).
        // Stay well above the ~2000 sample threshold where loop-rate is subsonic.
        uint32_t zone_val = (v.glitch_mangle_held >= 12288) ? (v.glitch_mangle_held - 12288) : 0;
        uint32_t grain_size = 8192 - ((zone_val * 6144) / (32767 - 12288)); // 8192..2048
        if (grain_size < 2048) grain_size = 2048;
        if (grain_size > 8192) grain_size = 8192;
        // Clamp to sample length
        if (sample_end > 0 && grain_size > sample_end) grain_size = sample_end;
        if (v.micro_loop_size != grain_size) {
          v.micro_loop_start = phase >> 32;
          if (sample_end > grain_size) {
            if (v.micro_loop_start > sample_end - grain_size) {
              v.micro_loop_start = sample_end - grain_size;
            }
          } else {
            v.micro_loop_start = 0;
            grain_size = sample_end;
          }
          v.micro_loop_size = grain_size;
        }
        loop_start_val = v.micro_loop_start;
        loop_end_val = v.micro_loop_start + grain_size;
        is_looped_val = true;
      }

      // 1. Raw linear-interpolated sample playback (optimized)
      uint32_t idx = phase >> 32;
      uint32_t frac = (phase >> 16) & 0xFFFF; // Q16

      if (sample_end > 0) {
        idx = idx % sample_end;
      } else {
        idx = 0;
      }

      // XOR address scrambling
      if (do_scrambling) {
        uint32_t xor_mask = (v.glitch_mangle_held - 20480) * 17;
        xor_mask ^= (v.note * 37);
        idx = idx ^ xor_mask;
        if (sample_end > 0) {
          idx = idx % sample_end;
        } else {
          idx = 0;
        }
      }

      int16_t s0 = s_data[idx];
      int16_t s1 = 0;

      uint32_t idx_next = idx + 1;
      if (is_looped_val) {
        if (idx_next >= loop_end_val) {
          idx_next = loop_start_val;
        }
      } else {
        if (idx_next >= sample_end) {
          idx_next = sample_end - 1;
        }
      }
      if (sample_end > 0) {
        idx_next = idx_next % sample_end;
      } else {
        idx_next = 0;
      }
      s1 = s_data[idx_next];

      int32_t interpolated = s0 + (((s1 - s0) * (int32_t)frac) >> 16);

      // 2. Volume AHR envelope update
      int32_t env_val = v.env_val;
      if (update_envelopes) {
        if (voice_state == VOICE_ATTACK) {
          if (env_val < 2147483647 - global_env_attack_step) {
            env_val += global_env_attack_step;
          } else {
            env_val = 2147483647;
            voice_state = VOICE_SUSTAIN; // Hold state at max level
          }
        } else if (voice_state == VOICE_SUSTAIN) {
          env_val = 2147483647;
        } else if (voice_state == VOICE_RELEASE) {
          int32_t rel_step = global_env_release_step;
          if (v.channel == 9 || global_config.channels[v.channel].target == 1 || global_config.channels[v.channel].target == 6 || v.channel == 1) {
            rel_step = global_drum_env_release_step;
          }
          if (env_val > rel_step) {
            env_val -= rel_step;
          } else {
            env_val = 0;
            v.env_val = 0;
            v.state = VOICE_OFF;
            continue;
          }
        }
        v.env_val = env_val;
        v.state = voice_state;
      }

      // 2b. SF2 Modulation envelope update (drives pitch sweep)
      uint64_t mod_phase_increment = v.phase_increment;
      if (v.mod_env_to_pitch != 0) {
        bool mod_changed = false;
        int32_t mod_env_val = v.mod_env_val;
        VoiceState mod_env_state = v.mod_env_state;

        if (update_envelopes) {
          if (mod_env_state == VOICE_ATTACK) {
            if (mod_env_val < 2147483647 - v.mod_env_attack_step) {
              mod_env_val += v.mod_env_attack_step;
            } else {
              mod_env_val = 2147483647;
              mod_env_state = VOICE_DECAY;
            }
            mod_changed = true;
          } else if (mod_env_state == VOICE_DECAY) {
            if (mod_env_val > v.mod_env_sustain + v.mod_env_decay_step) {
              mod_env_val -= v.mod_env_decay_step;
              mod_changed = true;
            } else {
              mod_env_val = v.mod_env_sustain;
              mod_env_state = VOICE_SUSTAIN;
              mod_changed = true; // One final update on transition
            }
          } else if (mod_env_state == VOICE_SUSTAIN) {
            // Value is constant — no recompute needed
            if (!v.key_down) {
              mod_env_state = VOICE_RELEASE;
              mod_changed = true;
            }
          } else if (mod_env_state == VOICE_RELEASE) {
            if (mod_env_val > v.mod_env_release_step) {
              mod_env_val -= v.mod_env_release_step;
            } else {
              mod_env_val = 0;
            }
            mod_changed = true;
          }
          v.mod_env_val = mod_env_val;
          v.mod_env_state = mod_env_state;
        }

        // Only do the expensive 64-bit multiply when the envelope value changed
        if (mod_changed || v.mod_increment_dirty) {
          int32_t pitch_cents = (int32_t)(((int64_t)mod_env_val >> 16) * v.mod_env_to_pitch >> 15);
          int64_t mod_factor_q16 = 65536LL + ((int64_t)pitch_cents * 38LL);
          if (mod_factor_q16 < 32768)
            mod_factor_q16 = 32768;
          if (mod_factor_q16 > 131072)
            mod_factor_q16 = 131072;
          v.cached_mod_increment = (v.phase_increment * (uint64_t)mod_factor_q16) >> 16;
          v.mod_increment_dirty = false;
        }
        mod_phase_increment = v.cached_mod_increment;
      }

      // Apply MIDI pitch bend
      uint32_t bend_mult = channel_bend_mult_q16[v.channel];
      if (bend_mult != 65536) {
        mod_phase_increment = (mod_phase_increment * (uint64_t)bend_mult) >> 16;
      }

      // Apply tape-stop speed scaling and freeze pitch detune
      if (freeze_active) {
        // Tape-stop speed scalar
        mod_phase_increment = ((uint64_t)mod_phase_increment * (uint32_t)glitch_speed_factor_q15) >> 15;
        // Pitch detune for harmonic freeze scrubbing
        mod_phase_increment = ((uint64_t)mod_phase_increment * (uint32_t)freeze_detune_factor_q15) >> 15;
      }

      // Update playhead phase
      // For reverse: use signed arithmetic to detect wrap correctly
      if (play_reverse) {
        // Check if subtracting would underflow before committing
        if (phase >= mod_phase_increment) {
          phase -= mod_phase_increment;
        } else {
          // Underflow: wrap to loop_end or sample_end
          if (is_looped_val && loop_end_val > loop_start_val) {
            // Jump to just before loop_end
            phase = ((uint64_t)loop_end_val << 32) - 1;
          } else {
            // Non-looped reverse: stop at beginning
            v.state = VOICE_OFF;
            continue;
          }
        }
      } else {
        phase += mod_phase_increment;
      }
      v.phase = phase;

      // Check sample boundary limits (forward only; reverse handled above)
      uint32_t next_idx = phase >> 32;
      if (is_looped_val) {
        if (!play_reverse) {
          if (next_idx >= loop_end_val) {
            uint64_t loop_len_q32 = (uint64_t)(loop_end_val - loop_start_val) << 32;
            phase -= loop_len_q32;
            v.phase = phase;
          }
        } else {
          // Reverse: wrap forward when we go below loop_start
          if (next_idx < loop_start_val || next_idx >= loop_end_val) {
            // Jump back to just before loop_end
            phase = ((uint64_t)(loop_end_val - 1) << 32) | (phase & 0xFFFFFFFF);
            v.phase = phase;
          }
        }
      } else {
        if (!play_reverse) {
          if (next_idx >= sample_end) {
            v.state = VOICE_OFF;
          }
        }
        // reverse non-looped: already handled underflow above with VOICE_OFF
      }

      int32_t amp_sample = ((interpolated * (env_val >> 16)) >> 15);
      amp_sample = (amp_sample * v.velocity * 23) >> 17;
      amp_sample = (amp_sample * v.vol_atten_q15) >> 15;

      // Apply per-channel volume trim
      int32_t trim_db = global_config.channels[v.channel].volume_trim;
      if (trim_db < -12) trim_db = -12;
      if (trim_db > 6) trim_db = 6;
      uint32_t gain_q15 = db_to_gain_q15[trim_db + 12];
      amp_sample = (amp_sample * (int32_t)gain_q15) >> 15;

      int32_t ch_target = global_config.channels[v.channel].target;
      if (!midi_playback_mode) {
        if (v.channel == 0) {
          ch_target = 0;
        } else if (v.channel == 1) {
          ch_target = 1;
        } else {
          if (ch_target == 2 || ch_target == 5) ch_target = 0;
          if (ch_target == 3 || ch_target == 6) ch_target = 1;
        }
      } else {
        if (ch_target == 2 || ch_target == 5) ch_target = 0;
        if (ch_target == 3 || ch_target == 6) ch_target = 1;
      }

      if (global_config.stereo_mode == 0) { // Stereo
        if (ch_target == 1 || v.channel == 9) { // Drums/Synth 2
          drums_L += amp_sample;
          drums_R += amp_sample;
        } else {
          synth_L += amp_sample;
          synth_R += amp_sample;
        }
      } else { // Dual Mono
        if (ch_target == 1 || v.channel == 9) { // Drums/Synth 2
          drums_R += amp_sample;
        } else { // Synth 1
          synth_L += amp_sample;
        }
      }
    }

    // Apply sidechain ducking to Synth 1 tracks ONLY (only in MIDI playback mode)
    if (midi_playback_mode) {
      synth_L = (synth_L * duck_gain) >> 15;
      synth_R = (synth_R * duck_gain) >> 15;
    }

    // Mix, filter, and glitch external Audio 2 input if connected (only in MIDI playback mode)
    if (midi_playback_mode && Connected(Input::Audio2)) {
      static int32_t dc_in2_x_prev = 0;
      static int32_t dc_in2_y_prev = 0;
      int32_t raw_audio = AudioIn2();
      // DC blocker: y[n] = x[n] - x[n-1] + 0.995 * y[n-1]
      int32_t ext_audio = raw_audio - dc_in2_x_prev + ((32604 * dc_in2_y_prev) >> 15);
      dc_in2_x_prev = raw_audio;
      dc_in2_y_prev = ext_audio;

      // Real-time glitcher for external audio input
      static int16_t ext_glitch_buffer[8192] = {0};
      static uint32_t ext_write_idx = 0;

      // Save clean sample to circular buffer
      ext_glitch_buffer[ext_write_idx] = (int16_t)ext_audio;
      uint32_t current_write_idx = ext_write_idx;
      ext_write_idx = (ext_write_idx + 1) & 8191;

      // Follow the global glitch mangle parameter
      int32_t effective_glitch_mangle = param_glitch_mangle;
      if (cv2_gate && effective_glitch_mangle < 4096) {
        effective_glitch_mangle = 4096;
      }

      static int32_t ext_glitch_state = 0; // 0=NONE, 1=REVERSE, 2=MICRO_LOOP, 3=XOR
      static int32_t ext_glitch_timer = 0;
      static int32_t ext_glitch_duration = 0;
      static int32_t ext_glitch_mangle_held = 0;
      static uint32_t ext_micro_loop_start = 0;
      static uint32_t ext_micro_loop_size = 4096;

      // Reset glitch timer if CV2 trigger occurs
      if (cv2_trigger) {
        ext_glitch_timer = 0;
        ext_glitch_duration = 0;
        ext_glitch_state = 0;
      }

      if (effective_glitch_mangle >= 4096) {
        ext_glitch_timer++;
        if (ext_glitch_timer >= ext_glitch_duration) {
          ext_glitch_timer = 0;
          ext_glitch_mangle_held = effective_glitch_mangle;

          int32_t samples_per_beat = 24000; // 120 BPM @ 48kHz
          if (midi_tempo_us > 0) {
            samples_per_beat = (midi_tempo_us * 48) / 1000;
          }
          int32_t step_32nd = samples_per_beat / 8;
          if (step_32nd < 100) step_32nd = 100;

          // Clean island probability
          int32_t island_prob = 70 - ((ext_glitch_mangle_held - 4096) * 70) / (32767 - 4096);
          if (cv2_gate) island_prob = 0;

          bool go_to_island = false;
          if (ext_glitch_state != 0 && (rand() % 100) < island_prob) {
            go_to_island = true;
          }

          if (go_to_island) {
            ext_glitch_state = 0;
            ext_glitch_duration = (1 + (rand() % 3)) * samples_per_beat;
          } else {
            int32_t prob = (ext_glitch_mangle_held - 4096) * 100 / (32767 - 4096);
            if (cv2_gate) prob = 100;

            if ((rand() % 100) < prob) {
              int32_t r_type = rand() % 100;
              if (ext_glitch_mangle_held < 13654) {
                // First Third: micro-loops, reverse, xor
                if (r_type < 80) ext_glitch_state = 2; // MICRO_LOOP
                else if (r_type < 90) ext_glitch_state = 1; // REVERSE
                else ext_glitch_state = 3; // XOR
              } else if (ext_glitch_mangle_held < 23211) {
                // Middle
                if (r_type < 50) ext_glitch_state = 2;
                else if (r_type < 80) ext_glitch_state = 3;
                else ext_glitch_state = 1;
              } else {
                // Last Third
                if (r_type < 50) ext_glitch_state = 3;
                else if (r_type < 80) ext_glitch_state = 2;
                else ext_glitch_state = 1;
              }

              // Duration
              if (rand() % 5 == 0) {
                ext_glitch_duration = (1 << (rand() % 3)) * samples_per_beat; // 1, 2, or 4 beats
              } else {
                if (ext_glitch_mangle_held < 13654) {
                  ext_glitch_duration = ((rand() % 2 == 0) ? 2 : 4) * step_32nd;
                } else if (ext_glitch_mangle_held < 23211) {
                  ext_glitch_duration = (1 + (rand() % 3)) * step_32nd;
                } else {
                  ext_glitch_duration = 1 + (rand() % 4) * (step_32nd / 2);
                }
              }

              // Setup loop parameters
              if (ext_glitch_state == 2) {
                uint32_t zone_val = (ext_glitch_mangle_held >= 12288) ? (ext_glitch_mangle_held - 12288) : 0;
                ext_micro_loop_size = 8192 - ((zone_val * 6144) / (32767 - 12288));
                if (ext_micro_loop_size < 2048) ext_micro_loop_size = 2048;
                if (ext_micro_loop_size > 8192) ext_micro_loop_size = 8192;
                ext_micro_loop_start = (current_write_idx + 8192 - ext_micro_loop_size) & 8191;
              }
            } else {
              ext_glitch_state = 0;
              ext_glitch_duration = ((rand() % 2 == 0) ? 2 : 4) * step_32nd;
            }
          }
        }
      } else {
        ext_glitch_state = 0;
        ext_glitch_timer = 0;
        ext_glitch_duration = 0;
      }

      // Read from buffer based on active glitch type
      if (ext_glitch_state == 2) {
        uint32_t play_offset = ext_glitch_timer % ext_micro_loop_size;
        ext_audio = ext_glitch_buffer[(ext_micro_loop_start + play_offset) & 8191];
      } else if (ext_glitch_state == 1) {
        uint32_t read_idx = (current_write_idx + 16384 - (2 * ext_glitch_timer)) & 8191;
        ext_audio = ext_glitch_buffer[read_idx];
      } else if (ext_glitch_state == 3) {
        int32_t samples_per_beat = 24000;
        if (midi_tempo_us > 0) {
          samples_per_beat = (midi_tempo_us * 48) / 1000;
        }
        int32_t step_32nd = samples_per_beat / 8;
        if (step_32nd < 100) step_32nd = 100;

        uint32_t tap_idx = (ext_glitch_timer / step_32nd) ^ (ext_glitch_mangle_held & 3);
        uint32_t delay_taps[4] = { 1024, 2048, 4096, 7168 };
        uint32_t delay = delay_taps[tap_idx & 3];

        uint32_t read_idx = (current_write_idx + 8192 - delay) & 8191;
        ext_audio = ext_glitch_buffer[read_idx];
      }

      synth_L += ext_audio;
      if (global_config.stereo_mode == 0) {
        synth_R += ext_audio;
      }
    }

    int32_t mixed_L = synth_L + drums_L;
    int32_t mixed_R = synth_R + drums_R;

    // No rhythmic gate — Y knob is now pure decimation/bitcrush
    int32_t gate_on = 1;

    int32_t target_gate_vol = gate_on ? 32767 : 0;
    gate_vol = gate_vol + ((target_gate_vol - gate_vol) >> 6);

    mixed_L = (mixed_L * gate_vol) >> 15;
    mixed_R = (mixed_R * gate_vol) >> 15;

    // 3. Queue for Core 0 post-processing
    AudioFrame frame_to_send = { mixed_L, mixed_R };
    queue_try_add(&core1_to_core0_queue, &frame_to_send);

    AudioFrame frame_received;
    if (queue_try_remove(&core0_to_core1_queue, &frame_received)) {
      AudioOut1(clamp_audio(frame_received.L));
      AudioOut2(clamp_audio(frame_received.R));
    } else {
      // Underrun fallback: output unprocessed audio to avoid interruption
      AudioOut1(clamp_audio(mixed_L));
      AudioOut2(clamp_audio(mixed_R));
    }

    // ── CV / Gate Outputs (Unified Live & Playback) ──
    int32_t cv1_transpose_offset = 0;
    if (Connected(Input::CV1)) {
      int32_t cv1_raw = CVIn1();
      cv1_transpose_offset = quantize_semitone((cv1_raw * 15) >> 1);
    }

    // Decrement trigger timers
    if (cv1_trigger_timer > 0) cv1_trigger_timer--;
    if (cv2_trigger_timer > 0) cv2_trigger_timer--;

    // Determine if CV2 is used as a pitch target
    bool cv2_is_pitch = false;
    for (int i = 0; i < 16; i++) {
      if (global_config.channels[i].target == 3 || global_config.channels[i].target == 6) {
        cv2_is_pitch = true;
        break;
      }
    }

    // Compute freeze tape speed pitch drop in whole semitones (integer, no floats).
    // Uses bit-counting integer log2: count octaves down then linearly interpolate
    // the fractional semitone within the octave. Each halving = -12 semitones.
    int32_t tape_drop = 0;
    if (freeze_active && glitch_speed_factor_q15 < 32768) {
      if (glitch_speed_factor_q15 <= 0) {
        tape_drop = -120;
      } else {
        int32_t f = glitch_speed_factor_q15;
        int32_t octaves = 0;
        while (f < 32768) { f <<= 1; octaves++; }
        // f is now in [32768, 65535]: 1.0..2.0 in Q15 fixed point
        // Linear approx of fractional semitones within this octave
        int32_t frac_semi = (12 * (f - 32768)) / 32768;
        tape_drop = -(octaves * 12 - frac_semi);
      }
    }

    // CV1 and Gate 1 (Pulse 1)
    if (cv1_out_note >= 0) {
      int32_t out_note = cv1_out_note;
      if (midi_playback_mode) {
        if (cv1_out_channel != 9) {
          out_note += midi_transpose + cv1_transpose_offset;
        }
      } else {
        out_note += page_coarse;
      }
      if (freeze_active) {
        out_note += (int32_t)tape_drop;
      }
      if (out_note < 0) out_note = 0;
      if (out_note > 127) out_note = 127;
      CVOut1MIDINote((uint8_t)out_note);
    }

    bool cv1_pin_state = false;
    if (global_config.cv1_mode == 1 || cv1_out_channel == 9) { // Trigger mode or Drums channel
      if (cv1_trigger_timer > 0) {
        cv1_pin_state = true;
      }
    } else { // Gate mode
      cv1_pin_state = cv1_out_gate;
    }
    PulseOut1(cv1_pin_state);

    // CV2 and Gate 2 (Pulse 2)
    if (cv2_is_pitch) {
      if (cv2_out_note >= 0) {
        int32_t out_note = cv2_out_note;
        if (midi_playback_mode) {
          if (cv2_out_channel != 9) {
            out_note += midi_transpose + cv1_transpose_offset;
          }
        } else {
          out_note += page_coarse;
        }
        if (freeze_active) {
          out_note += (int32_t)tape_drop;
        }
        if (out_note < 0) out_note = 0;
        if (out_note > 127) out_note = 127;
        CVOut2MIDINote((uint8_t)out_note);
      }
    } else {
      // Envelope Follower output on CVOut2 when CV2 note is inactive
      int32_t max_env = 0;
      for (int i = 0; i < MAX_VOICES; i++) {
        if (voices[i].state != VOICE_OFF && voices[i].channel == 0) {
          if (voices[i].env_val > max_env) {
            max_env = voices[i].env_val;
          }
        }
      }
      int32_t env_mv = ((uint64_t)max_env * 5000) >> 31;
      CVOut2Millivolts(env_mv);
    }

    if (cv2_is_pitch) {
      bool cv2_pin_state = false;
      if (global_config.cv2_mode == 1 || cv2_out_channel == 9) { // Trigger mode or Drums channel
        if (cv2_trigger_timer > 0) {
          cv2_pin_state = true;
        }
      } else { // Gate mode
        cv2_pin_state = cv2_out_gate;
      }
      PulseOut2(cv2_pin_state);
    } else {
      // Beat clock pulse trigger fallback on PulseOut2 (when not playing CV2)
      if (midi_playback_mode && !Connected(Input::Pulse2)) {
        if (pulse2_out_timer > 0) {
          pulse2_out_timer--;
          PulseOut2(true);
        } else {
          PulseOut2(false);
        }
      } else {
        PulseOut2(false);
      }
    }

  }
};

RomplerApp card;

void trigger_sustaining_notes(uint32_t play_tick, bool include_current_tick) {
  uint32_t len = 16 * midi_division;
  struct SustainingTracker {
    uint16_t min_dist;
    uint8_t type; // 0 = none, 1 = ON, 2 = OFF
    uint8_t velocity;
    uint16_t bank;
    uint8_t preset;
  };
  static SustainingTracker tracker[16][128];
  for (int c = 0; c < 16; c++) {
    for (int n = 0; n < 128; n++) {
      tracker[c][n].min_dist = 0xFFFF;
      tracker[c][n].type = 0;
    }
  }

  for (int e = 0; e < MAX_LOOP_EVENTS; e++) {
    if (tape_loop_buffer[e].active) {
      uint8_t c = tape_loop_buffer[e].channel;
      uint8_t n = tape_loop_buffer[e].note;
      if (c < 16 && n < 128) {
        uint32_t dist = (play_tick >= tape_loop_buffer[e].tick) ?
                        (play_tick - tape_loop_buffer[e].tick) :
                        (play_tick + len - tape_loop_buffer[e].tick);
        if (!include_current_tick && dist == 0) {
          continue;
        }
        if (dist < tracker[c][n].min_dist) {
          tracker[c][n].min_dist = dist;
          tracker[c][n].type = (tape_loop_buffer[e].type == AUDIO_EVT_NOTE_ON) ? 1 : 2;
          tracker[c][n].velocity = tape_loop_buffer[e].velocity;
          tracker[c][n].bank = tape_loop_buffer[e].bank;
          tracker[c][n].preset = tape_loop_buffer[e].preset;
        }
      }
    }
  }

  for (int c = 0; c < 16; c++) {
    for (int n = 0; n < 128; n++) {
      if (tracker[c][n].type == 1) {
        uint8_t vel = (tracker[c][n].velocity * stutter_volume) >> 15;
        if (vel > 0) {
          push_note_on_event(n, vel, tracker[c][n].bank, tracker[c][n].preset, c, false);
        }
      }
    }
  }
}

void tick_midi_sequencer() {
  // Check pending pattern changes even if not playing
  if (!midi_playing && pending_midi_idx != selectedMidiIdx && pending_midi_idx >= 0 && pending_midi_idx < (int32_t)num_midi_files) {
    selectedMidiIdx = pending_midi_idx;
    queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0xFF);
    start_midi_playback(midi_files[selectedMidiIdx].data, midi_files[selectedMidiIdx].size);
    midi_playing = false; // Stay paused
  }

  uint64_t now = time_us_64();
  if (last_seq_time_us == 0) {
    last_seq_time_us = now;
    return;
  }
  uint64_t dt_us = now - last_seq_time_us;
  if (dt_us > 100000)
    dt_us = 100000;
  last_seq_time_us = now;

  // If freeze tape-stop is active, the speed of clock ticks is scaled by glitch_speed_factor_q15
  uint64_t num = (uint64_t)dt_us * midi_division * glitch_speed_factor_q15 * 2;
  uint32_t ticks_to_process_q16 = num / midi_tempo_us;
  midi_tick_accumulator += ticks_to_process_q16;

  uint32_t ticks_to_step = midi_tick_accumulator >> 16;
  midi_tick_accumulator &= 0xFFFF;

  // Handle scrub changes even if ticks_to_step is 0
  if (freeze_active) {
    uint32_t len = 16 * midi_division;
    uint32_t scrub_start_tick = (param_scrub * len) / 4095;
    
    static uint32_t last_scrub_start = 0;
    if (scrub_start_tick != last_scrub_start) {
      last_scrub_start = scrub_start_tick;
      stutter_volume = 32767;

      // 1. Kill old notes
      queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0xFF);
      memset(sequencer_ratchets, 0, sizeof(sequencer_ratchets));
      elapsed_ticks = 0;

      // 2. Trigger sustaining notes at the new scrub playhead position immediately (include current tick)
      trigger_sustaining_notes(scrub_start_tick % len, true);
    }
  }

  if (ticks_to_step > 0) {
    if (freeze_active) {
      // Play events from tape loop buffer
      uint32_t len = 16 * midi_division;
      uint32_t scrub_start_tick = (param_scrub * len) / 4095;
      uint32_t freeze_len = get_stutter_length();

      for (uint32_t step = 0; step < ticks_to_step; step++) {
        // If we just wrapped around naturally, trigger sustaining notes at the beginning of the loop phase (exclude current tick because it will be processed by the normal event loop)
        if (elapsed_ticks > 0 && elapsed_ticks % freeze_len == 0) {
          uint32_t play_tick = (scrub_start_tick + (elapsed_ticks % freeze_len)) % len;
          trigger_sustaining_notes(play_tick, false);
        }

        // A. Process active sequencer ratchets on each sequencer tick
        for (int r = 0; r < 8; r++) {
          if (sequencer_ratchets[r].active) {
            if (sequencer_ratchets[r].tick_counter > 0) {
              sequencer_ratchets[r].tick_counter--;
            }
            if (sequencer_ratchets[r].tick_counter == 0) {
              // Trigger a repeat!
              // Send note-off first to avoid overlaps
              push_note_off_event(sequencer_ratchets[r].note, sequencer_ratchets[r].channel);
              // Trigger note-on
              push_note_on_event(sequencer_ratchets[r].note, sequencer_ratchets[r].velocity,
                                 sequencer_ratchets[r].bank, sequencer_ratchets[r].preset,
                                 sequencer_ratchets[r].channel, false);
              
              sequencer_ratchets[r].remaining_repeats--;
              if (sequencer_ratchets[r].remaining_repeats == 0) {
                sequencer_ratchets[r].active = false;
              } else {
                sequencer_ratchets[r].tick_counter = sequencer_ratchets[r].repeat_rate_ticks;
              }
            }
          }
        }

        uint32_t play_tick = (scrub_start_tick + (elapsed_ticks % freeze_len)) % len;
        
        // Calculate sequencer glitch probability based on freeze Y knob (0..32767, unipolar 0-100%)
        int32_t seq_glitch_prob = (param_freeze_detune * 100) / 32767;

        // 1. Step Address Scrambling: XOR play_tick address offsets
        if (seq_glitch_prob > 0 && (rand() % 100) < seq_glitch_prob) {
          uint32_t mask = 1 << (rand() % 4); // XOR with 1, 2, 4, or 8 ticks
          play_tick = play_tick ^ mask;
          if (play_tick >= len) play_tick = play_tick % len;
        }

        // Trigger tape loop events at play_tick
        for (int e = 0; e < MAX_LOOP_EVENTS; e++) {
          if (tape_loop_buffer[e].active && tape_loop_buffer[e].tick == play_tick) {
            if (tape_loop_buffer[e].type == AUDIO_EVT_NOTE_ON) {
              uint8_t vel = (tape_loop_buffer[e].velocity * stutter_volume) >> 15;
              if (vel > 0) {
                uint8_t note_to_trigger = tape_loop_buffer[e].note;
                uint16_t bank_to_trigger = tape_loop_buffer[e].bank;
                uint8_t preset_to_trigger = tape_loop_buffer[e].preset;
                
                // 2. Note circuit-bending: randomize note transpositions and ratchets
                if (seq_glitch_prob > 0 && (rand() % 100) < seq_glitch_prob) {
                  int32_t r_glitch = rand() % 100;
                  if (r_glitch < 60) {
                    // Schedule a Ratchet (repeat) with 60% probability
                    int free_slot = -1;
                    for (int r = 0; r < 8; r++) {
                      if (!sequencer_ratchets[r].active) {
                        free_slot = r;
                        break;
                      }
                    }
                    if (free_slot != -1) {
                      uint8_t repeats = 2 + (rand() % 3); // 2, 3, or 4 repeats
                      uint8_t rate = midi_division / 8; // 32nd note
                      if (rand() % 2 == 0) {
                        rate = midi_division / 16; // 64th note
                      }
                      if (rate < 1) rate = 1;
                      
                      sequencer_ratchets[free_slot].active = true;
                      sequencer_ratchets[free_slot].note = note_to_trigger;
                      sequencer_ratchets[free_slot].velocity = vel;
                      sequencer_ratchets[free_slot].bank = bank_to_trigger;
                      sequencer_ratchets[free_slot].preset = preset_to_trigger;
                      sequencer_ratchets[free_slot].channel = tape_loop_buffer[e].channel;
                      sequencer_ratchets[free_slot].remaining_repeats = repeats;
                      sequencer_ratchets[free_slot].repeat_rate_ticks = rate;
                      sequencer_ratchets[free_slot].tick_counter = rate;
                    }
                  } else {
                    // Transpose ±12 semitones with 40% probability
                    int32_t dt = -12 + (rand() % 25);
                    active_transpositions[tape_loop_buffer[e].channel][tape_loop_buffer[e].note] = dt;
                    int32_t transposed = (int32_t)note_to_trigger + dt;
                    if (transposed < 0) transposed = 0;
                    if (transposed > 127) transposed = 127;
                    note_to_trigger = (uint8_t)transposed;
                  }
                }
                
                push_note_on_event(note_to_trigger, vel, bank_to_trigger, preset_to_trigger, tape_loop_buffer[e].channel, false);
              }
            } else if (tape_loop_buffer[e].type == AUDIO_EVT_NOTE_OFF) {
              // Deactivate any matching active ratchet to avoid stuck notes
              for (int r = 0; r < 8; r++) {
                if (sequencer_ratchets[r].active &&
                    sequencer_ratchets[r].note == tape_loop_buffer[e].note &&
                    sequencer_ratchets[r].channel == tape_loop_buffer[e].channel) {
                  sequencer_ratchets[r].active = false;
                }
              }

              int8_t dt = active_transpositions[tape_loop_buffer[e].channel][tape_loop_buffer[e].note];
              int32_t transposed = (int32_t)tape_loop_buffer[e].note + dt;
              if (transposed < 0) transposed = 0;
              if (transposed > 127) transposed = 127;
              push_note_off_event((uint8_t)transposed, tape_loop_buffer[e].channel);
              active_transpositions[tape_loop_buffer[e].channel][tape_loop_buffer[e].note] = 0;
            } else if (tape_loop_buffer[e].type == AUDIO_EVT_ALL_NOTES_OFF) {
              push_all_notes_off_event(tape_loop_buffer[e].channel);
              // Deactivate all ratchets on this channel
              for (int r = 0; r < 8; r++) {
                if (sequencer_ratchets[r].active && sequencer_ratchets[r].channel == tape_loop_buffer[e].channel) {
                  sequencer_ratchets[r].active = false;
                }
              }
            }
          }
        }
        elapsed_ticks++;
        midi_sequence_ticks++;

        // Send All Notes Off on loop wrap-around to prevent stuck notes
        if (elapsed_ticks % freeze_len == 0) {
          queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0xFF);
          // Reset all active ratchets on loop wrap-around
          memset(sequencer_ratchets, 0, sizeof(sequencer_ratchets));
        }

        // Bar-aligned Pattern Switching during freeze
        if (midi_sequence_ticks % (4 * midi_division) == 0) {
          if (pending_midi_idx != selectedMidiIdx && pending_midi_idx >= 0 && pending_midi_idx < (int32_t)num_midi_files) {
            selectedMidiIdx = pending_midi_idx;
            queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0xFF);
            start_midi_playback(midi_files[selectedMidiIdx].data, midi_files[selectedMidiIdx].size);
            if (freeze_active) {
              freeze_active = false;
            }
            return;
          }
        }
      }
    } else {
      // RECORD/NORMAL MODE FOR TAPE LOOP:
      // Erase tape loop events as the write head moves forward
      erase_tape_ticks(midi_sequence_ticks, ticks_to_step);
      
      // Step the MIDI file player ONLY if midi_playing is true
      if (midi_playing) {
        bool sync_active = card.Connected(ComputerCard::Input::Pulse1) || 
                           ((last_midi_clock_time_us > 0) && (time_us_64() - last_midi_clock_time_us < 2000000));
        if (sync_active) {
          if (midi_sequence_ticks < midi_next_boundary_tick) {
            uint32_t max_allowed = midi_next_boundary_tick - midi_sequence_ticks;
            if (ticks_to_step > max_allowed) {
              ticks_to_step = max_allowed;
            }
            step_midi_sequencer_ticks(ticks_to_step);
          }
        } else {
          step_midi_sequencer_ticks(ticks_to_step);
        }
      } else {
        // If not playing, still advance the sequence ticks so tape loop recording matches real time
        uint32_t len = 16 * midi_division;
        for (uint32_t step = 0; step < ticks_to_step; step++) {
          midi_sequence_ticks = (midi_sequence_ticks + 1) % len;
          if (midi_sequence_ticks % midi_division == 0) {
            midi_last_beat_counter = midi_sequence_ticks / midi_division;
            if (midi_sequence_ticks % (4 * midi_division) == 0) {
              if (!freeze_active) {
                midi_bar_start_tick = midi_sequence_ticks;
              }
            }
            if (!freeze_active) {
              midi_beat_start_tick = midi_sequence_ticks;
            }
          }
        }
      }
    }
  }
}

void stop_all_notes_and_gates() {
  clear_voices_request = true;
  cv1_tracker.clear();
  cv2_tracker.clear();
  cv1_out_gate = false;
  cv2_out_gate = false;
  card.PulseOut1(false);
  card.PulseOut2(false);
}

static inline int32_t quantize_semitone(int32_t raw_pitch_q8) {
  if (raw_pitch_q8 >= 0) {
    return (raw_pitch_q8 + 128) / 256;
  } else {
    return (raw_pitch_q8 - 128) / 256;
  }
}

static uint32_t rand_seed = 12345;
static inline uint32_t quick_rand() {
  rand_seed = rand_seed * 1103515245 + 12345;
  return (rand_seed / 65536) % 32768;
}

void push_note_on_event(uint8_t note, uint8_t velocity, uint16_t bank,
                        uint16_t preset, uint8_t channel,
                        bool apply_pitch_knob) {
  if (!midi_playback_mode && card.Disconnected(ComputerCard::Input::Pulse2)) {
    pulse2_out_timer = 480; // 10ms trigger pulse at 48kHz
  }

  // 1. Get channel target and check overrides
  int32_t target = global_config.channels[channel].target;
  if (target == 4) {
    return; // Disabled channel
  }

  record_tape_event(AUDIO_EVT_NOTE_ON, note, velocity, channel);

  // Instrument override:
  int32_t override_idx = global_config.channels[channel].instrument_override;
  if (override_idx >= 0 && override_idx < (int32_t)indexed_presets_count) {
    bank = presets_index[override_idx].bank;
    preset = presets_index[override_idx].preset;
  }

  // 2. Mutate probability P (0..100%): Knob Y
  int32_t mutate_val = 0;
  if (midi_playback_mode) {
    mutate_val = param_mutate;
  }

  // Thin out / skip note on mutation (above 66% mutation, up to 40% skip rate)
  if (midi_playback_mode && mutate_val > 21845) {
    uint32_t skip_chance = ((mutate_val - 21845) * 40) / (32767 - 21845);
    if ((quick_rand() % 100) < skip_chance) {
      return; // Skip note!
    }
  }

  // 3. Apply Transpose (don't transpose drums on channel 9 or bank 128)
  int32_t offset_note = note;
  if (midi_playback_mode) {
    if (channel != 9 && bank != 128) {
      // CV1 Transpose Offset
      int32_t cv1_transpose_offset = 0;
      if (card.Connected(ComputerCard::Input::CV1)) {
        int32_t cv1_raw = card.CVIn1();
        cv1_transpose_offset = quantize_semitone((cv1_raw * 15) >> 1);
      }
      
      int32_t mutation_pitch_offset = 0;
      if (mutate_val > 0) {
        // We have three zones of mutation:
        if (mutate_val > 10922) { // Above 33%
          // Octave shift mutation
          uint32_t r = quick_rand() % 100;
          uint32_t shift_chance = (mutate_val * 50) / 32767; // up to 50% chance
          if (r < shift_chance) {
            mutation_pitch_offset = (quick_rand() % 2 == 0) ? 12 : -12;
          } else if (r < (shift_chance * 2) / 3 && mutate_val > 21845) { // Above 66%
            mutation_pitch_offset = (quick_rand() % 2 == 0) ? 24 : -24;
          }
        }
        
        if (mutate_val > 21845) { // Above 66%
          // Add musical intervals (+7, -5) with low probability (20% chance)
          if (quick_rand() % 100 < 20) {
            mutation_pitch_offset += (quick_rand() % 2 == 0) ? 7 : -5;
          }
        }
      }
      
      offset_note += midi_transpose + cv1_transpose_offset + mutation_pitch_offset;
    }
  } else if (apply_pitch_knob) {
    offset_note += page_coarse;
  }
  if (offset_note < 0)
    offset_note = 0;
  if (offset_note > 127)
    offset_note = 127;

  // 4. Apply Velocity Scaling and Mutation Randomization
  uint8_t scaled_velocity = velocity;
  if (midi_playback_mode) {
    if (channel != 9 && bank != 128) {
      int32_t sc_vel = (velocity * param_velocity_scaling) / 100;
      
      // Introduce velocity randomization (humanization) based on mutate_val
      if (mutate_val > 0) {
        int32_t rand_range = (mutate_val * 40) / 32767; // up to +/- 40 velocity units
        if (rand_range > 0) {
          sc_vel += ((int32_t)(quick_rand() % (rand_range * 2 + 1)) - rand_range);
        }
      }
      
      if (sc_vel < 1) sc_vel = 1;
      if (sc_vel > 127) sc_vel = 127;
      scaled_velocity = (uint8_t)sc_vel;
    }
  }

  // 5. Handle CV/Gate Routing Targets
  if (target == 2 || target == 5) { // CV1 / Pulse 1
    cv1_tracker.push(note);
    cv1_out_note = cv1_tracker.get_active_note();
    cv1_out_channel = channel;
    cv1_out_gate = true;
    cv1_trigger_timer = 480; // Start 10ms trigger pulse
    if (target == 2) return;
  }
  
  if (target == 3 || target == 6) { // CV2 / Pulse 2
    cv2_tracker.push(note);
    cv2_out_note = cv2_tracker.get_active_note();
    cv2_out_channel = channel;
    cv2_out_gate = true;
    cv2_trigger_timer = 480; // Start 10ms trigger pulse
    if (target == 3) return;
  }

  ResolvedParams zones[4];
  int zone_count =
      collect_note_zones(bank, preset, offset_note, scaled_velocity, zones, 4);

  for (int z = 0; z < zone_count; z++) {
    const ResolvedParams &params = zones[z];

    AudioEvent evt;
    evt.type = EVT_NOTE_ON;
    evt.channel = channel;
    evt.note = note;
    evt.velocity = scaled_velocity;

    evt.voice_data.sample_data = params.sample_data;
    evt.voice_data.sample_end = params.end - params.start;
    evt.voice_data.loop_start = params.loop_start;
    evt.voice_data.loop_end = params.loop_end;
    evt.voice_data.is_looped = params.is_looped;

    int32_t semitone_cents =
        ((int32_t)(offset_note - params.root_key) * params.scale_tuning);
    int32_t final_cents = semitone_cents - params.pitch_correction -
                          params.coarse_tune * 100 - params.fine_tune;
    uint32_t pitch_mult = cents_to_pitch_multiplier_q16(final_cents);
    uint32_t base_ratio = ((uint64_t)params.sample_rate << 16) / 48000;
    evt.voice_data.phase_increment = (uint64_t)base_ratio * pitch_mult;

    uint32_t attack_ms = timecents_to_ms(page_attack);
    uint32_t release_ms;
    if (channel == 9 || target == 1 || channel == 1) {
      int32_t page_drum_release = -12000 + (param_drum_release * 15000 / 32767);
      release_ms = timecents_to_ms(page_drum_release);
    } else {
      release_ms = timecents_to_ms(page_release);
    }
    evt.voice_data.env_sustain = 0;
    evt.voice_data.env_attack_step = (2147483647LL * 32) / (48 * attack_ms);
    if (evt.voice_data.env_attack_step < 1)
      evt.voice_data.env_attack_step = 1;
    evt.voice_data.env_decay_step = 32;
    evt.voice_data.env_release_step = (2147483647LL * 32) / (48 * release_ms);
    if (evt.voice_data.env_release_step < 1)
      evt.voice_data.env_release_step = 1;

    evt.voice_data.mod_env_to_pitch = params.mod_env_to_pitch;
    if (params.mod_env_to_pitch != 0) {
      uint32_t ma_ms = timecents_to_ms(params.mod_attack);
      uint32_t md_ms = timecents_to_ms(params.mod_decay);
      uint32_t mr_ms = timecents_to_ms(params.mod_release);
      evt.voice_data.mod_env_attack_step = (2147483647LL * 32) / (48 * ma_ms);
      if (evt.voice_data.mod_env_attack_step < 1)
        evt.voice_data.mod_env_attack_step = 1;

      int32_t sus_norm = 1000 - params.mod_sustain;
      if (sus_norm < 0)
        sus_norm = 0;
      if (sus_norm > 1000)
        sus_norm = 1000;
      evt.voice_data.mod_env_sustain = ((uint64_t)sus_norm * 2147483647) / 1000;

      evt.voice_data.mod_env_decay_step = (2147483647LL * 32) / (48 * md_ms);
      if (evt.voice_data.mod_env_decay_step < 1)
        evt.voice_data.mod_env_decay_step = 1;

      evt.voice_data.mod_env_release_step = (2147483647LL * 32) / (48 * mr_ms);
      if (evt.voice_data.mod_env_release_step < 1)
        evt.voice_data.mod_env_release_step = 1;    } else {
      evt.voice_data.mod_env_attack_step = 0;
      evt.voice_data.mod_env_decay_step = 0;
      evt.voice_data.mod_env_release_step = 0;
      evt.voice_data.mod_env_sustain = 0;
    }

    int32_t cb = params.initial_attenuation;
    if (cb <= 0) {
      evt.voice_data.vol_atten_q15 = 32767;
    } else if (cb >= 960) {
      evt.voice_data.vol_atten_q15 = 0;
    } else {
      int32_t shifts = cb / 60;
      int32_t rem = cb % 60;
      int32_t fine = 32767 - (rem * 32767 / 60) / 2;
      evt.voice_data.vol_atten_q15 = (fine >> shifts);
      if (evt.voice_data.vol_atten_q15 < 1)
        evt.voice_data.vol_atten_q15 = 1;
    }

    int32_t fc_cents = params.filter_cutoff;
    int32_t norm = fc_cents - 1500;
    if (norm < 0)
      norm = 0;
    if (norm > 12000)
      norm = 12000;
    evt.voice_data.filter_fc_base = 500 + ((norm * 15) >> 2);

    evt.voice_data.scale_tuning = params.scale_tuning;

    uint32_t next_head = (audio_event_head + 1) % AUDIO_EVENT_QUEUE_SIZE;
    while (next_head == audio_event_tail) {
      tight_loop_contents();
    }
    audio_event_queue[audio_event_head] = evt;
    audio_event_head = next_head;
  }
}

void push_note_off_event(uint8_t note, uint8_t channel) {
  int32_t target = global_config.channels[channel].target;
  if (target == 4) {
    return;
  }

  record_tape_event(AUDIO_EVT_NOTE_OFF, note, 0, channel);

  if (target == 2 || target == 5) { // CV1
    cv1_tracker.remove(note);
    int32_t active = cv1_tracker.get_active_note();
    if (active != -1) {
      cv1_out_note = active;
      cv1_out_gate = true;
    } else {
      cv1_out_gate = false;
    }
    if (target == 2) return;
  }
  
  if (target == 3 || target == 6) { // CV2
    cv2_tracker.remove(note);
    int32_t active = cv2_tracker.get_active_note();
    if (active != -1) {
      cv2_out_note = active;
      cv2_out_gate = true;
    } else {
      cv2_out_gate = false;
    }
    if (target == 3) return;
  }

  AudioEvent evt;
  evt.type = EVT_NOTE_OFF;
  evt.channel = channel;
  evt.note = note;
  evt.velocity = 0;

  uint32_t next_head = (audio_event_head + 1) % AUDIO_EVENT_QUEUE_SIZE;
  while (next_head == audio_event_tail) {
    tight_loop_contents();
  }
  audio_event_queue[audio_event_head] = evt;
  audio_event_head = next_head;
}

void push_all_notes_off_event(uint8_t channel) {
  if (channel == 0xFF) {
    cv1_tracker.clear();
    cv1_out_gate = false;
    cv2_tracker.clear();
    cv2_out_gate = false;
  } else {
    int32_t target = global_config.channels[channel].target;
    if (target == 2 || target == 5) {
      cv1_tracker.clear();
      cv1_out_gate = false;
    }
    if (target == 3 || target == 6) {
      cv2_tracker.clear();
      cv2_out_gate = false;
    }
  }

  AudioEvent evt;
  evt.type = EVT_ALL_NOTES_OFF;
  evt.channel = channel;
  evt.note = 0;
  evt.velocity = 0;

  uint32_t next_head = (audio_event_head + 1) % AUDIO_EVENT_QUEUE_SIZE;
  while (next_head == audio_event_tail) {
    tight_loop_contents();
  }
  audio_event_queue[audio_event_head] = evt;
  audio_event_head = next_head;
}

void start_resolved_voice_from_event(const TriggeredVoiceEvent &params,
                                     uint8_t note, uint8_t velocity,
                                     uint8_t channel) {
  int voice_to_use = -1;
  int32_t quietest_env = 2147483647;
  int quietest_voice = -1;
  uint32_t oldest_age = 0xFFFFFFFF;
  int oldest_voice = -1;

  for (int i = 0; i < MAX_VOICES; i++) {
    if (voices[i].state == VOICE_OFF) {
      voice_to_use = i;
      break;
    }
    if (voices[i].state == VOICE_RELEASE && voices[i].env_val < quietest_env) {
      quietest_env = voices[i].env_val;
      quietest_voice = i;
    }
    if (voices[i].age < oldest_age) {
      oldest_age = voices[i].age;
      oldest_voice = i;
    }
  }

  if (voice_to_use == -1) {
    if (quietest_voice != -1)
      voice_to_use = quietest_voice;
    else
      voice_to_use = oldest_voice;
  }

  if (voice_to_use == -1)
    return;

  SynthVoice &v = voices[voice_to_use];
  v.note = note;
  v.midi_note = note;
  v.channel = channel;
  v.key_down = true;
  v.phase = 0;
  v.age = ++global_voice_age;
  v.velocity = velocity;
  v.scale_tuning = params.scale_tuning;
  v.micro_loop_start = 0;
  v.micro_loop_size = 0;
  v.glitch_wrong_idx = -1;
  v.glitch_state = 0;
  v.glitch_timer = 0;
  v.glitch_duration = 0;

  v.phase_increment = params.phase_increment;

  v.env_sustain = params.env_sustain;
  v.env_attack_step = params.env_attack_step;
  v.env_decay_step = params.env_decay_step;
  v.env_release_step = params.env_release_step;

  v.mod_env_attack_step = params.mod_env_attack_step;
  v.mod_env_decay_step = params.mod_env_decay_step;
  v.mod_env_release_step = params.mod_env_release_step;
  v.mod_env_sustain = params.mod_env_sustain;
  v.mod_env_to_pitch = params.mod_env_to_pitch;
  v.mod_env_val = 0;
  v.mod_env_state = VOICE_ATTACK;

  v.vol_atten_q15 = params.vol_atten_q15;
  v.filter_fc_base = params.filter_fc_base;

  v.filter_f = v.filter_fc_base;
  v.filter_q = 120000;
  v.filter_low = 0;
  v.filter_band = 0;

  v.sample_data = params.sample_data;
  v.sample_start = 0;
  v.sample_end = params.sample_end;
  v.loop_start = params.loop_start;
  v.loop_end = params.loop_end;
  v.is_looped = params.is_looped;

  v.env_val = 0;
  v.state = VOICE_ATTACK;
  v.glitch_timer = 0;
  v.glitch_duration = 0;
  v.glitch_state = 0;
  v.glitch_mangle_held = 0;
  v.glitch_wrong_idx = -1;
}

void trigger_note_off_channel(uint8_t note, uint8_t channel) {
  if (channel == 9) return; // Drum channel ignores Note Off to let percussion samples play to completion
  for (int i = 0; i < MAX_VOICES; i++) {
    if (voices[i].state != VOICE_OFF && voices[i].midi_note == note &&
        voices[i].channel == channel) {
      voices[i].key_down = false;
      voices[i].state = VOICE_RELEASE;
    }
  }
}

void trigger_note_off(uint8_t note) { trigger_note_off_channel(note, 0); }

// Core 1 FIFO consumer background loop
void core1_dsp_loop() {
  multicore_lockout_victim_init();
  // Enable Normalisation Detection on Core 1 audio interrupts
  card.EnableNormalisationProbe();

  // Initialize voices pool
  for (int i = 0; i < MAX_VOICES; i++) {
    voices[i].state = VOICE_OFF;
  }

  // Start core 1 audio DMA interrupts (never returns)
  card.Run();
}

// MIDI Mount Callbacks
extern "C" {
void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep,
                       uint8_t num_cables_rx, uint16_t num_cables_tx) {
  (void)in_ep;
  (void)out_ep;
  (void)num_cables_rx;
  (void)num_cables_tx;
  (void)dev_addr;
}

void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance) {
  (void)instance;
  (void)dev_addr;
}

void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets) {
  while (num_packets--) {
    uint8_t packet[4];
    if (tuh_midi_packet_read(dev_addr, packet)) {
      handle_midi_message(packet + 1, 3);
    }
  }
}

void handle_midi_message(uint8_t *packet, int size) {
  if (size > 0) {
    uint8_t status = packet[0];
    if (status == 0xF8) {
      mclk_trigger_occurred = true;
      last_midi_clock_time_us = time_us_64();
      return;
    }
    if (status == 0xFA || status == 0xFB) {
      if (!midi_playing) {
        start_midi_playback(midi_files[selectedMidiIdx].data, midi_files[selectedMidiIdx].size);
      }
      return;
    }
    if (status == 0xFC) {
      if (midi_playing) {
        midi_playing = false;
        stop_all_notes_and_gates();
      }
      return;
    }
  }

  if (size < 3) {
    if (size == 2) {
      uint8_t status = packet[0];
      uint8_t type = status & 0xF0;
      uint8_t channel = status & 0x0F;
      if (type == 0xC0) {
        uint8_t program = packet[1];
        channel_presets[channel] = program;
        if (channel == 0) {
          master_preset = program;
          selectedPreset = program;
          activeBank = channel_banks[0];
        }
        queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0, 0, channel);
      }
    }
    return;
  }
  uint8_t status = packet[0];
  uint8_t type = status & 0xF0;
  uint8_t channel = status & 0x0F;
  uint8_t note = packet[1];
  uint8_t velocity = packet[2];

  if (type == 0x90 && velocity > 0) {
    queue_audio_event(AUDIO_EVT_NOTE_ON, note, velocity, channel);
  } else if (type == 0x80 || (type == 0x90 && velocity == 0)) {
    queue_audio_event(AUDIO_EVT_NOTE_OFF, note, 0, channel);
  } else if (type == 0xB0) {
    uint8_t controller = packet[1];
    uint8_t value = packet[2];
    int32_t val_q15 = (value * 32767) / 127;
    ComputerCard::Switch current_sw = card.SwitchVal();

    if (controller == global_config.cc_maps[0] || controller == 34 || controller == 74) { // Cutoff
      param_cutoff = val_q15;
      if (current_sw == ComputerCard::Switch::Up) {
        lockMain.engage(card.KnobVal(ComputerCard::Knob::Main));
      }
    } else if (controller == global_config.cc_maps[2] || controller == 35 || controller == 71 ||
               controller == 70) { // Modulation Depth / Vibrato
      param_res = val_q15;
      if (current_sw == ComputerCard::Switch::Up) {
        lockX.engage(card.KnobVal(ComputerCard::Knob::X));
      }
    } else if (controller == 1) { // Mod Wheel
      midi_mod_wheel = val_q15;
    } else if (controller == global_config.cc_maps[3] || controller == 36) { // Attack
      param_attack = val_q15;
      if (current_sw == ComputerCard::Switch::Middle) {
        lockX.engage(card.KnobVal(ComputerCard::Knob::X));
      }
    } else if (controller == global_config.cc_maps[4] || controller == 37) { // Decay
      param_decay = val_q15;
      param_release = val_q15; // Release tracks Decay
      if (current_sw == ComputerCard::Switch::Middle) {
        lockY.engage(card.KnobVal(ComputerCard::Knob::Y));
      }
    } else if (controller == 38) { // Release
      param_release = val_q15;
    } else if (controller == global_config.cc_maps[1] || controller == 39) { // Reverb Mix
      param_reverb_mix = val_q15;
      if (current_sw == ComputerCard::Switch::Up) {
        lockY.engage(card.KnobVal(ComputerCard::Knob::Y));
      }
    } else if (controller == 41) { // Pitch Offset
      param_pitch = val_q15;
      if (current_sw == ComputerCard::Switch::Middle) {
        lockMain.engage(card.KnobVal(ComputerCard::Knob::Main));
      }
    } else if (controller == 0) {
      midi_bank_msb = value & 0x7F;
      channel_banks[channel] =
          ((uint16_t)midi_bank_msb << 7) | (channel_banks[channel] & 0x007F);
      if (channel == 9) {
        channel_banks[channel] = 128;
      }
      if (channel == 0) {
        master_bank = channel_banks[0];
        activeBank = master_bank;
      }
    } else if (controller == 32) {
      midi_bank_lsb = value & 0x7F;
      channel_banks[channel] =
          (channel_banks[channel] & 0x3F80) | (midi_bank_lsb & 0x7F);
      if (channel == 9) {
        channel_banks[channel] = 128;
      }
      if (channel == 0) {
        master_bank = channel_banks[0];
        activeBank = master_bank;
      }
    } else if (controller == 120 || controller == 123) {
      queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0, 0, channel);
    }

    // Precompute dynamic values from parameters
    page_cutoff = -12000 + (param_cutoff * 22000 / 32767);
    page_res = (param_res * 100) / 32767; // Vibrato depth in cents
    page_attack = -12000 + (param_attack * 15000 / 32767);
    page_decay = -12000 + (param_decay * 15000 / 32767);
    page_release = -12000 + (param_release * 15000 / 32767);
    page_coarse = -36 + (param_pitch * 72) / 32767;
    page_reverb_mix = param_reverb_mix;

    uint32_t attack_ms = timecents_to_ms(page_attack);
    global_env_attack_step = 2147483647 / (48 * attack_ms);
    if (global_env_attack_step < 1)
      global_env_attack_step = 1;

    uint32_t release_ms = timecents_to_ms(page_release);
    global_env_release_step = 2147483647 / (48 * release_ms);
    if (global_env_release_step < 1)
      global_env_release_step = 1;
  } else if (type == 0xC0) {
    uint8_t program = packet[1];
    channel_presets[channel] = program;
    if (channel == 0) {
      master_preset = program;
      selectedPreset = program;
      activeBank = channel_banks[0];
    }
    queue_audio_event(AUDIO_EVT_ALL_NOTES_OFF, 0, 0, channel);
  } else if (type == 0xE0) {
    uint16_t bend_val = packet[1] | (packet[2] << 7); // 0..16383, 8192 is center
    int32_t bend_cents = ((int32_t)bend_val - 8192) * 200 / 8192; // Default range: +/- 2 semitones (+/- 200 cents)
    channel_bend_mult_q16[channel] = cents_to_pitch_multiplier_q16(bend_cents);
  }
}
}

int main() {
  // ── Clock Setup (Overclock to 240MHz @ 1.25V) ──
  vreg_set_voltage(VREG_VOLTAGE_1_25);
  sleep_ms(10);
  set_sys_clock_khz(240000, true);

  // Boot breadcrumbs: LED 0 before SF2 parse, LED 1 after parse.
  // If SF2 parsing locks up, watchdog reboots once and LED 2 marks that the
  // next boot skipped parsing so the card can still come up.
  card.BootLed(0, true);

  const bool parser_watchdog_reboot =
      watchdog_hw->reason &&
      watchdog_hw->scratch[0] == SF2_PARSE_WATCHDOG_MAGIC;
  watchdog_hw->scratch[0] = 0;

  if (parser_watchdog_reboot) {
    card.BootLed(2, true);
  } else {
    watchdog_hw->scratch[0] = SF2_PARSE_WATCHDOG_MAGIC;
    watchdog_enable(750, false);
    parse_sf2();
    watchdog_disable();
    watchdog_hw->scratch[0] = 0;
  }
  scan_midi_files();
  load_persistent_settings();
  apply_mode_configuration();
  card.BootLed(1, true);

  // Initialize Reverb
  reverb.Init();

  // Create and initialize Core 0 control logic
  card.InitUSB();

  // Initialize Core 0 <-> Core 1 audio queues
  queue_init(&core1_to_core0_queue, sizeof(AudioFrame), 128);
  queue_init(&core0_to_core1_queue, sizeof(AudioFrame), 128);

  // Launch Core 1 Audio rendering engine
  multicore_launch_core1(core1_dsp_loop);

  // Core 0 polling loop
  while (1) {
    if (isUSBMIDIHost) {
      tuh_task();
    } else {
      tud_task();
    }

    // Run UI Loop
    card.BackgroundLoop();

    // Process all pending audio frames from Core 1
    card.service_audio_queue();

    sleep_us(250);
  }
}
