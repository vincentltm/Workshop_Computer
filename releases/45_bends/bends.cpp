#define COMPUTERCARD_SAMPLE_RATE_DIV 1 // Run at full 48kHz sample rate

#include "ComputerCard.h"
#include "fixed_math.h"
#include "dsp_blocks.h"
#include "samples.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/vreg.h"
#include <atomic>
#include "hardware/flash.h"
#include "hardware/sync.h"

// ============================================================================
// Grains-Compatible Sample Manager Compatibility
// ============================================================================
struct FlashHeader {
  uint32_t magic;
  uint32_t count;
};
struct FlashSampleEntry {
  uint32_t offset;
  uint32_t length;
  char name[24];
};
#define FLASH_SAMPLE_ADDR 0x10100000
#define SAVE_FLASH_OFFSET 0x000F0000
#define SAVE_MAGIC        0xBE4D5003

uint32_t num_flash_samples = 0;
bool use_flash_samples = false;

void check_flash_samples() {
    FlashHeader *header = (FlashHeader *)FLASH_SAMPLE_ADDR;
    if ((uint32_t)header >= 0x10000000 && (uint32_t)header < 0x11000000 && header->magic == 0x6772616E) {
        num_flash_samples = header->count;
        if (num_flash_samples > 28) num_flash_samples = 28;
        if (num_flash_samples > 0) {
            // Validate the first sample entry to prevent uninitialised or corrupt flash header readings
            FlashSampleEntry *entries = (FlashSampleEntry *)(FLASH_SAMPLE_ADDR + 8);
            uint32_t offset = entries[0].offset;
            uint32_t length = entries[0].length;
            if (offset < 0x01000000 && length > 0 && length < 10000000) {
                use_flash_samples = true;
            }
        }
    }
}

const int16_t *get_active_sample_ptr(int idx, uint32_t &size) {
    if (use_flash_samples && idx >= 0 && idx < (int)num_flash_samples) {
        FlashSampleEntry *entries = (FlashSampleEntry *)(FLASH_SAMPLE_ADDR + 8);
        size = entries[idx].length;
        const int16_t *ptr = (const int16_t *)(FLASH_SAMPLE_ADDR + entries[idx].offset);
        // Fallback to internal ambient samples if offset/length is invalid or ptr is null
        if (ptr == nullptr || size == 0 || entries[idx].offset >= 0x01000000) {
            int fallback_idx = idx;
            if (fallback_idx < 0) fallback_idx = 0;
            if (fallback_idx >= NUM_AMBIENT_SAMPLES) fallback_idx = fallback_idx % NUM_AMBIENT_SAMPLES;
            size = AMBIENT_SAMPLE_LEN;
            return ambient_samples[fallback_idx];
        }
        return ptr;
    } else {
        int fallback_idx = idx;
        if (fallback_idx < 0) fallback_idx = 0;
        if (fallback_idx >= NUM_AMBIENT_SAMPLES) fallback_idx = fallback_idx % NUM_AMBIENT_SAMPLES;
        size = AMBIENT_SAMPLE_LEN;
        return ambient_samples[fallback_idx];
    }
}

int get_total_samples_count() {
    if (use_flash_samples) {
        return num_flash_samples;
    } else {
        return NUM_AMBIENT_SAMPLES;
    }
}

// Atomic active sample index (Core 0 -> Core 1)
std::atomic<bool>    atom_flash_writing{false};

// DC Blockers for dual inputs
DCBlocker dc_inL;
DCBlocker dc_inR;

// Ambient playhead state (64-bit for overflow protection on long samples)
uint64_t amb_playhead_q16 = 0;
int32_t amb_lfo_phase = 0;

// Persistent sample player parameters (latched when exiting Switch UP page)
int32_t amb_loop_len = -1;  // initialized to -1 to detect first-time setup
int32_t amb_start_pos = 0;
int32_t amb_speed_mult = 32768; // 1.0x in Q15


inline int32_t apply_deadzone(int32_t val) {
    if (val < 500) return 0;
    if (val > 32200) return 32767;
    return val;
}

// ============================================================================
// Global DSP Instances & State Variables
// ============================================================================
int16_t sine_table[SINE_TABLE_SIZE + 1];

// Random number generator seed — accessed exclusively on Core 1, no atomic needed
uint32_t rand_seed = 987654321u;

// Sequential DSP block instances — all live in SRAM (global scope)
ChorusBlock       chorus;
CodecDemolisherBlock codec;
MultiTapDelayBlock   delay_fx;  // avoid shadowing std::delay
GlitcherBlock     glitcher;
FilterBlock       filter;
ReverbBlock       reverb;

// ============================================================================
// Core Parameter Double-Buffer Struct (Core 0 → Core 1 Param Sharing)
// ============================================================================
struct Core1Params {
    int32_t chorus_mix;
    int32_t chorus_rate;
    int32_t chorus_depth_fb;

    int32_t codec_mix;
    int32_t codec_downsample;
    int32_t codec_ringing_xor;

    int32_t delay_mix;
    int32_t delay_time;
    int32_t delay_feedback;

    int32_t glitch_mix;
    int32_t glitch_size;
    int32_t glitch_speed;
    int32_t glitch_feedback;
    int32_t global_noise_scale;

    int32_t filter_cutoff;
    int32_t filter_res;
    int32_t filter_morph;

    int32_t cv1;
    int32_t cv2;

    int32_t active_sample;
    int32_t amb_loop_len;
    int32_t amb_start_pos;
    int32_t amb_speed_mult;
    int32_t amb_scrub_pos;
    int32_t grittiness_macro;

    bool freeze;
    bool stutter;
    bool no_audio1;
    bool no_audio2;
    bool is_sample_page;
    bool is_freeze_page;
    bool flash_writing;
};

volatile Core1Params g_params[2];
std::atomic<uint32_t> g_params_idx{0};

// Core 0 Reverb shared variables (written in UI loop, read in Reverb process)
static int32_t c0_reverb_mix = 0;
static int32_t c0_reverb_size = 16384;
static int32_t c0_reverb_fb_glitch = 16384;

// Visual feedback (Core 1 → Core 0)
std::atomic<uint16_t> vis_lfo_phase{0};       // Chorus LFO phase for LED glow
std::atomic<bool>     vis_frame_dropped{false}; // Codec dropout for LED flash

// ============================================================================
// ComputerCard Subclass
// ============================================================================
class BendsCard : public ComputerCard {
public:
    void ProcessSample() override;
    void BackgroundLoop() override {}

    // UI / control loop runs on Core 0 as a member to access protected I/O
    void run_core0_ui_loop();
    void tick_ui_once();

    // Public wrappers so main() can access protected ComputerCard members
    bool    JackDisconnected(Input i)   { return Disconnected(i); }
    int32_t ReadKnob(Knob k)            { return KnobVal(k); }
};

BendsCard card;

inline int32_t scale_grit(int32_t val, int32_t max_val, int32_t macro) {
    if (macro < 16384) {
        return (val * macro) >> 14;
    } else {
        int32_t diff = max_val - val;
        return val + ((diff * (macro - 16384)) / 16383);
    }
}

// ============================================================================
// Core 1: ProcessSample() — 48 kHz audio interrupt
// ============================================================================
// Marked __not_in_flash_func so the entire function runs from fast SRAM,
// avoiding XIP flash bus contention with Core 0.
void __not_in_flash_func(BendsCard::ProcessSample)() {

    // --- Load all parameters in a single burst from the double buffer ---
    const uint32_t params_idx     = g_params_idx.load(std::memory_order_relaxed);
    const volatile Core1Params &p = g_params[params_idx];

    const int32_t chorus_mix      = apply_deadzone(p.chorus_mix);
    const int32_t chorus_rate     = p.chorus_rate;
    const int32_t chorus_depth_fb = p.chorus_depth_fb;

    const int32_t codec_mix         = apply_deadzone(p.codec_mix);
    const int32_t codec_downsample  = p.codec_downsample;
    const int32_t codec_ringing_xor = p.codec_ringing_xor;

    const int32_t delay_mix      = apply_deadzone(p.delay_mix);
    const int32_t delay_time     = p.delay_time;
    const int32_t delay_feedback = p.delay_feedback;

    const int32_t glitch_mix      = apply_deadzone(p.glitch_mix);
    const int32_t glitch_size     = p.glitch_size;
    const int32_t glitch_speed    = p.glitch_speed;
    const int32_t glitch_feedback = p.glitch_feedback;
    const int32_t global_noise_scale = p.global_noise_scale;

    const int32_t filter_cutoff = p.filter_cutoff;
    const int32_t filter_res    = p.filter_res;
    const int32_t filter_morph  = p.filter_morph;

    const bool    freeze  = p.freeze;
    const bool    stutter = p.stutter;
    const int32_t cv1    = p.cv1;
    const int32_t cv2    = p.cv2;

    const bool no_audio1 = p.no_audio1;
    const bool no_audio2 = p.no_audio2;
    const bool is_freeze_page = p.is_freeze_page;
    const bool flash_writing = p.flash_writing;
    const int32_t grittiness_macro = p.grittiness_macro;

    // use_samples: true when no external audio source on Audio 1
    bool use_samples = no_audio1; // audio input takes priority over sample player

    int16_t L = 0;
    int16_t R = 0;

    // Crossfade state for click-free routing mode switches
    static bool    prev_use_samples  = true;
    static int32_t route_xfade_ctr   = 0;
    static int16_t route_xfade_from_L = 0; // last output of the OLD mode
    static int16_t route_xfade_from_R = 0;
    static int16_t last_pre_dc_L = 0;      // previous tick's raw signal (before DC blocker)
    static int16_t last_pre_dc_R = 0;
    const  int32_t ROUTE_XFADE_LEN  = 256; // ~5.3 ms @ 48 kHz

    if (prev_use_samples != use_samples) {
        // Mode just switched — start xfade from last tick's signal
        route_xfade_from_L = last_pre_dc_L;
        route_xfade_from_R = last_pre_dc_R;
        route_xfade_ctr    = ROUTE_XFADE_LEN;
        prev_use_samples   = use_samples;
    }

    if (use_samples) {
        if (flash_writing) {
            L = R = 0;
        } else {
            // ── Sample Player / Ambient Glitch Engine ──────────────────────────────
            int active_sample = p.active_sample;
        
            static const int16_t *core1_sample_ptr = nullptr;
            static uint32_t core1_sample_len = 0;
            static int32_t core1_last_sample_idx = -999;
            
            if (active_sample != core1_last_sample_idx || core1_sample_ptr == nullptr) {
                core1_last_sample_idx = active_sample;
                core1_sample_ptr = get_active_sample_ptr(active_sample, core1_sample_len);
            }
            
            uint32_t active_len = core1_sample_len;
            if (active_len < 256) active_len = 256; // clamp to safe minimum
            const int16_t *sample_ptr = core1_sample_ptr;
            
            int32_t loop_len = p.amb_loop_len;
            int32_t start_pos = p.amb_start_pos;
            int32_t speed_mult = p.amb_speed_mult;
            int32_t scrub_pos = p.amb_scrub_pos;
            
            // Base increment in Q16: 12kHz/48kHz = 16384 for internal samples, or 65536 for Grains flashed samples (48kHz)
            uint32_t base_inc = use_flash_samples ? 65536 : 16384;
            int32_t speed_multiplier = 32768; // 1.0 in Q15
            
            if (scrub_pos >= 0) {
                amb_playhead_q16 = (uint64_t)scrub_pos << 16;
                speed_multiplier = 0;
            } else {
                speed_multiplier = speed_mult;
                if (!no_audio2) {
                    int16_t raw_ctrl = AudioIn2();
                    int32_t cv_scaler = pow2_q15(raw_ctrl * 3);
                    speed_multiplier = ((int64_t)speed_multiplier * cv_scaler) >> 15;
                }
            }
            
            if (scrub_pos < 0) {
                // Calculate playhead increment
                int32_t inc_q16 = ((int64_t)base_inc * speed_multiplier) >> 15;
                
                // Update playhead using 64-bit precision to prevent overflow/wrap bugs
                int64_t loop_start_q16 = (int64_t)start_pos << 16;
                int64_t loop_len_q16 = (int64_t)loop_len << 16;
                int64_t loop_end_q16 = loop_start_q16 + loop_len_q16;
                
                int64_t playhead = (int64_t)amb_playhead_q16 + inc_q16;
                
                // Snap playhead back to loop if it's completely out-of-bounds
                if (playhead < loop_start_q16 - (16LL << 16) || playhead > loop_end_q16 + (16LL << 16)) {
                    playhead = loop_start_q16;
                } else {
                    // Safe wrap-around using division/modulo for large wraps to prevent freezes
                    if (loop_len_q16 >= (256LL << 16)) {
                        if (playhead >= loop_end_q16) {
                            if (playhead < loop_end_q16 + loop_len_q16) {
                                playhead -= loop_len_q16;
                            } else {
                                playhead = loop_start_q16 + ((playhead - loop_start_q16) % loop_len_q16);
                            }
                        }
                        if (playhead < loop_start_q16) {
                            if (playhead >= loop_start_q16 - loop_len_q16) {
                                playhead += loop_len_q16;
                            } else {
                                playhead = loop_end_q16 - 1 - ((loop_end_q16 - 1 - playhead) % loop_len_q16);
                            }
                        }
                    } else {
                        playhead = loop_start_q16;
                    }
                }
                amb_playhead_q16 = (uint64_t)playhead;
            }
            
            // Linear interpolation with nullptr guard
            if (sample_ptr == nullptr) {
                L = R = 0;
            } else {
                uint32_t idx = amb_playhead_q16 >> 16;
                uint32_t nxt = idx + 1;
                if (nxt >= active_len) nxt = 0;
                uint16_t frac = (uint16_t)(amb_playhead_q16 & 0xFFFF);
                
                int16_t y0 = sample_ptr[idx];
                int16_t y1 = sample_ptr[nxt];
                int16_t interp = (int16_t)(y0 + (((int32_t)(y1 - y0) * frac) >> 16));
                
                // Apply loop boundary fades to prevent clicks/pops
                int32_t pos_idx = idx;
                int32_t pos_from_start = pos_idx - start_pos;
                int32_t pos_from_end = (start_pos + loop_len) - pos_idx;
                
                int32_t fade_len = use_flash_samples ? 960 : 240;
                if (fade_len > loop_len / 8) {
                    fade_len = loop_len / 8;
                }
                if (fade_len < 4) fade_len = 4;
                
                int32_t fade_factor = 32767;
                if (pos_from_start < fade_len || pos_from_end < fade_len) {
                    static int32_t cached_fade_len = -1;
                    static int32_t fade_len_recip = 0;
                    if (fade_len != cached_fade_len) {
                        cached_fade_len = fade_len;
                        fade_len_recip = (32767 << 15) / fade_len;
                    }
                    if (pos_from_start < fade_len) {
                        fade_factor = ((int64_t)pos_from_start * fade_len_recip) >> 15;
                    } else {
                        fade_factor = ((int64_t)pos_from_end * fade_len_recip) >> 15;
                    }
                }
                if (fade_factor < 0) fade_factor = 0;
                if (fade_factor > 32767) fade_factor = 32767;
                
                interp = ((int32_t)interp * fade_factor) >> 15;
                
                L = R = interp;
            }
        }
    } else {
        // Audio input mode:
        //   Audio 1 only (no_audio2 = true)  → mono in, duplicate to R
        //   Audio 1 + Audio 2               → true stereo
        L = (int16_t)(AudioIn1() << 4);
        R = no_audio2 ? L : (int16_t)(AudioIn2() << 4);
    }

    // ── Click-free crossfade on routing mode switch ──────────────────────────
    if (route_xfade_ctr > 0) {
        // Blend from previous-mode signal toward current-mode signal.
        // route_xfade_from_* holds last output of the OLD mode (sampled at switch moment).
        // We simply fade the old signal out (it was 0 if old mode produced 0 this tick,
        // but that is fine — the xfade rapidly conceals the discontinuity).
        int16_t t = (int16_t)(((ROUTE_XFADE_LEN - route_xfade_ctr) * 32767) / ROUTE_XFADE_LEN);
        L = lerp_q15((int16_t)route_xfade_from_L, L, t);
        R = lerp_q15((int16_t)route_xfade_from_R, R, t);
        route_xfade_ctr--;
    }

    // --- Apply Input DC Blockers -------------------------------------------
    // Capture pre-DC values for routing crossfade reference
    last_pre_dc_L = L;
    last_pre_dc_R = R;
    L = dc_inL.process(L);
    R = dc_inR.process(R);

    // ── STAGE 1: 90s Chorus ─────────────────────────────────────────────────
    chorus.process(L, L, R, R, chorus_mix, chorus_rate, chorus_depth_fb, cv1);

    // ── STAGE 2: Codec Demolisher ────────────────────────────────────────────
    const int32_t eff_codec_mix         = scale_grit(codec_mix, 32767, grittiness_macro);
    const int32_t eff_codec_downsample  = scale_grit(codec_downsample, 24000, grittiness_macro);
    const int32_t eff_codec_ringing_xor = scale_grit(codec_ringing_xor, 32767, grittiness_macro);
    const int32_t eff_global_noise_scale = scale_grit(global_noise_scale, 49152, grittiness_macro);

    codec.process(L, L, R, R,
                  eff_codec_mix, eff_codec_downsample, eff_codec_ringing_xor,
                  cv2, rand_seed, eff_global_noise_scale);

    // ── STAGE 3: Multi-Tap Delay ─────────────────────────────────────────────
    int32_t eff_delay_time = delay_time;
    if (grittiness_macro < 16384) {
        if (delay_time > 16384) {
            int32_t diff = delay_time - 16384;
            eff_delay_time = 16384 + ((diff * grittiness_macro) >> 14);
        }
    } else {
        int32_t diff = 32767 - delay_time;
        eff_delay_time = delay_time + ((diff * (grittiness_macro - 16384)) / 16383);
    }

    delay_fx.process(L, L, R, R,
                     delay_mix, eff_delay_time, delay_feedback, freeze, cv1, cv2, eff_global_noise_scale);

    // ── STAGE 4: Granular Glitcher ───────────────────────────────────────────
    int32_t eff_glitch_mix   = glitch_mix;
    int32_t eff_glitch_size  = glitch_size;
    int32_t eff_glitch_speed = glitch_speed;
    int32_t scrub_offset     = 0;

    if (is_freeze_page || freeze) {
        // Freeze active or Freeze Scrub page: knobs/settings control glitcher directly.
        eff_glitch_mix   = 32767;  // full wet so frozen buffer is always audible
        eff_glitch_size  = p.amb_loop_len;    // Y knob
        eff_glitch_speed = p.amb_speed_mult; // X knob
        scrub_offset     = p.amb_start_pos;  // Main knob
    } else {
        eff_glitch_mix   = scale_grit(glitch_mix, 32767, grittiness_macro);
        eff_glitch_speed = scale_grit(glitch_speed, 32767, grittiness_macro);
    }
    const int32_t eff_glitch_feedback = scale_grit(glitch_feedback, 32767, grittiness_macro);

    glitcher.process(L, L, R, R,
                     eff_glitch_mix, eff_glitch_size, eff_glitch_speed,
                     stutter,        // Pulse 1 = glitch trigger
                     is_freeze_page || freeze, // Freeze Scrub page or global freeze keeps glitcher active
                     cv1, cv2, rand_seed,
                     scrub_offset,
                     eff_glitch_feedback,
                     eff_global_noise_scale);

    // ── STAGE 5: Resonant Filter (always inline — Main knob = Cutoff) ────────
    int32_t eff_filter_res = filter_res;
    if (grittiness_macro < 16384) {
        if (filter_res >= 18000) {
            int32_t diff = filter_res - 18000;
            eff_filter_res = 18000 + ((diff * grittiness_macro) >> 14);
        }
    } else {
        int32_t diff = 32767 - filter_res;
        eff_filter_res = filter_res + ((diff * (grittiness_macro - 16384)) / 16383);
    }
    filter.process(L, L, R, R, filter_cutoff, eff_filter_res, filter_morph, cv1);

    // ── STAGE 6: Studio Reverb (offloaded to Core 0 via inter-core FIFO pipeline) ──
    uint32_t packed_filt = ((uint32_t)(uint16_t)L << 16) | (uint16_t)R;
    multicore_fifo_push_blocking(packed_filt);

    // Pop the Reverb result from Core 0 (pipelined from previous sample)
    uint32_t packed_rev = multicore_fifo_pop_blocking();
    L = (int16_t)(packed_rev >> 16);
    R = (int16_t)(packed_rev & 0xFFFF);

    // --- Output: soft-limit then scale back to 12-bit DAC -------------------
    AudioOut1(soft_limit_q15(L) >> 4);
    AudioOut2(soft_limit_q15(R) >> 4);
}

// Core 1 entry point
void core1_entry() { card.Run(); }

// ============================================================================
// Core 0: UI State
// ============================================================================
static int     currentPage          = 0;    // [0, 5]
static uint32_t pageFlashTimer      = 0;    // ms remaining in page-change flash

// Virtual parameter table — 8 pages × 3 knobs [Main, X, Y]
// Values are Q15 [0..32767]. Match atomic defaults above.
static int32_t vp[8][3] = {
    // Page 0 — Chorus:   Wet,   Rate,   Depth/FB
    {     0,  8000,  8192 },
    // Page 1 — Compression & Grit: Artifacts, Decimate, Digital Vibe
    {     0,     0,     0 },
    // Page 2 — Delay:    Wet,   Time,   Feedback
    {     0,  8000, 16384 },
    // Page 3 — Glitcher: Mix,   Size,   Speed Probability
    {     0,  8000, 16384 },
    // Page 4 — Filter:   Cutoff, Resonance, Morph
    { 16384,  4000,     0 },
    // Page 5 — Reverb:   Wet,   Decay,  Damping
    {     0, 16384,  8000 },
    // Page 6 — Sample Player: Start, Speed, Sample Select
    {     0, 16384,     0 },
    // Page 7 — Freeze Scrub: Scrub Pos, Glitch Speed, Loop Size
    {     0, 16384,  8000 },
};

// KnobLock instances — one per physical knob
static KnobLock lockMain, lockX, lockY, lockMacro;

// Smoothed knob values (IIR-filtered raw ADC readings)
static int32_t smMain = 0, smX = 0, smY = 0;

// Last known switch state (for debounce edge detection)
static bool freeze_latched = false;

static int32_t grittiness_macro = 32767;
static bool g_macro_active = false;

static void push_params_to_core1() {
    int total_samples = get_total_samples_count();
    int sample_idx = (vp[6][2] * total_samples) >> 15;
    if (sample_idx < 0) sample_idx = 0;
    if (sample_idx >= total_samples) sample_idx = total_samples - 1;

    for (int idx = 0; idx < 2; idx++) {
        volatile Core1Params &p = g_params[idx];

        p.chorus_mix       = vp[0][0];
        p.chorus_rate      = vp[0][1];
        p.chorus_depth_fb  = vp[0][2];

        p.codec_mix         = vp[1][0];
        p.codec_downsample  = vp[1][1];
        p.codec_ringing_xor = vp[1][2];

        p.delay_mix      = vp[2][0];
        p.delay_time     = vp[2][1];
        p.delay_feedback = vp[2][2];

        p.glitch_mix      = vp[3][0];
        p.glitch_size     = vp[3][1];
        p.glitch_speed    = vp[3][2];

        int32_t raw_fb = vp[2][2];
        if (raw_fb > 22937) {
            p.glitch_feedback = ((raw_fb - 22937) * 32767) / (32767 - 22937);
        } else {
            p.glitch_feedback = 0;
        }

        p.filter_cutoff = vp[4][0];
        p.filter_res    = vp[4][1];
        p.filter_morph  = vp[4][2];

        bool is_sample_page = (currentPage == 6);
        bool is_freeze_page = (currentPage == 7);
        
        uint32_t active_len = 0;
        get_active_sample_ptr(sample_idx, active_len);
        if (active_len < 256) active_len = 256;

        int32_t loop_len = active_len;
        int32_t start_pos = 0;
        int32_t speed_mult = 32768;
        int32_t scrub_pos = -1;

        if (is_sample_page) {
            int32_t speed_ctrl = vp[6][1];
            int32_t start_ctrl = vp[6][0];
            
            if (speed_ctrl < 1000) {
                scrub_pos = ((uint64_t)start_ctrl * active_len) >> 15;
                loop_len = active_len;
                start_pos = 0;
                speed_mult = 0;
            } else {
                if (speed_ctrl < 16000) {
                    int32_t knob_val = speed_ctrl - 1000;
                    loop_len = 512 + (((int64_t)(active_len - 512) * knob_val) / 15000);
                    start_pos = ((int64_t)start_ctrl * (active_len - loop_len)) >> 15;
                    speed_mult = 32768;
                } else if (speed_ctrl <= 17500) {
                    loop_len = active_len;
                    start_pos = 0;
                    speed_mult = 32768;
                } else {
                    int32_t knob_val = speed_ctrl - 17500;
                    speed_mult = 32768 + (((int64_t)knob_val * 98304) / 15267);
                    loop_len = active_len;
                    start_pos = 0;
                }
            }
            if (idx == 0) {
                amb_loop_len = loop_len;
                amb_start_pos = start_pos;
                amb_speed_mult = speed_mult;
            }
        } else if (is_freeze_page) {
            loop_len = vp[7][2];
            start_pos = vp[7][0];
            speed_mult = vp[7][1];
        } else {
            if (amb_loop_len < 0) {
                amb_loop_len = active_len;
                amb_start_pos = 0;
                amb_speed_mult = 32768;
            }
            loop_len = amb_loop_len;
            start_pos = amb_start_pos;
            speed_mult = amb_speed_mult;
        }

        p.active_sample = sample_idx;
        p.amb_loop_len = loop_len;
        p.amb_start_pos = start_pos;
        p.amb_speed_mult = speed_mult;
        p.amb_scrub_pos = scrub_pos;

        p.freeze = freeze_latched;
        p.stutter = false;
        p.cv1 = 0;
        p.cv2 = 0;

        p.no_audio1 = true;
        p.no_audio2 = true;

        p.is_sample_page = is_sample_page;
        p.is_freeze_page = is_freeze_page;
        p.flash_writing  = false;
        p.grittiness_macro = 16384;

        int32_t noise_scale = 16384;
        int32_t rev_damping = vp[5][2];
        if (rev_damping > 16384) {
            int32_t diff = rev_damping - 16384;
            noise_scale = 16384 + ((diff * 32768) / (32767 - 16384));
        }
        p.global_noise_scale = noise_scale;
    }

    c0_reverb_mix = apply_deadzone(vp[5][0]);
    c0_reverb_size = vp[5][1];
    c0_reverb_fb_glitch = vp[5][2];
}

static void get_bar_graph_leds(int32_t val, int16_t out_brightness[6]) {
    for (int i = 0; i < 6; i++) {
        int32_t lower = i * 5461;
        int32_t upper = (i + 1) * 5461;
        if (val <= lower) {
            out_brightness[i] = 0;
        } else if (val >= upper) {
            out_brightness[i] = 3000;
        } else {
            int32_t frac = val - lower;
            out_brightness[i] = (frac * 3000) / 5461;
        }
    }
}

void BendsCard::tick_ui_once() {
    static ComputerCard::Switch debounced_sw = ComputerCard::Switch::Middle;
    static ComputerCard::Switch prev_sw = ComputerCard::Switch::Middle;
    static uint32_t sw_stable_timer = 0;
    static ComputerCard::Switch last_debounced_sw = ComputerCard::Switch::Middle;
    static uint32_t active_sw_held_ms = 0;
    static bool     hold_action_triggered = false;
    static int32_t  last_sample_idx = -1;
    static uint32_t sample_show_timer = 0;
    static int      pageBeforeUp = 0;
    static bool     macro_adjusted_this_hold = false;

    // ── 1. Read & IIR-smooth knobs ────────────────────────────────────────
    // KnobVal returns 0..4095 (12-bit ADC). Scale to Q15 via <<3 → 0..32760.
    // IIR: alpha = 1/16 → τ ≈ 16 ms at 1 kHz polling rate.
    int32_t rawMain = KnobVal(Knob::Main) << 3;
    int32_t rawX    = KnobVal(Knob::X)    << 3;
    int32_t rawY    = KnobVal(Knob::Y)    << 3;

    IIR_SMOOTH(smMain, rawMain, 4);
    IIR_SMOOTH(smX,    rawX,    4);
    IIR_SMOOTH(smY,    rawY,    4);

    // ── 2. Jacks → debouncing & state determination ───────────────────────
    static int pulse1_connected_ctr = 0;
    static int pulse2_connected_ctr = 0;
    static int cv1_connected_ctr    = 0;
    static int cv2_connected_ctr    = 0;
    const  int JACK_DEBOUNCE        = 50; // ms

    if (Disconnected(Input::Pulse1)) { if (pulse1_connected_ctr > 0) pulse1_connected_ctr--; }
    else { if (pulse1_connected_ctr < JACK_DEBOUNCE) pulse1_connected_ctr++; }

    if (Disconnected(Input::Pulse2)) { if (pulse2_connected_ctr > 0) pulse2_connected_ctr--; }
    else { if (pulse2_connected_ctr < JACK_DEBOUNCE) pulse2_connected_ctr++; }

    if (Disconnected(Input::CV1)) { if (cv1_connected_ctr > 0) cv1_connected_ctr--; }
    else { if (cv1_connected_ctr < JACK_DEBOUNCE) cv1_connected_ctr++; }

    if (Disconnected(Input::CV2)) { if (cv2_connected_ctr > 0) cv2_connected_ctr--; }
    else { if (cv2_connected_ctr < JACK_DEBOUNCE) cv2_connected_ctr++; }

    bool pulse1_live = (pulse1_connected_ctr >= JACK_DEBOUNCE);
    bool pulse2_live = (pulse2_connected_ctr >= JACK_DEBOUNCE);
    bool cv1_live    = (cv1_connected_ctr    >= JACK_DEBOUNCE);
    bool cv2_live    = (cv2_connected_ctr    >= JACK_DEBOUNCE);

    bool pulse2_freeze = pulse2_live && PulseIn2();
    bool is_frozen = freeze_latched || pulse2_freeze;

    // Debounce Audio 1 and Audio 2 connection states
    static int audio1_connected_ctr = 0;
    static int audio2_connected_ctr = 0;
    const int DEBOUNCE_LIMIT = 50; // 50 ms filter
    
    if (Disconnected(Input::Audio1)) {
        if (audio1_connected_ctr > 0) audio1_connected_ctr--;
    } else {
        if (audio1_connected_ctr < DEBOUNCE_LIMIT) audio1_connected_ctr++;
    }
    
    if (Disconnected(Input::Audio2)) {
        if (audio2_connected_ctr > 0) audio2_connected_ctr--;
    } else {
        if (audio2_connected_ctr < DEBOUNCE_LIMIT) audio2_connected_ctr++;
    }
    
    static bool debounced_no_audio1 = true;
    if (debounced_no_audio1 && audio1_connected_ctr >= DEBOUNCE_LIMIT) {
        debounced_no_audio1 = false; // jack plugged in
    } else if (!debounced_no_audio1 && audio1_connected_ctr == 0) {
        debounced_no_audio1 = true;  // jack unplugged
    }
    
    static bool debounced_no_audio2 = true;
    if (debounced_no_audio2 && audio2_connected_ctr >= DEBOUNCE_LIMIT) {
        debounced_no_audio2 = false; // jack plugged in
    } else if (!debounced_no_audio2 && audio2_connected_ctr == 0) {
        debounced_no_audio2 = true;  // jack unplugged
    }

    // ── 3. Switch Debouncing & Settle Delay ──────────────────────────────
    static uint32_t startup_delay_ms = 0;
    if (startup_delay_ms < 500) {
        startup_delay_ms++;
        prev_sw = SwitchVal();
        debounced_sw = prev_sw;
        last_debounced_sw = prev_sw;
        sw_stable_timer = 0;
        active_sw_held_ms = 0;
        hold_action_triggered = false;
        freeze_latched = false;
    } else {
        ComputerCard::Switch raw_sw = SwitchVal();
        if (raw_sw != prev_sw) {
            prev_sw = raw_sw;
            sw_stable_timer = 0;
        } else {
            sw_stable_timer++;
            if (sw_stable_timer >= 25) { // Stable for 25 ms
                debounced_sw = raw_sw;
            }
        }
    }

    // ── 3b. Switch hold & flick state machine ────────────────────────────
    bool param_changed = false;

    bool sw_down_entered = (last_debounced_sw != ComputerCard::Switch::Down && debounced_sw == ComputerCard::Switch::Down);
    bool sw_down_exited  = (last_debounced_sw == ComputerCard::Switch::Down && debounced_sw != ComputerCard::Switch::Down);

    if (sw_down_entered) {
        lockMacro.engage(smMain);
        macro_adjusted_this_hold = false;
    }
    if (sw_down_exited) {
        if (active_sw_held_ms >= 350) {
            // Bake Codec parameters
            vp[1][0] = scale_grit(vp[1][0], 32767, grittiness_macro);
            vp[1][1] = scale_grit(vp[1][1], 24000, grittiness_macro);
            vp[1][2] = scale_grit(vp[1][2], 32767, grittiness_macro);

            // Bake Delay parameters
            {
                int32_t val = vp[2][1];
                if (grittiness_macro < 16384) {
                    if (val > 16384) {
                        int32_t diff = val - 16384;
                        vp[2][1] = 16384 + ((diff * grittiness_macro) >> 14);
                    }
                } else {
                    int32_t diff = 32767 - val;
                    vp[2][1] = val + ((diff * (grittiness_macro - 16384)) / 16383);
                }
            }
            {
                int32_t val = vp[2][2];
                if (grittiness_macro < 16384) {
                    if (val > 22937) {
                        int32_t diff = val - 22937;
                        vp[2][2] = 22937 + ((diff * grittiness_macro) >> 14);
                    }
                } else {
                    int32_t diff = 32767 - val;
                    vp[2][2] = val + ((diff * (grittiness_macro - 16384)) / 16383);
                }
            }

            // Bake Glitcher parameters
            vp[3][0] = scale_grit(vp[3][0], 32767, grittiness_macro);
            vp[3][2] = scale_grit(vp[3][2], 32767, grittiness_macro);

            // Bake Filter parameters
            {
                int32_t val = vp[4][1];
                if (grittiness_macro < 16384) {
                    if (val >= 18000) {
                        int32_t diff = val - 18000;
                        vp[4][1] = 18000 + ((diff * grittiness_macro) >> 14);
                    }
                } else {
                    int32_t diff = 32767 - val;
                    vp[4][1] = val + ((diff * (grittiness_macro - 16384)) / 16383);
                }
            }

            // Bake Reverb parameters
            {
                int32_t val = vp[5][2];
                if (grittiness_macro < 16384) {
                    if (val > 16384) {
                        int32_t diff = val - 16384;
                        vp[5][2] = 16384 + ((diff * grittiness_macro) >> 14);
                    }
                } else {
                    int32_t diff = 32767 - val;
                    vp[5][2] = val + ((diff * (grittiness_macro - 16384)) / 16383);
                }
            }

            if (!macro_adjusted_this_hold) {
                if (currentPage >= 6) {
                    currentPage = 0;
                } else {
                    currentPage = (currentPage + 5) % 6;
                }
                lockMain.engage(smMain);
                lockX.engage(smX);
                lockY.engage(smY);
                pageFlashTimer = 400;
            }

            grittiness_macro = 16384;
            g_macro_active = false;
            lockMain.engage(smMain);
            param_changed = true;
        } else {
            lockMain.engage(smMain);
        }
    }

    if (debounced_sw == last_debounced_sw) {
        if (debounced_sw != ComputerCard::Switch::Middle) {
            active_sw_held_ms++;
            if (active_sw_held_ms >= 350) {
                if (!hold_action_triggered) {
                    hold_action_triggered = true;

                    // Trigger hold action
                    if (debounced_sw == ComputerCard::Switch::Down) {
                        // Hold DOWN: macro active, page change deferred until release if unadjusted
                    } else if (debounced_sw == ComputerCard::Switch::Up) {
                        // Hold UP
                        if (freeze_latched) {
                            if (currentPage != 7) {
                                if (currentPage < 6) {
                                    pageBeforeUp = currentPage;
                                }
                                currentPage = 7;
                                lockMain.engage(smMain);
                                lockX.engage(smX);
                                lockY.engage(smY);
                                param_changed = true;
                            }
                        } else {
                            if (debounced_no_audio1) {
                                if (currentPage != 6) {
                                    if (currentPage < 6) {
                                        pageBeforeUp = currentPage;
                                    }
                                    currentPage = 6;
                                    lockMain.engage(smMain);
                                    lockX.engage(smX);
                                    lockY.engage(smY);
                                    param_changed = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    } else {
        // Transition detected
        if (last_debounced_sw != ComputerCard::Switch::Middle && debounced_sw == ComputerCard::Switch::Middle) {
            // Switch was released
            if (active_sw_held_ms < 350) {
                // Trigger flick action
                if (last_debounced_sw == ComputerCard::Switch::Down) {
                    // Flick DOWN
                    if (!macro_adjusted_this_hold) {
                        if (currentPage >= 6) {
                            currentPage = 0;
                        } else {
                            currentPage = (currentPage + 1) % 6;
                        }
                        lockMain.engage(smMain);
                        lockX.engage(smX);
                        lockY.engage(smY);
                        pageFlashTimer = 400;
                        param_changed = true;
                    }
                } else if (last_debounced_sw == ComputerCard::Switch::Up) {
                    // Flick UP
                    freeze_latched = !freeze_latched;
                    is_frozen = freeze_latched || pulse2_freeze;
                    lockMain.engage(smMain);
                    lockX.engage(smX);
                    lockY.engage(smY);
                    param_changed = true;
                }
            } else {
                // Hold release action
                if (last_debounced_sw == ComputerCard::Switch::Up) {
                    // Releasing Switch UP: return to the page we were on before
                    if (currentPage >= 6) {
                        currentPage = pageBeforeUp;
                        lockMain.engage(smMain);
                        lockX.engage(smX);
                        lockY.engage(smY);
                        param_changed = true;
                    }
                }
            }
        }
        // Reset state
        last_debounced_sw = debounced_sw;
        active_sw_held_ms = 0;
        hold_action_triggered = false;
    }

    // ── 3c. Sample Show Timer ───────────────────────────────────────────
    int total_samples = get_total_samples_count();
    int sample_idx = (vp[6][2] * total_samples) >> 15;
    if (sample_idx < 0) sample_idx = 0;
    if (sample_idx >= total_samples) sample_idx = total_samples - 1;

    if (currentPage == 6 && sample_idx != last_sample_idx) {
        last_sample_idx = sample_idx;
        sample_show_timer = 1500;
    }

    if (sample_show_timer > 0) {
        sample_show_timer--;
    }

    // ── 4. KnobLock gating → update virtual parameters ───────────────────
    if (debounced_sw == ComputerCard::Switch::Down) {
        if (active_sw_held_ms >= 350) {
            if (lockMacro.update(smMain)) {
                if (grittiness_macro != smMain) {
                    grittiness_macro = smMain;
                    macro_adjusted_this_hold = true;
                    param_changed = true;
                }
            }
        }
    } else {
        if (lockMain.update(smMain)) {
            if (vp[currentPage][0] != smMain) {
                vp[currentPage][0] = smMain;
                param_changed = true;
            }
        }
    }
    if (lockX.update(smX)) {
        if (vp[currentPage][1] != smX) {
            vp[currentPage][1] = smX;
            param_changed = true;
        }
    }
    if (lockY.update(smY)) {
        if (vp[currentPage][2] != smY) {
            vp[currentPage][2] = smY;
            param_changed = true;
        }
    }

    // ── 5. Push virtual params to Core 1 double buffer ───────────────────
    {
        uint32_t next_idx = 1 - g_params_idx.load(std::memory_order_relaxed);
        volatile Core1Params &p = g_params[next_idx];

        p.chorus_mix       = vp[0][0];
        p.chorus_rate      = vp[0][1];
        p.chorus_depth_fb  = vp[0][2];

        p.codec_mix         = vp[1][0];
        p.codec_downsample  = vp[1][1];
        p.codec_ringing_xor = vp[1][2];

        p.delay_mix      = vp[2][0];
        p.delay_time     = vp[2][1];
        p.delay_feedback = vp[2][2];

        p.glitch_mix      = vp[3][0];
        p.glitch_size     = vp[3][1];
        p.glitch_speed    = vp[3][2];

        // Calculate Glitcher Feedback Loop
        // Controlled by physical Delay Feedback knob (vp[2][2]) when it exceeds 70%
        int32_t raw_fb = vp[2][2];
        if (raw_fb > 22937) {
            p.glitch_feedback = ((raw_fb - 22937) * 32767) / (32767 - 22937);
        } else {
            p.glitch_feedback = 0;
        }

        p.filter_cutoff = vp[4][0];
        p.filter_res    = vp[4][1];
        p.filter_morph  = vp[4][2];

        // Sample Player math
        bool is_sample_page = (currentPage == 6);
        bool is_freeze_page = (currentPage == 7);
        
        uint32_t active_len = 0;
        get_active_sample_ptr(sample_idx, active_len);
        if (active_len < 256) active_len = 256;

        int32_t loop_len = active_len;
        int32_t start_pos = 0;
        int32_t speed_mult = 32768;
        int32_t scrub_pos = -1;

        if (is_sample_page) {
            int32_t speed_ctrl = vp[6][1];
            int32_t start_ctrl = vp[6][0];
            
            if (speed_ctrl < 1000) {
                scrub_pos = ((uint64_t)start_ctrl * active_len) >> 15;
                loop_len = active_len;
                start_pos = 0;
                speed_mult = 0;
            } else {
                if (speed_ctrl < 16000) {
                    int32_t knob_val = speed_ctrl - 1000;
                    loop_len = 512 + (((int64_t)(active_len - 512) * knob_val) / 15000);
                    start_pos = ((int64_t)start_ctrl * (active_len - loop_len)) >> 15;
                    speed_mult = 32768;
                } else if (speed_ctrl <= 17500) {
                    loop_len = active_len;
                    start_pos = 0;
                    speed_mult = 32768;
                } else {
                    int32_t knob_val = speed_ctrl - 17500;
                    speed_mult = 32768 + (((int64_t)knob_val * 98304) / 15267);
                    loop_len = active_len;
                    start_pos = 0;
                }
            }
            amb_loop_len = loop_len;
            amb_start_pos = start_pos;
            amb_speed_mult = speed_mult;
        } else if (is_freeze_page) {
            loop_len = vp[7][2];
            start_pos = vp[7][0];
            speed_mult = vp[7][1];
        } else {
            if (amb_loop_len < 0) {
                amb_loop_len = active_len;
                amb_start_pos = 0;
                amb_speed_mult = 32768;
            }
            loop_len = amb_loop_len;
            start_pos = amb_start_pos;
            speed_mult = amb_speed_mult;
        }

        p.active_sample = sample_idx;
        p.amb_loop_len = loop_len;
        p.amb_start_pos = start_pos;
        p.amb_speed_mult = speed_mult;
        p.amb_scrub_pos = scrub_pos;

        p.freeze = is_frozen;
        p.stutter = pulse1_live && PulseIn1();
        p.cv1 = cv1_live ? CVIn1() : 0;
        p.cv2 = cv2_live ? CVIn2() : 0;

        p.no_audio1 = debounced_no_audio1;
        p.no_audio2 = debounced_no_audio2;

        p.is_sample_page = is_sample_page;
        p.is_freeze_page = is_freeze_page;
        p.flash_writing  = atom_flash_writing.load(std::memory_order_relaxed);
        g_macro_active = (debounced_sw == ComputerCard::Switch::Down && active_sw_held_ms >= 350);
        p.grittiness_macro = g_macro_active ? grittiness_macro : 16384;

        // Calculate Global Noise Scale (tied to Reverb Y knob, vp[5][2])
        // Upper 50% scales noise from 1.0x to 3.0x (16384 to 49152 in Q14)
        int32_t noise_scale = 16384;
        int32_t rev_damping = vp[5][2];
        if (rev_damping > 16384) {
            int32_t diff = rev_damping - 16384;
            noise_scale = 16384 + ((diff * 32768) / (32767 - 16384));
        }
        p.global_noise_scale = noise_scale;

        g_params_idx.store(next_idx, std::memory_order_release);
    }

    // ── Update Core 0 Reverb parameters ──────────────────────────────────
    {
        c0_reverb_mix = apply_deadzone(vp[5][0]);
        c0_reverb_size = vp[5][1];
        c0_reverb_fb_glitch = vp[5][2];
    }

    // ── 5b. Non-blocking Flash Auto-Save ─────────────────────────────────
    static bool save_dirty = false;
    static uint32_t save_cooldown_timer = 0;

    if (param_changed) {
        save_dirty = true;
        save_cooldown_timer = 5000; // 5 seconds
    }

    if (save_dirty) {
        if (save_cooldown_timer > 0) {
            save_cooldown_timer--;
        } else {
            save_dirty = false;
            
            // Pack settings
            union FlashSaveBuffer {
                struct {
                    uint32_t magic;
                    int32_t  vp[8][3];
                    bool     freeze_latched;
                    int32_t  grittiness_macro;
                    uint32_t checksum;
                } data;
                uint8_t bytes[256];
            } buf;
            
            memset(buf.bytes, 0, 256);
            buf.data.magic = SAVE_MAGIC;
            memcpy(buf.data.vp, vp, sizeof(vp));
            buf.data.freeze_latched = freeze_latched;
            buf.data.grittiness_macro = grittiness_macro;
            
            // Calculate simple checksum
            uint32_t sum = 0;
            for (size_t i = 0; i < sizeof(vp)/4; i++) {
                sum += ((uint32_t*)vp)[i];
            }
            sum += (uint32_t)freeze_latched;
            sum += (uint32_t)grittiness_macro;
            buf.data.checksum = sum;
            
            // Write to flash safely without halting Core 1 audio
            atom_flash_writing.store(true, std::memory_order_release);
            for (volatile int i = 0; i < 1000; i++);
            
            uint32_t ints = save_and_disable_interrupts();
            flash_range_erase(SAVE_FLASH_OFFSET, FLASH_SECTOR_SIZE);
            flash_range_program(SAVE_FLASH_OFFSET, buf.bytes, 256);
            restore_interrupts(ints);
            
            atom_flash_writing.store(false, std::memory_order_release);
        }
    }

    // ── 6. LED visualisation ──────────────────────────────────────────────
    bool is_sample_page = (currentPage == 6);
    bool is_freeze_page = (currentPage == 7);

    if (debounced_sw == ComputerCard::Switch::Down && active_sw_held_ms >= 350) {
        int16_t bar_leds[6];
        get_bar_graph_leds(grittiness_macro, bar_leds);
        for (int i = 0; i < 6; i++) {
            LedBrightness(i, bar_leds[i]);
        }
    } else if (is_sample_page) {
        // Sample Player page: show the selected sample slot temporarily in 1-based binary
        if (sample_show_timer > 0) {
            int val = sample_idx + 1;
            for (int i = 0; i < 6; i++) {
                int bit = (val >> i) & 1;
                LedBrightness(i, bit ? 4095 : 0);
            }
        } else {
            for (int i = 0; i < 6; i++) {
                int brightness = 0;
                if (i == 5 && is_frozen) {
                    brightness = 4095;
                }
                LedBrightness(i, brightness);
            }
        }
    } else if (is_freeze_page) {
        // Freeze Scrub page: all 6 LEDs pulse slowly — distinct "frozen" indicator
        static uint32_t freeze_led_phase = 0;
        freeze_led_phase += 80; // ~0.7 Hz triangle wave at 1 kHz tick rate
        uint32_t ph = (freeze_led_phase >> 16) & 0xFFFF;
        uint16_t bright = (ph < 0x8000) ? (ph >> 3) : ((0xFFFF - ph) >> 3);
        bright = (bright > 4095) ? 4095 : bright;
        for (int i = 0; i < 6; i++) {
            LedBrightness(i, bright);
        }
    } else if (pageFlashTimer > 0) {
        pageFlashTimer--;
        for (int i = 0; i < 6; i++) {
            int brightness = 0;
            if (i == currentPage) {
                brightness = 4095;
            }
            if (i == 5 && is_frozen) {
                brightness = 4095;
            }
            LedBrightness(i, brightness);
        }
    } else {
        // Normal mode: active page LED comfortable steady glow; others dark
        // Bottom right LED (LED 5) turns on full when frozen
        for (int i = 0; i < 6; i++) {
            int brightness = 0;
            if (i == currentPage) {
                brightness = 1500;
            }
            if (i == 5 && is_frozen) {
                brightness = 4095;
            }
            LedBrightness(i, brightness);
        }
    }
}

void BendsCard::run_core0_ui_loop() {
    // Pre-seed inter-core FIFO so Core 1 doesn't block on the first audio tick
    multicore_fifo_push_blocking(0);

    uint32_t sample_counter = 0;

    while (1) {
        // Pop filtered sample from Core 1
        uint32_t packed_filt = multicore_fifo_pop_blocking();
        int16_t filtL = (int16_t)(packed_filt >> 16);
        int16_t filtR = (int16_t)(packed_filt & 0xFFFF);

        // Process Reverb on Core 0
        int16_t L = filtL;
        int16_t R = filtR;
        int32_t eff_fb_glitch = c0_reverb_fb_glitch;
        int32_t active_macro = g_macro_active ? grittiness_macro : 16384;
        if (active_macro < 16384) {
            if (c0_reverb_fb_glitch > 16384) {
                int32_t diff = c0_reverb_fb_glitch - 16384;
                eff_fb_glitch = 16384 + ((diff * active_macro) >> 14);
            }
        } else {
            int32_t diff = 32767 - c0_reverb_fb_glitch;
            eff_fb_glitch = c0_reverb_fb_glitch + ((diff * (active_macro - 16384)) / 16383);
        }
        reverb.process(L, R, c0_reverb_mix, c0_reverb_size, eff_fb_glitch);

        // Push Reverb output back to Core 1
        uint32_t packed_rev = ((uint32_t)(uint16_t)L << 16) | (uint16_t)R;
        multicore_fifo_push_blocking(packed_rev);

        // Every 48 samples (~1 ms), execute one tick of the UI loop
        sample_counter++;
        if (sample_counter >= 48) {
            sample_counter = 0;
            tick_ui_once();
        }
    }
}

// ============================================================================
// main() — CPU setup, block init, core launch
// ============================================================================
int main() {
    // 1. Overclock to 240 MHz with boosted supply voltage — same as grains/modes.
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    sleep_ms(10);
    set_sys_clock_khz(240000, true);

    // 2. Init stdio and the fixed-point sine table.
    stdio_init_all();
    init_sine_table();

    // 3. Init sample headers and all DSP blocks — zeroes buffers.
    check_flash_samples();
    chorus.init();
    codec.init();
    delay_fx.init();
    glitcher.init();
    filter.init();
    reverb.init();

    // 4. Enable normalisation probe and let it settle before reading jack state.
    //    The probe needs a few cycles to stabilise; 50 ms is more than enough.
    card.EnableNormalisationProbe();
    for (int i = 0; i < 10; i++) {
        (void)card.JackDisconnected(ComputerCard::Input::Audio1);
        (void)card.JackDisconnected(ComputerCard::Input::Audio2);
        sleep_ms(5);
    }

    // 5. Initialize Core 1 double-buffered parameters
    memset((void*)g_params, 0, sizeof(g_params));
    g_params[0].amb_speed_mult = 32768;
    g_params[1].amb_speed_mult = 32768;
    g_params[0].global_noise_scale = 16384;
    g_params[1].global_noise_scale = 16384;
    g_params[0].no_audio1 = true;
    g_params[1].no_audio1 = true;
    g_params[0].no_audio2 = true;
    g_params[1].no_audio2 = true;
    g_params[0].grittiness_macro = 32767;
    g_params[1].grittiness_macro = 32767;

    // 6. Load saved settings from flash if valid
    {
        union FlashSaveBuffer {
            struct {
                uint32_t magic;
                int32_t  vp[8][3];
                bool     freeze_latched;
                int32_t  grittiness_macro;
                uint32_t checksum;
            } data;
            uint8_t bytes[256];
        } const *saved = (const FlashSaveBuffer *)(0x10000000 + SAVE_FLASH_OFFSET);

        if (saved->data.magic == SAVE_MAGIC) {
            uint32_t sum = 0;
            for (size_t i = 0; i < sizeof(saved->data.vp)/4; i++) {
                sum += ((const uint32_t*)saved->data.vp)[i];
            }
            sum += (uint32_t)saved->data.freeze_latched;
            sum += (uint32_t)saved->data.grittiness_macro;
            if (sum == saved->data.checksum) {
                memcpy(vp, saved->data.vp, sizeof(vp));
                freeze_latched = saved->data.freeze_latched;
                grittiness_macro = saved->data.grittiness_macro;
            }
        }
    }

    // 7. Push loaded/default settings to Core 1, launch Core 1, and wait
    //    for ADC readings to settle before engaging KnobLock.
    push_params_to_core1();

    // 8. NOW start Core 1 — the ISR fires into a fully-initialised parameter set.
    multicore_launch_core1(core1_entry);

    // Sleep for 100 ms to let Core 1 start and fill the knobs array with physical ADC values
    sleep_ms(100);

    smMain = card.ReadKnob(ComputerCard::Knob::Main) << 3;
    smX    = card.ReadKnob(ComputerCard::Knob::X)    << 3;
    smY    = card.ReadKnob(ComputerCard::Knob::Y)    << 3;
    lockMain.engage(smMain);
    lockX.engage(smX);
    lockY.engage(smY);
    lockMacro.engage(smMain);

    // 9. Enter Core 0 UI loop — never returns.
    card.run_core0_ui_loop();
}
