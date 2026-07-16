#define COMPUTERCARD_SAMPLE_RATE_DIV 2 // Run at 24kHz — doubles CPU budget per sample

#include "ComputerCard.h"
#include "fixed_math.h"
#include "dsp_blocks.h"

#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/vreg.h"
#include <atomic>
#include <cstring>
#include "hardware/flash.h"
#include "hardware/sync.h"

// DC Blockers and dynamic softener filters for dual inputs
DCBlocker dc_inL;
DCBlocker dc_inR;
int32_t lp_inL = 0;
int32_t lp_inR = 0;


inline int32_t apply_deadzone(int32_t val) {
    if (val < 500) return 0;
    if (val > 32200) return 32767;
    return val;
}

// ============================================================================
// Global DSP Instances & State Variables
// ============================================================================
int16_t sine_table[SINE_TABLE_SIZE + 1];

uint32_t rand_seed = 987654321u;
int16_t mulaw_decode_table[256];

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
    int32_t chorus_depth_fb; // Keep for legacy, but we will use the decoded ones
    int32_t chorus_depth;
    int32_t chorus_feedback;
    int32_t chorus_xor_mask;

    int32_t codec_mix;
    int32_t codec_downsample;
    int32_t codec_ringing_xor;
    int32_t codec_mp3_ring;
    int32_t codec_fuzz;
    int32_t codec_decimate;
    int32_t codec_pop_prob;
    int32_t codec_click_depth;
    int32_t codec_bad_conn;
    int32_t codec_scramble;
    int32_t codec_sputter_prob;
    int32_t codec_tape_sat;
    int32_t codec_tape_hiss;
    int32_t codec_active_loss;

    int32_t delay_mix;
    int32_t delay_time;
    int32_t delay_feedback;

    int32_t glitch_mix;
    int32_t glitch_size;
    int32_t glitch_speed;
    int32_t glitch_speed_mapped;
    int32_t glitch_feedback;
    int32_t global_noise_scale;

    int32_t filter_cutoff;
    int32_t filter_res;
    int32_t filter_morph;

    int32_t reverb_mix;
    int32_t reverb_size;
    int32_t reverb_decay;
    int32_t reverb_damp;
    int32_t reverb_lofi_level;
    int32_t reverb_sparkle_level;
    int32_t reverb_circuit_bent_level;
    int32_t reverb_lofi_shift;
    int32_t reverb_lofi_frac;

    int32_t cv1;
    int32_t cv2;

    int32_t grittiness_macro;

    bool freeze;
    bool stutter;
    bool no_audio1;
    bool no_audio2;
    bool is_freeze_page;
    bool flash_writing;
    bool pulse1_live;
    bool pulse2_live;
    bool cv1_live;
    bool cv2_live;
};

volatile Core1Params g_params[2];
std::atomic<uint32_t> g_params_idx{0};

// Grittiness macro state (written on Core 0, read in push_params_to_core1)
static int32_t grittiness_macro = 32767;
static bool g_macro_active = false;

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

inline int32_t scale_grit(int32_t val, int32_t max_val, int32_t macro, int32_t gain_q15 = 32768) {
    if (macro < 16384) {
        return (val * macro) >> 14;
    } else {
        int32_t target_max = val;
        if (gain_q15 >= 32768) {
            target_max = max_val;
        } else {
            target_max = val + (((max_val - val) * gain_q15) >> 15);
        }
        int32_t diff = target_max - val;
        int32_t scale_q15 = (macro - 16384) * 2;
        return val + ((diff * scale_q15) >> 15);
    }
}

// ============================================================================
// Core 1: ProcessSample() — 24 kHz audio interrupt
// ============================================================================
// ALL DSP including reverb runs here on Core 1.
// Core 0 is a pure UI loop — no inter-core FIFO, no mutual blocking.
void __not_in_flash_func(BendsCard::ProcessSample)() {

    // --- Load all parameters in a single burst from the double buffer ---
    const uint32_t params_idx     = g_params_idx.load(std::memory_order_relaxed);
    const volatile Core1Params &p = g_params[params_idx];

    const int32_t eff_chorus_mix      = p.chorus_mix;
    const int32_t chorus_rate         = p.chorus_rate;
    const int32_t eff_chorus_depth    = p.chorus_depth;
    const int32_t eff_chorus_feedback = p.chorus_feedback;
    const int32_t eff_chorus_xor_mask = p.chorus_xor_mask;

    const int32_t eff_codec_mix         = p.codec_mix;
    const int32_t eff_codec_mp3_ring    = p.codec_mp3_ring;
    const int32_t eff_codec_fuzz        = p.codec_fuzz;
    const int32_t eff_codec_decimate    = p.codec_decimate;
    const int32_t eff_codec_pop_prob    = p.codec_pop_prob;
    const int32_t eff_codec_click_depth = p.codec_click_depth;
    const int32_t eff_codec_bad_conn    = p.codec_bad_conn;
    const int32_t eff_codec_scramble    = p.codec_scramble;
    const int32_t eff_codec_sputter_prob = p.codec_sputter_prob;
    const int32_t eff_codec_tape_sat    = p.codec_tape_sat;
    const int32_t eff_codec_tape_hiss   = p.codec_tape_hiss;
    const int32_t eff_codec_active_loss = p.codec_active_loss;

    const int32_t eff_delay_mix      = p.delay_mix;
    const int32_t eff_delay_time     = p.delay_time;
    const int32_t eff_delay_feedback = p.delay_feedback;

    const int32_t eff_glitch_mix      = p.glitch_mix;
    const int32_t eff_glitch_size     = p.glitch_size;
    const int32_t eff_glitch_speed    = p.glitch_speed;
    const int32_t eff_glitch_speed_mapped = p.glitch_speed_mapped;
    const int32_t eff_glitch_feedback = p.glitch_feedback;
    const int32_t eff_global_noise_scale = p.global_noise_scale;

    const int32_t eff_filter_cutoff = p.filter_cutoff;
    const int32_t eff_filter_res    = p.filter_res;
    const int32_t eff_filter_morph  = p.filter_morph;

    const int32_t eff_reverb_mix      = p.reverb_mix;
    const int32_t reverb_size_scale   = p.reverb_size;
    const int32_t eff_reverb_decay    = p.reverb_decay;
    const int32_t eff_reverb_damp     = p.reverb_damp;
    const int32_t eff_reverb_lofi_level = p.reverb_lofi_level;
    const int32_t eff_reverb_sparkle_level = p.reverb_sparkle_level;
    const int32_t eff_reverb_circuit_bent_level = p.reverb_circuit_bent_level;
    const int32_t eff_reverb_lofi_shift = p.reverb_lofi_shift;
    const int32_t eff_reverb_lofi_frac  = p.reverb_lofi_frac;

    const bool    freeze  = p.freeze;
    const bool    stutter = p.stutter;
    const int32_t cv1    = p.cv1;
    const int32_t cv2    = p.cv2;
    const bool    pulse1_live = p.pulse1_live;
    const bool    pulse2_live = p.pulse2_live;
    const bool    cv1_live    = p.cv1_live;

    const bool no_audio2 = p.no_audio2;
    const bool is_freeze_page = p.is_freeze_page;

    // --- Sample-Accurate Edge Detection for Eurorack Pulses ---
    bool p1_val = PulseIn1();
    bool p2_val = PulseIn2();
    static bool last_p1_val = false;
    static bool last_p2_val = false;
    bool p1_rising = p1_val && !last_p1_val;
    bool p2_rising = p2_val && !last_p2_val;
    last_p1_val = p1_val;
    last_p2_val = p2_val;

    // Track clock period on Pulse 1 (Stutter Clock)
    static uint32_t clk_timer = 0;
    static uint32_t clk_period_samples = 0;
    clk_timer++;
    if (pulse1_live && p1_rising) {
        if (clk_timer > 240) { // filter noise (>10ms)
            clk_period_samples = clk_timer;
        }
        clk_timer = 0;
    }

    // --- Read Audio Inputs & Attenuate for Headroom ---
    int16_t L = (int16_t)((AudioIn1() << 4) >> 1);
    int16_t R = no_audio2 ? L : (int16_t)((AudioIn2() << 4) >> 1);

    // --- Apply Input DC Blockers ---
    L = dc_inL.process(L);
    R = dc_inR.process(R);

    // --- Dynamic Transient Softener for Hot/Clipping Inputs ---
    {
        int32_t absL = L < 0 ? -L : L;
        int32_t coefL = 32767;
        if (absL >= 12288) {
            int32_t overshoot = absL - 12288;
            coefL = 32767 - (overshoot * 4);
        }
        lp_inL += (((int32_t)L - lp_inL) * coefL) >> 15;
        L = (int16_t)lp_inL;

        int32_t absR = R < 0 ? -R : R;
        int32_t coefR = 32767;
        if (absR >= 12288) {
            int32_t overshoot = absR - 12288;
            coefR = 32767 - (overshoot * 4);
        }
        lp_inR += (((int32_t)R - lp_inR) * coefR) >> 15;
        R = (int16_t)lp_inR;
    }

    // ── STAGE 1: Chorus ──────────────────────────────────────────────────────
    chorus.process(L, L, R, R, eff_chorus_mix, chorus_rate, eff_chorus_depth, eff_chorus_feedback, eff_chorus_xor_mask, cv1_live ? cv1 : 0);

    // ── STAGE 2: Codec Demolisher ────────────────────────────────────────────
    codec.process(L, L, R, R,
                  eff_codec_mix,
                  eff_codec_mp3_ring, eff_codec_fuzz, eff_codec_decimate,
                  eff_codec_pop_prob, eff_codec_click_depth, eff_codec_bad_conn, eff_codec_scramble,
                  eff_codec_sputter_prob, eff_codec_tape_sat, eff_codec_tape_hiss, eff_codec_active_loss,
                  rand_seed);

    // ── STAGE 3: Multi-Tap Delay ─────────────────────────────────────────────
    delay_fx.process(L, L, R, R,
                     eff_delay_mix, eff_delay_time, eff_delay_feedback, freeze, 0, cv2, eff_global_noise_scale,
                     pulse1_live, clk_period_samples);

    // ── STAGE 4: Granular Glitcher ───────────────────────────────────────────
    int32_t scrub_offset = 0;
    if (is_freeze_page) {
        scrub_offset = eff_glitch_mix;
    }

    glitcher.process(L, L, R, R,
                     is_freeze_page ? 32767 : eff_glitch_mix, eff_glitch_size, eff_glitch_speed, eff_glitch_speed_mapped,
                     stutter, is_freeze_page || freeze,
                     cv1, cv2, rand_seed,
                     scrub_offset, eff_glitch_feedback, eff_global_noise_scale,
                     pulse1_live, p1_rising, p1_val,
                     pulse2_live, p2_rising, p2_val,
                     clk_period_samples, clk_timer);

    // ── STAGE 5: Resonant Filter ─────────────────────────────────────────────
    filter.process(L, L, R, R, eff_filter_cutoff, eff_filter_res, eff_filter_morph, 0);

    // ── STAGE 6: Reverb ──────────────────────────────────────────────────────
    reverb.process(L, R, eff_reverb_mix, reverb_size_scale,
                   eff_reverb_decay, eff_reverb_damp, eff_reverb_lofi_level,
                   eff_reverb_sparkle_level, eff_reverb_circuit_bent_level,
                   eff_reverb_lofi_shift, eff_reverb_lofi_frac);

    // --- CV Outputs (Envelope Follower and Arpeggiator CV / LFO) ---
    // CV Out 1: envelope follower of the audio signal (fast attack, slow decay)
    int32_t env_in = (L < 0 ? -L : L) + (R < 0 ? -R : R);
    static int32_t env_followed = 0;
    if (env_in > env_followed) {
        env_followed += ((env_in - env_followed) * 8192) >> 15;
    } else {
        env_followed += ((env_in - env_followed) * 256) >> 15;
    }
    int16_t cv_out1_val = -2048 + ((env_followed * 4095) >> 15);
    CVOut1(cv_out1_val);

    // CV Out 2: subtle stepped CV in Zone 3 (matching tape-drift vibe), or slow LFO in other zones
    if (glitcher.active && (eff_glitch_speed >= 19661 && eff_glitch_speed < 26214)) {
        static const int16_t steps[8] = {0, 120, -120, 0, -80, 160, -160, 80};
        int16_t cv_val = steps[glitcher.arpeggio_step & 7];
        CVOut2(cv_val);
    } else {
        static uint16_t lfo_phase = 0;
        lfo_phase += 16;
        int32_t lfo_tri = (lfo_phase < 32768) ? ((lfo_phase << 1) - 32768) : (32767 - ((lfo_phase - 32768) << 1));
        int16_t cv_out2_val = (lfo_tri * 2047) >> 15;
        CVOut2(cv_out2_val);
    }

    // Rhythmic trigger outputs & Glitchy Square-Wave Audio Output
    static int16_t p1_trig_timer = 0;
    if (glitcher.trig_out1) {
        glitcher.trig_out1 = false;
        p1_trig_timer = 48; // 2 ms pulse
    }
    
    if (p1_trig_timer > 0) {
        p1_trig_timer--;
        PulseOut1(true);
    } else {
        PulseOut1(false);
    }

    // Pulse 2 outputs raw composite glitchy noise when active for Eurorack mixing/filtering
    static uint32_t square_phase = 0;
    if (glitcher.active) {
        uint8_t g711 = glitcher.current_g711_sample;
        
        // Layer 1: Audio-rate 1-bit comparator fuzz (sign bit) to track audio waveforms
        bool audio_fuzz = (g711 & 0x80) != 0;
        
        // Layer 2: Sub-harmonic frequency tracking square wave (tracks active loop speed)
        int32_t abs_speed = glitcher.current_speed_q16 < 0 ? -glitcher.current_speed_q16 : glitcher.current_speed_q16;
        square_phase += (abs_speed * 11) / 1200; // maps 1.0x speed to 220Hz C3 pitch
        bool speed_sq = (square_phase & 0x80000000) != 0;
        
        // Combine Layer 1 and 2 (ring-modulated digital fuzz)
        bool composite_val = audio_fuzz ^ speed_sq;
        
        // Layer 3: Lower-bit G.711 textures injected at 25% rate for digital crackling and gating
        if ((fast_rand(rand_seed) & 0x7FFF) < 8192) {
            composite_val ^= ((g711 & 0x10) != 0); // bit 4 texture
        }
        
        // Layer 4: High-frequency digital static / crackle spits (modulated by CV2 corruption and noise scale)
        int32_t cv2_abs = cv2 < 0 ? -cv2 : cv2;
        int32_t static_prob = 1000 + ((cv2_abs * eff_global_noise_scale) >> 15); // ranges 1000 to ~6000
        bool digital_static = (int32_t)(fast_rand(rand_seed) & 0x7FFF) < static_prob;
        composite_val ^= digital_static;
        
        // Layer 5: Vinyl CD-skip loop clicks / pops on loop boundary crossings (trig_out1)
        static int16_t click_timer = 0;
        if (glitcher.trig_out1) {
            click_timer = 48; // 2 ms click burst
        }
        if (click_timer > 0) {
            click_timer--;
            // Toggles at 12 kHz to generate a bright, scratchy high-frequency pop
            composite_val = (click_timer & 2) != 0;
        }

        PulseOut2(composite_val);
    } else {
        PulseOut2(false);
    }

    // --- Output ---
    // Gain makeup: scale back up by 6dB (shift left by 1) to restore output levels
    AudioOut1(soft_limit_q15((int32_t)L << 1) >> 4);
    AudioOut2(soft_limit_q15((int32_t)R << 1) >> 4);
}

// Core 1 entry point
void core1_entry() {
    // Configure hardware interpolators for Core 1's audio ISR:
    //   INTERP0 = blend mode  → lerp_delay_q15() (1-cycle delay interpolation)
    //   INTERP1 = clamp mode  → saturate_q15()   (branch-free Q15 saturation)
    init_hardware_interp();
    card.Run();
}

// ============================================================================
// Core 0: UI State
// ============================================================================
static int     currentPage          = 0;    // [0, 5]
static uint32_t pageFlashTimer      = 0;    // ms remaining in page-change flash

// Virtual parameter table — 8 pages × 3 knobs [Main, X, Y]
// Values are Q15 [0..32767]. Match atomic defaults above.
static int32_t vp[8][3] = {
    // Page 0 — Chorus / Tape Loss: Mix, Rate/Drift, Saturation/Damping
    {     0, 12000, 16384 },
    // Page 1 — Digital Loss: Mix/Strength, Decimate/Bitcrush, Corruption/Glitches
    {     0, 10000, 12000 },
    // Page 2 — Delay:    Wet,   Time,   Feedback
    {     0, 16384, 16384 },
    // Page 3 — Glitcher: Mix,   Size,   Speed Probability
    {     0, 16384, 16384 },
    // Page 4 — Filter:   Cutoff (DJ LP/HP), Resonance, Grit (Wavefolder)
    { 16384,  4000,  8000 },
    // Page 5 — Reverb:   Wet,   Decay,  Damping
    {     0, 16384,  8000 },
    // Page 6 — Sample Player: Start, Speed, Sample Select
    {     0, 16384,     0 },
    // Page 7 — Freeze Scrub: Scrub Pos, Loop Size, Playback Speed
    { 32767, 23000, 16384 },
};

// KnobLock instances — one per physical knob
static KnobLock lockMain, lockX, lockY, lockMacro;

// Smoothed knob values (IIR-filtered raw ADC readings)
static int32_t smMain = 0, smX = 0, smY = 0;

// Last known switch state (for debounce edge detection)
static bool freeze_latched = false;
static bool is_frozen       = false;



static void push_params_to_core1() {
    int32_t active_macro = g_macro_active ? grittiness_macro : 16384;
    for (int idx = 0; idx < 2; idx++) {
        volatile Core1Params &p = g_params[idx];

        // Precompute global noise scale first
        int32_t noise_scale = 16384;
        int32_t rev_damping = vp[5][2];
        if (rev_damping > 16384) {
            int32_t diff = rev_damping - 16384;
            noise_scale = 16384 + (diff << 1);
        }
        int32_t globalNoiseScale = scale_grit(noise_scale, 49152, active_macro);
        p.global_noise_scale = globalNoiseScale;

        p.chorus_mix       = scale_grit(apply_deadzone(vp[0][0]), 32767, active_macro, 16384);
        p.chorus_rate      = vp[0][1];
        p.chorus_depth_fb  = scale_grit(vp[0][2], 32767, active_macro, 16384);

        // Pre-decode Chorus depth, feedback, and destroy
        int32_t raw_depth_fb = p.chorus_depth_fb;
        int32_t chorus_depth = 0;
        int32_t chorus_feedback = 0;
        int32_t chorus_xor_mask = 0;
        if (raw_depth_fb < 16384) {
            chorus_depth = raw_depth_fb * 2;
            chorus_feedback = 0;
        } else {
            chorus_depth = 32767;
            if (raw_depth_fb < 26214) {
                chorus_feedback = ((raw_depth_fb - 16384) * 65536) >> 15;
            } else {
                chorus_feedback = 19660 + (((raw_depth_fb - 26214) * 57343) >> 15);
                chorus_xor_mask = (raw_depth_fb - 26214) >> 9;
            }
        }
        p.chorus_depth = chorus_depth;
        p.chorus_feedback = chorus_feedback;
        p.chorus_xor_mask = chorus_xor_mask;

        // Pre-calculate Codec levels
        int32_t raw_strength = scale_grit(apply_deadzone(vp[1][0]), 32767, active_macro, 16384);
        int32_t raw_downsample = scale_grit(vp[1][1], 24000, active_macro, 16384);
        int32_t raw_ringing_xor = scale_grit(vp[1][2], 32767, active_macro, 16384);
        
        p.codec_mix = raw_strength;
        p.codec_downsample = raw_downsample;
        p.codec_ringing_xor = raw_ringing_xor;

        int32_t X = raw_downsample;
        int32_t warped_ringing = (raw_ringing_xor * raw_ringing_xor) >> 15;

        int32_t mp3_ring_level = 0;
        int32_t fuzz_level = 0;
        int32_t decimate_level = 0;

        if (X < 11000) {
            mp3_ring_level = (X * 24402) >> 13;
            fuzz_level = 0;
            decimate_level = 0;
        } else if (X < 22000) {
            int32_t t = X - 11000;
            mp3_ring_level = 32767 - ((t * 24402) >> 13);
            fuzz_level = (t * 24402) >> 13;
            decimate_level = 0;
        } else {
            int32_t t = X - 22000;
            mp3_ring_level = 0;
            fuzz_level = 32767 - ((t * 24931) >> 13);
            decimate_level = (t * 24931) >> 13;
        }

        int32_t Y = warped_ringing; // cv2 is 0 at startup
        Y = clamp_i32(Y, 0, 32767);

        // --- NEW Y KNOB REBALANCING ---
        // 1. Tape/Vinyl Saturation & Noise (Y < 10000)
        int32_t tape_sat = 0;
        int32_t tape_hiss = 0;
        if (Y < 10000) {
            tape_sat = (Y * 3); // rises to 30000 at Y = 10000
            tape_hiss = (Y * 80) >> 15; // gentle noise
        } else {
            tape_sat = 30000 - (((Y - 10000) * 30000) / 22767);
            tape_hiss = 80 - (((Y - 10000) * 80) / 22767);
        }

        // 2. Vinyl Click slips (rises between 3000 and 8000, falls to 16000)
        int32_t pop_prob = 0;
        if (Y >= 3000 && Y < 8000) {
            pop_prob = ((Y - 3000) * 1) / 5000; // rises 0..1
        } else if (Y >= 8000 && Y < 16000) {
            pop_prob = 1 - (((Y - 8000) * 1) / 8000); // falls 1..0
        }

        // 3. CD Skips & Packet drops (Y >= 8000 && Y < 26000, peaks at 18000)
        int32_t bad_conn_level = 0;
        if (Y >= 8000 && Y < 18000) {
            bad_conn_level = ((Y - 8000) * 32767) / 10000;
        } else if (Y >= 18000 && Y < 26000) {
            bad_conn_level = 32767 - (((Y - 18000) * 32767) / 8000);
        }

        // 4. Broken Connection Scramble (Y >= 22000, rises 0..32767)
        int32_t scramble_level = 0;
        if (Y >= 22000) {
            scramble_level = ((Y - 22000) * 32767) / 10767;
        }

        // Scale pop_prob by X quality
        int32_t x_scale_q15 = 8000 + ((X * 24767) >> 15);
        pop_prob = (pop_prob * x_scale_q15) >> 15;

        // Fuzz level quadratic warp
        fuzz_level = (fuzz_level * fuzz_level) >> 15;

        // Scale by Main knob (strength)
        int32_t glitch_strength = 16384 + (raw_strength >> 1);
        fuzz_level = (fuzz_level * raw_strength) >> 15;
        decimate_level = (decimate_level * raw_strength) >> 15;
        mp3_ring_level = (mp3_ring_level * raw_strength) >> 15;
        bad_conn_level = (bad_conn_level * glitch_strength) >> 15;
        scramble_level = (scramble_level * glitch_strength) >> 15;

        int32_t pop_strength = 16384 + (raw_strength >> 1);
        pop_prob = (pop_prob * pop_strength) >> 15;

        // Global Noise Scale modifier
        fuzz_level = (fuzz_level * globalNoiseScale) >> 14;
        decimate_level = (decimate_level * globalNoiseScale) >> 14;
        mp3_ring_level = (mp3_ring_level * globalNoiseScale) >> 14;
        bad_conn_level = (bad_conn_level * globalNoiseScale) >> 14;
        scramble_level = (scramble_level * globalNoiseScale) >> 14;
        pop_prob = (pop_prob * globalNoiseScale) >> 14;

        if (fuzz_level > 32767) fuzz_level = 32767;
        if (decimate_level > 32767) decimate_level = 32767;
        if (mp3_ring_level > 32767) mp3_ring_level = 32767;
        if (bad_conn_level > 32767) bad_conn_level = 32767;
        if (scramble_level > 32767) scramble_level = 32767;

        int32_t click_ratio = 0;
        if (Y >= 3000 && Y < 11000) {
            click_ratio = ((Y - 3000) * 16384) / 8000;
        } else if (Y >= 11000 && Y < 22000) {
            int32_t t = Y - 11000;
            click_ratio = 16384 - ((t * 16384) / 11000);
        }
        int32_t click_depth = (click_ratio * raw_strength) >> 15;
        click_depth = (click_depth * x_scale_q15) >> 15;
        click_depth = (click_depth * 4000) >> 15;

        int32_t sputter_prob = ((raw_ringing_xor * raw_strength) >> 15) * 80 >> 15;
        sputter_prob = (sputter_prob * globalNoiseScale) >> 14;

        int32_t active_loss = bad_conn_level > scramble_level ? bad_conn_level : scramble_level;

        p.codec_mp3_ring = mp3_ring_level;
        p.codec_fuzz = fuzz_level;
        p.codec_decimate = decimate_level;
        p.codec_pop_prob = pop_prob;
        p.codec_click_depth = click_depth;
        p.codec_bad_conn = bad_conn_level;
        p.codec_scramble = scramble_level;
        p.codec_sputter_prob = sputter_prob;
        p.codec_tape_sat = tape_sat;
        p.codec_tape_hiss = tape_hiss;
        p.codec_active_loss = active_loss;

        p.delay_mix      = scale_grit(apply_deadzone(vp[2][0]), 32767, active_macro, 16384);
        
        // Scale delay time by active_macro
        int32_t raw_delay_time = vp[2][1];
        int32_t scaled_delay_time = raw_delay_time;
        if (active_macro < 16384) {
            if (raw_delay_time > 16384) {
                int32_t diff = raw_delay_time - 16384;
                scaled_delay_time = 16384 + ((diff * active_macro) >> 14);
            }
        } else {
            int32_t diff = 32767 - raw_delay_time;
            scaled_delay_time = raw_delay_time + ((diff * (active_macro - 16384)) / 16383);
        }
        p.delay_time     = scaled_delay_time;
        p.delay_feedback = scale_grit(vp[2][2], 32767, active_macro, 13107);

        bool is_freeze_page = (currentPage == 7);
        int32_t raw_glitch_speed = 16384;
        if (is_freeze_page || freeze_latched) {
            p.glitch_mix   = vp[7][0]; // scrub offset
            raw_glitch_speed = vp[7][2]; // speed (Y knob)
            p.glitch_size  = vp[7][1]; // loop size (X knob)
        } else {
            p.glitch_mix   = scale_grit(apply_deadzone(vp[3][0]), 32767, active_macro, 16384);
            p.glitch_size  = vp[3][1];
            raw_glitch_speed = scale_grit(vp[3][2], 32767, active_macro, 16384);
        }
        p.glitch_speed = raw_glitch_speed;

        // Glitch speed mapping
        int32_t glitch_speed_mapped = 65536;
        if (raw_glitch_speed > 18000) {
            glitch_speed_mapped = 65536 + (((raw_glitch_speed - 18000) * 65536) / 14767);
        } else if (raw_glitch_speed < 14000) {
            glitch_speed_mapped = -65536 + ((raw_glitch_speed * 131072) / 14000);
        }
        p.glitch_speed_mapped = glitch_speed_mapped;

        int32_t raw_fb = vp[2][2];
        int32_t glitch_fb = 0;
        if (raw_fb > 22937) {
            glitch_fb = ((raw_fb - 22937) * 109224) >> 15;
        }
        p.glitch_feedback = scale_grit(glitch_fb, 32767, active_macro, 13107);

        // Filter cutoff pre-scaling
        int32_t raw_filter_cutoff = vp[4][0];
        int32_t scaled_filter_cutoff = raw_filter_cutoff;
        if (active_macro < 16384) {
            int32_t diff = raw_filter_cutoff - 16384;
            scaled_filter_cutoff = 16384 + ((diff * active_macro) >> 14);
        }
        p.filter_cutoff = scaled_filter_cutoff;
        p.filter_res    = scale_grit(vp[4][1], 32767, active_macro, 13107);
        p.filter_morph  = scale_grit(vp[4][2], 32767, active_macro, 9830);

        p.freeze = freeze_latched;
        p.stutter = false;
        p.cv1 = 0;
        p.cv2 = 0;

        p.no_audio1 = true;
        p.no_audio2 = true;

        p.is_freeze_page = is_freeze_page;
        p.flash_writing  = false;
        p.grittiness_macro = active_macro;


        // Reverb params — computed with grittiness scaling on damping
        p.reverb_mix  = scale_grit(apply_deadzone(vp[5][0]), 32767, active_macro, 13107);
        {
            int32_t fb = vp[5][2];
            int32_t fb_glitch = fb;
            if (active_macro < 16384) {
                if (fb > 16384) fb_glitch = 16384 + (((fb - 16384) * active_macro) >> 14);
            } else {
                int32_t target_max = fb + (((32767 - fb) * 13107) >> 15);
                fb_glitch = fb + (((target_max - fb) * (active_macro - 16384)) / 16383);
            }
            
            int32_t size_scale = 4915 + (((int32_t)vp[5][1] * 27852) >> 15);
            p.reverb_size = size_scale;
            int32_t max_decay = 18000 + (((size_scale - 4915) * 38429) >> 20);

            int32_t decay = 0;
            if (fb_glitch < 20000) {
                int32_t val = (fb_glitch * 107374) >> 16;
                decay = (val * max_decay) >> 15;
            } else {
                decay = max_decay;
            }

            int32_t lofi_level = 0;
            if (fb_glitch < 12000) {
                lofi_level = 0;
            } else if (fb_glitch < 26000) {
                int32_t diff = fb_glitch - 12000;
                int32_t raw_lofi = (diff * 153391) >> 16;
                int32_t raw_sq = (raw_lofi * raw_lofi) >> 15;
                lofi_level = (raw_sq * raw_sq) >> 15;
            } else {
                lofi_level = 32767;
            }

            int32_t sparkle_level = 0;
            if (fb_glitch < 14000) {
                sparkle_level = 0;
            } else if (fb_glitch < 28000) {
                int32_t diff = fb_glitch - 14000;
                sparkle_level = (diff * 153391) >> 16;
            } else {
                sparkle_level = 32767;
            }

            int32_t circuit_bent_level = 0;
            if (fb_glitch < 20000) {
                circuit_bent_level = 0;
            } else {
                int32_t diff = fb_glitch - 20000;
                circuit_bent_level = (diff * 168188) >> 16;
                if (circuit_bent_level > 32767) circuit_bent_level = 32767;
            }

            int32_t damp = 30000 - ((lofi_level * 6000) >> 15);

            int32_t shift_q15 = (lofi_level * 6);
            int32_t int_shift = shift_q15 >> 15;
            int32_t frac_shift = shift_q15 & 0x7FFF;

            p.reverb_decay = decay;
            p.reverb_damp = damp;
            p.reverb_lofi_level = lofi_level;
            p.reverb_sparkle_level = sparkle_level;
            p.reverb_circuit_bent_level = circuit_bent_level;
            p.reverb_lofi_shift = int_shift;
            p.reverb_lofi_frac = frac_shift;
        }
    }
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

    // Apply deadzones to smoothed values to guarantee reaching absolute 0 and 32767
    int32_t dzMain = apply_deadzone(smMain);
    int32_t dzX    = apply_deadzone(smX);
    int32_t dzY    = apply_deadzone(smY);

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
    is_frozen = freeze_latched || pulse2_freeze;

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
        lockMacro.engage(dzMain, grittiness_macro, true);
        macro_adjusted_this_hold = false;
    }
    if (sw_down_exited) {
        if (active_sw_held_ms >= 350) {
            // Bake Chorus parameters
            vp[0][0] = scale_grit(vp[0][0], 32767, grittiness_macro, 16384);
            vp[0][2] = scale_grit(vp[0][2], 32767, grittiness_macro, 16384);

            // Bake Codec parameters
            vp[1][0] = scale_grit(vp[1][0], 32767, grittiness_macro, 16384);
            vp[1][1] = scale_grit(vp[1][1], 24000, grittiness_macro, 16384);
            vp[1][2] = scale_grit(vp[1][2], 32767, grittiness_macro, 16384);

            // Bake Delay parameters
            vp[2][0] = scale_grit(vp[2][0], 32767, grittiness_macro, 16384);
            {
                int32_t val = vp[2][1];
                if (grittiness_macro < 16384) {
                    if (val > 16384) {
                        int32_t diff = val - 16384;
                        vp[2][1] = 16384 + ((diff * grittiness_macro) >> 14);
                    }
                } else {
                    int32_t diff = 32767 - val;
                    vp[2][1] = val + ((diff * ((grittiness_macro - 16384) * 2)) >> 15);
                }
            }
            vp[2][2] = scale_grit(vp[2][2], 32767, grittiness_macro, 13107);

            // Bake Glitcher parameters
            vp[3][0] = scale_grit(vp[3][0], 32767, grittiness_macro, 16384);
            vp[3][2] = scale_grit(vp[3][2], 32767, grittiness_macro, 16384);

            // Bake Filter parameters
            {
                int32_t val = vp[4][0];
                if (grittiness_macro < 16384) {
                    int32_t diff = val - 16384;
                    vp[4][0] = 16384 + ((diff * grittiness_macro) >> 14);
                }
            }
            vp[4][1] = scale_grit(vp[4][1], 32767, grittiness_macro, 13107);
            vp[4][2] = scale_grit(vp[4][2], 32767, grittiness_macro, 9830);

            // Bake Reverb parameters
            vp[5][0] = scale_grit(vp[5][0], 32767, grittiness_macro, 13107);
            {
                int32_t val = vp[5][2];
                if (grittiness_macro < 16384) {
                    if (val > 16384) {
                        int32_t diff = val - 16384;
                        vp[5][2] = 16384 + ((diff * grittiness_macro) >> 14);
                    }
                } else {
                    int32_t target_max = val + (((32767 - val) * 13107) >> 15);
                    vp[5][2] = val + (((target_max - val) * (grittiness_macro - 16384)) / 16383);
                }
            }

            if (!macro_adjusted_this_hold) {
                if (currentPage >= 6) {
                    currentPage = 0;
                    freeze_latched = false;
                    is_frozen = freeze_latched || pulse2_freeze;
                } else {
                    currentPage = (currentPage + 5) % 6;
                }
                lockMain.engage(dzMain, vp[currentPage][0]);
                lockX.engage(dzX, vp[currentPage][1]);
                lockY.engage(dzY, vp[currentPage][2]);
                pageFlashTimer = 400;
            }

            grittiness_macro = 16384;
            g_macro_active = false;
            lockMain.engage(dzMain, vp[currentPage][0]);
            lockX.engage(dzX, vp[currentPage][1]);
            lockY.engage(dzY, vp[currentPage][2]);
            lockMacro.engage(dzMain, grittiness_macro);
            param_changed = true;
        } else {
            lockMain.engage(dzMain, vp[currentPage][0]);
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
                        if (currentPage != 7) {
                            if (currentPage < 6) {
                                pageBeforeUp = currentPage;
                            }
                            currentPage = 7;
                            freeze_latched = true;
                            is_frozen = true;
                            lockMain.engage(dzMain, vp[currentPage][0]);
                            lockX.engage(dzX, vp[currentPage][1]);
                            lockY.engage(dzY, vp[currentPage][2]);
                            param_changed = true;
                        }
                    }
                }
            }
        }
    } else {
        // Transition detected
        if (last_debounced_sw == ComputerCard::Switch::Middle && debounced_sw == ComputerCard::Switch::Up) {
            // Instant freeze transition on switch press to eliminate 350ms latency!
            if (currentPage != 7) {
                if (currentPage < 6) {
                    pageBeforeUp = currentPage;
                }
                currentPage = 7;
                freeze_latched = true;
                is_frozen = true;
                lockMain.engage(dzMain, vp[currentPage][0]);
                lockX.engage(dzX, vp[currentPage][1]);
                lockY.engage(dzY, vp[currentPage][2]);
                param_changed = true;
            }
        }
        if (last_debounced_sw != ComputerCard::Switch::Middle && debounced_sw == ComputerCard::Switch::Middle) {
            // Switch was released
            if (active_sw_held_ms < 350) {
                // Trigger flick action
                if (last_debounced_sw == ComputerCard::Switch::Down) {
                    // Flick DOWN
                    if (!macro_adjusted_this_hold) {
                        if (currentPage >= 6) {
                            currentPage = 0;
                            freeze_latched = false;
                            is_frozen = freeze_latched || pulse2_freeze;
                        } else {
                            currentPage = (currentPage + 1) % 6;
                        }
                        lockMain.engage(dzMain, vp[currentPage][0]);
                        lockX.engage(dzX, vp[currentPage][1]);
                        lockY.engage(dzY, vp[currentPage][2]);
                        pageFlashTimer = 400;
                        param_changed = true;
                    }
                } else if (last_debounced_sw == ComputerCard::Switch::Up) {
                    // Flick UP: unfreeze immediately since we froze instantly on press!
                    freeze_latched = false;
                    is_frozen = freeze_latched || pulse2_freeze;
                    if (currentPage >= 6) {
                        currentPage = pageBeforeUp;
                        lockMain.engage(dzMain, vp[currentPage][0]);
                        lockX.engage(dzX, vp[currentPage][1]);
                        lockY.engage(dzY, vp[currentPage][2]);
                        param_changed = true;
                    }
                }
            } else {
                // Hold release action
                if (last_debounced_sw == ComputerCard::Switch::Up) {
                    // Releasing Switch UP: return to the page we were on before and unfreeze
                    freeze_latched = false;
                    is_frozen = freeze_latched || pulse2_freeze;
                    if (currentPage >= 6) {
                        currentPage = pageBeforeUp;
                        lockMain.engage(dzMain, vp[currentPage][0]);
                        lockX.engage(dzX, vp[currentPage][1]);
                        lockY.engage(dzY, vp[currentPage][2]);
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
    // ── 4. KnobLock gating → update virtual parameters ───────────────────
    static uint32_t boot_lock_timer = 300;
    if (boot_lock_timer > 0) {
        boot_lock_timer--;
        lockMain.engage(dzMain, vp[currentPage][0]);
        lockX.engage(dzX, vp[currentPage][1]);
        lockY.engage(dzY, vp[currentPage][2]);
        lockMacro.engage(dzMain, grittiness_macro);
    }

    if (debounced_sw == ComputerCard::Switch::Down) {
        if (active_sw_held_ms >= 350) {
            int32_t nextMacro = lockMacro.update(dzMain);
            if (grittiness_macro != nextMacro) {
                grittiness_macro = nextMacro;
                macro_adjusted_this_hold = true;
                param_changed = true;
            }
        }
    } else {
        int32_t nextMain = lockMain.update(dzMain);
        if (vp[currentPage][0] != nextMain) {
            vp[currentPage][0] = nextMain;
            param_changed = true;
        }
    }
    int32_t nextX = lockX.update(dzX);
    if (vp[currentPage][1] != nextX) {
        vp[currentPage][1] = nextX;
        param_changed = true;
    }
    int32_t nextY = lockY.update(dzY);
    if (vp[currentPage][2] != nextY) {
        vp[currentPage][2] = nextY;
        param_changed = true;
    }

    static bool last_is_frozen = false;
    if (is_frozen && !last_is_frozen) {
        // Re-engage knob locks to prevent sudden physical knob jumps
        lockMain.engage(dzMain, vp[7][0]);
        lockX.engage(dzX, vp[7][1]);
        lockY.engage(dzY, vp[7][2]);
    }
    last_is_frozen = is_frozen;

    // ── 5. Push virtual params to Core 1 double buffer ───────────────────
    {
        uint32_t next_idx = 1 - g_params_idx.load(std::memory_order_relaxed);
        volatile Core1Params &p = g_params[next_idx];
        
        g_macro_active = (debounced_sw == ComputerCard::Switch::Down && active_sw_held_ms >= 350);
        int32_t base_macro = g_macro_active ? grittiness_macro : 16384;

        // Read CV1 and calculate active_macro on Core 0
        int32_t cv1_val = cv1_live ? (CVIn1()) : 0;
        int32_t cv1_offset = cv1_live ? (cv1_val * 8) : 0;
        int32_t active_macro = clamp_i32(base_macro + cv1_offset, 0, 32767);

        // Precompute global noise scale first
        int32_t noise_scale = 16384;
        int32_t rev_damping = vp[5][2];
        if (rev_damping > 16384) {
            int32_t diff = rev_damping - 16384;
            noise_scale = 16384 + (diff << 1);
        }
        int32_t globalNoiseScale = scale_grit(noise_scale, 49152, active_macro);
        p.global_noise_scale = globalNoiseScale;

        p.chorus_mix       = scale_grit(apply_deadzone(vp[0][0]), 32767, active_macro, 16384);
        p.chorus_rate      = vp[0][1];
        p.chorus_depth_fb  = scale_grit(vp[0][2], 32767, active_macro, 16384);

        // Pre-decode Chorus depth, feedback, and destroy
        int32_t raw_depth_fb = p.chorus_depth_fb;
        int32_t chorus_depth = 0;
        int32_t chorus_feedback = 0;
        int32_t chorus_xor_mask = 0;
        if (raw_depth_fb < 16384) {
            chorus_depth = raw_depth_fb * 2;
            chorus_feedback = 0;
        } else {
            chorus_depth = 32767;
            if (raw_depth_fb < 26214) {
                chorus_feedback = ((raw_depth_fb - 16384) * 65536) >> 15;
            } else {
                chorus_feedback = 19660 + (((raw_depth_fb - 26214) * 57343) >> 15);
                chorus_xor_mask = (raw_depth_fb - 26214) >> 9;
            }
        }
        p.chorus_depth = chorus_depth;
        p.chorus_feedback = chorus_feedback;
        p.chorus_xor_mask = chorus_xor_mask;

        // Pre-calculate Codec levels
        int32_t raw_strength = scale_grit(apply_deadzone(vp[1][0]), 32767, active_macro, 16384);
        int32_t raw_downsample = scale_grit(vp[1][1], 24000, active_macro, 16384);
        int32_t raw_ringing_xor = scale_grit(vp[1][2], 32767, active_macro, 16384);
        
        p.codec_mix = raw_strength;
        p.codec_downsample = raw_downsample;
        p.codec_ringing_xor = raw_ringing_xor;

        int32_t X = raw_downsample;
        int32_t warped_ringing = (raw_ringing_xor * raw_ringing_xor) >> 15;

        int32_t mp3_ring_level = 0;
        int32_t fuzz_level = 0;
        int32_t decimate_level = 0;

        if (X < 11000) {
            mp3_ring_level = (X * 24402) >> 13;
            fuzz_level = 0;
            decimate_level = 0;
        } else if (X < 22000) {
            int32_t t = X - 11000;
            mp3_ring_level = 32767 - ((t * 24402) >> 13);
            fuzz_level = (t * 24402) >> 13;
            decimate_level = 0;
        } else {
            int32_t t = X - 22000;
            mp3_ring_level = 0;
            fuzz_level = 32767 - ((t * 24931) >> 13);
            decimate_level = (t * 24931) >> 13;
        }

        int32_t cv2_val = cv2_live ? (CVIn2()) : 0;
        int32_t Y = warped_ringing + (cv2_val * 8);
        Y = clamp_i32(Y, 0, 32767);

        // --- NEW Y KNOB REBALANCING ---
        // 1. Tape/Vinyl Saturation & Noise (Y < 10000)
        int32_t tape_sat = 0;
        int32_t tape_hiss = 0;
        if (Y < 10000) {
            tape_sat = (Y * 3); // rises to 30000 at Y = 10000
            tape_hiss = (Y * 80) >> 15; // gentle noise
        } else {
            tape_sat = 30000 - (((Y - 10000) * 30000) / 22767);
            tape_hiss = 80 - (((Y - 10000) * 80) / 22767);
        }

        // 2. Vinyl Click slips (rises between 3000 and 8000, falls to 16000)
        int32_t pop_prob = 0;
        if (Y >= 3000 && Y < 8000) {
            pop_prob = ((Y - 3000) * 1) / 5000; // rises 0..1
        } else if (Y >= 8000 && Y < 16000) {
            pop_prob = 1 - (((Y - 8000) * 1) / 8000); // falls 1..0
        }

        // 3. CD Skips & Packet drops (Y >= 8000 && Y < 26000, peaks at 18000)
        int32_t bad_conn_level = 0;
        if (Y >= 8000 && Y < 18000) {
            bad_conn_level = ((Y - 8000) * 32767) / 10000;
        } else if (Y >= 18000 && Y < 26000) {
            bad_conn_level = 32767 - (((Y - 18000) * 32767) / 8000);
        }

        // 4. Broken Connection Scramble (Y >= 22000, rises 0..32767)
        int32_t scramble_level = 0;
        if (Y >= 22000) {
            scramble_level = ((Y - 22000) * 32767) / 10767;
        }

        // Scale pop_prob by X quality
        int32_t x_scale_q15 = 8000 + ((X * 24767) >> 15);
        pop_prob = (pop_prob * x_scale_q15) >> 15;

        // Fuzz level quadratic warp
        fuzz_level = (fuzz_level * fuzz_level) >> 15;

        // Scale by Main knob (strength)
        int32_t glitch_strength = 16384 + (raw_strength >> 1);
        fuzz_level = (fuzz_level * raw_strength) >> 15;
        decimate_level = (decimate_level * raw_strength) >> 15;
        mp3_ring_level = (mp3_ring_level * raw_strength) >> 15;
        bad_conn_level = (bad_conn_level * glitch_strength) >> 15;
        scramble_level = (scramble_level * glitch_strength) >> 15;

        int32_t pop_strength = 16384 + (raw_strength >> 1);
        pop_prob = (pop_prob * pop_strength) >> 15;

        // Global Noise Scale modifier
        fuzz_level = (fuzz_level * globalNoiseScale) >> 14;
        decimate_level = (decimate_level * globalNoiseScale) >> 14;
        mp3_ring_level = (mp3_ring_level * globalNoiseScale) >> 14;
        bad_conn_level = (bad_conn_level * globalNoiseScale) >> 14;
        scramble_level = (scramble_level * globalNoiseScale) >> 14;
        pop_prob = (pop_prob * globalNoiseScale) >> 14;

        if (fuzz_level > 32767) fuzz_level = 32767;
        if (decimate_level > 32767) decimate_level = 32767;
        if (mp3_ring_level > 32767) mp3_ring_level = 32767;
        if (bad_conn_level > 32767) bad_conn_level = 32767;
        if (scramble_level > 32767) scramble_level = 32767;

        int32_t click_ratio = 0;
        if (Y >= 3000 && Y < 11000) {
            click_ratio = ((Y - 3000) * 16384) / 8000;
        } else if (Y >= 11000 && Y < 22000) {
            int32_t t = Y - 11000;
            click_ratio = 16384 - ((t * 16384) / 11000);
        }
        int32_t click_depth = (click_ratio * raw_strength) >> 15;
        click_depth = (click_depth * x_scale_q15) >> 15;
        click_depth = (click_depth * 4000) >> 15;

        int32_t sputter_prob = ((raw_ringing_xor * raw_strength) >> 15) * 80 >> 15;
        sputter_prob = (sputter_prob * globalNoiseScale) >> 14;

        int32_t active_loss = bad_conn_level > scramble_level ? bad_conn_level : scramble_level;

        p.codec_mp3_ring = mp3_ring_level;
        p.codec_fuzz = fuzz_level;
        p.codec_decimate = decimate_level;
        p.codec_pop_prob = pop_prob;
        p.codec_click_depth = click_depth;
        p.codec_bad_conn = bad_conn_level;
        p.codec_scramble = scramble_level;
        p.codec_sputter_prob = sputter_prob;
        p.codec_tape_sat = tape_sat;
        p.codec_tape_hiss = tape_hiss;
        p.codec_active_loss = active_loss;

        p.delay_mix      = scale_grit(apply_deadzone(vp[2][0]), 32767, active_macro, 16384);
        
        // Scale delay time by active_macro
        int32_t raw_delay_time = vp[2][1];
        int32_t scaled_delay_time = raw_delay_time;
        if (active_macro < 16384) {
            if (raw_delay_time > 16384) {
                int32_t diff = raw_delay_time - 16384;
                scaled_delay_time = 16384 + ((diff * active_macro) >> 14);
            }
        } else {
            int32_t diff = 32767 - raw_delay_time;
            scaled_delay_time = raw_delay_time + ((diff * (active_macro - 16384)) / 16383);
        }
        p.delay_time     = scaled_delay_time;
        p.delay_feedback = scale_grit(vp[2][2], 32767, active_macro, 13107);

        bool is_freeze_page = (currentPage == 7);
        int32_t raw_glitch_speed = 16384;
        if (is_freeze_page || is_frozen) {
            p.glitch_mix   = vp[7][0]; // scrub offset
            raw_glitch_speed = vp[7][2]; // speed (Y knob)
            p.glitch_size  = vp[7][1]; // loop size (X knob)
        } else {
            p.glitch_mix   = scale_grit(apply_deadzone(vp[3][0]), 32767, active_macro, 16384);
            p.glitch_size  = vp[3][1];
            raw_glitch_speed = scale_grit(vp[3][2], 32767, active_macro, 16384);
        }
        p.glitch_speed = raw_glitch_speed;

        // Glitch speed mapping
        int32_t glitch_speed_mapped = 65536;
        if (raw_glitch_speed > 18000) {
            glitch_speed_mapped = 65536 + (((raw_glitch_speed - 18000) * 65536) / 14767);
        } else if (raw_glitch_speed < 14000) {
            glitch_speed_mapped = -65536 + ((raw_glitch_speed * 131072) / 14000);
        }
        p.glitch_speed_mapped = glitch_speed_mapped;

        // Calculate Glitcher Feedback Loop scaled by grittiness macro
        int32_t raw_fb = vp[2][2];
        int32_t glitch_fb = 0;
        if (raw_fb > 22937) {
            glitch_fb = ((raw_fb - 22937) * 109224) >> 15;
        }
        p.glitch_feedback = scale_grit(glitch_fb, 32767, active_macro, 13107);

        // Filter cutoff pre-scaling
        int32_t raw_filter_cutoff = vp[4][0];
        int32_t scaled_filter_cutoff = raw_filter_cutoff;
        if (active_macro < 16384) {
            int32_t diff = raw_filter_cutoff - 16384;
            scaled_filter_cutoff = 16384 + ((diff * active_macro) >> 14);
        }
        p.filter_cutoff = scaled_filter_cutoff;
        p.filter_res    = scale_grit(vp[4][1], 32767, active_macro, 13107);
        p.filter_morph  = scale_grit(vp[4][2], 32767, active_macro, 9830);

        p.freeze = is_frozen;
        p.stutter = pulse1_live && PulseIn1();
        p.cv1 = cv1_live ? cv1_val : 0;
        p.cv2 = cv2_live ? CVIn2() : 0;
        p.pulse1_live = pulse1_live;
        p.pulse2_live = pulse2_live;
        p.cv1_live = cv1_live;
        p.cv2_live = cv2_live;

        p.no_audio1 = debounced_no_audio1;
        p.no_audio2 = debounced_no_audio2;

        p.is_freeze_page = is_freeze_page;
        p.flash_writing  = false;
        p.grittiness_macro = active_macro;



        // Reverb params
        p.reverb_mix  = scale_grit(apply_deadzone(vp[5][0]), 32767, active_macro, 13107);
        {
            int32_t fb = vp[5][2];
            int32_t fb_glitch = fb;
            if (active_macro < 16384) {
                if (fb > 16384) fb_glitch = 16384 + (((fb - 16384) * active_macro) >> 14);
            } else {
                int32_t target_max = fb + (((32767 - fb) * 13107) >> 15);
                fb_glitch = fb + (((target_max - fb) * (active_macro - 16384)) / 16383);
            }
            
            int32_t size_scale = 4915 + (((int32_t)vp[5][1] * 27852) >> 15);
            p.reverb_size = size_scale;
            int32_t max_decay = 18000 + (((size_scale - 4915) * 38429) >> 20);

            int32_t decay = 0;
            if (fb_glitch < 20000) {
                int32_t val = (fb_glitch * 107374) >> 16;
                decay = (val * max_decay) >> 15;
            } else {
                decay = max_decay;
            }

            int32_t lofi_level = 0;
            if (fb_glitch < 12000) {
                lofi_level = 0;
            } else if (fb_glitch < 26000) {
                int32_t diff = fb_glitch - 12000;
                int32_t raw_lofi = (diff * 153391) >> 16;
                int32_t raw_sq = (raw_lofi * raw_lofi) >> 15;
                lofi_level = (raw_sq * raw_sq) >> 15;
            } else {
                lofi_level = 32767;
            }

            int32_t sparkle_level = 0;
            if (fb_glitch < 14000) {
                sparkle_level = 0;
            } else if (fb_glitch < 28000) {
                int32_t diff = fb_glitch - 14000;
                sparkle_level = (diff * 153391) >> 16;
            } else {
                sparkle_level = 32767;
            }

            int32_t circuit_bent_level = 0;
            if (fb_glitch < 20000) {
                circuit_bent_level = 0;
            } else {
                int32_t diff = fb_glitch - 20000;
                circuit_bent_level = (diff * 168188) >> 16;
                if (circuit_bent_level > 32767) circuit_bent_level = 32767;
            }

            int32_t damp = 30000 - ((lofi_level * 6000) >> 15);

            int32_t shift_q15 = (lofi_level * 6);
            int32_t int_shift = shift_q15 >> 15;
            int32_t frac_shift = shift_q15 & 0x7FFF;

            p.reverb_decay = decay;
            p.reverb_damp = damp;
            p.reverb_lofi_level = lofi_level;
            p.reverb_sparkle_level = sparkle_level;
            p.reverb_circuit_bent_level = circuit_bent_level;
            p.reverb_lofi_shift = int_shift;
            p.reverb_lofi_frac = frac_shift;
        }

        g_params_idx.store(next_idx, std::memory_order_release);
    }
    // ── 6. LED visualisation ──────────────────────────────────────────────
    bool is_freeze_page = (currentPage == 7);

    if (debounced_sw == ComputerCard::Switch::Down && active_sw_held_ms >= 350) {
        int16_t bar_leds[6];
        if (lockMacro.locked) {
            // Knob is locked, show catchup helper
            get_bar_graph_leds(lockMacro.val, bar_leds);
            // Make the virtual value target dim
            for (int i = 0; i < 6; i++) {
                bar_leds[i] = (bar_leds[i] * 300) >> 12; // dim it down
            }
            // Flash the physical position at 5Hz
            static uint32_t blink_counter = 0;
            blink_counter++;
            bool blink_on = (blink_counter % 200 < 100);
            if (blink_on) {
                int32_t phys_val = dzMain;
                int phys_idx = phys_val / 5461;
                if (phys_idx < 0) phys_idx = 0;
                if (phys_idx > 5) phys_idx = 5;
                bar_leds[phys_idx] = 4095; // flash bright
            }
        } else {
            // Unlocked, show normal macro value solid
            get_bar_graph_leds(grittiness_macro, bar_leds);
        }
        for (int i = 0; i < 6; i++) {
            LedBrightness(i, bar_leds[i]);
        }
    } else if (is_freeze_page) {
        // Freeze Scrub page: show the active loop window (glow) and the moving playhead (bright)
        int32_t start_idx = ((int32_t)glitcher.freeze_wr - glitcher.active_offset) & 0x7FFF;
        int32_t end_idx = (start_idx + glitcher.current_loop_len) & 0x7FFF;
        int32_t play_pos = (start_idx + (int32_t)(glitcher.rd_q16 >> 16)) & 0x7FFF;
        
        for (int i = 0; i < 6; i++) {
            int32_t led_sample = i * 5461 + 2730; // center of LED's sector
            bool in_loop = false;
            if (start_idx <= end_idx) {
                in_loop = (led_sample >= start_idx && led_sample < end_idx);
            } else {
                in_loop = (led_sample >= start_idx || led_sample < end_idx);
            }
            
            // Determine base brightness: glow if inside the loop window, very dim if outside
            int32_t brightness = in_loop ? 800 : 80;
            
            // Highlight the playhead position
            int32_t play_led = play_pos / 5461;
            if (play_led < 0) play_led = 0;
            if (play_led > 5) play_led = 5;
            if (play_led == i) {
                brightness = 4095;
            }
            
            LedBrightness(i, brightness);
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
    // Core 0 is a pure UI loop — no audio processing, no FIFO.
    // All DSP including reverb runs on Core 1 inside ProcessSample().
    // tick_ui_once() fires every ~1 ms using the hardware microsecond timer.
    uint32_t last_tick_us = time_us_32();
    while (1) {
        uint32_t now = time_us_32();
        if ((now - last_tick_us) >= 1000) {
            last_tick_us += 1000;
            tick_ui_once();
        }
    }
}

// ============================================================================
// main() — CPU setup, block init, core launch
// ============================================================================
int main() {
    // 1. Run at 192 MHz — clean PLL multiple of 48 kHz, slight overvolt for stability.
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_ms(10);
    set_sys_clock_khz(192000, true);

    // 2. Init stdio and the fixed-point sine table.
    stdio_init_all();
    init_sine_table();

    // Precompute mu-law decode table for branch-free lookups on Core 1
    for (int i = 0; i < 256; i++) {
        int32_t sign = (i & 0x80) ? -1 : 1;
        int32_t exponent = (i >> 4) & 0x07;
        int32_t mantissa = i & 0x0F;
        int32_t reconstructed = 0;
        if (exponent == 0) {
            reconstructed = (mantissa << 3) + 4;
        } else {
            reconstructed = ((mantissa << 3) + 132) << exponent;
        }
        if (reconstructed > 32767) reconstructed = 32767;
        mulaw_decode_table[i] = (int16_t)(sign * reconstructed);
    }

    // 3. Init sample headers and all DSP blocks — zeroes buffers.
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
    g_params[0].global_noise_scale = 16384;
    g_params[1].global_noise_scale = 16384;
    g_params[0].no_audio1 = true;
    g_params[1].no_audio1 = true;
    g_params[0].no_audio2 = true;
    g_params[1].no_audio2 = true;
    g_params[0].grittiness_macro = 16384;
    g_params[1].grittiness_macro = 16384;

    // 6. Push default settings to Core 1 double-buffer before Core 1 starts.
    push_params_to_core1();

    // 7. Launch Core 1 (starts the background ADC interrupts & DMA).
    multicore_launch_core1(core1_entry);

    // 8. Sleep for 10 ms to let the background ADC/multiplexer interrupts populate the knobs array.
    sleep_ms(10);

    // 9. Read initial knob positions (now populated with true physical readings!)
    //    and engage KnobLocks so they start locked to the page's defaults.
    smMain = card.ReadKnob(ComputerCard::Knob::Main) << 3;
    smX    = card.ReadKnob(ComputerCard::Knob::X)    << 3;
    smY    = card.ReadKnob(ComputerCard::Knob::Y)    << 3;
    lockMain.engage(smMain, vp[0][0]);
    lockX.engage(smX, vp[0][1]);
    lockY.engage(smY, vp[0][2]);
    lockMacro.engage(smMain, grittiness_macro);

    // 10. Enter Core 0 UI loop — never returns.
    card.run_core0_ui_loop();
}
