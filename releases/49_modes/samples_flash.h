// =============================================================================
// samples_flash.h — Flash-resident exciter sample data declarations
//
// These arrays are stored in flash (XIP) using __in_flash() attribute,
// meaning they cost ZERO RAM even with copy_to_ram binary type.
// Read access is slightly slower than RAM (~1 extra wait state on cache miss)
// but perfectly fine for sequential sample playback.
//
// Total flash usage: ~330 KB (16% of 2MB flash)
// =============================================================================

#ifndef SAMPLES_FLASH_H_
#define SAMPLES_FLASH_H_

#include <stdint.h>

// ── Exciter sample data (250 KB) ────────────────────────────────────────────
// Contains 9 short percussion/pluck samples used by the sample player exciter.
// Segments are delimited by smp_boundaries[].
extern const int16_t smp_sample_data[];   // [128013]

// ── Noise sample data (80 KB) ───────────────────────────────────────────────
// Long noise/texture recording used by the granular sample player exciter.
// Different offsets (via signature parameter) give different timbres.
extern const int16_t smp_noise_sample[];  // [40963]

// ── Sample segment boundaries ───────────────────────────────────────────────
// Index into smp_sample_data for 9 different sample segments.
// boundary[i] is the start offset, boundary[i+1]-1 is the end.
extern const uint32_t smp_boundaries[];   // [10]

#define SMP_NUM_SAMPLES       9
#define SMP_NOISE_LENGTH      40963

#endif  // SAMPLES_FLASH_H_
