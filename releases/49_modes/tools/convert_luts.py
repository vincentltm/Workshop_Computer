#!/usr/bin/env python3
"""
Convert Elements float lookup tables to Q15/Q16 fixed-point.

Reads the original resources.cc and generates resources_q15.h/cpp
with integer lookup tables suitable for the RP2040 (no FPU).

Usage: python3 convert_luts.py > ../resources_q15.cpp
"""

import re
import sys
import os

RESOURCES_CC = os.path.join(os.path.dirname(__file__), 
    '..', 'lib', 'mi-UGens-main', 'eurorack', 'elements', 'resources.cc')

def extract_float_array(content, name):
    """Extract a float array from resources.cc by name."""
    pattern = rf'const float {name}\[\] = \{{([^;]+)\}};'
    m = re.search(pattern, content, re.DOTALL)
    if not m:
        print(f"WARNING: Could not find {name}", file=sys.stderr)
        return []
    raw = m.group(1)
    # Parse all float values
    values = []
    for token in re.findall(r'[-+]?\d+\.\d+(?:e[+-]?\d+)?', raw):
        values.append(float(token))
    return values

def extract_int16_array(content, name):
    """Extract an int16 array from resources.cc by name."""
    pattern = rf'const int16_t {name}\[\] = \{{([^;]+)\}};'
    m = re.search(pattern, content, re.DOTALL)
    if not m:
        print(f"WARNING: Could not find {name}", file=sys.stderr)
        return []
    raw = m.group(1)
    values = []
    for token in re.findall(r'[-+]?\d+', raw):
        values.append(int(token))
    return values

def to_q15(values, scale=32767.0):
    """Convert float values (assumed -1..1) to Q15 int16."""
    result = []
    for v in values:
        q = int(round(v * scale))
        q = max(-32768, min(32767, q))
        result.append(q)
    return result

def to_q16(values, scale=65536.0):
    """Convert float values to Q16 int32 (16 fractional bits)."""
    result = []
    for v in values:
        q = int(round(v * scale))
        result.append(q)
    return result

def format_array_int16(name, values, per_line=8):
    """Format as C int16_t array."""
    lines = [f'const int16_t {name}[] = {{']
    for i in range(0, len(values), per_line):
        chunk = values[i:i+per_line]
        lines.append('    ' + ', '.join(f'{v:6d}' for v in chunk) + ',')
    lines.append('};')
    return '\n'.join(lines)

def format_array_int32(name, values, per_line=6):
    """Format as C int32_t array."""
    lines = [f'const int32_t {name}[] = {{']
    for i in range(0, len(values), per_line):
        chunk = values[i:i+per_line]
        lines.append('    ' + ', '.join(f'{v:10d}' for v in chunk) + ',')
    lines.append('};')
    return '\n'.join(lines)

def format_array_uint32(name, values, per_line=6):
    """Format as C uint32_t array."""
    lines = [f'const uint32_t {name}[] = {{']
    for i in range(0, len(values), per_line):
        chunk = values[i:i+per_line]
        lines.append('    ' + ', '.join(f'{v:10d}u' for v in chunk) + ',')
    lines.append('};')
    return '\n'.join(lines)


def main():
    with open(RESOURCES_CC, 'r') as f:
        content = f.read()
    
    print("// =============================================================================")
    print("// resources_q15.cpp — Auto-generated from MI Elements resources.cc")
    print("// Converted float LUTs to Q15/Q16 fixed-point for RP2040 (no FPU)")
    print("// =============================================================================")
    print()
    print('#include "resources_q15.h"')
    print()
    
    # ── Sine table (Q15): values are -1.0..1.0, 4097 entries ──
    sine = extract_float_array(content, 'lut_sine')
    if sine:
        print(f"// lut_sine: {len(sine)} entries, float -1.0..1.0 → Q15")
        print(format_array_int16('lut_sine_q15', to_q15(sine)))
        print()
    
    # ── SVF coefficient tables (these need special handling) ──
    # lut_approx_svf_g: g coefficient, range ~0.001 to ~1.5
    # These values can exceed 1.0, so we use Q14 (scale 16384) to give 2× headroom
    svf_g = extract_float_array(content, 'lut_approx_svf_g')
    if svf_g:
        max_g = max(abs(v) for v in svf_g)
        print(f"// lut_approx_svf_g: {len(svf_g)} entries, max={max_g:.4f}")
        print(f"// Stored as Q14 (÷16384) to accommodate values > 1.0")
        q14_g = []
        for v in svf_g:
            q = int(round(v * 16384.0))
            q14_g.append(max(-32768, min(32767, q)))
        print(format_array_int16('lut_approx_svf_g_q14', q14_g))
        print()
    
    # lut_approx_svf_r: 1/Q, range ~0.002 to ~2.0 
    svf_r = extract_float_array(content, 'lut_approx_svf_r')
    if svf_r:
        max_r = max(abs(v) for v in svf_r)
        print(f"// lut_approx_svf_r: {len(svf_r)} entries, max={max_r:.4f}")
        print(f"// Stored as Q14 (÷16384)")
        q14_r = []
        for v in svf_r:
            q = int(round(v * 16384.0))
            q14_r.append(max(-32768, min(32767, q)))
        print(format_array_int16('lut_approx_svf_r_q14', q14_r))
        print()
    
    # lut_approx_svf_h: 1/(1+rg+g²), range ~0.0 to ~1.0
    svf_h = extract_float_array(content, 'lut_approx_svf_h')
    if svf_h:
        max_h = max(abs(v) for v in svf_h)
        print(f"// lut_approx_svf_h: {len(svf_h)} entries, max={max_h:.4f}")
        print(format_array_int16('lut_approx_svf_h_q15', to_q15(svf_h)))
        print()

    # lut_approx_svf_gain: overall gain compensation
    svf_gain = extract_float_array(content, 'lut_approx_svf_gain')
    if svf_gain:
        max_gain = max(abs(v) for v in svf_gain)
        print(f"// lut_approx_svf_gain: {len(svf_gain)} entries, max={max_gain:.4f}")
        # Gain ranges from ~1.18 to ~420.0, output as int32_t in Q15
        q15_gain = [int(round(v * 32768.0)) for v in svf_gain]
        print(format_array_int32('lut_approx_svf_gain_q15', q15_gain))
        print()
    
    # ── Stiffness table ──
    stiffness = extract_float_array(content, 'lut_stiffness')
    if stiffness:
        max_s = max(abs(v) for v in stiffness)
        print(f"// lut_stiffness: {len(stiffness)} entries, max={max_s:.4f}")
        print(format_array_int16('lut_stiffness_q15', to_q15(stiffness)))
        print()
    
    # ── 4 decades: exponential scaling 0.001..10.0 ──
    decades = extract_float_array(content, 'lut_4_decades')
    if decades:
        max_d = max(abs(v) for v in decades)
        # Range is large (0.001 to 10), use Q16 int32
        print(f"// lut_4_decades: {len(decades)} entries, max={max_d:.4f}")
        print(f"// Stored as Q16 (÷65536) to handle range 0..10+")
        print(format_array_int32('lut_4_decades_q16', to_q16(decades)))
        print()
    
    # ── Accent gain ──
    accent_c = extract_float_array(content, 'lut_accent_gain_coarse')
    if accent_c:
        max_ac = max(abs(v) for v in accent_c)
        print(f"// lut_accent_gain_coarse: {len(accent_c)} entries, max={max_ac:.4f}")
        # Values can be > 1.0, use Q14
        q14_ac = []
        for v in accent_c:
            q = int(round(v * 16384.0))
            q14_ac.append(q)
        print(format_array_int32('lut_accent_gain_coarse_q14', q14_ac))
        print()
    
    accent_f = extract_float_array(content, 'lut_accent_gain_fine')
    if accent_f:
        max_af = max(abs(v) for v in accent_f)
        print(f"// lut_accent_gain_fine: {len(accent_f)} entries, max={max_af:.4f}")
        print(format_array_int16('lut_accent_gain_fine_q15', to_q15(accent_f)))
        print()
    
    # ── Envelope tables ──
    env_inc = extract_float_array(content, 'lut_env_increments')
    if env_inc:
        max_ei = max(abs(v) for v in env_inc)
        print(f"// lut_env_increments: {len(env_inc)} entries, max={max_ei:.6f}")
        # Very small values (0..0.05ish), use Q20 for precision
        q20_ei = []
        for v in env_inc:
            q = int(round(v * (1 << 20)))
            q20_ei.append(q)
        print(format_array_int32('lut_env_increments_q20', q20_ei))
        print()
    
    env_lin = extract_float_array(content, 'lut_env_linear')
    if env_lin:
        print(f"// lut_env_linear: {len(env_lin)} entries (0..1)")
        print(format_array_int16('lut_env_linear_q15', to_q15(env_lin)))
        print()
    
    env_expo = extract_float_array(content, 'lut_env_expo')
    if env_expo:
        print(f"// lut_env_expo: {len(env_expo)} entries (0..1)")
        print(format_array_int16('lut_env_expo_q15', to_q15(env_expo)))
        print()
    
    env_quartic = extract_float_array(content, 'lut_env_quartic')
    if env_quartic:
        print(f"// lut_env_quartic: {len(env_quartic)} entries (0..1)")
        print(format_array_int16('lut_env_quartic_q15', to_q15(env_quartic)))
        print()
    
    # ── MIDI pitch tables ──
    # lut_midi_to_f_high: frequency ratio per semitone. Values range from
    # very small (sub-audio) to ~1500+ (high MIDI notes). Store as Q8
    # (multiply by 256) for the musically useful range, clamp the rest.
    midi_f_high = extract_float_array(content, 'lut_midi_to_f_high')
    if midi_f_high:
        max_fh = max(abs(v) for v in midi_f_high)
        print(f"// lut_midi_to_f_high: {len(midi_f_high)} entries, max={max_fh:.4f}")
        print(f"// Stored as Q8 (÷256). Max representable = {2**31/256:.0f}")
        q8_fh = []
        for v in midi_f_high:
            q = int(round(v * 256.0))
            # Clamp to int32 range
            q = max(-2147483648, min(2147483647, q))
            q8_fh.append(q)
        print(format_array_int32('lut_midi_to_f_high_q8', q8_fh))
        print()
    
    midi_f_low = extract_float_array(content, 'lut_midi_to_f_low')
    if midi_f_low:
        max_fl = max(abs(v) for v in midi_f_low)
        print(f"// lut_midi_to_f_low: {len(midi_f_low)} entries, max={max_fl:.4f}")
        # Fine pitch ratios ~0.94..1.06, fits nicely in Q15
        print(format_array_int16('lut_midi_to_f_low_q15', to_q15(midi_f_low)))
        print()
    
    # lut_midi_to_increment_high: phase increments for uint32 accumulator.
    # These are NOT normalized 0..1 floats — they're already integer-scale
    # values designed for a uint32 phase accumulator (0..2^32 = one cycle).
    # Values range from ~68K (sub-Hz) to ~1.6B (Nyquist cap at 32kHz).
    # Store as uint32_t directly.
    midi_inc_high = extract_float_array(content, 'lut_midi_to_increment_high')
    if midi_inc_high:
        max_ih = max(abs(v) for v in midi_inc_high)
        print(f"// lut_midi_to_increment_high: {len(midi_inc_high)} entries, max={max_ih:.0f}")
        print(f"// Stored as uint32_t (raw phase increment for uint32 accumulator)")
        u32_inc = []
        for v in midi_inc_high:
            q = int(round(v))
            q = max(0, min(0xFFFFFFFF, q))
            u32_inc.append(q)
        print(format_array_uint32('lut_midi_to_increment_high_u32', u32_inc))
        print()
    
    # ── SVF shift table ──
    svf_shift = extract_float_array(content, 'lut_svf_shift')
    if svf_shift:
        max_ss = max(abs(v) for v in svf_shift)
        print(f"// lut_svf_shift: {len(svf_shift)} entries, max={max_ss:.4f}")
        print(format_array_int16('lut_svf_shift_q15', to_q15(svf_shift)))
        print()
    
    # ── LED brightness (already int16, just copy) ──
    led = extract_int16_array(content, 'lut_db_led_brightness')
    if led:
        print(f"// lut_db_led_brightness: {len(led)} entries (already int16)")
        print(format_array_int16('lut_db_led_brightness', led))
        print()
    
    # ── LUT pointer tables for envelope shapes ──
    print("// Envelope shape LUT pointers")
    print("const int16_t* const lut_env_shapes_q15[] = {")
    print("    lut_env_linear_q15,")
    print("    lut_env_expo_q15,")
    print("    lut_env_quartic_q15,")
    print("};")
    print()
    
    # Print stats
    total_q15 = 0
    total_q32 = 0
    stats = []
    for name in ['lut_sine_q15', 'lut_approx_svf_g_q14', 'lut_approx_svf_r_q14', 
                  'lut_approx_svf_h_q15', 'lut_approx_svf_gain_q15',
                  'lut_stiffness_q15', 'lut_accent_gain_fine_q15',
                  'lut_env_linear_q15', 'lut_env_expo_q15', 'lut_env_quartic_q15',
                  'lut_midi_to_f_low_q15', 'lut_svf_shift_q15', 'lut_db_led_brightness']:
        pass  # These are int16
    
    print(f"// Total LUT memory estimate:", file=sys.stderr)
    print(f"//   int16 arrays: ~{len(sine) + len(svf_g) + len(svf_r) + len(svf_h) + len(svf_gain) + len(stiffness) + len(accent_f) + len(env_lin) + len(env_expo) + len(env_quartic) + len(midi_f_low) + len(svf_shift) + len(led)} entries × 2 bytes", file=sys.stderr)
    print(f"//   int32 arrays: ~{len(decades) + len(accent_c) + len(env_inc) + len(midi_f_high) + len(midi_inc_high)} entries × 4 bytes", file=sys.stderr)
    
    n16 = len(sine) + len(svf_g) + len(svf_r) + len(svf_h) + len(svf_gain) + len(stiffness) + len(accent_f) + len(env_lin) + len(env_expo) + len(env_quartic) + len(midi_f_low) + len(svf_shift) + len(led)
    n32 = len(decades) + len(accent_c) + len(env_inc) + len(midi_f_high) + len(midi_inc_high)
    total = n16 * 2 + n32 * 4
    print(f"//   Total: {total} bytes ({total/1024:.1f} KB)", file=sys.stderr)

if __name__ == '__main__':
    main()
