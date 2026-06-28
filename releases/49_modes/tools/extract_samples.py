#!/usr/bin/env python3
"""
Extract smp_sample_data and smp_noise_sample from resources.cc.
These stay in flash (XIP) via __in_flash() attribute — zero RAM cost.

Usage: python3 extract_samples.py > ../samples_flash.cpp
"""

import re
import sys
import os

RESOURCES_CC = os.path.join(os.path.dirname(__file__), 
    '..', 'lib', 'mi-UGens-main', 'eurorack', 'elements', 'resources.cc')

def extract_raw_array(content, type_decl, name):
    """Extract raw array declaration including all values."""
    pattern = rf'{re.escape(type_decl)} {name}\[\] = \{{(.*?)\}};'
    m = re.search(pattern, content, re.DOTALL)
    if not m:
        print(f"WARNING: Could not find {name}", file=sys.stderr)
        return None
    return m.group(1)

def main():
    with open(RESOURCES_CC, 'r') as f:
        content = f.read()
    
    print("// =============================================================================")
    print("// samples_flash.cpp — Exciter sample data, stored in flash (XIP)")
    print("//")
    print("// Extracted from MI Elements resources.cc")
    print("// Uses __in_flash() to keep data in flash even with copy_to_ram binary type.")
    print("// Total: ~330 KB flash, 0 bytes RAM.")
    print("//")
    print("// Original code: Émilie Gillet (Mutable Instruments) — MIT License")
    print("// =============================================================================")
    print()
    print('#include "samples_flash.h"')
    print('#include "pico/platform.h"')
    print()
    
    # Extract smp_sample_data
    data = extract_raw_array(content, 'const int16_t', 'smp_sample_data')
    if data:
        # Count entries
        entries = len(re.findall(r'[-+]?\d+', data))
        print(f"// smp_sample_data: {entries} entries ({entries * 2} bytes)")
        print(f'const int16_t __in_flash("samples") smp_sample_data[] = {{{data}}};')
        print()
        print(f"// Sample data size: {entries}", file=sys.stderr)
    
    # Extract smp_noise_sample
    noise = extract_raw_array(content, 'const int16_t', 'smp_noise_sample')
    if noise:
        entries_n = len(re.findall(r'[-+]?\d+', noise))
        print(f"// smp_noise_sample: {entries_n} entries ({entries_n * 2} bytes)")
        print(f'const int16_t __in_flash("samples") smp_noise_sample[] = {{{noise}}};')
        print()
        print(f"// Noise sample size: {entries_n}", file=sys.stderr)
    
    # Extract boundaries
    bounds = extract_raw_array(content, 'const size_t', 'smp_boundaries')
    if bounds:
        print(f"// Sample segment boundaries")
        print(f'const uint32_t smp_boundaries[] = {{{bounds}}};')
        print()

if __name__ == '__main__':
    main()
