import wave
import os
import struct

TARGET_RATE = 12000
DURATION_SEC = 4
TARGET_LEN = TARGET_RATE * DURATION_SEC # 48000 samples

def read_wav(filename):
    if not os.path.exists(filename):
        print(f"Error: {filename} does not exist.")
        return None, 0
        
    with wave.open(filename, 'rb') as w:
        nch = w.getnchannels()
        width = w.getsampwidth()
        rate = w.getframerate()
        nframes = w.getnframes()
        frames = w.readframes(nframes)
        
        # Parse bytes to float range [-1.0, 1.0]
        samples = []
        if width == 2:
            fmt = f"<{len(frames)//2}h"
            raw = struct.unpack(fmt, frames)
            if nch == 2:
                samples = [(raw[i] + raw[i+1]) / 65536.0 for i in range(0, len(raw), 2)]
            else:
                samples = [x / 32768.0 for x in raw]
        elif width == 3:
            num_samples = len(frames) // 3
            raw = []
            for i in range(num_samples):
                b = frames[i*3 : i*3+3]
                val = b[0] | (b[1] << 8) | (b[2] << 16)
                if val & 0x800000:
                    val -= 0x1000000
                raw.append(val / 8388608.0)
            if nch == 2:
                samples = [(raw[i] + raw[i+1]) / 2.0 for i in range(0, len(raw), 2)]
            else:
                samples = raw
        elif width == 1:
            raw = [x - 128 for x in frames]
            if nch == 2:
                samples = [(raw[i] + raw[i+1]) / 256.0 for i in range(0, len(raw), 2)]
            else:
                samples = [x / 128.0 for x in raw]
        else:
            print(f"Error: Unsupported sample width {width} in {filename}")
            return None, 0
            
        return samples, rate

def resample(samples, orig_rate, target_rate=TARGET_RATE):
    if orig_rate == target_rate:
        return samples
    ratio = orig_rate / target_rate
    target_len = int(len(samples) / ratio)
    resampled = []
    for i in range(target_len):
        pos = i * ratio
        idx = int(pos)
        frac = pos - idx
        if idx + 1 < len(samples):
            val = samples[idx] + frac * (samples[idx+1] - samples[idx])
        else:
            val = samples[idx]
        resampled.append(val)
    return resampled

def process_single_file(filename):
    print(f"Processing {os.path.basename(filename)}...")
    samples, rate = read_wav(filename)
    if samples is None:
        return None
    resampled = resample(samples, rate)
    
    # Pad or crop to TARGET_LEN
    if len(resampled) > TARGET_LEN:
        resampled = resampled[:TARGET_LEN]
    else:
        resampled += [0.0] * (TARGET_LEN - len(resampled))
        
    # Apply loop fade-in and fade-out to prevent clicks
    fade_len = int(TARGET_RATE * 0.02) # 20ms fade = 240 samples
    for i in range(fade_len):
        t = i / fade_len
        resampled[i] *= t
        resampled[-1 - i] *= t

    return resampled

def process_cutlery_collage():
    print("Creating Cutlery Glitch Collage...")
    cutlery_dir = "/Users/vmaurer/Music/Samples/SS_Cutlery_Percussion_Foley_CC0"
    files = [
        "Cutlery_Percussion_01.wav",
        "Cutlery_Percussion_03.wav",
        "Cutlery_Percussion_05.wav",
        "Cutlery_Percussion_07.wav",
        "Cutlery_Percussion_10.wav",
        "Cutlery_Percussion_12.wav",
        "Cutlery_Percussion_15.wav",
        "Cutlery_Percussion_20.wav"
    ]
    
    collage = []
    for f in files:
        path = os.path.join(cutlery_dir, f)
        samples, rate = read_wav(path)
        if samples is not None:
            resampled = resample(samples, rate)
            collage.extend(resampled)
            # Add a small gap of silence (e.g. 50ms) between hits
            collage.extend([0.0] * int(TARGET_RATE * 0.05))
            
    # Pad or crop to TARGET_LEN
    if len(collage) > TARGET_LEN:
        collage = collage[:TARGET_LEN]
    else:
        collage += [0.0] * (TARGET_LEN - len(collage))
        
    # Apply loop fade-in and fade-out to prevent clicks
    fade_len = int(TARGET_RATE * 0.02)
    for i in range(fade_len):
        t = i / fade_len
        collage[i] *= t
        collage[-1 - i] *= t

    return collage

def main():
    samples_sources = [
        # 0: Burial Pad
        "/Users/vmaurer/Music/Samples/SS_Burial_Pads_CC0/Burial_Pad_Long_01.wav",
        # 1: Strings
        "/Users/vmaurer/Music/Samples/Loops Of Ambience/Strings/Amient String Loop 01 77BPM.wav",
        # 2: Guitars
        "/Users/vmaurer/Music/Samples/selection/Ambient Guitars Loop 6 90BPM.wav"
    ]
    
    processed_samples = []
    for path in samples_sources:
        data = process_single_file(path)
        if data is None:
            print("Failed to process all files. Aborting.")
            return
        processed_samples.append(data)
        
    # 3: Cutlery Collage
    collage = process_cutlery_collage()
    processed_samples.append(collage)
    
    # Normalize and convert to int16
    int16_samples = []
    for i, data in enumerate(processed_samples):
        # find max amp
        max_amp = max(abs(x) for x in data)
        if max_amp > 1e-4:
            scale = 28000 / max_amp
            normalized = [int(x * scale) for x in data]
        else:
            normalized = [0] * len(data)
        int16_samples.append(normalized)
        
    # Write to samples.h
    header_path = "/Users/vmaurer/Music/Workshop_Computer/releases/45_bends/samples.h"
    with open(header_path, "w") as f:
        f.write("// ============================================================================\n")
        f.write("// samples.h - Packed Real Audio Samples for 45_bends Ambient Engine\n")
        f.write("// Generated automatically by generate_samples.py\n")
        f.write("// 12kHz, 16-bit mono PCM. Each sample is exactly 4 seconds (48,000 samples).\n")
        f.write("// ============================================================================\n\n")
        f.write("#ifndef SAMPLES_H\n")
        f.write("#define SAMPLES_H\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write("#define AMBIENT_SAMPLE_RATE 12000\n")
        f.write("#define AMBIENT_SAMPLE_LEN 48000\n")
        f.write("#define NUM_AMBIENT_SAMPLES 4\n\n")
        
        sample_names = ["burial_pad", "ambient_strings", "ambient_guitars", "glitch_clicks"]
        
        for idx, (name, data) in enumerate(zip(sample_names, int16_samples)):
            f.write(f"// Sample {idx}: {name.replace('_', ' ').title()}\n")
            f.write(f"static const int16_t sample_data_{idx}[AMBIENT_SAMPLE_LEN] = {{\n")
            # Write 12 samples per line to keep file clean
            for r in range(0, len(data), 12):
                chunk = data[r:r+12]
                f.write("    " + ", ".join(str(x) for x in chunk) + ",\n")
            f.write("};\n\n")
            
        f.write("// Table of pointers to the sample arrays\n")
        f.write("static const int16_t* const ambient_samples[NUM_AMBIENT_SAMPLES] = {\n")
        for idx in range(len(sample_names)):
            f.write(f"    sample_data_{idx},\n")
        f.write("};\n\n")
        
        f.write("#endif // SAMPLES_H\n")
        
    print(f"Successfully generated {header_path}!")

if __name__ == "__main__":
    main()
