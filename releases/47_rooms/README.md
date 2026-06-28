# Rooms — Stereo Binaural Room Simulator

**Rooms** places a sound source in a virtual room, providing control over rotation (azimuth), distance, and binaural width (spread). It includes a multi-tap room boundary delay simulator (bouncing reflections) and a late-field Dattorro plate reverb, creating a realistic, immersive audio experience (Dolby Atmos/VR style) optimized for headphones.

---

## 🎛 Controls

The module’s behavior is divided into **Positioning Modes** (Switch Up or Middle) and **Room Setup Mode** (Switch Down).

### Positioning Modes (Switch Up or Switch Middle)
*   **Switch Up**: Spatializes **Audio Input 1** (Left/Mono).
*   **Switch Middle**: Spatializes **Audio Input 2** (Right/Mono).
*   In both positions, the knobs control where the sound source sits in the virtual room:
    *   **Main Knob**: **Rotation (Azimuth)**. Rotates the sound smoothly around the listener's head ($0^\circ$ to $360^\circ$).
    *   **X Knob**: **Distance**. Moves the sound from close (dry, clear) to far (quiet, low-passed, receding into the reverb tail).
    *   **Y Knob**: **Binaural Width**. Controls the spatial width of the source, spreading it from a tight pinpoint point-source (CCW) to a wide, immersive spatial cloud (CW).

### Room Setup Mode (Switch Down)
*   Keeps processing the last selected audio input (from Switch Up or Middle).
*   The knobs adjust the dimensions and dynamics of the room:
    *   **Main Knob**: **Reverb Size**. Adjusts the decay time and mix level of the late-field plate reverb.
    *   **X Knob**: **Delay Time**. Sets the delay time (reflection spacing / room size) from 20ms to 400ms. If turned fully counter-clockwise (X < 500), the delay reflections are completely bypassed/off.
    *   **Y Knob**: **Feedback (Bounces)**. Adjusts how much the sound echoes and bounces off the virtual walls.

*Note: Turning a knob after changing switch modes locks the control to prevent sudden jumps. Rotate the knob to match its previous value to unlock it.*

---

## 🎚 Jacks

### Inputs
*   **Audio In 1**: Left / Mono input source (processed when Switch is Up).
*   **Audio In 2**: Right / Mono input source (processed when Switch is Middle).
*   **CV In 1 (Audio 1 Azimuth Mod)**: Modulates the rotation angle of Audio In 1.
*   **CV In 2 (Audio 2 Azimuth Mod)**: Modulates the rotation angle of Audio In 2.
*   **Pulse In 1 (Audio 1 Randomize)**: Rising-edge trigger randomizes the position (azimuth, distance, width) of Audio In 1.
*   **Pulse In 2 (Audio 2 Randomize)**: Rising-edge trigger randomizes the position (azimuth, distance, width) of Audio In 2.

### Outputs
*   **Binaural Out Left & Right**: Main processed stereo output. **Headphones are highly recommended** for the full 3D binaural effect.
*   **CV Out 1 (Azimuth)**: Outputs a CV representation of the source's current azimuth (-2048 to +2047).
*   **CV Out 2 (Distance)**: Outputs a CV representation of the source's current distance (-2048 to +2047).
*   **Pulse Out 1 (Zero-Cross)**: Pulses high briefly when the azimuth rotates past $0^\circ$ (directly in front).
*   **Pulse Out 2**: (Unused in this version).

---

## 💡 LED Interface

The 6 LEDs arranged in a 2x3 grid represent a spatial map around your head:

```
[ LED 0 ] Front-Left    [ LED 1 ] Front-Right
[ LED 2 ] Left          [ LED 3 ] Right
[ LED 4 ] Back-Left     [ LED 5 ] Back-Right
```

*   **Spatial Visualizer**: When playing, the LEDs glow and fade to show the 3D position of the sound source. A sound directly on your left will light up LED 2; a sound orbiting your head will make the light circle around the grid. As the source moves far away, the lights dim.
*   **Parameter Feedback**: In **Switch Down** mode, adjusting a knob temporarily displays a bar graph of that parameter's value across the grid for 1.5 seconds before returning to the spatial visualizer.
