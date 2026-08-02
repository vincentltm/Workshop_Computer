### Global Overview & Signal Routing

Bends is a 6-stage stereo multi-effects processor, live stutter looper, and signal degradation engine designed for the Workshop Computer.

The module makes digital imperfection its core concept. Inspired by 90s circuit-bent rack units, Bends exposes the artifacts, memory corruption, and data bus shorting of early digital processors. It functions both as a traditional stereo multi-effects unit and as a self-contained generative sound source.

Bends spans a wide range of sound processing:
* **Clean stereo utility**: BBD chorus, pitch-glide delays, state-variable filters, and spring or hall reverbs.
* **Glitch textures**: Granular freeze pendulums, clock-synced breakbeat slicing, and shimmer pitch-shifting.
* **Digital corruption**: Wavefolder fuzz, bitwise BBD XOR data bus shorting, lossy MP3 compression, network packet dropouts, and CD sputter.

#### Switch Controls & Navigation
- **Tap DOWN (<350ms)**: Step forward through parameter pages (`1. Chorus` -> `2. Loss` -> `3. Delay` -> `4. Glitcher` -> `5. Filter` -> `6. Reverb`).
- **Hold DOWN (>=350ms)**: Enter **Global Mode** to adjust global bend, stereo width, and routing presets.
- **Hold DOWN (3s)**: Save current parameters and routing setup to persistent flash memory (600ms strobe confirmation).
- **Flick UP**: Toggle live buffer **Freeze Mode**.

#### Global Controls (Global Mode)
- **Main (Bend)**: Sweeps background noise, vinyl crackle, bitcrush fuzz, and packet dropouts across all 6 effect stages simultaneously.
- **X (Width & Mode)**: Extended Mono Mode (CCW limit <2%), Stereo Width (0% to 100%), or Dual Mono Mode (CW limit >95%).
- **Y (Routing & Reset)**: Selects 1 of 4 signal routing chains (`0: Series FX`, `1: Space Wash`, `2: Granular Cloud`, `3: Filtered Dub`). Turn fully CW (>96%) to Clean Reset.

#### Mode LED Indicators
- **LEDs 0 to 3**: Active signal routing chain (0 to 3).
- **LED 4**: Dual Mono Mode active.
- **LED 5**: Extended Mono Mode active.

<details class="program-card-section program-card-collapsible program-card-io-section" open>
  <summary class="program-card-io-summary">
    <h3 class="program-card-io-mobile-heading">Inputs &amp; Outputs</h3>
    <span class="program-card-io-headings">
      <h3>Inputs</h3>
      <h3>Outputs</h3>
    </span>
  </summary>
  <div class="program-card-io-columns">
    <section class="program-card-socket-section program-card-socket-section--inputs">
      <div class="program-card-socket-list">
        <div class="program-card-socket">
          <strong class="program-card-component-key">Audio In 1</strong>
          <p><span class="program-card-component-role">Audio 1</span><br>Stereo left audio input (normalled to Right if Audio 2 is unplugged)</p>
        </div>
        <div class="program-card-socket">
          <strong class="program-card-component-key">Audio In 2</strong>
          <p><span class="program-card-component-role">Audio 2</span><br>Stereo right audio input (unplug for ~2.73s Extended Mono Mode)</p>
        </div>
        <div class="program-card-socket">
          <strong class="program-card-component-key">CV In 1</strong>
          <p><span class="program-card-component-role">CV 1</span><br>Bipolar CV input modulating primary page parameter or tuned 1V/Oct pitch</p>
        </div>
        <div class="program-card-socket">
          <strong class="program-card-component-key">CV In 2</strong>
          <p><span class="program-card-component-role">CV 2</span><br>Bipolar CV input modulating secondary page parameter and glitch/reverb depth</p>
        </div>
        <div class="program-card-socket">
          <strong class="program-card-component-key">Pulse In 1</strong>
          <p><span class="program-card-component-role">Clock Sync</span><br>External clock pulse sync (1, 2, 4, and 24 PPQN DIN Sync) for delay subdivisions, loop stepping, and pitch CV</p>
        </div>
        <div class="program-card-socket">
          <strong class="program-card-component-key">Pulse In 2</strong>
          <p><span class="program-card-component-role">Freeze Gate</span><br>Gate input (&gt; +1.2V) locking delay &amp; glitch looper buffers into live freeze</p>
        </div>
      </div>
    </section>

    <section class="program-card-socket-section program-card-socket-section--outputs">
      <div class="program-card-socket-list">
        <div class="program-card-socket">
          <strong class="program-card-component-key">Audio Out 1</strong>
          <p><span class="program-card-component-role">Out 1</span><br>Main processed stereo left audio output</p>
        </div>
        <div class="program-card-socket">
          <strong class="program-card-component-key">Audio Out 2</strong>
          <p><span class="program-card-component-role">Out 2</span><br>Main processed stereo right audio output</p>
        </div>
        <div class="program-card-socket">
          <strong class="program-card-component-key">CV Out 1</strong>
          <p><span class="program-card-component-role">Pitch CV</span><br>Turing Machine 1V/Oct quantized semitone CV sequence output</p>
        </div>
        <div class="program-card-socket">
          <strong class="program-card-component-key">CV Out 2</strong>
          <p><span class="program-card-component-role">Random CV</span><br>Stepped random Sample &amp; Hold CV output for self-modulation</p>
        </div>
        <div class="program-card-socket">
          <strong class="program-card-component-key">Pulse Out 1</strong>
          <p><span class="program-card-component-role">Loop Trig</span><br>+5V 2ms trigger pulse output emitted on granular loop resets or clock steps</p>
        </div>
        <div class="program-card-socket">
          <strong class="program-card-component-key">Pulse Out 2</strong>
          <p><span class="program-card-component-role">Texture</span><br>High-frequency PWM audio stream output of microsound bleeps and vinyl crackle</p>
        </div>
      </div>
    </section>
  </div>
</details>

#### Reset Procedures
- **Preset Clean Reset**: Turn Knob Y fully CW (>96%) in Global Mode to restore all 6 parameter pages to clean factory defaults.
- **Factory Hardware Reset**: Power on system while holding Z Switch DOWN (1s LED flash) to restore original factory settings from flash memory.
