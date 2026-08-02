import fs from 'fs/promises';
import path from 'path';
import { fileURLToPath } from 'url';
import YAML from 'yaml';
import { chromium } from 'playwright';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const bendsDir = path.resolve(__dirname, '..');
const targetDir = path.join(bendsDir, 'panels');
const infoYamlPath = path.join(bendsDir, 'info.yaml');

(async () => {
  console.log('Reading info.yaml...');
  const rawYamlText = await fs.readFile(infoYamlPath, 'utf8');
  const rawObj = YAML.parse(rawYamlText);

  // Panel tab mapping matching manifest.yaml with exact C++ firmware LED representations
  const panelMap = [
    { id: 'page-0-global', file: 'page0_global.svg', mode: 'macro', activePos: 'down' }, // Global page: Global button (down) highlighted yellow
    { id: 'page-1-chorus', file: 'page1_chorus.svg', mode: 'normal', activeLed: 1, activePos: 'middle' }, // Perform (middle) highlighted yellow
    { id: 'page-2-codec', file: 'page2_codec.svg', mode: 'normal', activeLed: 2, activePos: 'middle' },
    { id: 'page-3-delay', file: 'page3_delay.svg', mode: 'normal', activeLed: 3, activePos: 'middle' },
    { id: 'page-4-glitcher', file: 'page4_glitcher.svg', mode: 'normal', activeLed: 4, activePos: 'middle' },
    { id: 'page-4-freeze', file: 'page4_freeze.svg', mode: 'freeze', activePos: 'up' },   // Freeze page: Freeze button (up) highlighted yellow
    { id: 'page-5-filter', file: 'page5_filter.svg', mode: 'normal', activeLed: 5, activePos: 'middle' },
    { id: 'page-6-reverb', file: 'page6_reverb.svg', mode: 'normal', activeLed: 6, activePos: 'middle' }
  ];

  // Socket mapping with exact short labels from info.yaml using panelPositions.js keys
  const inputsObj = {
    audio_l: { label: 'Audio 1' },
    audio_r: { label: 'Audio 2' },
    cv_1: { label: 'CV 1' },
    cv_2: { label: 'CV 2' },
    pulse_1: { label: 'Clock Sync' },
    pulse_2: { label: 'Freeze Gate' }
  };

  const outputsObj = {
    audio_out_l: { label: 'Out 1' },
    audio_out_r: { label: 'Out 2' },
    cv_out_1: { label: 'Pitch CV' },
    cv_out_2: { label: 'Random CV' },
    pulse_out_1: { label: 'Loop Trig' },
    pulse_out_2: { label: 'Texture' }
  };

  const switchModesObj = {
    up: 'Freeze',
    middle: 'Perform',
    down: 'Global',
    tap: 'Page'
  };

  const positionControlItems = [
    { id: 'up', name: 'Freeze' },
    { id: 'middle', name: 'Perform' },
    { id: 'down', name: 'Global' }
  ];

  // Extract knob configs per panel tab
  const knobConfigsByPanel = {};
  for (const row of (rawObj.controls?.knobs || [])) {
    const panelId = row.when?.panel;
    if (panelId) {
      knobConfigsByPanel[panelId] = {
        main: row.main ? { label: row.main.name, description: row.main.description } : undefined,
        x: row.x ? { label: row.x.name, description: row.x.description } : undefined,
        y: row.y ? { label: row.y.name, description: row.y.description } : undefined,
      };
    }
  }

  console.log('Launching Playwright Chromium for complete web panel SVGs...');
  const browser = await chromium.launch();
  const page = await browser.newPage();

  console.log('Navigating to local site preview...');
  await page.goto('http://localhost:8080/programs/45-bends/', { waitUntil: 'networkidle' });

  for (const item of panelMap) {
    const knobsForTab = knobConfigsByPanel[item.id] || {};
    
    // Create card snapshot for this tab with specific knobs + ALL 12 jacks + switch modes
    const tabCard = {
      panel: {
        controls: knobsForTab,
        inputs: inputsObj,
        outputs: outputsObj
      },
      switch_modes: switchModesObj
    };

    const positionControl = {
      items: positionControlItems,
      groupId: 'panel-views-export',
      activeId: item.activePos
    };

    // Dynamic import from sitegen
    const { renderPanelArtwork } = await import('../../../tools/sitegen/src/render/cardPage.js');
    const panelHtml = renderPanelArtwork(tabCard, '/assets/program_cards/Standalone_computer_rev1.svg', positionControl);

    console.log(`Generating complete web SVG for ${item.id}...`);

    const svgStr = await page.evaluate(async ({ panelHtml, item, switchModesObj }) => {
      const { renderPanelElementToSvg } = await import('/assets/js/panel-export.js');
      
      const tempWrapper = document.createElement('div');
      tempWrapper.className = 'program-card-use';
      tempWrapper.style.position = 'absolute';
      tempWrapper.style.left = '0px';
      tempWrapper.style.top = '0px';
      tempWrapper.style.width = '280px';
      tempWrapper.style.zIndex = '99999';
      tempWrapper.style.backgroundColor = '#1d1d1f';
      
      // Inject CSS stylesheet link so labels are styled, absolutely positioned over faceplate, and measure pill background rects
      tempWrapper.innerHTML = `
        <link rel="stylesheet" href="/assets/program-cards.css">
        <div class="program-card-use__panel">${panelHtml}</div>
      `;
      
      document.body.appendChild(tempWrapper);

      // Wait for image, stylesheet, and fonts
      const img = tempWrapper.querySelector('img');
      if (img && !img.complete) {
        await new Promise(res => { img.onload = res; img.onerror = res; });
      }
      if (document.fonts) await document.fonts.ready;
      await new Promise(res => setTimeout(res, 50)); // Ensure CSS layout pass completes

      const panelEl = tempWrapper.querySelector('.program-card-panel');
      if (!panelEl) throw new Error('Could not find .program-card-panel element');

      let svg = await renderPanelElementToSvg(panelEl);

      // Set active switch position button rect fill to bright yellow (#f7d738)
      if (item.activePos && switchModesObj[item.activePos]) {
        const roleName = switchModesObj[item.activePos].replace('&', '&amp;');
        const regex = new RegExp(`(<rect\\s+[^>]*fill=")[^"]*("\\s*/><text\\s+[^>]*><tspan\\s+[^>]*>${roleName}</tspan>)`, 'g');
        svg = svg.replace(regex, `$1#f7d738$2`);
      }

      const ledCoords = [
        { cx: '6.7142', cy: '309.2807' },
        { cx: '19.952', cy: '309.309' },
        { cx: '6.6859', cy: '322.2634' },
        { cx: '19.952', cy: '322.235' },
        { cx: '6.7142', cy: '335.1894' },
        { cx: '19.9237', cy: '335.1894' }
      ];

      if (item.mode === 'freeze') {
        const freezeSawCss = `@keyframes ph-saw-1{0%,24.9%{fill:#f80012;opacity:1}25%,100%{fill:#88000a;opacity:.75}}@keyframes ph-saw-2{25%,49.9%{fill:#f80012;opacity:1}0%,49.9%,50%,100%{fill:#88000a;opacity:.75}}@keyframes ph-saw-3{50%,74.9%{fill:#f80012;opacity:1}0%,49.9%,75%,100%{fill:#88000a;opacity:.75}}@keyframes ph-saw-4{75%,99.9%{fill:#f80012;opacity:1}0%,74.9%,100%{fill:#88000a;opacity:.75}}.ph-saw-1{animation:ph-saw-1 1.2s linear infinite}.ph-saw-2{animation:ph-saw-2 1.2s linear infinite}.ph-saw-3{animation:ph-saw-3 1.2s linear infinite}.ph-saw-4{animation:ph-saw-4 1.2s linear infinite}`;
        svg = svg.replace('</style>', `${freezeSawCss}</style>`);

        for (let idx = 1; idx <= 6; idx++) {
          const { cx, cy } = ledCoords[idx - 1];
          const escapedCx = cx.replace('.', '\\.');
          const escapedCy = cy.replace('.', '\\.');
          const regex = new RegExp(`(<circle\\s+cx="${escapedCx}"\\s+cy="${escapedCy}"\\s+r="4.252"\\s+fill=")[^"]*(")`);
          if (idx <= 4) {
            svg = svg.replace(regex, `<circle cx="${cx}" cy="${cy}" r="4.252" class="ph-saw-${idx}" fill="#88000a"`);
          } else {
            svg = svg.replace(regex, `<circle cx="${cx}" cy="${cy}" r="4.252" fill="#1d1d1b"`);
          }
        }
      } else if (item.mode === 'macro') {
        for (let idx = 1; idx <= 6; idx++) {
          const { cx, cy } = ledCoords[idx - 1];
          const color = (idx <= 4) ? '#f80012' : '#1d1d1b';
          const escapedCx = cx.replace('.', '\\.');
          const escapedCy = cy.replace('.', '\\.');
          const regex = new RegExp(`(<circle\\s+cx="${escapedCx}"\\s+cy="${escapedCy}"\\s+r="4.252"\\s+fill=")[^"]*(")`);
          svg = svg.replace(regex, `<circle cx="${cx}" cy="${cy}" r="4.252" fill="${color}"`);
        }
      } else {
        for (let idx = 1; idx <= 6; idx++) {
          const { cx, cy } = ledCoords[idx - 1];
          const color = (idx === item.activeLed) ? '#f80012' : '#1d1d1b';
          const escapedCx = cx.replace('.', '\\.');
          const escapedCy = cy.replace('.', '\\.');
          const regex = new RegExp(`(<circle\\s+cx="${escapedCx}"\\s+cy="${escapedCy}"\\s+r="4.252"\\s+fill=")[^"]*(")`);
          svg = svg.replace(regex, `<circle cx="${cx}" cy="${cy}" r="4.252" fill="${color}"`);
        }
      }

      document.body.removeChild(tempWrapper);
      return svg;
    }, { panelHtml, item, switchModesObj });

    const filePath = path.join(targetDir, item.file);
    await fs.writeFile(filePath, svgStr, 'utf8');
    console.log(`Exported complete SVG with active switch highlight: ${item.file}`);
  }

  await browser.close();
  console.log('All 8 panel SVGs exported cleanly!');
})();
