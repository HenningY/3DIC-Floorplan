// Webview script: talk to extension, parse .out/.block/.nets, draw multi-die floorplan,
// allow moving modules and recompute HPWL in real time.

/* global acquireVsCodeApi */

const vscode = acquireVsCodeApi();

let canvas;
let ctx;

let layoutData = null; // parsed combined data
let scale = 0.4;
const padding = 40;

let draggingModule = null;
// Last mouse position in canvas pixel coordinates (for delta-based dragging)
let lastMousePos = { x: 0, y: 0 };

window.addEventListener('DOMContentLoaded', () => {
  canvas = document.getElementById('floorplanCanvas');
  ctx = canvas.getContext('2d');

  const runBtn = document.getElementById('run-fp-btn');
  const alphaInput = document.getElementById('alpha-input');
  const blockInput = document.getElementById('block-path');
  const netInput = document.getElementById('net-path');

  runBtn.addEventListener('click', () => {
    const alpha = parseFloat(alphaInput.value) || 0.5;
    const blockPath = blockInput.value.trim();
    const netPath = netInput.value.trim();
    vscode.postMessage({
      type: 'runFp',
      alpha,
      blockPath,
      netPath,
    });
  });

  setupCanvasInteractions();
  setCanvasMessage('Set input files, then click "Run ./bin/fp & Visualize".');
});

window.addEventListener('message', (event) => {
  const msg = event.data;
  switch (msg.type) {
    case 'status':
      setStatus(msg.text || '');
      break;
    case 'data':
      handleData(msg);
      break;
    default:
      break;
  }
});

function setStatus(text) {
  const el = document.getElementById('status-text');
  if (el) el.textContent = text;
}

function setCanvasMessage(message) {
  canvas.width = 800;
  canvas.height = 520;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#d4d4d4';
  ctx.font = '14px sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText(message, canvas.width / 2, canvas.height / 2);
  ctx.textAlign = 'start';
}

function handleData(msg) {
  try {
    const { outText, blockText, netText, meta } = msg;
    const outParsed = parseOutText(outText);
    const blockParsed = parseBlockText(blockText);
    const netParsed = parseNetText(netText);

    layoutData = buildLayoutData(outParsed, blockParsed, netParsed);

    updateInfo(meta, outParsed.hpwl);
    redraw();
  } catch (err) {
    console.error(err);
    setCanvasMessage('Failed to parse data, see devtools console.');
    setStatus(String(err));
  }
}

function updateInfo(meta, hpwlOriginal) {
  const hpwlEl = document.getElementById('hpwl-text');
  const hpwlOrigEl = document.getElementById('hpwl-original-text');
  const dieInfoEl = document.getElementById('die-info');

  if (!layoutData) return;

  const currentHpwl = computeTotalHPWL(layoutData);

  if (hpwlEl) {
    hpwlEl.textContent = `HPWL (current): ${currentHpwl.toFixed(2)}`;
  }
  if (hpwlOrigEl) {
    hpwlOrigEl.textContent = `Original HPWL (from .out): ${hpwlOriginal.toFixed(
      2,
    )}`;
  }
  if (dieInfoEl) {
    const dieIds = Object.keys(layoutData.dies).map((d) => parseInt(d, 10));
    dieInfoEl.textContent = `Dies: ${dieIds
      .sort((a, b) => a - b)
      .join(', ')}  |  Blocks: ${layoutData.modules.length}  |  Nets: ${
      layoutData.nets.length
    }`;
  }
}

// ---------- Parsing ----------

function parseOutText(text) {
  const lines = text
    .split(/\r?\n/)
    .map((l) => l.trim())
    .filter((l) => l.length > 0);
  if (lines.length < 6) {
    throw new Error('Out file too short.');
  }

  const cost = parseFloat(lines[0]);
  const hpwl = parseFloat(lines[1]);
  const area = parseFloat(lines[2]);
  const wh = lines[3].split(/\s+/).map(Number);
  const width = wh[0];
  const height = wh[1];
  const runtime = parseFloat(lines[4]);

  const modules = [];

  for (let i = 5; i < lines.length; i++) {
    const parts = lines[i].split(/\s+/);
    if (parts.length < 5) continue;

    const name = parts[0];
    let dieId = 0;
    let idx = 1;
    if (parts.length === 6) {
      dieId = parseInt(parts[1], 10);
      idx = 2;
    }
    const x1 = parseFloat(parts[idx]);
    const y1 = parseFloat(parts[idx + 1]);
    const x2 = parseFloat(parts[idx + 2]);
    const y2 = parseFloat(parts[idx + 3]);

    modules.push({
      name,
      dieId,
      x: x1,
      y: y1,
      width: x2 - x1,
      height: y2 - y1,
    });
  }

  return { cost, hpwl, area, width, height, runtime, modules };
}

function parseBlockText(text) {
  const lines = text.split(/\r?\n/);
  let i = 0;

  function nextNonEmpty() {
    while (i < lines.length) {
      const line = lines[i].trim();
      i++;
      if (line.length > 0) return line;
    }
    return null;
  }

  let header = nextNonEmpty();
  if (!header) throw new Error('Empty block file.');

  let numDies = 1;
  let outlines = [];

  if (header.startsWith('NumDie:')) {
    const parts = header.split(/\s+/);
    numDies = parseInt(parts[1], 10);
    for (let d = 0; d < numDies; d++) {
      const line = nextNonEmpty();
      const ps = line.split(/\s+/);
      outlines.push({ width: parseInt(ps[1], 10), height: parseInt(ps[2], 10) });
    }
  } else if (header.startsWith('Outline:')) {
    const ps = header.split(/\s+/);
    outlines.push({ width: parseInt(ps[1], 10), height: parseInt(ps[2], 10) });
  }

  const numBlocksLine = nextNonEmpty();
  const nb = parseInt(numBlocksLine.split(/\s+/)[1], 10);

  const numTermsLine = nextNonEmpty();
  const nt = parseInt(numTermsLine.split(/\s+/)[1], 10);

  const blocks = [];
  for (let b = 0; b < nb; b++) {
    const line = nextNonEmpty();
    if (!line) break;
    const ps = line.split(/\s+/);
    const name = ps[0];
    const w = parseInt(ps[1], 10);
    const h = parseInt(ps[2], 10);
    let dieId = 0;
    if (ps.length >= 4) {
      dieId = parseInt(ps[3], 10);
    }
    blocks.push({ name, width: w, height: h, dieId });
  }

  const terminals = [];
  for (let t = 0; t < nt; t++) {
    const line = nextNonEmpty();
    if (!line) break;
    const ps = line.split(/\s+/);
    const name = ps[0];
    const x = parseFloat(ps[2]);
    const y = parseFloat(ps[3]);
    terminals.push({ name, x, y });
  }

  return { numDies, outlines, blocks, terminals };
}

function parseNetText(text) {
  const lines = text.split(/\r?\n/);
  let i = 0;

  function nextNonEmpty() {
    while (i < lines.length) {
      const line = lines[i].trim();
      i++;
      if (line.length > 0) return line;
    }
    return null;
  }

  const first = nextNonEmpty();
  if (!first || !first.startsWith('NumNets:')) {
    throw new Error('Invalid net file header.');
  }
  const numNets = parseInt(first.split(/\s+/)[1], 10);

  const nets = [];
  for (let n = 0; n < numNets; n++) {
    const degLine = nextNonEmpty();
    if (!degLine) break;
    const deg = parseInt(degLine.split(/\s+/)[1], 10);
    const nodes = [];
    for (let d = 0; d < deg; d++) {
      const nodeLine = nextNonEmpty();
      if (!nodeLine) break;
      nodes.push(nodeLine.trim());
    }
    nets.push({ nodes });
  }

  return { numNets, nets };
}

function buildLayoutData(outParsed, blockParsed, netParsed) {
  const dies = {};
  outParsed.modules.forEach((m) => {
    if (!dies[m.dieId]) {
      dies[m.dieId] = { modules: [] };
    }
    dies[m.dieId].modules.push({ ...m });
  });

  const modules = outParsed.modules.map((m) => ({ ...m }));

  const moduleByName = new Map();
  modules.forEach((m) => {
    moduleByName.set(m.name, m);
  });

  const terminalByName = new Map();
  blockParsed.terminals.forEach((t) => {
    terminalByName.set(t.name, { ...t });
  });

  const nets = netParsed.nets.map((n) => ({
    nodes: n.nodes.slice(),
  }));

  return {
    dies,
    modules,
    moduleByName,
    terminalByName,
    nets,
  };
}

// ---------- HPWL ----------

function computeTotalHPWL(data) {
  let total = 0;
  for (const net of data.nets) {
    let minX = Number.POSITIVE_INFINITY;
    let maxX = Number.NEGATIVE_INFINITY;
    let minY = Number.POSITIVE_INFINITY;
    let maxY = Number.NEGATIVE_INFINITY;
    let count = 0;

    for (const name of net.nodes) {
      let x;
      let y;
      const mod = data.moduleByName.get(name);
      if (mod) {
        x = mod.x + mod.width / 2;
        y = mod.y + mod.height / 2;
      } else {
        const term = data.terminalByName.get(name);
        if (!term) continue;
        x = term.x;
        y = term.y;
      }
      minX = Math.min(minX, x);
      maxX = Math.max(maxX, x);
      minY = Math.min(minY, y);
      maxY = Math.max(maxY, y);
      count++;
    }

    if (count >= 2) {
      total += maxX - minX + (maxY - minY);
    }
  }
  return total;
}

// ---------- Drawing & interaction ----------

function setupCanvasInteractions() {
  let isDragging = false;

  canvas.addEventListener('mousedown', (e) => {
    if (!layoutData) return;
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;

    const hit = hitTestModuleScreen(cx, cy);
    if (hit) {
      isDragging = true;
      draggingModule = hit;
      lastMousePos.x = cx;
      lastMousePos.y = cy;
    }
  });

  canvas.addEventListener('mousemove', (e) => {
    if (!isDragging || !draggingModule) return;

    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;

    const dxPixels = cx - lastMousePos.x;
    const dyPixels = cy - lastMousePos.y;

    // Convert pixel delta to layout units (y-axis inverted)
    draggingModule.x += dxPixels / scale;
    draggingModule.y -= dyPixels / scale;

    lastMousePos.x = cx;
    lastMousePos.y = cy;

    const hpwlNow = computeTotalHPWL(layoutData);
    const hpwlEl = document.getElementById('hpwl-text');
    if (hpwlEl) {
      hpwlEl.textContent = `HPWL (current): ${hpwlNow.toFixed(2)}`;
    }
    redraw();
  });

  window.addEventListener('mouseup', () => {
    isDragging = false;
    draggingModule = null;
  });
}

function getLayoutBounds() {
  if (!layoutData || layoutData.modules.length === 0) {
    return {
      minX: 0,
      minY: 0,
      maxX: 1,
      maxY: 1,
    };
  }
  let minX = Number.POSITIVE_INFINITY;
  let minY = Number.POSITIVE_INFINITY;
  let maxX = Number.NEGATIVE_INFINITY;
  let maxY = Number.NEGATIVE_INFINITY;

  for (const m of layoutData.modules) {
    minX = Math.min(minX, m.x);
    minY = Math.min(minY, m.y);
    maxX = Math.max(maxX, m.x + m.width);
    maxY = Math.max(maxY, m.y + m.height);
  }

  return { minX, minY, maxX, maxY };
}

function redraw() {
  if (!layoutData) {
    setCanvasMessage('No data.');
    return;
  }

  const bounds = getLayoutBounds();
  const width = (bounds.maxX - bounds.minX) * scale + 2 * padding;
  const height = (bounds.maxY - bounds.minY) * scale + 2 * padding;

  canvas.width = Math.max(400, width);
  canvas.height = Math.max(300, height);

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const dieColors = {};
  const dieIds = Object.keys(layoutData.dies)
    .map((d) => parseInt(d, 10))
    .sort((a, b) => a - b);
  dieIds.forEach((id, idx) => {
    const hue = (idx * 137) % 360;
    dieColors[id] = {
      fill: `hsla(${hue}, 70%, 70%, 0.4)`,
      stroke: `hsla(${hue}, 80%, 35%, 1.0)`,
    };
  });

  for (const m of layoutData.modules) {
    const color = dieColors[m.dieId] || {
      fill: 'rgba(180,180,180,0.4)',
      stroke: '#ffffff',
    };

    const x = padding + (m.x - bounds.minX) * scale;
    const y =
      canvas.height -
      padding -
      (m.y - bounds.minY + m.height + m.dieId * 5) * scale;
    const w = m.width * scale;
    const h = m.height * scale;

    ctx.fillStyle = color.fill;
    ctx.fillRect(x, y, w, h);

    ctx.strokeStyle = color.stroke;
    ctx.lineWidth = 1;
    ctx.strokeRect(x, y, w, h);

    ctx.fillStyle = '#ffffff';
    ctx.font = '10px sans-serif';
    ctx.fillText(`${m.name} (D${m.dieId})`, x + 3, y + 12);
  }
}

function getLayoutCoordsFromEvent(e) {
  const rect = canvas.getBoundingClientRect();
  const cx = e.clientX - rect.left;
  const cy = e.clientY - rect.top;
  const bounds = getLayoutBounds();

  const lx = (cx - padding) / scale + bounds.minX;
  const ly =
    (canvas.height - padding - cy) / scale + bounds.minY; // approximate
  return { x: lx, y: ly };
}

function hitTestModule(x, y) {
  if (!layoutData) return null;
  for (let i = layoutData.modules.length - 1; i >= 0; i--) {
    const m = layoutData.modules[i];
    if (x >= m.x && x <= m.x + m.width && y >= m.y && y <= m.y + m.height) {
      return m;
    }
  }
  return null;
}

// Hit-test in SCREEN coordinates (pixels), using the same mapping as redraw().
function hitTestModuleScreen(cx, cy) {
  if (!layoutData) return null;

  const bounds = getLayoutBounds();
  const dieIds = Object.keys(layoutData.dies)
    .map((d) => parseInt(d, 10))
    .sort((a, b) => a - b);

  // Iterate from top-most drawn module to bottom (reverse order).
  for (let i = layoutData.modules.length - 1; i >= 0; i--) {
    const m = layoutData.modules[i];

    const x = padding + (m.x - bounds.minX) * scale;
    const y =
      canvas.height -
      padding -
      (m.y - bounds.minY + m.height + m.dieId * 5) * scale;
    const w = m.width * scale;
    const h = m.height * scale;

    if (cx >= x && cx <= x + w && cy >= y && cy <= y + h) {
      return m;
    }
  }

  return null;
}

