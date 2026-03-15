// Multi-Die Floorplan Visualizer
// 支援 .out (block 位置) 與 .block (outline + terminals) 兩種檔案同時載入。

let canvas, ctx;
let scale = 0.3;
const PADDING = 50;

// .out 解析結果
// { cost, hpwl, area, width, height, runtime,
//   dies: { [dieId]: { blocks:[{name,x,y,width,height}], bbox:{minX,minY,maxX,maxY} } } }
let outData = null;

// .block 解析結果
// { numDie, outlines:[{w,h}], modules:[{name,w,h,dieId}], terminals:[{name,x,y}] }
let blockData = null;

// 顯示選項
let showOutline    = true;
let showTerminals  = true;
let showTsvStrips  = true;
let showTsvPoints  = true;

// 畫面上 terminal 的像素位置（供 hover 檢測）
let terminalPixels = [];   // [{name, cx, cy, worldX, worldY}]

// 畫面上 TSV assignment 的像素位置（供 hover 檢測）
let tsvPixels = [];        // [{netId, tierLo, tierHi, cx, cy, worldX, worldY, fallback, noSlot}]

// 目前 hover 的元素
let hoveredTerminal = null;
let hoveredTsv      = null;

// TSV tier 顏色
const TSV_TIER_COLORS = [
    { fill: '#fb923c', stroke: '#c2410c' },  // tier 0-1：橙色
    { fill: '#a78bfa', stroke: '#6d28d9' },  // tier 1-2：紫色
    { fill: '#34d399', stroke: '#065f46' },  // tier 2-3：翠綠
    { fill: '#60a5fa', stroke: '#1d4ed8' },  // tier 3-4：藍色
];

// ── 固定色板（die 0~4）─────────────────────────────────────────────────────────
// fill (semi-transparent) / stroke / outline (dashed border)
const DIE_PALETTE = [
    { fill: 'hsla(210,70%,70%,0.45)', stroke: 'hsla(210,80%,32%,1)', outline: '#1d4ed8' },
    { fill: 'hsla( 30,80%,70%,0.45)', stroke: 'hsla( 30,80%,32%,1)', outline: '#b45309' },
    { fill: 'hsla(140,60%,65%,0.45)', stroke: 'hsla(140,70%,28%,1)', outline: '#15803d' },
    { fill: 'hsla(280,65%,70%,0.45)', stroke: 'hsla(280,75%,30%,1)', outline: '#7e22ce' },
    { fill: 'hsla(  0,70%,70%,0.45)', stroke: 'hsla(  0,80%,32%,1)', outline: '#b91c1c' },
];

function getPalette(idx) {
    if (idx < DIE_PALETTE.length) return DIE_PALETTE[idx];
    const hue = (idx * 137) % 360;
    return {
        fill:    `hsla(${hue},70%,70%,0.45)`,
        stroke:  `hsla(${hue},80%,32%,1)`,
        outline: `hsl(${hue},70%,35%)`,
    };
}

// ── 初始化 ─────────────────────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {
    canvas = document.getElementById('floorplanCanvas');
    ctx    = canvas.getContext('2d');

    setupUI();
    setCanvasMessage('Load an .out or .block file to view the floorplan.');
});

// ── UI 設定 ────────────────────────────────────────────────────────────────────
function setupUI() {
    // 使用 label 元素（直接用 for= 綁定 file input，不需 JS 觸發 click）
    const outLabel      = document.getElementById('loadOutLabel');
    const outInput      = document.getElementById('outFileInput');
    const blockLabel    = document.getElementById('loadBlockLabel');
    const blockInput    = document.getElementById('blockFileInput');
    const scaleSlider   = document.getElementById('scale-slider');
    const scaleValue    = document.getElementById('scale-value');
    const dieSelect     = document.getElementById('die-select');
    const outlineCheck  = document.getElementById('showOutlineCheck');
    const terminalCheck = document.getElementById('showTerminalsCheck');
    const tooltip       = document.getElementById('tooltip');

    // ── .out 檔案 ──
    outInput.addEventListener('change', e => {
        const file = e.target.files[0];
        if (!file) return;
        readFile(file, text => {
            try {
                outData = parseOutContent(text);
                outLabel.textContent = `✓ ${file.name}`;
                outLabel.classList.add('loaded');
                updateDieSelect();
                updateInfoPanel();
                updateLegend();
                redraw();
            } catch (err) {
                console.error(err);
                alert('Failed to parse .out file: ' + err.message);
            }
        });
    });

    // ── .block 檔案 ──
    blockInput.addEventListener('change', e => {
        const file = e.target.files[0];
        if (!file) return;
        readFile(file, text => {
            try {
                blockData = parseBlockContent(text);
                blockLabel.textContent = `✓ ${file.name}`;
                blockLabel.classList.add('loaded');
                // 若還沒載入 .out，也能根據 .block 單獨顯示
                if (!outData) updateDieSelectFromBlock();
                updateInfoPanel();
                updateLegend();
                redraw();
            } catch (err) {
                console.error(err);
                alert('Failed to parse .block file: ' + err.message);
            }
        });
    });

    // ── Zoom ──
    scaleSlider.addEventListener('input', () => {
        scale = parseFloat(scaleSlider.value);
        scaleValue.textContent = scale.toFixed(2) + 'x';
        redraw();
    });

    // ── Die 選擇 ──
    dieSelect.addEventListener('change', () => {
        updateInfoPanel();
        redraw();
    });

    // ── 顯示選項 ──
    outlineCheck.addEventListener('change', () => {
        showOutline = outlineCheck.checked; redraw();
    });
    terminalCheck.addEventListener('change', () => {
        showTerminals = terminalCheck.checked; redraw();
    });
    document.getElementById('showTsvStripsCheck').addEventListener('change', e => {
        showTsvStrips = e.target.checked; redraw();
    });
    document.getElementById('showTsvPointsCheck').addEventListener('change', e => {
        showTsvPoints = e.target.checked; redraw();
    });

    // ── Canvas hover（偵測 terminal + TSV point）──
    canvas.addEventListener('mousemove', e => {
        const rect = canvas.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;
        const HIT_R = 8;

        // 優先偵測 TSV 點
        let foundTsv = null;
        if (showTsvPoints) {
            for (const tp of tsvPixels) {
                const dx = mx - tp.cx, dy = my - tp.cy;
                if (dx * dx + dy * dy <= HIT_R * HIT_R) { foundTsv = tp; break; }
            }
        }

        // 再偵測 terminal
        let foundTerm = null;
        if (!foundTsv && showTerminals) {
            for (const tp of terminalPixels) {
                const dx = mx - tp.cx, dy = my - tp.cy;
                if (dx * dx + dy * dy <= HIT_R * HIT_R) { foundTerm = tp; break; }
            }
        }

        const changed = foundTsv !== hoveredTsv || foundTerm !== hoveredTerminal;
        hoveredTsv      = foundTsv;
        hoveredTerminal = foundTerm;
        if (changed) redraw();

        if (foundTsv) {
            const fb = foundTsv.fallback ? '  [fallback]' : '';
            const ns = foundTsv.noSlot   ? '  [no_slot]' : '';
            tooltip.textContent =
                `${foundTsv.netId}  tier${foundTsv.tierLo}-${foundTsv.tierHi}` +
                `  (${foundTsv.worldX.toFixed(1)}, ${foundTsv.worldY.toFixed(1)})${fb}${ns}`;
            tooltip.style.display = 'block';
            tooltip.style.left = (e.clientX + 14) + 'px';
            tooltip.style.top  = (e.clientY - 10) + 'px';
        } else if (foundTerm) {
            tooltip.textContent = `${foundTerm.name}  (${foundTerm.worldX}, ${foundTerm.worldY})`;
            tooltip.style.display = 'block';
            tooltip.style.left = (e.clientX + 14) + 'px';
            tooltip.style.top  = (e.clientY - 10) + 'px';
        } else {
            tooltip.style.display = 'none';
        }
    });

    canvas.addEventListener('mouseleave', () => {
        hoveredTerminal = null;
        hoveredTsv      = null;
        tooltip.style.display = 'none';
        redraw();
    });
}

function readFile(file, cb) {
    const reader = new FileReader();
    reader.onload = () => cb(reader.result);
    reader.readAsText(file);
}

// ── 解析 .out 檔 ───────────────────────────────────────────────────────────────
// 支援新格式三段：blocks / NumTsvStrips ... / NumTsvAssignments ...
function parseOutContent(text) {
    const lines = text.split(/\r?\n/).map(l => l.trim()).filter(l => l.length > 0);
    if (lines.length < 6) throw new Error('Out file too short.');

    const cost    = parseFloat(lines[0]);
    const hpwl    = parseFloat(lines[1]);
    const area    = parseFloat(lines[2]);
    const wh      = lines[3].split(/\s+/).map(Number);
    const width   = wh[0], height = wh[1];
    const runtime = parseFloat(lines[4]);

    const dies           = {};
    const tsvStrips      = [];  // {blockName, dieId, x1, y1, x2, y2}
    const tsvAssignments = [];  // {netId, tierLo, tierHi, x, y, fallback, noSlot}

    let mode = 'blocks';

    for (let i = 5; i < lines.length; i++) {
        const line  = lines[i];
        const parts = line.split(/\s+/);

        // 段落切換
        if (parts[0] === 'NumTsvStrips')      { mode = 'strips';      continue; }
        if (parts[0] === 'NumTsvAssignments') { mode = 'assignments';  continue; }

        if (mode === 'blocks') {
            if (parts.length < 5) continue;
            const name = parts[0];
            let dieId = 0, idx = 1;
            if (parts.length === 6) { dieId = parseInt(parts[1], 10); idx = 2; }
            const x1 = parseFloat(parts[idx]);
            const y1 = parseFloat(parts[idx + 1]);
            const x2 = parseFloat(parts[idx + 2]);
            const y2 = parseFloat(parts[idx + 3]);
            if (!dies[dieId]) {
                dies[dieId] = {
                    blocks: [],
                    bbox: { minX: Infinity, minY: Infinity, maxX: -Infinity, maxY: -Infinity }
                };
            }
            dies[dieId].blocks.push({ name, x: x1, y: y1, width: x2 - x1, height: y2 - y1 });
            const b = dies[dieId].bbox;
            b.minX = Math.min(b.minX, x1); b.minY = Math.min(b.minY, y1);
            b.maxX = Math.max(b.maxX, x2); b.maxY = Math.max(b.maxY, y2);

        } else if (mode === 'strips') {
            // format: blockName dieN x1 y1 x2 y2
            if (parts.length < 6) continue;
            const blockName = parts[0];
            const dieId     = parseInt(parts[1].replace('die', ''), 10);
            const x1 = parseFloat(parts[2]), y1 = parseFloat(parts[3]);
            const x2 = parseFloat(parts[4]), y2 = parseFloat(parts[5]);
            tsvStrips.push({ blockName, dieId, x1, y1, x2, y2 });

        } else if (mode === 'assignments') {
            // format: netN tierX-Y x y [fallback] [no_slot]
            if (parts.length < 4) continue;
            const netId    = parts[0];
            const tierStr  = parts[1];
            const tierM    = tierStr.match(/tier(\d+)-(\d+)/);
            const tierLo   = tierM ? parseInt(tierM[1], 10) : 0;
            const tierHi   = tierM ? parseInt(tierM[2], 10) : 1;
            const x        = parseFloat(parts[2]);
            const y        = parseFloat(parts[3]);
            const fallback = parts.includes('fallback');
            const noSlot   = parts.includes('no_slot');
            tsvAssignments.push({ netId, tierLo, tierHi, x, y, fallback, noSlot });
        }
    }

    return { cost, hpwl, area, width, height, runtime, dies, tsvStrips, tsvAssignments };
}

// ── 解析 .block 檔 ─────────────────────────────────────────────────────────────
function parseBlockContent(text) {
    const lines = text.split(/\r?\n/).map(l => l.trim()).filter(l => l.length > 0);
    const result = {
        numDie:    1,
        outlines:  [],    // [{w, h}]  一個 die 一條 Outline 行
        modules:   [],    // [{name, w, h, dieId}]
        terminals: [],    // [{name, x, y}]
    };

    for (const line of lines) {
        if (line.startsWith('#')) continue;
        const parts = line.split(/\s+/);
        if (parts.length < 2) continue;

        if (parts[0] === 'NumDie:') {
            result.numDie = parseInt(parts[1], 10);
        } else if (parts[0] === 'Outline:') {
            result.outlines.push({ w: parseFloat(parts[1]), h: parseFloat(parts[2]) });
        } else if (parts[0] === 'NumBlocks:' || parts[0] === 'NumTerminals:') {
            // skip counts
        } else if (parts.length >= 4 && parts[1] === 'terminal') {
            // name terminal x y
            result.terminals.push({
                name: parts[0],
                x: parseFloat(parts[2]),
                y: parseFloat(parts[3]),
            });
        } else if (parts.length >= 4 && !isNaN(parts[1]) && !isNaN(parts[2]) && !isNaN(parts[3])) {
            // name w h dieId
            result.modules.push({
                name:  parts[0],
                w:     parseFloat(parts[1]),
                h:     parseFloat(parts[2]),
                dieId: parseInt(parts[3], 10),
            });
        }
    }

    return result;
}

// ── Die 選單 ───────────────────────────────────────────────────────────────────
function updateDieSelect() {
    const sel = document.getElementById('die-select');
    sel.innerHTML = '';
    addOption(sel, 'all', 'All (overlap)');

    if (outData) {
        Object.keys(outData.dies)
            .map(Number).sort((a, b) => a - b)
            .forEach(d => addOption(sel, String(d), `Die ${d}`));
    }
}

function updateDieSelectFromBlock() {
    if (!blockData) return;
    const sel = document.getElementById('die-select');
    sel.innerHTML = '';
    addOption(sel, 'all', 'All (overlap)');
    for (let d = 0; d < blockData.numDie; d++) {
        addOption(sel, String(d), `Die ${d}`);
    }
}

function addOption(select, value, text) {
    const o = document.createElement('option');
    o.value = value; o.textContent = text;
    select.appendChild(o);
}

// ── Legend ─────────────────────────────────────────────────────────────────────
function updateLegend() {
    const legend = document.getElementById('legend');
    legend.innerHTML = '';

    // Die swatches（來自 .out 或 .block）
    const numDie = outData
        ? Object.keys(outData.dies).length
        : (blockData ? blockData.numDie : 0);

    for (let i = 0; i < numDie; i++) {
        const p = getPalette(i);
        const item = document.createElement('div');
        item.className = 'legend-item';
        item.innerHTML = `<div class="legend-swatch" style="background:${p.fill};border-color:${p.stroke}"></div>
                          <span>Die ${i}</span>`;
        legend.appendChild(item);
    }

    if (blockData && blockData.terminals.length > 0) {
        const item = document.createElement('div');
        item.className = 'legend-item';
        item.innerHTML = `<div class="legend-dot"></div><span>Terminal (${blockData.terminals.length})</span>`;
        legend.appendChild(item);
    }

    if (blockData && blockData.outlines.length > 0) {
        const item = document.createElement('div');
        item.className = 'legend-item';
        item.innerHTML = `<div class="legend-outline"></div><span>Outline</span>`;
        legend.appendChild(item);
    }

    // TSV entries（來自 .out）
    if (outData && outData.tsvStrips && outData.tsvStrips.length > 0) {
        const item = document.createElement('div');
        item.className = 'legend-item';
        item.innerHTML = `<div class="legend-strip"></div><span>TSV Strip (${outData.tsvStrips.length})</span>`;
        legend.appendChild(item);
    }

    if (outData && outData.tsvAssignments && outData.tsvAssignments.length > 0) {
        // 找出所有 tier boundaries 的種類
        const tiers = [...new Set(outData.tsvAssignments.map(a => `${a.tierLo}-${a.tierHi}`))].sort();
        tiers.forEach(t => {
            const lo  = parseInt(t.split('-')[0], 10);
            const cnt = outData.tsvAssignments.filter(a => a.tierLo === lo).length;
            const c   = TSV_TIER_COLORS[lo] || TSV_TIER_COLORS[TSV_TIER_COLORS.length - 1];
            const item = document.createElement('div');
            item.className = 'legend-item';
            // 小方塊代表 TSV 正方形
            item.innerHTML =
                `<div style="width:10px;height:10px;background:${c.fill};border:1.5px solid ${c.stroke};border-radius:1px;flex-shrink:0"></div>` +
                `<span>TSV tier${t} (${cnt})</span>`;
            legend.appendChild(item);
        });
        // fallback indicator
        const fbCnt = outData.tsvAssignments.filter(a => a.fallback).length;
        if (fbCnt > 0) {
            const item = document.createElement('div');
            item.className = 'legend-item';
            item.innerHTML =
                `<div style="width:10px;height:10px;background:#fca5a5;border:1.5px solid #ef4444;border-radius:1px;outline:1.5px solid #ef4444;outline-offset:1px;flex-shrink:0"></div>` +
                `<span>Fallback (${fbCnt})</span>`;
            legend.appendChild(item);
        }
    }
}

// ── Info Panel ─────────────────────────────────────────────────────────────────
function updateInfoPanel() {
    const panel = document.getElementById('info-content');
    if (!outData && !blockData) {
        panel.textContent = 'Load an .out or .block file to begin.';
        return;
    }

    const sel = document.getElementById('die-select').value;
    let html = '';

    if (outData) {
        html += `<strong>Cost:</strong> ${outData.cost} &nbsp;|&nbsp;
                 <strong>HPWL:</strong> ${outData.hpwl} &nbsp;|&nbsp;
                 <strong>Area:</strong> ${outData.area} &nbsp;|&nbsp;
                 <strong>BBox:</strong> ${outData.width} × ${outData.height} &nbsp;|&nbsp;
                 <strong>Runtime:</strong> ${outData.runtime.toFixed(3)} s<br><br>`;

        const ids = Object.keys(outData.dies).map(Number).sort((a, b) => a - b);
        if (sel === 'all') {
            ids.forEach(id => {
                const d = outData.dies[id];
                const w = d.bbox.maxX - d.bbox.minX, h = d.bbox.maxY - d.bbox.minY;
                html += `<span>Die ${id}: ${d.blocks.length} blocks, bbox = ${w.toFixed(0)} × ${h.toFixed(0)}</span>  `;
            });
        } else {
            const id = parseInt(sel, 10);
            const d  = outData.dies[id];
            if (d) {
                const w = d.bbox.maxX - d.bbox.minX, h = d.bbox.maxY - d.bbox.minY;
                html += `<span>Die ${id}: ${d.blocks.length} blocks, bbox = ${w.toFixed(0)} × ${h.toFixed(0)}</span>`;
            }
        }
    }

    // TSV strip & assignment stats
    if (outData && outData.tsvStrips && outData.tsvStrips.length > 0) {
        if (html) html += '<br>';
        const strips = outData.tsvStrips;
        const assigns = outData.tsvAssignments || [];
        const fbCnt   = assigns.filter(a => a.fallback).length;
        const nsCnt   = assigns.filter(a => a.noSlot).length;

        // 統計各 tier
        const tierCounts = {};
        assigns.forEach(a => {
            const k = `${a.tierLo}-${a.tierHi}`;
            tierCounts[k] = (tierCounts[k] || 0) + 1;
        });
        const tierStr = Object.entries(tierCounts)
            .sort((a, b) => a[0].localeCompare(b[0]))
            .map(([t, n]) => `tier${t}: ${n}`)
            .join(' | ');

        html += `<strong>TSV Strips:</strong> ${strips.length} &nbsp;|&nbsp; ` +
                `<strong>TSV Assignments:</strong> ${assigns.length} ` +
                `(fallback: ${fbCnt}, no_slot: ${nsCnt}) &nbsp;|&nbsp; ` +
                `${tierStr}`;
    }

    if (blockData) {
        if (html) html += '<br><br>';
        html += `<strong>.block:</strong> ${blockData.numDie} dies, `;
        html += `${blockData.modules.length} modules, `;
        html += `${blockData.terminals.length} terminals &nbsp;|&nbsp; `;
        html += `Outline: `;
        html += blockData.outlines.map((o, i) => `Die ${i}: ${o.w} × ${o.h}`).join(', ');
    }

    panel.innerHTML = html || 'No data.';
}

// ── 主繪圖函式 ─────────────────────────────────────────────────────────────────
function redraw() {
    if (!outData && !blockData) return;

    const sel = document.getElementById('die-select').value;
    terminalPixels = [];
    tsvPixels      = [];

    // ── 決定世界座標範圍 ──
    let wMinX = Infinity, wMinY = Infinity, wMaxX = -Infinity, wMaxY = -Infinity;

    // 來自 .out block 位置
    if (outData) {
        const ids = sel === 'all'
            ? Object.keys(outData.dies).map(Number)
            : [parseInt(sel, 10)];
        ids.forEach(id => {
            const d = outData.dies[id];
            if (!d) return;
            wMinX = Math.min(wMinX, d.bbox.minX); wMinY = Math.min(wMinY, d.bbox.minY);
            wMaxX = Math.max(wMaxX, d.bbox.maxX); wMaxY = Math.max(wMaxY, d.bbox.maxY);
        });
    }

    // 來自 TSV strips（.out）
    if (outData && outData.tsvStrips && showTsvStrips) {
        outData.tsvStrips.forEach(s => {
            wMinX = Math.min(wMinX, s.x1); wMinY = Math.min(wMinY, s.y1);
            wMaxX = Math.max(wMaxX, s.x2); wMaxY = Math.max(wMaxY, s.y2);
        });
    }

    // 來自 TSV assignments（.out）
    if (outData && outData.tsvAssignments && showTsvPoints) {
        outData.tsvAssignments.forEach(a => {
            wMinX = Math.min(wMinX, a.x); wMinY = Math.min(wMinY, a.y);
            wMaxX = Math.max(wMaxX, a.x); wMaxY = Math.max(wMaxY, a.y);
        });
    }

    // 來自 .block outline（以 (0,0) 為原點）
    if (blockData && blockData.outlines.length > 0) {
        blockData.outlines.forEach(o => {
            wMinX = Math.min(wMinX, 0); wMinY = Math.min(wMinY, 0);
            wMaxX = Math.max(wMaxX, o.w); wMaxY = Math.max(wMaxY, o.h);
        });
    }

    // 來自 .block terminal 位置
    if (blockData && showTerminals) {
        blockData.terminals.forEach(t => {
            wMinX = Math.min(wMinX, t.x); wMinY = Math.min(wMinY, t.y);
            wMaxX = Math.max(wMaxX, t.x); wMaxY = Math.max(wMaxY, t.y);
        });
    }

    if (!isFinite(wMinX)) { setCanvasMessage('No data to display.'); return; }

    // ── Canvas 大小 ──
    const cw = Math.max(400, (wMaxX - wMinX) * scale + 2 * PADDING);
    const ch = Math.max(300, (wMaxY - wMinY) * scale + 2 * PADDING);
    canvas.width  = cw;
    canvas.height = ch;

    // ── 世界座標 → 畫布座標轉換 ──
    const toCanvasX = wx => PADDING + (wx - wMinX) * scale;
    const toCanvasY = wy => ch - PADDING - (wy - wMinY) * scale;  // y 軸往上為正

    ctx.clearRect(0, 0, cw, ch);

    // ── 1. 畫 Outline（.block）──
    if (blockData && showOutline && blockData.outlines.length > 0) {
        const diesToDraw = sel === 'all'
            ? blockData.outlines.map((_, i) => i)
            : [parseInt(sel, 10)].filter(i => i < blockData.outlines.length);

        diesToDraw.forEach(i => {
            const o = blockData.outlines[i];
            if (!o) return;
            const p  = getPalette(i);
            const ox = toCanvasX(0);
            const oy = toCanvasY(o.h);    // top-left in canvas coords
            const ow = o.w * scale;
            const oh = o.h * scale;

            // 外框底色（極淡）
            ctx.fillStyle = p.fill.replace('0.45', '0.08');
            ctx.fillRect(ox, oy, ow, oh);

            // Dashed border
            ctx.save();
            ctx.strokeStyle = p.outline;
            ctx.lineWidth   = 2.5;
            ctx.setLineDash([8, 5]);
            ctx.strokeRect(ox, oy, ow, oh);
            ctx.restore();

            // 標籤
            ctx.fillStyle = p.outline;
            ctx.font      = 'bold 13px Arial';
            ctx.fillText(`Die ${i} Outline (${o.w}×${o.h})`, ox + 6, oy + 16);
        });
    }

    // ── 1.5. 畫 TSV Strips（.out）──
    if (outData && showTsvStrips && outData.tsvStrips && outData.tsvStrips.length > 0) {
        // 用對角線 pattern 表示「預留但不是主電路」的性質
        const offscreenCanvas = document.createElement('canvas');
        offscreenCanvas.width = 8; offscreenCanvas.height = 8;
        const octx = offscreenCanvas.getContext('2d');
        octx.strokeStyle = 'rgba(217,119,6,0.6)';
        octx.lineWidth = 1;
        octx.beginPath();
        octx.moveTo(0, 8); octx.lineTo(8, 0);
        octx.moveTo(-2, 2); octx.lineTo(2, -2);
        octx.moveTo(6, 10); octx.lineTo(10, 6);
        octx.stroke();
        const hatchPattern = ctx.createPattern(offscreenCanvas, 'repeat');

        const filteredStrips = sel === 'all'
            ? outData.tsvStrips
            : outData.tsvStrips.filter(s => s.dieId === parseInt(sel, 10));

        filteredStrips.forEach(s => {
            const sx  = toCanvasX(s.x1);
            const sy  = toCanvasY(s.y2);   // y2 = top in world coords
            const sw  = (s.x2 - s.x1) * scale;
            const sh  = (s.y2 - s.y1) * scale;

            // 半透明金色底色
            ctx.fillStyle = 'rgba(251,191,36,0.30)';
            ctx.fillRect(sx, sy, sw, sh);

            // 對角線 hatch
            ctx.fillStyle = hatchPattern;
            ctx.fillRect(sx, sy, sw, sh);

            // 深金色邊框
            ctx.strokeStyle = 'rgba(180,83,9,0.85)';
            ctx.lineWidth   = 1.2;
            ctx.strokeRect(sx, sy, sw, sh);

            // 名稱（夠大才顯示）
            if (scale >= 1.5 && sw > 16 && sh > 8) {
                ctx.fillStyle = '#78350f';
                ctx.font      = '9px Arial';
                ctx.fillText(s.blockName, sx + 2, sy + 10);
            }
        });
    }

    // ── 2. 畫 Blocks（.out）──
    if (outData) {
        const ids = sel === 'all'
            ? Object.keys(outData.dies).map(Number)
            : [parseInt(sel, 10)];

        ids.forEach(id => {
            const d = outData.dies[id];
            if (!d) return;
            const p = getPalette(id);

            d.blocks.forEach(b => {
                const bx = toCanvasX(b.x);
                const by = toCanvasY(b.y + b.height);
                const bw = b.width  * scale;
                const bh = b.height * scale;

                ctx.fillStyle   = p.fill;
                ctx.fillRect(bx, by, bw, bh);
                ctx.strokeStyle = p.stroke;
                ctx.lineWidth   = 1;
                ctx.strokeRect(bx, by, bw, bh);

                // 名稱置中顯示（zoom 夠大才顯示）
                if (scale >= 0.5 && bw > 20 && bh > 10) {
                    const fontSize = Math.min(11, bh * 0.45, bw * 0.3);
                    ctx.save();
                    ctx.fillStyle  = '#111';
                    ctx.font       = `${fontSize}px Arial`;
                    ctx.textAlign  = 'center';
                    ctx.textBaseline = 'middle';
                    // 若文字比方塊寬，縮放後再畫
                    const textW = ctx.measureText(b.name).width;
                    if (textW > bw - 4) {
                        ctx.scale((bw - 4) / textW, 1);
                        ctx.fillText(b.name,
                            (bx + bw / 2) * textW / (bw - 4),
                            by + bh / 2);
                    } else {
                        ctx.fillText(b.name, bx + bw / 2, by + bh / 2);
                    }
                    ctx.restore();
                }
            });
        });
    }

    // ── 3. 畫 Terminals（.block）──
    if (blockData && showTerminals && blockData.terminals.length > 0) {
        const R = Math.max(3, Math.min(6, scale * 4));   // 半徑隨 zoom 微調

        blockData.terminals.forEach(t => {
            const tx = toCanvasX(t.x);
            const ty = toCanvasY(t.y);

            terminalPixels.push({ name: t.name, cx: tx, cy: ty,
                                   worldX: t.x, worldY: t.y });

            const isHovered = hoveredTerminal && hoveredTerminal.name === t.name;

            // 外圈（hover 時放大）
            ctx.beginPath();
            ctx.arc(tx, ty, isHovered ? R + 3 : R, 0, Math.PI * 2);
            ctx.fillStyle   = isHovered ? '#f97316' : '#e53e3e';
            ctx.fill();
            ctx.strokeStyle = isHovered ? '#7c2d12' : '#9b2c2c';
            ctx.lineWidth   = isHovered ? 1.5 : 1;
            ctx.stroke();

            // 名稱（hover 時顯示；zoom 夠大時全顯示）
            if (isHovered || scale >= 1.5) {
                ctx.fillStyle = '#1a1a1a';
                ctx.font      = '10px Arial';
                ctx.fillText(t.name, tx + R + 2, ty + 4);
            }
        });
    }

    // ── 4. 畫 TSV Assignment 點位（.out）──
    if (outData && showTsvPoints && outData.tsvAssignments && outData.tsvAssignments.length > 0) {

        const filteredAssigns = sel === 'all'
            ? outData.tsvAssignments
            : outData.tsvAssignments.filter(a =>
                a.tierLo === parseInt(sel, 10) || a.tierHi === parseInt(sel, 10));

        filteredAssigns.forEach(a => {
            const c  = TSV_TIER_COLORS[a.tierLo] ||
                       TSV_TIER_COLORS[TSV_TIER_COLORS.length - 1];

            const isHovered = hoveredTsv && hoveredTsv.netId === a.netId &&
                              hoveredTsv.tierLo === a.tierLo;

            // TSV 實際佔地：世界座標 (x±1.5, y±1.5)，畫布對應 3*scale 大小
            const sqX = toCanvasX(a.x - 1.5);
            const sqY = toCanvasY(a.y + 1.5);   // y+1.5 = world 上方 = canvas 上方
            const sqW = Math.max(3, 3 * scale);
            const sqH = Math.max(3, 3 * scale);

            // fallback: 外圍紅色警示框
            if (a.fallback) {
                ctx.strokeStyle = 'rgba(239,68,68,0.70)';
                ctx.lineWidth   = 1.5;
                ctx.strokeRect(sqX - 2, sqY - 2, sqW + 4, sqH + 4);
            }

            // 主正方形（hover 時用較深填色）
            ctx.fillStyle   = isHovered ? c.stroke : c.fill;
            ctx.fillRect(sqX, sqY, sqW, sqH);
            ctx.strokeStyle = c.stroke;
            ctx.lineWidth   = 1;
            ctx.strokeRect(sqX, sqY, sqW, sqH);

            // hover 時顯示 net 名稱
            if (isHovered || scale >= 2.5) {
                ctx.fillStyle = '#1a1a1a';
                ctx.font      = '9px Arial';
                ctx.fillText(a.netId, sqX + sqW + 2, sqY + sqH * 0.7);
            }

            // 記錄 hover 檢測資料（以正方形中心為參考點）
            tsvPixels.push({
                netId: a.netId, tierLo: a.tierLo, tierHi: a.tierHi,
                cx: sqX + sqW / 2, cy: sqY + sqH / 2,
                worldX: a.x, worldY: a.y,
                fallback: a.fallback, noSlot: a.noSlot,
            });
        });
    }

    // ── 5. 座標軸刻度標示（可選的方向感）──
    drawAxisTicks(toCanvasX, toCanvasY, wMinX, wMinY, wMaxX, wMaxY);
}

// ── 座標軸刻度 ─────────────────────────────────────────────────────────────────
function drawAxisTicks(toX, toY, minX, minY, maxX, maxY) {
    const rangeX = maxX - minX, rangeY = maxY - minY;
    const step   = niceStep(Math.max(rangeX, rangeY) / 6);

    ctx.save();
    ctx.strokeStyle = '#aaa';
    ctx.fillStyle   = '#666';
    ctx.font        = '10px Arial';
    ctx.lineWidth   = 0.5;

    // X ticks (bottom)
    for (let v = Math.ceil(minX / step) * step; v <= maxX; v += step) {
        const cx = toX(v);
        const cy = canvas.height - PADDING;
        ctx.beginPath();
        ctx.moveTo(cx, cy); ctx.lineTo(cx, cy + 5);
        ctx.stroke();
        ctx.textAlign = 'center';
        ctx.fillText(v, cx, cy + 14);
    }

    // Y ticks (left)
    for (let v = Math.ceil(minY / step) * step; v <= maxY; v += step) {
        const cx = PADDING;
        const cy = toY(v);
        ctx.beginPath();
        ctx.moveTo(cx - 5, cy); ctx.lineTo(cx, cy);
        ctx.stroke();
        ctx.textAlign = 'right';
        ctx.fillText(v, cx - 7, cy + 3);
    }
    ctx.restore();
}

function niceStep(rawStep) {
    const mag = Math.pow(10, Math.floor(Math.log10(rawStep)));
    const r   = rawStep / mag;
    if (r < 1.5) return mag;
    if (r < 3.5) return 2 * mag;
    if (r < 7.5) return 5 * mag;
    return 10 * mag;
}

// ── 訊息畫面 ───────────────────────────────────────────────────────────────────
function setCanvasMessage(message) {
    canvas.width  = 800;
    canvas.height = 500;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle  = '#555';
    ctx.font       = '16px Arial';
    ctx.textAlign  = 'center';
    ctx.fillText(message, canvas.width / 2, canvas.height / 2);
    ctx.textAlign  = 'start';
}
