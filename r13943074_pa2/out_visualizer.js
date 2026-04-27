// Multi-Die Floorplan Visualizer
// 支援 .out (block 位置) 與 .block (outline + terminals) 兩種檔案同時載入。

let canvas, ctx;
let scale = 0.3;
const PADDING = 50;

// .out 解析結果
// { cost, hpwl, area, width, height, runtime,
//   dies: { [dieId]: { blocks:[{name,x,y,width,height}], bbox:{minX,minY,maxX,maxY} } } }
let outData = null;
// .out 原始解析結果（用於 Reset）
let outDataOriginal = null;

// .block 解析結果
// { numDie, outlines:[{w,h}], modules:[{name,w,h,dieId}], terminals:[{name,x,y}] }
let blockData = null;

// .nets 解析結果（每個 net 的 terminals 名稱清單）
// netsData: { numNets, nets: Array<Array<string>> }
let netsData = null;

// 顯示選項
let showOutline    = true;
let showTerminals  = true;
let showTsvStrips     = true;
let showTsvPoints     = true;
let showTsvLabels     = true;   // TSV strip 名稱、TSV assignment 的 net 文字（畫在圖上）
let tsvLowerTierOnly  = false;  // true = tier d→d+1 的 TSV 只在 die d（下層）顯示
let showNets          = false;  // 顯示同層 intra-die net 連線

// 畫面上 terminal 的像素位置（供 hover 檢測）
let terminalPixels = [];   // [{name, cx, cy, worldX, worldY}]

// 畫面上 TSV assignment 的像素位置（供 hover 檢測）
let tsvPixels = [];        // [{netId, tierLo, tierHi, cx, cy, worldX, worldY, fallback, noSlot}]

// 目前 hover 的元素
let hoveredTerminal = null;
let hoveredTsv      = null;

// ── 編輯狀態（點選/拖曳/旋轉）──────────────────────────────────────────────
// selected:
//   { type:'module', dieId, blockIndex, name }
//   { type:'tsv', assignmentIndex }
let selected = null;
let dragging = false;
let dragOffsetX = 0;
let dragOffsetY = 0;
let currentView = null; // { wMinX, wMinY, ch, scale }

// ── Legalize process replay ────────────────────────────────────────────────────
let legalizeData        = null;  // { tierData: {N: {steps,moves}}, tier0Steps, tier0Moves }
let legalizeStepIndex   = -1;    // -1 = initial; 0..N-1 = after applying step i
let legalizeCache       = [];    // [0]=initial die state; [i+1]=state after step i
let legalizeCurrentTier = 0;     // 目前播放中的 tier（與 die-select 同步）
let die0ModuleMap       = new Map(); // module_id(number) -> blockIndex in current tier die
let legalizeBaseDims    = {};    // module_id -> {w, h}（保留但不用於旋轉判斷）

function deepCopy(obj) {
    return JSON.parse(JSON.stringify(obj));
}

// TSV centers 在既有輸出中通常是 .0/.5；用 0.5 網格對齊，確保 3x3 正方形四邊落在整數座標。
function snapTsvCenter(v) {
    return Math.round(v * 2) / 2;
}

function canvasToWorldX(cx) {
    if (!currentView) return 0;
    return currentView.wMinX + (cx - PADDING) / currentView.scale;
}

function canvasToWorldY(cy) {
    if (!currentView) return 0;
    return currentView.wMinY + (currentView.ch - PADDING - cy) / currentView.scale;
}

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
    const netsLabel     = document.getElementById('loadNetsLabel');
    const netsInput     = document.getElementById('netsFileInput');
    const scaleSlider   = document.getElementById('scale-slider');
    const scaleValue    = document.getElementById('scale-value');
    const dieSelect     = document.getElementById('die-select');
    const outlineCheck  = document.getElementById('showOutlineCheck');
    const terminalCheck = document.getElementById('showTerminalsCheck');
    const resetBtn      = document.getElementById('resetPositionsBtn');
    const rotateBtn     = document.getElementById('rotateSelectedBtn');
    const moveCenterBtn = document.getElementById('moveSelectedCenterBtn');
    const moduleCenterXInput = document.getElementById('moduleCenterXInput');
    const moduleCenterYInput = document.getElementById('moduleCenterYInput');
    const recomputeHpwlBtn = document.getElementById('recomputeHpwlBtn');
    const tooltip       = document.getElementById('tooltip');

    // ── .out 檔案 ──
    outInput.addEventListener('change', e => {
        const file = e.target.files[0];
        if (!file) return;
        readFile(file, text => {
            try {
                outDataOriginal = parseOutContent(text);
                outData = deepCopy(outDataOriginal);
                buildDie0ModuleMap();
                // 重新載入 .out 後重置 legalize replay 狀態
                if (legalizeData) resetLegalizeReplay();
                selected = null;
                dragging = false;
                hoveredTerminal = null;
                hoveredTsv = null;
                syncSelectedModuleCenterInputs();
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
                buildLegalizeBaseDims();
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

    // ── .nets 檔案 ──
    netsInput.addEventListener('change', e => {
        const file = e.target.files[0];
        if (!file) return;
        readFile(file, text => {
            try {
                netsData = parseNetsContent(text);
                netsLabel.textContent = `✓ ${file.name}`;
                netsLabel.classList.add('loaded');
                updateInfoPanel();
                redraw();
            } catch (err) {
                console.error(err);
                alert('Failed to parse .nets file: ' + err.message);
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
        // 換 die 時同步切換 legalize replay 到對應 tier
        if (legalizeData) {
            const val = dieSelect.value;
            const tierNum = (val === 'all') ? legalizeCurrentTier : parseInt(val, 10);
            switchLegalizeTier(tierNum);
        }
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
    document.getElementById('showTsvLabelsCheck').addEventListener('change', e => {
        showTsvLabels = e.target.checked; redraw();
    });
    document.getElementById('tsvLowerTierOnlyCheck').addEventListener('change', e => {
        tsvLowerTierOnly = e.target.checked; redraw();
    });
    document.getElementById('showNetsCheck').addEventListener('change', e => {
        showNets = e.target.checked; redraw();
    });

    // ── Edit: Reset / Rotate ───────────────────────────────────────────────
    resetBtn.addEventListener('click', () => {
        if (!outDataOriginal) return;
        outData = deepCopy(outDataOriginal);
        selected = null;
        dragging = false;
        hoveredTerminal = null;
        hoveredTsv = null;
        syncSelectedModuleCenterInputs();
        tooltip.style.display = 'none';
        redraw();
        updateInfoPanel();
    });

    rotateBtn.addEventListener('click', () => {
        if (!outData || !selected || selected.type !== 'module') return;
        const die = outData.dies[selected.dieId];
        if (!die) return;
        const b = die.blocks[selected.blockIndex];
        if (!b) return;
        // 以 module 中心點為 anchor 旋轉 90°（交換 w/h 並回推左下角）
        const cx = b.x + b.width / 2;
        const cy = b.y + b.height / 2;
        const oldW = b.width;
        b.width = b.height;
        b.height = oldW;
        b.x = cx - b.width / 2;
        b.y = cy - b.height / 2;
        syncSelectedModuleCenterInputs();
        redraw();
        updateInfoPanel();
    });

    moveCenterBtn.addEventListener('click', () => {
        if (!outData || !selected || selected.type !== 'module') {
            alert('Please select a module first.');
            return;
        }
        const cx = parseFloat(moduleCenterXInput.value);
        const cy = parseFloat(moduleCenterYInput.value);
        if (!Number.isFinite(cx) || !Number.isFinite(cy)) {
            alert('Please input valid center coordinates (Cx, Cy).');
            return;
        }

        const die = outData.dies[selected.dieId];
        if (!die) return;
        const b = die.blocks[selected.blockIndex];
        if (!b) return;
        b.x = cx - b.width / 2;
        b.y = cy - b.height / 2;
        syncSelectedModuleCenterInputs();
        redraw();
        updateInfoPanel();
    });

    // ── Edit: Recompute HPWL（分層 bbox + TSV 雙邊計入）──────────────────────
    recomputeHpwlBtn.addEventListener('click', () => {
        if (!outData || !blockData || !netsData) {
            alert('Please upload .out, .block, and .nets first.');
            return;
        }
        try {
            const recomputed = recomputeHpwlFromCurrentState(outData, blockData, netsData);
            outData.hpwl = recomputed.hpwl;

            updateInfoPanel();
            redraw();
        } catch (err) {
            console.error(err);
            alert('Failed to recompute HPWL: ' + err.message);
        }
    });

    // ── .process 檔案載入 + Replay 按鈕 ─────────────────────────────────────
    const processInput     = document.getElementById('processFileInput');
    const processLabel     = document.getElementById('loadProcessLabel');
    const legalizeBackBtn  = document.getElementById('legalizeBackBtn');
    const legalizeForwardBtn = document.getElementById('legalizeForwardBtn');

    processInput.addEventListener('change', e => {
        const file = e.target.files[0];
        if (!file) return;
        readFile(file, text => {
            try {
                legalizeData = parseLegalizeProcess(text);
                processLabel.textContent = `✓ ${file.name}`;
                processLabel.classList.add('loaded');
                buildDie0ModuleMap();
                buildLegalizeBaseDims();
                resetLegalizeReplay();
                updateLegalizeUI();
            } catch (err) {
                console.error(err);
                alert('Failed to parse .process file: ' + err.message);
            }
        });
    });

    legalizeBackBtn.addEventListener('click', () => legalizeGoBack());
    legalizeForwardBtn.addEventListener('click', () => legalizeGoForward());

    // 鍵盤快捷鍵：← / → 控制 replay（input 焦點時不攔截）
    document.addEventListener('keydown', e => {
        if (!legalizeData) return;
        const tag = document.activeElement && document.activeElement.tagName;
        if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
        if (e.key === 'ArrowRight') { e.preventDefault(); legalizeGoForward(); }
        if (e.key === 'ArrowLeft')  { e.preventDefault(); legalizeGoBack(); }
    });

    // ── Canvas edit（點選/拖曳）────────────────────────────────────────────
    canvas.addEventListener('mousedown', e => {
        if (!outData) return;
        const rect = canvas.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;
        const worldX = canvasToWorldX(mx);
        const worldY = canvasToWorldY(my);

        // 當前 die 選擇範圍
        const sel = document.getElementById('die-select').value;
        const dieIds = sel === 'all'
            ? Object.keys(outData.dies).map(Number)
            : [parseInt(sel, 10)];

        // 1) 優先 hit-test module（較大、好抓）
        let bestMod = null; // {dieId, blockIndex, dist2, b}
        dieIds.forEach(dieId => {
            const die = outData.dies[dieId];
            if (!die) return;
            die.blocks.forEach((b, bi) => {
                const x1 = b.x, x2 = b.x + b.width;
                const y1 = b.y, y2 = b.y + b.height;
                if (worldX < x1 || worldX > x2 || worldY < y1 || worldY > y2) return;
                const cx = x1 + (x2 - x1) / 2;
                const cy = y1 + (y2 - y1) / 2;
                const dx = worldX - cx, dy = worldY - cy;
                const dist2 = dx * dx + dy * dy;
                if (!bestMod || dist2 < bestMod.dist2) {
                    bestMod = { dieId, blockIndex: bi, dist2, b };
                }
            });
        });

        // 2) hit-test TSV（依目前 showTsvPoints + die selection + tsvLowerTierOnly）
        let bestTsv = null; // {assignmentIndex, a, dist2}
        if (showTsvPoints && outData.tsvAssignments && outData.tsvAssignments.length > 0) {
            const selDie = sel === 'all' ? -1 : parseInt(sel, 10);
            outData.tsvAssignments.forEach((a, assignmentIndex) => {
                const shown = selDie === -1
                    ? true
                    : (tsvLowerTierOnly
                        ? a.tierLo === selDie
                        : (a.tierLo === selDie || a.tierHi === selDie));
                if (!shown) return;
                const x1 = a.x - 1.5, x2 = a.x + 1.5;
                const y1 = a.y - 1.5, y2 = a.y + 1.5;
                if (worldX < x1 || worldX > x2 || worldY < y1 || worldY > y2) return;
                const cx = a.x, cy = a.y;
                const dx = worldX - cx, dy = worldY - cy;
                const dist2 = dx * dx + dy * dy;
                if (!bestTsv || dist2 < bestTsv.dist2) bestTsv = { assignmentIndex, a, dist2 };
            });
        }

        // 選取
        if (bestMod) {
            selected = { type: 'module', dieId: bestMod.dieId, blockIndex: bestMod.blockIndex, name: bestMod.b.name };
            dragOffsetX = worldX - bestMod.b.x;
            dragOffsetY = worldY - bestMod.b.y;
            dragging = true;
            hoveredTerminal = null;
            hoveredTsv = null;
            tooltip.style.display = 'none';
            syncSelectedModuleCenterInputs();
            redraw();
            updateInfoPanel();
            return;
        }

        if (bestTsv) {
            selected = { type: 'tsv', assignmentIndex: bestTsv.assignmentIndex };
            dragOffsetX = worldX - bestTsv.a.x;
            dragOffsetY = worldY - bestTsv.a.y;
            dragging = true;
            hoveredTerminal = null;
            hoveredTsv = null;
            tooltip.style.display = 'none';
            syncSelectedModuleCenterInputs();
            redraw();
            updateInfoPanel();
            return;
        }

        selected = null;
        dragging = false;
        hoveredTerminal = null;
        hoveredTsv = null;
        tooltip.style.display = 'none';
        syncSelectedModuleCenterInputs();
        redraw();
        updateInfoPanel();
    });

    window.addEventListener('mouseup', () => {
        if (!dragging) return;
        dragging = false;
        updateInfoPanel();
        redraw();
    });

    // ── Canvas hover（偵測 terminal + TSV point）──
    canvas.addEventListener('mousemove', e => {
        const rect = canvas.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;
        const HIT_R = 8;

        // 拖曳時：直接更新座標（整數/TSV 0.5 網格）並重繪
        if (dragging && selected) {
            const worldX = canvasToWorldX(mx);
            const worldY = canvasToWorldY(my);
            if (selected.type === 'module') {
                const die = outData && outData.dies ? outData.dies[selected.dieId] : null;
                if (die && die.blocks && die.blocks[selected.blockIndex]) {
                    const b = die.blocks[selected.blockIndex];
                    // snap to integer grid
                    // const targetX = worldX - dragOffsetX;
                    // const targetY = worldY - dragOffsetY;
                    // b.x = snapInt(targetX);
                    // b.y = snapInt(targetY);
                    b.x = worldX - dragOffsetX;
                    b.y = worldY - dragOffsetY;
                    syncSelectedModuleCenterInputs();
                }
            } else if (selected.type === 'tsv') {
                const a = outData && outData.tsvAssignments ? outData.tsvAssignments[selected.assignmentIndex] : null;
                if (a) {
                    const targetX = worldX - dragOffsetX;
                    const targetY = worldY - dragOffsetY;
                    a.x = snapTsvCenter(targetX);
                    a.y = snapTsvCenter(targetY);
                }
            }
            redraw();
            tooltip.style.display = 'none';
            return;
        }

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
            let bboxHint = '';
            if (outData && blockData && netsData && netsData.nets) {
                const netIdx = parseInt(String(foundTsv.netId).replace(/net/i, ''), 10);
                if (Number.isFinite(netIdx) && netIdx >= 0 && netIdx < netsData.nets.length) {
                    const ext = computeNetPerDieExtents(outData, blockData, netsData, netIdx);
                    if (ext) {
                        const summarize = die => {
                            const e = ext.perDie[die];
                            if (!e.has) return `D${die}:∅`;
                            const w = e.maxX - e.minX, h = e.maxY - e.minY;
                            if (w < 1e-6 && h < 1e-6) return `D${die}:pt`;
                            return `D${die}:bbox`;
                        };
                        bboxHint = `  |  ${summarize(foundTsv.tierLo)} ${summarize(foundTsv.tierHi)}`;
                    }
                }
            } else if (!netsData) {
                bboxHint = '  |  (load .nets for layer bbox)';
            }
            // tooltip.textContent =
            //     `${foundTsv.netId}  tier${foundTsv.tierLo}-${foundTsv.tierHi}` +
            //     `  (${foundTsv.worldX.toFixed(1)}, ${foundTsv.worldY.toFixed(1)})${fb}${ns}${bboxHint}`;
            tooltip.textContent =
                `${foundTsv.netId}`;
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

function syncSelectedModuleCenterInputs() {
    const xInput = document.getElementById('moduleCenterXInput');
    const yInput = document.getElementById('moduleCenterYInput');
    if (!xInput || !yInput) return;

    if (!outData || !selected || selected.type !== 'module') {
        xInput.value = '';
        yInput.value = '';
        return;
    }

    const die = outData.dies[selected.dieId];
    const b = die && die.blocks ? die.blocks[selected.blockIndex] : null;
    if (!b) {
        xInput.value = '';
        yInput.value = '';
        return;
    }
    const cx = b.x + b.width / 2;
    const cy = b.y + b.height / 2;
    xInput.value = cx.toFixed(2);
    yInput.value = cy.toFixed(2);
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

// ── 解析 .nets 檔 ──────────────────────────────────────────────────────────────
// 格式（以 n100.nets 為例）：
// NumNets: 885
// NetDegree: 2
// p1
// sb26
// NetDegree: 2
// p2
// sb46
function parseNetsContent(text) {
    const lines = text.split(/\r?\n/).map(l => l.trim()).filter(l => l.length > 0);
    if (lines.length < 2) throw new Error('Nets file too short.');

    let numNets = null;
    if (lines[0].startsWith('NumNets:')) {
        numNets = parseInt(lines[0].split(':')[1], 10);
    }

    const nets = [];
    let i = 1;
    while (i < lines.length) {
        if (!lines[i].startsWith('NetDegree:')) {
            i++;
            continue;
        }
        const degree = parseInt(lines[i].split(':')[1], 10);
        i++;
        const terms = [];
        for (let k = 0; k < degree; k++) {
            if (i >= lines.length) break;
            terms.push(lines[i]);
            i++;
        }
        nets.push(terms);
        if (numNets !== null && nets.length >= numNets) break;
    }

    if (numNets !== null && nets.length !== numNets) {
        console.warn(`Parsed nets count mismatch: expected ${numNets}, got ${nets.length}`);
    }

    return { numNets: nets.length, nets };
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
                let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
                d.blocks.forEach(b => {
                    minX = Math.min(minX, b.x);
                    minY = Math.min(minY, b.y);
                    maxX = Math.max(maxX, b.x + b.width);
                    maxY = Math.max(maxY, b.y + b.height);
                });
                const w = maxX - minX, h = maxY - minY;
                html += `<span>Die ${id}: ${d.blocks.length} blocks, bbox = ${w.toFixed(0)} × ${h.toFixed(0)}</span>  `;
            });
        } else {
            const id = parseInt(sel, 10);
            const d  = outData.dies[id];
            if (d) {
                let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
                d.blocks.forEach(b => {
                    minX = Math.min(minX, b.x);
                    minY = Math.min(minY, b.y);
                    maxX = Math.max(maxX, b.x + b.width);
                    maxY = Math.max(maxY, b.y + b.height);
                });
                const w = maxX - minX, h = maxY - minY;
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

    // Selected info
    if (selected) {
        if (html) html += '<br><br>';
        if (selected.type === 'module') {
            const d = outData && outData.dies ? outData.dies[selected.dieId] : null;
            const b = d && d.blocks ? d.blocks[selected.blockIndex] : null;
            if (b) {
                html += `<strong>Selected module:</strong> ${b.name} &nbsp; (die ${selected.dieId})<br>` +
                        `x=${b.x}, y=${b.y}, w=${b.width}, h=${b.height}`;
            } else {
                html += `<strong>Selected module:</strong> (missing)`;
            }
        } else if (selected.type === 'tsv') {
            const a = outData && outData.tsvAssignments ? outData.tsvAssignments[selected.assignmentIndex] : null;
            if (a) {
                html += `<strong>Selected TSV:</strong> net${a.netId}  tier${a.tierLo}-${a.tierHi}<br>` +
                        `x=${a.x}, y=${a.y}`;
            } else {
                html += `<strong>Selected TSV:</strong> (missing)`;
            }
        }
    }

    panel.innerHTML = html || 'No data.';
}

// ── Recompute HPWL（分層 bbox + TSV 雙邊計入）────────────────────────────────────
function recomputeHpwlFromCurrentState(currentOut, currentBlock, currentNets) {
    if (!currentOut || !currentBlock || !currentNets) {
        throw new Error('Missing currentOut / currentBlock / currentNets.');
    }
    if (!currentOut.dies) throw new Error('outData missing dies.');
    if (!currentBlock.terminals) throw new Error('blockData missing terminals.');
    if (!currentOut.tsvAssignments) throw new Error('outData missing tsvAssignments.');
    if (!currentNets.nets) throw new Error('netsData missing nets.');

    const numDies = currentBlock.numDie || 1;

    // module name -> {dieId, cx, cy}
    const moduleMap = new Map();
    Object.keys(currentOut.dies).forEach(dieIdStr => {
        const dieId = parseInt(dieIdStr, 10);
        const die = currentOut.dies[dieId];
        if (!die || !die.blocks) return;
        die.blocks.forEach(b => {
            const cx = b.x + b.width / 2;
            const cy = b.y + b.height / 2;
            moduleMap.set(b.name, { dieId, cx, cy });
        });
    });

    // terminal name -> {cx, cy}，tier0
    const terminalMap = new Map();
    currentBlock.terminals.forEach(t => terminalMap.set(t.name, { cx: t.x, cy: t.y }));

    // tsv assignments grouped by net index
    const tsvByNet = new Map(); // netIdx -> array of {tierLo, tierHi, x, y}
    currentOut.tsvAssignments.forEach(a => {
        const netIdx = parseInt(String(a.netId).replace(/net/i, ''), 10);
        if (!Number.isFinite(netIdx)) return;
        if (!tsvByNet.has(netIdx)) tsvByNet.set(netIdx, []);
        tsvByNet.get(netIdx).push(a);
    });

    const nets = currentNets.nets;
    let totalHpwl = 0.0;

    for (let netIdx = 0; netIdx < nets.length; netIdx++) {
        const terms = nets[netIdx];

        const minX = Array(numDies).fill(Infinity);
        const minY = Array(numDies).fill(Infinity);
        const maxX = Array(numDies).fill(-Infinity);
        const maxY = Array(numDies).fill(-Infinity);
        const has   = Array(numDies).fill(false);

        function upd(d, px, py) {
            if (d < 0 || d >= numDies) return;
            minX[d] = Math.min(minX[d], px);
            minY[d] = Math.min(minY[d], py);
            maxX[d] = Math.max(maxX[d], px);
            maxY[d] = Math.max(maxY[d], py);
            has[d] = true;
        }

        // 1) Add module / terminal members
        terms.forEach(name => {
            if (typeof name !== 'string') return;
            if (name.startsWith('sb') || name.startsWith('r')) {
                if (!moduleMap.has(name)) return;
                const m = moduleMap.get(name);
                upd(m.dieId, m.cx, m.cy);
            } else if (name.startsWith('p')) {
                if (!terminalMap.has(name)) return;
                const t = terminalMap.get(name);
                upd(0, t.cx, t.cy);
            }
        });

        // 2) Add TSV points (included in both adjacent dies)
        const tsvList = tsvByNet.get(netIdx) || [];
        tsvList.forEach(tsv => {
            upd(tsv.tierLo, tsv.x, tsv.y);
            upd(tsv.tierHi, tsv.x, tsv.y);
        });

        // 3) Sum per-die HPWL
        for (let d = 0; d < numDies; d++) {
            if (!has[d]) continue;
            totalHpwl += (maxX[d] - minX[d]) + (maxY[d] - minY[d]);
        }
    }

    return {
        hpwl: totalHpwl,
        hpwlOld: currentOut.hpwl,
        costOld: currentOut.cost,
    };
}

/**
 * 單一 net 在各 die 上的座標範圍（與 recomputeHpwlFromCurrentState 邏輯一致，供 TSV hover 顯示）。
 * @returns {{ perDie: Array<{has:boolean,minX:number,minY:number,maxX:number,maxY:number}> }} | null
 */
function computeNetPerDieExtents(currentOut, currentBlock, currentNets, netIdx) {
    if (!currentOut || !currentOut.dies || !currentBlock || !currentBlock.terminals || !currentNets || !currentNets.nets) {
        return null;
    }
    const terms = currentNets.nets[netIdx];
    if (!terms) return null;

    const numDies = currentBlock.numDie || 1;

    const moduleMap = new Map();
    Object.keys(currentOut.dies).forEach(dieIdStr => {
        const dieId = parseInt(dieIdStr, 10);
        const die = currentOut.dies[dieId];
        if (!die || !die.blocks) return;
        die.blocks.forEach(b => {
            const cx = b.x + b.width / 2;
            const cy = b.y + b.height / 2;
            moduleMap.set(b.name, { dieId, cx, cy });
        });
    });

    const terminalMap = new Map();
    currentBlock.terminals.forEach(t => terminalMap.set(t.name, { cx: t.x, cy: t.y }));

    const tsvList = [];
    if (currentOut.tsvAssignments) {
        currentOut.tsvAssignments.forEach(a => {
            const idx = parseInt(String(a.netId).replace(/net/i, ''), 10);
            if (idx === netIdx) tsvList.push(a);
        });
    }

    const minX = Array(numDies).fill(Infinity);
    const minY = Array(numDies).fill(Infinity);
    const maxX = Array(numDies).fill(-Infinity);
    const maxY = Array(numDies).fill(-Infinity);
    const has = Array(numDies).fill(false);

    function upd(d, px, py) {
        if (d < 0 || d >= numDies) return;
        minX[d] = Math.min(minX[d], px);
        minY[d] = Math.min(minY[d], py);
        maxX[d] = Math.max(maxX[d], px);
        maxY[d] = Math.max(maxY[d], py);
        has[d] = true;
    }

    terms.forEach(name => {
        if (typeof name !== 'string') return;
        if (name.startsWith('sb') || name.startsWith('r')) {
            if (!moduleMap.has(name)) return;
            const m = moduleMap.get(name);
            upd(m.dieId, m.cx, m.cy);
        } else if (name.startsWith('p')) {
            if (!terminalMap.has(name)) return;
            const t = terminalMap.get(name);
            upd(0, t.cx, t.cy);
        }
    });

    tsvList.forEach(tsv => {
        upd(tsv.tierLo, tsv.x, tsv.y);
        upd(tsv.tierHi, tsv.x, tsv.y);
    });

    const perDie = [];
    for (let d = 0; d < numDies; d++) {
        perDie.push({ has: has[d], minX: minX[d], minY: minY[d], maxX: maxX[d], maxY: maxY[d] });
    }
    return { perDie };
}

// TSV hover：畫出該 net 在相鄰兩層的 bounding box（單點則放大為可見方塊）
function drawTsvHoverNetAdjacentBBoxes(toCanvasX, toCanvasY, scale, hoveredTsv, extents) {
    if (!extents || !hoveredTsv || !extents.perDie) return;

    const EPS = 1e-7;
    const POINT_HALF_W = 3; // world coords：單一幾何點時的半邊長
    const uniqueTiers = [...new Set([hoveredTsv.tierLo, hoveredTsv.tierHi])].sort((a, b) => a - b);

    ctx.save();
    for (const d of uniqueTiers) {
        if (d < 0 || d >= extents.perDie.length) continue;
        const ext = extents.perDie[d];
        if (!ext.has) continue;

        let x1 = ext.minX, y1 = ext.minY, x2 = ext.maxX, y2 = ext.maxY;
        const degenerate = (x2 - x1) < EPS && (y2 - y1) < EPS;
        if (degenerate) {
            const cx = (x1 + x2) / 2;
            const cy = (y1 + y2) / 2;
            x1 = cx - POINT_HALF_W;
            x2 = cx + POINT_HALF_W;
            y1 = cy - POINT_HALF_W;
            y2 = cy + POINT_HALF_W;
        }

        const p = getPalette(d);
        const bx = toCanvasX(x1);
        const by = toCanvasY(y2);
        const bw = Math.max((x2 - x1) * scale, 4);
        const bh = Math.max((y2 - y1) * scale, 4);

        ctx.fillStyle = p.fill.replace('0.45', '0.14');
        ctx.fillRect(bx, by, bw, bh);
        ctx.strokeStyle = p.outline;
        ctx.lineWidth = degenerate ? 2.2 : 2;
        ctx.setLineDash(degenerate ? [3, 3] : [10, 5]);
        ctx.strokeRect(bx, by, bw, bh);

        ctx.setLineDash([]);
        ctx.font = 'bold 11px Arial';
        ctx.fillStyle = p.outline;
        const tag = degenerate ? `Die ${d} (1 pt)` : `Die ${d} net bbox`;
        ctx.fillText(tag, bx + 4, Math.max(by + 14, 14));
    }
    ctx.restore();
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
            d.blocks.forEach(b => {
                wMinX = Math.min(wMinX, b.x);
                wMinY = Math.min(wMinY, b.y);
                wMaxX = Math.max(wMaxX, b.x + b.width);
                wMaxY = Math.max(wMaxY, b.y + b.height);
            });
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

    // 記錄目前視窗轉換參數（供 mousedown/mousemove 互動使用）
    currentView = { wMinX, wMinY, ch, scale };

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
            if (showTsvLabels && scale >= 1.5 && sw > 16 && sh > 8) {
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

            d.blocks.forEach((b, bi) => {
                const bx = toCanvasX(b.x);
                const by = toCanvasY(b.y + b.height);
                const bw = b.width  * scale;
                const bh = b.height * scale;

                ctx.fillStyle   = p.fill;
                ctx.fillRect(bx, by, bw, bh);
                ctx.strokeStyle = p.stroke;
                ctx.lineWidth   = 1;
                ctx.strokeRect(bx, by, bw, bh);

                // Selected module highlight
                if (selected && selected.type === 'module' &&
                    selected.dieId === id && selected.blockIndex === bi) {
                    ctx.strokeStyle = 'rgba(34,197,94,1)'; // green
                    ctx.lineWidth   = 3;
                    ctx.strokeRect(bx, by, bw, bh);
                    ctx.lineWidth   = 1;
                }

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

    // ── 2.6. 畫 Intra-die Nets（.nets）──
    if (showNets && netsData && netsData.nets && outData) {
        drawIntraDieNets(toCanvasX, toCanvasY);
    }

    // ── 2.5. TSV hover：該 net 在上下相鄰層的 bounding box（需 .nets）
    if (hoveredTsv && outData && blockData && netsData && netsData.nets) {
        const netIdx = parseInt(String(hoveredTsv.netId).replace(/net/i, ''), 10);
        if (Number.isFinite(netIdx) && netIdx >= 0 && netIdx < netsData.nets.length) {
            const extents = computeNetPerDieExtents(outData, blockData, netsData, netIdx);
            if (extents) {
                drawTsvHoverNetAdjacentBBoxes(toCanvasX, toCanvasY, scale, hoveredTsv, extents);
            }
        }
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

        const selDie = sel === 'all' ? -1 : parseInt(sel, 10);
        const filteredAssigns = [];
        outData.tsvAssignments.forEach((a, assignmentIndex) => {
            const shown = selDie === -1
                ? true
                : (tsvLowerTierOnly
                    ? a.tierLo === selDie
                    : (a.tierLo === selDie || a.tierHi === selDie));
            if (!shown) return;
            filteredAssigns.push({ a, assignmentIndex });
        });

        filteredAssigns.forEach(({ a, assignmentIndex }) => {
            const c  = TSV_TIER_COLORS[a.tierLo] ||
                       TSV_TIER_COLORS[TSV_TIER_COLORS.length - 1];

            const isHovered = hoveredTsv && hoveredTsv.netId === a.netId &&
                              hoveredTsv.tierLo === a.tierLo;
            const isSelected = selected && selected.type === 'tsv' && selected.assignmentIndex === assignmentIndex;

            // TSV 實際佔地：世界座標 (x±1.5, y±1.5)，畫布對應 3*scale 大小
            let tsv_size = 3;
            const sqX = toCanvasX(a.x - tsv_size / 2);
            const sqY = toCanvasY(a.y + tsv_size / 2);   // y+1.5 = world 上方 = canvas 上方
            const sqW = Math.max(tsv_size, tsv_size * scale);
            const sqH = Math.max(tsv_size, tsv_size * scale);

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

            // Selected TSV highlight
            if (isSelected) {
                ctx.strokeStyle = 'rgba(239,68,68,1)'; // red
                ctx.lineWidth   = 3;
                ctx.strokeRect(sqX, sqY, sqW, sqH);
                ctx.lineWidth   = 1;
            }

            // hover 或高 zoom 時顯示 net 名稱
            if (showTsvLabels && (isHovered || scale >= 2.5)) {
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

// ── Intra-die Net 連線 ───────────────────────────────────────────────────────
function drawIntraDieNets(toCanvasX, toCanvasY) {
    if (!netsData || !outData) return;

    // 建立 moduleName → {cx, cy, dieId} 對照表
    const modulePos = new Map();
    Object.keys(outData.dies).forEach(dStr => {
        const dId = parseInt(dStr, 10);
        outData.dies[dId].blocks.forEach(b => {
            modulePos.set(b.name, {
                cx: b.x + b.width  / 2,
                cy: b.y + b.height / 2,
                dieId: dId,
            });
        });
    });

    // 建立 terminalName → {cx, cy, dieId=0} 對照表（I/O pin 歸屬 die 0）
    const terminalPos = new Map();
    if (blockData && blockData.terminals) {
        blockData.terminals.forEach(t => {
            terminalPos.set(t.name, { cx: t.x, cy: t.y, dieId: 0 });
        });
    }

    // 目前選取的 die（-1 代表 all）
    const selDie = (() => {
        const v = document.getElementById('die-select').value;
        return v === 'all' ? -1 : parseInt(v, 10);
    })();

    ctx.save();
    ctx.lineWidth = 0.8;

    netsData.nets.forEach(terms => {
        if (!terms || terms.length < 2) return;

        const pins = [];
        let netDie = -1;
        let valid  = true;

        for (const name of terms) {
            let pos = modulePos.get(name) ?? terminalPos.get(name) ?? null;
            if (!pos) { valid = false; break; }
            if (netDie === -1) netDie = pos.dieId;
            else if (pos.dieId !== netDie) { valid = false; break; }  // 跨層 → 跳過
            pins.push(pos);
        }

        if (!valid || pins.length < 2) return;
        if (selDie !== -1 && netDie !== selDie) return;  // 只畫目前選取層

        // 使用該層 palette 顏色（半透明）
        const p = getPalette(netDie);
        ctx.strokeStyle = p.stroke;
        ctx.globalAlpha = 0.3;

        if (pins.length === 2) {
            // 兩端點直連
            ctx.beginPath();
            ctx.moveTo(toCanvasX(pins[0].cx), toCanvasY(pins[0].cy));
            ctx.lineTo(toCanvasX(pins[1].cx), toCanvasY(pins[1].cy));
            ctx.stroke();
        } else {
            // 多端點：star topology（各端點連至重心）
            const gcx = pins.reduce((s, q) => s + q.cx, 0) / pins.length;
            const gcy = pins.reduce((s, q) => s + q.cy, 0) / pins.length;
            const ccx = toCanvasX(gcx);
            const ccy = toCanvasY(gcy);
            for (const pin of pins) {
                ctx.beginPath();
                ctx.moveTo(ccx, ccy);
                ctx.lineTo(toCanvasX(pin.cx), toCanvasY(pin.cy));
                ctx.stroke();
            }
        }
    });

    ctx.globalAlpha = 1;
    ctx.restore();
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

// ── Legalize Process Replay ────────────────────────────────────────────────────

// 解析 legalize_process.txt，解析所有 Tier
// 回傳：
//   tierData  : { [tierNum]: { steps:[{label,moves}], moves:[扁平單步] } }
//   tier0Steps / tier0Moves : 向下相容，等同 tierData[0]
function parseLegalizeProcess(text) {
    const lines = text.split(/\r?\n/);
    const tierData = {};
    let cur = null;

    for (const rawLine of lines) {
        const line = rawLine.trim();
        const tierMatch = line.match(/^Tier\s+(\d+)\s+\[.+?\]/);
        if (tierMatch) {
            if (cur) {
                const td = tierData[cur.tier] || (tierData[cur.tier] = { steps: [], moves: [] });
                td.steps.push({ label: cur.label, moves: cur.moves });
            }
            cur = { tier: parseInt(tierMatch[1], 10), label: line, moves: [] };
            continue;
        }
        if (!cur) continue;
        const m = line.match(/\[move_apply\]\s+module_id=(\d+)\s+center=\(([^,]+),\s*([^)]+)\).*?rot90=(\d+)/);
        if (m) {
            cur.moves.push({
                module_id: parseInt(m[1], 10),
                cx:        parseFloat(m[2]),
                cy:        parseFloat(m[3]),
                rot90:     parseInt(m[4], 10),
            });
        }
    }
    if (cur) {
        const td = tierData[cur.tier] || (tierData[cur.tier] = { steps: [], moves: [] });
        td.steps.push({ label: cur.label, moves: cur.moves });
    }

    // 每個 tier 展平為單步陣列，附加 pass 資訊
    Object.keys(tierData).forEach(tierNum => {
        const td = tierData[tierNum];
        td.steps.forEach((step, passIdx) => {
            step.moves.forEach((mv, moveIdxInPass) => {
                td.moves.push({
                    ...mv,
                    passIdx,
                    passLabel:     step.label,
                    moveIdxInPass,
                    passTotal:     step.moves.length,
                });
            });
        });
    });

    const td0 = tierData[0] || { steps: [], moves: [] };
    return { tierData, tier0Steps: td0.steps, tier0Moves: td0.moves };
}

// 依 outData.dies[dieId] 建立 module_id -> blockIndex 映射
function buildDie0ModuleMap(dieId = 0) {
    die0ModuleMap = new Map();
    const die = outData && outData.dies && outData.dies[dieId];
    if (!die) return;
    die.blocks.forEach((b, bi) => {
        const m = b.name.match(/(\d+)$/);
        if (m) die0ModuleMap.set(parseInt(m[1], 10), bi);
    });
}

// 從 blockData.modules 建立 module_id -> {w, h}（目前僅供參考，旋轉已改用 relative 邏輯）
function buildLegalizeBaseDims() {
    legalizeBaseDims = {};
    if (!blockData || !blockData.modules) return;
    blockData.modules.forEach(mod => {
        const m = mod.name.match(/(\d+)$/);
        if (m) legalizeBaseDims[parseInt(m[1], 10)] = { w: mod.w, h: mod.h };
    });
}

// 重置 replay 到初始狀態，tier 預設回 0
function resetLegalizeReplay() {
    legalizeCurrentTier = 0;
    buildDie0ModuleMap(0);
    legalizeStepIndex = -1;
    legalizeCache = [];
    const die = outData && outData.dies && outData.dies[0];
    if (die) legalizeCache[0] = die.blocks.map(b => ({ ...b }));
    updateLegalizeUI();
}

// 切換 replay 到指定 tier（換 die 時呼叫）
function switchLegalizeTier(tierNum) {
    if (!legalizeData) return;
    const td = legalizeData.tierData[tierNum];
    if (!td || td.moves.length === 0) return; // 沒有該 tier 的資料就不切換

    legalizeCurrentTier = tierNum;
    buildDie0ModuleMap(tierNum);
    legalizeStepIndex = -1;
    legalizeCache = [];
    const die = outData && outData.dies && outData.dies[tierNum];
    if (die) legalizeCache[0] = die.blocks.map(b => ({ ...b }));
    selected = null;
    syncSelectedModuleCenterInputs();
    updateLegalizeUI();
    redraw();
    updateInfoPanel();
}

// 套用單一 move 到 outData.dies[legalizeCurrentTier]
function applyLegalizeSingleMove(move) {
    const die = outData && outData.dies && outData.dies[legalizeCurrentTier];
    if (!die) return;
    const bi = die0ModuleMap.get(move.module_id);
    if (bi === undefined) return;
    const b = die.blocks[bi];

    // 旋轉處理（relative rot90：1=swap，0=不動）
    if (move.rot90 === 1) {
        const tmp = b.width; b.width = b.height; b.height = tmp;
    }

    b.x = move.cx - b.width  / 2;
    b.y = move.cy - b.height / 2;
}

// 前進一個 [move_apply]
function legalizeGoForward() {
    const td = legalizeData && legalizeData.tierData[legalizeCurrentTier];
    if (!td || !outData || !outData.dies || !outData.dies[legalizeCurrentTier]) return;
    const moves = td.moves;
    if (legalizeStepIndex >= moves.length - 1) return;

    legalizeStepIndex++;
    const die = outData.dies[legalizeCurrentTier];

    if (legalizeCache[legalizeStepIndex + 1]) {
        die.blocks = legalizeCache[legalizeStepIndex + 1].map(b => ({ ...b }));
    } else {
        const prev = legalizeCache[legalizeStepIndex];
        if (prev) die.blocks = prev.map(b => ({ ...b }));
        applyLegalizeSingleMove(moves[legalizeStepIndex]);
        legalizeCache[legalizeStepIndex + 1] = die.blocks.map(b => ({ ...b }));
    }

    selectCurrentLegalizeModule();
    syncSelectedModuleCenterInputs();
    updateLegalizeUI();
    redraw();
    updateInfoPanel();
}

// 後退一個 [move_apply]
function legalizeGoBack() {
    if (!legalizeData || legalizeStepIndex < 0) return;
    const die = outData && outData.dies && outData.dies[legalizeCurrentTier];
    if (!die) return;

    legalizeStepIndex--;
    const cached = legalizeCache[legalizeStepIndex + 1];
    if (cached) die.blocks = cached.map(b => ({ ...b }));

    selectCurrentLegalizeModule();
    syncSelectedModuleCenterInputs();
    updateLegalizeUI();
    redraw();
    updateInfoPanel();
}

// 將目前步驟的 module 設為 selected（綠色高亮）
function selectCurrentLegalizeModule() {
    const td = legalizeData && legalizeData.tierData[legalizeCurrentTier];
    const die = outData && outData.dies && outData.dies[legalizeCurrentTier];
    if (!td || legalizeStepIndex < 0 || !die) { selected = null; return; }
    const mv = td.moves[legalizeStepIndex];
    const bi = die0ModuleMap.get(mv.module_id);
    if (bi === undefined) { selected = null; return; }
    selected = { type: 'module', dieId: legalizeCurrentTier, blockIndex: bi, name: die.blocks[bi].name };
}

// 更新 Replay ctrl-group 顯示
function updateLegalizeUI() {
    const grp     = document.getElementById('legalizePlaybackGroup');
    const backBtn = document.getElementById('legalizeBackBtn');
    const fwdBtn  = document.getElementById('legalizeForwardBtn');
    const info    = document.getElementById('legalizeStepInfo');
    if (!grp) return;

    if (!legalizeData) { grp.style.display = 'none'; return; }
    grp.style.display = '';

    const td = legalizeData.tierData[legalizeCurrentTier];
    if (!td || td.moves.length === 0) {
        backBtn.disabled = true;
        fwdBtn.disabled  = true;
        info.textContent = `Tier ${legalizeCurrentTier}: no data`;
        return;
    }

    const moves = td.moves;
    const N     = moves.length;
    backBtn.disabled = (legalizeStepIndex < 0);
    fwdBtn.disabled  = (legalizeStepIndex >= N - 1);

    if (legalizeStepIndex < 0) {
        info.textContent = `Tier ${legalizeCurrentTier} initial  (0 / ${N} moves, ${td.steps.length} passes)`;
        return;
    }

    const mv = moves[legalizeStepIndex];
    const pm = mv.passLabel.match(/(\[.+?\]\s+init_w=[\d.]+)/);
    const passShort = pm ? pm[1] : mv.passLabel.replace(/^Tier\s+\d+\s+/, '');
    const passIdx1  = mv.passIdx + 1;
    const passTotal = td.steps.length;
    const die = outData && outData.dies && outData.dies[legalizeCurrentTier];
    const bi  = die0ModuleMap.get(mv.module_id);
    const blockName = (die && bi !== undefined) ? die.blocks[bi].name : `id=${mv.module_id}`;

    info.innerHTML =
        `<b>Tier ${legalizeCurrentTier} &nbsp; Pass ${passIdx1}/${passTotal}</b> ${passShort}<br>` +
        `move ${mv.moveIdxInPass + 1}/${mv.passTotal} &nbsp;` +
        `<span style="color:#888">(step ${legalizeStepIndex + 1}/${N})</span> &nbsp;` +
        `<b style="color:#16a34a">▶ ${blockName}</b>`;
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
