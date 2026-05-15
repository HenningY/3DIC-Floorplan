"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || (function () {
    var ownKeys = function(o) {
        ownKeys = Object.getOwnPropertyNames || function (o) {
            var ar = [];
            for (var k in o) if (Object.prototype.hasOwnProperty.call(o, k)) ar[ar.length] = k;
            return ar;
        };
        return ownKeys(o);
    };
    return function (mod) {
        if (mod && mod.__esModule) return mod;
        var result = {};
        if (mod != null) for (var k = ownKeys(mod), i = 0; i < k.length; i++) if (k[i] !== "default") __createBinding(result, mod, k[i]);
        __setModuleDefault(result, mod);
        return result;
    };
})();
Object.defineProperty(exports, "__esModule", { value: true });
exports.activate = activate;
exports.deactivate = deactivate;
const fs = __importStar(require("fs"));
const path = __importStar(require("path"));
const vscode = __importStar(require("vscode"));
const parsers_1 = require("./parsers");
const EXT_ID = "floorplanViewer";
/**
 * 專案根目錄：底下有 `PA2_3DIC/input_pa2`（例如 .../PD）。
 * 若工作區開在 `floorplan-viewer-extension` 或 `PA2_3DIC`，會從路徑往上找。
 */
function resolvePa2ProjectRoot(workspaceRoot, extensionPath) {
    const seeds = [];
    if (workspaceRoot) {
        let cur = path.resolve(workspaceRoot);
        for (let i = 0; i < 10; i++) {
            seeds.push(cur);
            cur = path.dirname(cur);
        }
    }
    let ext = path.resolve(extensionPath);
    for (let i = 0; i < 12; i++) {
        seeds.push(ext);
        ext = path.dirname(ext);
    }
    const uniq = [...new Set(seeds)];
    const inputDirMarker = path.join("PA2_3DIC", "input_pa2");
    for (const r of uniq) {
        if (fs.existsSync(path.join(r, inputDirMarker))) {
            return r;
        }
    }
    return workspaceRoot ? path.resolve(workspaceRoot) : undefined;
}
function quotePath(p) {
    if (process.platform === "win32") {
        return `"${p.replace(/"/g, '\\"')}"`;
    }
    return `'${p.replace(/'/g, "'\\''")}'`;
}
function resolveAnalyticalDir(config, extensionPath) {
    const custom = config
        .get("floorplanViewer.analyticalDirectory")
        ?.trim();
    if (custom) {
        const abs = path.resolve(custom);
        if (fs.existsSync(abs)) {
            return abs;
        }
    }
    const folders = vscode.workspace.workspaceFolders;
    const ws0 = folders?.[0]?.uri.fsPath;
    const seeds = [];
    if (ws0) {
        let cur = path.resolve(ws0);
        for (let i = 0; i < 10; i++) {
            seeds.push(cur);
            cur = path.dirname(cur);
        }
    }
    if (extensionPath) {
        let cur = path.resolve(extensionPath);
        for (let i = 0; i < 12; i++) {
            seeds.push(cur);
            cur = path.dirname(cur);
        }
    }
    const uniq = [...new Set(seeds)];
    const exe = process.platform === "win32" ? "analytical.exe" : "analytical";
    const candidateDirs = [];
    for (const root of uniq) {
        candidateDirs.push(path.join(root, "PA2_3DIC", "analytical"), path.join(root, "PD", "PA2_3DIC", "analytical"), path.join(root, "analytical"));
    }
    for (const c of candidateDirs) {
        if (fs.existsSync(path.join(c, exe))) {
            return c;
        }
    }
    return undefined;
}
function getHtml(webview) {
    const csp = [
        "default-src 'none';",
        `style-src ${webview.cspSource} 'unsafe-inline';`,
        `script-src ${webview.cspSource} 'unsafe-inline';`,
        `img-src ${webview.cspSource} data:;`,
    ].join(" ");
    return `<!DOCTYPE html>
<html lang="zh-Hant">
<head>
  <meta charset="UTF-8" />
  <meta http-equiv="Content-Security-Policy" content="${csp}" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>3DIC Floorplan 2D Viewer</title>
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0; padding: 0;
      font-family: var(--vscode-font-family);
      font-size: 13px;
      color: var(--vscode-foreground);
      background: var(--vscode-editor-background);
      display: flex; height: 100vh; overflow: hidden;
    }

    /* ── 三欄版型 ── */
    .layout { display: flex; flex: 1; min-width: 0; }

    /* 可拖曳分隔線 */
    .resizer {
      width: 3px; flex-shrink: 0; cursor: col-resize;
      background: var(--vscode-panel-border);
      transition: background 0.15s;
    }
    .resizer:hover, .resizer.dragging { background: var(--vscode-focusBorder, #007fd4); }

    /* 左側面板 */
    .left-panel {
      flex: 0 0 280px; min-width: 200px;
      display: flex; flex-direction: column;
      overflow: hidden;
    }
    .left-panel .panel-header {
      padding: 7px 10px 5px;
      font-size: 11px; font-weight: 600;
      text-transform: uppercase; letter-spacing: 0.06em;
      opacity: 0.7; border-bottom: 1px solid var(--vscode-panel-border);
    }
    .file-section { padding: 8px 8px 4px; display: flex; flex-direction: column; gap: 6px; }
    .field-row { display: flex; flex-direction: column; gap: 2px; }
    .field-row label { font-size: 11px; opacity: 0.75; }
    .input-group { display: flex; gap: 3px; }
    .input-group input {
      flex: 1; min-width: 0; padding: 3px 6px;
      background: var(--vscode-input-background);
      color: var(--vscode-input-foreground);
      border: 1px solid var(--vscode-input-border, #444);
      border-radius: 2px; font-size: 11px;
    }
    select.preset-select {
      width: 100%; padding: 4px 6px; font-size: 11px;
      background: var(--vscode-dropdown-background);
      color: var(--vscode-dropdown-foreground);
      border: 1px solid var(--vscode-input-border, #444);
      border-radius: 2px;
    }
    .btn-row { display: flex; flex-direction: column; gap: 4px; padding: 4px 8px 8px; }
    button {
      padding: 4px 10px; cursor: pointer;
      background: var(--vscode-button-background);
      color: var(--vscode-button-foreground);
      border: none; border-radius: 2px;
      white-space: nowrap; font-size: 12px;
      text-align: center;
    }
    button.secondary {
      background: var(--vscode-button-secondaryBackground);
      color: var(--vscode-button-secondaryForeground);
    }
    button:disabled { opacity: 0.5; cursor: not-allowed; }
    button.browse-btn { padding: 3px 7px; font-size: 11px; flex-shrink: 0; }
    .divider { border: none; border-top: 1px solid var(--vscode-panel-border); margin: 0; }
    .log-section { flex: 1; display: flex; flex-direction: column; min-height: 0; }
    .log-section .panel-header { flex-shrink: 0; }
    #log {
      flex: 1; margin: 0; padding: 6px 8px; overflow: auto;
      font-family: var(--vscode-editor-font-family);
      font-size: 11px; line-height: 1.4;
      white-space: pre-wrap; word-break: break-all;
    }

    /* 中間面板 */
    .center-panel {
      flex: 1; min-width: 0;
      display: flex; flex-direction: column;
      position: relative;
    }
    .canvas-toolbar {
      height: 32px; flex-shrink: 0;
      display: flex; align-items: center;
      justify-content: space-between;
      padding: 0 8px;
      border-bottom: 1px solid var(--vscode-panel-border);
      position: relative;
    }
    .canvas-toolbar-left, .canvas-toolbar-right {
      display: flex; align-items: center; gap: 4px; position: relative;
    }
    .toolbar-btn {
      padding: 3px 10px; font-size: 12px;
      background: var(--vscode-button-secondaryBackground);
      color: var(--vscode-button-secondaryForeground);
      border: 1px solid var(--vscode-panel-border);
      border-radius: 3px; cursor: pointer;
    }
    .toolbar-btn:hover { opacity: 0.85; }

    /* 浮動面板 */
    .float-panel {
      position: absolute; top: 30px; left: 0;
      background: var(--vscode-editor-background);
      border: 1px solid var(--vscode-panel-border);
      border-radius: 4px; padding: 6px 0;
      z-index: 200; min-width: 160px;
      box-shadow: 0 4px 14px rgba(0,0,0,0.45);
      display: none;
    }
    .float-panel.open { display: block; }
    .float-panel.panel-right { left: auto; right: 0; }
    .float-panel .fp-title {
      font-size: 10px; font-weight: 600;
      text-transform: uppercase; letter-spacing: 0.06em;
      opacity: 0.55; padding: 2px 12px 5px;
    }
    .fp-option {
      padding: 5px 12px; font-size: 12px; cursor: pointer;
      display: flex; align-items: center; gap: 7px;
      user-select: none;
    }
    .fp-option:hover { background: var(--vscode-list-hoverBackground); }
    .fp-option.active { color: var(--vscode-button-background); font-weight: 600; }
    .fp-option .tier-dot {
      display: inline-block; width: 9px; height: 9px;
      border-radius: 2px; flex-shrink: 0;
    }
    .fp-sep { border: none; border-top: 1px solid var(--vscode-panel-border); margin: 4px 0; }
    .fp-check {
      padding: 4px 12px; font-size: 12px;
      display: flex; align-items: center; gap: 7px;
      cursor: pointer; user-select: none;
    }
    .fp-check:hover { background: var(--vscode-list-hoverBackground); }
    .fp-check input[type=checkbox] { cursor: pointer; }
    .color-dot {
      display: inline-block; width: 9px; height: 9px; flex-shrink: 0; border-radius: 1px;
    }

    #canvas-container {
      flex: 1; overflow: hidden; background: #1a1e24; position: relative;
    }
    #canvas-container svg { display: block; }
    .placeholder {
      position: absolute; top: 50%; left: 50%;
      transform: translate(-50%,-50%);
      opacity: 0.35; font-size: 13px; text-align: center; pointer-events: none;
    }

    /* 右側面板 */
    .right-panel {
      flex: 0 0 350px; min-width: 250px;
      display: flex; flex-direction: column;
    }
    .right-panel .panel-header {
      padding: 7px 10px 5px;
      font-size: 11px; font-weight: 600;
      text-transform: uppercase; letter-spacing: 0.06em;
      opacity: 0.7; border-bottom: 1px solid var(--vscode-panel-border);
    }
    .right-placeholder {
      flex: 1; display: flex; align-items: center; justify-content: center;
      opacity: 0.25; font-size: 12px;
    }
    /* 右側操作面板 */
    .rp-body { flex: 1; display: flex; flex-direction: column; overflow-y: auto; }
    .rp-hint { flex: 1; display: flex; align-items: center; justify-content: center; opacity: 0.3; font-size: 12px; padding: 12px; text-align: center; }
    .rp-section { padding: 10px 10px 6px; border-bottom: 1px solid var(--vscode-panel-border); }
    .rp-name-row { display: flex; align-items: baseline; gap: 8px; flex-wrap: wrap; margin-bottom: 8px; }
    .rp-modname { font-size: 12px; font-weight: 600; color: var(--vscode-button-background); white-space: nowrap; overflow: hidden; text-overflow: ellipsis; max-width: 100%; flex: 1; min-width: 0; }
    .rp-size-inline { font-size: 11px; opacity: 0.75; white-space: nowrap; flex-shrink: 0; }
    .rp-row-rotate-xy { display: flex; align-items: center; gap: 6px; margin-bottom: 8px; }
    .rp-row-rotate-xy #btnRotate { flex: 0 0 auto; padding: 4px 8px; font-size: 11px; }
    .rp-row-rotate-xy input[type=number] {
      flex: 1; min-width: 0; padding: 4px 6px;
      background: var(--vscode-input-background);
      color: var(--vscode-input-foreground);
      border: 1px solid var(--vscode-input-border, #444);
      border-radius: 2px; font-size: 12px;
    }
    .rp-field { display: flex; flex-direction: column; gap: 2px; margin-bottom: 6px; }
    .rp-field label { font-size: 11px; opacity: 0.75; }
    .rp-field input[type=number] {
      padding: 3px 6px; width: 100%;
      background: var(--vscode-input-background);
      color: var(--vscode-input-foreground);
      border: 1px solid var(--vscode-input-border, #444);
      border-radius: 2px; font-size: 12px;
    }
    .rp-btn-row { display: flex; gap: 4px; flex-wrap: wrap; }
    .rp-section-title { font-size: 10px; font-weight: 600; text-transform: uppercase; letter-spacing: 0.06em; opacity: 0.55; margin-bottom: 6px; }
    /* urx/ury 唯讀座標 */
    .rp-row-ur { display: flex; align-items: center; gap: 5px; margin-bottom: 6px; flex-wrap: wrap; }
    .rp-coord-label { font-size: 10px; opacity: 0.55; flex-shrink: 0; }
    .rp-coord-val { font-size: 12px; font-weight: 600; min-width: 40px; font-variant-numeric: tabular-nums; color: var(--vscode-descriptionForeground); }
    /* TSV 邊長調整 */
    .fp-tsv-size {
      display: flex; align-items: center; gap: 6px;
      padding: 2px 4px 4px 24px; font-size: 11px; opacity: 0.85;
    }
    .fp-tsv-size input {
      width: 46px; padding: 2px 4px; font-size: 11px;
      background: var(--vscode-input-background);
      color: var(--vscode-input-foreground);
      border: 1px solid var(--vscode-input-border, #444); border-radius: 2px;
    }
    /* Result 統計 */
    .rp-stat-row { display: flex; align-items: baseline; gap: 8px; padding: 3px 0; }
    .rp-stat-label { font-size: 10px; opacity: 0.6; width: 40px; flex-shrink: 0; }
    .rp-stat-value { font-size: 12px; font-weight: 600; font-variant-numeric: tabular-nums; word-break: break-all; }
    .module-rect { cursor: move; }
    /* Constraint list */
    .cst-section { padding: 8px 10px 4px; border-bottom: 1px solid var(--vscode-panel-border); }
    .cst-row {
      display: flex; align-items: flex-start; gap: 4px;
      padding: 5px 0; border-bottom: 1px solid rgba(128,128,128,0.12);
    }
    .cst-row:last-child { border-bottom: none; }
    .cst-badge {
      flex-shrink: 0; width: 16px; height: 16px;
      border-radius: 3px; font-size: 10px; font-weight: 700;
      display: flex; align-items: center; justify-content: center;
      margin-top: 2px;
    }
    .cst-badge-fixed   { background: #a06010; color: #fff; }
    .cst-badge-repulse { background: #226688; color: #fff; }
    /* 可編輯欄位 */
    .cst-fields { flex: 1; display: flex; flex-wrap: wrap; gap: 3px; min-width: 0; }
    .cst-inp {
      padding: 2px 4px; font-size: 11px;
      background: var(--vscode-input-background);
      color: var(--vscode-input-foreground);
      border: 1px solid var(--vscode-input-border, #444);
      border-radius: 2px; min-width: 0;
    }
    .cst-inp-name     { width: 62px; }
    .cst-inp-coord    { width: 36px; }
    .cst-inp-strength { width: 40px; }
    .cst-inp-names    { flex: 1; min-width: 70px; }
    .cst-confirm {
      flex-shrink: 0; padding: 1px 5px; font-size: 11px; cursor: pointer;
      background: transparent;
      color: var(--vscode-terminal-ansiGreen, #4ec9b0);
      border: 1px solid currentColor; border-radius: 2px; line-height: 1; margin-top: 2px;
    }
    .cst-confirm:hover { background: var(--vscode-terminal-ansiGreen, #4ec9b0); color: #000; }
    .cst-del {
      flex-shrink: 0; padding: 1px 5px; font-size: 11px; cursor: pointer;
      background: transparent;
      color: var(--vscode-errorForeground, #f48771);
      border: 1px solid currentColor; border-radius: 2px;
      line-height: 1; margin-top: 2px;
    }
    .cst-del:hover { background: var(--vscode-errorForeground, #f48771); color: #fff; }
    /* 新增列 */
    .cst-add-row { display: flex; align-items: center; gap: 5px; padding: 7px 0 2px; }
    .cst-add-type {
      flex: 1; padding: 3px 4px; font-size: 11px;
      background: var(--vscode-dropdown-background);
      color: var(--vscode-dropdown-foreground);
      border: 1px solid var(--vscode-dropdown-border, #444); border-radius: 2px;
    }
    .cst-add-btn {
      flex-shrink: 0; padding: 3px 8px; font-size: 11px; cursor: pointer;
    }
  </style>
</head>
<body>
<div class="layout">

  <!-- ── 左側面板 ── -->
  <div class="left-panel" id="leftPanel">
    <div class="panel-header">File / Run</div>
    <div class="file-section">
      <div class="field-row">
        <label>Default set</label>
        <select id="presetSelect" class="preset-select">
          <option value="">— Custom —</option>
        </select>
      </div>
      <div class="field-row">
        <label>.block file</label>
        <div class="input-group">
          <input type="text" id="blockPath" readonly placeholder="Select .block" />
          <button class="browse-btn secondary" id="pickBlock">…</button>
        </div>
      </div>
      <div class="field-row">
        <label>.nets file</label>
        <div class="input-group">
          <input type="text" id="netsPath" readonly placeholder="Select .nets" />
          <button class="browse-btn secondary" id="pickNets">…</button>
        </div>
      </div>
      <div class="field-row">
        <label>Output file path</label>
        <div class="input-group">
          <input type="text" id="outPath" placeholder="./output/n100_output.txt" />
        </div>
      </div>
      <div class="field-row">
        <label>.constraint (optional)</label>
        <div class="input-group">
          <input type="text" id="constraintPath" readonly placeholder="None" />
          <button class="browse-btn secondary" id="pickConstraint">…</button>
        </div>
      </div>
    </div>
    <div class="btn-row">
      <button id="runFloorplan">▶ Run Floorplan</button>
      <button class="secondary" id="reload2d">Reload 2D View</button>
    </div>
    <hr class="divider" />
    <div class="log-section">
      <div class="panel-header">Status / Log</div>
      <pre id="log"></pre>
    </div>
    <!-- Result 統計 -->
    <div id="rpResultSection" style="display:none">
      <hr class="divider" />
      <div class="panel-header">Result</div>
      <div style="padding: 4px 10px 8px">
        <div class="rp-stat-row">
          <span class="rp-stat-label">HPWL</span>
          <span class="rp-stat-value" id="rpHpwl">—</span>
        </div>
        <div class="rp-stat-row">
          <span class="rp-stat-label">Die</span>
          <span class="rp-stat-value" id="rpDie">—</span>
        </div>
        <div class="rp-stat-row">
          <span class="rp-stat-label">Time</span>
          <span class="rp-stat-value" id="rpTime">—</span>
        </div>
      </div>
    </div>
  </div>

  <div class="resizer" id="resizerL"></div>

  <!-- ── 中間面板 ── -->
  <div class="center-panel">
    <div class="canvas-toolbar">
      <!-- 左上：Tier 選擇 -->
      <div class="canvas-toolbar-left">
        <button class="toolbar-btn" id="btnTier">Tier ▾</button>
        <div id="panelTier" class="float-panel">
          <div class="fp-title">顯示層數</div>
          <div class="fp-option active" id="optOverview" data-mode="overview">
            <span class="tier-dot" style="background:#8899aa"></span>Overview（全覽）
          </div>
        </div>
      </div>
      <!-- 右上：View 選項 -->
      <div class="canvas-toolbar-right">
        <button class="toolbar-btn" id="btnView">View ▾</button>
        <div id="panelView" class="float-panel panel-right">
          <div class="fp-title">顯示項目</div>
          <label class="fp-check"><input type="checkbox" id="chkModules" checked /><span class="color-dot" style="background:#4a90d9;opacity:0.7"></span>Modules</label>
          <label class="fp-check"><input type="checkbox" id="chkTsvs" checked /><span class="color-dot" style="background:#ff5566"></span>TSVs</label>
          <div class="fp-tsv-size">
            <span>Side</span>
            <input type="number" id="tsvSize" value="3" min="0.5" max="20" step="0.5" title="TSV 邊長（floorplan 單位）" />
          </div>
          <label class="fp-check"><input type="checkbox" id="chkTerminals" checked /><span class="color-dot" style="background:#44cc88"></span>Terminals</label>
          <hr class="fp-sep" />
          <div class="fp-title">標籤</div>
          <label class="fp-check"><input type="checkbox" id="chkModuleLabels" />Module 名稱</label>
          <label class="fp-check"><input type="checkbox" id="chkTsvLabels" />TSV 名稱</label>
          <label class="fp-check"><input type="checkbox" id="chkTerminalLabels" />Terminal 名稱</label>
        </div>
      </div>
    </div>
    <div id="canvas-container">
      <div class="placeholder">請按「↺ 重新載入 2D」載入佈局</div>
    </div>
  </div>

  <div class="resizer" id="resizerR"></div>

  <!-- ── 右側面板 ── -->
  <div class="right-panel" id="rightPanel">
    <div class="panel-header">Operation</div>
    <div class="rp-body">
      <!-- Constraint 清單 -->
      <div id="rpConstraintSection" style="display:none">
        <div class="cst-section">
          <div class="rp-section-title">Constraints</div>
          <div id="constraintList"><span class="cst-empty">No constraints</span></div>
        </div>
      </div>
      <!-- 未選取時的提示 -->
      <div class="rp-hint" id="rpHint">點選畫布中的 module 以編輯</div>
      <!-- 選取 module 後顯示 -->
      <div id="rpModControls" style="display:none">
        <div class="rp-section">
          <div class="rp-section-title">Selected</div>
          <div class="rp-name-row">
            <span class="rp-modname" id="rpModName">—</span>
            <span class="rp-size-inline" id="rpSize">—</span>
          </div>
          <div class="rp-row-rotate-xy">
            <button id="btnRotate" title="Rotate 90°">Rotate 90°</button>
            <input type="number" id="rpInputX" step="0.5" placeholder="llx" title="lower-left X" />
            <input type="number" id="rpInputY" step="0.5" placeholder="lly" title="lower-left Y" />
          </div>
          <div class="rp-row-ur">
            <span class="rp-coord-label">urx</span>
            <span class="rp-coord-val" id="rpUrX">—</span>
            <span class="rp-coord-label">ury</span>
            <span class="rp-coord-val" id="rpUrY">—</span>
          </div>
          <div class="rp-btn-row">
            <button id="btnSetPos" style="flex:1">✓ Set</button>
            <button class="secondary" id="btnResetPos" style="flex:1">Reset</button>
            <button class="secondary" id="btnResetAll" style="flex:1">Reset All</button>
          </div>
        </div>
      </div>
    </div>
  </div>

</div>
  <script>
    const vscode = acquireVsCodeApi();
    const logEl = document.getElementById('log');

    function log(msg) {
      logEl.textContent += msg + '\\n';
      logEl.scrollTop = logEl.scrollHeight;
    }

    // ── 狀態 ──
    var sceneData = null;
    var currentMode = 'overview'; // 'overview' | 數字 tier index
    var moduleOverrides = {};     // { [name]: { xll, yll, xur, yur } } 暫存位置
    var selectedMod = null;       // 目前選取的 module name
    var currentTransform = null;  // 最新一次 makeTransform 的結果（用於座標轉換）
    var dragState = null;         // 拖曳狀態
    var constraintData = [];      // 已解析的 constraint 陣列
    var constraintFileLoaded = false; // 是否已載入 constraint 檔（用於顯示區塊）

    var TIER_FILLS   = ['#4a90d9','#5cb85c','#f0ad4e','#d9534f','#9b59b6','#1abc9c'];
    var TIER_STROKES = ['#2c6fa8','#3d8b3d','#c8892a','#b33030','#7d3f9c','#148a74'];

    function chk(id) { return document.getElementById(id).checked; }

    function escXml(s) {
      return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
    }

    // 取得 module 的有效位置（若有 override 則用 override）
    function getEffMod(m) {
      var ov = moduleOverrides[m.name];
      if (!ov) return m;
      return { name: m.name, tier: m.tier, xll: ov.xll, yll: ov.yll, xur: ov.xur, yur: ov.yur };
    }

    // Constraint 查詢
    function isFixed(name) {
      return constraintData.some(function(c) { return c.type === 'FIXED' && c.name === name; });
    }
    // 回傳 module 所屬的 REPULSE 群組編號（1-based），0 表示不屬於任何群組
    function repulseGroupIdx(name) {
      var groups = constraintData.filter(function(c) { return c.type === 'REPULSE'; });
      for (var gi = 0; gi < groups.length; gi++) {
        if (groups[gi].names && groups[gi].names.indexOf(name) >= 0) { return gi + 1; }
      }
      return 0;
    }

    function populatePresets(presets) {
      var sel = document.getElementById('presetSelect');
      while (sel.options.length > 1) {
        sel.remove(1);
      }
      (presets || []).forEach(function(p) {
        var o = document.createElement('option');
        o.value = JSON.stringify({ b: p.block, n: p.nets, o: p.output, c: p.constraint || '' });
        o.textContent = p.label;
        sel.appendChild(o);
      });
    }

    function syncPresetSelectFromPaths() {
      var sel = document.getElementById('presetSelect');
      var b = document.getElementById('blockPath').value;
      var n = document.getElementById('netsPath').value;
      var o = document.getElementById('outPath').value;
      var match = '';
      for (var i = 1; i < sel.options.length; i++) {
        try {
          var j = JSON.parse(sel.options[i].value);
          if (j.b === b && j.n === n && j.o === o) {
            match = sel.options[i].value;
            break;
          }
        } catch (e2) {}
      }
      sel.value = match || '';
    }

    document.getElementById('presetSelect').onchange = function() {
      var sel = document.getElementById('presetSelect');
      if (!sel.value) { return; }
      try {
        var j = JSON.parse(sel.value);
        document.getElementById('blockPath').value      = j.b;
        document.getElementById('netsPath').value       = j.n;
        document.getElementById('outPath').value        = j.o;
        document.getElementById('constraintPath').value = j.c || '';
        // 立即同步 constraint 清單
        if (j.c) {
          vscode.postMessage({ type: 'loadConstraint', path: j.c });
        } else {
          constraintFileLoaded = false;
          constraintData = [];
          buildConstraintList();
          render();
        }
      } catch (e3) {}
    };

    // ── 浮動面板開關 ──
    function togglePanel(id) {
      var target = document.getElementById(id);
      var wasOpen = target.classList.contains('open');
      // 關閉所有
      document.querySelectorAll('.float-panel').forEach(function(p) { p.classList.remove('open'); });
      if (!wasOpen) { target.classList.add('open'); }
    }
    document.addEventListener('click', function(e) {
      if (!e.target.closest('.canvas-toolbar-left') && !e.target.closest('.canvas-toolbar-right')) {
        document.querySelectorAll('.float-panel').forEach(function(p) { p.classList.remove('open'); });
      }
    });
    document.getElementById('btnTier').onclick = function(e) { e.stopPropagation(); togglePanel('panelTier'); };
    document.getElementById('btnView').onclick  = function(e) { e.stopPropagation(); togglePanel('panelView'); };

    // ── Tier 面板：動態建立選項 ──
    function buildTierPanel(numDie) {
      var panel = document.getElementById('panelTier');
      // 清除舊的 tier 選項（保留 overview）
      panel.querySelectorAll('.fp-option[data-mode]').forEach(function(el) {
        if (el.dataset.mode !== 'overview') { el.remove(); }
      });
      panel.querySelector('hr.fp-sep') && panel.querySelector('hr.fp-sep').remove();

      var sep = document.createElement('hr');
      sep.className = 'fp-sep';
      panel.appendChild(sep);

      for (var t = 0; t < numDie; t++) {
        (function(tier) {
          var opt = document.createElement('div');
          opt.className = 'fp-option' + (currentMode === tier ? ' active' : '');
          opt.dataset.mode = String(tier);
          var dot = '<span class="tier-dot" style="background:' + TIER_FILLS[tier % TIER_FILLS.length] + '"></span>';
          opt.innerHTML = dot + 'Tier ' + tier;
          opt.onclick = function() {
            currentMode = tier;
            updateTierBtn();
            updateTierPanelActive();
            document.getElementById('panelTier').classList.remove('open');
            render();
          };
          panel.appendChild(opt);
        })(t);
      }

      // Overview option click
      document.getElementById('optOverview').onclick = function() {
        currentMode = 'overview';
        updateTierBtn();
        updateTierPanelActive();
        document.getElementById('panelTier').classList.remove('open');
        render();
      };
    }

    function updateTierBtn() {
      var btn = document.getElementById('btnTier');
      btn.textContent = (currentMode === 'overview') ? 'Overview ▾' : 'Tier ' + currentMode + ' ▾';
    }

    function updateTierPanelActive() {
      document.querySelectorAll('#panelTier .fp-option').forEach(function(el) {
        var mode = el.dataset.mode === 'overview' ? 'overview' : parseInt(el.dataset.mode);
        el.classList.toggle('active', currentMode === mode);
      });
    }

    // Die outline + 50-unit grid + axis ticks（外框細實線；格線為淡虛線）
    var AXIS_GRID_STEP = 50;
    function appendDieWithGridAxes(parts, tx, ty, W, H, scale) {
      var x0 = tx(0), yTop = ty(H), wPx = W * scale, hPx = H * scale;
      parts.push('<rect x="' + x0 + '" y="' + yTop + '" width="' + wPx + '" height="' + hPx + '" fill="#252830"/>');
      var g;
      for (g = AXIS_GRID_STEP; g < W; g += AXIS_GRID_STEP) {
        parts.push('<line x1="' + tx(g) + '" y1="' + ty(0) + '" x2="' + tx(g) + '" y2="' + ty(H) + '" stroke="#3a4558" stroke-width="0.5" stroke-dasharray="3 4" opacity="0.5"/>');
      }
      for (g = AXIS_GRID_STEP; g < H; g += AXIS_GRID_STEP) {
        parts.push('<line x1="' + tx(0) + '" y1="' + ty(g) + '" x2="' + tx(W) + '" y2="' + ty(g) + '" stroke="#3a4558" stroke-width="0.5" stroke-dasharray="3 4" opacity="0.5"/>');
      }
      parts.push('<rect x="' + x0 + '" y="' + yTop + '" width="' + wPx + '" height="' + hPx + '" fill="none" stroke="#6b7d92" stroke-width="1"/>');
      var fs = Math.max(6, Math.min(10, scale * 0.14));
      var tick = Math.max(3, scale * 0.1);
      for (g = 0; g <= W; g += AXIS_GRID_STEP) {
        parts.push('<line x1="' + tx(g) + '" y1="' + ty(0) + '" x2="' + tx(g) + '" y2="' + (ty(0) + tick) + '" stroke="#8b9db0" stroke-width="1"/>');
        parts.push('<text x="' + tx(g) + '" y="' + (ty(0) + tick + fs + 2) + '" text-anchor="middle" font-size="' + fs + '" fill="#9aacbf">' + g + '</text>');
      }
      for (g = 0; g <= H; g += AXIS_GRID_STEP) {
        parts.push('<line x1="' + (tx(0) - tick) + '" y1="' + ty(g) + '" x2="' + tx(0) + '" y2="' + ty(g) + '" stroke="#8b9db0" stroke-width="1"/>');
        parts.push('<text x="' + (tx(0) - tick - 4) + '" y="' + ty(g) + '" text-anchor="end" dominant-baseline="middle" font-size="' + fs + '" fill="#9aacbf">' + g + '</text>');
      }
    }

    // ── 共用繪製工具：計算置中的座標轉換（預留四周給座標軸標示）──
    function makeTransform(outline, ox, oy, pw, ph) {
      var W = outline.width, H = outline.height;
      var PAD_L = 46, PAD_R = 12, PAD_T = 12, PAD_B = 54;
      var availW = pw - PAD_L - PAD_R;
      var availH = ph - PAD_T - PAD_B;
      var scale = Math.min(availW / W, availH / H);
      var diePxW = W * scale, diePxH = H * scale;
      var startX = ox + PAD_L + (availW - diePxW) / 2;
      var startY = oy + PAD_T + (availH - diePxH) / 2;
      currentTransform = { startX: startX, startY: startY, scale: scale, W: W, H: H };
      return {
        scale: scale, W: W, H: H,
        tx: function(x) { return startX + x * scale; },
        ty: function(y) { return startY + (H - y) * scale; },
        diePxW: diePxW, diePxH: diePxH
      };
    }

    // ── 單層渲染（個別 Tier 全螢幕） ──
    function renderTierInto(parts, tier, ox, oy, pw, ph) {
      var outline = sceneData.outlines[Math.min(tier, sceneData.outlines.length - 1)];
      var tr = makeTransform(outline, ox, oy, pw, ph);
      var tx = tr.tx, ty = tr.ty, scale = tr.scale;

      appendDieWithGridAxes(parts, tx, ty, tr.W, tr.H, tr.scale);

      // Modules（只顯示此層，支援 override 位置與選取高亮）
      if (chk('chkModules')) {
        var fill   = TIER_FILLS[tier % TIER_FILLS.length];
        var stroke = TIER_STROKES[tier % TIER_STROKES.length];
        var showML = chk('chkModuleLabels');
        var mods = sceneData.modules.filter(function(m) { return m.tier === tier; });
        for (var mi = 0; mi < mods.length; mi++) {
          var m = mods[mi];
          var em = getEffMod(m);
          var rx = tx(em.xll), ry = ty(em.yur);
          var rw = Math.max((em.xur - em.xll) * scale, 0.8);
          var rh = Math.max((em.yur - em.yll) * scale, 0.8);
          var isSel = selectedMod === m.name;
          var fixed = isFixed(m.name);
          var rgi = repulseGroupIdx(m.name);
          var multiRepulse = constraintData.filter(function(c) { return c.type === 'REPULSE'; }).length > 1;
          parts.push('<rect class="module-rect" data-modname="' + escXml(m.name) + '"' +
            ' x="' + rx + '" y="' + ry + '" width="' + rw + '" height="' + rh + '"' +
            ' fill="' + fill + '" fill-opacity="' + (isSel ? '0.6' : '0.35') + '"' +
            ' stroke="' + (isSel ? '#ffffff' : (fixed ? '#e08020' : stroke)) + '" stroke-width="' + (isSel ? '2' : (fixed ? '1.5' : '0.8')) + '"/>');
          if (fixed) {
            parts.push('<rect x="' + rx + '" y="' + ry + '" width="' + rw + '" height="' + rh + '" fill="url(#hatch-fixed)" style="pointer-events:none"/>');
          }
          if (rgi) {
            var bs = Math.min(14, rw * 0.4, rh * 0.4);
            var rlabel = multiRepulse ? 'R' + rgi : 'R';
            parts.push('<rect x="' + rx + '" y="' + ry + '" width="' + bs + '" height="' + bs + '" fill="#2288bb" rx="2" style="pointer-events:none"/>');
            parts.push('<text x="' + (rx + bs/2) + '" y="' + (ry + bs/2) + '" text-anchor="middle" dominant-baseline="middle" font-size="' + Math.max(6, bs*0.65) + '" font-weight="bold" fill="#fff" style="pointer-events:none">' + rlabel + '</text>');
          }
          if (showML && rw > 14 && rh > 8) {
            var cx = rx + rw/2, cy = ry + rh/2;
            var fs = Math.min(9, rh * 0.45, rw * 0.18);
            if (fs > 3.5) {
              parts.push('<text x="' + cx + '" y="' + cy + '" text-anchor="middle" dominant-baseline="middle" font-size="' + fs + '" fill="#fff" opacity="0.85" style="pointer-events:none">' + escXml(m.name) + '</text>');
            }
          }
        }
      }

      // TSVs：只顯示在下層（tierBelow === tier）
      if (chk('chkTsvs')) {
        var showTL = chk('chkTsvLabels');
        var tsvUserSz = parseFloat(document.getElementById('tsvSize').value) || 3;
        var tsvHalf = Math.max(1.5, tsvUserSz * scale / 2);
        var tsvList = sceneData.tsvs.filter(function(t) { return t.tierBelow === tier; });
        for (var ti = 0; ti < tsvList.length; ti++) {
          var t = tsvList[ti];
          var px = tx(t.x), py = ty(t.y);
          parts.push('<rect x="' + (px - tsvHalf) + '" y="' + (py - tsvHalf) + '" width="' + (tsvHalf*2) + '" height="' + (tsvHalf*2) + '" fill="#ff5566" opacity="0.5" stroke="#ffaabb" stroke-width="0.7"/>');
          if (showTL) {
            parts.push('<text x="' + (px + tsvHalf + 2) + '" y="' + py + '" font-size="6" fill="#ff8899" dominant-baseline="middle" opacity="0.7">' + escXml(t.netName) + '</text>');
          }
        }
      }

      // Terminals
      if (chk('chkTerminals')) {
        var showTmL = chk('chkTerminalLabels');
        var S = Math.max(4, scale * 0.8);
        for (var ti2 = 0; ti2 < sceneData.terminals.length; ti2++) {
          var tp = sceneData.terminals[ti2];
          var px = tx(tp.x), py = ty(tp.y);
          parts.push('<rect x="' + (px-S/2) + '" y="' + (py-S/2) + '" width="' + S + '" height="' + S + '" fill="#44cc88" opacity="0.9"/>');
          if (showTmL) {
            parts.push('<text x="' + (px+S/2+2) + '" y="' + py + '" font-size="6" fill="#66ddaa" dominant-baseline="middle" opacity="0.7">' + escXml(tp.name) + '</text>');
          }
        }
      }

      // Tier 標籤（左上角）
      // var tc = TIER_FILLS[tier % TIER_FILLS.length];
      // parts.push('<text x="' + (tx(0) + 6) + '" y="' + (ty(tr.H) + 14) + '" font-size="12" font-weight="bold" fill="' + tc + '">Tier ' + tier + '</text>');
    }

    // ── Overview 渲染：所有 tier 疊合在同一張圖 ──
    function renderOverlaid(parts, cw, ch) {
      // 以最大 outline 決定 die 尺寸
      var maxW = 0, maxH = 0;
      for (var t = 0; t < sceneData.outlines.length; t++) {
        maxW = Math.max(maxW, sceneData.outlines[t].width);
        maxH = Math.max(maxH, sceneData.outlines[t].height);
      }
      var fakeOutline = { width: maxW, height: maxH };
      var tr = makeTransform(fakeOutline, 0, 0, cw, ch);
      var tx = tr.tx, ty = tr.ty, scale = tr.scale;

      appendDieWithGridAxes(parts, tx, ty, maxW, maxH, tr.scale);

      // 所有 tier 的 Modules 疊合（各 tier 不同顏色，支援 override 與選取高亮）
      if (chk('chkModules')) {
        var showML = chk('chkModuleLabels');
        for (var t = 0; t < sceneData.outlines.length; t++) {
          var fill   = TIER_FILLS[t % TIER_FILLS.length];
          var stroke = TIER_STROKES[t % TIER_STROKES.length];
          var mods = sceneData.modules.filter(function(m) { return m.tier === t; });
          for (var mi = 0; mi < mods.length; mi++) {
            var m = mods[mi];
            var em = getEffMod(m);
            var rx = tx(em.xll), ry = ty(em.yur);
            var rw = Math.max((em.xur - em.xll) * scale, 0.8);
            var rh = Math.max((em.yur - em.yll) * scale, 0.8);
            var isSel = selectedMod === m.name;
            var fixed = isFixed(m.name);
            var rgi = repulseGroupIdx(m.name);
            var multiRepulse = constraintData.filter(function(c) { return c.type === 'REPULSE'; }).length > 1;
            parts.push('<rect class="module-rect" data-modname="' + escXml(m.name) + '"' +
              ' x="' + rx + '" y="' + ry + '" width="' + rw + '" height="' + rh + '"' +
              ' fill="' + fill + '" fill-opacity="' + (isSel ? '0.6' : '0.28') + '"' +
              ' stroke="' + (isSel ? '#ffffff' : (fixed ? '#e08020' : stroke)) + '" stroke-width="' + (isSel ? '2' : (fixed ? '1.5' : '0.7')) + '"/>');
            if (fixed) {
              parts.push('<rect x="' + rx + '" y="' + ry + '" width="' + rw + '" height="' + rh + '" fill="url(#hatch-fixed)" style="pointer-events:none"/>');
            }
            if (rgi) {
              var bs = Math.min(14, rw * 0.4, rh * 0.4);
              var rlabel = multiRepulse ? 'R' + rgi : 'R';
              parts.push('<rect x="' + rx + '" y="' + ry + '" width="' + bs + '" height="' + bs + '" fill="#2288bb" rx="2" style="pointer-events:none"/>');
              parts.push('<text x="' + (rx + bs/2) + '" y="' + (ry + bs/2) + '" text-anchor="middle" dominant-baseline="middle" font-size="' + Math.max(6, bs*0.65) + '" font-weight="bold" fill="#fff" style="pointer-events:none">' + rlabel + '</text>');
            }
            if (showML && rw > 14 && rh > 8) {
              var cx = rx + rw/2, cy = ry + rh/2;
              var fs = Math.min(9, rh * 0.45, rw * 0.18);
              if (fs > 3.5) {
                parts.push('<text x="' + cx + '" y="' + cy + '" text-anchor="middle" dominant-baseline="middle" font-size="' + fs + '" fill="#fff" opacity="0.8" style="pointer-events:none">' + escXml(m.name) + '</text>');
              }
            }
          }
        }
      }

      // TSVs（顯示在下層位置，即 tierBelow 所在層）
      if (chk('chkTsvs')) {
        var showTL = chk('chkTsvLabels');
        var tsvUserSz = parseFloat(document.getElementById('tsvSize').value) || 3;
        var tsvHalf = Math.max(1.5, tsvUserSz * scale / 2);
        for (var ti = 0; ti < sceneData.tsvs.length; ti++) {
          var t = sceneData.tsvs[ti];
          var px = tx(t.x), py = ty(t.y);
          parts.push('<rect x="' + (px - tsvHalf) + '" y="' + (py - tsvHalf) + '" width="' + (tsvHalf*2) + '" height="' + (tsvHalf*2) + '" fill="#ff5566" opacity="0.5" stroke="#ffaabb" stroke-width="0.7"/>');
          if (showTL) {
            parts.push('<text x="' + (px + tsvHalf + 2) + '" y="' + py + '" font-size="6" fill="#ff8899" dominant-baseline="middle" opacity="0.7">' + escXml(t.netName) + '</text>');
          }
        }
      }

      // Terminals
      if (chk('chkTerminals')) {
        var showTmL = chk('chkTerminalLabels');
        var S = Math.max(4, scale * 0.8);
        for (var ti2 = 0; ti2 < sceneData.terminals.length; ti2++) {
          var tp = sceneData.terminals[ti2];
          var px = tx(tp.x), py = ty(tp.y);
          parts.push('<rect x="' + (px-S/2) + '" y="' + (py-S/2) + '" width="' + S + '" height="' + S + '" fill="#44cc88" opacity="0.9"/>');
          if (showTmL) {
            parts.push('<text x="' + (px+S/2+2) + '" y="' + py + '" font-size="6" fill="#66ddaa" dominant-baseline="middle" opacity="0.7">' + escXml(tp.name) + '</text>');
          }
        }
      }

      // 圖例（右上角）
      var numDie = sceneData.outlines.length;
      for (var lt = 0; lt < numDie; lt++) {
        var lx = cw - 8, ly = 14 + lt * 18;
        parts.push('<rect x="' + (lx-52) + '" y="' + (ly-9) + '" width="11" height="11" fill="' + TIER_FILLS[lt%TIER_FILLS.length] + '" fill-opacity="0.6" rx="2"/>');
        parts.push('<text x="' + (lx-37) + '" y="' + ly + '" font-size="11" fill="' + TIER_FILLS[lt%TIER_FILLS.length] + '">Tier ' + lt + '</text>');
      }
    }

    // ── 主渲染函數 ──
    function render() {
      var container = document.getElementById('canvas-container');
      if (!sceneData) {
        container.innerHTML = '<div class="placeholder">請按「↺ 重新載入 2D」載入佈局</div>';
        return;
      }
      var cw = Math.max(container.clientWidth  - 2, 200);
      var ch = Math.max(container.clientHeight - 2, 200);
      var parts = [];
      parts.push('<defs><pattern id="hatch-fixed" patternUnits="userSpaceOnUse" width="7" height="7"><line x1="0" y1="7" x2="7" y2="0" stroke="#ffffff" stroke-width="0.8" opacity="0.28"/></pattern></defs>');
      parts.push('<rect x="0" y="0" width="' + cw + '" height="' + ch + '" fill="#1a1e24"/>');

      if (currentMode === 'overview') {
        renderOverlaid(parts, cw, ch);
      } else {
        renderTierInto(parts, currentMode, 0, 0, cw, ch);
      }

      container.innerHTML = '<svg width="' + cw + '" height="' + ch + '" xmlns="http://www.w3.org/2000/svg">' + parts.join('') + '</svg>';
      addModuleListeners(container);
    }

    // ── 檔案操作事件 ──
    document.getElementById('pickBlock').onclick = function() { vscode.postMessage({ type: 'pickBlock' }); };
    document.getElementById('pickNets').onclick  = function() { vscode.postMessage({ type: 'pickNets' }); };
    document.getElementById('runFloorplan').onclick = function() {
      vscode.postMessage({
        type:       'run',
        block:      document.getElementById('blockPath').value,
        nets:       document.getElementById('netsPath').value,
        output:     document.getElementById('outPath').value,
        constraint: document.getElementById('constraintPath').value.trim(),
      });
    };
    document.getElementById('reload2d').onclick = function() {
      vscode.postMessage({
        type: 'reload2d',
        blockPath:      document.getElementById('blockPath').value,
        outputPath:     document.getElementById('outPath').value.trim(),
        constraintPath: document.getElementById('constraintPath').value.trim(),
      });
    };

    // ── Module 互動：加入事件監聽（每次 render 後呼叫）──
    function addModuleListeners(container) {
      var svgEl = container.querySelector('svg');
      if (svgEl) {
        svgEl.addEventListener('mousedown', function(e) {
          if (!e.target.classList.contains('module-rect')) {
            if (selectedMod !== null) { selectedMod = null; updateRightPanel(); render(); }
          }
        });
      }
      container.querySelectorAll('.module-rect').forEach(function(el) {
        el.addEventListener('mousedown', function(e) {
          e.stopPropagation(); e.preventDefault();
          var name = el.dataset.modname;
          selectedMod = name;
          updateRightPanel();
          if (!currentTransform) { render(); return; }
          var bounds = container.getBoundingClientRect();
          var orig = sceneData.modules.find(function(m) { return m.name === name; });
          var em = getEffMod(orig);
          dragState = {
            name: name,
            startSvgX: e.clientX - bounds.left,
            startSvgY: e.clientY - bounds.top,
            origXll: em.xll, origYll: em.yll,
            w: em.xur - em.xll, h: em.yur - em.yll
          };
          document.addEventListener('mousemove', onDragMove);
          document.addEventListener('mouseup',   onDragEnd);
          render();
        });
      });
    }

    function onDragMove(e) {
      if (!dragState || !currentTransform) return;
      var container = document.getElementById('canvas-container');
      var bounds = container.getBoundingClientRect();
      var dx = (e.clientX - bounds.left - dragState.startSvgX) / currentTransform.scale;
      var dy = -((e.clientY - bounds.top)  - dragState.startSvgY) / currentTransform.scale;
      var newXll = dragState.origXll + dx;
      var newYll = dragState.origYll + dy;
      var orig = sceneData.modules.find(function(m) { return m.name === dragState.name; });
      if (orig) {
        var outline = sceneData.outlines[Math.min(orig.tier, sceneData.outlines.length - 1)];
        newXll = Math.max(0, Math.min(outline.width  - dragState.w, newXll));
        newYll = Math.max(0, Math.min(outline.height - dragState.h, newYll));
      }
      moduleOverrides[dragState.name] = { xll: newXll, yll: newYll, xur: newXll + dragState.w, yur: newYll + dragState.h };
      render();
      updateRightPanel();
    }

    function onDragEnd() {
      dragState = null;
      document.removeEventListener('mousemove', onDragMove);
      document.removeEventListener('mouseup',   onDragEnd);
    }

    // ── 右側面板更新 ──
    function updateRightPanel() {
      var hint     = document.getElementById('rpHint');
      var controls = document.getElementById('rpModControls');
      if (!selectedMod || !sceneData) {
        hint.style.display = 'flex'; controls.style.display = 'none'; return;
      }
      hint.style.display = 'none'; controls.style.display = 'block';
      var orig = sceneData.modules.find(function(m) { return m.name === selectedMod; });
      if (!orig) return;
      var em = getEffMod(orig);
      document.getElementById('rpModName').textContent = selectedMod;
      var w = (em.xur - em.xll).toFixed(1);
      var h = (em.yur - em.yll).toFixed(1);
      document.getElementById('rpSize').textContent = 'X × Y: (' + w + ' × ' + h + ')';
      document.getElementById('rpInputX').value = em.xll.toFixed(2);
      document.getElementById('rpInputY').value = em.yll.toFixed(2);
      document.getElementById('rpUrX').textContent = em.xur.toFixed(2);
      document.getElementById('rpUrY').textContent = em.yur.toFixed(2);
    }

    // ── 右側按鈕 ──
    document.getElementById('btnRotate').onclick = function() {
      if (!selectedMod) return;
      var orig = sceneData.modules.find(function(m) { return m.name === selectedMod; });
      if (!orig) return;
      var em = getEffMod(orig);
      var cx = (em.xll + em.xur) / 2, cy = (em.yll + em.yur) / 2;
      var oldW = em.xur - em.xll, oldH = em.yur - em.yll;
      moduleOverrides[selectedMod] = { xll: cx - oldH/2, yll: cy - oldW/2, xur: cx + oldH/2, yur: cy + oldW/2 };
      render(); updateRightPanel();
    };

    document.getElementById('btnSetPos').onclick = function() {
      if (!selectedMod) return;
      var x = parseFloat(document.getElementById('rpInputX').value);
      var y = parseFloat(document.getElementById('rpInputY').value);
      if (isNaN(x) || isNaN(y)) return;
      var orig = sceneData.modules.find(function(m) { return m.name === selectedMod; });
      if (!orig) return;
      var em = getEffMod(orig);
      var w = em.xur - em.xll, h = em.yur - em.yll;
      moduleOverrides[selectedMod] = { xll: x, yll: y, xur: x + w, yur: y + h };
      render(); updateRightPanel();
    };

    document.getElementById('btnResetPos').onclick = function() {
      if (!selectedMod) return;
      delete moduleOverrides[selectedMod];
      render(); updateRightPanel();
    };

    document.getElementById('btnResetAll').onclick = function() {
      moduleOverrides = {};
      render(); updateRightPanel();
    };

    // ── Constraint 清單渲染（可編輯）──
    function buildConstraintList() {
      var sec  = document.getElementById('rpConstraintSection');
      var list = document.getElementById('constraintList');
      // 有載入 constraint 檔才顯示區塊
      if (!constraintFileLoaded) { sec.style.display = 'none'; return; }
      sec.style.display = 'block';
      list.innerHTML = '';

      // ---- 每一行 ----
      constraintData.forEach(function(c, i) {
        var row = document.createElement('div');
        row.className = 'cst-row';

        // 類型 badge
        var badge = document.createElement('span');
        badge.className = 'cst-badge cst-badge-' + c.type.toLowerCase();
        badge.textContent = c.type === 'FIXED' ? 'F' : 'R';

        // 可編輯欄位
        var fields = document.createElement('div');
        fields.className = 'cst-fields';

        if (c.type === 'FIXED') {
          // name llx lly urx ury
          ['name','llx','lly','urx','ury'].forEach(function(key) {
            var inp = document.createElement('input');
            inp.type = (key === 'name') ? 'text' : 'text';
            inp.value = c[key];
            inp.placeholder = key;
            inp.title = key;
            inp.className = 'cst-inp ' + (key === 'name' ? 'cst-inp-name' : 'cst-inp-coord');
            inp.dataset.key = key;
            fields.appendChild(inp);
          });
        } else {
          // REPULSE: strength, names（空白分隔）
          var strengthInp = document.createElement('input');
          strengthInp.type = 'text';
          strengthInp.value = c.strength; strengthInp.placeholder = 'strength';
          strengthInp.title = 'strength'; strengthInp.dataset.key = 'strength';
          strengthInp.className = 'cst-inp cst-inp-strength';

          var namesInp = document.createElement('input');
          namesInp.type = 'text';
          namesInp.value = c.names.join(' '); namesInp.placeholder = 'name1 name2 …';
          namesInp.title = 'module names (space-separated)'; namesInp.dataset.key = 'names';
          namesInp.className = 'cst-inp cst-inp-names';

          fields.appendChild(strengthInp);
          fields.appendChild(namesInp);
        }

        // ✓ 確認按鈕
        var confirm = document.createElement('button');
        confirm.className = 'cst-confirm'; confirm.textContent = '✓';
        confirm.title = 'Save (writes file)';
        confirm.onclick = (function(idx, type, fieldsEl) {
          return function() {
            var line;
            // 使用 data-key 屬性讀值，避免依賴 NodeList 順序與 \s 逸出問題
            function v(key) {
              var el = fieldsEl.querySelector('[data-key="' + key + '"]');
              return el ? el.value.trim() : '';
            }
            if (type === 'FIXED') {
              line = 'FIXED ' + v('name') + ' ' + v('llx') + ' ' + v('lly') + ' ' + v('urx') + ' ' + v('ury');
            } else {
              line = 'REPULSE ' + v('strength') + ' ' + v('names');
            }
            vscode.postMessage({ type: 'updateConstraint', index: idx, line: line });
          };
        })(i, c.type, fields);

        // ✕ 刪除按鈕
        var del = document.createElement('button');
        del.className = 'cst-del'; del.textContent = '✕';
        del.title = 'Delete (writes file)';
        del.onclick = (function(idx) {
          return function() { vscode.postMessage({ type: 'deleteConstraint', index: idx }); };
        })(i);

        row.appendChild(badge); row.appendChild(fields);
        row.appendChild(confirm); row.appendChild(del);
        list.appendChild(row);
      });

      // ---- 新增列 ----
      var addRow = document.createElement('div');
      addRow.className = 'cst-add-row';

      var typeSel = document.createElement('select');
      typeSel.className = 'cst-add-type';
      ['FIXED', 'REPULSE'].forEach(function(t) {
        var opt = document.createElement('option');
        opt.value = t; opt.textContent = t;
        typeSel.appendChild(opt);
      });

      var addBtn = document.createElement('button');
      addBtn.className = 'cst-add-btn'; addBtn.textContent = '+ Add';
      addBtn.onclick = function() {
        var line = (typeSel.value === 'FIXED')
          ? 'FIXED name 0 0 1 1'
          : 'REPULSE 1.0 name1 name2';
        vscode.postMessage({ type: 'addConstraint', line: line });
      };

      addRow.appendChild(typeSel); addRow.appendChild(addBtn);
      list.appendChild(addRow);
    }

    document.getElementById('pickConstraint').onclick = function() {
      vscode.postMessage({ type: 'pickConstraint' });
    };

    // View 面板 checkbox / input 變更 → 重新渲染
    ['chkModules','chkTsvs','chkTerminals','chkModuleLabels','chkTsvLabels','chkTerminalLabels'].forEach(function(id) {
      document.getElementById(id).onchange = render;
    });
    document.getElementById('tsvSize').oninput = render;

    // ── Canvas resize → 重新渲染 ──
    new ResizeObserver(function() { render(); }).observe(document.getElementById('canvas-container'));

    // ── 可拖曳分隔線 ──
    function makeResizer(resizerId, getTarget, getSide) {
      var resizer = document.getElementById(resizerId);
      var startX, startW;
      resizer.addEventListener('mousedown', function(e) {
        startX = e.clientX;
        startW = getTarget().getBoundingClientRect().width;
        resizer.classList.add('dragging');
        document.body.style.cursor = 'col-resize';
        document.body.style.userSelect = 'none';
        function onMove(e) {
          var dx = e.clientX - startX;
          var newW = Math.max(120, Math.min(600, startW + getSide() * dx));
          getTarget().style.flex = '0 0 ' + newW + 'px';
        }
        function onUp() {
          resizer.classList.remove('dragging');
          document.body.style.cursor = '';
          document.body.style.userSelect = '';
          document.removeEventListener('mousemove', onMove);
          document.removeEventListener('mouseup', onUp);
          render();
        }
        document.addEventListener('mousemove', onMove);
        document.addEventListener('mouseup', onUp);
        e.preventDefault();
      });
    }
    // 左分隔線：拖右 → 左面板變寬（dx 正方向）
    makeResizer('resizerL', function() { return document.getElementById('leftPanel'); }, function() { return 1; });
    // 右分隔線：拖左 → 右面板變寬（dx 負方向）
    makeResizer('resizerR', function() { return document.getElementById('rightPanel'); }, function() { return -1; });

    // ── 接收 extension host 訊息 ──
    window.addEventListener('message', function(e) {
      var msg = e.data;
      if (msg.type === 'setPaths') {
        if (msg.block)      document.getElementById('blockPath').value      = msg.block;
        if (msg.nets)       document.getElementById('netsPath').value       = msg.nets;
        if (msg.output)     document.getElementById('outPath').value        = msg.output;
        if (msg.constraint !== undefined) document.getElementById('constraintPath').value = msg.constraint;
        syncPresetSelectFromPaths();
      }
      if (msg.type === 'constraintData') {
        constraintData = msg.constraints || [];
        constraintFileLoaded = true;
        buildConstraintList();
        render();
      }
      if (msg.type === 'presets') {
        populatePresets(msg.presets);
        syncPresetSelectFromPaths();
      }
      if (msg.type === 'log') { log(msg.message); }
      if (msg.type === 'sceneData') {
        sceneData = msg.payload;
        currentMode = 'overview';
        buildTierPanel(sceneData.outlines.length);
        updateTierBtn();
        render();
        log('[OK] ' + sceneData.modules.length + ' modules, ' +
            sceneData.tsvs.length + ' TSVs, ' +
            sceneData.terminals.length + ' terminals');
        // 顯示 Result 統計
        var resSec = document.getElementById('rpResultSection');
        if (msg.payload.hpwl !== undefined) {
          document.getElementById('rpHpwl').textContent = msg.payload.hpwl;
          document.getElementById('rpDie').textContent  = msg.payload.die;
          document.getElementById('rpTime').textContent = msg.payload.time;
          resSec.style.display = 'block';
        } else {
          resSec.style.display = 'none';
        }
      }
    });

    vscode.postMessage({ type: 'ready' });
  </script>
</body>
</html>`;
}
function buildViewPayload(block, floor, outText) {
    const outLines = outText.split(/\r?\n/);
    const hpwl = outLines[0]?.trim() ?? "";
    const die = outLines[3]?.trim() ?? "";
    const time = outLines[4]?.trim() ?? "";
    return {
        outlines: block.outlines,
        modules: floor.modules,
        tsvs: floor.tsvs,
        terminals: block.terminals,
        hpwl,
        die,
        time,
    };
}
function activate(context) {
    const disposable = vscode.commands.registerCommand(`${EXT_ID}.open`, () => {
        const panel = vscode.window.createWebviewPanel(`${EXT_ID}.panel`, "3DIC Floorplan Viewer", vscode.ViewColumn.One, {
            enableScripts: true,
            retainContextWhenHidden: true,
            localResourceRoots: [],
        });
        panel.webview.html = getHtml(panel.webview);
        let constraintFilePath = "";
        function appendLog(message) {
            panel.webview.postMessage({ type: "log", message });
        }
        function loadAndSendConstraint(cPath) {
            if (!cPath || !fs.existsSync(cPath)) {
                panel.webview.postMessage({ type: "constraintData", constraints: [] });
                return;
            }
            try {
                const text = fs.readFileSync(cPath, "utf8");
                const constraints = text
                    .split(/\r?\n/)
                    .filter((l) => l.trim().length > 0)
                    .map((l) => {
                    const parts = l.trim().split(/\s+/);
                    if (parts[0] === "FIXED") {
                        return {
                            type: "FIXED",
                            name: parts[1],
                            llx: +parts[2],
                            lly: +parts[3],
                            urx: +parts[4],
                            ury: +parts[5],
                        };
                    }
                    if (parts[0] === "REPULSE") {
                        return {
                            type: "REPULSE",
                            strength: +parts[1],
                            names: parts.slice(2),
                        };
                    }
                    return null;
                })
                    .filter(Boolean);
                panel.webview.postMessage({ type: "constraintData", constraints });
                appendLog(`Constraints: ${constraints.length} entries`);
            }
            catch (e) {
                const err = e instanceof Error ? e.message : String(e);
                appendLog(`Constraint load failed: ${err}`);
            }
        }
        function resolveOutputPath(analyticalCwd, outInput) {
            const trimmed = outInput.trim();
            if (!trimmed) {
                return "";
            }
            if (path.isAbsolute(trimmed)) {
                return trimmed;
            }
            return path.join(analyticalCwd, trimmed);
        }
        async function loadAndSend2D(blockPath, outputPath) {
            try {
                const blockText = fs.readFileSync(blockPath, "utf8");
                const outText = fs.readFileSync(outputPath, "utf8");
                const block = (0, parsers_1.parseBlockFile)(blockText);
                const floor = (0, parsers_1.parseFloorplanOutput)(outText);
                const payload = buildViewPayload(block, floor, outText);
                panel.webview.postMessage({ type: "sceneData", payload });
                appendLog(`載入完成：${floor.modules.length} modules、` +
                    `${floor.tsvs.length} TSVs、${block.terminals.length} terminals`);
            }
            catch (e) {
                const err = e instanceof Error ? e.message : String(e);
                appendLog(`載入失敗：${err}`);
                vscode.window.showErrorMessage(err);
            }
        }
        panel.webview.onDidReceiveMessage(async (msg) => {
            // --- ready ---
            if (msg.type === "ready") {
                const cfg = vscode.workspace.getConfiguration();
                const analyticalDir = resolveAnalyticalDir(cfg, context.extensionPath);
                const folders = vscode.workspace.workspaceFolders;
                const root = resolvePa2ProjectRoot(folders?.[0]?.uri.fsPath, context.extensionPath);
                let defaultBlock = "";
                let defaultNets = "";
                let defaultOut = "";
                let defaultConstraint = "";
                if (root) {
                    const guessBlock = path.join(root, "PA2_3DIC", "input_pa2", "n100.block");
                    const guessNets = path.join(root, "PA2_3DIC", "input_pa2", "n100.nets");
                    const guessOut = path.join(root, "PA2_3DIC", "analytical", "output", "n100_output.txt");
                    const guessCst = path.join(root, "PA2_3DIC", "input_pa2", "n100.constraint");
                    if (fs.existsSync(guessBlock)) {
                        defaultBlock = guessBlock;
                    }
                    if (fs.existsSync(guessNets)) {
                        defaultNets = guessNets;
                    }
                    if (fs.existsSync(guessOut)) {
                        defaultOut = guessOut;
                    }
                    if (fs.existsSync(guessCst)) {
                        defaultConstraint = guessCst;
                    }
                }
                constraintFilePath = defaultConstraint;
                panel.webview.postMessage({
                    type: "setPaths",
                    block: defaultBlock,
                    nets: defaultNets,
                    output: defaultOut,
                    constraint: defaultConstraint,
                });
                const presets = [];
                if (root) {
                    for (const id of ["n100", "n200", "n300"]) {
                        const blockPath = path.join(root, "PA2_3DIC", "input_pa2", `${id}.block`);
                        const netsPath = path.join(root, "PA2_3DIC", "input_pa2", `${id}.nets`);
                        const outPath = path.join(root, "PA2_3DIC", "analytical", "output", `${id}_output.txt`);
                        const cstPath = path.join(root, "PA2_3DIC", "input_pa2", `${id}.constraint`);
                        if (fs.existsSync(blockPath) && fs.existsSync(netsPath)) {
                            presets.push({
                                label: `${id} (default paths)`,
                                block: blockPath,
                                nets: netsPath,
                                output: outPath,
                                constraint: fs.existsSync(cstPath) ? cstPath : "",
                            });
                        }
                    }
                }
                panel.webview.postMessage({ type: "presets", presets });
                if (defaultBlock && defaultOut && fs.existsSync(defaultOut)) {
                    await loadAndSend2D(defaultBlock, defaultOut);
                }
                if (defaultConstraint) {
                    loadAndSendConstraint(defaultConstraint);
                }
                const exe = process.platform === "win32" ? "analytical.exe" : "analytical";
                appendLog(analyticalDir
                    ? `executable：${path.join(analyticalDir, exe)}`
                    : "警告：找不到 analytical，請設定 floorplanViewer.analyticalDirectory");
                return;
            }
            // --- pickBlock ---
            if (msg.type === "pickBlock") {
                const uris = await vscode.window.showOpenDialog({
                    canSelectMany: false,
                    filters: { Block: ["block"], "All files": ["*"] },
                });
                if (uris?.[0]) {
                    panel.webview.postMessage({ type: "setPaths", block: uris[0].fsPath });
                }
                return;
            }
            // --- pickNets ---
            if (msg.type === "pickNets") {
                const uris = await vscode.window.showOpenDialog({
                    canSelectMany: false,
                    filters: { Nets: ["nets", "net"], "All files": ["*"] },
                });
                if (uris?.[0]) {
                    panel.webview.postMessage({ type: "setPaths", nets: uris[0].fsPath });
                }
                return;
            }
            // --- loadConstraint（切換 preset 或外部觸發）---
            if (msg.type === "loadConstraint") {
                const p = msg.path || "";
                if (p) {
                    constraintFilePath = p;
                    loadAndSendConstraint(constraintFilePath);
                }
                return;
            }
            // --- pickConstraint ---
            if (msg.type === "pickConstraint") {
                const uris = await vscode.window.showOpenDialog({
                    canSelectMany: false,
                    filters: { Constraint: ["constraint"], "All files": ["*"] },
                });
                if (uris?.[0]) {
                    constraintFilePath = uris[0].fsPath;
                    panel.webview.postMessage({ type: "setPaths", constraint: constraintFilePath });
                    loadAndSendConstraint(constraintFilePath);
                }
                return;
            }
            // --- deleteConstraint ---
            if (msg.type === "deleteConstraint") {
                if (!constraintFilePath || !fs.existsSync(constraintFilePath)) {
                    vscode.window.showWarningMessage("No constraint file loaded");
                    return;
                }
                try {
                    const text = fs.readFileSync(constraintFilePath, "utf8");
                    const allLines = text.split(/\r?\n/);
                    const nonEmpty = allLines.filter((l) => l.trim().length > 0);
                    const targetIdx = msg.index;
                    if (targetIdx < 0 || targetIdx >= nonEmpty.length) {
                        return;
                    }
                    const targetLine = nonEmpty[targetIdx];
                    let removed = false;
                    const newLines = allLines.filter((l) => {
                        if (!removed && l === targetLine) {
                            removed = true;
                            return false;
                        }
                        return true;
                    });
                    fs.writeFileSync(constraintFilePath, newLines.join("\n"));
                    appendLog(`Deleted constraint #${targetIdx}`);
                    loadAndSendConstraint(constraintFilePath);
                }
                catch (e) {
                    const err = e instanceof Error ? e.message : String(e);
                    appendLog(`Delete failed: ${err}`);
                    vscode.window.showErrorMessage(err);
                }
                return;
            }
            // --- updateConstraint（編輯後確認）---
            if (msg.type === "updateConstraint") {
                if (!constraintFilePath || !fs.existsSync(constraintFilePath)) {
                    vscode.window.showWarningMessage("No constraint file loaded");
                    return;
                }
                try {
                    const text = fs.readFileSync(constraintFilePath, "utf8");
                    const allLines = text.split(/\r?\n/);
                    const nonEmpty = allLines.filter((l) => l.trim().length > 0);
                    const targetIdx = msg.index;
                    if (targetIdx < 0 || targetIdx >= nonEmpty.length) {
                        return;
                    }
                    const oldLine = nonEmpty[targetIdx];
                    let replaced = false;
                    const newLines = allLines.map((l) => {
                        if (!replaced && l === oldLine) {
                            replaced = true;
                            return msg.line;
                        }
                        return l;
                    });
                    fs.writeFileSync(constraintFilePath, newLines.join("\n"));
                    appendLog(`Updated constraint #${targetIdx}`);
                    loadAndSendConstraint(constraintFilePath);
                }
                catch (e) {
                    const err = e instanceof Error ? e.message : String(e);
                    appendLog(`Update failed: ${err}`);
                    vscode.window.showErrorMessage(err);
                }
                return;
            }
            // --- addConstraint（新增一行）---
            if (msg.type === "addConstraint") {
                if (!constraintFilePath) {
                    vscode.window.showWarningMessage("No constraint file loaded – please pick a .constraint file first");
                    return;
                }
                try {
                    const existing = fs.existsSync(constraintFilePath)
                        ? fs.readFileSync(constraintFilePath, "utf8")
                        : "";
                    const trimmed = existing.trimEnd();
                    const newContent = trimmed ? trimmed + "\n" + msg.line + "\n"
                        : msg.line + "\n";
                    fs.writeFileSync(constraintFilePath, newContent);
                    appendLog(`Added constraint: ${msg.line}`);
                    loadAndSendConstraint(constraintFilePath);
                }
                catch (e) {
                    const err = e instanceof Error ? e.message : String(e);
                    appendLog(`Add failed: ${err}`);
                    vscode.window.showErrorMessage(err);
                }
                return;
            }
            // --- run（使用絕對路徑執行 analytical）---
            if (msg.type === "run") {
                const cfg = vscode.workspace.getConfiguration();
                const adir = resolveAnalyticalDir(cfg, context.extensionPath);
                if (!adir) {
                    vscode.window.showErrorMessage("找不到 analytical：請設定 floorplanViewer.analyticalDirectory 或將工作區開在含 PA2_3DIC/analytical 的專案");
                    return;
                }
                const blockFs = msg.block;
                const netsFs = msg.nets;
                const outRaw = msg.output;
                const cstFs = (msg.constraint || "").trim();
                if (!blockFs || !netsFs) {
                    vscode.window.showWarningMessage("請選取 .block 與 .nets");
                    return;
                }
                const outAbs = resolveOutputPath(adir, outRaw);
                if (!outAbs) {
                    vscode.window.showWarningMessage("請指定輸出檔路徑");
                    return;
                }
                // 建構絕對路徑執行指令（有 .constraint 時附加為第四個參數）
                const exe = process.platform === "win32" ? "analytical.exe" : "analytical";
                const exeAbs = path.join(adir, exe);
                const cstArg = cstFs ? ` ${quotePath(cstFs)}` : "";
                const cmd = `${quotePath(exeAbs)} ${quotePath(blockFs)} ${quotePath(netsFs)} ${quotePath(outAbs)}${cstArg}`;
                appendLog(`$ ${cmd}`);
                const term = vscode.window.createTerminal({
                    name: "analytical floorplan",
                    cwd: adir,
                });
                term.show();
                term.sendText(cmd);
                appendLog("已在終端機執行，完成後按「重新載入 2D」更新視圖。");
                return;
            }
            // --- reload2d ---
            if (msg.type === "reload2d") {
                const blockPath = msg.blockPath;
                const cfg = vscode.workspace.getConfiguration();
                const adir = resolveAnalyticalDir(cfg, context.extensionPath);
                let outputPath = msg.outputPath;
                if (outputPath && !path.isAbsolute(outputPath) && adir) {
                    outputPath = path.join(adir, outputPath);
                }
                if (!blockPath || !outputPath) {
                    vscode.window.showWarningMessage("請填好 .block 與輸出檔路徑後再載入");
                    return;
                }
                if (!fs.existsSync(blockPath)) {
                    vscode.window.showErrorMessage("找不到 .block 檔");
                    return;
                }
                if (!fs.existsSync(outputPath)) {
                    vscode.window.showErrorMessage("找不到輸出檔，請先執行 floorplan 或使用絕對路徑");
                    return;
                }
                await loadAndSend2D(blockPath, outputPath);
                const cPath = msg.constraintPath || "";
                if (cPath) {
                    constraintFilePath = cPath;
                }
                loadAndSendConstraint(constraintFilePath);
            }
        }, undefined, context.subscriptions);
    });
    context.subscriptions.push(disposable);
}
function deactivate() { }
//# sourceMappingURL=extension.js.map