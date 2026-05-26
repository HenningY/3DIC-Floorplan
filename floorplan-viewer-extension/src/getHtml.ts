import * as vscode from "vscode";

export function getHtml(
  webview: vscode.Webview,
  extensionUri: vscode.Uri
): string {
  const styleUri  = webview.asWebviewUri(
    vscode.Uri.joinPath(extensionUri, "media", "style.css")
  );
  const scriptUri = webview.asWebviewUri(
    vscode.Uri.joinPath(extensionUri, "media", "webview.js")
  );

  const csp = [
    "default-src 'none';",
    `style-src ${webview.cspSource} 'unsafe-inline';`,
    `script-src ${webview.cspSource};`,
    `img-src ${webview.cspSource} data:;`,
  ].join(" ");

  return `<!DOCTYPE html>
<html lang="zh-Hant">
<head>
  <meta charset="UTF-8" />
  <meta http-equiv="Content-Security-Policy" content="${csp}" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>3DIC Floorplan 2D Viewer</title>
  <link rel="stylesheet" href="${styleUri}" />
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
          <button class="browse-btn secondary" id="clearConstraint" style="display:none" title="Clear constraint file">✕</button>
          <button class="browse-btn secondary" id="pickConstraint">…</button>
        </div>
      </div>
    </div>
    <div class="btn-row">
      <button id="runFloorplan">Run Floorplan</button>
      <button class="secondary" id="reload2d">Reload 2D</button>
    </div>
    <hr class="divider" />
    <div class="log-section">
      <div class="panel-header">Status / Log</div>
      <pre id="log"></pre>
    </div>
    <!-- Result / Info 統計 -->
    <div id="rpResultSection" style="display:none">
      <hr class="divider" />
      <div class="panel-header">Result / Info</div>
      <div id="rpHpwlBlock" style="padding: 8px 10px 0">
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
      <!-- 動態 tier / module / tsv 資訊 -->
      <div id="rpInfoRows" style="padding: 4px 10px 8px"></div>
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
          <div class="fp-title">Show Tiers</div>
          <div class="fp-option active" id="optOverview" data-mode="overview">
            <span class="tier-dot" style="background:#8899aa"></span>Overview
          </div>
        </div>
      </div>
      <!-- 右上：縮放 + View 選項 -->
      <div class="canvas-toolbar-right">
        <div class="zoom-controls">
          <button class="toolbar-btn zoom-btn" id="btnZoomOut" title="Zoom out">−</button>
          <button class="toolbar-btn zoom-btn" id="btnZoomReset" title="Reset zoom (100%)">100%</button>
          <button class="toolbar-btn zoom-btn" id="btnZoomIn" title="Zoom in">+</button>
        </div>
        <button class="toolbar-btn" id="btnView">View ▾</button>
        <div id="panelView" class="float-panel panel-right">
          <div class="fp-title">Show Items</div>
          <label class="fp-check"><input type="checkbox" id="chkModules" checked /><span class="color-dot" style="background:#4a90d9;opacity:0.7"></span>Modules</label>
          <label class="fp-check"><input type="checkbox" id="chkTsvs" checked /><span class="color-dot" style="background:#ff5566"></span>TSVs</label>
          <div class="fp-tsv-size">
            <span>Side</span>
            <input type="number" id="tsvSize" value="3" min="0" step="0.5" max="10" title="TSV 邊長（floorplan 單位，可直接輸入任意 >0.1 正數）" />
          </div>
          <label class="fp-check"><input type="checkbox" id="chkTerminals" checked /><span class="color-dot" style="background:#44cc88"></span>Terminals</label>
          <hr class="fp-sep" />
          <div class="fp-title">Nets</div>
          <label class="fp-check"><input type="checkbox" id="chkNetsIntra" /><span class="line-legend"><svg width="14" height="8" aria-hidden="true"><line x1="0" y1="4" x2="14" y2="4" stroke="currentColor" stroke-width="1.5"/></svg></span>Intra-tier nets</label>
          <label class="fp-check"><input type="checkbox" id="chkNetsInter" /><span class="line-legend"><svg width="14" height="8" aria-hidden="true"><line x1="0" y1="4" x2="14" y2="4" stroke="currentColor" stroke-width="1.5" stroke-dasharray="4 2"/></svg></span>Inter-tier nets</label>
          <hr class="fp-sep" />
          <div class="fp-title">Labels</div>
          <label class="fp-check"><input type="checkbox" id="chkModuleLabels" checked />Module labels</label>
          <label class="fp-check"><input type="checkbox" id="chkTsvLabels" />TSV labels</label>
          <label class="fp-check"><input type="checkbox" id="chkTerminalLabels" />Terminal labels</label>
        </div>
      </div>
    </div>
    <div id="canvas-container">
      <div class="placeholder">Please press「Reload 2D」to load the layout</div>
    </div>
  </div>

  <div class="resizer" id="resizerR"></div>

  <!-- ── 右側面板 ── -->
  <div class="right-panel" id="rightPanel">
    <div class="panel-header">Operation</div>
    <div class="rp-body">
      <!-- Selected 區域（固定顯示在上方） -->
      <div id="rpModControls">
        <div class="rp-section">
          <div class="rp-section-title">Selected</div>
          <div class="rp-name-row">
            <span class="rp-modname" id="rpModName">—</span>
            <span class="rp-size-inline" id="rpSize"></span>
          </div>
          <!-- Rotate + llx + lly + Set 同一行 -->
          <div class="rp-row-rotate-xy">
            <button id="btnRotate" title="Rotate 90°">Rotate 90°</button>
            <input type="number" id="rpInputX" step="0.5" placeholder="llx" title="lower-left X" />
            <input type="number" id="rpInputY" step="0.5" placeholder="lly" title="lower-left Y" />
            <button id="btnSetPos" title="Set position">Set Position</button>
          </div>
          <!-- Current position（llx/lly/urx/ury 唯讀）-->
          <div class="rp-row-ur">
            <span class="rp-coord-label" style="flex:0 0 auto;margin-right:4px">Cur Pos:</span>
            <span class="rp-coord-label">llx</span>
            <span class="rp-coord-val" id="rpLlX">—</span>
            <span class="rp-coord-label">lly</span>
            <span class="rp-coord-val" id="rpLlY">—</span>
            <span class="rp-coord-label">urx</span>
            <span class="rp-coord-val" id="rpUrX">—</span>
            <span class="rp-coord-label">ury</span>
            <span class="rp-coord-val" id="rpUrY">—</span>
          </div>
          <!-- Reset / Reset All / Apply Fixed -->
          <div class="rp-btn-row">
            <button class="secondary" id="btnResetPos" style="flex:1">Reset</button>
            <button class="secondary" id="btnResetAll" style="flex:1">Reset All</button>
            <button id="btnApplyFixed" style="flex:1.2" title="Apply as FIXED constraint">Add to Fixed Constraints</button>
          </div>
        </div>
      </div>
      <!-- Constraint 清單 -->
      <div id="rpConstraintSection" style="display:none">
        <div class="cst-section">
          <div class="rp-section-title">Constraints</div>
          <div id="constraintList"><span class="cst-empty">No constraints</span></div>
        </div>
      </div>
    </div>
  </div>

</div>
<script src="${scriptUri}"></script>
</body>
</html>`;
}
