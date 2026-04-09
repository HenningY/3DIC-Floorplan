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
function quotePath(p) {
    if (process.platform === "win32") {
        return `"${p.replace(/"/g, '\\"')}"`;
    }
    return `'${p.replace(/'/g, "'\\''")}'`;
}
function resolveAnalyticalDir(config) {
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
    if (!folders) {
        return undefined;
    }
    const root = folders[0].uri.fsPath;
    const candidates = [
        path.join(root, "PA2_3DIC", "analytical"),
        path.join(root, "PD", "PA2_3DIC", "analytical"),
        path.join(root, "analytical"),
    ];
    const exe = process.platform === "win32" ? "analytical.exe" : "analytical";
    for (const c of candidates) {
        if (fs.existsSync(path.join(c, exe))) {
            return c;
        }
    }
    return undefined;
}
function getHtml(webview, extensionUri) {
    const threeUri = "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.min.js";
    const sceneUri = webview.asWebviewUri(vscode.Uri.joinPath(extensionUri, "media", "scene.js"));
    const csp = [
        "default-src 'none';",
        `style-src ${webview.cspSource} 'unsafe-inline';`,
        `script-src ${webview.cspSource} https://cdn.jsdelivr.net 'unsafe-inline';`,
        `img-src ${webview.cspSource} data:;`,
    ].join(" ");
    return `<!DOCTYPE html>
<html lang="zh-Hant">
<head>
  <meta charset="UTF-8" />
  <meta http-equiv="Content-Security-Policy" content="${csp}" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>3DIC Floorplan</title>
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0;
      padding: 0;
      font-family: var(--vscode-font-family);
      font-size: 13px;
      color: var(--vscode-foreground);
      background: var(--vscode-editor-background);
      display: flex;
      flex-direction: column;
      height: 100vh;
      overflow: hidden;
    }
    .toolbar {
      padding: 10px 12px;
      border-bottom: 1px solid var(--vscode-panel-border);
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      align-items: center;
    }
    .toolbar label { margin-right: 4px; opacity: 0.9; }
    .path-row {
      display: flex;
      align-items: center;
      gap: 6px;
      flex: 1 1 280px;
      min-width: 200px;
    }
    .path-row input {
      flex: 1;
      min-width: 0;
      padding: 4px 8px;
      background: var(--vscode-input-background);
      color: var(--vscode-input-foreground);
      border: 1px solid var(--vscode-input-border, #444);
      border-radius: 2px;
    }
    button {
      padding: 5px 12px;
      cursor: pointer;
      background: var(--vscode-button-background);
      color: var(--vscode-button-foreground);
      border: none;
      border-radius: 2px;
    }
    button:disabled { opacity: 0.5; cursor: not-allowed; }
    button.secondary {
      background: var(--vscode-button-secondaryBackground);
      color: var(--vscode-button-secondaryForeground);
    }
    .main-split {
      flex: 1;
      display: flex;
      min-height: 0;
    }
    .left-panel {
      width: 38%;
      min-width: 220px;
      border-right: 1px solid var(--vscode-panel-border);
      display: flex;
      flex-direction: column;
      min-height: 0;
    }
    .left-panel h3 {
      margin: 0;
      padding: 8px 12px;
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 0.06em;
      opacity: 0.75;
      border-bottom: 1px solid var(--vscode-panel-border);
    }
    #log {
      flex: 1;
      margin: 0;
      padding: 10px 12px;
      overflow: auto;
      font-family: var(--vscode-editor-font-family);
      font-size: 12px;
      line-height: 1.45;
      white-space: pre-wrap;
      word-break: break-all;
    }
    .right-panel {
      flex: 1;
      min-width: 200px;
      display: flex;
      flex-direction: column;
      min-height: 0;
    }
    .right-panel h3 {
      margin: 0;
      padding: 8px 12px;
      font-size: 11px;
      text-transform: uppercase;
      opacity: 0.75;
      border-bottom: 1px solid var(--vscode-panel-border);
    }
    #canvas-container {
      flex: 1;
      min-height: 200px;
      position: relative;
    }
    .hint {
      font-size: 11px;
      opacity: 0.65;
      padding: 0 12px 8px;
    }
  </style>
</head>
<body>
  <div class="toolbar">
    <div class="path-row">
      <label>.block</label>
      <input type="text" id="blockPath" readonly placeholder="選取 .block" />
      <button class="secondary" id="pickBlock">瀏覽…</button>
    </div>
    <div class="path-row">
      <label>.nets</label>
      <input type="text" id="netsPath" readonly placeholder="選取 .nets" />
      <button class="secondary" id="pickNets">瀏覽…</button>
    </div>
    <div class="path-row">
      <label>輸出</label>
      <input type="text" id="outPath" placeholder="./output/n300_output.txt" />
    </div>
    <button id="runFloorplan">執行 Floorplan</button>
    <button class="secondary" id="reload3d">重新載入 3D</button>
  </div>
  <p class="hint">終端機工作目錄為 analytical（<code>./analytical</code>）。輸出路徑相對於該目錄。</p>
  <div class="main-split">
    <div class="left-panel">
      <h3>狀態 / 日誌</h3>
      <pre id="log"></pre>
    </div>
    <div class="right-panel">
      <h3>3D 視圖（Orthographic · 45° 俯視）</h3>
      <div id="canvas-container"></div>
    </div>
  </div>
  <script src="${threeUri}"></script>
  <script src="${sceneUri}"></script>
  <script>
    const vscode = acquireVsCodeApi();
    const logEl = document.getElementById('log');
    function log(msg) {
      logEl.textContent += msg + '\\n';
      logEl.scrollTop = logEl.scrollHeight;
    }
    document.getElementById('pickBlock').onclick = () => vscode.postMessage({ type: 'pickBlock' });
    document.getElementById('pickNets').onclick = () => vscode.postMessage({ type: 'pickNets' });
    document.getElementById('runFloorplan').onclick = () => {
      vscode.postMessage({
        type: 'run',
        block: document.getElementById('blockPath').value,
        nets: document.getElementById('netsPath').value,
        output: document.getElementById('outPath').value,
      });
    };
    document.getElementById('reload3d').onclick = () => {
      vscode.postMessage({
        type: 'reload3d',
        blockPath: document.getElementById('blockPath').value,
        outputPath: document.getElementById('outPath').value.trim(),
      });
    };
    window.addEventListener('message', (e) => {
      if (e.data.type === 'setPaths') {
        if (e.data.block) document.getElementById('blockPath').value = e.data.block;
        if (e.data.nets) document.getElementById('netsPath').value = e.data.nets;
        if (e.data.output) document.getElementById('outPath').value = e.data.output;
      }
      if (e.data.type === 'log') log(e.data.message);
    });
    vscode.postMessage({ type: 'ready' });
  </script>
</body>
</html>`;
}
function buildScenePayload(block, floor, tsvHalf) {
    const maxW = Math.max(...block.outlines.map((o) => o.width), 1);
    const maxH = Math.max(...block.outlines.map((o) => o.height), 1);
    const stackPitch = Math.max(maxW, maxH) * 0.18;
    const dieThickness = Math.max(maxW, maxH) * 0.022;
    const moduleThickness = Math.max(maxW, maxH) * 0.01;
    return {
        outlines: block.outlines,
        modules: floor.modules,
        tsvs: floor.tsvs,
        tsvHalfSize: tsvHalf,
        stackPitch,
        dieThickness,
        moduleThickness,
    };
}
function activate(context) {
    const disposable = vscode.commands.registerCommand(`${EXT_ID}.open`, () => {
        const panel = vscode.window.createWebviewPanel(`${EXT_ID}.panel`, "3DIC Floorplan Viewer", vscode.ViewColumn.One, {
            enableScripts: true,
            retainContextWhenHidden: true,
            localResourceRoots: [
                vscode.Uri.joinPath(context.extensionUri, "media"),
            ],
        });
        panel.webview.html = getHtml(panel.webview, context.extensionUri);
        const cfgFlat = vscode.workspace.getConfiguration();
        const tsvSize = cfgFlat.get("floorplanViewer.tsvSize") ?? 3;
        function appendLog(message) {
            panel.webview.postMessage({ type: "log", message });
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
        async function loadAndSend3D(blockPath, outputPath) {
            try {
                const blockText = fs.readFileSync(blockPath, "utf8");
                const outText = fs.readFileSync(outputPath, "utf8");
                const block = (0, parsers_1.parseBlockFile)(blockText);
                const floor = (0, parsers_1.parseFloorplanOutput)(outText);
                const payload = buildScenePayload(block, floor, tsvSize / 2);
                panel.webview.postMessage({ type: "sceneData", payload });
                appendLog(`3D: ${floor.modules.length} modules, ${floor.tsvs.length} TSV markers.`);
            }
            catch (e) {
                const err = e instanceof Error ? e.message : String(e);
                appendLog(`載入 3D 失敗: ${err}`);
                vscode.window.showErrorMessage(err);
            }
        }
        panel.webview.onDidReceiveMessage(async (msg) => {
            if (msg.type === "ready") {
                const cfg = vscode.workspace.getConfiguration();
                const analyticalDir = resolveAnalyticalDir(cfg);
                const folders = vscode.workspace.workspaceFolders;
                const root = folders?.[0]?.uri.fsPath;
                let defaultBlock = "";
                let defaultNets = "";
                let defaultOut = "";
                if (root) {
                    const guessBlock = path.join(root, "PA2_3DIC", "input_pa2", "n100.block");
                    const guessNets = path.join(root, "PA2_3DIC", "input_pa2", "n100.nets");
                    const guessOut = path.join(root, "PA2_3DIC", "analytical", "output", "n100_output.txt");
                    if (fs.existsSync(guessBlock)) {
                        defaultBlock = guessBlock;
                    }
                    if (fs.existsSync(guessNets)) {
                        defaultNets = guessNets;
                    }
                    if (fs.existsSync(guessOut)) {
                        defaultOut = guessOut;
                    }
                }
                panel.webview.postMessage({
                    type: "setPaths",
                    block: defaultBlock,
                    nets: defaultNets,
                    output: defaultOut,
                });
                if (defaultBlock && defaultOut && fs.existsSync(defaultOut)) {
                    await loadAndSend3D(defaultBlock, defaultOut);
                }
                appendLog(analyticalDir
                    ? `analytical 目錄: ${analyticalDir}`
                    : "警告: 找不到 analytical 執行檔目錄，請設定 floorplanViewer.analyticalDirectory");
                return;
            }
            if (msg.type === "pickBlock") {
                const uris = await vscode.window.showOpenDialog({
                    canSelectMany: false,
                    filters: { Block: ["block"], "All files": ["*"] },
                });
                if (uris?.[0]) {
                    panel.webview.postMessage({
                        type: "setPaths",
                        block: uris[0].fsPath,
                    });
                }
                return;
            }
            if (msg.type === "pickNets") {
                const uris = await vscode.window.showOpenDialog({
                    canSelectMany: false,
                    filters: { Nets: ["nets", "net"], "All files": ["*"] },
                });
                if (uris?.[0]) {
                    panel.webview.postMessage({
                        type: "setPaths",
                        nets: uris[0].fsPath,
                    });
                }
                return;
            }
            if (msg.type === "run") {
                const cfg = vscode.workspace.getConfiguration();
                const adir = resolveAnalyticalDir(cfg);
                if (!adir) {
                    vscode.window.showErrorMessage("找不到 analytical：請設定 floorplanViewer.analyticalDirectory 或將工作區開在含 PA2_3DIC/analytical 的專案");
                    return;
                }
                const blockFs = msg.block;
                const netsFs = msg.nets;
                const outRaw = msg.output;
                if (!blockFs || !netsFs) {
                    vscode.window.showWarningMessage("請選取 .block 與 .nets");
                    return;
                }
                const outAbs = resolveOutputPath(adir, outRaw);
                if (!outAbs) {
                    vscode.window.showWarningMessage("請指定輸出檔路徑");
                    return;
                }
                const term = vscode.window.createTerminal({
                    name: "analytical floorplan",
                    cwd: adir,
                });
                const cmd = `./analytical ${quotePath(blockFs)} ${quotePath(netsFs)} ${quotePath(outAbs)}`;
                appendLog(`$ ${cmd}`);
                term.show();
                term.sendText(cmd);
                appendLog("已在終端機執行。完成後按「重新載入 3D」更新視圖。");
                return;
            }
            if (msg.type === "reload3d") {
                const blockPath = msg.blockPath;
                const cfg = vscode.workspace.getConfiguration();
                const adir = resolveAnalyticalDir(cfg);
                let outputPath = msg.outputPath;
                if (outputPath && !path.isAbsolute(outputPath) && adir) {
                    outputPath = path.join(adir, outputPath);
                }
                if (!blockPath || !outputPath) {
                    vscode.window.showWarningMessage("請填好 .block 與輸出檔路徑後再載入 3D");
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
                await loadAndSend3D(blockPath, outputPath);
            }
        }, undefined, context.subscriptions);
    });
    context.subscriptions.push(disposable);
}
function deactivate() { }
//# sourceMappingURL=extension.js.map