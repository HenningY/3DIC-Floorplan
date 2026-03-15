const vscode = require('vscode');
const cp = require('child_process');
const path = require('path');
const fs = require('fs');

/**
 * @param {vscode.ExtensionContext} context
 */
function activate(context) {
  const disposable = vscode.commands.registerCommand(
    'pa2Floorplan.openVisualizer',
    () => {
      const panel = vscode.window.createWebviewPanel(
        'pa2FloorplanVisualizer',
        'PA2 3DIC Floorplan Visualizer',
        vscode.ViewColumn.Beside,
        {
          enableScripts: true,
          retainContextWhenHidden: true,
        },
      );

      panel.webview.html = getWebviewContent(panel.webview, context.extensionUri);

      panel.webview.onDidReceiveMessage(
        async (message) => {
          if (message.type === 'runFp') {
            await handleRunFp(panel, message);
          }
        },
        undefined,
        context.subscriptions,
      );
    },
  );

  context.subscriptions.push(disposable);
}

/**
 * Build HTML for the webview, wiring up local script.
 * @param {vscode.Webview} webview
 * @param {vscode.Uri} extensionUri
 */
function getWebviewContent(webview, extensionUri) {
  const scriptUri = webview.asWebviewUri(
    vscode.Uri.joinPath(extensionUri, 'media', 'visualizer.js'),
  );

  const stylesUri = webview.asWebviewUri(
    vscode.Uri.joinPath(extensionUri, 'media', 'visualizer.css'),
  );

  const nonce = getNonce();

  return /* html */ `<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>PA2 3DIC Floorplan Visualizer</title>
  <link rel="stylesheet" href="${stylesUri}">
  <meta http-equiv="Content-Security-Policy" content="
    default-src 'none';
    img-src data: https:;
    style-src 'unsafe-inline' ${webview.cspSource};
    script-src 'nonce-${nonce}';
  ">
</head>
<body>
  <div class="controls">
    <div class="row">
      <label>Alpha (area / HPWL weight):</label>
      <input id="alpha-input" type="number" step="0.05" min="0" max="1" value="0.5" />
    </div>
    <div class="row">
      <label>Block file (.block):</label>
      <input id="block-path" type="text" value="input_pa2/ami33.block" />
    </div>
    <div class="row">
      <label>Net file (.nets):</label>
      <input id="net-path" type="text" value="input_pa2/ami33.nets" />
    </div>
    <div class="row">
      <button id="run-fp-btn">Run ./bin/fp & Visualize</button>
      <span id="status-text"></span>
    </div>
  </div>

  <div class="info-panel">
    <div id="hpwl-text">HPWL: -</div>
    <div id="hpwl-original-text">Original HPWL (from .out): -</div>
    <div id="die-info"></div>
  </div>

  <canvas id="floorplanCanvas"></canvas>

  <script nonce="${nonce}" src="${scriptUri}"></script>
</body>
</html>`;
}

/**
 * Handle runFp request from the webview:
 *  - spawn ./bin/fp in r13943074_pa2
 *  - read generated .out, plus the chosen .block/.nets files
 *  - send the raw texts back to the webview for parsing & visualization
 * @param {vscode.WebviewPanel} panel
 * @param {any} message
 */
async function handleRunFp(panel, message) {
  const alpha = typeof message.alpha === 'number' ? message.alpha : 0.5;
  const blockRel = String(message.blockPath || '').trim();
  const netRel = String(message.netPath || '').trim();

  const workspaceFolders = vscode.workspace.workspaceFolders;
  if (!workspaceFolders || workspaceFolders.length === 0) {
    vscode.window.showErrorMessage('No workspace folder is open.');
    return;
  }

  const workspaceRoot = workspaceFolders[0].uri.fsPath;

  // Detect actual PA2 project root.
  // Case A: workspaceRoot = /home/henningy/PD/PA2_3DIC
  // Case B: workspaceRoot = /home/henningy/PD  (then project is under PA2_3DIC/)
  let projectRoot = workspaceRoot;
  const nestedCandidate = path.join(workspaceRoot, 'PA2_3DIC', 'r13943074_pa2');
  const flatCandidate = path.join(workspaceRoot, 'r13943074_pa2');
  if (fs.existsSync(nestedCandidate)) {
    projectRoot = path.join(workspaceRoot, 'PA2_3DIC');
  }

  const projectDir = path.join(projectRoot, 'r13943074_pa2');
  const fpBinary = path.join(projectDir, 'bin', 'fp');

  const blockAbs = path.isAbsolute(blockRel)
    ? blockRel
    : path.join(projectRoot, blockRel);
  const netAbs = path.isAbsolute(netRel)
    ? netRel
    : path.join(projectRoot, netRel);

  const blockBase = path.basename(blockAbs, path.extname(blockAbs));
  const alphaSuffix = Math.round(alpha * 100);
  const outputDir = path.join(projectRoot, 'output_pa2');
  const outAbs = path.join(outputDir, `${blockBase}_${alphaSuffix}.out`);

  panel.webview.postMessage({
    type: 'status',
    text: `Running fp on ${path.basename(blockAbs)} / ${path.basename(netAbs)}...`,
  });

  try {
    await runFpBinary(fpBinary, projectDir, alpha, blockAbs, netAbs, outAbs);
  } catch (err) {
    vscode.window.showErrorMessage(String(err));
    panel.webview.postMessage({
      type: 'status',
      text: `Error running fp: ${err}`,
    });
    return;
  }

  try {
    const [outText, blockText, netText] = await Promise.all([
      fs.promises.readFile(outAbs, 'utf8'),
      fs.promises.readFile(blockAbs, 'utf8'),
      fs.promises.readFile(netAbs, 'utf8'),
    ]);

    panel.webview.postMessage({
      type: 'data',
      outText,
      blockText,
      netText,
      meta: {
        outPath: outAbs,
        blockPath: blockAbs,
        netPath: netAbs,
        alpha,
      },
    });
  } catch (err) {
    vscode.window.showErrorMessage(`Failed to read fp outputs: ${err}`);
    panel.webview.postMessage({
      type: 'status',
      text: `Failed to read fp outputs: ${err}`,
    });
  }
}

/**
 * Spawn the fp binary with the correct working directory and relative paths.
 * @param {string} fpBinary
 * @param {string} cwd
 * @param {number} alpha
 * @param {string} blockAbs
 * @param {string} netAbs
 * @param {string} outAbs
 */
function runFpBinary(fpBinary, cwd, alpha, blockAbs, netAbs, outAbs) {
  return new Promise((resolve, reject) => {
    if (!fs.existsSync(fpBinary)) {
      reject(
        new Error(
          `fp binary not found at ${fpBinary}. Did you run "make" in r13943074_pa2?`,
        ),
      );
      return;
    }

    const blockRelToCwd = path.relative(cwd, blockAbs);
    const netRelToCwd = path.relative(cwd, netAbs);
    const outRelToCwd = path.relative(cwd, outAbs);

    const args = [
      String(alpha),
      blockRelToCwd,
      netRelToCwd,
      outRelToCwd,
    ];

    const child = cp.spawn(fpBinary, args, { cwd });

    let stdout = '';
    let stderr = '';

    child.stdout.on('data', (d) => {
      stdout += d.toString();
    });
    child.stderr.on('data', (d) => {
      stderr += d.toString();
    });

    child.on('error', (err) => {
      reject(err);
    });

    child.on('close', (code) => {
      if (code !== 0) {
        reject(
          new Error(
            `fp exited with code ${code}. stderr: ${stderr || '<none>'}`,
          ),
        );
      } else {
        if (stdout.trim().length > 0) {
          vscode.window.showInformationMessage(`fp: ${stdout.trim()}`);
        }
        resolve();
      }
    });
  });
}

function getNonce() {
  let text = '';
  const possible =
    'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
  for (let i = 0; i < 16; i++) {
    text += possible.charAt(Math.floor(Math.random() * possible.length));
  }
  return text;
}

function deactivate() {}

module.exports = {
  activate,
  deactivate,
};

