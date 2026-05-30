import * as fs from "fs";
import * as path from "path";
import * as vscode from "vscode";
import { parseBlockFile, parseFloorplanOutput } from "./parsers";
import { getHtml } from "./getHtml";
import {
  resolvePa2ProjectRoot,
  resolveAnalyticalDir,
  quotePath,
  defaultOpenDirUri,
} from "./pathUtils";
import {
  parseConstraintFile,
  deleteConstraintByIndex,
  updateConstraintByIndex,
  appendConstraint,
} from "./constraintUtils";
import { buildViewPayload } from "./viewPayload";

const EXT_ID = "floorplanViewer";

export function activate(context: vscode.ExtensionContext): void {
  const disposable = vscode.commands.registerCommand(
    `${EXT_ID}.open`,
    () => {
      const panel = vscode.window.createWebviewPanel(
        `${EXT_ID}.panel`,
        "3DIC Floorplan Viewer",
        vscode.ViewColumn.One,
        {
          enableScripts: true,
          retainContextWhenHidden: true,
          localResourceRoots: [
            vscode.Uri.joinPath(context.extensionUri, "media"),
          ],
        }
      );

      panel.webview.html = getHtml(panel.webview, context.extensionUri);

      let constraintFilePath = "";

      function appendLog(message: string): void {
        panel.webview.postMessage({ type: "log", message });
      }

      function loadAndSendConstraint(cPath: string): void {
        try {
          const constraints = parseConstraintFile(cPath);
          panel.webview.postMessage({ type: "constraintData", constraints });
          if (cPath && fs.existsSync(cPath)) {
            appendLog(`Constraints: ${constraints.length} entries`);
          }
        } catch (e) {
          const err = e instanceof Error ? e.message : String(e);
          appendLog(`Constraint load failed: ${err}`);
        }
      }

      function resolveOutputPath(
        analyticalCwd: string,
        outInput: string
      ): string {
        const trimmed = outInput.trim();
        if (!trimmed) { return ""; }
        if (path.isAbsolute(trimmed)) { return trimmed; }
        return path.join(analyticalCwd, trimmed);
      }

      async function loadAndSend2D(
        blockPath: string,
        outputPath: string,
        netsPath?: string
      ): Promise<void> {
        try {
          const blockText = fs.readFileSync(blockPath, "utf8");
          const outText   = fs.readFileSync(outputPath, "utf8");
          const netsText  = (netsPath && fs.existsSync(netsPath))
            ? fs.readFileSync(netsPath, "utf8")
            : undefined;
          const block   = parseBlockFile(blockText);
          const floor   = parseFloorplanOutput(outText);
          const payload = buildViewPayload(block, floor, outText, netsText);
          panel.webview.postMessage({ type: "sceneData", payload });
          appendLog(
            `[OK] Load successful: ${floor.modules.length} modules、` +
            `${floor.tsvs.length} TSVs、${block.terminals.length} terminals` +
            (payload.nets.length > 0 ? `、${payload.nets.length} nets` : "")
          );
        } catch (e) {
          const err = e instanceof Error ? e.message : String(e);
          appendLog(`載入失敗：${err}`);
          vscode.window.showErrorMessage(err);
        }
      }

      panel.webview.onDidReceiveMessage(
        async (msg) => {
          // --- ready ---
          if (msg.type === "ready") {
            const cfg = vscode.workspace.getConfiguration();
            const analyticalDir = resolveAnalyticalDir(cfg, context.extensionPath);
            const folders = vscode.workspace.workspaceFolders;
            const root = resolvePa2ProjectRoot(
              folders?.[0]?.uri.fsPath,
              context.extensionPath
            );
            let defaultBlock      = "";
            let defaultNets       = "";
            let defaultOut        = "";
            let defaultConstraint = "";
            if (root) {
              const guessBlock = path.join(root, "PA2_3DIC", "input_pa2", "n100.block");
              const guessNets  = path.join(root, "PA2_3DIC", "input_pa2", "n100.nets");
              const guessOut   = path.join(root, "PA2_3DIC", "analytical", "output", "n100_output.txt");
              const guessCst   = path.join(root, "PA2_3DIC", "input_pa2", "n100.constraint");
              if (fs.existsSync(guessBlock)) { defaultBlock      = guessBlock; }
              if (fs.existsSync(guessNets))  { defaultNets       = guessNets;  }
              if (fs.existsSync(guessOut))   { defaultOut        = guessOut;   }
              if (fs.existsSync(guessCst))   { defaultConstraint = guessCst;   }
            }
            constraintFilePath = defaultConstraint;
            panel.webview.postMessage({
              type: "setPaths",
              block:      defaultBlock,
              nets:       defaultNets,
              output:     defaultOut,
              constraint: defaultConstraint,
            });

            const presets: {
              label: string;
              block: string;
              nets: string;
              output: string;
              constraint: string;
            }[] = [];
            if (root) {
              for (const id of ["n100", "n200", "n300"]) {
                const bp  = path.join(root, "PA2_3DIC", "input_pa2", `${id}.block`);
                const np  = path.join(root, "PA2_3DIC", "input_pa2", `${id}.nets`);
                const op  = path.join(root, "PA2_3DIC", "analytical", "output", `${id}_output.txt`);
                const cp  = path.join(root, "PA2_3DIC", "input_pa2", `${id}.constraint`);
                if (fs.existsSync(bp) && fs.existsSync(np)) {
                  presets.push({
                    label: `${id} (default paths)`,
                    block: bp,
                    nets:  np,
                    output: op,
                    constraint: fs.existsSync(cp) ? cp : "",
                  });
                }
              }
            }
            panel.webview.postMessage({ type: "presets", presets });
            if (defaultBlock && defaultOut && fs.existsSync(defaultOut)) {
              await loadAndSend2D(defaultBlock, defaultOut, defaultNets);
            }
            if (defaultConstraint) {
              loadAndSendConstraint(defaultConstraint);
            }
            const exe = process.platform === "win32" ? "analytical.exe" : "analytical";
            appendLog(
              analyticalDir
                ? `[OK] executable：${path.join(analyticalDir, exe)}`
                : "[WARN] 找不到 analytical，請設定 floorplanViewer.analyticalDirectory"
            );
            return;
          }

          // --- pickBlock ---
          if (msg.type === "pickBlock") {
            const uris = await vscode.window.showOpenDialog({
              canSelectMany: false,
              defaultUri: defaultOpenDirUri(
                msg.blockPath,
                msg.netsPath,
                msg.constraintPath
              ),
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
              defaultUri: defaultOpenDirUri(
                msg.netsPath,
                msg.blockPath,
                msg.constraintPath
              ),
              filters: { Nets: ["nets", "net"], "All files": ["*"] },
            });
            if (uris?.[0]) {
              panel.webview.postMessage({ type: "setPaths", nets: uris[0].fsPath });
            }
            return;
          }

          // --- loadConstraint（切換 preset 或外部觸發）---
          if (msg.type === "loadConstraint") {
            const p: string = msg.path || "";
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
              defaultUri: defaultOpenDirUri(
                msg.constraintPath,
                msg.blockPath,
                msg.netsPath
              ),
              filters: { Constraint: ["constraint"], "All files": ["*"] },
            });
            if (uris?.[0]) {
              constraintFilePath = uris[0].fsPath;
              panel.webview.postMessage({ type: "setPaths", constraint: constraintFilePath });
              loadAndSendConstraint(constraintFilePath);
            }
            return;
          }

          // --- clearConstraint（使用者按 ✕ 清除）---
          if (msg.type === "clearConstraint") {
            constraintFilePath = "";
            panel.webview.postMessage({ type: "constraintData", constraints: [], reset: true });
            return;
          }

          // --- deleteConstraint ---
          if (msg.type === "deleteConstraint") {
            if (!constraintFilePath || !fs.existsSync(constraintFilePath)) {
              vscode.window.showWarningMessage("No constraint file loaded");
              return;
            }
            try {
              deleteConstraintByIndex(constraintFilePath, msg.index as number);
              appendLog(`[DELETE] Constraint deleted: #${msg.index}`);
              loadAndSendConstraint(constraintFilePath);
            } catch (e) {
              const err = e instanceof Error ? e.message : String(e);
              appendLog(`[ERROR] Constraint delete failed: ${err}`);
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
              updateConstraintByIndex(constraintFilePath, msg.index as number, msg.line as string);
              appendLog(`Updated constraint #${msg.index}`);
              loadAndSendConstraint(constraintFilePath);
            } catch (e) {
              const err = e instanceof Error ? e.message : String(e);
              appendLog(`Update failed: ${err}`);
              vscode.window.showErrorMessage(err);
            }
            return;
          }

          // --- addConstraint（新增一行）---
          if (msg.type === "addConstraint") {
            if (!constraintFilePath) {
              vscode.window.showWarningMessage(
                "No constraint file loaded – please pick a .constraint file first"
              );
              return;
            }
            try {
              appendConstraint(constraintFilePath, msg.line as string);
              appendLog(`[ADD] Constraint added: ${msg.line}`);
              loadAndSendConstraint(constraintFilePath);
            } catch (e) {
              const err = e instanceof Error ? e.message : String(e);
              appendLog(`[ERROR] Constraint add failed: ${err}`);
              vscode.window.showErrorMessage(err);
            }
            return;
          }

          // --- run（使用絕對路徑執行 analytical）---
          if (msg.type === "run") {
            const cfg  = vscode.workspace.getConfiguration();
            const adir = resolveAnalyticalDir(cfg, context.extensionPath);
            if (!adir) {
              vscode.window.showErrorMessage(
                "找不到 analytical：請設定 floorplanViewer.analyticalDirectory 或將工作區開在含 PA2_3DIC/analytical 的專案"
              );
              return;
            }
            const blockFs: string = msg.block;
            const netsFs:  string = msg.nets;
            const outRaw:  string = msg.output;
            const cstFs:   string = (msg.constraint || "").trim();
            if (!blockFs || !netsFs) {
              vscode.window.showWarningMessage("請選取 .block 與 .nets");
              return;
            }
            const outAbs = resolveOutputPath(adir, outRaw);
            if (!outAbs) {
              vscode.window.showWarningMessage("請指定輸出檔路徑");
              return;
            }
            const exe    = process.platform === "win32" ? "analytical.exe" : "analytical";
            const exeAbs = path.join(adir, exe);
            const cstArg = cstFs ? ` ${quotePath(cstFs)}` : "";
            const cmd    = `${quotePath(exeAbs)} ${quotePath(blockFs)} ${quotePath(netsFs)} ${quotePath(outAbs)}${cstArg}`;
            appendLog(`[EXEC] $ ${cmd}`);
            const term = vscode.window.createTerminal({
              name: "analytical floorplan",
              cwd: adir,
            });
            term.show();
            term.sendText(cmd);
            appendLog("已在終端機執行，完成後按「Reload 2D」更新視圖。");
            return;
          }

          // --- reload2d ---
          if (msg.type === "reload2d") {
            const blockPath: string = msg.blockPath;
            const netsPath: string  = msg.netsPath || "";
            const cfg  = vscode.workspace.getConfiguration();
            const adir = resolveAnalyticalDir(cfg, context.extensionPath);
            let outputPath: string = msg.outputPath;
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
            await loadAndSend2D(blockPath, outputPath, netsPath);
            const cPath: string = msg.constraintPath || "";
            if (cPath) {
              constraintFilePath = cPath;
            }
            loadAndSendConstraint(constraintFilePath);
          }
        },
        undefined,
        context.subscriptions
      );
    }
  );

  context.subscriptions.push(disposable);
}

export function deactivate(): void {}
