import * as fs from "fs";
import * as path from "path";
import * as vscode from "vscode";

/**
 * 專案根目錄：底下有 `PA2_3DIC/input_pa2`（例如 .../PD）。
 * 若工作區開在 `floorplan-viewer-extension` 或 `PA2_3DIC`，會從路徑往上找。
 */
export function resolvePa2ProjectRoot(
  workspaceRoot: string | undefined,
  extensionPath: string
): string | undefined {
  const seeds: string[] = [];
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

export function quotePath(p: string): string {
  if (process.platform === "win32") {
    return `"${p.replace(/"/g, '\\"')}"`;
  }
  return `'${p.replace(/'/g, "'\\''")}'`;
}

export function resolveAnalyticalDir(
  config: vscode.WorkspaceConfiguration,
  extensionPath?: string
): string | undefined {
  const custom = config
    .get<string>("floorplanViewer.analyticalDirectory")
    ?.trim();
  if (custom) {
    const abs = path.resolve(custom);
    if (fs.existsSync(abs)) {
      return abs;
    }
  }
  const folders = vscode.workspace.workspaceFolders;
  const ws0 = folders?.[0]?.uri.fsPath;
  const seeds: string[] = [];
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
  const exe =
    process.platform === "win32" ? "analytical.exe" : "analytical";
  const candidateDirs: string[] = [];
  for (const root of uniq) {
    candidateDirs.push(
      path.join(root, "PA2_3DIC", "analytical"),
      path.join(root, "PD", "PA2_3DIC", "analytical"),
      path.join(root, "analytical")
    );
  }
  for (const c of candidateDirs) {
    if (fs.existsSync(path.join(c, exe))) {
      return c;
    }
  }
  return undefined;
}
