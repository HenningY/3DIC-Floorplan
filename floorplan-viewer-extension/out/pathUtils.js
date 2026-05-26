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
exports.resolvePa2ProjectRoot = resolvePa2ProjectRoot;
exports.quotePath = quotePath;
exports.resolveAnalyticalDir = resolveAnalyticalDir;
const fs = __importStar(require("fs"));
const path = __importStar(require("path"));
const vscode = __importStar(require("vscode"));
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
//# sourceMappingURL=pathUtils.js.map