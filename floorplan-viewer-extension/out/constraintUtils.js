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
exports.parseConstraintFile = parseConstraintFile;
exports.deleteConstraintByIndex = deleteConstraintByIndex;
exports.updateConstraintByIndex = updateConstraintByIndex;
exports.appendConstraint = appendConstraint;
const fs = __importStar(require("fs"));
function parseConstraintFile(cPath) {
    if (!cPath || !fs.existsSync(cPath)) {
        return [];
    }
    const text = fs.readFileSync(cPath, "utf8");
    const result = [];
    for (const l of text.split(/\r?\n/)) {
        if (!l.trim()) {
            continue;
        }
        const parts = l.trim().split(/\s+/);
        if (parts[0] === "FIXED") {
            result.push({
                type: "FIXED",
                name: parts[1],
                llx: +parts[2],
                lly: +parts[3],
                urx: +parts[4],
                ury: +parts[5],
            });
        }
        else if (parts[0] === "REPULSE") {
            result.push({
                type: "REPULSE",
                strength: +parts[1],
                names: parts.slice(2),
            });
        }
    }
    return result;
}
function deleteConstraintByIndex(filePath, index) {
    const text = fs.readFileSync(filePath, "utf8");
    const allLines = text.split(/\r?\n/);
    const nonEmpty = allLines.filter((l) => l.trim().length > 0);
    if (index < 0 || index >= nonEmpty.length) {
        return;
    }
    const targetLine = nonEmpty[index];
    let removed = false;
    const newLines = allLines.filter((l) => {
        if (!removed && l === targetLine) {
            removed = true;
            return false;
        }
        return true;
    });
    fs.writeFileSync(filePath, newLines.join("\n"));
}
function updateConstraintByIndex(filePath, index, newLine) {
    const text = fs.readFileSync(filePath, "utf8");
    const allLines = text.split(/\r?\n/);
    const nonEmpty = allLines.filter((l) => l.trim().length > 0);
    if (index < 0 || index >= nonEmpty.length) {
        return;
    }
    const oldLine = nonEmpty[index];
    let replaced = false;
    const newLines = allLines.map((l) => {
        if (!replaced && l === oldLine) {
            replaced = true;
            return newLine;
        }
        return l;
    });
    fs.writeFileSync(filePath, newLines.join("\n"));
}
function appendConstraint(filePath, line) {
    const existing = fs.existsSync(filePath)
        ? fs.readFileSync(filePath, "utf8")
        : "";
    const trimmed = existing.trimEnd();
    const newContent = trimmed
        ? trimmed + "\n" + line + "\n"
        : line + "\n";
    fs.writeFileSync(filePath, newContent);
}
//# sourceMappingURL=constraintUtils.js.map