import * as fs from "fs";

export const BOUNDARY_SIDE_LABELS = [
  "",
  "LEFT",
  "RIGHT",
  "TOP",
  "BOTTOM",
  "TOP-LEFT",
  "TOP-RIGHT",
  "BOTTOM-LEFT",
  "BOTTOM-RIGHT",
] as const;

export interface ConstraintEntry {
  type: "FIXED" | "REPULSE" | "BOUNDARY";
  // FIXED
  name?: string;
  llx?: number;
  lly?: number;
  urx?: number;
  ury?: number;
  // REPULSE
  minDist?: number;
  names?: string[];
  // BOUNDARY
  side?: number;
}

export function parseConstraintFile(cPath: string): ConstraintEntry[] {
  if (!cPath || !fs.existsSync(cPath)) {
    return [];
  }
  const text = fs.readFileSync(cPath, "utf8");
  const result: ConstraintEntry[] = [];
  for (const l of text.split(/\r?\n/)) {
    const trimmed = l.trim();
    if (!trimmed || trimmed.startsWith("#")) { continue; }
    const parts = trimmed.split(/\s+/);
    if (parts[0] === "FIXED") {
      result.push({
        type: "FIXED",
        name: parts[1],
        llx: +parts[2],
        lly: +parts[3],
        urx: +parts[4],
        ury: +parts[5],
      });
    } else if (parts[0] === "REPULSE") {
      result.push({
        type: "REPULSE",
        minDist: +parts[1],
        names: parts.slice(2),
      });
    } else if (parts[0] === "BOUNDARY") {
      result.push({
        type: "BOUNDARY",
        name: parts[1],
        side: +parts[2],
      });
    }
  }
  return result;
}

export function deleteConstraintByIndex(filePath: string, index: number): void {
  const text = fs.readFileSync(filePath, "utf8");
  const allLines = text.split(/\r?\n/);
  const nonEmpty = allLines.filter((l) => l.trim().length > 0);
  if (index < 0 || index >= nonEmpty.length) { return; }
  const targetLine = nonEmpty[index];
  let removed = false;
  const newLines = allLines.filter((l) => {
    if (!removed && l === targetLine) { removed = true; return false; }
    return true;
  });
  fs.writeFileSync(filePath, newLines.join("\n"));
}

export function updateConstraintByIndex(
  filePath: string,
  index: number,
  newLine: string
): void {
  const text = fs.readFileSync(filePath, "utf8");
  const allLines = text.split(/\r?\n/);
  const nonEmpty = allLines.filter((l) => l.trim().length > 0);
  if (index < 0 || index >= nonEmpty.length) { return; }
  const oldLine = nonEmpty[index];
  let replaced = false;
  const newLines = allLines.map((l) => {
    if (!replaced && l === oldLine) { replaced = true; return newLine; }
    return l;
  });
  fs.writeFileSync(filePath, newLines.join("\n"));
}

export function appendConstraint(filePath: string, line: string): void {
  const existing = fs.existsSync(filePath)
    ? fs.readFileSync(filePath, "utf8")
    : "";
  const trimmed = existing.trimEnd();
  const newContent = trimmed
    ? trimmed + "\n" + line + "\n"
    : line + "\n";
  fs.writeFileSync(filePath, newContent);
}
