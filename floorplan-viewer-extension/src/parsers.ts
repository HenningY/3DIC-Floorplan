/**
 * Parse PA2 .block and analytical floorplan output for 2D visualization.
 */

export interface DieOutline {
  width: number;
  height: number;
}

export interface BlockDef {
  name: string;
  width: number;
  height: number;
  tier: number;
  isSoft?: boolean;
}

export interface TerminalPin {
  name: string;
  x: number;
  y: number;
}

export interface BlockFile {
  numDie: number;
  outlines: DieOutline[];
  blocks: BlockDef[];
  terminals: TerminalPin[];
}

export interface PlacedModule {
  name: string;
  tier: number;
  xll: number;
  yll: number;
  xur: number;
  yur: number;
}

export interface TsvAssignment {
  netName: string;
  tierBelow: number;
  tierAbove: number;
  x: number;
  y: number;
}

export interface FloorplanOutput {
  modules: PlacedModule[];
  tsvs: TsvAssignment[];
}

export interface NetDef {
  /** pin 名稱列表，第一個通常是 terminal，也對應 TsvAssignment.netName */
  pins: string[];
}

export function parseNetsFile(text: string): NetDef[] {
  const lines = text.split(/\r?\n/);
  const nets: NetDef[] = [];
  let i = 0;
  // 跳過 NumNets 行，直接掃描 NetDegree
  while (i < lines.length) {
    const m = lines[i].trim().match(/^NetDegree:\s*(\d+)/i);
    if (m) {
      const degree = parseInt(m[1], 10);
      const pins: string[] = [];
      for (let j = 0; j < degree; j++) {
        i++;
        if (i < lines.length) {
          const pin = lines[i].trim();
          if (pin) { pins.push(pin); }
        }
      }
      if (pins.length > 0) { nets.push({ pins }); }
    }
    i++;
  }
  return nets;
}

export function parseBlockFile(text: string): BlockFile {
  const lines = text.split(/\r?\n/).filter((l) => l.trim().length > 0);
  let i = 0;

  const numDieLine = lines[i++].match(/NumDie:\s*(\d+)/i);
  if (!numDieLine) {
    throw new Error("Expected NumDie in .block file");
  }
  const numDie = parseInt(numDieLine[1], 10);

  // Outline 允許整數或小數（例如 187 或 187.39000）
  const outlineRe =
    /Outline:\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s+([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)/i;
  const outlines: DieOutline[] = [];
  for (let d = 0; d < numDie; d++) {
    const m = lines[i++].match(outlineRe);
    if (!m) {
      throw new Error(`Expected Outline line ${d + 1}`);
    }
    outlines.push({
      width: parseFloat(m[1]),
      height: parseFloat(m[2]),
    });
  }

  const nb = lines[i++].match(/NumBlocks:\s*(\d+)/i);
  if (!nb) {
    throw new Error("Expected NumBlocks");
  }
  const numBlocks = parseInt(nb[1], 10);

  const nt = lines[i++].match(/NumTerminals:\s*(\d+)/i);
  if (!nt) {
    throw new Error("Expected NumTerminals");
  }

  // 跳過 Weight: 等 header 行，找到第一個 block 行（格式：name w h tier）
  while (i < lines.length) {
    const parts = lines[i].trim().split(/\s+/);
    if (
      parts.length >= 4 &&
      !isNaN(parseFloat(parts[1])) &&
      parts[1].toLowerCase() !== "terminal"
    ) {
      break;
    }
    i++;
  }

  const blocks: BlockDef[] = [];
  for (let b = 0; b < numBlocks && i < lines.length; b++) {
    const parts = lines[i++].trim().split(/\s+/);
    if (parts.length < 4) {
      throw new Error(`Bad block line: ${lines[i - 1]}`);
    }
    blocks.push({
      name: parts[0],
      width: parseFloat(parts[1]),
      height: parseFloat(parts[2]),
      tier: parseInt(parts[3], 10),
      isSoft: parts.length > 4 && parts[4].toLowerCase() === "s",
    });
  }

  // 解析 terminal 行（格式：name terminal x y）
  const terminals: TerminalPin[] = [];
  while (i < lines.length) {
    const parts = lines[i++].trim().split(/\s+/);
    if (parts.length >= 4 && parts[1].toLowerCase() === "terminal") {
      terminals.push({
        name: parts[0],
        x: parseFloat(parts[2]),
        y: parseFloat(parts[3]),
      });
    }
  }

  return { numDie, outlines, blocks, terminals };
}

/**
 * write_output 格式：5 header lines，then modules，optional NumTsvAssignments + lines。
 */
export function parseFloorplanOutput(text: string): FloorplanOutput {
  const lines = text.split(/\r?\n/);
  let idx = 5;
  const modules: PlacedModule[] = [];
  const tsvs: TsvAssignment[] = [];

  while (idx < lines.length) {
    const line = lines[idx].trim();
    if (!line) {
      idx++;
      continue;
    }
    if (line.startsWith("NumTsvAssignments")) {
      idx++;
      break;
    }
    const parts = line.split(/\s+/);
    if (parts.length >= 6) {
      modules.push({
        name: parts[0],
        tier: parseInt(parts[1], 10),
        xll: parseFloat(parts[2]),
        yll: parseFloat(parts[3]),
        xur: parseFloat(parts[4]),
        yur: parseFloat(parts[5]),
      });
    }
    idx++;
  }

  while (idx < lines.length) {
    const line = lines[idx].trim();
    if (!line) {
      idx++;
      continue;
    }
    const m = line.match(
      /^(\S+)\s+tier(\d+)-(\d+)\s+([0-9.+-eE]+)\s+([0-9.+-eE]+)/
    );
    if (m) {
      tsvs.push({
        netName: m[1],
        tierBelow: parseInt(m[2], 10),
        tierAbove: parseInt(m[3], 10),
        x: parseFloat(m[4]),
        y: parseFloat(m[5]),
      });
    }
    idx++;
  }

  return { modules, tsvs };
}
