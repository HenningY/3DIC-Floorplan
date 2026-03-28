/**
 * Parse PA2 .block and analytical floorplan output for 3D visualization.
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
}

export interface BlockFile {
  numDie: number;
  outlines: DieOutline[];
  blocks: BlockDef[];
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

export function parseBlockFile(text: string): BlockFile {
  const lines = text.split(/\r?\n/).filter((l) => l.trim().length > 0);
  let i = 0;
  const numDieLine = lines[i++].match(/NumDie:\s*(\d+)/i);
  if (!numDieLine) {
    throw new Error("Expected NumDie in .block file");
  }
  const numDie = parseInt(numDieLine[1], 10);
  const outlines: DieOutline[] = [];
  for (let d = 0; d < numDie; d++) {
    const m = lines[i++].match(/Outline:\s*(\d+)\s+(\d+)/i);
    if (!m) {
      throw new Error(`Expected Outline line ${d + 1}`);
    }
    outlines.push({ width: parseInt(m[1], 10), height: parseInt(m[2], 10) });
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
  const blocks: BlockDef[] = [];
  for (let b = 0; b < numBlocks; b++) {
    const parts = lines[i++].trim().split(/\s+/);
    if (parts.length < 4) {
      throw new Error(`Bad block line: ${lines[i - 1]}`);
    }
    const name = parts[0];
    const w = parseFloat(parts[1]);
    const h = parseFloat(parts[2]);
    const tier = parseInt(parts[3], 10);
    blocks.push({ name, width: w, height: h, tier });
  }
  return { numDie, outlines, blocks };
}

/**
 * write_output format: 5 header lines, then modules, optional NumTsvAssignments + lines.
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
      const name = parts[0];
      const tier = parseInt(parts[1], 10);
      const xll = parseInt(parts[2], 10);
      const yll = parseInt(parts[3], 10);
      const xur = parseInt(parts[4], 10);
      const yur = parseInt(parts[5], 10);
      modules.push({ name, tier, xll, yll, xur, yur });
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
