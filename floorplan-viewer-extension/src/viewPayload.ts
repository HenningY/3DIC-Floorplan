import { type BlockFile, type FloorplanOutput, parseNetsFile } from "./parsers";

export function buildViewPayload(
  block: BlockFile,
  floor: FloorplanOutput,
  outText: string,
  netsText?: string
) {
  const outLines = outText.split(/\r?\n/);
  const hpwl = outLines[0]?.trim() ?? "";
  const die  = outLines[3]?.trim() ?? "";
  const time = outLines[4]?.trim() ?? "";
  const nets = netsText ? parseNetsFile(netsText) : [];
  return {
    outlines:  block.outlines,
    modules:   floor.modules,
    tsvs:      floor.tsvs,
    terminals: block.terminals,
    nets,
    hpwl,
    die,
    time,
  };
}
