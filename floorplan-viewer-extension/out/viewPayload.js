"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.buildViewPayload = buildViewPayload;
const parsers_1 = require("./parsers");
function buildViewPayload(block, floor, outText, netsText) {
    const outLines = outText.split(/\r?\n/);
    const hpwl = outLines[0]?.trim() ?? "";
    const die = outLines[3]?.trim() ?? "";
    const time = outLines[4]?.trim() ?? "";
    const nets = netsText ? (0, parsers_1.parseNetsFile)(netsText) : [];
    const softNames = new Set(block.blocks.filter((b) => b.isSoft).map((b) => b.name));
    return {
        outlines: block.outlines,
        modules: floor.modules.map((m) => ({
            ...m,
            isSoft: softNames.has(m.name),
        })),
        tsvs: floor.tsvs,
        terminals: block.terminals,
        nets,
        hpwl,
        die,
        time,
    };
}
//# sourceMappingURL=viewPayload.js.map