import { readFile } from "node:fs/promises";
import { loadPinocchioWasm } from "../../js/pinocchio_wasm.mjs";

const wasmPath = new URL("../../target/wasm32-unknown-unknown/release/pinocchio_wasm.wasm", import.meta.url);
const wasmBytes = await readFile(wasmPath);
const pino = await loadPinocchioWasm(wasmBytes);

const modelJson = JSON.stringify({
  links: [
    {
      name: "base",
      parent: null,
      mass: 0.1,
      com: [0, 0, 0],
      inertia: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      joint: null,
    },
    {
      name: "l1",
      parent: 0,
      mass: 1,
      com: [0.5, 0, 0],
      inertia: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      joint: { axis: [0, 0, 1], origin: [0, 0, 0] },
    },
  ],
});

const model = pino.createModelFromJson(modelJson);
const ws = pino.newWorkspace(model);
const qdd = pino.aba(model, ws, [0.2], [0.1], [1.0]);
console.log("qdd:", qdd);

pino.disposeWorkspace(ws);
pino.disposeModel(model);
