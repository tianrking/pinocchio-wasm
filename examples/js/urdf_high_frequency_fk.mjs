import { readFile } from "node:fs/promises";
import { loadPinocchioWasm } from "../../js/pinocchio_wasm.mjs";

const wasmPath = new URL(
  "../../target/wasm32-unknown-unknown/release/pinocchio_wasm.wasm",
  import.meta.url,
);
const urdfPath = new URL("../urdf/two_link.urdf", import.meta.url);

const [wasmBytes, urdf] = await Promise.all([readFile(wasmPath), readFile(urdfPath, "utf8")]);
const pino = await loadPinocchioWasm(wasmBytes);

const model = pino.createModelFromUrdf(urdf);
const ws = pino.newWorkspace(model);

const nq = pino.modelNq(model);
const nv = pino.modelNv(model);
const nlinks = pino.modelNlinks(model);

const qPtr = pino.allocBytes(nq * 8);
const translationsPtr = pino.allocBytes(nlinks * 3 * 8);
const rotationsPtr = pino.allocBytes(nlinks * 9 * 8);
const q = new Float64Array(nq);

function computeFk(q0, q1) {
  q[0] = q0;
  q[1] = q1;
  pino.memoryF64().set(q, qPtr / 8);

  const code = pino.exports.pino_forward_kinematics_poses(
    model,
    ws,
    qPtr,
    translationsPtr,
    rotationsPtr,
  );
  if (code !== 0) throw new Error(`pino_forward_kinematics_poses failed: ${code}`);
}

function toolPointWorld(localPoint = [1, 0, 0]) {
  const memory = pino.memoryF64();
  const link = nlinks - 1;
  const tBase = translationsPtr / 8 + link * 3;
  const rBase = rotationsPtr / 8 + link * 9;

  const tx = memory[tBase];
  const ty = memory[tBase + 1];
  const tz = memory[tBase + 2];

  return [
    tx + memory[rBase] * localPoint[0] + memory[rBase + 1] * localPoint[1] + memory[rBase + 2] * localPoint[2],
    ty + memory[rBase + 3] * localPoint[0] + memory[rBase + 4] * localPoint[1] + memory[rBase + 5] * localPoint[2],
    tz + memory[rBase + 6] * localPoint[0] + memory[rBase + 7] * localPoint[1] + memory[rBase + 8] * localPoint[2],
  ];
}

computeFk(0.2, 0.3);
console.log("URDF high-frequency FK model:", { nq, nv, nlinks });
console.log("tool point:", toolPointWorld().map((v) => v.toFixed(6)));

const iterations = 200_000;
const t0 = performance.now();
for (let i = 0; i < iterations; i++) {
  computeFk(0.7 * Math.sin(i * 0.003), 0.9 * Math.cos(i * 0.002));
}
const elapsedMs = performance.now() - t0;
console.log(`${iterations} FK calls in ${elapsedMs.toFixed(3)} ms`);
console.log(`${((elapsedMs * 1000) / iterations).toFixed(3)} us/call`);
console.log("final tool point:", toolPointWorld().map((v) => v.toFixed(6)));

pino.freeBytes(qPtr, nq * 8);
pino.freeBytes(translationsPtr, nlinks * 3 * 8);
pino.freeBytes(rotationsPtr, nlinks * 9 * 8);
pino.disposeWorkspace(ws);
pino.disposeModel(model);
