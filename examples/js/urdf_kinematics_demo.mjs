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

console.log("URDF model loaded");
console.log({ nq, nv, nlinks });

const q = [0.2, 0.3];
const qd = [0.1, 0.2];
const qdd = [0, 0];

const poses = pino.forwardKinematicsPoses(model, ws, q);
const jacobian = pino.frameJacobian(model, ws, q, 2);
const tau = pino.rnea(model, ws, q, qd, qdd);
const mass = pino.crba(model, ws, q);

console.log("\nForward kinematics link origins:");
for (let i = 0; i < poses.translations.length; i++) {
  console.log(`  link ${i}: [${poses.translations[i].map((v) => v.toFixed(6)).join(", ")}]`);
}

console.log("\nEnd-effector link Jacobian:");
for (const row of jacobian) {
  console.log(`  [${row.map((v) => v.toFixed(6)).join(", ")}]`);
}

console.log("\nInverse dynamics tau:", tau.map((v) => v.toFixed(6)));
console.log("Mass matrix:");
for (const row of mass) {
  console.log(`  [${row.map((v) => v.toFixed(6)).join(", ")}]`);
}

pino.disposeWorkspace(ws);
pino.disposeModel(model);
