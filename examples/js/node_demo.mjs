import { readFile } from "node:fs/promises";
import { loadPinocchioWasm } from "../../js/pinocchio_wasm.mjs";

const wasmPath = new URL("../../target/wasm32-unknown-unknown/release/pinocchio_wasm.wasm", import.meta.url);
const wasmBytes = await readFile(wasmPath);
const pino = await loadPinocchioWasm(wasmBytes);

// ---------------------------------------------------------------------------
// 2-link pendulum model
// ---------------------------------------------------------------------------
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
    {
      name: "l2",
      parent: 1,
      mass: 1,
      com: [0.5, 0, 0],
      inertia: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      joint: { axis: [0, 0, 1], origin: [1, 0, 0] },
    },
  ],
});

console.log("=== Pinocchio WASM JS SDK Demo ===\n");

const model = pino.createModelFromJson(modelJson);
const ws = pino.newWorkspace(model);

const nq = pino.modelNq(model);
const nl = pino.modelNlinks(model);
console.log(`Model: ${nl} links, ${nq} DOF\n`);

const q = [0.2, 0.3];
const qd = [0.1, 0.2];
const qdd = [0.0, 0.0];
const tau = [1.0, 0.5];

// ---------------------------------------------------------------------------
// 1. ABA (forward dynamics)
// ---------------------------------------------------------------------------
const qddAba = pino.aba(model, ws, q, qd, tau);
console.log("ABA (forward dynamics):");
console.log("  q =", q);
console.log("  qd =", qd);
console.log("  tau =", tau);
console.log("  qdd =", qddAba.map(v => v.toFixed(6)));
console.log();

// ---------------------------------------------------------------------------
// 2. RNEA (inverse dynamics)
// ---------------------------------------------------------------------------
const tauRnea = pino.rnea(model, ws, q, qd, qdd);
console.log("RNEA (inverse dynamics):");
console.log("  q =", q);
console.log("  qd =", qd);
console.log("  qdd =", qdd);
console.log("  tau =", tauRnea.map(v => v.toFixed(6)));
console.log();

// ---------------------------------------------------------------------------
// 3. CRBA (mass matrix)
// ---------------------------------------------------------------------------
const M = pino.crba(model, ws, q);
console.log("CRBA (mass matrix):");
for (const row of M) {
  console.log("  [" + row.map(v => v.toFixed(6)).join(", ") + "]");
}
console.log();

// ---------------------------------------------------------------------------
// 4. Gravity torques
// ---------------------------------------------------------------------------
const grav = pino.gravityTorques(model, ws, q);
console.log("Gravity torques:", grav.map(v => v.toFixed(6)));
console.log();

// ---------------------------------------------------------------------------
// 5. Coriolis torques
// ---------------------------------------------------------------------------
const cor = pino.coriolisTorques(model, ws, q, qd);
console.log("Coriolis torques:", cor.map(v => v.toFixed(6)));
console.log();

// ---------------------------------------------------------------------------
// 6. Center of mass
// ---------------------------------------------------------------------------
const com = pino.centerOfMass(model, ws, q);
console.log("Center of mass:", com.map(v => v.toFixed(6)));
console.log();

// ---------------------------------------------------------------------------
// 7. Energy
// ---------------------------------------------------------------------------
const e = pino.energy(model, ws, q, qd);
console.log("Energy:");
console.log("  kinetic:", e.kinetic.toFixed(6));
console.log("  potential:", e.potential.toFixed(6));
console.log();

// ---------------------------------------------------------------------------
// 8. Compute all terms at once
// ---------------------------------------------------------------------------
const all = pino.computeAllTerms(model, ws, q, qd);
console.log("Compute all terms:");
console.log("  mass matrix:");
for (const row of all.mass) {
  console.log("    [" + row.map(v => v.toFixed(6)).join(", ") + "]");
}
console.log("  bias forces:", all.bias.map(v => v.toFixed(6)));
console.log("  gravity torques:", all.gravity.map(v => v.toFixed(6)));
console.log("  coriolis torques:", all.coriolis.map(v => v.toFixed(6)));
console.log("  center of mass:", all.com.map(v => v.toFixed(6)));
console.log("  kinetic energy:", all.kineticEnergy.toFixed(6));
console.log("  potential energy:", all.potentialEnergy.toFixed(6));
console.log();

// ---------------------------------------------------------------------------
// 9. Forward kinematics poses
// ---------------------------------------------------------------------------
const poses = pino.forwardKinematicsPoses(model, ws, q);
console.log("Forward kinematics poses:");
for (let i = 0; i < poses.translations.length; i++) {
  console.log(`  link ${i}: pos = [${poses.translations[i].map(v => v.toFixed(6)).join(", ")}]`);
}
console.log();

// ---------------------------------------------------------------------------
// 10. Frame Jacobian (for link 2, the end-effector)
// ---------------------------------------------------------------------------
const J = pino.frameJacobian(model, ws, q, 2);
console.log("Frame Jacobian (link 2):");
for (const row of J) {
  console.log("  [" + row.map(v => v.toFixed(6)).join(", ") + "]");
}
console.log();

// ---------------------------------------------------------------------------
// 11. Model export
// ---------------------------------------------------------------------------
try {
  const json = pino.modelToJson(model);
  console.log("Model JSON export (first 200 chars):", json.substring(0, 200) + "...");
} catch (e) {
  console.log("Model JSON export: skipped (", e.message, ")");
}
console.log();

// ---------------------------------------------------------------------------
// Cleanup
// ---------------------------------------------------------------------------
pino.disposeWorkspace(ws);
pino.disposeModel(model);

console.log("=== Demo complete ===");
