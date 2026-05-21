# JavaScript Usage and Verification Guide

This document explains how to verify the existing `pinocchio_wasm` functionality and how to use the JavaScript SDK from Node.js or a browser.

## 1. Build and Verify the Project

Run these commands from the repository root:

```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features
cargo test --all-targets --all-features
rustup target add wasm32-unknown-unknown
cargo build --release --target wasm32-unknown-unknown
node --check js/pinocchio_wasm.mjs
node --check js/sdk/runtime.mjs
node examples/js/node_demo.mjs
node examples/js/urdf_kinematics_demo.mjs
node examples/js/urdf_high_frequency_fk.mjs
```

Expected results:

- `cargo fmt --all -- --check`: no diff.
- `cargo clippy --all-targets --all-features`: exit code 0. Warnings may still be printed unless they are promoted to errors.
- `cargo test --all-targets --all-features`: all Rust, FFI, model, dynamics, contact, collision, and quaternion/config-space tests pass.
- `cargo build --release --target wasm32-unknown-unknown`: creates `target/wasm32-unknown-unknown/release/pinocchio_wasm.wasm`.
- `node examples/js/node_demo.mjs`: prints dynamics, mass matrix, CoM, energy, FK poses, Jacobian, and JSON export output.
- `node examples/js/urdf_kinematics_demo.mjs`: loads `examples/urdf/two_link.urdf` and computes FK/Jacobian/dynamics from the URDF model.
- `node examples/js/urdf_high_frequency_fk.mjs`: loads the same URDF and benchmarks direct-buffer FK.

For a function-by-function SDK reference, see `docs/JS_API_REFERENCE.md`.

To run the browser kinematics demo:

```bash
cargo build --release --target wasm32-unknown-unknown
python3 -m http.server 8000
```

Then open:

```text
http://localhost:8000/examples/js/browser_kinematics_demo.html
http://localhost:8000/examples/js/browser_urdf_kinematics.html
```

Do not open the HTML file directly with a `file://` URL. Browsers usually block `fetch()` and ES module imports from local files.

## 2. What the Current Tests Cover

The Rust test suite is the strongest verification layer. Important areas:

| Area | Representative tests |
| --- | --- |
| Dynamics | `tests/dynamics.rs`, `tests/joint_types.rs` |
| ABA derivatives | `tests/aba_derivatives.rs` |
| RNEA derivatives | `tests/rnea_derivatives_analytical.rs` |
| Kinematics derivatives | `tests/kinematics_derivatives_analytical.rs` |
| Batch APIs | `tests/batch.rs`, `tests/ffi_extended.rs` |
| Contact dynamics | `tests/contact_dynamics.rs`, `tests/contact_friction.rs` |
| Collision | `tests/collision.rs`, `tests/ffi_collision.rs` |
| Model import/export | `tests/model_json.rs`, `tests/model_urdf.rs`, `tests/model_sdf.rs`, `tests/model_mjcf.rs`, `tests/model_serde.rs` |
| Floating joints | `tests/floating_joints.rs` |
| Quaternion/config-space | `tests/quaternion_and_config.rs` |
| FFI smoke tests | files named `tests/ffi_*.rs` |

Good sanity checks:

- `aba` should invert `rnea` in tested cases.
- `crba` mass matrices should be symmetric and match dynamics closure checks.
- `computeAllTerms` should agree with separate `crba`, `bias`, `gravity`, `coriolis`, `centerOfMass`, and `energy` calls.
- Spherical/free-flyer configurations should keep quaternions normalized after model-aware integration/interpolation/random sampling.

## 3. JavaScript SDK Overview

The SDK entry point is:

```js
import { loadPinocchioWasm } from "./js/pinocchio_wasm.mjs";
```

`loadPinocchioWasm(wasmBytes)` returns a `pino` object with high-level wrappers around the WASM C ABI.

The SDK uses opaque numeric handles:

- `model`: returned by `createModel*`
- `ws`: returned by `newWorkspace(model)`
- `collision`: returned by `createCollisionModel*`

Always release handles when done:

```js
pino.disposeWorkspace(ws);
pino.disposeModel(model);
pino.disposeCollisionModel(collision);
```

## 4. Node.js Minimal Example

Build the WASM first:

```bash
cargo build --release --target wasm32-unknown-unknown
```

Then run a script like this:

```js
import { readFile } from "node:fs/promises";
import { loadPinocchioWasm } from "./js/pinocchio_wasm.mjs";

const wasmBytes = await readFile(
  "./target/wasm32-unknown-unknown/release/pinocchio_wasm.wasm"
);
const pino = await loadPinocchioWasm(wasmBytes);

const model = pino.createModel([
  {
    parent: -1,
    mass: 0.1,
    com: [0, 0, 0],
    inertia: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
  },
  {
    parent: 0,
    mass: 1,
    com: [0.5, 0, 0],
    inertia: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    joint: { type: "revolute", axis: [0, 0, 1], origin: [0, 0, 0] },
  },
  {
    parent: 1,
    mass: 1,
    com: [0.5, 0, 0],
    inertia: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    joint: { type: "revolute", axis: [0, 0, 1], origin: [1, 0, 0] },
  },
]);

const ws = pino.newWorkspace(model);

const nq = pino.modelNq(model);
const nv = pino.modelNv(model);
console.log({ nq, nv, nlinks: pino.modelNlinks(model) });

const q = [0.2, 0.3];
const qd = [0.1, 0.2];
const qdd = [0.0, 0.0];
const tau = [1.0, 0.5];

console.log("rnea tau:", pino.rnea(model, ws, q, qd, qdd));
console.log("aba qdd:", pino.aba(model, ws, q, qd, tau));
console.log("mass:", pino.crba(model, ws, q));
console.log("com:", pino.centerOfMass(model, ws, q));
console.log("energy:", pino.energy(model, ws, q, qd));

pino.disposeWorkspace(ws);
pino.disposeModel(model);
```

The repository already includes a larger Node demo at `examples/js/node_demo.mjs`.

## 5. Browser Minimal Example

In a browser or bundler environment, fetch the `.wasm` file as bytes:

```js
import { loadPinocchioWasm } from "./js/pinocchio_wasm.mjs";

const response = await fetch("/pinocchio_wasm.wasm");
const wasmBytes = await response.arrayBuffer();
const pino = await loadPinocchioWasm(wasmBytes);

const model = pino.createModel([
  { parent: -1, mass: 0.1, com: [0, 0, 0] },
  {
    parent: 0,
    mass: 1,
    com: [0.5, 0, 0],
    joint: { type: "revolute", axis: [0, 0, 1], origin: [0, 0, 0] },
  },
]);
const ws = pino.newWorkspace(model);

const q = [0.2];
const qd = [0.1];
const tau = [1.0];
const qdd = pino.aba(model, ws, q, qd, tau);

console.log(qdd);

pino.disposeWorkspace(ws);
pino.disposeModel(model);
```

Serve `target/wasm32-unknown-unknown/release/pinocchio_wasm.wasm` from your app or copy it into your public/static assets.

## 5.1 High-Frequency Browser FK Pattern

For simple scripts and UI actions, the high-level SDK calls are easiest:

```js
const poses = pino.forwardKinematicsPoses(model, ws, q);
```

For animation, control loops, simulation previews, or thousands of FK calls per frame, prefer preallocated WASM memory and direct FFI calls. This avoids repeated JS array conversion and repeated allocation.

The repository includes a complete browser example:

- `examples/js/browser_kinematics_demo.html`
- `examples/js/browser_kinematics_demo.mjs`
- `examples/js/browser_urdf_kinematics.html`
- `examples/js/browser_urdf_kinematics.mjs`

The key pattern is:

```js
const qPtr = pino.allocBytes(nq * 8);
const translationsPtr = pino.allocBytes(nlinks * 3 * 8);
const rotationsPtr = pino.allocBytes(nlinks * 9 * 8);
const q = new Float64Array(nq);

function computeFk(qValues) {
  q.set(qValues);
  pino.memoryF64().set(q, qPtr / 8);

  const code = pino.exports.pino_forward_kinematics_poses(
    model,
    ws,
    qPtr,
    translationsPtr,
    rotationsPtr
  );
  if (code !== 0) throw new Error(`FK failed: ${code}`);

  const memory = pino.memoryF64();
  const base = translationsPtr / 8;
  const link0 = [memory[base], memory[base + 1], memory[base + 2]];
  return link0;
}

// Later, when done:
pino.freeBytes(qPtr, nq * 8);
pino.freeBytes(translationsPtr, nlinks * 3 * 8);
pino.freeBytes(rotationsPtr, nlinks * 9 * 8);
```

Use this pattern when performance matters:

- allocate buffers once
- reuse one `Workspace`
- write into WASM memory with `Float64Array#set`
- call `pino.exports.pino_*` directly
- read only the output values you actually need
- free buffers and handles when the page or worker is done

For browser production use, a good architecture is:

- main thread: UI, rendering, user input
- Web Worker: WASM load, model/workspace ownership, FK/dynamics calls
- messages: compact typed arrays or SharedArrayBuffer when available

This keeps large FK batches from blocking interaction and makes the WASM side easier to keep hot.

## 6. Model Creation

### Structured JS Model

`createModel(links)` expects the first link to be the root. Every non-root link needs a `parent` and usually a `joint`.

```js
const model = pino.createModel([
  {
    parent: -1,
    mass: 0.1,
    com: [0, 0, 0],
    inertia: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
  },
  {
    parent: 0,
    mass: 1,
    com: [0.5, 0, 0],
    inertia: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    joint: {
      type: "revolute",
      axis: [0, 0, 1],
      origin: [0, 0, 0],
    },
  },
]);
```

Supported joint types:

| JS `joint.type` | `nq` | `nv` | Meaning |
| --- | ---: | ---: | --- |
| `"revolute"` or omitted | 1 | 1 | rotation around `axis` |
| `"prismatic"` | 1 | 1 | translation along `axis` |
| `"fixed"` | 0 | 0 | no motion |
| `"spherical"` | 4 | 3 | quaternion orientation |
| `"freeflyer"`, `"free_flyer"`, or `"floating"` | 7 | 6 | xyz + quaternion |

Important: for spherical and free-flyer joints, `nq !== nv`. Use `pino.modelNq(model)` for `q` length and `pino.modelNv(model)` for `qd`, `qdd`, and `tau` length.

### JSON/URDF/SDF/MJCF

You can also load textual model formats:

```js
const modelFromJson = pino.createModelFromJson(jsonString);
const modelFromUrdf = pino.createModelFromUrdf(urdfString);
const modelFromSdf = pino.createModelFromSdf(sdfString);
const modelFromMjcf = pino.createModelFromMjcf(mjcfString);
```

URDF kinematics example:

```js
const response = await fetch("/models/robot.urdf");
const urdf = await response.text();

const model = pino.createModelFromUrdf(urdf);
const ws = pino.newWorkspace(model);

const q = Array(pino.modelNq(model)).fill(0);
q[0] = 0.2;
q[1] = 0.3;

const poses = pino.forwardKinematicsPoses(model, ws, q);
const jacobian = pino.frameJacobian(model, ws, q, pino.modelNlinks(model) - 1);

console.log(poses.translations);
console.log(jacobian);

pino.disposeWorkspace(ws);
pino.disposeModel(model);
```

Current URDF loader support:

| URDF feature | Status |
| --- | --- |
| `<robot>` root | supported |
| `<link name="...">` | supported |
| `<inertial>`, `<mass value="...">`, `<inertia ...>` | supported |
| inertial `<origin xyz="...">` | supported |
| joint `<parent>`, `<child>` | supported |
| joint `<origin xyz="...">` | supported |
| joint `<axis xyz="...">` | supported |
| `revolute`, `continuous` | supported as revolute |
| `prismatic` | supported |
| `fixed` | supported |
| `ball`, `spherical` | supported |
| `floating`, `freeflyer`, `free_flyer` | supported |
| `origin rpy` | not parsed yet |
| `<limit>`, `<dynamics>`, `<mimic>`, `<transmission>` | not parsed yet |
| `<visual>`, `<collision>`, mesh geometry | not parsed by the model loader |

For accurate current kinematics, keep joint frame rotations expressed without `rpy`, or pre-process the URDF into the subset above.

Export helpers:

```js
const json = pino.modelToJson(model);
const urdf = pino.modelToUrdf(model, "robot_name");
const sdf = pino.modelToSdf(model, "model_name");
const mjcf = pino.modelToMjcf(model, "model_name");
```

## 7. Core JS APIs

### Dimensions and lifecycle

```js
const nq = pino.modelNq(model);
const nv = pino.modelNv(model);
const nlinks = pino.modelNlinks(model);
const ws = pino.newWorkspace(model);
```

### Dynamics

```js
const tau = pino.rnea(model, ws, q, qd, qdd, [0, 0, -9.81]);
const qddOut = pino.aba(model, ws, q, qd, tau, [0, 0, -9.81]);
const massMatrix = pino.crba(model, ws, q);
const gravity = pino.gravityTorques(model, ws, q);
const coriolis = pino.coriolisTorques(model, ws, q, qd);
```

Shapes:

- `q`: length `nq`
- `qd`, `qdd`, `tau`: length `nv`
- `rnea` returns length `nv`
- `aba` returns length `nv`
- `crba` returns `nv x nv`

### Kinematics and analysis

```js
const poses = pino.forwardKinematicsPoses(model, ws, q);
const J = pino.frameJacobian(model, ws, q, targetLinkIndex);
const com = pino.centerOfMass(model, ws, q);
const e = pino.energy(model, ws, q, qd);
const all = pino.computeAllTerms(model, ws, q, qd);
```

Return shapes:

- `forwardKinematicsPoses`: `{ translations, rotations }`
  - `translations[linkIndex]` is `[x, y, z]`
  - `rotations[linkIndex]` is a `3 x 3` matrix
- `frameJacobian`: `6 x nv`
- `centerOfMass`: `[x, y, z]`
- `energy`: `{ kinetic, potential }`
- `computeAllTerms`: `{ mass, bias, gravity, coriolis, com, kineticEnergy, potentialEnergy }`

### Batch operations

Batch arrays are flattened by step:

```js
const batchSize = 3;
const qBatch = [
  ...qStep0,
  ...qStep1,
  ...qStep2,
];
const qdBatch = [
  ...qdStep0,
  ...qdStep1,
  ...qdStep2,
];
const qddBatch = [
  ...qddStep0,
  ...qddStep1,
  ...qddStep2,
];

const tauBatch = pino.rneaBatch(model, ws, qBatch, qdBatch, qddBatch, batchSize);
const qddBatchOut = pino.abaBatch(model, ws, qBatch, qdBatch, tauBatch, batchSize);
const massBatch = pino.crbaBatch(model, ws, qBatch, batchSize);
```

Rollout:

```js
const rollout = pino.rolloutAbaEuler(
  model,
  ws,
  q0,
  qd0,
  tauBatch,
  batchSize,
  0.001
);

console.log(rollout.q);  // flattened batchSize * nq
console.log(rollout.qd); // flattened batchSize * nv
```

### Configuration-space operations

Use these for spherical/free-flyer models instead of plain elementwise math:

```js
const qNeutral = pino.neutralConfiguration(model);
const qNormalized = pino.normalizeConfiguration(model, q);
const ok = pino.isNormalized(model, qNormalized, 1e-6);
const dv = pino.differenceConfiguration(model, q0, q1);
const qMid = pino.interpolateConfiguration(model, q0, q1, 0.5);
const qRandom = pino.randomConfiguration(model, lower, upper, 42);
```

Shapes:

- `neutralConfiguration` returns length `nq`.
- `differenceConfiguration` returns length `nv`.
- `interpolateConfiguration` returns length `nq`.
- `randomConfiguration` returns length `nq`.

For spherical/free-flyer joints:

- quaternion layout is `[w, x, y, z]`
- spherical `q`: `[w, x, y, z]`
- free-flyer `q`: `[x, y, z, w, qx, qy, qz]`

## 8. Contact and Collision

### Contact dynamics

Contacts passed to JS wrappers use this shape:

```js
const contacts = [
  {
    linkIndex: 2,
    point: [0, 0, 0],
    normal: [0, 0, 1],
    accelBias: 0,
  },
];
```

Calls:

```js
const solve = pino.contactConstrainedDynamics(model, ws, q, qd, tau, contacts);
console.log(solve.qdd);
console.log(solve.lambda);

const impulse = pino.applyContactImpulse(model, ws, q, qdMinus, 0.0, contacts);
console.log(impulse.qdPlus);
console.log(impulse.impulse);

const Jn = pino.contactJacobianNormal(model, ws, q, contacts);
```

### Collision

Sphere-only helper:

```js
const collision = pino.createCollisionModel([
  { linkIndex: 1, center: [0.5, 0, 0], radius: 0.1 },
  { linkIndex: 2, center: [0.5, 0, 0], radius: 0.1 },
]);
```

Multi-geometry helper:

```js
const collision = pino.createCollisionModelGeometries(
  [
    { type: 0, linkIndex: 1, center: [0.5, 0, 0], params: [0.1, 0, 0] },
    { type: 1, linkIndex: 2, center: [0.5, 0, 0], params: [0.2, 0.1, 0.1] },
  ],
  { ignoreSameLink: true, ignoreParentChild: true }
);
```

Geometry type IDs:

| Type | Meaning | `params` |
| ---: | --- | --- |
| 0 | sphere | `[radius, 0, 0]` |
| 1 | box | `[halfX, halfY, halfZ]` |
| 2 | capsule | `[halfLength, radius, 0]` |
| 3 | cylinder | `[halfLength, radius, 0]` |
| 4 | mesh approximation | `[halfX, halfY, halfZ]` |

Queries:

```js
const min = pino.collisionMinDistance(model, collision, ws, q);
console.log(min.distance, min.pair);

const details = pino.collisionQueryDetails(model, collision, ws, q, 16);
console.log(details);

pino.disposeCollisionModel(collision);
```

## 9. Centroidal, Derivatives, Regressors, Constraints

```js
const h = pino.centroidalMomentum(model, ws, q, qd);
const Ag = pino.centroidalMap(model, ws, q);
const centroidal = pino.centroidalFullTerms(model, ws, q, qd, qdd);

const Y = pino.inverseDynamicsRegressor(model, q, qd, qdd);
const keReg = pino.kineticEnergyRegressor(model, q, qd);

const dRnea = pino.rneaDerivatives(model, ws, q, qd, qdd);
const dAba = pino.abaDerivatives(model, ws, q, qd, tau);
const dKin = pino.kinematicsDerivatives(model, ws, q, targetLinkIndex);
const d2 = pino.rneaSecondOrderDerivatives(model, ws, q, qd, qdd);

const lockedMask = Array(nv).fill(0);
lockedMask[0] = 1;
const qddLocked = pino.constrainedAbaLockedJoints(model, ws, q, qd, tau, lockedMask);
```

Some Rust FFI exports are not yet wrapped by the JS SDK. If you need one of those immediately, use `pino.exports.pino_*` plus the memory helpers, or add a high-level wrapper in `js/pinocchio_wasm.mjs`.

## 10. Common Failure Modes

### `invalid state length: nq=..., nv=...`

You passed a vector with the wrong dimension.

- `q.length` must equal `modelNq(model)`.
- `qd.length`, `qdd.length`, and `tau.length` must equal `modelNv(model)`.

This matters for:

- spherical: `nq = 4`, `nv = 3`
- free-flyer: `nq = 7`, `nv = 6`

### `pino_* failed: -1`

Usually null pointer or allocation/handle misuse. Check that the model/workspace/collision handle is still alive.

### `pino_* failed: -2`

Usually invalid input, bad dimensions, malformed data, or invalid bounds.

### `pino_* failed: -3`

Model construction failed. Check parent indices, root link, joint definitions, or import file contents.

### `pino_* failed: -4`

Algorithm failed, often due to dimensions, singular matrix, invalid contact setup, or invalid model state.

### `pino_* failed: -128`

The FFI panic guard caught a panic. Treat this as a bug or unsupported edge case.

## 11. Recommended Verification Before Release

Use this local release checklist:

```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features
cargo test --all-targets --all-features
cargo build --release --target wasm32-unknown-unknown
node --check js/pinocchio_wasm.mjs
node --check js/sdk/runtime.mjs
node examples/js/node_demo.mjs
```

For JS-facing changes, also add or update a small Node example that calls the wrapper you changed. The Rust tests prove the native/FFI layer; the Node demo proves the WASM build and JS memory packing path.

For high-frequency browser kinematics, also run:

```bash
node examples/js/urdf_high_frequency_fk.mjs
```

Then open the browser demos through `python3 -m http.server 8000` and test the sliders.

## 12. Python Usage

Python can use this project, but the current helper calls the native dynamic library, not the browser `.wasm` file.

Build the native shared library:

```bash
cargo build --release
```

Then use:

```python
from bindings.python.pinocchio_wasm import PinocchioWasm

pino = PinocchioWasm()
model = pino.model_from_json(json_string)
ws = pino.workspace_new(model)

qdd = pino.aba(model, ws, q, qd, tau, [0.0, 0.0, -9.81])

pino.free_workspace(ws)
pino.free_model(model)
```

This path is useful for native Python scripts, tests, or tooling. It is not the same deployment target as browser WASM.

Calling the `.wasm` from Python is also possible through runtimes such as Wasmtime, but this repository does not currently provide a dedicated Wasmtime Python wrapper. For browser kinematics, use the JavaScript SDK and the high-frequency buffer reuse pattern above.
