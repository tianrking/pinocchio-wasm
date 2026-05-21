# JavaScript API Reference

This file documents the public JS SDK returned by `loadPinocchioWasm(wasmBytes)`.

The implementation lives in `js/pinocchio_wasm.mjs`. It wraps the Rust/WASM C ABI and manages temporary WASM memory for ordinary calls.

## Loading

Node.js:

```js
import { readFile } from "node:fs/promises";
import { loadPinocchioWasm } from "./js/pinocchio_wasm.mjs";

const wasmBytes = await readFile("./target/wasm32-unknown-unknown/release/pinocchio_wasm.wasm");
const pino = await loadPinocchioWasm(wasmBytes);
```

Browser:

```js
import { loadPinocchioWasm } from "./js/pinocchio_wasm.mjs";

const wasmBytes = await fetch("/pinocchio_wasm.wasm").then((r) => r.arrayBuffer());
const pino = await loadPinocchioWasm(wasmBytes);
```

## Handles and Lifetime

The SDK returns numeric handles for Rust-owned objects:

| Handle | Created by | Freed by |
| --- | --- | --- |
| `model` | `createModel`, `createModelFromJson`, `createModelFromUrdf`, `createModelFromSdf`, `createModelFromMjcf` | `disposeModel` |
| `ws` | `newWorkspace(model)` | `disposeWorkspace` |
| `collision` | `createCollisionModel`, `createCollisionModelGeometries` | `disposeCollisionModel` |

Free every handle exactly once. Do not use a handle after freeing it.

## Dimension Rules

Every model has:

- `nq`: configuration dimension
- `nv`: velocity/tangent dimension
- `nlinks`: number of links

Use:

```js
const nq = pino.modelNq(model);
const nv = pino.modelNv(model);
const nlinks = pino.modelNlinks(model);
```

Vector lengths:

| Vector | Length |
| --- | ---: |
| `q` | `nq` |
| `qd` | `nv` |
| `qdd` | `nv` |
| `tau` | `nv` |
| gravity `g` | 3 |

For spherical/free-flyer joints, `nq !== nv`. Do not assume they are equal.

Quaternion convention:

- spherical joint `q`: `[w, x, y, z]`
- free-flyer joint `q`: `[x, y, z, w, qx, qy, qz]`

## Model APIs

### `createModel(links)`

Creates a model from structured JavaScript link data.

```js
const model = pino.createModel([
  { parent: -1, mass: 0.1, com: [0, 0, 0] },
  {
    parent: 0,
    mass: 1,
    com: [0.5, 0, 0],
    inertia: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    joint: { type: "revolute", axis: [0, 0, 1], origin: [0, 0, 0] },
  },
]);
```

Supported `joint.type` values:

| Type | Also accepted | `nq` | `nv` |
| --- | --- | ---: | ---: |
| `"revolute"` | omitted, `0` | 1 | 1 |
| `"prismatic"` | `1` | 1 | 1 |
| `"fixed"` | `2` | 0 | 0 |
| `"spherical"` | `3` | 4 | 3 |
| `"freeflyer"` | `"free_flyer"`, `"floating"`, `4` | 7 | 6 |

### Text import/export

```js
const model = pino.createModelFromJson(jsonString);
const model = pino.createModelFromUrdf(urdfString);
const model = pino.createModelFromSdf(sdfString);
const model = pino.createModelFromMjcf(mjcfString);

const json = pino.modelToJson(model);
const urdf = pino.modelToUrdf(model, "robot_name");
const sdf = pino.modelToSdf(model, "model_name");
const mjcf = pino.modelToMjcf(model, "model_name");
```

Current URDF support is intentionally small:

- reads links, inertial mass/com/inertia, parent/child joints, joint origin `xyz`, and joint axis `xyz`
- supports revolute, continuous, prismatic, fixed, ball/spherical, floating/freeflyer joints
- does not yet parse `origin rpy`, limits, mimic, transmission, visual mesh, or collision mesh

## Dynamics APIs

### `rnea(model, ws, q, qd, qdd, g = [0, 0, -9.81])`

Inverse dynamics. Returns `tau`, length `nv`.

```js
const tau = pino.rnea(model, ws, q, qd, qdd);
```

### `aba(model, ws, q, qd, tau, g = [0, 0, -9.81])`

Forward dynamics. Returns `qdd`, length `nv`.

```js
const qdd = pino.aba(model, ws, q, qd, tau);
```

### `crba(model, ws, q)`

Returns mass matrix as `nv x nv` nested arrays.

```js
const M = pino.crba(model, ws, q);
```

### `gravityTorques(model, ws, q, g = [0, 0, -9.81])`

Returns gravity torque vector, length `nv`.

### `coriolisTorques(model, ws, q, qd)`

Returns Coriolis/bias-without-gravity vector, length `nv`.

## Kinematics and Analysis APIs

### `forwardKinematicsPoses(model, ws, q)`

Returns link poses:

```js
const poses = pino.forwardKinematicsPoses(model, ws, q);
```

Shape:

```js
{
  translations: [
    [x, y, z],
    ...
  ],
  rotations: [
    [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]],
    ...
  ]
}
```

The translation is the link origin. If you need a tool point on the final link, transform it manually:

```js
function transformPoint(pose, local) {
  const t = pose.translation;
  const r = pose.rotation;
  return [
    t[0] + r[0][0] * local[0] + r[0][1] * local[1] + r[0][2] * local[2],
    t[1] + r[1][0] * local[0] + r[1][1] * local[1] + r[1][2] * local[2],
    t[2] + r[2][0] * local[0] + r[2][1] * local[1] + r[2][2] * local[2],
  ];
}
```

### `frameJacobian(model, ws, q, targetLink)`

Returns a `6 x nv` Jacobian for a link index. Rows are linear xyz followed by angular xyz.

### `centerOfMass(model, ws, q)`

Returns `[x, y, z]`.

### `energy(model, ws, q, qd, g = [0, 0, -9.81])`

Returns:

```js
{ kinetic, potential }
```

### `computeAllTerms(model, ws, q, qd, g = [0, 0, -9.81])`

Returns:

```js
{
  mass,
  bias,
  gravity,
  coriolis,
  com,
  kineticEnergy,
  potentialEnergy
}
```

## Batch APIs

Batch inputs are flattened by step:

```js
const qBatch = [...q0, ...q1, ...q2];
const qdBatch = [...qd0, ...qd1, ...qd2];
```

Available calls:

```js
const tauBatch = pino.rneaBatch(model, ws, qBatch, qdBatch, qddBatch, batchSize);
const qddBatch = pino.abaBatch(model, ws, qBatch, qdBatch, tauBatch, batchSize);
const massBatch = pino.crbaBatch(model, ws, qBatch, batchSize);
const rollout = pino.rolloutAbaEuler(model, ws, q0, qd0, tauBatch, batchSize, dt);
```

Return shapes:

| Function | Return |
| --- | --- |
| `rneaBatch` | flat `batchSize * nv` |
| `abaBatch` | flat `batchSize * nv` |
| `crbaBatch` | array of `nv x nv` matrices |
| `rolloutAbaEuler` | `{ q: flat batchSize * nq, qd: flat batchSize * nv }` |

## Configuration-Space APIs

Use these for spherical/free-flyer models:

```js
const q0 = pino.neutralConfiguration(model);
const qn = pino.normalizeConfiguration(model, q);
const ok = pino.isNormalized(model, qn, 1e-6);
const dv = pino.differenceConfiguration(model, q0, q1);
const qMid = pino.interpolateConfiguration(model, q0, q1, 0.5);
const qRand = pino.randomConfiguration(model, lower, upper, 42);
```

Return lengths:

| Function | Return length |
| --- | ---: |
| `neutralConfiguration` | `nq` |
| `normalizeConfiguration` | `nq` |
| `isNormalized` | boolean |
| `differenceConfiguration` | `nv` |
| `interpolateConfiguration` | `nq` |
| `randomConfiguration` | `nq` |

## Contact APIs

Contact object:

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
const impulse = pino.applyContactImpulse(model, ws, q, qdMinus, restitution, contacts);
const Jn = pino.contactJacobianNormal(model, ws, q, contacts);
```

Return shapes:

| Function | Return |
| --- | --- |
| `contactConstrainedDynamics` | `{ qdd, lambda }` |
| `applyContactImpulse` | `{ qdPlus, impulse }` |
| `contactJacobianNormal` | `numContacts x nv` |

## Collision APIs

Sphere helper:

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

Geometry types:

| Type | Geometry | Params |
| ---: | --- | --- |
| 0 | sphere | `[radius, 0, 0]` |
| 1 | box | `[halfX, halfY, halfZ]` |
| 2 | capsule | `[halfLength, radius, 0]` |
| 3 | cylinder | `[halfLength, radius, 0]` |
| 4 | mesh approximation | `[halfX, halfY, halfZ]` |

Queries:

```js
const min = pino.collisionMinDistance(model, collision, ws, q);
const details = pino.collisionQueryDetails(model, collision, ws, q, maxResults);
```

Free:

```js
pino.disposeCollisionModel(collision);
```

## Centroidal, Derivatives, Regressors, Constraints

```js
const h = pino.centroidalMomentum(model, ws, q, qd);
const Ag = pino.centroidalMap(model, ws, q);
const full = pino.centroidalFullTerms(model, ws, q, qd, qdd);

const Y = pino.inverseDynamicsRegressor(model, q, qd, qdd);
const keReg = pino.kineticEnergyRegressor(model, q, qd);

const dRnea = pino.rneaDerivatives(model, ws, q, qd, qdd);
const dAba = pino.abaDerivatives(model, ws, q, qd, tau);
const dKin = pino.kinematicsDerivatives(model, ws, q, targetLink);
const d2 = pino.rneaSecondOrderDerivatives(model, ws, q, qd, qdd);

const lockedMask = Array(nv).fill(0);
lockedMask[0] = 1;
const qddLocked = pino.constrainedAbaLockedJoints(model, ws, q, qd, tau, lockedMask);
```

## Direct FFI and Memory Helpers

The SDK exposes low-level helpers for high-performance paths:

```js
const ptr = pino.allocBytes(byteLength);
pino.freeBytes(ptr, byteLength);

const f64 = pino.memoryF64();
const i32 = pino.memoryI32();
const u8 = pino.memoryU8();

const code = pino.exports.pino_forward_kinematics_poses(
  model,
  ws,
  qPtr,
  translationsPtr,
  rotationsPtr
);
```

Use direct FFI when:

- you call the same operation many times per frame
- you can keep input/output buffers alive
- you only need part of the output
- you want to run inside a Web Worker

Use high-level wrappers when:

- call frequency is low
- readability matters more than allocations
- you want ordinary JS arrays as input/output

## Error Codes

High-level wrappers throw `Error("pino_* failed: CODE")`.

| Code | Meaning |
| ---: | --- |
| `0` | ok |
| `-1` | null pointer |
| `-2` | invalid input |
| `-3` | model construction failed |
| `-4` | algorithm failed |
| `-128` | panic caught by FFI guard |

## Currently Wrapped vs Raw FFI

The JS SDK wraps the most commonly used FFI functions. Some Rust FFI exports are not wrapped yet, including several friction/batch/contact/regressor helpers. They are still reachable through `pino.exports.pino_*` plus memory helpers.

For production browser kinematics, the most important exported raw function is:

```js
pino.exports.pino_forward_kinematics_poses(model, ws, qPtr, translationsPtr, rotationsPtr)
```

It is the recommended fast path for repeated FK.
