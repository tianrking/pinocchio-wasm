# pinocchio_wasm

A modern WASM-oriented rigid-body dynamics engine inspired by Pinocchio.

## Implemented Features

- Modern layered architecture:
  - `core`: error model + linear algebra primitives (`Vec3`, `Mat3`, `Transform`)
  - `model`: tree model, reusable `Workspace`, JSON/URDF/SDF/MJCF loaders (`Model::from_json_str`, `Model::from_urdf_str`, `Model::from_sdf_str`, `Model::from_mjcf_str`)
  - `algo`: FK, Jacobian, RNEA, CRBA, ABA, CoM, kinetic/potential energy
  - `ffi`: stable C ABI for WASM/JS embedding
- Efficient data path:
  - contiguous numeric arrays across FFI
  - reusable workspace to avoid repeated allocations
  - release profile tuned for WASM (`lto`, `codegen-units = 1`, `opt-level = 3`)
- End-to-end tests:
  - dynamics closure: `tau = M(q) * qdd + b(q, qd)`
  - ABA inversion of inverse dynamics
  - Jacobian/CoM/Energy API smoke tests

## Build

```bash
cargo test
cargo build --release
```

WASM target (if installed):

```bash
rustup target add wasm32-unknown-unknown
cargo build --release --target wasm32-unknown-unknown
```

Output:

- `target/wasm32-unknown-unknown/release/pinocchio_wasm.wasm`

## C ABI Overview

- Model lifecycle:
  - `pino_model_create`
  - `pino_model_free`
- Workspace lifecycle:
  - `pino_workspace_new`
  - `pino_workspace_free`
- Algorithms:
  - `pino_rnea`
  - `pino_rnea_batch`
  - `pino_aba`
  - `pino_aba_batch`
  - `pino_rollout_aba_euler`
  - `pino_crba`
  - `pino_frame_jacobian`
  - `pino_center_of_mass`
  - `pino_energy`
  - model import: `pino_model_create_from_json`, `pino_model_create_from_urdf`, `pino_model_create_from_sdf`, `pino_model_create_from_mjcf`

All algorithm APIs are pointer + length driven and return status code (`0` means success).

## Roadmap

- URDF/SDF/MJCF parser bridge (WASM-friendly schema precompile)
- Collision layer and distance queries
- Batch algorithm APIs for trajectory rollouts
- SIMD/SoA acceleration pass for large batched workloads
