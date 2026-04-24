# pinocchio-wasm Development Charter

> 本文档定义开发范式、架构约定和执行规则。
> Agent 可依据本文档自主执行所有 Batch，无需人类逐条确认。
> 版本: 2.0 | 日期: 2026-04-24

---

## 0. 项目身份

| 项目 | pinocchio-wasm |
|------|---------------|
| 语言 | Rust (edition 2024) |
| 目标 | 浏览器端刚体动力学引擎 (WASM) |
| 构建 | `cargo build --release --target wasm32-unknown-unknown` |
| 仓库 | `C:\Users\tianr\Downloads\AMOTOR\pinocchio-wasm` |
| 作者 | tianrking <tian.r.king@gmail.com> |
| 许可 | BSD-2-Clause |

---

## 1. 架构分层

```
┌─────────────────────────────────────────┐
│  JS SDK    (js/pinocchio_wasm.mjs)      │  ← 浏览器/Node.js 用户接口
│            (js/sdk/runtime.mjs)         │  ← WASM 内存运行时 helpers
├─────────────────────────────────────────┤
│  FFI 层    (src/ffi/*.rs)               │  ← extern "C" C ABI 导出 (72 个)
├─────────────────────────────────────────┤
│  算法层     (src/algo/*.rs)              │  ← 纯 Rust，无 unsafe，无 FFI 意识
├─────────────────────────────────────────┤
│  模型层     (src/model/*.rs)             │  ← Model, Workspace, Link, Joint, 解析器
├─────────────────────────────────────────┤
│  核心层     (src/core/*.rs)              │  ← Vec3, Mat3, Transform, Spatial, Error
└─────────────────────────────────────────┘
```

**核心原则: 单向依赖，算法层不知道 FFI 的存在。**

```
ffi → algo → model → core
ffi → model → core
ffi → collision → model → core
```

---

## 2. 编码约定

### 2.1 Rust 风格

| 规则 | 说明 |
|------|------|
| 无 unsafe | algo/model/collision 层禁止 unsafe，只有 ffi 层允许 |
| 无 unwrap | 用 `?` 和 `Result` 传播错误 |
| 无注释 | 代码即文档，只在 WHY 不明显时加一行注释 |
| `pub(crate)` 优先 | 内部函数不公开，只有需要跨模块的才 `pub` |
| 命名 | 函数 snake_case, 类型 PascalCase, 模块 snake_case |

### 2.2 算法层签名模式

```rust
pub fn xxx(model: &Model, q: &[f64], ..., ws: &mut Workspace) -> Result<OutputType>
```

### 2.3 FFI 层签名模式

```rust
#[unsafe(no_mangle)]
pub extern "C" fn pino_xxx(model: *const ModelHandle, ws: *mut WorkspaceHandle, ...) -> i32
```

### 2.4 垂直切片要求

每个 Slice 必须包含完整链路:
```
1. src/algo/xxx.rs        ← 算法实现
2. src/ffi/xxx.rs         ← FFI 导出
3. js/pinocchio_wasm.mjs  ← JS 包装
4. include/pinocchio_wasm.h ← C 头文件声明
5. tests/                 ← 至少一个测试文件
```

### 2.5 质量门禁

| # | 检查项 | 命令 |
|---|--------|------|
| 1 | 编译通过 (native) | `cargo build` |
| 2 | 编译通过 (WASM) | `cargo build --release --target wasm32-unknown-unknown` |
| 3 | 全部测试通过 | `cargo test --all-targets --all-features` |
| 4 | 无 clippy 警告 | `cargo clippy --all-targets --all-features` |
| 5 | 格式正确 | `cargo fmt --check` |
| 6 | JS 语法正确 | `node --check js/pinocchio_wasm.mjs` |

### 2.6 提交规范

```
<type>(<scope>): <描述> [Batch N]

type: feat | fix | refactor | docs | chore | test
scope: algo | ffi | js | model | collision | core | arch
禁止包含任何 Co-Authored-By 或 Signed-off-by 行。
```

---

## 3. 已完成进度

| Batch | 内容 | 状态 | Commit |
|-------|------|------|--------|
| **Batch 0** | 架构拆分 (mono→per-feature modules) | ✅ 完成 | `2e90b91` |
| **Batch 1** | Fixed + Prismatic 关节 | ✅ 完成 | `8a9b568` |
| **Batch 2** | JS SDK (40+ 函数) | ✅ 完成 | `00c33c5` |
| **Batch 3** | O(n) 递归 ABA | ✅ 完成 | `226d38f` |
| **Batch 4** | JS SDK 扩展 (batch/contact/collision/centroidal/regressors) | ✅ 完成 | (合并入 Batch 2) |
| **Batch 5** | 解析导数 (混合实现) | ✅ 完成 | `7b0a827` |
| **Batch 6** | FreeFlyer + Spherical 关节 | ✅ 完成 | `6a23fc4` |
| **Batch 7** | ABA 多自由度通用化 + JS runtime 抽取 + C Header 同步 | ✅ 完成 | `a790559` |

**当前测试基线**: 97 passed, 0 failed | `cargo build` ✅ | `node --check` ✅

---

## 4. 目录结构 (当前)

```
src/
├── lib.rs
├── core/
│   ├── mod.rs               (Vec3, Mat3, Transform)
│   ├── error.rs             (PinocchioError, Result)
│   └── math.rs              (264 行)
├── model/
│   ├── mod.rs               (Model, Workspace, Link, Joint, JointType)
│   ├── json.rs              (from_json_str, to_json_string)
│   ├── urdf.rs              (from_urdf_str, to_urdf_string)
│   ├── sdf.rs               (from_sdf_str, to_sdf_string)
│   └── mjcf.rs              (from_mjcf_str, to_mjcf_string)
├── algo/
│   ├── mod.rs               (pub use re-export)
│   ├── kinematics.rs        (FK, FK_poses, FK_poses_batch)
│   ├── dynamics.rs          (rnea, aba, aba_crba, crba, bias/gravity/coriolis)
│   ├── jacobian.rs          (frame_jacobian, contact_jacobian_normal)
│   ├── com_energy.rs        (center_of_mass, kinetic/potential_energy, compute_all_terms)
│   ├── contact.rs           (contact FD + impulse, friction 变体, batch)
│   ├── constrained.rs       (locked joints ABA + batch)
│   ├── centroidal.rs        (Ag, dAg, hdot, centroidal_derivatives)
│   ├── regressors.rs        (ID/KE/PE/CoM regressor, column selection, batch)
│   ├── derivatives.rs       (rnea/aba/kinematics/jacobian/com derivatives)
│   ├── batch.rs             (rnea/aba/crba/bias/gravity batch)
│   ├── rollout.rs           (rollout_aba_euler, integrate/diff/interp/random config)
│   └── solvers.rs           (cholesky, delassus, PGS, ADMM)
├── collision/
│   └── mod.rs               (CollisionModel, AABB, sphere-based)
├── ffi/
│   ├── mod.rs               (Status, Handle, 内存管理, helpers)
│   ├── model.rs             (model create/free, load/export, nv/nq)
│   ├── dynamics.rs          (rnea, aba, crba, bias/gravity/coriolis)
│   ├── kinematics.rs        (FK_poses, compute_all_terms)
│   ├── contact.rs           (contact FD, impulse, friction, batch)
│   ├── collision.rs         (collision model, distance, query)
│   ├── centroidal.rs        (centroidal momentum/map/full)
│   ├── regressors.rs        (ID/KE regressor)
│   ├── derivatives.rs       (all derivatives)
│   └── batch.rs             (batch dynamics)
├── autodiff/
│   └── mod.rs
├── codegen/
│   └── mod.rs
└── visualization/
    └── mod.rs
```

---

## 5. 已知问题 (按优先级)

### P0 — 功能性 BUG

| # | Bug | 文件 | 说明 |
|---|-----|------|------|
| B1 | JS 用 `nq` 校验 `qd`/`tau` 维度，应为 `nv` | `js/pinocchio_wasm.mjs` | Spherical/FreeFlyer 模型会误拒 |
| B2 | `integrate_configuration()` 纯加法，nq≠nv 崩溃 | `algo/rollout.rs` | 四元数关节出错 |
| B3 | `difference_configuration()` 纯减法 | `algo/rollout.rs` | 四元数差分数学错误 |
| B4 | `interpolate_configuration()` 线性插值 | `algo/rollout.rs` | 四元数非单位 |
| B5 | `random_configuration()` 不归一化四元数 | `algo/rollout.rs` | 产生无效位形 |
| B6 | 缺少 `neutral_configuration()` | 无 | 四元数初值应为 [1,0,0,0] |
| B7 | Spherical Jacobian 返回零值 | `algo/jacobian.rs` | 1 个测试失败 |

### P1 — 代码质量

| # | 问题 | 说明 |
|---|------|------|
| Q1 | 四元数运算散落在 rollout.rs 私有函数 | 无 Quaternion 类型，不可复用 |
| Q2 | 缺少 `from_rotation_matrix` (R→q) | 无法闭环四元数/旋转转换 |
| Q3 | 缺少 SO(3) `log_map` | 无法从旋转矩阵提取轴角 |
| Q4 | JS SDK 1,423 行单文件，43% 样板代码 | 维护困难 |
| Q5 | Clippy 39 个警告 | too_many_arguments, useless_vec, needless_range |

### P2 — 性能

| # | 项目 | 当前 | 理想 |
|---|------|------|------|
| P1 | CRBA O(n²) | 逐列 RNEA | O(n) 递归 |
| P2 | 多数导数用 FD | O(n²) 有限差分 | O(n) 解析递推 |

---

## 6. 剩余开发计划

### Batch 8: Bug 修复 + Quaternion 基础 [优先级: P0]

**目标:** 修复所有已知 BUG，建立四元数数学基础设施。

**前置条件:** 无。

**Slice 8.1 — Quaternion 模块 (core/quaternion.rs)**

创建 `src/core/quaternion.rs`，从 `rollout.rs` 提取并扩展:

```rust
pub(crate) struct Quat { pub w: f64, pub x: f64, pub y: f64, pub z: f64 }

impl Quat {
    pub fn identity() -> Self                           // [1, 0, 0, 0]
    pub fn normalize(self) -> Self
    pub fn mul(self, other: Self) -> Self               // Hamilton product
    pub fn conjugate(self) -> Self
    pub fn from_rotation_matrix(m: &Mat3) -> Self       // R → q (Shepperd)
    pub fn to_rotation_matrix(self) -> Mat3             // q → R (已有 Mat3::from_quaternion)
    pub fn delta(omega: Vec3, dt: f64) -> Self           // exp_map: ω*dt → Δq
    pub fn slerp(self, other: Self, t: f64) -> Self     // 球面线性插值
    pub fn log(self) -> Vec3                            // q → ω (轴角)
    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Self
}
```

- `rollout.rs` 中的 `normalize_quat`, `quat_mul`, `delta_quat` 改为调用 `Quat` 方法
- `kinematics.rs` 中 FK 的四元数→旋转矩阵改用 `Quat::to_rotation_matrix`
- 测试: 四元数往返 (axis_angle→quat→R→quat→axis_angle)、归一化不变量、slerp 端点

**Slice 8.2 — 修复位形空间操作 (B2~B6)**

在 `algo/rollout.rs` 中修复:

- `neutral_configuration(model) -> Vec<f64>` — 返回全关节零位形 (四元数=[1,0,0,0])
- `integrate_configuration(q, v, dt)` → 改为 `integrate_model_configuration` 的无 model 版本? 或者标记为 deprecated 并在文档注明仅适用于 nq==nv 模型。**推荐**: 让这些函数接收 `&Model` 参数，按关节类型分派。如果不想改签名，则在内部检查 nq==nv 并返回错误。
- `difference_configuration(model, q1, q0)` — Spherical: log(q0⁻¹·q1)→3-vector; FreeFlyer: 位置差 + 四元数 log
- `interpolate_configuration(model, q0, q1, alpha)` — Spherical: slerp; FreeFlyer: 线性位置 + slerp 旋转
- `random_configuration(model, lower, upper, rng)` — Spherical/FreeFlyer: 均匀采样 SO(3)
- 更新 FFI 签名: 新增 `pino_neutral_configuration`
- 更新 JS SDK: 新增 `neutralConfiguration()`
- 测试: 与 FD 交叉验证; Spherical/FreeFlyer 模型 roundtrip (neutral→integrate→difference≈velocity)

**Slice 8.3 — 修复 JS 维度校验 (B1)**

在 `js/pinocchio_wasm.mjs` 中修复 3 处 `nq` → `nv`:

- `rneaDerivatives` (line ~1277): `qd.length !== nq` → `qd.length !== nv`
- `abaDerivatives` (line ~1312): `qd.length !== nq` → `qd.length !== nv`, `tau.length !== nq` → `tau.length !== nv`
- `constrainedAbaLockedJoints` (line ~1351): 同上

**Slice 8.4 — 修复 Spherical Jacobian (B7)**

在 `algo/jacobian.rs` 中修复 Spherical 关节列填充:

- Spherical 的 `world_motion_angular` 应填入关节的 3 个运动子空间列 (R·ωx, R·ωy, R·ωz 或等效)
- 检查 FK 中 Spherical 的 `world_motion_angular`/`world_motion_linear` 是否正确填充
- 测试: `tests/floating_joints.rs` 的 `spherical_joint_dimensions_and_dynamics` 应通过

**质量门禁:**
- `cargo test` 全绿 (含修复的 Spherical 测试)
- `cargo clippy` 无新增警告
- `node --check js/pinocchio_wasm.mjs` 通过
- 新增测试 ≥ 5 个

**提交:** `fix(core): add Quaternion type, fix configuration-space ops for floating joints [Batch 8]`

---

### Batch 9: JS SDK 内部重构 [优先级: P1]

**目标:** JS SDK 内部消除样板代码，API 零变更。

**前置条件:** Batch 8 完成。

**Slice 9.1 — 提取 WASM 调用 helper**

在 `js/sdk/runtime.mjs` 中新增:

```javascript
// 通用 vector-in/vector-out WASM 调用
function callWasm(w, fnName, { model, ws, inputs, outputs, extras }) { ... }

// flat → 2D 矩阵
function reshape2D(flat, rows, cols) { ... }

// contact 数据打包
function packContacts(contacts) { ... }

// model 维度缓存 (避免 30 次 pino_model_nq 调用)
function modelDims(w, model) { return { nq, nv, nl }; }
```

**Slice 9.2 — 重构 25+ 个标准函数**

将以下函数从手写 alloc→write→call→read→free 改为调用 `callWasm`:

- 动力学: `rnea`, `aba`, `crba`, `gravityTorques`, `coriolisTorques`
- 运动学: `frameJacobian`, `centerOfMass`, `energy`, `forwardKinematicsPoses`
- 批量: `rneaBatch`, `abaBatch`, `crbaBatch`
- 导数: `rneaDerivatives`, `abaDerivatives`, `kinematicsDerivatives`, `rneaSecondOrderDerivatives`
- 质心: `centroidalMomentum`, `centroidalMap`, `centroidalFullTerms`
- 回归: `inverseDynamicsRegressor`, `kineticEnergyRegressor`
- 约束: `constrainedAbaLockedJoints`

**Slice 9.3 — 消除 contact 重复**

3 个 contact 函数中的数据打包循环提取为 `packContacts`:
- `contactConstrainedDynamics`
- `applyContactImpulse`
- `contactJacobianNormal`

**Slice 9.4 — 消除 model loader 重复**

4 个 `createModelFrom*` 和 3 个 `modelTo*` 提取为工厂函数。

**质量门禁:**
- 对外 API 零变更 (所有 44 个 export 签名不变)
- `node --check js/pinocchio_wasm.mjs` 通过
- `node examples/js/node_demo.mjs` 运行无报错
- 行数减少 ≥ 25% (目标: 1,423 → ~1,000)

**提交:** `refactor(js): extract WASM call helpers, reduce SDK boilerplate [Batch 9]`

---

### Batch 10: Clippy 清零 + Workspace 优化 [优先级: P1]

**目标:** Clippy 零警告，Workspace 预分配。

**前置条件:** Batch 8 完成。

**Slice 10.1 — 修复 Clippy 警告**

修复 39 个警告:

| 类型 | 数量 | 修复方式 |
|------|------|---------|
| `useless vec!` | 12 | `vec![0.0; n]` → `[0.0; N]` (已知大小时) 或保留 vec! |
| `too_many_arguments` | 8 | 引入参数结构体: `BatchInput`, `ContactProblem`, `ContactFrictionProblem` |
| `needless_range_loop` | 7 | `for i in 0..n` → `.iter().enumerate()` 或直接迭代 |
| `type_complexity` | 1 | 引入 type alias |

注意: `too_many_arguments` 的 FFI 函数签名不能改 (C ABI 稳定)，但 algo 层可以引入结构体，FFI 层在调用时解包。

**Slice 10.2 — Workspace 预分配**

在 `Workspace::new()` 中预分配 ABA scratch buffers:

```rust
pub struct Workspace {
    // ... 现有字段 ...
    pub aba_d: Vec<SpatialMatrix>,   // 已有
    pub aba_u: Vec<SpatialVector>,   // 已有
    pub aba_u_cols: Vec<SpatialMatrix>, // 已有
    // 新增:
    pub aba_articulated_inertia: Vec<SpatialMatrix>,  // 移入 workspace
    pub aba_bias_force: Vec<SpatialVector>,            // 移入 workspace
    pub aba_d_scalar: Vec<f64>,                        // per-joint scalar D
    pub aba_u_scalar: Vec<f64>,                        // per-joint scalar u
}
```

- `aba()` 不再每帧 `vec![...]`，直接写 Workspace 字段
- 测试: ABA 输出不变 (回归测试)

**质量门禁:**
- `cargo clippy --all-targets --all-features` 零警告
- `cargo test` 全绿

**提交:** `chore: zero clippy warnings, pre-allocate workspace buffers [Batch 10]`

---

### Batch 11: C Header + 文档同步 [优先级: P1]

**目标:** C Header 与 FFI 完全同步，PLAN 文档反映真实状态。

**前置条件:** Batch 8 完成。

**Slice 11.1 — C Header 补齐 19 个缺失声明**

在 `include/pinocchio_wasm.h` 中添加:

```
pino_aba_batch, pino_aba_derivatives, pino_rnea_batch, pino_rnea_derivatives,
pino_bias_forces_batch, pino_gravity_torques_batch, pino_crba, pino_crba_batch,
pino_coriolis_torques, pino_gravity_torques, pino_center_of_mass, pino_energy,
pino_forward_kinematics_poses, pino_forward_kinematics_poses_batch,
pino_frame_jacobian, pino_kinematics_derivatives, pino_model_nv,
pino_constrained_aba_locked_joints, pino_constrained_aba_locked_joints_batch,
pino_rollout_aba_euler, pino_neutral_configuration (Batch 8 新增)
```

验证: 将所有 `#[unsafe(no_mangle)] pub extern "C" fn pino_*` 与 header 逐一比对。

**Slice 11.2 — 更新 AUDIT.md**

基于当前代码重写 `PLAN/AUDIT.md`:
- 修正 "ABA 是 O(n³)" → "ABA 是 O(n)"
- 修正 "仅支持 revolute" → "支持 5 种关节类型"
- 修正 "JS SDK 6 个函数" → "JS SDK 44 个函数"
- 修正 "所有导数有限差分" → "混合实现 (FD + CRBA + 解析)"
- 更新关节类型支持矩阵
- 更新测试计数 (97)

**Slice 11.3 — 更新 GAP_ANALYSIS**

更新 `PLAN/PINOCCHIO_API_GAP_ANALYSIS.md` 中已实现的功能标记:
- FK/RNEA/ABA/CRBA/Jacobian 等对 Spherical/FreeFlyer 的支持状态
- 导数实现方式 (混合/解析/FD)
- 新增 Batch 6/7 已实现的项目

**Slice 11.4 — 项目文件补全**

创建:
- `LICENSE` (BSD-2-Clause 全文)
- `CHANGELOG.md` (从 git log 生成)
- Cargo.toml 添加 `repository` 字段

**质量门禁:**
- C Header 与 FFI 100% 同步
- AUDIT.md 与代码一致

**提交:** `docs: sync C header, AUDIT, GAP analysis with codebase [Batch 11]`

---

### Batch 12: Frame 系统 [优先级: P2]

**目标:** 实现 Pinocchio Frame 抽象，支持任意参考点的运动学/动力学查询。

**前置条件:** Batch 8 完成 (Quaternion 模块)。

**Slice 12.1 — Frame 数据结构**

```rust
pub struct Frame {
    pub name: String,
    pub parent_link: usize,
    pub placement: Transform,     // link → frame 变换
    pub frame_type: FrameType,    // { Fixed, Movable }
}
```

Model 新增:
- `frames: Vec<Frame>`
- `add_frame(name, parent_link, placement) -> usize`
- `frame_id(name) -> Option<usize>`

**Slice 12.2 — Frame 运动学**

```rust
pub fn frame_placement(model, q, frame_id, ws) -> Result<Transform>
pub fn frame_velocity(model, q, qd, frame_id, ws) -> Result<(Vec3, Vec3)>  // (ω, v)
pub fn frame_classical_acceleration(model, q, qd, qdd, frame_id, ws) -> Result<(Vec3, Vec3)>
pub fn update_frame_placements(model, q, ws) -> Result<()>  // 所有 frame
```

**Slice 12.3 — Frame Jacobian**

```rust
pub fn frame_jacobian(model, q, frame_id, ws) -> Result<Vec<Vec<f64>>>  // 6 × nv
pub fn frame_jacobian_time_derivative(model, q, qd, frame_id, ws) -> Result<Vec<Vec<f64>>>
```

**Slice 12.4 — FFI + JS + 测试**

- FFI: `pino_add_frame`, `pino_frame_placement`, `pino_frame_velocity`, `pino_frame_jacobian`
- JS: `addFrame`, `framePlacement`, `frameVelocity`, `frameJacobian`
- 测试: FK frame 与直接 link 结果一致; Jacobian 与 FD 交叉验证

**质量门禁:** 标准六项全过。

**提交:** `feat(algo): add Frame system with placement/velocity/jacobian [Batch 12]`

---

### Batch 13: Lie 群导数 [优先级: P2]

**目标:** 实现位形空间李群操作的解析导数，为轨迹优化提供基础。

**前置条件:** Batch 8 (Quaternion) + Batch 12 (Frame)。

**Slice 13.1 — dIntegrate / dDifference**

```rust
pub fn d_integrate(model, q, v, dt, which_arg) -> Vec<Vec<f64>>  // nv × nv
pub fn d_difference(model, q1, q0, which_arg) -> Vec<Vec<f64>>   // nv × nv
```

- Revolute: 单位矩阵 (trivial)
- Spherical: 四元数导数 (旋转矩阵的 skew 对称部分)
- FreeFlyer: 6×6 块对角 (位置 + 四元数)

**Slice 13.2 — Jintegrate / Jdifference**

```rust
pub fn j_integrate(model, q, v, dt) -> (Vec<Vec<f64>>, Vec<Vec<f64>>)  // (dq, dv)
pub fn j_difference(model, q1, q0) -> (Vec<Vec<f64>>, Vec<Vec<f64>>)  // (dq1, dq0)
```

**Slice 13.3 — FFI + JS + 测试**

- FFI: `pino_d_integrate`, `pino_d_difference`
- JS: `dIntegrate`, `dDifference`
- 测试: 与 FD 交叉验证所有关节类型

**提交:** `feat(algo): add Lie group dIntegrate/dDifference derivatives [Batch 13]`

---

### Batch 14: 高级动力学 [优先级: P2]

**目标:** Coriolis 矩阵、M⁻¹、CoM Jacobian。

**前置条件:** 无额外依赖。

**Slice 14.1 — Coriolis 矩阵**

```rust
pub fn coriolis_matrix(model, q, qd, ws) -> Result<Vec<Vec<f64>>>  // nv × nv
```

利用已有 `rnea_derivatives`: C(q,qd) = dtau/dqd - 2*dtau/dq (Christoffel 对称)

**Slice 14.2 — 质量矩阵逆**

```rust
pub fn compute_minverse(model, q, ws) -> Result<Vec<Vec<f64>>>  // nv × nv
```

利用 Cholesky 分解逐列求逆。

**Slice 14.3 — CoM Jacobian**

```rust
pub fn jacobian_center_of_mass(model, q, ws) -> Result<Vec<Vec<f64>>>  // 3 × nv
```

逐关节累加: Jcom = Σ(m_i * J_i) / m_total

**Slice 14.4 — FFI + JS + 测试**

- FFI: `pino_coriolis_matrix`, `pino_compute_minverse`, `pino_jacobian_center_of_mass`
- JS: `coriolisMatrix`, `computeMinverse`, `jacobianCenterOfMass`
- 测试: C*qd ≈ coriolis_torques; M*M⁻¹ ≈ I; Jcom*qd ≈ dCoM/dt

**提交:** `feat(algo): add Coriolis matrix, M^-1, and CoM Jacobian [Batch 14]`

---

### Batch 15: 解析导数升级 [优先级: P3]

**目标:** 将剩余 FD 导数替换为 O(n) 解析递推。

**前置条件:** Batch 10 (Workspace 预分配) + Batch 13 (Lie 群导数)。

**Slice 15.1 — RNEA 完整解析导数**

实现 O(n) `computeRNEADerivatives`:
- dtau/dq: 通过 differentiated backward pass，逐关节追踪 FK 敏感度传播
- dtau/dv: 同上，对 qd 的导数
- dtau/dqdd: = M(q) via CRBA (已有)
- 测试: 与 FD 交叉验证所有 5 种关节类型

**Slice 15.2 — ABA 完整解析导数**

基于 RNEA 解析导数 + M⁻¹:
- dqdd/dq = -M⁻¹ * (dtau/dq - dM/dq * qdd)  (已有框架，升级 dtau/dq)
- dqdd/dv = -M⁻¹ * dtau/dv
- dqdd/dtau = M⁻¹

**Slice 15.3 — Centroidal 解析导数**

`centroidal_derivatives`, `centroidal_map_derivatives` 改为解析实现。

**质量门禁:** 所有解析导数与 FD 差异 < 1e-6。

**提交:** `feat(algo): upgrade all derivatives to O(n) analytical [Batch 15]`

---

### Batch 16: 更多关节类型 [优先级: P3]

**目标:** 支持 Helical、Universal、Planar 等关节。

**前置条件:** Batch 8 (Quaternion)。

**Slice 16.1 — Helical (螺旋关节)**

nq=1, nv=1, 同时旋转 + 平移 (螺距 pitch)。

**Slice 16.2 — Universal (万向节)**

nq=2, nv=2, 两个正交旋转轴。

**Slice 16.3 — Planar (平面关节)**

nq=3, nv=3, 平面内 2D 平移 + 1D 旋转。

**Slice 16.4 — Translation (3D 平移关节)**

nq=3, nv=3, 纯 3D 平移。

**每个 Slice 包含:** FK + RNEA + ABA + Jacobian + FFI + JS + 测试。

**提交:** `feat(joints): add Helical/Universal/Planar/Translation joint types [Batch 16]`

---

### Batch 17: 通用约束框架 [优先级: P3]

**目标:** 替代当前硬编码 contact，支持任意约束。

**前置条件:** Batch 12 (Frame) + Batch 14 (M⁻¹)。

**Slice 17.1 — 约束数据结构**

```rust
pub struct RigidConstraint {
    pub frame1: usize,     // frame id on body 1
    pub frame2: usize,     // frame id on body 2
    pub reference_frame: ReferenceFrame,  // { LOCAL, WORLD }
    pub contact_model: ContactModel,      // { Rigid, Compliance }
}
```

**Slice 17.2 — constraintDynamics**

```rust
pub fn constraint_dynamics(model, q, qd, tau, constraints, ws) -> Result<Vec<f64>>
```

基于 Delassus 矩阵 + KKT 系统。

**Slice 17.3 — FFI + JS + 测试**

**提交:** `feat(algo): add general constraint dynamics framework [Batch 17]`

---

### Batch 18: 碰撞升级 [优先级: P3]

**目标:** 超越球体近似，支持精确距离计算。

**前置条件:** 无额外依赖。

**Slice 18.1 — 基础几何对**

Box-Box, Sphere-Capsule, Capsule-Capsule 精确距离计算。

**Slice 18.2 — 碰撞结果增强**

接触法向量、最近点、穿透深度。

**Slice 18.3 — FFI + JS + 测试**

**提交:** `feat(collision): add exact geometry pair distance computation [Batch 18]`

---

### Batch 19: 发布基础设施 [优先级: P3]

**目标:** npm 发布准备，CI 自动化。

**前置条件:** Batch 9 (JS 重构) + Batch 11 (文档)。

**Slice 19.1 — npm 包**

- `package.json`: name, version, main, types, files
- TypeScript `.d.ts` 类型定义 (从 JS 函数签名生成)
- `wasm-pack.toml` 配置

**Slice 19.2 — CI 增强**

- 多 OS 矩阵 (ubuntu + macos + windows)
- 自动 wasm-pack build
- 自动 npm publish (on tag)
- `rust-toolchain.toml` 固定版本

**Slice 19.3 — Python 绑定补全**

当前 Python ctypes 绑定仅 5/72 函数。补全核心:
- `rnea`, `aba`, `crba`, `forward_kinematics`, `frame_jacobian`
- 使用 `ctypes` 直接调 WASM .so/.dll

**提交:** `chore: add npm packaging, CI automation, Python bindings [Batch 19]`

---

## 7. 依赖关系图 (更新)

```
Batch 0 (架构拆分) ✅
  │
  ├──→ Batch 1 (Fixed + Prismatic) ✅
  │       │
  │       └──→ Batch 2 (JS SDK 核心) ✅ → Batch 4 (JS SDK 扩展) ✅
  │
  ├──→ Batch 3 (O(n) ABA) ✅
  │
  ├──→ Batch 5 (解析导数) ✅
  │
  └──→ Batch 6 (FreeFlyer/Spherical) ✅ → Batch 7 (ABA 多自由度) ✅
                                               │
                                               ▼
                                        ┌── Batch 8 (Bug + Quaternion) ← 下一个
                                        │      │
                                        │      ├──→ Batch 9  (JS 重构)
                                        │      │
                                        │      ├──→ Batch 10 (Clippy + Workspace)
                                        │      │
                                        │      ├──→ Batch 11 (C Header + 文档)
                                        │      │
                                        │      └──→ Batch 12 (Frame 系统)
                                        │             │
                                        │             ├──→ Batch 13 (Lie 群导数)
                                        │             │
                                        │             └──→ Batch 17 (通用约束)
                                        │
                                        ├──→ Batch 14 (Coriolis/M⁻¹/Jcom)
                                        │
                                        ├──→ Batch 15 (解析导数升级)
                                        │        ↑ Batch 10, 13
                                        │
                                        ├──→ Batch 16 (更多关节)
                                        │        ↑ Batch 8
                                        │
                                        ├──→ Batch 18 (碰撞升级)
                                        │
                                        └──→ Batch 19 (发布基础设施)
                                                 ↑ Batch 9, 11
```

---

## 8. Agent 自主执行规则

### 8.1 Agent 权限

- 创建/修改 `src/` 下的 Rust 源文件
- 创建/修改 `js/`, `include/`, `bindings/` 下的 SDK 文件
- 创建/修改 `tests/` 下的测试文件
- 运行 `cargo build`, `cargo test`, `cargo clippy`, `cargo fmt`
- 创建本地 git commit

### 8.2 Agent 禁止事项

- **禁止** `git push`
- **禁止** 修改 `Cargo.toml` 的 edition/version/license
- **禁止** 添加新依赖
- **禁止** 在 commit 中添加 Co-Authored-By 或 Signed-off-by

### 8.3 Agent 出错处理

| 情况 | 处理 |
|------|------|
| `cargo test` 失败 | 修复后重新运行，不跳过 |
| `cargo clippy` 警告 | 必须修复，不允许 `#[allow]` |
| WASM 编译失败 | 修复，不能降级为仅 native |
| 重构导致测试回归 | 回退该 commit，重新分析 |

### 8.4 进度记录

每完成一个 Batch，Agent 在 commit message 中标注:

```
feat(core): add Quaternion type, fix configuration-space ops [Batch 8]
```

---

## 9. 参考文档

| 文件 | 用途 |
|------|------|
| `PLAN/DEVELOPMENT_CHARTER.md` | 本文档: 架构约定和执行规范 |
| `PLAN/AUDIT.md` | 功能盘点与 JS SDK 覆盖 |
| `PLAN/PINOCCHIO_API_GAP_ANALYSIS.md` | vs C++ Pinocchio 329 项对比 |
