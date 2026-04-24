# pinocchio-wasm Development Charter

> 本文档定义开发范式、架构约定和执行规则。
> Agent 可依据本文档自主执行所有 Batch，无需人类逐条确认。
> 版本: 1.1 | 日期: 2026-04-24

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
┌─────────────────────────────────┐
│  JS SDK    (js/pinocchio_wasm.mjs)    │  ← 浏览器/Node.js 用户接口
├─────────────────────────────────┤
│  FFI 层    (src/ffi/*.rs)       │  ← extern "C" C ABI 导出
├─────────────────────────────────┤
│  算法层     (src/algo/*.rs)      │  ← 纯 Rust，无 unsafe，无 FFI 意识
├─────────────────────────────────┤
│  模型层     (src/model/*.rs)     │  ← Model, Workspace, 关节, 解析器
├─────────────────────────────────┤
│  核心层     (src/core/*.rs)      │  ← Vec3, Mat3, Transform, Error
└─────────────────────────────────┘
```

**核心原则: 单向依赖，算法层不知道 FFI 的存在。**

```
ffi → algo → model → core
ffi → model → core
ffi → collision → model → core
```

禁止反向依赖。禁止跨层调用（FFI 不得绕过 algo 直接操作 core）。

---

## 2. 目录结构约定

### 2.1 当前结构 (Batch 0 之前)

```
src/
├── lib.rs
├── core/
│   ├── mod.rs
│   ├── error.rs
│   └── math.rs
├── model/
│   └── mod.rs          (939行: Model + 4种解析器 + 4种导出器)
├── algo/
│   └── mod.rs          (2918行: 全部算法)
├── collision/
│   └── mod.rs          (521行)
├── ffi/
│   └── mod.rs          (2543行: 全部66个导出函数)
├── autodiff/
│   └── mod.rs
├── codegen/
│   └── mod.rs
└── visualization/
    └── mod.rs
```

### 2.2 目标结构 (Batch 0 完成后)

```
src/
├── lib.rs
├── core/
│   ├── mod.rs
│   ├── error.rs
│   └── math.rs
├── model/
│   ├── mod.rs          (Model, Workspace, Link, Joint + re-export)
│   ├── json.rs         (from_json_str, to_json_string)
│   ├── urdf.rs         (from_urdf_str, to_urdf_string)
│   ├── sdf.rs          (from_sdf_str, to_sdf_string)
│   └── mjcf.rs         (from_mjcf_str, to_mjcf_string)
├── algo/
│   ├── mod.rs          (pub use re-export)
│   ├── kinematics.rs   (forward_kinematics, fk_poses, fk_poses_batch)
│   ├── dynamics.rs     (rnea, aba, crba, bias_forces, gravity_torques, coriolis_torques)
│   ├── jacobian.rs     (frame_jacobian, contact_jacobian_normal)
│   ├── com_energy.rs   (center_of_mass, kinetic_energy, potential_energy, compute_all_terms)
│   ├── contact.rs      (constrained_forward_dynamics_contacts*, apply_contact_impulses*)
│   ├── constrained.rs  (constrained_aba_locked_joints*)
│   ├── centroidal.rs   (centroidal_momentum, centroidal_map, centroidal_full_terms*)
│   ├── regressors.rs   (inverse_dynamics_regressor*, kinetic/potential/com_regressor)
│   ├── derivatives.rs  (所有 _derivatives 函数)
│   ├── batch.rs        (所有 _batch 函数)
│   ├── rollout.rs      (rollout_aba_euler, integrate/diff/interp/random_configuration)
│   └── solvers.rs      (cholesky_solve, build_delassus_matrix, solve_contact_*)
├── collision/
│   ├── mod.rs          (CollisionModel, Geometry, PairFilter, AABB)
│   ├── broadphase.rs   (AABB 计算、重叠检测 — 未来)
│   └── narrowphase.rs  (距离查询、碰撞详情 — 未来拆分)
├── ffi/
│   ├── mod.rs          (Status, Handle 类型, 辅助函数, 内存管理)
│   ├── model.rs        (pino_model_create/free, from_json/urdf/sdf/mjcf, to_json/urdf/sdf/mjcf)
│   ├── dynamics.rs     (pino_rnea, pino_aba, pino_crba, pino_gravity/coriolis/bias)
│   ├── kinematics.rs   (pino_forward_kinematics_poses*, pino_compute_all_terms)
│   ├── contact.rs      (pino_contact_constrained_dynamics*, pino_apply_contact_impulse*)
│   ├── collision.rs    (pino_collision_model_*, pino_collision_min_distance*)
│   ├── centroidal.rs   (pino_centroidal_*)
│   ├── regressors.rs   (pino_*_regressor*)
│   ├── derivatives.rs  (pino_*_derivatives*)
│   └── batch.rs        (pino_*_batch)
├── autodiff/
│   └── mod.rs
├── codegen/
│   └── mod.rs
└── visualization/
    └── mod.rs
```

### 2.3 拆分规则

- **algo/mod.rs** → 只保留 `pub use` re-export，不保留实现代码
- **ffi/mod.rs** → 只保留 `Status`, `*Handle`, 辅助函数 (`check_non_null`, `as_slice` 等), 内存函数 (`pino_alloc/dealloc`)，以及 `pub mod` 声明
- **model/mod.rs** → 只保留 `Model`, `Workspace`, `Link`, `Joint`, `Model::new()`, `pub mod` 声明; 解析器代码全部移到对应文件
- 每个子文件头部 `use crate::...` 引入所需类型
- `lib.rs` 的 `pub mod` 列表不变

---

## 3. 编码约定

### 3.1 Rust 风格

| 规则 | 说明 |
|------|------|
| 无 unsafe | algo/model/collision 层禁止 unsafe，只有 ffi 层允许 |
| 无 unwrap | 用 `?` 和 `Result` 传播错误 |
| 无注释 | 代码即文档，只在 WHY 不明显时加一行注释 |
| `pub(crate)` 优先 | 内部函数不公开，只有需要跨模块的才 `pub` |
| `#[cfg(test)]` | 测试写在 `tests/` 目录，不在 src 内 |
| 命名 | 函数 snake_case, 类型 PascalCase, 模块 snake_case |

### 3.2 算法层签名模式

```rust
// 输入: model 引用 + 原始切片 + workspace 可变引用
// 输出: Result<输出类型>
// gravity 作为 Vec3 参数传入，不存 Model
pub fn xxx(model: &Model, q: &[f64], ..., ws: &mut Workspace) -> Result<OutputType>
```

### 3.3 FFI 层签名模式

```rust
// 统一返回 i32 (Status 枚举值)
// 输出通过 *mut f64 指针写入
// 用 run_status 包装 panic 捕获
#[unsafe(no_mangle)]
pub extern "C" fn pino_xxx(model: *const ModelHandle, ws: *mut WorkspaceHandle, ...) -> i32 {
    run_status(|| {
        check_non_null(model)?;
        let model_ref = unsafe { &(*model).model };
        let n = model_ref.nv();
        // ... 解指针，校验维度，调 algo::xxx，写输出 ...
        Ok(())
    }) as i32
}
```

### 3.4 JS SDK 签名模式

```javascript
// 每个 JS 函数: 接收 JS 类型，内部转换 WASM 指针，返回 JS 类型
// 所有数值数组用 Float64Array
// 返回 Array.from() 转为普通 JS 数组
// model/ws 用不透明数字 (指针值)

function xxx(model, ws, q, qd, ...) {
  const nq = Number(w.pino_model_nq(model));
  // 校验输入维度
  // 分配 WASM 内存 → 写入 → 调用 → 读出 → 释放
  return result;
}
```

---

## 4. 垂直切片执行规范

### 4.1 定义

一个 **垂直切片** = 一组相关功能的完整实现，从 Core → FFI → JS SDK → 测试。

### 4.2 每个 Slice 必须包含

```
1. src/algo/xxx.rs        ← 算法实现 (或修改现有)
2. src/ffi/xxx.rs         ← FFI 导出
3. js/pinocchio_wasm.mjs  ← JS 包装函数
4. include/pinocchio_wasm.h ← C 头文件声明 (如有新函数)
5. tests/                 ← 至少一个测试文件
6. examples/js/           ← 如有重大新功能，更新 demo
```

### 4.3 Slice 质量门禁

在标记 Slice 完成前，必须通过以下检查：

| # | 检查项 | 命令 |
|---|--------|------|
| 1 | 编译通过 (native) | `cargo build` |
| 2 | 编译通过 (WASM) | `cargo build --release --target wasm32-unknown-unknown` |
| 3 | 全部测试通过 | `cargo test --all-targets --all-features` |
| 4 | 无 clippy 警告 | `cargo clippy --all-targets --all-features` |
| 5 | 格式正确 | `cargo fmt --check` |

### 4.4 提交规范

```
<type>(<scope>): <描述>

type: feat | fix | refactor | docs | chore | test
scope: algo | ffi | js | model | collision | core | arch

禁止包含任何 Co-Authored-By 或 Signed-off-by 行。
作者: tianrking <tian.r.king@gmail.com>
```

示例:
```
feat(algo): add O(n) recursive ABA implementation
refactor(arch): split algo/mod.rs into per-algorithm modules
feat(js): wrap rnea, crba, jacobian, com, energy in JS SDK
```

---

## 5. Batch 执行计划

### Batch 0: 架构拆分 [优先级: P0]

**目标:** 将单文件巨型模块拆成目标结构，零功能变更。

**Slice 0.1 — 拆分 model/**
- 从 `model/mod.rs` 提取 JSON/URDF/SDF/MJCF 到独立文件
- `model/mod.rs` 只保留 Model/Workspace/Link/Joint + re-export
- 运行全部测试，确保零回归

**Slice 0.2 — 拆分 algo/**
- 按功能域拆分到 `kinematics.rs`, `dynamics.rs`, `jacobian.rs`, `com_energy.rs`, `contact.rs`, `constrained.rs`, `centroidal.rs`, `regressors.rs`, `derivatives.rs`, `batch.rs`, `rollout.rs`, `solvers.rs`
- `algo/mod.rs` 只保留 `pub use` re-export
- 运行全部测试，确保零回归

**Slice 0.3 — 拆分 ffi/**
- 按功能域拆分到 `model.rs`, `dynamics.rs`, `kinematics.rs`, `contact.rs`, `collision.rs`, `centroidal.rs`, `regressors.rs`, `derivatives.rs`, `batch.rs`
- `ffi/mod.rs` 只保留 Status/Handle/辅助函数
- 运行全部测试，确保零回归

**质量门禁:** Batch 0 完成后 `cargo test` 全绿，零功能变更，纯重构。

**状态:** ✅ 已完成 (commit 2e90b91)

---

### Batch 1: 关节类型扩展 (Fixed + Prismatic) [优先级: P0]

**目标:** 支持 Fixed 和 Prismatic 关节，使 URDF/SDF/MJCF 解析器能处理更多模型。

**Slice 1.1 — Core 关节类型** ✅
- `model/mod.rs` 添加 `JointType` 枚举 (Revolute | Prismatic | Fixed)
- `Joint` 添加 `jtype` 字段 + `prismatic()` / `fixed()` 构造器
- `Model` 添加 `joint_nq`, `joint_nv`, `joint_idx_q`, `joint_idx_v` 向量
- `nq()` / `nv()` 按关节类型求和

**Slice 1.2 — 算法适配** ✅
- `forward_kinematics` 按关节类型分派 (revolute 旋转, prismatic 平移, fixed 直连)
- `rnea` 后向传递: revolute 用 torque.dot, prismatic 用 force.dot, fixed 无 tau 贡献
- `jacobian`: revolute 线性部分 axis.cross(...), prismatic 直接 axis
- `contact_jacobian_row` 同上适配

**Slice 1.3 — 解析器适配** ✅
- URDF: 支持 `revolute` / `continuous` / `prismatic` / `fixed`
- SDF: 同上
- MJCF: 支持 `hinge` / `slide` / 无关节(fixed)
- JSON: 可选 `"type"` 字段，默认 `"revolute"`

**Slice 1.4 — FFI + JS SDK** ✅
- `pino_model_create` 新增 `joint_types_i32` 参数 (0=revolute, 1=prismatic, 2=fixed)
- C Header: 添加 `PINO_JOINT_REVOLUTE/PRISMATIC/FIXED` 常量
- JS SDK: `createModel()` 支持结构化 link 数据含关节类型
- 测试: 21 个新测试全部通过

**状态:** ✅ 已完成

---

### Batch 2: JS SDK 核心 [优先级: P1]

**目标:** JS SDK 从 6 个函数扩展到覆盖核心动力学。

**Slice 2.1 — 基础动力学** ✅
- JS: `rnea`, `crba` (返回 2D 数组), `gravityTorques`, `coriolisTorques`
- JS: `frameJacobian` (6×N), `centerOfMass`, `energy` ({kinetic, potential}), `computeAllTerms`

**Slice 2.2 — 模型加载** ✅
- JS: `createModelFromUrdf`, `createModelFromSdf`, `createModelFromMjcf`
- JS: `modelToJson`, `modelToUrdf`, `modelToSdf`, `modelToMjcf`
- JS: `modelNq`, `modelNlinks`

**Slice 2.3 — FK + Rollout + Batch + Contact + Collision + Centroidal + Regressors** ✅
- JS: `forwardKinematicsPoses`, `rolloutAbaEuler`
- JS: `rneaBatch`, `abaBatch`, `crbaBatch`
- JS: `contactConstrainedDynamics`, `applyContactImpulse`, `contactJacobianNormal`
- JS: `createCollisionModel`, `collisionMinDistance`, `collisionQueryDetails`
- JS: `centroidalMomentum`, `centroidalMap`, `centroidalFullTerms`
- JS: `inverseDynamicsRegressor`, `kineticEnergyRegressor`
- JS: `rneaSecondOrderDerivatives`, `constrainedAbaLockedJoints`

**JS SDK 总计:** 40+ 包装函数（从 6 个增长），覆盖 FFI 层 ~60 个导出函数
**Demo:** 更新 node_demo.mjs 展示 ABA/RNEA/CRBA/CoM/Energy/Jacobian 等

**状态:** ✅ 已完成

---

### Batch 3: O(n) ABA [优先级: P1]

**目标:** 将 ABA 从 O(n³) CRBA+Cholesky 改为标准 O(n) 递归算法。

**状态:** ✅ 已完成 [Batch 3]

**实现记录:**
- `aba()` 已替换为 O(n) 三遍 Articulated Body Algorithm，内部使用 6×6 spatial inertia / force 表示，与原版 Pinocchio 的 `Yaba` 思路对齐。
- 旧 CRBA+Cholesky 正动力学保留为 `aba_crba()`，用于交叉验证和回归保护。
- 覆盖 revolute、fixed、prismatic、混合关节、零速度/零重力、多构型链的 `aba` vs `aba_crba` 数值一致性测试。
- 质量门禁: `cargo test --all-targets --all-features` 全部通过，累计 81 个测试；`node --check js/pinocchio_wasm.mjs` 通过。
- 备注: `cargo clippy --all-targets --all-features -- -D warnings` 当前仍被既有 batch/contact/rollout API 的 `too_many_arguments` 和旧循环 lint 阻塞，非 Batch 3 新增功能失败。

**Slice 3.1 — 实现 Articulated Body Algorithm**
- 实现 O(n) 递归前向 pass + 后向 pass
- 保留旧实现作为 `aba_crba()` 用于交叉验证
- 测试: 新旧实现输出在数值精度内一致 (atol=1e-10)

**Slice 3.2 — CRBA 优化**
- 可选: 实现 O(n) 递归 CRBA 替代逐列 RNEA
- 测试: 对称性和正定性验证

---

### Batch 4: JS SDK 扩展 [优先级: P1]

**目标:** JS SDK 覆盖 batch/接触/碰撞/质心/回归。

**Slice 4.1 — Batch 系列**
- JS: `rneaBatch`, `abaBatch`, `crbaBatch`, `biasForcesBatch`, `gravityTorquesBatch`

**Slice 4.2 — 接触动力学**
- JS: `contactConstrainedDynamics`, `applyContactImpulse`
- JS: 摩擦变体
- JS: `contactJacobianNormal`

**Slice 4.3 — 碰撞**
- JS: `createCollisionModel`, `collisionMinDistance`, `collisionQueryDetails`

**Slice 4.4 — 质心 + 回归**
- JS: `centroidalMap`, `centroidalMomentum`, `centroidalFullTerms`
- JS: `inverseDynamicsRegressor`, `kineticEnergyRegressor`

---

### Batch 5: 解析导数 [优先级: P1]

**目标:** 所有导数从有限差分改为解析递归算法。

**Slice 5.1 — RNEA 解析导数**
- 实现 O(n) `computeRNEADerivatives` (dtau_dq, dtau_dv, dtau_da)
- 测试: 与有限差分交叉验证 (atol=1e-6)

**Slice 5.2 — ABA 解析导数**
- 实现 O(n) `computeABADerivatives` (ddq_dq, ddq_dv, ddq_dtau)
- 测试: 交叉验证

**Slice 5.3 — 其他解析导数**
- 运动学导数、雅可比导数、质心导数
- 每个都与有限差分交叉验证后再替换

---

### Batch 6: FreeFlyer + Spherical 关节 [优先级: P2]

**目标:** 支持浮动基和球关节，可建模人形/四足机器人。

**Slice 6.1 — 关节类型**
- `FreeFlyer` (nq=7/nv=6, 四元数表示)
- `Spherical` (nq=4/nv=3, 四元数表示)

**Slice 6.2 — 数学基础**
- 四元数运算 (乘法、归一化、从/到旋转矩阵)
- SO(3) exp/log 映射
- 李群积分 (integrate on quaternion manifolds)

**Slice 6.3 — 算法适配**
- FK/RNEA/ABA/CRBA 适配 nq≠nv 的关节
- 位形空间操作适配四元数

---

## 6. 依赖关系图

```
Batch 0 (架构拆分)
  │
  ├──→ Batch 1 (Fixed + Prismatic)
  │       │
  │       └──→ Batch 2 (JS SDK 核心) ← 可与 Batch 1 部分并行
  │
  ├──→ Batch 3 (O(n) ABA) ← 独立于 Batch 1/2，可在 Batch 0 后任意时间
  │
  ├──→ Batch 4 (JS SDK 扩展) ← 依赖 Batch 2
  │
  ├──→ Batch 5 (解析导数) ← 独立，但建议 Batch 3 之后
  │
  └──→ Batch 6 (FreeFlyer/Spherical) ← 依赖 Batch 1 的关节框架
```

---

## 7. Agent 自主执行规则

### 7.1 Agent 权限

Agent 在以下范围内自主操作，无需逐条确认：
- 创建/修改 `src/` 下的 Rust 源文件
- 创建/修改 `js/`, `include/`, `bindings/` 下的 SDK 文件
- 创建/修改 `tests/` 下的测试文件
- 运行 `cargo build`, `cargo test`, `cargo clippy`, `cargo fmt`
- 创建本地 git commit（遵循提交规范，无签名）

### 7.2 Agent 禁止事项

- **禁止** `git push` — 推送必须人工确认
- **禁止** 修改 `Cargo.toml` 的 edition/version/license — 需确认
- **禁止** 添加新依赖 — 需确认
- **禁止** 修改 `PLAN/` 下的文档 — 这些是规范不是产出
- **禁止** 在 commit 中添加 Co-Authored-By 或 Signed-off-by

### 7.3 Agent 质量自检清单

每个 Slice 完成时 Agent 必须执行:

```
□ cargo fmt --check          → 通过
□ cargo clippy --all-targets → 无警告
□ cargo test --all-targets   → 全绿
□ cargo build --release --target wasm32-unknown-unknown → 通过
□ git diff --stat            → 确认变更范围合理
□ git commit                 → 符合提交规范
```

### 7.4 Agent 出错处理

| 情况 | 处理 |
|------|------|
| `cargo test` 失败 | 修复后重新运行，不跳过 |
| `cargo clippy` 警告 | 必须修复，不允许 `#[allow]` |
| WASM 编译失败 | 修复，不能降级为仅 native |
| 重构导致测试回归 | 回退该 commit，重新分析 |
| 不确定设计决策 | 记录到 PLAN/NOTES.md 并继续下一个 Slice |

### 7.5 进度记录

每完成一个 Slice，Agent 在 commit message 中标注 Slice 编号:

```
feat(algo): implement O(n) recursive ABA [Batch 3 Slice 3.1]
```

---

## 8. 参考文档

| 文件 | 用途 |
|------|------|
| `PLAN/AUDIT.md` | 自身功能盘点，JS SDK 补全清单 |
| `PLAN/PINOCCHIO_API_GAP_ANALYSIS.md` | vs C++ Pinocchio 329 项对比，P0/P1/P2 优先级 |
| `PLAN/DEVELOPMENT_CHARTER.md` | 本文档，架构约定和执行规范 |
