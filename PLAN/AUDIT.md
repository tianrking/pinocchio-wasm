# pinocchio-wasm 功能梳理

> 更新时间: 2026-04-24 | 版本: v0.0.1
> 基于: src/algo/*.rs, src/model/mod.rs, src/collision/mod.rs, src/ffi/*.rs, js/pinocchio_wasm.mjs
> 测试基线: 97 passed, 0 failed | 39 test files | 72 FFI exports

---

## 一、内核功能清单

### 1. 运动学与动力学算法 (src/algo/)

| # | 算法 | 函数名 | 状态 | 备注 |
|---|------|--------|------|------|
| 1 | 正运动学 (FK) | `forward_kinematics` | ✅ 完整 | 世界位姿、角速度、线速度、角加速度、线加速度 |
| 2 | 逆动力学 (RNEA) | `rnea` | ✅ 完整 | 标准前向+后向传播，支持 5 种关节 |
| 3 | 偏置力 | `bias_forces` | ✅ 完整 | qdd=0 的 RNEA |
| 4 | 重力项 | `gravity_torques` | ✅ 完整 | qd=0, qdd=0 的 RNEA |
| 5 | 科氏力项 | `coriolis_torques` | ✅ 完整 | qdd=0, gravity=0 的 RNEA |
| 6 | 质量矩阵 (CRBA) | `crba` | ✅ 完整 | 逐列 RNEA + 对称化，O(n²) |
| 7 | 正动力学 (ABA) | `aba` | ✅ 完整 | **O(n) articulated-body 递推**；支持 Revolute/Prismatic/Fixed/Spherical/FreeFlyer |
| 8 | 正动力学 (CRBA) | `aba_crba` | ✅ 完整 | O(n³) 回退路径，用于交叉验证 |
| 9 | 帧雅可比 | `frame_jacobian` | ✅ 完整 | 6×n 矩阵 (世界坐标系) |
| 10 | 质心位置 | `center_of_mass` | ✅ 完整 | 加权质心 |
| 11 | 动能 | `kinetic_energy` | ✅ 完整 | 平动+转动 |
| 12 | 势能 | `potential_energy` | ✅ 完整 | 重力势能 |

### 2. 接触动力学 (src/algo/contact.rs)

| # | 算法 | 函数名 | 状态 |
|---|------|--------|------|
| 13 | 接触正动力学 (法向) | `constrained_forward_dynamics_contacts` | ✅ 完整 |
| 14 | 接触正动力学 (摩擦) | `constrained_forward_dynamics_contacts_friction` | ✅ 完整 |
| 15 | 接触冲量 (法向) | `apply_contact_impulses` | ✅ 完整 |
| 16 | 接触冲量 (摩擦) | `apply_contact_impulses_friction` | ✅ 完整 |
| 17 | 接触法向雅可比 | `contact_jacobian_normal` | ✅ 完整 |
| 18 | 关节锁定约束 ABA | `constrained_aba_locked_joints` | ✅ 完整 |
| 19 | 综合计算 | `compute_all_terms` | ✅ 完整 |

### 3. Batch 批量接口 (src/algo/batch.rs, contact.rs)

| # | 算法 | 函数名 | 状态 |
|---|------|--------|------|
| 20 | RNEA 批量 | `rnea_batch` | ✅ 完整 |
| 21 | ABA 批量 | `aba_batch` | ✅ 完整 |
| 22 | 偏置力批量 | `bias_forces_batch` | ✅ 完整 |
| 23 | 重力项批量 | `gravity_torques_batch` | ✅ 完整 |
| 24 | CRBA 批量 | `crba_batch` | ✅ 完整 |
| 25 | 关节锁定 ABA 批量 | `constrained_aba_locked_joints_batch` | ✅ 完整 |
| 26 | 接触正动力学批量 (法向) | `constrained_forward_dynamics_contacts_batch` | ✅ 完整 |
| 27 | 接触冲量批量 (法向) | `apply_contact_impulses_batch` | ✅ 完整 |
| 28 | 接触正动力学批量 (摩擦) | `constrained_forward_dynamics_contacts_friction_batch` | ✅ 完整 |
| 29 | 接触冲量批量 (摩擦) | `apply_contact_impulses_friction_batch` | ✅ 完整 |

### 4. 轨迹与积分 (src/algo/rollout.rs)

| # | 算法 | 函数名 | 状态 | 备注 |
|---|------|--------|------|------|
| 30 | ABA Euler 积分 | `rollout_aba_euler` | ✅ 完整 | 四元数流形积分 |
| 31 | FK 位姿输出 | `forward_kinematics_poses` | ✅ 完整 |
| 32 | FK 位姿批量 | `forward_kinematics_poses_batch` | ✅ 完整 |
| 33 | 位形积分 | `integrate_model_configuration` | ✅ 完整 | 按关节类型分派，四元数正确 |
| 34 | 位形积分 (无 model) | `integrate_configuration` | ⚠️ BUG | 纯加法，nq≠nv 时出错 |
| 35 | 位形差分 | `difference_configuration` | ⚠️ BUG | 纯减法，四元数错误 |
| 36 | 位形插值 | `interpolate_configuration` | ⚠️ BUG | 线性插值，四元数非单位 |
| 37 | 随机位形 | `random_configuration` | ⚠️ BUG | 不归一化四元数 |
| 38 | 中立位形 | `neutral_configuration` | ❌ 缺失 | 四元数初值应为 [1,0,0,0] |

### 5. 质心量与导数 (src/algo/centroidal.rs)

| # | 算法 | 函数名 | 状态 | 备注 |
|---|------|--------|------|------|
| 39 | 质心动量 | `centroidal_momentum` | ✅ 完整 | |
| 40 | 质心映射矩阵 | `centroidal_map` | ✅ 完整 | 6×nv |
| 41 | 质心导数 | `centroidal_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 42 | 质心映射导数 | `centroidal_map_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 43 | 质心动量变化率 | `centroidal_momentum_rate` | ✅ 完整 | |
| 44 | 质心接触力 | `centroidal_contact_wrench` | ✅ 完整 | |
| 45 | 质心全量 | `centroidal_full_terms` | ✅ 完整 | |
| 46 | 质心全量+接触 | `centroidal_full_terms_with_contacts` | ✅ 完整 | |

### 6. 导数与回归矩阵 (src/algo/derivatives.rs, regressors.rs)

| # | 算法 | 函数名 | 状态 | 备注 |
|---|------|--------|------|------|
| 47 | RNEA 导数 | `rnea_derivatives` | ✅ 完整 | 混合: dq/dv=FD, dqdd=CRBA |
| 48 | ABA 导数 | `aba_derivatives` | ✅ 完整 | 隐式: M⁻¹·dtau |
| 49 | 运动学导数 | `kinematics_derivatives` | ✅ 完整 | 解析 |
| 50 | 帧雅可比导数 | `frame_jacobian_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 51 | 质心导数 | `center_of_mass_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 52 | RNEA 二阶导数 | `rnea_second_order_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 53 | 关节锁定导数 | `constrained_dynamics_derivatives_locked_joints` | ✅ 完整 | ⚠️ 有限差分 |
| 54 | 冲量动力学导数 | `impulse_dynamics_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 55 | 逆动力学回归矩阵 | `inverse_dynamics_regressor` | ✅ 完整 | ⚠️ 有限差分 (扰动惯性参数) |
| 56 | 逆动力学回归矩阵批量 | `inverse_dynamics_regressor_batch` | ✅ 完整 | ⚠️ 同上 |
| 57 | 动能回归矩阵 | `kinetic_energy_regressor` | ✅ 完整 | ⚠️ 有限差分 |
| 58 | 势能回归矩阵 | `potential_energy_regressor` | ✅ 完整 | ⚠️ 有限差分 |
| 59 | 质心回归矩阵 | `center_of_mass_regressor` | ✅ 完整 | ⚠️ 有限差分 |
| 60 | 独立列选择 | `select_independent_regressor_columns` | ✅ 完整 | |

### 7. 接触求解器 (src/algo/solvers.rs)

| # | 算法 | 函数名 | 状态 |
|---|------|--------|------|
| 61 | Cholesky 分解 | `cholesky_solve` | ✅ 完整 |
| 62 | Delassus 矩阵构建 | `build_delassus_matrix` | ✅ 完整 |
| 63 | 接触问题构建 | `build_contact_problem` | ✅ 完整 |
| 64 | Cholesky 求解 | `solve_contact_cholesky` | ✅ 完整 |
| 65 | PGS 求解 | `solve_contact_pgs` | ✅ 完整 |
| 66 | ADMM 求解 | `solve_contact_admm` | ✅ 完整 |

### 8. 碰撞检测 (src/collision/mod.rs)

| # | 算法 | 函数名 | 状态 | 备注 |
|---|------|--------|------|------|
| 67 | 碰撞模型创建 (球体) | `CollisionModel::from_spheres` | ✅ 完整 | |
| 68 | 碰撞模型创建 (几何体) | `CollisionModel::from_geometries` | ✅ 完整 | Sphere/Box/Capsule/Cylinder |
| 69 | 最小距离 | `minimum_distance` | ✅ 完整 | AABB + 球体近似 |
| 70 | 碰撞详情查询 | `collision_details` | ✅ 完整 | |
| 71 | 批量距离查询 | `minimum_distance_batch` | ✅ 完整 | |

### 9. 模型管理 (src/model/)

| # | 功能 | 函数名 | 状态 | 备注 |
|---|------|--------|------|------|
| 72 | 模型构建 | `Model::new` | ✅ 完整 | 拓扑验证，5 种关节类型 |
| 73 | JSON 加载/导出 | `from_json_str` / `to_json_string` | ✅ 完整 | |
| 74 | URDF 加载/导出 | `from_urdf_str` / `to_urdf_string` | ✅ 完整 | revolute/continuous/prismatic/fixed/ball/floating |
| 75 | SDF 加载/导出 | `from_sdf_str` / `to_sdf_string` | ✅ 完整 | 同上 |
| 76 | MJCF 加载/导出 | `from_mjcf_str` / `to_mjcf_string` | ✅ 完整 | hinge/slide/fixed/ball/free |
| 77 | Workspace 创建 | `Workspace::new` | ✅ 完整 | 含 ABA scratch buffers |

---

## 二、关节类型支持矩阵

| 算法 | Revolute | Prismatic | Fixed | Spherical | FreeFlyer |
|------|----------|-----------|-------|-----------|-----------|
| 正运动学 (FK) | ✅ | ✅ | ✅ | ✅ | ✅ |
| 逆动力学 (RNEA) | ✅ | ✅ | ✅ | ✅ | ✅ |
| 质量矩阵 (CRBA) | ✅ | ✅ | ✅ | ✅ | ✅ |
| 正动力学 (ABA O(n)) | ✅ | ✅ | ✅ | ✅ | ✅ |
| 帧雅可比 | ✅ | ✅ | N/A | ⚠️ BUG | ✅ |
| 接触雅可比 | ✅ | ✅ | N/A | ✅ | ✅ |
| 质心/能量 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 接触动力学 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 质心动量 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 回归矩阵 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 位形积分 (model-aware) | ✅ | ✅ | ✅ | ✅ | ✅ |
| 位形积分 (plain) | ✅ | ✅ | N/A | ❌ | ❌ |
| 位形差分 (plain) | ✅ | ✅ | N/A | ❌ | ❌ |
| 位形插值 (plain) | ✅ | ✅ | N/A | ❌ | ❌ |
| 随机位形 | ✅ | ✅ | ✅ | ❌ | ❌ |

---

## 三、FFI 导出覆盖度

FFI 层共导出 **72** 个 `pino_*` 函数。

| 文件 | 导出数 | 覆盖算法 |
|------|--------|---------|
| `ffi/model.rs` | 18 | model create/free, load/export (JSON/URDF/SDF/MJCF), nq/nv/nlinks |
| `ffi/dynamics.rs` | 5 | rnea, aba, crba, gravity/coriolis torques |
| `ffi/kinematics.rs` | 6 | FK_poses, FK_poses_batch, compute_all_terms, center_of_mass, energy, frame_jacobian |
| `ffi/contact.rs` | 11 | contact FD (normal/friction), impulse (normal/friction), batch variants |
| `ffi/collision.rs` | 5 | collision model create (spheres/geometries), distance, query, dispose |
| `ffi/centroidal.rs` | 6 | momentum, map, full_terms, derivatives, momentum_rate, contact_wrench |
| `ffi/regressors.rs` | 6 | ID/KE regressor, batch ID regressor, PE/CoM regressor, column selection |
| `ffi/derivatives.rs` | 6 | RNEA/ABA/kinematics derivatives, 2nd-order RNEA, constrained/impulse |
| `ffi/batch.rs` | 6 | rnea/aba/crba/bias/gravity batch, constrained ABA batch, rollout |
| `ffi/mod.rs` | 3 | alloc, dealloc, workspace |

C Header (`include/pinocchio_wasm.h`) 声明 52 个函数，**缺 20 个** (见 Batch 11 计划)。

---

## 四、JS SDK 覆盖度

JS SDK (`js/pinocchio_wasm.mjs`, 1,423 行 + `js/sdk/runtime.mjs`, 89 行)

**已包装的用户 API 函数 (~50 个):**

| 类别 | 函数 |
|------|------|
| 模型生命周期 | `createModel`, `createModelFromJson/Urdf/Sdf/Mjcf`, `modelToJson/Urdf/Sdf/Mjcf`, `modelNq`, `modelNv`, `modelNlinks`, `newWorkspace`, `disposeModel`, `disposeWorkspace` |
| 核心动力学 | `rnea`, `aba`, `crba`, `gravityTorques`, `coriolisTorques` |
| 运动学 | `frameJacobian`, `centerOfMass`, `energy`, `computeAllTerms`, `forwardKinematicsPoses` |
| 批量操作 | `rneaBatch`, `abaBatch`, `crbaBatch`, `rolloutAbaEuler` |
| 接触动力学 | `contactConstrainedDynamics`, `applyContactImpulse`, `contactJacobianNormal` |
| 碰撞 | `createCollisionModel`, `createCollisionModelGeometries`, `disposeCollisionModel`, `collisionMinDistance`, `collisionQueryDetails` |
| 质心 | `centroidalMomentum`, `centroidalMap`, `centroidalFullTerms` |
| 回归 | `inverseDynamicsRegressor`, `kineticEnergyRegressor` |
| 导数 | `rneaDerivatives`, `abaDerivatives`, `kinematicsDerivatives`, `rneaSecondOrderDerivatives` |
| 约束 | `constrainedAbaLockedJoints` |
| 内存工具 | `allocBytes`, `freeBytes`, `memoryU8/F64/I32/U32`, `writeBytes` |

**JS SDK 未包装的 FFI (~20 个):**

| 未包装函数 | FFI 名称 |
|-----------|---------|
| 偏置力批量 | `pino_bias_forces_batch` |
| 重力项批量 | `pino_gravity_torques_batch` |
| 关节锁定 ABA 批量 | `pino_constrained_aba_locked_joints_batch` |
| 接触正动力学批量 | `pino_constrained_forward_dynamics_contacts_batch` 等 |
| 接触冲量批量 | `pino_apply_contact_impulses_batch` 等 |
| 质心导数 | `pino_centroidal_derivatives` 等 |
| 势能/质心回归 | `pino_potential_energy_regressor`, `pino_center_of_mass_regressor` |
| 约束动力学导数 | `pino_constrained_dynamics_derivatives_locked_joints` |
| 冲量动力学导数 | `pino_impulse_dynamics_derivatives` |
| 帧雅可比导数 | `pino_frame_jacobian_derivatives` |
| 质心导数 | `pino_center_of_mass_derivatives` |
| 回归列选择 | `pino_select_independent_regressor_columns` |
| FK (有速度/加速度) | 未导出 FFI |
| 偏置力 (单次) | 未导出 FFI |

---

## 五、已知问题汇总

### BUG (需修复)

| # | 问题 | 严重度 | 位置 |
|---|------|--------|------|
| B1 | JS `nq`/`nv` 维度校验错误 | P0 | `js/pinocchio_wasm.mjs` |
| B2 | `integrate_configuration` 纯加法 | P0 | `algo/rollout.rs` |
| B3 | `difference_configuration` 纯减法 | P0 | `algo/rollout.rs` |
| B4 | `interpolate_configuration` 线性插值 | P0 | `algo/rollout.rs` |
| B5 | `random_configuration` 不归一化 | P0 | `algo/rollout.rs` |
| B6 | 缺少 `neutral_configuration` | P0 | — |
| B7 | Spherical Jacobian 返回零 | P0 | `algo/jacobian.rs` |

### 代码质量

| # | 问题 | 位置 |
|---|------|------|
| Q1 | 四元数运算散落在 rollout.rs 私有函数 | `algo/rollout.rs` |
| Q2 | 缺 `from_rotation_matrix` | `core/math.rs` |
| Q3 | 缺 SO(3) `log_map` | `core/math.rs` |
| Q4 | JS SDK 1,423 行单文件 43% 样板 | `js/pinocchio_wasm.mjs` |
| Q5 | Clippy 39 个警告 | 多文件 |

### 性能

| # | 当前 | 理想 |
|---|------|------|
| CRBA | O(n²) 逐列 RNEA | O(n) 递归 |
| 导数 | O(n²) 有限差分 | O(n) 解析递推 |

---

## 六、统计总览

| 指标 | 数值 |
|------|------|
| FFI 导出函数 | 72 |
| JS SDK API 函数 | ~50 |
| 算法函数 (algo/) | 77 |
| 测试文件 | 39 |
| 测试用例 | 97 passed, 0 failed |
| 关节类型 | 5 (Revolute/Prismatic/Fixed/Spherical/FreeFlyer) |
| 模型格式 | 4 (JSON/URDF/SDF/MJCF, 各支持 import+export) |
| Rust 源文件 | 37 |
