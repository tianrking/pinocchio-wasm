# pinocchio-wasm 功能梳理

> 生成时间: 2026-04-24
> 基于: src/algo/mod.rs, src/model/mod.rs, src/collision/mod.rs, src/ffi/mod.rs, js/pinocchio_wasm.mjs

---

## 一、内核功能清单

### 1. 运动学与动力学算法 (src/algo/mod.rs)

| # | 算法 | 函数名 | 实现状态 | 备注 |
|---|------|--------|----------|------|
| 1 | 正运动学 (FK) | `forward_kinematics` | ✅ 完整 | 世界位姿、角速度、加速度 |
| 2 | 逆动力学 (RNEA) | `rnea` | ✅ 完整 | 标准前向+后向传播 |
| 3 | 偏置力 | `bias_forces` | ✅ 完整 | qdd=0 的 RNEA |
| 4 | 重力项 | `gravity_torques` | ✅ 完整 | qd=0, qdd=0 的 RNEA |
| 5 | 科氏力项 | `coriolis_torques` | ✅ 完整 | qdd=0, gravity=0 的 RNEA |
| 6 | 质量矩阵 (CRBA) | `crba` | ✅ 完整 | 逐列 RNEA + 对称化 |
| 7 | 正动力学 (ABA) | `aba` | ✅ 完整 | O(n) articulated-body 递推；支持 1-DoF、Spherical、FreeFlyer |
| 8 | 帧雅可比 | `frame_jacobian` | ✅ 完整 | 6×n 矩阵 |
| 9 | 质心位置 | `center_of_mass` | ✅ 完整 | 加权质心 |
| 10 | 动能 | `kinetic_energy` | ✅ 完整 | 平动+转动 |
| 11 | 势能 | `potential_energy` | ✅ 完整 | 重力势能 |

### 2. 接触动力学 (src/algo/mod.rs)

| # | 算法 | 函数名 | 实现状态 | 备注 |
|---|------|--------|----------|------|
| 12 | 接触约束正动力学 (法向) | `constrained_forward_dynamics_contacts` | ✅ 完整 | Gauss-Seidel 求解器 |
| 13 | 接触约束正动力学 (摩擦) | `constrained_forward_dynamics_contacts_friction` | ✅ 完整 | 摩擦锥近似 |
| 14 | 接触冲量 (法向) | `apply_contact_impulses` | ✅ 完整 | |
| 15 | 接触冲量 (摩擦) | `apply_contact_impulses_friction` | ✅ 完整 | |
| 16 | 接触法向雅可比 | `contact_jacobian_normal` | ✅ 完整 | |
| 17 | 关节锁定约束 ABA | `constrained_aba_locked_joints` | ✅ 完整 | |
| 18 | 综合计算 | `compute_all_terms` | ✅ 完整 | 一次性算出 M, b, CoM, 能量等 |

### 3. Batch 批量接口 (src/algo/mod.rs)

| # | 算法 | 函数名 | 实现状态 |
|---|------|--------|----------|
| 19 | RNEA 批量 | `rnea_batch` | ✅ 完整 |
| 20 | ABA 批量 | `aba_batch` | ✅ 完整 |
| 21 | 偏置力批量 | `bias_forces_batch` | ✅ 完整 |
| 22 | 重力项批量 | `gravity_torques_batch` | ✅ 完整 |
| 23 | CRBA 批量 | `crba_batch` | ✅ 完整 |
| 24 | 关节锁定 ABA 批量 | `constrained_aba_locked_joints_batch` | ✅ 完整 |
| 25 | 接触正动力学批量 (法向) | `constrained_forward_dynamics_contacts_batch` | ✅ 完整 |
| 26 | 接触冲量批量 (法向) | `apply_contact_impulses_batch` | ✅ 完整 |
| 27 | 接触正动力学批量 (摩擦) | `constrained_forward_dynamics_contacts_friction_batch` | ✅ 完整 |
| 28 | 接触冲量批量 (摩擦) | `apply_contact_impulses_friction_batch` | ✅ 完整 |

### 4. 轨迹与积分 (src/algo/mod.rs)

| # | 算法 | 函数名 | 实现状态 |
|---|------|--------|----------|
| 29 | ABA Euler 积分 | `rollout_aba_euler` | ✅ 完整 |
| 30 | FK 位姿输出 | `forward_kinematics_poses` | ✅ 完整 |
| 31 | FK 位姿批量 | `forward_kinematics_poses_batch` | ✅ 完整 |
| 32 | 位形积分 | `integrate_configuration` | ✅ 完整 |
| 33 | 位形差分 | `difference_configuration` | ✅ 完整 |
| 34 | 位形插值 | `interpolate_configuration` | ✅ 完整 |
| 35 | 随机位形 | `random_configuration` | ✅ 完整 |

### 5. 质心量与导数 (src/algo/mod.rs)

| # | 算法 | 函数名 | 实现状态 | 备注 |
|---|------|--------|----------|------|
| 36 | 质心动量 | `centroidal_momentum` | ✅ 完整 | |
| 37 | 质心映射矩阵 | `centroidal_map` | ✅ 完整 | |
| 38 | 质心导数 | `centroidal_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 39 | 质心映射导数 | `centroidal_map_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 40 | 质心动量变化率 | `centroidal_momentum_rate` | ✅ 完整 | |
| 41 | 质心接触力 | `centroidal_contact_wrench` | ✅ 完整 | |
| 42 | 质心全量 | `centroidal_full_terms` | ✅ 完整 | |
| 43 | 质心全量+接触 | `centroidal_full_terms_with_contacts` | ✅ 完整 | |

### 6. 二阶导数与回归矩阵 (src/algo/mod.rs)

| # | 算法 | 函数名 | 实现状态 | 备注 |
|---|------|--------|----------|------|
| 44 | RNEA 二阶导数 | `rnea_second_order_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 45 | 关节锁定导数 | `constrained_dynamics_derivatives_locked_joints` | ✅ 完整 | ⚠️ 有限差分 |
| 46 | 冲量动力学导数 | `impulse_dynamics_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 47 | RNEA 导数 | `rnea_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 48 | ABA 导数 | `aba_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 49 | 运动学导数 | `kinematics_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 50 | 帧雅可比导数 | `frame_jacobian_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 51 | 质心导数 | `center_of_mass_derivatives` | ✅ 完整 | ⚠️ 有限差分 |
| 52 | 逆动力学回归矩阵 | `inverse_dynamics_regressor` | ✅ 完整 | ⚠️ 有限差分, 每次调用克隆 20n 个 model |
| 53 | 逆动力学回归矩阵批量 | `inverse_dynamics_regressor_batch` | ✅ 完整 | ⚠️ 同上 |
| 54 | 动能回归矩阵 | `kinetic_energy_regressor` | ✅ 完整 | ⚠️ 有限差分 |
| 55 | 势能回归矩阵 | `potential_energy_regressor` | ✅ 完整 | ⚠️ 有限差分 |
| 56 | 质心回归矩阵 | `center_of_mass_regressor` | ✅ 完整 | ⚠️ 有限差分 |
| 57 | 独立列选择 | `select_independent_regressor_columns` | ✅ 完整 | |

### 7. 接触求解器 (src/algo/mod.rs)

| # | 算法 | 函数名 | 实现状态 |
|---|------|--------|----------|
| 58 | Delassus 矩阵构建 | `build_delassus_matrix` | ✅ 完整 |
| 59 | 接触问题构建 | `build_contact_problem` | ✅ 完整 |
| 60 | Cholesky 求解 | `solve_contact_cholesky` | ✅ 完整 |
| 61 | PGS 求解 | `solve_contact_pgs` | ✅ 完整 |
| 62 | ADMM 求解 | `solve_contact_admm` | ✅ 完整 |

### 8. 碰撞检测 (src/collision/mod.rs)

| # | 算法 | 函数名 | 实现状态 | 备注 |
|---|------|--------|----------|------|
| 63 | 碰撞详情查询 | `collision_details` | ✅ 完整 | ⚠️ narrowphase 全部退化为球体近似 |
| 64 | 最小距离 | `minimum_distance` | ✅ 完整 | |
| 65 | 最小距离 (详细) | `minimum_distance_detailed` | ✅ 完整 | |
| 66 | 最小距离批量 | `minimum_distance_batch` | ✅ 完整 | |
| 67 | 最小距离详细批量 | `minimum_distance_detailed_batch` | ✅ 完整 | |
| 68 | AABB 包围盒 | `Aabb::overlaps` | ✅ 代码存在 | ⚠️ 结果 `_broadphase_overlaps` 计算了但从未使用 |

### 9. 模型管理 (src/model/mod.rs)

| # | 功能 | 函数名 | 实现状态 | 备注 |
|---|------|--------|----------|------|
| 69 | 模型构建 | `Model::new` | ✅ 完整 | 拓扑验证 |
| 70 | JSON 加载 | `Model::from_json_str` | ✅ 完整 | |
| 71 | JSON 导出 | `Model::to_json_string` | ✅ 完整 | |
| 72 | URDF 加载 | `Model::from_urdf_str` | ✅ 完整 | 支持 revolute/continuous/prismatic/fixed/ball/floating |
| 73 | URDF 导出 | `Model::to_urdf_string` | ✅ 完整 | |
| 74 | SDF 加载 | `Model::from_sdf_str` | ✅ 完整 | 支持 revolute/continuous/prismatic/fixed/ball/floating |
| 75 | SDF 导出 | `Model::to_sdf_string` | ✅ 完整 | |
| 76 | MJCF 加载 | `Model::from_mjcf_str` | ✅ 完整 | 支持 hinge/slide/fixed/ball/free |
| 77 | MJCF 导出 | `Model::to_mjcf_string` | ✅ 完整 | |
| 78 | Workspace 创建 | `Workspace::new` | ✅ 完整 | |

---

## 二、FFI 导出覆盖度 (src/ffi/mod.rs)

FFI 层共导出 **72 个** `pino_*` 函数，并提供 `include/pinocchio_wasm.h` C 头文件。

### 与内核算法的对应关系

| 内核函数 | FFI 导出 | 状态 |
|----------|----------|------|
| `forward_kinematics` | — | ❌ 未导出（仅通过 `compute_all_terms` 间接调用） |
| `rnea` | `pino_rnea` | ✅ |
| `rnea_batch` | `pino_rnea_batch` | ✅ |
| `bias_forces` | `pino_bias_forces_batch` (batch only) | ⚠️ 缺少单次版 |
| `gravity_torques` | `pino_gravity_torques` | ✅ |
| `gravity_torques_batch` | `pino_gravity_torques_batch` | ✅ |
| `coriolis_torques` | `pino_coriolis_torques` | ✅ |
| `crba` | `pino_crba` | ✅ |
| `crba_batch` | `pino_crba_batch` | ✅ |
| `aba` | `pino_aba` | ✅ |
| `aba_batch` | `pino_aba_batch` | ✅ |
| `frame_jacobian` | `pino_frame_jacobian` | ✅ |
| `center_of_mass` | `pino_center_of_mass` | ✅ |
| `kinetic_energy` | `pino_energy` | ✅ |
| `potential_energy` | — | ⚠️ 合并在 `pino_energy` 中 |
| `compute_all_terms` | `pino_compute_all_terms` | ✅ |
| `constrained_aba_locked_joints` | `pino_constrained_aba_locked_joints` | ✅ |
| `constrained_aba_locked_joints_batch` | `pino_constrained_aba_locked_joints_batch` | ✅ |
| `constrained_forward_dynamics_contacts` | `pino_contact_constrained_dynamics` | ✅ |
| `*_batch` (法向) | `pino_contact_constrained_dynamics_batch` | ✅ |
| `apply_contact_impulses` | `pino_apply_contact_impulse` | ✅ |
| `*_batch` (法向冲量) | `pino_apply_contact_impulse_batch` | ✅ |
| `contact_jacobian_normal` | `pino_contact_jacobian_normal` | ✅ |
| `*_friction` (摩擦接触) | `pino_contact_constrained_dynamics_friction` | ✅ |
| `*_friction_batch` | `pino_contact_constrained_dynamics_friction_batch` | ✅ |
| `apply_contact_impulses_friction` | `pino_apply_contact_impulse_friction` | ✅ |
| `*_friction_batch` (冲量) | `pino_apply_contact_impulse_friction_batch` | ✅ |
| `forward_kinematics_poses` | `pino_forward_kinematics_poses` | ✅ |
| `forward_kinematics_poses_batch` | `pino_forward_kinematics_poses_batch` | ✅ |
| `rollout_aba_euler` | `pino_rollout_aba_euler` | ✅ |
| `centroidal_map` | `pino_centroidal_map` | ✅ |
| `centroidal_map_derivatives` | `pino_centroidal_map_derivatives` | ✅ |
| `centroidal_momentum` | `pino_centroidal_momentum` | ✅ |
| `centroidal_momentum_rate` | `pino_centroidal_momentum_rate` | ✅ |
| `centroidal_full_terms` | `pino_centroidal_full_terms` | ✅ |
| `centroidal_full_terms_with_contacts` | `pino_centroidal_full_terms_with_contacts` | ✅ |
| `rnea_second_order_derivatives` | `pino_rnea_second_order_derivatives` | ✅ |
| `constrained_dynamics_derivatives_locked_joints` | `pino_constrained_dynamics_derivatives_locked_joints` | ✅ |
| `impulse_dynamics_derivatives` | `pino_impulse_dynamics_derivatives` | ✅ |
| `inverse_dynamics_regressor` | `pino_inverse_dynamics_regressor` | ✅ |
| `inverse_dynamics_regressor_batch` | `pino_inverse_dynamics_regressor_batch` | ✅ |
| `kinetic_energy_regressor` | `pino_kinetic_energy_regressor` | ✅ |
| `potential_energy_regressor` | `pino_potential_energy_regressor` | ✅ |
| `center_of_mass_regressor` | `pino_center_of_mass_regressor` | ✅ |
| `select_independent_regressor_columns` | `pino_regressor_select_independent_columns` | ✅ |
| 碰撞: `collision_model_create_geometries` | `pino_collision_model_create_geometries` | ✅ |
| 碰撞: `minimum_distance` | `pino_collision_min_distance` | ✅ |
| 碰撞: `minimum_distance_detailed` | `pino_collision_min_distance_detailed` | ✅ |
| 碰撞: `minimum_distance_batch` | `pino_collision_min_distance_batch` | ✅ |
| 碰撞: `minimum_distance_detailed_batch` | `pino_collision_min_distance_detailed_batch` | ✅ |
| 碰撞: `collision_query_details` | `pino_collision_query_details` | ✅ |
| 模型创建 (代码构建) | `pino_model_create` | ✅ |
| 模型创建 (JSON) | `pino_model_create_from_json` | ✅ |
| 模型创建 (URDF) | `pino_model_create_from_urdf` | ✅ |
| 模型创建 (SDF) | `pino_model_create_from_sdf` | ✅ |
| 模型创建 (MJCF) | `pino_model_create_from_mjcf` | ✅ |
| 模型导出 (JSON) | `pino_model_to_json` | ✅ |
| 模型导出 (URDF) | `pino_model_to_urdf` | ✅ |
| 模型导出 (SDF) | `pino_model_to_sdf` | ✅ |
| 模型导出 (MJCF) | `pino_model_to_mjcf` | ✅ |

### FFI 未导出的内核函数

| 内核函数 | 说明 | 严重程度 |
|----------|------|----------|
| `forward_kinematics` | 单独 FK，被 `compute_all_terms` 覆盖 | 低 |
| `bias_forces` (单次) | 只有 batch 版 | 低 |
| `integrate_configuration` | 位形积分 | 中 |
| `difference_configuration` | 位形差分 | 中 |
| `interpolate_configuration` | 位形插值 | 中 |
| `random_configuration` | 随机位形 | 低 |
| `rnea_derivatives` | 一阶导数 | 中 |
| `aba_derivatives` | 一阶导数 | 中 |
| `kinematics_derivatives` | 运动学导数 | 中 |
| `frame_jacobian_derivatives` | 雅可比导数 | 中 |
| `center_of_mass_derivatives` | 质心导数 | 中 |
| `centroidal_derivatives` | 质心导数 | 中 |
| `centroidal_contact_wrench` | 质心接触力 | 低 |
| `build_delassus_matrix` | Delassus 矩阵 | 中 |
| `build_contact_problem` | 接触问题构建 | 中 |
| `solve_contact_*` (3个求解器) | 求解器 | 中 |

---

## 三、JS SDK 覆盖度 (js/pinocchio_wasm.mjs)

### 已包装 (6 个)

| JS 函数 | 对应 FFI |
|---------|----------|
| `createModelFromJson` | `pino_model_create_from_json` |
| `newWorkspace` | `pino_workspace_new` |
| `aba` | `pino_aba` |
| `disposeModel` | `pino_model_free` |
| `disposeWorkspace` | `pino_workspace_free` |
| 内存辅助函数 | `pino_alloc` / `pino_dealloc` |

### 未包装 (60 个 FFI 函数缺少 JS 包装)

#### 高优先级（核心动力学）
| FFI 函数 | 说明 |
|----------|------|
| `pino_rnea` | 逆动力学 |
| `pino_crba` | 质量矩阵 |
| `pino_frame_jacobian` | 雅可比 |
| `pino_center_of_mass` | 质心 |
| `pino_energy` | 动能+势能 |
| `pino_compute_all_terms` | 综合计算 |
| `pino_gravity_torques` | 重力项 |
| `pino_coriolis_torques` | 科氏力 |
| `pino_forward_kinematics_poses` | FK 位姿输出 |

#### 高优先级（模型加载）
| FFI 函数 | 说明 |
|----------|------|
| `pino_model_create_from_urdf` | URDF 加载 |
| `pino_model_create_from_sdf` | SDF 加载 |
| `pino_model_create_from_mjcf` | MJCF 加载 |
| `pino_model_create` | 代码构建模型 |
| `pino_model_to_json/urdf/sdf/mjcf` | 模型导出 (4个) |

#### 中优先级（批量接口）
| FFI 函数 | 说明 |
|----------|------|
| `pino_rnea_batch` | RNEA 批量 |
| `pino_aba_batch` | ABA 批量 |
| `pino_crba_batch` | CRBA 批量 |
| `pino_bias_forces_batch` | 偏置力批量 |
| `pino_gravity_torques_batch` | 重力项批量 |
| `pino_rollout_aba_euler` | 轨迹积分 |
| `pino_forward_kinematics_poses_batch` | FK 位姿批量 |

#### 中优先级（接触动力学）
| FFI 函数 | 说明 |
|----------|------|
| `pino_contact_constrained_dynamics` | 接触正动力学 |
| `pino_contact_constrained_dynamics_batch` | 接触正动力学批量 |
| `pino_apply_contact_impulse` | 接触冲量 |
| `pino_contact_jacobian_normal` | 接触雅可比 |
| `pino_contact_constrained_dynamics_friction` | 摩擦接触 |
| `pino_constrained_aba_locked_joints` | 关节锁定 |

#### 中优先级（质心量）
| FFI 函数 | 说明 |
|----------|------|
| `pino_centroidal_map` | 质心映射 |
| `pino_centroidal_momentum` | 质心动量 |
| `pino_centroidal_momentum_rate` | 质心动量率 |
| `pino_centroidal_full_terms` | 质心全量 |

#### 低优先级（导数与回归）
| FFI 函数 | 说明 |
|----------|------|
| `pino_rnea_second_order_derivatives` | 二阶导数 |
| `pino_inverse_dynamics_regressor` | 逆动力学回归 |
| `pino_kinetic_energy_regressor` | 动能回归 |
| `pino_potential_energy_regressor` | 势能回归 |
| `pino_center_of_mass_regressor` | 质心回归 |
| `pino_regressor_select_independent_columns` | 独立列选择 |
| `pino_constrained_dynamics_derivatives_locked_joints` | 约束导数 |
| `pino_impulse_dynamics_derivatives` | 冲量导数 |
| `pino_centroidal_map_derivatives` | 质心导数 |

#### 低优先级（碰撞）
| FFI 函数 | 说明 |
|----------|------|
| `pino_collision_model_create` | 碰撞模型 |
| `pino_collision_model_create_geometries` | 碰撞几何体 |
| `pino_collision_model_free` | 碰撞模型释放 |
| `pino_collision_min_distance` | 最小距离 |
| `pino_collision_min_distance_detailed` | 最小距离详细 |
| `pino_collision_min_distance_batch` | 最小距离批量 |
| `pino_collision_min_distance_detailed_batch` | 最小距离详细批量 |
| `pino_collision_query_details` | 碰撞详情 |

---

## 四、C Header 覆盖度 (include/pinocchio_wasm.h)

已声明: ~52 个函数
缺失: **14 个** FFI 导出函数未在头文件中声明

| 缺失函数 | 说明 |
|----------|------|
| `pino_model_create` | 代码构建模型 |
| `pino_gravity_torques` | 重力项 |
| `pino_coriolis_torques` | 科氏力 |
| `pino_rnea_batch` | RNEA 批量 |
| `pino_bias_forces_batch` | 偏置力批量 |
| `pino_gravity_torques_batch` | 重力项批量 |
| `pino_crba_batch` | CRBA 批量 |
| `pino_aba_batch` | ABA 批量 |
| `pino_constrained_aba_locked_joints` | 关节锁定 |
| `pino_constrained_aba_locked_joints_batch` | 关节锁定批量 |
| `pino_forward_kinematics_poses` | FK 位姿 |
| `pino_forward_kinematics_poses_batch` | FK 位姿批量 |
| `pino_rollout_aba_euler` | 轨迹积分 |
| `pino_collision_model_create_geometries` | 碰撞几何体构建 |

---

## 五、已知内核质量问题

| # | 问题 | 严重程度 | 位置 |
|---|------|----------|------|
| 1 | ABA 是 O(n³) 而非 O(n) | 中 | algo/mod.rs:233-258 |
| 2 | 所有导数用有限差分，非解析 | 中 | algo/mod.rs 多处 |
| 3 | 回归矩阵每次调用克隆 20n 个 model | 高 | algo/mod.rs:2402+ |
| 4 | 碰撞 narrowphase 全退化球体 | 中 | collision/mod.rs:321 |
| 5 | AABB broadphase 结果未使用 | 中 | collision/mod.rs:374 |
| 6 | 仅支持 revolute/continuous/hinge 关节 | 中 | model/mod.rs:468,553,775 |
| 7 | 不支持 prismatic/fixed/floating-base 关节 | 中 | — |
| 8 | autodiff/codegen/visualization 模块无测试 | 低 | — |
| 9 | Python SDK 未正确设置 restype | 高 | bindings/python/pinocchio_wasm.py |

---

## 六、统计总览

| 维度 | 数量 | 说明 |
|------|------|------|
| 内核算法函数 | 78 | 全部实现，无 stub |
| FFI 导出函数 | 66 | 覆盖 ~85% 内核 |
| FFI 未覆盖内核函数 | ~16 | 主要是求解器和部分导数函数 |
| C Header 声明 | ~52 | 缺 14 个 |
| JS SDK 包装 | 6 | 仅覆盖 ~9% FFI |
| JS SDK 未包装 | 60 | 需要补全 |
| 测试文件 | 32 | 覆盖主要功能 |

---

## 七、建议开发优先级

### Phase 1: JS SDK 核心 (浏览器可用的最小集)
1. 补全 JS SDK: `rnea`, `crba`, `frame_jacobian`, `center_of_mass`, `energy`, `compute_all_terms`
2. 补全 JS SDK: `model_create_from_urdf/sdf/mjcf`, `model_to_json`
3. 修复 C Header 缺失的 14 个声明

### Phase 2: JS SDK 扩展
4. 补全 JS SDK: batch 系列函数
5. 补全 JS SDK: 接触动力学 + 碰撞检测
6. 补全 JS SDK: 质心量 + 回归矩阵

### Phase 3: 内核质量提升
7. ABA 改为 O(n) 标准算法
8. 解析导数替代有限差分
9. 碰撞 narrowphase 实现真正的几何对检测
10. AABB broadphase 结果实际使用
11. 支持 prismatic/fixed/floating-base 关节
