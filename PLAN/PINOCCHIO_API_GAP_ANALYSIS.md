# Pinocchio vs Pinocchio-WASM 接口差异分析

> 基准: `pinocchio` (C++ 原版) | 对比: `pinocchio-wasm` (Rust/WASM 移植版)
> 生成日期: 2026-04-24
> 状态: ✅ 已实现 | ❌ 未实现 | ⚠️ 部分实现(有差异)

---

## 目录

1. [核心数据类型](#1-核心数据类型)
2. [空间代数 (Spatial Algebra)](#2-空间代数-spatial-algebra)
3. [关节类型 (Joint Types)](#3-关节类型-joint-types)
4. [模型与数据 (Model / Data)](#4-模型与数据-model--data)
5. [正向运动学 (Forward Kinematics)](#5-正向运动学)
6. [逆向动力学 RNEA](#6-逆向动力学-rnea)
7. [正向动力学 ABA](#7-正向动力学-aba)
8. [质量矩阵 CRBA](#8-质量矩阵-crba)
9. [雅可比矩阵 (Jacobians)](#9-雅可比矩阵)
10. [质心与能量 (CoM & Energy)](#10-质心与能量)
11. [质心动量 (Centroidal)](#11-质心动量-centroidal)
12. [接触动力学 (Contact Dynamics)](#12-接触动力学)
13. [约束动力学 (Constrained Dynamics)](#13-约束动力学)
14. [冲量动力学 (Impulse Dynamics)](#14-冲量动力学)
15. [一阶导数 (First-Order Derivatives)](#15-一阶导数)
16. [二阶导数 (Second-Order Derivatives)](#16-二阶导数)
17. [回归矩阵 (Regressors)](#17-回归矩阵-regressors)
18. [构型空间操作 (Configuration Space)](#18-构型空间操作)
19. [碰撞检测 (Collision)](#19-碰撞检测)
20. [解析器 (Parsers)](#20-解析器-parsers)
21. [帧算法 (Frame Algorithms)](#21-帧算法-frame-algorithms)
22. [Cholesky 分解](#22-cholesky-分解)
23. [约束求解器 (Constraint Solvers)](#23-约束求解器)
24. [模型操作 (Model Manipulation)](#24-模型操作)
25. [序列化 (Serialization)](#25-序列化)
26. [自动微分与代码生成](#26-自动微分与代码生成)
27. [可视化 (Visualization)](#27-可视化)
28. [并行计算 (Parallel)](#28-并行计算)
29. [FFI / 绑定 (Bindings)](#29-ffi--绑定)
30. [总览统计](#30-总览统计)

---

## 1. 核心数据类型

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 1.1 | `ModelTpl<Scalar>` 模板化模型 | ⚠️ | WASM 用 `Model` struct, 无模板, 仅 f64 |
| 1.2 | `DataTpl<Scalar>` 算法数据(100+字段) | ⚠️ | WASM 用 `Workspace`, 字段大幅精简 |
| 1.3 | `FrameTpl` 帧对象 | ❌ | 无 Frame 概念, 仅 Link |
| 1.4 | `GeometryModel` 几何模型 | ⚠️ | 有 `CollisionModel`, 但功能受限 |
| 1.5 | `GeometryData` 几何数据 | ⚠️ | 有碰撞数据, 但无 hpp-fcl 集成 |
| 1.6 | `GeometryObject` 几何体对象 | ⚠️ | 仅基础几何体(Sphere/Box/Capsule/Cylinder), 无 Mesh |
| 1.7 | `JointCollectionDefaultTpl` 关节集合 | ❌ | 无 variant-based 关节集合 |
| 1.8 | `ProximalSettingsTpl` 求解器参数 | ❌ | 无独立求解器设置对象 |

---

## 2. 空间代数 (Spatial Algebra)

### 2.1 基本类型

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 2.1.1 | `SE3Tpl` 刚体变换 | ⚠️ | 有 `Transform` (Mat3+Vec3), 缺少 toHomogeneousMatrix, toActionMatrix, toDualActionMatrix |
| 2.1.2 | `MotionTpl` 空间速度(6D) | ❌ | 无 Motion 类型, 用分开的 Vec3(angular/linear) |
| 2.1.3 | `ForceTpl` 空间力(6D) | ❌ | 无 Force 类型 |
| 2.1.4 | `InertiaTpl` 刚体惯性(6x6) | ❌ | 无 Inertia 类型, 惯性用 Mat3 表示 |
| 2.1.5 | `Symmetric3Tpl` 对称3x3矩阵 | ❌ | 无 |
| 2.1.6 | `PseudoInertiaTpl` 伪惯性参数化 | ❌ | 无 |
| 2.1.7 | `LogCholeskyParametersTpl` | ❌ | 无 |

### 2.2 SE3 操作

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 2.2.1 | `SE3::Identity()` | ✅ | `Transform::identity()` |
| 2.2.2 | `SE3::Random()` | ❌ | |
| 2.2.3 | `SE3::inverse()` | ✅ | `Transform::inverse()` |
| 2.2.4 | `SE3::rotation()` / `translation()` | ✅ | `Transform.rotation` / `Transform.translation` |
| 2.2.5 | `SE3::toHomogeneousMatrix()` | ❌ | |
| 2.2.6 | `SE3::toActionMatrix()` / `toActionMatrixInverse()` | ❌ | |
| 2.2.7 | `SE3::toDualActionMatrix()` | ❌ | |
| 2.2.8 | `SE3::act(v)` 作用于 Vector3 | ✅ | `Transform::transform_point(p)` |
| 2.2.9 | `SE3::act(SE3)` / `actInv(SE3)` | ⚠️ | 有 `multiply()`, 无 `actInv(SE3)` |
| 2.2.10 | `SE3::act(Motion)` / `act(Force)` / `act(Inertia)` | ❌ | 无 Motion/Force/Inertia 类型 |
| 2.2.11 | `SE3::isApprox()` / `isIdentity()` / `isNormalized()` | ❌ | |
| 2.2.12 | `SE3::normalize()` | ❌ | |
| 2.2.13 | `SE3::Interpolate(A, B, alpha)` SLERP | ❌ | |
| 2.2.14 | `SE3::cast<NewScalar>()` 标量转换 | ❌ | 无模板 |
| 2.2.15 | 从四元数构造 | ❌ | 无四元数支持 |

### 2.3 线性代数工具

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 2.3.1 | `skew(v)` 反对称矩阵 | ✅ | `Mat3::skew(v)` |
| 2.3.2 | `addSkew(v, M)` | ❌ | |
| 2.3.3 | `unSkew(M)` | ❌ | |
| 2.3.4 | `skewSquare(u, v)` | ❌ | |
| 2.3.5 | `alphaSkew(alpha, v)` | ❌ | |
| 2.3.6 | `Mat3::outer(a, b)` 外积 | ✅ | `Mat3::outer(a, b)` |
| 2.3.7 | `Mat3::from_axis_angle()` | ✅ | Rodrigues 公式 |
| 2.3.8 | 四元数工具 (`uniformRandom`, `assignQuaternion`) | ❌ | |
| 2.3.9 | RPY 转换 (`rpyToMatrix`, `matrixToRpy`, `rotate`) | ❌ | |
| 2.3.10 | 旋转投影 (`orthogonalProjection`) | ❌ | |

### 2.4 Exp/Log 映射

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 2.4.1 | `exp3(v)` so3→SO3 | ❌ | |
| 2.4.2 | `log3(R)` SO3→so3 | ❌ | |
| 2.4.3 | `Jexp3(v, J)` exp3导数 | ❌ | |
| 2.4.4 | `exp6(v)` se3→SE3 | ❌ | |
| 2.4.5 | `log6(M)` SE3→se3 | ❌ | |
| 2.4.6 | `Jexp6(v, J)` exp6导数 | ❌ | |
| 2.4.7 | `exp3quat(v)` 返回四元数 | ❌ | |

---

## 3. 关节类型 (Joint Types)

| # | 原版关节 | nq/nv | WASM 状态 | 备注 |
|---|---------|-------|-----------|------|
| 3.1 | `JointModelRevoluteTpl<X/Y/Z>` 绕轴旋转 | 1/1 | ⚠️ | 支持, 但仅通用轴, 非特化X/Y/Z |
| 3.2 | `JointModelRevoluteUnalignedTpl` 任意轴旋转 | 1/1 | ✅ | `Joint::revolute(axis, origin)` |
| 3.3 | `JointModelRevoluteUnboundedTpl<X/Y/Z>` 连续旋转(cos,sin) | 2/1 | ❌ | |
| 3.4 | `JointModelRevoluteUnboundedUnalignedTpl` | 2/1 | ❌ | |
| 3.5 | `JointModelPrismaticTpl<X/Y/Z>` 沿轴平移 | 1/1 | ⚠️ | 支持通用轴，未特化 X/Y/Z |
| 3.6 | `JointModelPrismaticUnalignedTpl` 任意轴平移 | 1/1 | ✅ | `Joint::prismatic(axis, origin)` |
| 3.7 | `JointModelSphericalTpl` 球关节(四元数) | 4/3 | ✅ | `Joint::spherical(origin)`，nq/nv 分离 |
| 3.8 | `JointModelSphericalZYXTpl` 球关节(Euler ZYX) | 3/3 | ❌ | |
| 3.9 | `JointModelEllipsoidTpl` 椭球关节 | 4/3 | ❌ | |
| 3.10 | `JointModelTranslationTpl` 3D平移 | 3/3 | ❌ | |
| 3.11 | `JointModelFreeFlyerTpl` 6自由度浮动基(四元数) | 7/6 | ✅ | `Joint::freeflyer(origin)`，nq=7/nv=6 |
| 3.12 | `JointModelPlanarTpl` 平面(2D平移+1旋转) | 3/3 | ❌ | |
| 3.13 | `JointModelCompositeTpl` 复合关节 | 变化 | ❌ | |
| 3.14 | `JointModelMimicTpl` 模仿关节 | 变化 | ❌ | |
| 3.15 | `JointModelHelicalTpl<X/Y/Z>` 螺旋关节 | 1/1 | ❌ | |
| 3.16 | `JointModelHelicalUnalignedTpl` 任意轴螺旋 | 1/1 | ❌ | |
| 3.17 | `JointModelUniversalTpl` 万向节(2旋转) | 2/2 | ❌ | |
| 3.18 | Fixed Joint 固定关节 | 0/0 | ✅ | `Joint::fixed(origin)` |
| 3.19 | Joint 基类方法 (`id`, `idx_q`, `idx_v`, `nq`, `nv`, `calc`) | - | ⚠️ | 有基础方法, 不完全 |

---

## 4. 模型与数据 (Model / Data)

### 4.1 Model 构建与字段

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 4.1.1 | `Model.nq` 配置维度 | ✅ | `Model::nq()` |
| 4.1.2 | `Model.nv` 速度维度 | ✅ | `Model::nv()` |
| 4.1.3 | `Model.njoints` 关节数 | ✅ | `Model::nlinks()` (link 数) |
| 4.1.4 | `Model.nframes` 帧数 | ❌ | 无 Frame 系统 |
| 4.1.5 | `Model.inertias` 空间惯性数组 | ⚠️ | 用 link 的 `inertia_local_com: Mat3` |
| 4.1.6 | `Model.jointPlacements` 关节位姿 | ✅ | `Joint.origin` |
| 4.1.7 | `Model.joints` 关节模型数组 | ✅ | `Link.joint` |
| 4.1.8 | `Model.parents` 父关节索引 | ✅ | `Link.parent` |
| 4.1.9 | `Model.children` 子关节索引 | ✅ | `Model::children_of()` |
| 4.1.10 | `Model.names` 关节名 | ⚠️ | `Link.name` (link名, 非关节名) |
| 4.1.11 | `Model.armature` 转子惯量补偿 | ❌ | |
| 4.1.12 | `Model.rotorInertia` / `rotorGearRatio` | ❌ | |
| 4.1.13 | `Model.friction/damping` 摩擦阻尼 | ❌ | |
| 4.1.14 | `Model.effortLimit` 力矩限制 | ❌ | |
| 4.1.15 | `Model.velocityLimit` 速度限制 | ❌ | |
| 4.1.16 | `Model.lowerPositionLimit` / `upperPositionLimit` | ❌ | |
| 4.1.17 | `Model.referenceConfigurations` 参考配置 | ❌ | |
| 4.1.18 | `Model.gravity` 重力向量 | ⚠️ | 作为函数参数传入, 非模型字段 |
| 4.1.19 | `Model.frames` 帧数组 | ❌ | |
| 4.1.20 | `Model.supports` / `subtrees` | ❌ | |
| 4.1.21 | `Model.sparsityPattern` 雅可比稀疏模式 | ❌ | |

### 4.2 Model 方法

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 4.2.1 | `addJoint()` 添加关节 | ⚠️ | 通过 `Model::new(links)` 批量构建 |
| 4.2.2 | `addJointFrame()` 添加关节帧 | ❌ | |
| 4.2.3 | `appendBodyToJoint()` 添加体惯量 | ⚠️ | 通过 Link 构造时指定 |
| 4.2.4 | `addBodyFrame()` 添加体帧 | ❌ | |
| 4.2.5 | `addFrame()` 添加操作帧 | ❌ | |
| 4.2.6 | `getBodyId()` / `getJointId()` / `getFrameId()` | ⚠️ | 无按名查找, 仅有索引访问 |
| 4.2.7 | `existBodyName()` / `existJointName()` / `existFrame()` | ❌ | |
| 4.2.8 | `createData()` 创建数据 | ✅ | `Workspace::new(model)` |
| 4.2.9 | `check()` 模型校验 | ✅ | `check_state_dims()` |
| 4.2.10 | `hasConfigurationLimit()` | ❌ | |
| 4.2.11 | `cast<NewScalar>()` 标量转换 | ❌ | |
| 4.2.12 | `getChildJoints()` 叶关节 | ❌ | |

### 4.3 Data / Workspace 字段

| # | 原版 Data 字段 | WASM 状态 | 备注 |
|---|---------------|-----------|------|
| 4.3.1 | `oMi` 绝对关节位姿 | ✅ | `Workspace::world_pose` |
| 4.3.2 | `liMi` 相对关节位姿 | ❌ | |
| 4.3.3 | `v` / `ov` 关节速度 | ⚠️ | `omega`, `vel_origin` |
| 4.3.4 | `a` / `oa` 关节加速度 | ⚠️ | `alpha`, `acc_origin` |
| 4.3.5 | `f` / `of` 体力 | ✅ | `force`, `torque` |
| 4.3.6 | `tau` 关节力矩 | ✅ | 算法返回值 |
| 4.3.7 | `nle` 非线性效应(Coriolis+重力) | ✅ | `bias_forces()` 返回 |
| 4.3.8 | `g` 广义重力 | ✅ | `gravity_torques()` 返回 |
| 4.3.9 | `oMf` 帧位姿 | ❌ | 无 Frame 系统 |
| 4.3.10 | `Ycrb` 复合刚体惯性 | ❌ | |
| 4.3.11 | `M` 质量矩阵 | ✅ | `crba()` 返回 |
| 4.3.12 | `Minv` 逆质量矩阵 | ❌ | |
| 4.3.13 | `C` Coriolis矩阵 | ✅ | `coriolis_torques()` (向量形式) |
| 4.3.14 | `J` / `dJ` / `ddJ` 雅可比及导数 | ⚠️ | 有 `frame_jacobian()`, 无 dJ/ddJ |
| 4.3.15 | `Ag` / `dAg` 质心动量矩阵 | ⚠️ | `centroidal_map()` 返回 |
| 4.3.16 | `hg` / `dhg` 质心动量及变化率 | ⚠️ | `centroidal_momentum()` / `centroidal_momentum_rate()` |
| 4.3.17 | `Ig` 质心复合刚体惯性 | ❌ | |
| 4.3.18 | `com` / `vcom` / `acom` 质心系列 | ⚠️ | 有 `center_of_mass()`, 无 vcom/acom |
| 4.3.19 | `mass` 子树质量 | ❌ | |
| 4.3.20 | `Jcom` 质心雅可比 | ❌ | |
| 4.3.21 | `kinetic_energy` / `potential_energy` | ✅ | 两个独立函数 |
| 4.3.22 | `ddq` ABA输出加速度 | ✅ | `aba()` 返回 |
| 4.3.23 | `dtau_dq` / `dtau_dv` RNEA导数 | ⚠️ | 有限差分近似 |
| 4.3.24 | `ddq_dq` / `ddq_dv` / `ddq_dtau` ABA导数 | ⚠️ | 有限差分近似 |
| 4.3.25 | `lambda_c` 拉格朗日乘子 | ⚠️ | 接触结果中包含 |
| 4.3.26 | `JMinvJt` 操作空间惯性 | ❌ | |
| 4.3.27 | `staticRegressor` 静态回归 | ❌ | |
| 4.3.28 | `bodyRegressor` 体回归 | ❌ | |
| 4.3.29 | `jointTorqueRegressor` 关节力矩回归 | ⚠️ | `inverse_dynamics_regressor()` |
| 4.3.30 | `kinematic_hessians` 运动学Hessian | ❌ | |

---

## 5. 正向运动学

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 5.1 | `forwardKinematics(model, data, q)` 仅位姿 | ✅ | `forward_kinematics_poses()` |
| 5.2 | `forwardKinematics(model, data, q, v)` 位姿+速度 | ✅ | `forward_kinematics()` 含 qd |
| 5.3 | `forwardKinematics(model, data, q, v, a)` 位姿+速度+加速度 | ✅ | `forward_kinematics()` 含 qd, qdd |
| 5.4 | `updateGlobalPlacements(model, data)` 更新全局位姿 | ⚠️ | FK 内部完成 |
| 5.5 | `framesForwardKinematics(model, data, q)` 帧FK | ❌ | 无 Frame 系统 |
| 5.6 | 批量 FK | ✅ | `forward_kinematics_poses_batch()` |

---

## 6. 逆向动力学 RNEA

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 6.1 | `rnea(model, data, q, v, a)` → τ | ✅ | `rnea()` |
| 6.2 | 批量 RNEA | ✅ | `rnea_batch()` |
| 6.3 | RNEA 导数 (`computeRNEADerivatives`) | ⚠️ | 有限差分 `rnea_derivatives()` |
| 6.4 | 重力导数 (`computeGeneralizedGravityDerivative`) | ❌ | |
| 6.5 | 静力矩导数 (`computeStaticTorqueDerivative`) | ❌ | |
| 6.6 | Coriolis 矩阵 (`computeCoriolisMatrix`) | ⚠️ | `coriolis_torques()` 仅向量, 非矩阵 |
| 6.7 | M逆 (`computeMinverse`) | ❌ | |
| 6.8 | 二阶导数 | ⚠️ | `rnea_second_order_derivatives()` (有限差分) |

---

## 7. 正向动力学 ABA

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 7.1 | `aba(model, data, q, v, tau)` → q̈ | ⚠️ | 已实现, 但用 CRBA+Cholesky 而非 O(n) 递归 ABA |
| 7.2 | 批量 ABA | ✅ | `aba_batch()` |
| 7.3 | ABA 导数 (`computeABADerivatives`) | ⚠️ | 有限差分 `aba_derivatives()` |

---

## 8. 质量矩阵 CRBA

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 8.1 | `crba(model, data, q)` → M | ⚠️ | 已实现, 但用 RNEA 逐列构建, 非递归 CRBA |
| 8.2 | 批量 CRBA | ✅ | `crba_batch()` |

---

## 9. 雅可比矩阵

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 9.1 | `computeJointJacobians(model, data, q)` 全雅可比 | ❌ | |
| 9.2 | `getJointJacobian(model, data, joint_id, rf, J)` | ⚠️ | `frame_jacobian()` WORLD 帧下 |
| 9.3 | `computeJointJacobian(model, data, q, joint_id, J)` | ✅ | `frame_jacobian()` |
| 9.4 | `getJacobians(model, data, rf, J)` 全身雅可比 | ❌ | |
| 9.5 | 参考坐标系选择 (WORLD/LOCAL/LOCAL_WORLD_ALIGNED) | ❌ | 仅 WORLD |
| 9.6 | 接触法向雅可比 | ✅ | `contact_jacobian_normal()` |
| 9.7 | 质心雅可比 (`jacobianCenterOfMass`) | ❌ | |
| 9.8 | 雅可比时间导数 (`dJ`, `ddJ`) | ❌ | |
| 9.9 | 运动学导数 (`getJointVelocityDerivatives`, `getJointAccelerationDerivatives`) | ⚠️ | 有限差分 `kinematics_derivatives()` |
| 9.10 | 帧雅可比 (`getFrameJacobian`) | ⚠️ | `frame_jacobian()` 但无 Frame 系统 |
| 9.11 | 帧雅可比导数 | ⚠️ | `frame_jacobian_derivatives()` (有限差分) |

---

## 10. 质心与能量

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 10.1 | `centerOfMass(model, data, q)` → CoM位置 | ✅ | `center_of_mass()` |
| 10.2 | `centerOfMass(q, v)` → 含CoM速度 | ❌ | |
| 10.3 | `centerOfMass(q, v, a)` → 含CoM加速度 | ❌ | |
| 10.4 | `jacobianCenterOfMass(model, data, q)` → Jcom | ❌ | |
| 10.5 | `computeTotalMass(model)` | ❌ | |
| 10.6 | `computeSubtreeMasses(model, data)` | ❌ | |
| 10.7 | CoM 速度导数 (`getCenterOfMassVelocityDerivatives`) | ⚠️ | `center_of_mass_derivatives()` (有限差分) |
| 10.8 | `computeKineticEnergy(model, data)` | ✅ | `kinetic_energy()` |
| 10.9 | `computePotentialEnergy(model, data)` | ✅ | `potential_energy()` |
| 10.10 | `computeMechanicalEnergy(model, data)` | ❌ | |
| 10.11 | `computeAllTerms(model, data, q, v)` | ✅ | `compute_all_terms()` |

---

## 11. 质心动量 (Centroidal)

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 11.1 | `computeCentroidalMomentum(model, data)` → hg | ✅ | `centroidal_momentum()` |
| 11.2 | `computeCentroidalMomentumTimeVariation(model, data)` → dhg | ⚠️ | `centroidal_momentum_rate()` (有限差分) |
| 11.3 | `ccoRNEA(model, data, q, v, a)` | ❌ | |
| 11.4 | 质心动量矩阵 (`Ag`) | ✅ | `centroidal_map()` |
| 11.5 | 质心动量矩阵导数 (`dAg`) | ⚠️ | `centroidal_map_derivatives()` (有限差分) |
| 11.6 | `computeCentroidalDynamicsDerivatives` | ⚠️ | `centroidal_derivatives()` (有限差分) |
| 11.7 | 质心接触力矩 | ✅ | `centroidal_contact_wrench()` |
| 11.8 | 质心全量计算 | ✅ | `centroidal_full_terms()` |
| 11.9 | 含接触的质心全量 | ✅ | `centroidal_full_terms_with_contacts()` |

---

## 12. 接触动力学

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 12.1 | `forwardDynamics(model, data, tau, J, gamma)` 接触正向动力学 | ⚠️ | 有接触约束版本, 但接口不同 |
| 12.2 | 接触动力学(法向) | ✅ | `constrained_forward_dynamics_contacts()` PGS求解 |
| 12.3 | 接触动力学(含摩擦) | ✅ | `constrained_forward_dynamics_contacts_friction()` |
| 12.4 | 接触动力学批量(法向) | ✅ | `_batch()` 版本 |
| 12.5 | 接触动力学批量(含摩擦) | ✅ | `_batch()` 版本 |
| 12.6 | Delassus矩阵计算 | ✅ | `build_delassus_matrix()` |
| 12.7 | 接触问题构建 | ✅ | `build_contact_problem()` |
| 12.8 | 接触逆动力学 (`computeContactInverseDynamics`) | ❌ | |
| 12.9 | 接触雅可比(法向) | ✅ | `contact_jacobian_normal()` |

---

## 13. 约束动力学

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 13.1 | `initConstraintDynamics` 约束初始化 | ❌ | 无通用约束框架 |
| 13.2 | `constraintDynamics` 通用约束动力学 | ❌ | |
| 13.3 | `computeConstraintDynamicsDerivatives` | ❌ | |
| 13.4 | 锁关节约束 ABA | ✅ | `constrained_aba_locked_joints()` |
| 13.5 | 锁关节约束 ABA 批量 | ✅ | `_batch()` |
| 13.6 | 锁关节约束导数 | ⚠️ | `constrained_dynamics_derivatives_locked_joints()` (有限差分) |
| 13.7 | 闭环约束 ABA (`loopConstrainedABA`) | ❌ | |
| 13.8 | 刚性接触约束 (`RigidConstraintModel`) | ❌ | |
| 13.9 | Baumgarte 稳定化 | ❌ | |
| 13.10 | 摩擦锥 (`CoulombFrictionCone`) | ⚠️ | 内嵌在接触动力学中, 非独立类型 |
| 13.11 | 关节限制约束 (`JointLimitConstraint`) | ❌ | |
| 13.12 | 关节摩擦约束 (`JointFrictionConstraint`) | ❌ | |

---

## 14. 冲量动力学

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 14.1 | `impulseDynamics(model, data, v_before, J, r_coeff)` | ⚠️ | 有独立接口 |
| 14.2 | 冲量(法向) | ✅ | `apply_contact_impulses()` |
| 14.3 | 冲量(含摩擦) | ✅ | `apply_contact_impulses_friction()` |
| 14.4 | 冲量批量(法向) | ✅ | `_batch()` |
| 14.5 | 冲量批量(含摩擦) | ✅ | `_batch()` |
| 14.6 | 冲量动力学导数 | ⚠️ | `impulse_dynamics_derivatives()` (有限差分) |
| 14.7 | 冲量动力学二阶导数 | ❌ | |

---

## 15. 一阶导数

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 15.1 | `computeRNEADerivatives` (解析) | ⚠️ | `rnea_derivatives()` 有限差分 |
| 15.2 | `computeABADerivatives` (解析) | ⚠️ | `aba_derivatives()` 有限差分 |
| 15.3 | 运动学导数 | ⚠️ | `kinematics_derivatives()` 有限差分 |
| 15.4 | 帧雅可比导数 | ⚠️ | `frame_jacobian_derivatives()` 有限差分 |
| 15.5 | CoM 导数 | ⚠️ | `center_of_mass_derivatives()` 有限差分 |
| 15.6 | 约束动力学导数 | ⚠️ | `constrained_dynamics_derivatives_locked_joints()` 有限差分 |
| 15.7 | 冲量动力学导数 | ⚠️ | `impulse_dynamics_derivatives()` 有限差分 |

> **关键差异**: 原版所有导数都有 O(n) 解析递归算法实现; WASM 版全部使用有限差分数值近似, 精度和性能较低.

---

## 16. 二阶导数

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 16.1 | RNEA 二阶导数 (d²τ/dq², d²τ/dv², d²τ/dqdv) | ⚠️ | `rnea_second_order_derivatives()` 4点有限差分 |

---

## 17. 回归矩阵 (Regressors)

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 17.1 | `computeStaticRegressor` (3×4N) | ❌ | |
| 17.2 | `computeBodyRegressor` (6×10) | ❌ | |
| 17.3 | `computeJointKinematicRegressor` | ❌ | |
| 17.4 | `computeFrameKinematicRegressor` | ❌ | |
| 17.5 | `computeJointTorqueRegressor` (N×10N) | ⚠️ | `inverse_dynamics_regressor()` (有限差分) |
| 17.6 | `computeKineticEnergyRegressor` | ✅ | `kinetic_energy_regressor()` |
| 17.7 | `computePotentialEnergyRegressor` | ✅ | `potential_energy_regressor()` |
| 17.8 | CoM 回归 | ✅ | `center_of_mass_regressor()` |
| 17.9 | 回归列选择 (QR分解) | ✅ | `select_independent_regressor_columns()` |
| 17.10 | 批量逆动力学回归 | ✅ | `inverse_dynamics_regressor_batch()` |

---

## 18. 构型空间操作

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 18.1 | `integrate(model, q, v)` 李群积分 | ⚠️ | `integrate_configuration()` 仅 q+=v*dt, 无李群运算 |
| 18.2 | `interpolate(model, q0, q1, u)` 李群插值 | ⚠️ | `interpolate_configuration()` 仅线性插值 |
| 18.3 | `difference(model, q0, q1)` 李群差 | ⚠️ | `difference_configuration()` 仅 q1-q0 |
| 18.4 | `squaredDistance(model, q0, q1)` | ❌ | |
| 18.5 | `distance(model, q0, q1)` | ❌ | |
| 18.6 | `randomConfiguration(model, lower, upper)` | ✅ | `random_configuration()` LCG随机 |
| 18.7 | `neutral(model)` 中性配置 | ❌ | |
| 18.8 | `normalize(model, q)` 归一化(如四元数) | ❌ | |
| 18.9 | `isNormalized(model, q)` | ❌ | |
| 18.10 | 李群 dIntegrate / dDifference / Jintegrate / Jdifference | ❌ | 无李群代数 |
| 18.11 | 轨迹前向积分 | ✅ | `rollout_aba_euler()` Euler积分 |

---

## 19. 碰撞检测

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 19.1 | `computeCollision(geom_model, geom_data, pair_id)` | ✅ | `collision_details()` AABB+代理球 |
| 19.2 | `computeCollisions(..., stopAtFirstCollision)` | ❌ | 无提前终止 |
| 19.3 | `computeDistance(geom_model, geom_data, pair_id)` | ✅ | `minimum_distance()` |
| 19.4 | `computeDistances(...)` 全距离 | ✅ | `minimum_distance_detailed()` |
| 19.5 | `updateGeometryPlacements(model, data, geom_model, geom_data)` | ❌ | 内部完成 |
| 19.6 | 接触面片 (`computeContactPatch`) | ❌ | |
| 19.7 | BroadPhaseManager | ❌ | 无宽相位管理器 |
| 19.8 | TreeBroadPhaseManager | ❌ | |
| 19.9 | 几何体类型: Sphere | ✅ | |
| 19.10 | 几何体类型: Box | ✅ | |
| 19.11 | 几何体类型: Capsule | ✅ | |
| 19.12 | 几何体类型: Cylinder | ✅ | |
| 19.13 | 几何体类型: Mesh (三角网格) | ❌ | |
| 19.14 | 几何体类型: Convex | ❌ | |
| 19.15 | 几何体类型: Halfspace / Plane | ❌ | |
| 19.16 | GJK/EPA 精确窄相 | ❌ | 仅代理球近似 |
| 19.17 | hpp-fcl / coal 集成 | ❌ | |
| 19.18 | 碰撞对过滤 | ✅ | `PairFilter` (same_link, parent_child) |
| 19.19 | 安全边距 (`setSecurityMargins`) | ❌ | |
| 19.20 | 批量碰撞查询 | ✅ | `minimum_distance_batch()` / `minimum_distance_detailed_batch()` |
| 19.21 | 碰撞可视化材质 | ❌ | |

---

## 20. 解析器 (Parsers)

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 20.1 | URDF 模型解析 (`buildModel`) | ⚠️ | `from_urdf_str()` 仅解析运动学+惯性, 不解析几何 |
| 20.2 | URDF 几何解析 (`buildGeom`) | ❌ | |
| 20.3 | SDF 模型解析 | ⚠️ | `from_sdf_str()` 仅运动学+惯性 |
| 20.4 | SDF 几何解析 | ❌ | |
| 20.5 | MJCF 模型解析 | ⚠️ | `from_mjcf_str()` 仅 hinge 关节 |
| 20.6 | MJCF 几何解析 | ❌ | |
| 20.7 | SRDF 解析 (参考配置/碰撞对移除) | ❌ | |
| 20.8 | 从文件加载 | ❌ | 仅支持字符串输入 |
| 20.9 | 从 XML 字符串加载 | ✅ | 所有格式支持字符串 |
| 20.10 | 模型图解析基础设施 | ❌ | |
| 20.11 | JSON 导入 | ✅ | `from_json_str()` |
| 20.12 | JSON 导出 | ✅ | `to_json_string()` |
| 20.13 | URDF 导出 | ✅ | `to_urdf_string()` |
| 20.14 | SDF 导出 | ✅ | `to_sdf_string()` |
| 20.15 | MJCF 导出 | ✅ | `to_mjcf_string()` |

---

## 21. 帧算法 (Frame Algorithms)

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 21.1 | `updateFramePlacements(model, data)` | ❌ | 无 Frame 系统 |
| 21.2 | `updateFramePlacement(model, data, frame_id)` | ❌ | |
| 21.3 | `getFrameVelocity(model, data, frame_id, rf)` | ❌ | |
| 21.4 | `getFrameAcceleration(model, data, frame_id, rf)` | ❌ | |
| 21.5 | `getFrameClassicalAcceleration(model, data, frame_id, rf)` | ❌ | |
| 21.6 | `getFrameJacobian(model, data, frame_id, rf, J)` | ⚠️ | `frame_jacobian()` 但基于 link, 非 frame |
| 21.7 | `computeFrameJacobian(model, data, q, frame_id, rf, J)` | ⚠️ | 同上 |
| 21.8 | 帧加速度导数 | ❌ | |
| 21.9 | 帧速度导数 | ❌ | |

---

## 22. Cholesky 分解

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 22.1 | `cholesky::decompose(model, data)` → U | ⚠️ | 内部 `cholesky_solve()`, 不暴露 U |
| 22.2 | `cholesky::Uv(model, data, v)` | ❌ | |
| 22.3 | `cholesky::Utv(model, data, v)` | ❌ | |
| 22.4 | `cholesky::Uiv(model, data, v)` | ❌ | |
| 22.5 | `cholesky::Utiv(model, data, v)` | ❌ | |
| 22.6 | `cholesky::Mv(model, data, v, out)` | ❌ | |
| 22.7 | `cholesky::solve(model, data, y)` | ⚠️ | 内部使用 |

---

## 23. 约束求解器

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 23.1 | ADMM 求解器 | ✅ | `solve_contact_admm()` |
| 23.2 | PGS 求解器 | ✅ | `solve_contact_pgs()` |
| 23.3 | Cholesky 求解 | ✅ | `solve_contact_cholesky()` |
| 23.4 | Anderson 加速 | ❌ | |
| 23.5 | 对角预处理 (`DiagonalPreconditioner`) | ❌ | |
| 23.6 | Delassus 操作符(密集) | ❌ | |
| 23.7 | Delassus 操作符(稀疏) | ❌ | |
| 23.8 | Delassus 操作符(Cholesky) | ❌ | |
| 23.9 | Delassus 操作符(刚体) | ❌ | |
| 23.10 | Delassus 操作符(预处理) | ❌ | |

---

## 24. 模型操作

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 24.1 | `appendModel` 拼接两个模型 | ❌ | |
| 24.2 | `buildReducedModel` 构建简化模型 | ❌ | |
| 24.3 | `frameIdx` 查找或创建帧 | ❌ | |
| 24.4 | 示例模型: manipulator (6-DOF) | ❌ | |
| 24.5 | 示例模型: humanoid (28-DOF) | ❌ | |
| 24.6 | 示例模型: humanoidRandom | ❌ | |
| 24.7 | 示例几何: manipulatorGeometries | ❌ | |
| 24.8 | 示例几何: humanoidGeometries | ❌ | |
| 24.9 | 模型配置转换器 | ❌ | |

---

## 25. 序列化

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 25.1 | `saveToFile` / `loadFromFile` | ❌ | |
| 25.2 | `saveToString` / `loadFromString` | ❌ | |
| 25.3 | `saveToBinary` / `loadFromBinary` | ❌ | |
| 25.4 | `saveToXML` / `loadFromXML` | ❌ | |
| 25.5 | 所有类型的序列化器 | ❌ | 仅 JSON 导入导出 |

---

## 26. 自动微分与代码生成

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 26.1 | CasADi 自动微分支持 | ❌ | |
| 26.2 | CppAD 自动微分支持 | ❌ | |
| 26.3 | CppADCodeGen 代码生成 | ❌ | |
| 26.4 | 前向模式对偶数 (Dual Number) | ✅ | `autodiff::Dual` |
| 26.5 | 有限差分雅可比 | ✅ | `autodiff::jacobian_fd()` |
| 26.6 | C 头文件生成 | ✅ | `codegen::generate_c_header_summary()` |
| 26.7 | JS 加载器生成 | ✅ | `codegen::generate_js_loader()` |

---

## 27. 可视化

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 27.1 | `BaseVisualizer` 基类 | ❌ | |
| 27.2 | GepettoViewer | ❌ | |
| 27.3 | MeshcatViewer | ❌ | |
| 27.4 | ASCII 树可视化 | ✅ | `visualization::ascii_tree()` |

---

## 28. 并行计算

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 28.1 | 并行 ABA | ❌ | |
| 28.2 | 并行 RNEA | ❌ | |
| 28.3 | 并行碰撞检测 | ❌ | |
| 28.4 | Model/Geometry Pool | ❌ | |
| 28.5 | BroadPhase Pool | ❌ | |

---

## 29. FFI / 绑定

### 29.1 FFI (C ABI)

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 29.1.1 | C ABI FFI 导出 | ✅ | 55+ `extern "C"` 函数 |
| 29.1.2 | 不透明句柄 (Model/Workspace/Collision) | ✅ | |
| 29.1.3 | 状态码错误处理 | ✅ | |
| 29.1.4 | 内存管理 (`alloc`/`dealloc`) | ✅ | |

### 29.2 JavaScript SDK

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 29.2.1 | 模型创建 (from JSON) | ✅ | |
| 29.2.2 | Workspace 创建/释放 | ✅ | |
| 29.2.3 | ABA 调用 | ✅ | `aba()` |
| 29.2.4 | 其他所有算法 (50+) | ❌ | 需直接调用 WASM exports |
| 29.2.5 | 类型安全的包装 | ❌ | |

### 29.3 Python SDK

| # | 原版接口 | WASM 状态 | 备注 |
|---|---------|-----------|------|
| 29.3.1 | 模型创建 | ✅ | `model_from_json()` |
| 29.3.2 | Workspace 创建/释放 | ✅ | |
| 29.3.3 | ABA 调用 | ✅ | |
| 29.3.4 | 其他所有算法 | ❌ | 仅3个方法封装 |

---

## 30. 总览统计

### 30.1 实现状态汇总

| 类别 | 总项数 | ✅ 完成 | ⚠️ 部分完成 | ❌ 未实现 |
|------|--------|---------|-------------|----------|
| 核心数据类型 | 8 | 0 | 4 | 4 |
| 空间代数 | 34 | 5 | 2 | 27 |
| 关节类型 | 19 | 1 | 2 | 16 |
| 模型字段/方法 | 33 | 8 | 7 | 18 |
| Data/Workspace | 30 | 7 | 6 | 17 |
| 正向运动学 | 6 | 5 | 1 | 0 |
| RNEA | 8 | 3 | 2 | 3 |
| ABA | 3 | 1 | 2 | 0 |
| CRBA | 2 | 1 | 1 | 0 |
| 雅可比 | 11 | 2 | 4 | 5 |
| 质心与能量 | 11 | 4 | 1 | 6 |
| 质心动量 | 9 | 4 | 3 | 2 |
| 接触动力学 | 9 | 6 | 1 | 2 |
| 约束动力学 | 12 | 2 | 1 | 9 |
| 冲量动力学 | 7 | 4 | 1 | 2 |
| 一阶导数 | 7 | 0 | 7 | 0 |
| 二阶导数 | 1 | 0 | 1 | 0 |
| 回归矩阵 | 10 | 4 | 1 | 5 |
| 构型空间 | 11 | 1 | 3 | 7 |
| 碰撞检测 | 21 | 7 | 0 | 14 |
| 解析器 | 15 | 7 | 3 | 5 |
| 帧算法 | 9 | 0 | 2 | 7 |
| Cholesky | 7 | 0 | 2 | 5 |
| 约束求解器 | 10 | 3 | 0 | 7 |
| 模型操作 | 9 | 0 | 0 | 9 |
| 序列化 | 5 | 0 | 0 | 5 |
| 自动微分/代码生成 | 7 | 4 | 0 | 3 |
| 可视化 | 4 | 1 | 0 | 3 |
| 并行计算 | 5 | 0 | 0 | 5 |
| FFI/绑定 | 11 | 6 | 0 | 5 |
| **总计** | **329** | **80** | **55** | **194** |

### 30.2 总体完成度

| 指标 | 数值 |
|------|------|
| ✅ 完全实现 | **80 / 329 (24.3%)** |
| ⚠️ 部分实现 (含差异) | **55 / 329 (16.7%)** |
| ❌ 未实现 | **194 / 329 (59.0%)** |
| ✅ + ⚠️ 功能覆盖 | **135 / 329 (41.0%)** |

### 30.3 已实现的核心能力

WASM 版本已实现的功能足以支撑以下核心工作流:

1. **刚体动力学仿真**: RNEA, ABA, CRBA, 轨迹积分
2. **运动学分析**: 正向运动学 (单次+批量)
3. **接触仿真**: 法向接触, 摩擦接触, 冲量响应
4. **质心分析**: CoM, 质心动量, 质心动量矩阵
5. **系统辨识**: 逆动力学/动能/势能/CoM 回归矩阵
6. **碰撞检测**: 基础几何体 AABB+代理球
7. **模型导入/导出**: JSON, URDF, SDF, MJCF
8. **FFI**: 完整的 C ABI 导出 (55+ 函数)
9. **批量操作**: 多数核心算法支持批量计算

### 30.4 主要差距与优先建议

#### P0 - 关键缺失 (影响基础功能完整性)

| 优先级 | 缺失功能 | 影响 | 建议 |
|--------|---------|------|------|
| P0 | 更多关节类型 (FreeFlyer, Spherical, Prismatic, Fixed) | ✅ 已解除主要阻塞 | 已实现 Fixed/Prismatic/Spherical/FreeFlyer；剩余为 Planar/Composite/Mimic/Helical/Universal 等扩展关节 |
| P0 | 解析导数替代有限差分 | 精度和性能问题 | 实现解析 RNEA/ABA 导数递归算法 |
| P0 | 李群运算 (integrate/difference on SO(3)/SE(3)) | 无法正确处理四元数/连续旋转关节 | 实现 exp/log 映射 |
| P0 | Frame 系统 | 无法定义末端执行器/传感器位姿 | 添加 Frame struct 和帧算法 |

#### P1 - 重要缺失 (影响高级应用)

| 优先级 | 缺失功能 | 影响 | 建议 |
|--------|---------|------|------|
| P1 | 碰撞窄相 (GJK/EPA) | 碰撞检测精度不足 | 或集成 hpp-fcl WASM 编译 |
| P1 | 通用约束框架 | 无法表达复杂约束 | 实现 RigidConstraintModel 等 |
| P1 | 空间代数类型 (Motion/Force/Inertia) | 代码不够规范, 难以扩展 | 添加 6D 空间类型 |
| P1 | Jacobian 参考坐标系 (LOCAL/LOCAL_WORLD_ALIGNED) | 无法在本地坐标系下计算雅可比 | 添加参考帧选择参数 |
| P1 | 逆向运动学 (IK) | 无 IK 求解器 | 实现基于雅可比的迭代 IK |
| P1 | JS/Python SDK 完善 | 绑定只封装了 ABA, 50+函数未封装 | 系统性封装所有 FFI 函数 |

#### P2 - 增强功能 (锦上添花)

| 优先级 | 缺失功能 | 影响 | 建议 |
|--------|---------|------|------|
| P2 | 模型操作 (append/buildReduced) | 无法动态修改模型 | 按需实现 |
| P2 | 序列化 (Binary/XML) | 仅 JSON | 低优先级 |
| P2 | 并行计算 | 批量操作已足够 | SIMD 优化优先 |
| P2 | 可视化 (Meshcat) | 有 ASCII 树 | Web可视化可独立开发 |
| P2 | 闭环约束 (loopConstrainedABA) | 仅开链机器人 | 按需实现 |
| P2 | 模型限制 (力矩/速度/位置) | 无安全限制检查 | 按需实现 |
