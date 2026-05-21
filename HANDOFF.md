# pinocchio-wasm 交接文档

本文档面向后续接手 `pinocchio-wasm` 的开发者，说明项目定位、架构、构建验证、Rust/FFI/WASM/JS/Python 的接口关系、URDF 使用方式、二次开发方法、测试策略和当前边界。本文按当前代码状态整理，不把计划能力当成已完成能力。

## 项目定位

`pinocchio-wasm` 是一个面向 WASM 的刚体动力学/运动学内核，设计灵感来自 Pinocchio，但它不是官方 Pinocchio 的直接绑定，也不是 C++ Pinocchio 的完整移植。当前实现是 Rust 原生算法内核 + C ABI + `wasm32-unknown-unknown` 输出 + 手写 JavaScript SDK。

核心目标是：

```text
Rust 内核负责模型、数学、运动学、动力学、接触、碰撞、导数等计算
C ABI 负责稳定的跨语言边界
WASM 负责浏览器/Node.js 嵌入
JS SDK 负责把裸指针接口包装成易用 API
Python ctypes helper 负责基础本地动态库调用验证
```

典型使用场景：

```text
浏览器中加载 URDF，做高频正运动学 FK
浏览器中做机器人臂配置空间插值、姿态更新、轨迹预览
Node.js 中做动力学/接触/碰撞批处理
前端仿真页面中用 WASM 替代 JS 手写运动学
本地 Rust 测试中验证算法正确性
C/C++ 或 Python 通过 C ABI 调用核心算法
```

当前项目最适合作为“浏览器高频运动学和轻量动力学计算内核”。如果要用于工业级复杂机器人仿真、接触求解、完整 Pinocchio API 兼容或高精度数值求解，还需要继续补能力和对标验证。

## 本次配套变更说明

本交接文档整理时，项目中同时补充了 JS/URDF 使用文档、浏览器示例、Node.js 示例，以及少量格式化和严格 clippy 清理。相关文件包括：

```text
README.md
src/algo/rollout.rs
src/core/quaternion.rs
src/ffi/model.rs
tests/advanced_algo.rs
tests/quaternion_and_config.rs
docs/
examples/js/
examples/urdf/
```

这些变更的核心目的不是重写算法，而是补齐浏览器/JS 使用路径、URDF 示例、项目交接说明，并让严格质量检查更稳定。后续继续开发时，建议仍然按“算法/FFI 变更”和“文档/示例变更”拆分提交，方便审查和回退。

## 项目结构

仓库主要结构：

```text
Cargo.toml
Cargo.lock
README.md
include/
  pinocchio_wasm.h          C ABI 头文件
js/
  pinocchio_wasm.mjs        高层 JS SDK
  sdk/runtime.mjs           WASM 内存读写和分配辅助
bindings/python/
  pinocchio_wasm.py         Python ctypes helper
examples/
  js/                       Node/Browser 示例
  urdf/                     示例 URDF
docs/
  JS_USAGE_AND_VERIFICATION.md
  JS_API_REFERENCE.md
PLAN/
  AUDIT.md
  DEVELOPMENT_CHARTER.md
  PINOCCHIO_API_GAP_ANALYSIS.md
  ROADMAP.md
src/
  lib.rs
  core/
  model/
  algo/
  collision/
  autodiff/
  codegen/
  visualization/
  ffi/
tests/
  *.rs
```

Rust 模块分工：

```text
src/core:
  基础错误类型、Vec3、Mat3、Transform、spatial vector/matrix、Quaternion。

src/model:
  Link、Joint、JointType、Model、Workspace。
  JSON/URDF/SDF/MJCF 导入导出。
  配置空间维度 nq、速度空间维度 nv。

src/algo:
  FK、Jacobian、RNEA、CRBA、ABA、compute-all-terms、CoM、Energy。
  接触约束动力学、摩擦接触、冲量、质心项、导数、回归器、rollout。

src/collision:
  多几何碰撞/距离查询，支持 sphere/box/capsule/cylinder/mesh-approx 等抽象。

src/autodiff:
  轻量 dual number 和有限差分 Jacobian 辅助。

src/codegen:
  小型 C/JS 代码生成辅助。

src/visualization:
  模型 ASCII tree 可视化辅助。

src/ffi:
  C ABI/WASM ABI，导出 pino_* 函数。
```

## 构建和验证

基础 Rust 验证：

```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features
cargo test --all-targets --all-features
cargo build --release
```

构建 WASM：

```bash
rustup target add wasm32-unknown-unknown
cargo build --release --target wasm32-unknown-unknown
```

WASM 输出：

```text
target/wasm32-unknown-unknown/release/pinocchio_wasm.wasm
```

JS 语法检查和示例：

```bash
node --check js/pinocchio_wasm.mjs
node --check js/sdk/runtime.mjs
node examples/js/node_demo.mjs
node examples/js/urdf_kinematics_demo.mjs
node examples/js/urdf_high_frequency_fk.mjs
```

浏览器示例：

```bash
cargo build --release --target wasm32-unknown-unknown
python3 -m http.server 8000
```

然后打开：

```text
http://localhost:8000/examples/js/browser_kinematics_demo.html
http://localhost:8000/examples/js/browser_urdf_kinematics.html
```

不要用 `file://` 直接打开 HTML。浏览器通常会拦截本地文件下的 `fetch()` 和 ES module import。

## Cargo 和构建产物

`Cargo.toml` 当前关键信息：

```toml
[package]
name = "pinocchio_wasm"
version = "0.0.1"
edition = "2024"
license = "BSD-2-Clause"

[lib]
crate-type = ["rlib", "cdylib"]

[profile.release]
lto = true
codegen-units = 1
opt-level = 3
panic = "abort"
```

依赖很少：

```text
serde
serde_json
roxmltree
```

这说明项目倾向于保持轻量，不依赖大型数学库或 wasm-bindgen。数值类型和矩阵运算大多由项目自己实现。

发布/版本管理时需要注意：当前版本号还是 `0.0.1`，但功能已经很多。如果要对外发版，应同步更新：

```text
Cargo.toml version
README.md
docs/
examples/
JS SDK 文档
C header
CHANGELOG 或 release notes
```

## 核心数据模型

`Model` 是树结构机器人模型：

```text
links: Vec<Link>
parent_to_children
joint_to_link
link_to_joint
joint_nq
joint_nv
joint_idx_q
joint_idx_v
```

第 0 个 link 必须是 root：

```text
link[0].parent = None
link[0].joint = None
```

非 root link 必须：

```text
有 parent
有 joint
parent 必须出现在 child 前面，也就是拓扑有序
```

当前 JointType：

```text
Revolute   nq=1, nv=1
Prismatic  nq=1, nv=1
Fixed      nq=0, nv=0
Spherical  nq=4, nv=3
FreeFlyer  nq=7, nv=6
```

这是项目里非常关键的维度规则。所有算法调用都要区分：

```text
q  使用 nq
qd 使用 nv
qdd 使用 nv
tau 使用 nv
Jacobian 通常是 6 x nv
Mass matrix 是 nv x nv
```

对于 spherical/free-flyer，`q` 里包含 quaternion，但 `qd/tau/qdd` 是切空间速度维度。不要用 `q.len()` 去推导速度向量长度，应使用：

```js
const nq = pino.modelNq(model);
const nv = pino.modelNv(model);
```

## 模型导入导出

Rust 层支持：

```text
Model::from_json_str
Model::from_urdf_str
Model::from_sdf_str
Model::from_mjcf_str
```

FFI/JS 层支持：

```text
pino_model_create_from_json
pino_model_create_from_urdf
pino_model_create_from_sdf
pino_model_create_from_mjcf

pino_model_to_json
pino_model_to_urdf
pino_model_to_sdf
pino_model_to_mjcf
```

JS SDK 对应：

```js
pino.createModelFromJson(json)
pino.createModelFromUrdf(urdf)
pino.createModelFromSdf(sdf)
pino.createModelFromMjcf(mjcf)

pino.modelToJson(model)
pino.modelToUrdf(model, name)
pino.modelToSdf(model, name)
pino.modelToMjcf(model, name)
```

也可以直接用结构化 JS 对象创建模型：

```js
const model = pino.createModel([
  { parent: -1, mass: 0.1, com: [0, 0, 0] },
  {
    parent: 0,
    mass: 1,
    com: [0.5, 0, 0],
    joint: { type: "revolute", axis: [0, 0, 1], origin: [0, 0, 0] },
  },
]);
```

URDF 示例：

```bash
cargo build --release --target wasm32-unknown-unknown
node examples/js/urdf_kinematics_demo.mjs
node examples/js/urdf_high_frequency_fk.mjs
```

浏览器 URDF 示例：

```bash
python3 -m http.server 8000
```

打开：

```text
http://localhost:8000/examples/js/browser_urdf_kinematics.html
```

注意：当前 URDF/SDF/MJCF parser 是项目内轻量 parser，不应默认认为支持所有复杂机器人描述特性。接入真实机器人 URDF 时，应先做：

```text
模型能否成功导入
nq/nv/nlinks 是否符合预期
link parent 关系是否正确
joint axis/origin 是否正确
FK 结果是否与参考工具一致
Jacobian 是否与有限差分或参考库一致
```

## C ABI 和 WASM 边界

C ABI 头文件：

```text
include/pinocchio_wasm.h
```

底层导出函数名统一以 `pino_` 开头。

生命周期：

```text
pino_model_create*
pino_model_free

pino_workspace_new
pino_workspace_free

pino_collision_model_create*
pino_collision_model_free
```

内存分配：

```text
pino_alloc(size)
pino_dealloc(ptr, size)
```

FFI status code：

```text
0     Ok
-1    NullPtr
-2    InvalidInput
-3    BuildModelFailed
-4    AlgoFailed
-128  Panic
```

大部分算法函数返回 `int32_t` 状态码，`0` 表示成功。模型创建函数失败时通常返回空指针。

FFI 设计原则：

```text
输入输出使用连续数组
调用方负责传入正确长度
Rust 内部不持有 JS/C 传入数组引用
导出的字符串由 Rust 分配，调用方读完后必须 pino_dealloc
Workspace 可复用以减少分配
```

线程安全注意：

当前 FFI handle 是裸指针形式，`WorkspaceHandle` 内部持有可变 workspace。不要从多个线程同时操作同一个 workspace 或同一个可变 handle。浏览器中如果用 Web Worker，可每个 worker 独立创建 model/workspace，或在应用层做互斥。不要把同一个 `ws` 在并发调用中共享。

WASM 内存注意：

如果某次分配导致 `memory.buffer` 增长，旧的 TypedArray view 可能失效。JS SDK 的 `memoryF64()`、`memoryU8()` 每次都会重新取 view，直接使用底层高频接口时也建议按这个模式重新获取 view。

## JavaScript SDK

入口：

```js
import { loadPinocchioWasm } from "./js/pinocchio_wasm.mjs";
```

加载：

```js
const wasmBytes = await fetch("/pinocchio_wasm.wasm").then((r) => r.arrayBuffer());
const pino = await loadPinocchioWasm(wasmBytes);
```

Node.js：

```js
import { readFile } from "node:fs/promises";
import { loadPinocchioWasm } from "./js/pinocchio_wasm.mjs";

const wasmBytes = await readFile("./target/wasm32-unknown-unknown/release/pinocchio_wasm.wasm");
const pino = await loadPinocchioWasm(wasmBytes);
```

常用生命周期：

```js
const model = pino.createModelFromUrdf(urdf);
const ws = pino.newWorkspace(model);

// ... call algorithms ...

pino.disposeWorkspace(ws);
pino.disposeModel(model);
```

当前 SDK 暴露的主要函数：

```text
Model:
  createModelFromJson
  createModelFromUrdf
  createModelFromSdf
  createModelFromMjcf
  createModel
  modelToJson
  modelToUrdf
  modelToSdf
  modelToMjcf
  modelNq
  modelNv
  modelNlinks
  newWorkspace
  disposeModel
  disposeWorkspace

Dynamics:
  rnea
  aba
  crba
  gravityTorques
  coriolisTorques

Kinematics:
  frameJacobian
  centerOfMass
  energy
  computeAllTerms
  forwardKinematicsPoses

Batch:
  rneaBatch
  abaBatch
  crbaBatch
  rolloutAbaEuler

Contact:
  contactConstrainedDynamics
  applyContactImpulse
  contactJacobianNormal

Collision:
  createCollisionModel
  createCollisionModelGeometries
  disposeCollisionModel
  collisionMinDistance
  collisionQueryDetails

Centroidal:
  centroidalMomentum
  centroidalMap
  centroidalFullTerms

Regressors:
  inverseDynamicsRegressor
  kineticEnergyRegressor

Derivatives:
  rneaSecondOrderDerivatives
  kinematicsDerivatives
  rneaDerivatives
  abaDerivatives

Constrained:
  constrainedAbaLockedJoints

Configuration-space:
  neutralConfiguration
  normalizeConfiguration
  isNormalized
  differenceConfiguration
  interpolateConfiguration
  randomConfiguration

Memory:
  allocBytes
  freeBytes
  memoryU8
  memoryF64
  memoryI32
  memoryU32
  exports
```

高层 SDK 适合普通调用。高频 FK、动画、仿真预览建议走预分配 buffer + 直接 FFI：

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
    rotationsPtr,
  );
  if (code !== 0) throw new Error(`FK failed: ${code}`);

  const memory = pino.memoryF64();
  // read only what you need
}

pino.freeBytes(qPtr, nq * 8);
pino.freeBytes(translationsPtr, nlinks * 3 * 8);
pino.freeBytes(rotationsPtr, nlinks * 9 * 8);
```

这是浏览器高频运动学的推荐模式：

```text
模型只创建一次
Workspace 只创建一次
WASM buffer 只分配一次
每帧只写 q
直接调用 pino.exports.pino_forward_kinematics_poses
只读取渲染需要的 link pose
页面销毁或 worker 结束时释放 buffer 和 handle
```

## Python helper

Python 文件：

```text
bindings/python/pinocchio_wasm.py
```

它通过 `ctypes.CDLL` 加载本地动态库：

```text
target/release/libpinocchio_wasm.so
target/release/libpinocchio_wasm.dylib
target/release/pinocchio_wasm.dll
```

使用前先构建本地动态库：

```bash
cargo build --release
```

当前 Python helper 支持范围很小：

```text
model_from_json
workspace_new
free_model
free_workspace
aba
```

所以它的定位是 ctypes 调用验证和最小 Python 示例，不是完整 Python SDK。不要把它当作与 JS SDK 等价的完整绑定。如果后续要做 Python 包，应补：

```text
完整函数 argtypes/restype
错误码翻译
资源上下文管理器
numpy array 支持
URDF/SDF/MJCF 创建
FK/Jacobian/RNEA/CRBA/ABA/Collision 等完整 API
单元测试
wheel 构建和平台动态库打包
```

## 算法能力概览

当前 Rust/FFI/JS 覆盖面很广，主要包括：

```text
Forward Kinematics
Frame Jacobian
RNEA inverse dynamics
ABA forward dynamics
CRBA mass matrix
gravity torques
coriolis torques
compute all terms
center of mass
kinetic/potential energy
batch RNEA/ABA/CRBA
ABA Euler rollout
locked-joint constrained ABA
contact constrained dynamics
contact impulse
friction contact
contact normal Jacobian
collision distance/query
centroidal map/momentum/full terms
inverse dynamics regressor
energy regressors
RNEA/ABA/kinematics derivatives
configuration-space neutral/normalize/difference/interpolate/random
```

验证时不要只跑一个 demo。真正判断核心是否可靠，应跑完整测试：

```bash
cargo test --all-targets --all-features
```

测试文件覆盖：

```text
dynamics.rs
joint_types.rs
aba_derivatives.rs
rnea_derivatives_analytical.rs
kinematics_derivatives_analytical.rs
batch.rs
contact_dynamics.rs
contact_friction.rs
collision.rs
model_json.rs
model_urdf.rs
model_sdf.rs
model_mjcf.rs
model_serde.rs
floating_joints.rs
quaternion_and_config.rs
rollout.rs
ffi_*.rs
```

重要数值一致性：

```text
ABA 应能反推 RNEA 的加速度关系
CRBA mass matrix 应该对称
computeAllTerms 应和单独 crba/bias/gravity/coriolis/com/energy 结果一致
quaternion 配置空间 integrate/interpolate/random 后应保持归一化
FFI 和 Rust 原生 API 结果应一致
URDF/SDF/MJCF 导入导出维度应一致
```

## 配置空间和 Quaternion

项目已经有 `Quaternion` 和模型感知的配置空间操作。相关接口：

```text
pino_neutral_configuration
pino_normalize_configuration
pino_is_normalized
pino_difference_configuration
pino_interpolate_configuration
pino_random_configuration
```

JS SDK：

```js
pino.neutralConfiguration(model)
pino.normalizeConfiguration(model, q)
pino.isNormalized(model, q, tol)
pino.differenceConfiguration(model, q0, q1)
pino.interpolateConfiguration(model, q0, q1, alpha)
pino.randomConfiguration(model, lower, upper, seed)
```

对 spherical/free-flyer 模型，不能用普通向量加减替代配置空间运算。正确做法是：

```text
需要求 q1-q0 时，用 differenceConfiguration
需要插值时，用 interpolateConfiguration
需要归一化时，用 normalizeConfiguration
需要生成默认姿态时，用 neutralConfiguration
```

这是浏览器机器人臂/移动基座仿真里非常重要的细节。

## URDF 浏览器高频 FK 推荐方案

浏览器里要高效做运动学，推荐架构：

```text
主线程:
  UI、Three.js 渲染、用户输入

Web Worker:
  加载 pinocchio_wasm.wasm
  createModelFromUrdf
  newWorkspace
  预分配 q/poses buffers
  每帧或按需计算 FK/Jacobian
  只把必要 pose 数据 postMessage 回主线程
```

基本流程：

```text
1. fetch URDF 文本。
2. fetch WASM bytes。
3. loadPinocchioWasm。
4. createModelFromUrdf。
5. 读取 nq/nv/nlinks。
6. newWorkspace。
7. 为 q、translations、rotations 分配 WASM 内存。
8. 每次关节角变化时写 q。
9. 调 pino_forward_kinematics_poses。
10. 从输出 buffer 读取 link pose。
11. 更新 Three.js link transform。
12. 页面关闭时 free buffer、dispose workspace/model。
```

如果只是低频按钮点击或调试页面，可以用：

```js
pino.forwardKinematicsPoses(model, ws, q)
```

如果要 60Hz/120Hz 或大量采样，使用直接 FFI buffer 模式。

## C/C++ 嵌入注意事项

C/C++ 使用时包含：

```c
#include "pinocchio_wasm.h"
```

典型生命周期：

```c
PinoModelHandle *model = pino_model_create_from_urdf(ptr, len);
PinoWorkspaceHandle *ws = pino_workspace_new(model);

int32_t status = pino_forward_kinematics_poses(model, ws, q, translations, rotations);

pino_workspace_free(ws);
pino_model_free(model);
```

调用方必须保证：

```text
指针非空
数组长度正确
输出 buffer 足够大
model 和 workspace 匹配
free 不重复调用
多线程不并发写同一个 workspace
```

当前 C header 主要表达函数签名，没有详细线程安全和生命周期注释。后续如果对外提供 C SDK，建议补齐 header 文档。

## 二次开发指南

新增算法建议流程：

```text
1. 先在 src/algo/ 中实现纯 Rust API。
2. 用 Model/Workspace 和 Vec<f64> 等安全类型表达算法。
3. 写 Rust 单元测试，验证数值性质。
4. 再在 src/ffi/ 中加 pino_* C ABI。
5. 补 tests/ffi_*.rs，验证 FFI 和 Rust API 一致。
6. 在 include/pinocchio_wasm.h 加声明。
7. 在 js/pinocchio_wasm.mjs 加高层 wrapper。
8. 如需高频调用，暴露 direct buffer 示例。
9. 更新 docs/JS_API_REFERENCE.md 和示例。
```

新增模型格式或扩展 URDF parser：

```text
1. 先在 src/model/<format>.rs 里扩展 parser。
2. 保持 Model::new 的拓扑和维度约束。
3. 增加 tests/model_<format>.rs。
4. 增加 FFI create/export 函数。
5. 更新 JS createModelFrom* / modelTo*。
6. 加最小 examples。
```

新增 JS SDK API 时要注意：

```text
高层 wrapper 要检查 status code。
内存分配要 try/finally 或确保异常时释放。
数组维度要按 nq/nv/nlinks 推导。
导出字符串必须 read 后 dealloc。
不要缓存旧 memory buffer view。
```

新增 FFI API 时要注意：

```text
所有裸指针先 check_non_null。
所有 slice 长度用模型维度严格计算。
catch_unwind 后返回 Status::Panic。
不要 panic 穿过 FFI 边界。
不要返回 Rust 内部临时引用。
返回字符串时使用 pino_alloc/pino_dealloc 约定。
```

## 发布前检查清单

建议每次发布前执行：

```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-targets --all-features
rustup target add wasm32-unknown-unknown
cargo build --release --target wasm32-unknown-unknown
node --check js/pinocchio_wasm.mjs
node --check js/sdk/runtime.mjs
node examples/js/node_demo.mjs
node examples/js/urdf_kinematics_demo.mjs
node examples/js/urdf_high_frequency_fk.mjs
```

浏览器手测：

```text
python3 -m http.server 8000
打开 browser_kinematics_demo.html
打开 browser_urdf_kinematics.html
确认 WASM fetch 成功
确认 URDF 加载成功
确认 FK/Jacobian 数字变化正常
确认控制台无异常
```

如果更新了 FFI：

```text
同步 include/pinocchio_wasm.h
同步 js/pinocchio_wasm.mjs
同步 docs/JS_API_REFERENCE.md
同步 tests/ffi_*.rs
同步 README C ABI Overview
```

如果更新了配置空间或 quaternion：

```text
重点跑 tests/quaternion_and_config.rs
重点跑 tests/floating_joints.rs
重点跑 JS URDF/FK 示例
确认 spherical/free-flyer nq/nv 没有退化
```

## 已知边界和风险

项目名和灵感来自 Pinocchio，但当前不是官方 Pinocchio API 的完整兼容层。不要承诺与 C++ Pinocchio 所有算法、模型格式、数值结果完全一致。

URDF/SDF/MJCF parser 是轻量实现。真实机器人模型导入前必须对 FK/Jacobian/维度做交叉验证。

FFI 当前是裸指针 API。它适合 WASM 和 C ABI，但需要调用方严格遵守生命周期和线程安全约定。

JS SDK 是手写封装，没有 npm package 元数据，也没有 TypeScript 类型定义。后续对外使用建议补 `package.json`、`.d.ts`、打包示例和版本策略。

Python helper 不是完整 Python binding，只是最小 ctypes 示例。

当前错误信息在 JS 侧主要是 status code 或创建失败，缺少详细 last error message 机制。如果用户导入复杂 URDF 失败，调试体验可能不够好。

当前 release profile 是 `panic = "abort"`，FFI 里虽然对很多算法路径用了 `catch_unwind` 返回 `Panic`，但 release abort 策略下不应依赖 panic 恢复。算法和 FFI 应尽量用 Result/Status 返回错误。

高频浏览器计算应避免高层 wrapper 的反复分配。对于实时仿真和机器人臂可视化，应使用预分配 buffer。

## 后续推荐路线

短期：

```text
补 HANDOFF/README/JS docs 一致性。
把当前未提交改动拆成清晰提交。
确认 cargo fmt/clippy/test 全部通过。
确认 wasm build 和 Node/browser demo 全部可运行。
为 JS SDK 补 TypeScript 类型声明。
为 C header 补生命周期和线程安全注释。
```

中期：

```text
扩展 URDF parser 对真实机器人模型的支持。
增加更多与参考实现的数值对标测试。
完善 Python binding 或明确移除对完整 Python SDK 的暗示。
增加 npm 包发布结构。
增加 browser worker 示例和 Three.js robot viewer 示例。
增加错误详情 API，例如 last_error 或 out-error-buffer。
```

长期：

```text
与官方 Pinocchio 做 API gap matrix。
引入 SIMD/SoA 批处理优化。
完善接触/碰撞求解器数值稳定性。
支持更多几何、mesh、collision filtering。
支持更完整的机器人描述格式。
构建可被 motorbridge-studio / motorbridge-arm 直接使用的浏览器机器人学内核。
```

## 接手建议

接手时建议按这个顺序读代码：

```text
1. README.md 和 docs/JS_USAGE_AND_VERIFICATION.md
2. src/model/mod.rs，理解 Model、JointType、nq/nv
3. src/core/math.rs 和 src/core/quaternion.rs
4. src/algo/kinematics.rs、dynamics.rs、rollout.rs
5. src/ffi/mod.rs 和 src/ffi/model.rs
6. js/sdk/runtime.mjs 和 js/pinocchio_wasm.mjs
7. examples/js/urdf_kinematics_demo.mjs
8. tests/quaternion_and_config.rs、tests/model_urdf.rs、tests/ffi_api.rs
```

开发时最重要的工程纪律：

```text
Rust 安全 API 先行，FFI 后补。
每个 FFI 函数都要有测试。
每个 JS wrapper 都要有示例或文档。
模型维度永远区分 nq/nv。
quaternion 配置空间不能普通线性相减。
高频路径避免重复分配。
发布前同步 README、docs、header、JS SDK 和 tests。
```

这份文档反映当前项目状态。后续如果补齐 npm 包、TypeScript、完整 Python SDK、真实机器人 URDF 支持或 motorbridge-studio 集成，请同步更新本文档。
