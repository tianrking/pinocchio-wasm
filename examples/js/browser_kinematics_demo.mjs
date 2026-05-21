import { loadPinocchioWasm } from "../../js/pinocchio_wasm.mjs";

const canvas = document.querySelector("#arm");
const ctx = canvas.getContext("2d");
const statusEl = document.querySelector("#status");
const dimsEl = document.querySelector("#dims");
const eeEl = document.querySelector("#ee");
const benchEl = document.querySelector("#bench");
const posesEl = document.querySelector("#poses");
const q1Input = document.querySelector("#q1");
const q2Input = document.querySelector("#q2");
const q1Value = document.querySelector("#q1Value");
const q2Value = document.querySelector("#q2Value");
const benchmarkButton = document.querySelector("#benchmark");

const wasmUrl = new URL(
  "../../target/wasm32-unknown-unknown/release/pinocchio_wasm.wasm",
  import.meta.url,
);

const wasmResponse = await fetch(wasmUrl);
if (!wasmResponse.ok) {
  throw new Error(`failed to fetch wasm: ${wasmResponse.status} ${wasmUrl}`);
}

const pino = await loadPinocchioWasm(await wasmResponse.arrayBuffer());

const model = pino.createModel([
  {
    parent: -1,
    mass: 0.1,
    com: [0, 0, 0],
    inertia: [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ],
  },
  {
    parent: 0,
    mass: 1,
    com: [0.5, 0, 0],
    inertia: [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ],
    joint: { type: "revolute", axis: [0, 0, 1], origin: [0, 0, 0] },
  },
  {
    parent: 1,
    mass: 1,
    com: [0.5, 0, 0],
    inertia: [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ],
    joint: { type: "revolute", axis: [0, 0, 1], origin: [1, 0, 0] },
  },
]);

const ws = pino.newWorkspace(model);
const nq = pino.modelNq(model);
const nv = pino.modelNv(model);
const nlinks = pino.modelNlinks(model);

const qPtr = pino.allocBytes(nq * 8);
const translationsPtr = pino.allocBytes(nlinks * 3 * 8);
const rotationsPtr = pino.allocBytes(nlinks * 9 * 8);
const q = new Float64Array(nq);

dimsEl.textContent = `nq=${nq}, nv=${nv}, links=${nlinks}`;
statusEl.textContent = "Ready";

function f64() {
  return pino.memoryF64();
}

function writeQ(q1, q2) {
  q[0] = q1;
  q[1] = q2;
  f64().set(q, qPtr / 8);
}

function computeFk(q1, q2) {
  writeQ(q1, q2);
  const code = pino.exports.pino_forward_kinematics_poses(
    model,
    ws,
    qPtr,
    translationsPtr,
    rotationsPtr,
  );
  if (code !== 0) {
    throw new Error(`pino_forward_kinematics_poses failed: ${code}`);
  }
  return readPoses();
}

function readPoses() {
  const memory = f64();
  const translations = [];
  const rotations = [];
  const tBase = translationsPtr / 8;
  const rBase = rotationsPtr / 8;

  for (let link = 0; link < nlinks; link++) {
    translations.push([
      memory[tBase + link * 3],
      memory[tBase + link * 3 + 1],
      memory[tBase + link * 3 + 2],
    ]);

    const rb = rBase + link * 9;
    rotations.push([
      [memory[rb], memory[rb + 1], memory[rb + 2]],
      [memory[rb + 3], memory[rb + 4], memory[rb + 5]],
      [memory[rb + 6], memory[rb + 7], memory[rb + 8]],
    ]);
  }

  return { translations, rotations };
}

function endEffector(poses) {
  const p = poses.translations[2];
  const r = poses.rotations[2];
  return [p[0] + r[0][0], p[1] + r[1][0], p[2] + r[2][0]];
}

function toCanvas(point) {
  const scale = Math.min(canvas.width, canvas.height) * 0.31;
  const originX = canvas.width * 0.5;
  const originY = canvas.height * 0.57;
  return [originX + point[0] * scale, originY - point[1] * scale];
}

function draw(poses) {
  const base = [0, 0, 0];
  const joint2 = poses.translations[2];
  const ee = endEffector(poses);
  const points = [base, joint2, ee].map(toCanvas);

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#f8fafc";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.strokeStyle = "#d0d7e2";
  ctx.lineWidth = 1;
  for (let x = 0; x <= canvas.width; x += 45) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, canvas.height);
    ctx.stroke();
  }
  for (let y = 0; y <= canvas.height; y += 45) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(canvas.width, y);
    ctx.stroke();
  }

  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.strokeStyle = "#176b87";
  ctx.lineWidth = 18;
  ctx.beginPath();
  ctx.moveTo(points[0][0], points[0][1]);
  ctx.lineTo(points[1][0], points[1][1]);
  ctx.lineTo(points[2][0], points[2][1]);
  ctx.stroke();

  ctx.fillStyle = "#172033";
  for (const [x, y] of points) {
    ctx.beginPath();
    ctx.arc(x, y, 10, 0, Math.PI * 2);
    ctx.fill();
  }

  ctx.fillStyle = "#c2410c";
  ctx.beginPath();
  ctx.arc(points[2][0], points[2][1], 6, 0, Math.PI * 2);
  ctx.fill();

  eeEl.textContent = `[${ee.map((v) => v.toFixed(4)).join(", ")}]`;
  posesEl.textContent = JSON.stringify(
    {
      linkOrigins: poses.translations.map((p) => p.map((v) => Number(v.toFixed(5)))),
      endEffector: ee.map((v) => Number(v.toFixed(5))),
    },
    null,
    2,
  );
}

function update() {
  const q1 = Number(q1Input.value);
  const q2 = Number(q2Input.value);
  q1Value.textContent = `${q1.toFixed(3)} rad`;
  q2Value.textContent = `${q2.toFixed(3)} rad`;
  draw(computeFk(q1, q2));
}

function runBenchmark() {
  benchmarkButton.disabled = true;
  statusEl.textContent = "Benchmarking";

  requestAnimationFrame(() => {
    const iterations = 100_000;
    const start = performance.now();
    for (let i = 0; i < iterations; i++) {
      const a = 0.7 * Math.sin(i * 0.003);
      const b = 0.9 * Math.cos(i * 0.002);
      computeFk(a, b);
    }
    const elapsed = performance.now() - start;
    const perCallUs = (elapsed * 1000) / iterations;
    benchEl.textContent = `${iterations.toLocaleString()} calls, ${perCallUs.toFixed(3)} us/call`;
    statusEl.textContent = "Ready";
    benchmarkButton.disabled = false;
    update();
  });
}

q1Input.addEventListener("input", update);
q2Input.addEventListener("input", update);
benchmarkButton.addEventListener("click", runBenchmark);

window.addEventListener("beforeunload", () => {
  pino.freeBytes(qPtr, nq * 8);
  pino.freeBytes(translationsPtr, nlinks * 3 * 8);
  pino.freeBytes(rotationsPtr, nlinks * 9 * 8);
  pino.disposeWorkspace(ws);
  pino.disposeModel(model);
});

update();
