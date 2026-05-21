import { loadPinocchioWasm } from "../../js/pinocchio_wasm.mjs";

const wasmUrl = new URL(
  "../../target/wasm32-unknown-unknown/release/pinocchio_wasm.wasm",
  import.meta.url,
);
const urdfUrl = new URL("../urdf/two_link.urdf", import.meta.url);

const [wasmResponse, urdfResponse] = await Promise.all([fetch(wasmUrl), fetch(urdfUrl)]);
if (!wasmResponse.ok) throw new Error(`failed to fetch wasm: ${wasmResponse.status}`);
if (!urdfResponse.ok) throw new Error(`failed to fetch urdf: ${urdfResponse.status}`);

const pino = await loadPinocchioWasm(await wasmResponse.arrayBuffer());

const canvas = document.querySelector("#canvas");
const ctx = canvas.getContext("2d");
const statusEl = document.querySelector("#status");
const dimsEl = document.querySelector("#dims");
const jointsEl = document.querySelector("#joints");
const outputEl = document.querySelector("#output");
const urdfEl = document.querySelector("#urdf");
const loadButton = document.querySelector("#load");

let model = 0;
let ws = 0;
let q = [];

urdfEl.value = await urdfResponse.text();

function cleanup() {
  if (ws) pino.disposeWorkspace(ws);
  if (model) pino.disposeModel(model);
  ws = 0;
  model = 0;
}

function loadUrdf() {
  cleanup();
  model = pino.createModelFromUrdf(urdfEl.value);
  ws = pino.newWorkspace(model);
  q = Array(pino.modelNq(model)).fill(0);

  statusEl.textContent = "URDF loaded";
  dimsEl.textContent = `nq=${pino.modelNq(model)}, nv=${pino.modelNv(model)}, links=${pino.modelNlinks(model)}`;
  renderJointControls();
  update();
}

function renderJointControls() {
  jointsEl.textContent = "";
  q.forEach((value, index) => {
    const label = document.createElement("label");
    const span = document.createElement("span");
    const input = document.createElement("input");
    span.textContent = `q[${index}] = ${value.toFixed(3)} rad`;
    input.type = "range";
    input.min = "-3.1416";
    input.max = "3.1416";
    input.step = "0.001";
    input.value = String(value);
    input.addEventListener("input", () => {
      q[index] = Number(input.value);
      span.textContent = `q[${index}] = ${q[index].toFixed(3)} rad`;
      update();
    });
    label.append(span, input);
    jointsEl.append(label);
  });
}

function update() {
  const poses = pino.forwardKinematicsPoses(model, ws, q);
  const ee = toolPointWorld(poses, [1, 0, 0]);
  draw(poses);
  outputEl.textContent = JSON.stringify(
    {
      q: q.map((v) => Number(v.toFixed(5))),
      linkOrigins: poses.translations.map((p) => p.map((v) => Number(v.toFixed(5)))),
      endEffectorOrigin: poses.translations.at(-1).map((v) => Number(v.toFixed(5))),
      toolPointLocal: [1, 0, 0],
      toolPointWorld: ee.map((v) => Number(v.toFixed(5))),
    },
    null,
    2,
  );
}

function toolPointWorld(poses, localPoint) {
  const lastIndex = poses.translations.length - 1;
  const t = poses.translations[lastIndex];
  const r = poses.rotations[lastIndex];
  return [
    t[0] + r[0][0] * localPoint[0] + r[0][1] * localPoint[1] + r[0][2] * localPoint[2],
    t[1] + r[1][0] * localPoint[0] + r[1][1] * localPoint[1] + r[1][2] * localPoint[2],
    t[2] + r[2][0] * localPoint[0] + r[2][1] * localPoint[1] + r[2][2] * localPoint[2],
  ];
}

function toCanvas(point) {
  const scale = Math.min(canvas.width, canvas.height) * 0.31;
  return [canvas.width * 0.5 + point[0] * scale, canvas.height * 0.57 - point[1] * scale];
}

function draw(poses) {
  const worldPoints = [[0, 0, 0], ...poses.translations.slice(1), toolPointWorld(poses, [1, 0, 0])];
  const points = worldPoints.map(toCanvas);
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

  ctx.strokeStyle = "#176b87";
  ctx.lineWidth = 16;
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.beginPath();
  points.forEach(([x, y], index) => {
    if (index === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();

  ctx.fillStyle = "#172033";
  points.forEach(([x, y]) => {
    ctx.beginPath();
    ctx.arc(x, y, 9, 0, Math.PI * 2);
    ctx.fill();
  });

  const [tipX, tipY] = points.at(-1);
  ctx.fillStyle = "#c2410c";
  ctx.beginPath();
  ctx.arc(tipX, tipY, 6, 0, Math.PI * 2);
  ctx.fill();
}

loadButton.addEventListener("click", () => {
  try {
    loadUrdf();
  } catch (error) {
    statusEl.textContent = "URDF failed";
    outputEl.textContent = error instanceof Error ? error.message : String(error);
  }
});

window.addEventListener("beforeunload", cleanup);

loadUrdf();
