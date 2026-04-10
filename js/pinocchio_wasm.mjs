export async function loadPinocchioWasm(wasmBytes) {
  const { instance } = await WebAssembly.instantiate(wasmBytes, {});
  const w = instance.exports;

  const textEncoder = new TextEncoder();

  function memoryU8() {
    return new Uint8Array(w.memory.buffer);
  }
  function memoryF64() {
    return new Float64Array(w.memory.buffer);
  }
  function memoryI32() {
    return new Int32Array(w.memory.buffer);
  }

  function allocBytes(size) {
    const ptr = w.pino_alloc(size);
    if (!ptr) throw new Error("pino_alloc failed");
    return ptr;
  }

  function freeBytes(ptr, size) {
    w.pino_dealloc(ptr, size);
  }

  function writeBytes(ptr, bytes) {
    memoryU8().set(bytes, ptr);
  }

  function writeF64Array(arr) {
    const bytes = arr.length * 8;
    const ptr = allocBytes(bytes);
    memoryF64().set(arr, ptr / 8);
    return { ptr, bytes };
  }

  function createModelFromJson(json) {
    const bytes = textEncoder.encode(json);
    const ptr = allocBytes(bytes.length);
    writeBytes(ptr, bytes);
    const model = w.pino_model_create_from_json(ptr, bytes.length);
    freeBytes(ptr, bytes.length);
    if (!model) throw new Error("pino_model_create_from_json failed");
    return model;
  }

  function newWorkspace(model) {
    const ws = w.pino_workspace_new(model);
    if (!ws) throw new Error("pino_workspace_new failed");
    return ws;
  }

  function aba(model, ws, q, qd, tau, g = [0, 0, -9.81]) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq || qd.length !== nq || tau.length !== nq) {
      throw new Error(`invalid state length: nq=${nq}`);
    }

    const qMem = writeF64Array(q);
    const qdMem = writeF64Array(qd);
    const tauMem = writeF64Array(tau);
    const gMem = writeF64Array(g);
    const outMem = writeF64Array(new Float64Array(nq));

    const code = w.pino_aba(model, ws, qMem.ptr, qdMem.ptr, tauMem.ptr, gMem.ptr, outMem.ptr);
    if (code !== 0) throw new Error(`pino_aba failed: ${code}`);

    const qdd = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + nq));

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(tauMem.ptr, tauMem.bytes);
    freeBytes(gMem.ptr, gMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return qdd;
  }

  function disposeModel(model) {
    w.pino_model_free(model);
  }

  function disposeWorkspace(ws) {
    w.pino_workspace_free(ws);
  }

  /**
   * Full SE(3) inverse kinematics.
   * @param {number} model - Model handle
   * @param {number} ws - Workspace handle
   * @param {Float64Array|number[]} qInit - Initial joint angles
   * @param {number} targetLink - Target link index
   * @param {number[]} targetPos - [x, y, z] target position
   * @param {number[]} targetRot - 9-element row-major rotation matrix
   * @param {number} [maxIter=200]
   * @param {number} [eps=1e-6]
   * @param {number} [damping=1e-4]
   * @returns {{ q: number[], converged: boolean, err: number }}
   */
  function inverseKinematics(model, ws, qInit, targetLink, targetPos, targetRot,
                             maxIter = 200, eps = 1e-6, damping = 1e-4) {
    const nq = Number(w.pino_model_nq(model));
    if (qInit.length !== nq) throw new Error(`qInit length ${qInit.length} !== nq ${nq}`);
    if (targetPos.length !== 3) throw new Error("targetPos must have 3 elements");
    if (targetRot.length !== 9) throw new Error("targetRot must have 9 elements (row-major 3x3)");

    const qMem = writeF64Array(qInit);
    const posMem = writeF64Array(targetPos);
    const rotMem = writeF64Array(targetRot);
    const outMem = writeF64Array(new Float64Array(nq));
    const convergedMem = writeF64Array(new Float64Array(1));
    const errMem = writeF64Array(new Float64Array(1));

    const code = w.pino_inverse_kinematics(
      model, ws, qMem.ptr, targetLink,
      posMem.ptr, rotMem.ptr,
      maxIter, eps, damping,
      outMem.ptr, convergedMem.ptr, errMem.ptr
    );
    if (code !== 0) throw new Error(`pino_inverse_kinematics failed: ${code}`);

    const q = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + nq));
    const converged = memoryI32()[convergedMem.ptr / 4] !== 0;
    const err = memoryF64()[errMem.ptr / 8];

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(posMem.ptr, posMem.bytes);
    freeBytes(rotMem.ptr, rotMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    freeBytes(convergedMem.ptr, convergedMem.bytes);
    freeBytes(errMem.ptr, errMem.bytes);

    return { q, converged, err };
  }

  /**
   * Position-only inverse kinematics.
   * @param {number} model - Model handle
   * @param {number} ws - Workspace handle
   * @param {Float64Array|number[]} qInit - Initial joint angles
   * @param {number} targetLink - Target link index
   * @param {number[]} targetPos - [x, y, z] target position
   * @param {number} [maxIter=200]
   * @param {number} [eps=1e-6]
   * @param {number} [damping=1e-4]
   * @returns {{ q: number[], converged: boolean, err: number }}
   */
  function inverseKinematicsPosition(model, ws, qInit, targetLink, targetPos,
                                      maxIter = 200, eps = 1e-6, damping = 1e-4) {
    const nq = Number(w.pino_model_nq(model));
    if (qInit.length !== nq) throw new Error(`qInit length ${qInit.length} !== nq ${nq}`);
    if (targetPos.length !== 3) throw new Error("targetPos must have 3 elements");

    const qMem = writeF64Array(qInit);
    const posMem = writeF64Array(targetPos);
    const outMem = writeF64Array(new Float64Array(nq));
    const convergedMem = writeF64Array(new Float64Array(1));
    const errMem = writeF64Array(new Float64Array(1));

    const code = w.pino_inverse_kinematics_position(
      model, ws, qMem.ptr, targetLink,
      posMem.ptr,
      maxIter, eps, damping,
      outMem.ptr, convergedMem.ptr, errMem.ptr
    );
    if (code !== 0) throw new Error(`pino_inverse_kinematics_position failed: ${code}`);

    const q = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + nq));
    const converged = memoryI32()[convergedMem.ptr / 4] !== 0;
    const err = memoryF64()[errMem.ptr / 8];

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(posMem.ptr, posMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    freeBytes(convergedMem.ptr, convergedMem.bytes);
    freeBytes(errMem.ptr, errMem.bytes);

    return { q, converged, err };
  }

  return {
    exports: w,
    createModelFromJson,
    newWorkspace,
    aba,
    inverseKinematics,
    inverseKinematicsPosition,
    disposeModel,
    disposeWorkspace,
    allocBytes,
    freeBytes,
    memoryU8,
    memoryF64,
    memoryI32,
  };
}
