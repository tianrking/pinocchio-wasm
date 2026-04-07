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

  return {
    exports: w,
    createModelFromJson,
    newWorkspace,
    aba,
    disposeModel,
    disposeWorkspace,
    allocBytes,
    freeBytes,
    memoryU8,
    memoryF64,
    memoryI32,
  };
}
