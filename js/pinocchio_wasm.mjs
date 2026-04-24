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

  function writeI32Array(arr) {
    const bytes = arr.length * 4;
    const ptr = allocBytes(bytes);
    memoryI32().set(arr, ptr / 4);
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

  /**
   * Create a model from structured link data.
   * @param {Array} links - Array of link objects. First link is root.
   *   Each link: { parent, joint: { axis, origin, type }, mass, com, inertia }
   *   joint.type: "revolute" (0, default), "prismatic" (1), "fixed" (2)
   */
  function createModel(links) {
    const nlinks = links.length;
    if (nlinks === 0) throw new Error("links array must not be empty");

    const parents = new Int32Array(nlinks);
    const axes = new Float64Array(nlinks * 3);
    const origins = new Float64Array(nlinks * 3);
    const masses = new Float64Array(nlinks);
    const coms = new Float64Array(nlinks * 3);
    const inertias = new Float64Array(nlinks * 9);
    const jointTypes = new Int32Array(nlinks);

    for (let i = 0; i < nlinks; i++) {
      const l = links[i];
      parents[i] = l.parent !== undefined ? l.parent : -1;
      masses[i] = l.mass || 1.0;

      if (l.com) {
        coms[3 * i] = l.com[0] || 0;
        coms[3 * i + 1] = l.com[1] || 0;
        coms[3 * i + 2] = l.com[2] || 0;
      }

      if (l.inertia) {
        for (let r = 0; r < 3; r++) {
          for (let c = 0; c < 3; c++) {
            inertias[9 * i + 3 * r + c] = l.inertia[r][c] || (r === c ? 1 : 0);
          }
        }
      } else {
        // identity inertia
        inertias[9 * i] = 1;
        inertias[9 * i + 4] = 1;
        inertias[9 * i + 8] = 1;
      }

      if (i === 0) continue; // root has no joint

      const j = l.joint || {};
      const typeVal = typeof j.type === "number" ? j.type :
        j.type === "prismatic" ? 1 :
        j.type === "fixed" ? 2 : 0;
      jointTypes[i] = typeVal;

      if (j.axis) {
        axes[3 * i] = j.axis[0] || 0;
        axes[3 * i + 1] = j.axis[1] || 0;
        axes[3 * i + 2] = j.axis[2] || 0;
      } else {
        axes[3 * i + 2] = 1; // default z-axis
      }

      if (j.origin) {
        origins[3 * i] = j.origin[0] || 0;
        origins[3 * i + 1] = j.origin[1] || 0;
        origins[3 * i + 2] = j.origin[2] || 0;
      }
    }

    const parentsMem = writeI32Array(parents);
    const axesMem = writeF64Array(axes);
    const originsMem = writeF64Array(origins);
    const massesMem = writeF64Array(masses);
    const comsMem = writeF64Array(coms);
    const inertiasMem = writeF64Array(inertias);
    const jtypesMem = writeI32Array(jointTypes);

    const model = w.pino_model_create(
      nlinks,
      parentsMem.ptr,
      axesMem.ptr,
      originsMem.ptr,
      massesMem.ptr,
      comsMem.ptr,
      inertiasMem.ptr,
      jtypesMem.ptr
    );

    freeBytes(parentsMem.ptr, parentsMem.bytes);
    freeBytes(axesMem.ptr, axesMem.bytes);
    freeBytes(originsMem.ptr, originsMem.bytes);
    freeBytes(massesMem.ptr, massesMem.bytes);
    freeBytes(comsMem.ptr, comsMem.bytes);
    freeBytes(inertiasMem.ptr, inertiasMem.bytes);
    freeBytes(jtypesMem.ptr, jtypesMem.bytes);

    if (!model) throw new Error("pino_model_create failed");
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
    createModel,
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
