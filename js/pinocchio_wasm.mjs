export async function loadPinocchioWasm(wasmBytes) {
  const { instance } = await WebAssembly.instantiate(wasmBytes, {});
  const w = instance.exports;

  const textEncoder = new TextEncoder();
  const textDecoder = new TextDecoder();

  function memoryU8() {
    return new Uint8Array(w.memory.buffer);
  }
  function memoryF64() {
    return new Float64Array(w.memory.buffer);
  }
  function memoryI32() {
    return new Int32Array(w.memory.buffer);
  }
  function memoryU32() {
    return new Uint32Array(w.memory.buffer);
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

  // ---------------------------------------------------------------------------
  // Model lifecycle
  // ---------------------------------------------------------------------------

  function createModelFromJson(json) {
    const bytes = textEncoder.encode(json);
    const ptr = allocBytes(bytes.length);
    writeBytes(ptr, bytes);
    const model = w.pino_model_create_from_json(ptr, bytes.length);
    freeBytes(ptr, bytes.length);
    if (!model) throw new Error("pino_model_create_from_json failed");
    return model;
  }

  function createModelFromUrdf(urdfString) {
    const bytes = textEncoder.encode(urdfString);
    const ptr = allocBytes(bytes.length);
    writeBytes(ptr, bytes);
    const model = w.pino_model_create_from_urdf(ptr, bytes.length);
    freeBytes(ptr, bytes.length);
    if (!model) throw new Error("pino_model_create_from_urdf failed");
    return model;
  }

  function createModelFromSdf(sdfString) {
    const bytes = textEncoder.encode(sdfString);
    const ptr = allocBytes(bytes.length);
    writeBytes(ptr, bytes);
    const model = w.pino_model_create_from_sdf(ptr, bytes.length);
    freeBytes(ptr, bytes.length);
    if (!model) throw new Error("pino_model_create_from_sdf failed");
    return model;
  }

  function createModelFromMjcf(mjcfString) {
    const bytes = textEncoder.encode(mjcfString);
    const ptr = allocBytes(bytes.length);
    writeBytes(ptr, bytes);
    const model = w.pino_model_create_from_mjcf(ptr, bytes.length);
    freeBytes(ptr, bytes.length);
    if (!model) throw new Error("pino_model_create_from_mjcf failed");
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

  /**
   * Helper to read a string exported via double-indirect pointer (out_ptr, out_len).
   */
  function readExportedString(outPtrPtr, outLenPtr) {
    const u32 = memoryU32();
    // outPtrPtr/outLenPtr are byte offsets into U32 view
    const strPtr = Number(u32[outPtrPtr / 4]);
    const strLen = Number(u32[outLenPtr / 4]);
    const bytes = memoryU8().slice(strPtr, strPtr + strLen);
    // Free the string buffer allocated by WASM
    freeBytes(strPtr, strLen);
    return textDecoder.decode(bytes);
  }

  function modelToJson(model) {
    // Allocate space for output pointer and length (pointer-sized: usize = 4 bytes on wasm32)
    const outPtrPtr = allocBytes(4);
    const outLenPtr = allocBytes(4);
    const code = w.pino_model_to_json(model, outPtrPtr, outLenPtr);
    if (code !== 0) {
      freeBytes(outPtrPtr, 4);
      freeBytes(outLenPtr, 4);
      throw new Error(`pino_model_to_json failed: ${code}`);
    }
    const json = readExportedString(outPtrPtr, outLenPtr);
    freeBytes(outPtrPtr, 4);
    freeBytes(outLenPtr, 4);
    return json;
  }

  function modelToUrdf(model, robotName = "pinocchio_wasm") {
    const nameBytes = textEncoder.encode(robotName);
    const namePtr = allocBytes(nameBytes.length);
    writeBytes(namePtr, nameBytes);
    const outPtrPtr = allocBytes(4);
    const outLenPtr = allocBytes(4);
    const code = w.pino_model_to_urdf(model, namePtr, nameBytes.length, outPtrPtr, outLenPtr);
    freeBytes(namePtr, nameBytes.length);
    if (code !== 0) {
      freeBytes(outPtrPtr, 4);
      freeBytes(outLenPtr, 4);
      throw new Error(`pino_model_to_urdf failed: ${code}`);
    }
    const urdf = readExportedString(outPtrPtr, outLenPtr);
    freeBytes(outPtrPtr, 4);
    freeBytes(outLenPtr, 4);
    return urdf;
  }

  function modelToSdf(model, modelName = "pinocchio_wasm") {
    const nameBytes = textEncoder.encode(modelName);
    const namePtr = allocBytes(nameBytes.length);
    writeBytes(namePtr, nameBytes);
    const outPtrPtr = allocBytes(4);
    const outLenPtr = allocBytes(4);
    const code = w.pino_model_to_sdf(model, namePtr, nameBytes.length, outPtrPtr, outLenPtr);
    freeBytes(namePtr, nameBytes.length);
    if (code !== 0) {
      freeBytes(outPtrPtr, 4);
      freeBytes(outLenPtr, 4);
      throw new Error(`pino_model_to_sdf failed: ${code}`);
    }
    const sdf = readExportedString(outPtrPtr, outLenPtr);
    freeBytes(outPtrPtr, 4);
    freeBytes(outLenPtr, 4);
    return sdf;
  }

  function modelToMjcf(model, modelName = "pinocchio_wasm") {
    const nameBytes = textEncoder.encode(modelName);
    const namePtr = allocBytes(nameBytes.length);
    writeBytes(namePtr, nameBytes);
    const outPtrPtr = allocBytes(4);
    const outLenPtr = allocBytes(4);
    const code = w.pino_model_to_mjcf(model, namePtr, nameBytes.length, outPtrPtr, outLenPtr);
    freeBytes(namePtr, nameBytes.length);
    if (code !== 0) {
      freeBytes(outPtrPtr, 4);
      freeBytes(outLenPtr, 4);
      throw new Error(`pino_model_to_mjcf failed: ${code}`);
    }
    const mjcf = readExportedString(outPtrPtr, outLenPtr);
    freeBytes(outPtrPtr, 4);
    freeBytes(outLenPtr, 4);
    return mjcf;
  }

  function modelNq(model) {
    return Number(w.pino_model_nq(model));
  }

  function modelNlinks(model) {
    return Number(w.pino_model_nlinks(model));
  }

  function newWorkspace(model) {
    const ws = w.pino_workspace_new(model);
    if (!ws) throw new Error("pino_workspace_new failed");
    return ws;
  }

  function disposeModel(model) {
    w.pino_model_free(model);
  }

  function disposeWorkspace(ws) {
    w.pino_workspace_free(ws);
  }

  // ---------------------------------------------------------------------------
  // Core dynamics
  // ---------------------------------------------------------------------------

  function rnea(model, ws, q, qd, qdd, g = [0, 0, -9.81]) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq || qd.length !== nq || qdd.length !== nq) {
      throw new Error(`invalid state length: nq=${nq}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const qdMem = writeF64Array(new Float64Array(qd));
    const qddMem = writeF64Array(new Float64Array(qdd));
    const gMem = writeF64Array(new Float64Array(g));
    const outMem = writeF64Array(new Float64Array(nq));

    const code = w.pino_rnea(model, ws, qMem.ptr, qdMem.ptr, qddMem.ptr, gMem.ptr, outMem.ptr);
    if (code !== 0) throw new Error(`pino_rnea failed: ${code}`);

    const tau = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + nq));

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(qddMem.ptr, qddMem.bytes);
    freeBytes(gMem.ptr, gMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return tau;
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

  function crba(model, ws, q) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq) {
      throw new Error(`invalid q length: expected ${nq}, got ${q.length}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const outMem = writeF64Array(new Float64Array(nq * nq));

    const code = w.pino_crba(model, ws, qMem.ptr, outMem.ptr);
    if (code !== 0) throw new Error(`pino_crba failed: ${code}`);

    const flat = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + nq * nq));
    // Reshape row-major flat to 2D array
    const M = [];
    for (let r = 0; r < nq; r++) {
      M.push(flat.slice(r * nq, r * nq + nq));
    }

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return M;
  }

  function gravityTorques(model, ws, q, g = [0, 0, -9.81]) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq) {
      throw new Error(`invalid q length: expected ${nq}, got ${q.length}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const gMem = writeF64Array(new Float64Array(g));
    const outMem = writeF64Array(new Float64Array(nq));

    const code = w.pino_gravity_torques(model, ws, qMem.ptr, gMem.ptr, outMem.ptr);
    if (code !== 0) throw new Error(`pino_gravity_torques failed: ${code}`);

    const result = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + nq));

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(gMem.ptr, gMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return result;
  }

  function coriolisTorques(model, ws, q, qd) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq || qd.length !== nq) {
      throw new Error(`invalid state length: nq=${nq}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const qdMem = writeF64Array(new Float64Array(qd));
    const outMem = writeF64Array(new Float64Array(nq));

    const code = w.pino_coriolis_torques(model, ws, qMem.ptr, qdMem.ptr, outMem.ptr);
    if (code !== 0) throw new Error(`pino_coriolis_torques failed: ${code}`);

    const result = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + nq));

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return result;
  }

  // ---------------------------------------------------------------------------
  // Kinematics & analysis
  // ---------------------------------------------------------------------------

  function frameJacobian(model, ws, q, targetLink) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq) {
      throw new Error(`invalid q length: expected ${nq}, got ${q.length}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const outMem = writeF64Array(new Float64Array(6 * nq));

    const code = w.pino_frame_jacobian(model, ws, qMem.ptr, targetLink, outMem.ptr);
    if (code !== 0) throw new Error(`pino_frame_jacobian failed: ${code}`);

    const flat = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + 6 * nq));
    // Reshape row-major flat to 2D array (6 x nq)
    const J = [];
    for (let r = 0; r < 6; r++) {
      J.push(flat.slice(r * nq, r * nq + nq));
    }

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return J;
  }

  function centerOfMass(model, ws, q) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq) {
      throw new Error(`invalid q length: expected ${nq}, got ${q.length}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const outMem = writeF64Array(new Float64Array(3));

    const code = w.pino_center_of_mass(model, ws, qMem.ptr, outMem.ptr);
    if (code !== 0) throw new Error(`pino_center_of_mass failed: ${code}`);

    const com = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + 3));

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return com;
  }

  function energy(model, ws, q, qd, g = [0, 0, -9.81]) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq || qd.length !== nq) {
      throw new Error(`invalid state length: nq=${nq}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const qdMem = writeF64Array(new Float64Array(qd));
    const gMem = writeF64Array(new Float64Array(g));
    const keMem = writeF64Array(new Float64Array(1));
    const peMem = writeF64Array(new Float64Array(1));

    const code = w.pino_energy(model, ws, qMem.ptr, qdMem.ptr, gMem.ptr, keMem.ptr, peMem.ptr);
    if (code !== 0) throw new Error(`pino_energy failed: ${code}`);

    const ke = memoryF64()[keMem.ptr / 8];
    const pe = memoryF64()[peMem.ptr / 8];

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(gMem.ptr, gMem.bytes);
    freeBytes(keMem.ptr, keMem.bytes);
    freeBytes(peMem.ptr, peMem.bytes);
    return { kinetic: ke, potential: pe };
  }

  function computeAllTerms(model, ws, q, qd, g = [0, 0, -9.81]) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq || qd.length !== nq) {
      throw new Error(`invalid state length: nq=${nq}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const qdMem = writeF64Array(new Float64Array(qd));
    const gMem = writeF64Array(new Float64Array(g));
    const massMem = writeF64Array(new Float64Array(nq * nq));
    const biasMem = writeF64Array(new Float64Array(nq));
    const gravityMem = writeF64Array(new Float64Array(nq));
    const coriolisMem = writeF64Array(new Float64Array(nq));
    const comMem = writeF64Array(new Float64Array(3));
    const keMem = writeF64Array(new Float64Array(1));
    const peMem = writeF64Array(new Float64Array(1));

    const code = w.pino_compute_all_terms(
      model, ws,
      qMem.ptr, qdMem.ptr, gMem.ptr,
      massMem.ptr, biasMem.ptr, gravityMem.ptr, coriolisMem.ptr,
      comMem.ptr, keMem.ptr, peMem.ptr
    );
    if (code !== 0) throw new Error(`pino_compute_all_terms failed: ${code}`);

    const f64 = memoryF64();
    const mass = Array.from(f64.slice(massMem.ptr / 8, massMem.ptr / 8 + nq * nq));
    const M = [];
    for (let r = 0; r < nq; r++) {
      M.push(mass.slice(r * nq, r * nq + nq));
    }
    const bias = Array.from(f64.slice(biasMem.ptr / 8, biasMem.ptr / 8 + nq));
    const grav = Array.from(f64.slice(gravityMem.ptr / 8, gravityMem.ptr / 8 + nq));
    const cor = Array.from(f64.slice(coriolisMem.ptr / 8, coriolisMem.ptr / 8 + nq));
    const com = Array.from(f64.slice(comMem.ptr / 8, comMem.ptr / 8 + 3));
    const ke = f64[keMem.ptr / 8];
    const pe = f64[peMem.ptr / 8];

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(gMem.ptr, gMem.bytes);
    freeBytes(massMem.ptr, massMem.bytes);
    freeBytes(biasMem.ptr, biasMem.bytes);
    freeBytes(gravityMem.ptr, gravityMem.bytes);
    freeBytes(coriolisMem.ptr, coriolisMem.bytes);
    freeBytes(comMem.ptr, comMem.bytes);
    freeBytes(keMem.ptr, keMem.bytes);
    freeBytes(peMem.ptr, peMem.bytes);

    return { mass: M, bias, gravity: grav, coriolis: cor, com, kineticEnergy: ke, potentialEnergy: pe };
  }

  function forwardKinematicsPoses(model, ws, q) {
    const nq = Number(w.pino_model_nq(model));
    const nl = Number(w.pino_model_nlinks(model));
    if (q.length !== nq) {
      throw new Error(`invalid q length: expected ${nq}, got ${q.length}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const transMem = writeF64Array(new Float64Array(3 * nl));
    const rotMem = writeF64Array(new Float64Array(9 * nl));

    const code = w.pino_forward_kinematics_poses(model, ws, qMem.ptr, transMem.ptr, rotMem.ptr);
    if (code !== 0) throw new Error(`pino_forward_kinematics_poses failed: ${code}`);

    const f64 = memoryF64();
    const translations = [];
    const rotations = [];
    for (let l = 0; l < nl; l++) {
      translations.push(Array.from(f64.slice(transMem.ptr / 8 + l * 3, transMem.ptr / 8 + l * 3 + 3)));
      const rotFlat = Array.from(f64.slice(rotMem.ptr / 8 + l * 9, rotMem.ptr / 8 + l * 9 + 9));
      const R = [];
      for (let r = 0; r < 3; r++) {
        R.push(rotFlat.slice(r * 3, r * 3 + 3));
      }
      rotations.push(R);
    }

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(transMem.ptr, transMem.bytes);
    freeBytes(rotMem.ptr, rotMem.bytes);
    return { translations, rotations };
  }

  // ---------------------------------------------------------------------------
  // Batch operations
  // ---------------------------------------------------------------------------

  function rneaBatch(model, ws, qBatch, qdBatch, qddBatch, batchSize, g = [0, 0, -9.81]) {
    const nq = Number(w.pino_model_nq(model));
    const total = batchSize * nq;
    if (qBatch.length !== total || qdBatch.length !== total || qddBatch.length !== total) {
      throw new Error(`invalid batch length: expected ${total} per array`);
    }

    const qMem = writeF64Array(new Float64Array(qBatch));
    const qdMem = writeF64Array(new Float64Array(qdBatch));
    const qddMem = writeF64Array(new Float64Array(qddBatch));
    const gMem = writeF64Array(new Float64Array(g));
    const outMem = writeF64Array(new Float64Array(total));

    const code = w.pino_rnea_batch(model, ws, qMem.ptr, qdMem.ptr, qddMem.ptr, batchSize, gMem.ptr, outMem.ptr);
    if (code !== 0) throw new Error(`pino_rnea_batch failed: ${code}`);

    const tau = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + total));

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(qddMem.ptr, qddMem.bytes);
    freeBytes(gMem.ptr, gMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return tau;
  }

  function abaBatch(model, ws, qBatch, qdBatch, tauBatch, batchSize, g = [0, 0, -9.81]) {
    const nq = Number(w.pino_model_nq(model));
    const total = batchSize * nq;
    if (qBatch.length !== total || qdBatch.length !== total || tauBatch.length !== total) {
      throw new Error(`invalid batch length: expected ${total} per array`);
    }

    const qMem = writeF64Array(new Float64Array(qBatch));
    const qdMem = writeF64Array(new Float64Array(qdBatch));
    const tauMem = writeF64Array(new Float64Array(tauBatch));
    const gMem = writeF64Array(new Float64Array(g));
    const outMem = writeF64Array(new Float64Array(total));

    const code = w.pino_aba_batch(model, ws, qMem.ptr, qdMem.ptr, tauMem.ptr, batchSize, gMem.ptr, outMem.ptr);
    if (code !== 0) throw new Error(`pino_aba_batch failed: ${code}`);

    const qdd = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + total));

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(tauMem.ptr, tauMem.bytes);
    freeBytes(gMem.ptr, gMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return qdd;
  }

  function crbaBatch(model, ws, qBatch, batchSize) {
    const nq = Number(w.pino_model_nq(model));
    const totalQ = batchSize * nq;
    const totalM = batchSize * nq * nq;
    if (qBatch.length !== totalQ) {
      throw new Error(`invalid q batch length: expected ${totalQ}, got ${qBatch.length}`);
    }

    const qMem = writeF64Array(new Float64Array(qBatch));
    const outMem = writeF64Array(new Float64Array(totalM));

    const code = w.pino_crba_batch(model, ws, qMem.ptr, batchSize, outMem.ptr);
    if (code !== 0) throw new Error(`pino_crba_batch failed: ${code}`);

    const flat = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + totalM));
    // Split into batchSize matrices, each nq x nq
    const result = [];
    for (let b = 0; b < batchSize; b++) {
      const M = [];
      const base = b * nq * nq;
      for (let r = 0; r < nq; r++) {
        M.push(flat.slice(base + r * nq, base + r * nq + nq));
      }
      result.push(M);
    }

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return result;
  }

  function rolloutAbaEuler(model, ws, q0, qd0, tauBatch, batchSize, dt, g = [0, 0, -9.81]) {
    const nq = Number(w.pino_model_nq(model));
    const total = batchSize * nq;

    const q0Mem = writeF64Array(new Float64Array(q0));
    const qd0Mem = writeF64Array(new Float64Array(qd0));
    const tauMem = writeF64Array(new Float64Array(tauBatch));
    const gMem = writeF64Array(new Float64Array(g));
    const qOutMem = writeF64Array(new Float64Array(total));
    const qdOutMem = writeF64Array(new Float64Array(total));

    const code = w.pino_rollout_aba_euler(
      model, ws,
      q0Mem.ptr, qd0Mem.ptr, tauMem.ptr, batchSize, dt, gMem.ptr,
      qOutMem.ptr, qdOutMem.ptr
    );
    if (code !== 0) throw new Error(`pino_rollout_aba_euler failed: ${code}`);

    const qOut = Array.from(memoryF64().slice(qOutMem.ptr / 8, qOutMem.ptr / 8 + total));
    const qdOut = Array.from(memoryF64().slice(qdOutMem.ptr / 8, qdOutMem.ptr / 8 + total));

    freeBytes(q0Mem.ptr, q0Mem.bytes);
    freeBytes(qd0Mem.ptr, qd0Mem.bytes);
    freeBytes(tauMem.ptr, tauMem.bytes);
    freeBytes(gMem.ptr, gMem.bytes);
    freeBytes(qOutMem.ptr, qOutMem.bytes);
    freeBytes(qdOutMem.ptr, qdOutMem.bytes);
    return { q: qOut, qd: qdOut };
  }

  // ---------------------------------------------------------------------------
  // Contact dynamics
  // ---------------------------------------------------------------------------

  function contactConstrainedDynamics(model, ws, q, qd, tau, contacts, g = [0, 0, -9.81]) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq || qd.length !== nq || tau.length !== nq) {
      throw new Error(`invalid state length: nq=${nq}`);
    }
    const numContacts = contacts.length;

    const qMem = writeF64Array(new Float64Array(q));
    const qdMem = writeF64Array(new Float64Array(qd));
    const tauMem = writeF64Array(new Float64Array(tau));
    const gMem = writeF64Array(new Float64Array(g));

    // Pack contact data
    const linkIndices = new Int32Array(numContacts);
    const points = new Float64Array(numContacts * 3);
    const normals = new Float64Array(numContacts * 3);
    const accelBias = new Float64Array(numContacts);

    for (let i = 0; i < numContacts; i++) {
      const c = contacts[i];
      linkIndices[i] = c.linkIndex;
      points[3 * i] = c.point[0] || 0;
      points[3 * i + 1] = c.point[1] || 0;
      points[3 * i + 2] = c.point[2] || 0;
      normals[3 * i] = c.normal[0] || 0;
      normals[3 * i + 1] = c.normal[1] || 0;
      normals[3 * i + 2] = c.normal[2] || 0;
      accelBias[i] = c.accelBias || 0;
    }

    const liMem = writeI32Array(linkIndices);
    const ptMem = writeF64Array(points);
    const nmMem = writeF64Array(normals);
    const abMem = writeF64Array(accelBias);
    const qddOutMem = writeF64Array(new Float64Array(nq));
    const lambdaOutMem = writeF64Array(new Float64Array(numContacts));

    const code = w.pino_contact_constrained_dynamics(
      model, ws,
      qMem.ptr, qdMem.ptr, tauMem.ptr, gMem.ptr,
      numContacts, liMem.ptr, ptMem.ptr, nmMem.ptr, abMem.ptr,
      qddOutMem.ptr, lambdaOutMem.ptr
    );
    if (code !== 0) throw new Error(`pino_contact_constrained_dynamics failed: ${code}`);

    const qdd = Array.from(memoryF64().slice(qddOutMem.ptr / 8, qddOutMem.ptr / 8 + nq));
    const lambda = Array.from(memoryF64().slice(lambdaOutMem.ptr / 8, lambdaOutMem.ptr / 8 + numContacts));

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(tauMem.ptr, tauMem.bytes);
    freeBytes(gMem.ptr, gMem.bytes);
    freeBytes(liMem.ptr, liMem.bytes);
    freeBytes(ptMem.ptr, ptMem.bytes);
    freeBytes(nmMem.ptr, nmMem.bytes);
    freeBytes(abMem.ptr, abMem.bytes);
    freeBytes(qddOutMem.ptr, qddOutMem.bytes);
    freeBytes(lambdaOutMem.ptr, lambdaOutMem.bytes);
    return { qdd, lambda };
  }

  function applyContactImpulse(model, ws, q, qdMinus, restitution, contacts) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq || qdMinus.length !== nq) {
      throw new Error(`invalid state length: nq=${nq}`);
    }
    const numContacts = contacts.length;

    const qMem = writeF64Array(new Float64Array(q));
    const qdMem = writeF64Array(new Float64Array(qdMinus));

    const linkIndices = new Int32Array(numContacts);
    const points = new Float64Array(numContacts * 3);
    const normals = new Float64Array(numContacts * 3);

    for (let i = 0; i < numContacts; i++) {
      const c = contacts[i];
      linkIndices[i] = c.linkIndex;
      points[3 * i] = c.point[0] || 0;
      points[3 * i + 1] = c.point[1] || 0;
      points[3 * i + 2] = c.point[2] || 0;
      normals[3 * i] = c.normal[0] || 0;
      normals[3 * i + 1] = c.normal[1] || 0;
      normals[3 * i + 2] = c.normal[2] || 0;
    }

    const liMem = writeI32Array(linkIndices);
    const ptMem = writeF64Array(points);
    const nmMem = writeF64Array(normals);
    const qdPlusMem = writeF64Array(new Float64Array(nq));
    const impulseMem = writeF64Array(new Float64Array(numContacts));

    const code = w.pino_apply_contact_impulse(
      model, ws, qMem.ptr, qdMem.ptr, restitution,
      numContacts, liMem.ptr, ptMem.ptr, nmMem.ptr,
      qdPlusMem.ptr, impulseMem.ptr
    );
    if (code !== 0) throw new Error(`pino_apply_contact_impulse failed: ${code}`);

    const qdPlus = Array.from(memoryF64().slice(qdPlusMem.ptr / 8, qdPlusMem.ptr / 8 + nq));
    const impulse = Array.from(memoryF64().slice(impulseMem.ptr / 8, impulseMem.ptr / 8 + numContacts));

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(liMem.ptr, liMem.bytes);
    freeBytes(ptMem.ptr, ptMem.bytes);
    freeBytes(nmMem.ptr, nmMem.bytes);
    freeBytes(qdPlusMem.ptr, qdPlusMem.bytes);
    freeBytes(impulseMem.ptr, impulseMem.bytes);
    return { qdPlus, impulse };
  }

  function contactJacobianNormal(model, ws, q, contacts) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq) {
      throw new Error(`invalid q length: expected ${nq}, got ${q.length}`);
    }
    const numContacts = contacts.length;

    const qMem = writeF64Array(new Float64Array(q));

    const linkIndices = new Int32Array(numContacts);
    const points = new Float64Array(numContacts * 3);
    const normals = new Float64Array(numContacts * 3);

    for (let i = 0; i < numContacts; i++) {
      const c = contacts[i];
      linkIndices[i] = c.linkIndex;
      points[3 * i] = c.point[0] || 0;
      points[3 * i + 1] = c.point[1] || 0;
      points[3 * i + 2] = c.point[2] || 0;
      normals[3 * i] = c.normal[0] || 0;
      normals[3 * i + 1] = c.normal[1] || 0;
      normals[3 * i + 2] = c.normal[2] || 0;
    }

    const liMem = writeI32Array(linkIndices);
    const ptMem = writeF64Array(points);
    const nmMem = writeF64Array(normals);
    const outMem = writeF64Array(new Float64Array(numContacts * nq));

    const code = w.pino_contact_jacobian_normal(
      model, ws, qMem.ptr,
      numContacts, liMem.ptr, ptMem.ptr, nmMem.ptr,
      outMem.ptr
    );
    if (code !== 0) throw new Error(`pino_contact_jacobian_normal failed: ${code}`);

    const flat = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + numContacts * nq));
    // Reshape to numContacts x nq
    const J = [];
    for (let k = 0; k < numContacts; k++) {
      J.push(flat.slice(k * nq, k * nq + nq));
    }

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(liMem.ptr, liMem.bytes);
    freeBytes(ptMem.ptr, ptMem.bytes);
    freeBytes(nmMem.ptr, nmMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return J;
  }

  // ---------------------------------------------------------------------------
  // Collision
  // ---------------------------------------------------------------------------

  function createCollisionModel(spheres) {
    const numSpheres = spheres.length;
    if (numSpheres === 0) throw new Error("spheres array must not be empty");

    const linkIndices = new Int32Array(numSpheres);
    const centers = new Float64Array(numSpheres * 3);
    const radii = new Float64Array(numSpheres);

    for (let i = 0; i < numSpheres; i++) {
      const s = spheres[i];
      linkIndices[i] = s.linkIndex;
      centers[3 * i] = s.center[0] || 0;
      centers[3 * i + 1] = s.center[1] || 0;
      centers[3 * i + 2] = s.center[2] || 0;
      radii[i] = s.radius;
    }

    const liMem = writeI32Array(linkIndices);
    const ctMem = writeF64Array(centers);
    const rdMem = writeF64Array(radii);

    const collision = w.pino_collision_model_create(numSpheres, liMem.ptr, ctMem.ptr, rdMem.ptr);

    freeBytes(liMem.ptr, liMem.bytes);
    freeBytes(ctMem.ptr, ctMem.bytes);
    freeBytes(rdMem.ptr, rdMem.bytes);

    if (!collision) throw new Error("pino_collision_model_create failed");
    return collision;
  }

  /**
   * Create collision model with multiple geometry types.
   * @param {Array} geometries - Array of geometry objects.
   *   Each: { type: 0-4, linkIndex, center: [x,y,z], params: [p0,p1,p2] }
   *   type: 0=sphere(p0=radius), 1=box(p0,p1,p2=half_extents),
   *         2=capsule(p0=half_length,p1=radius), 3=cylinder(p0=half_length,p1=radius),
   *         4=mesh_approx(p0,p1,p2=half_extents)
   * @param {Object} [filter] - { ignoreSameLink: bool, ignoreParentChild: bool }
   */
  function createCollisionModelGeometries(geometries, filter = {}) {
    const numGeoms = geometries.length;
    if (numGeoms === 0) throw new Error("geometries array must not be empty");

    const geomTypes = new Int32Array(numGeoms);
    const linkIndices = new Int32Array(numGeoms);
    const centers = new Float64Array(numGeoms * 3);
    const params = new Float64Array(numGeoms * 3);
    const flags = new Int32Array(2);
    flags[0] = filter.ignoreSameLink ? 1 : 0;
    flags[1] = filter.ignoreParentChild ? 1 : 0;

    for (let i = 0; i < numGeoms; i++) {
      const g = geometries[i];
      geomTypes[i] = g.type || 0;
      linkIndices[i] = g.linkIndex;
      centers[3 * i] = g.center[0] || 0;
      centers[3 * i + 1] = g.center[1] || 0;
      centers[3 * i + 2] = g.center[2] || 0;
      params[3 * i] = g.params[0] || 0;
      params[3 * i + 1] = g.params[1] || 0;
      params[3 * i + 2] = g.params[2] || 0;
    }

    const gtMem = writeI32Array(geomTypes);
    const liMem = writeI32Array(linkIndices);
    const ctMem = writeF64Array(centers);
    const pmMem = writeF64Array(params);
    const flMem = writeI32Array(flags);

    const collision = w.pino_collision_model_create_geometries(
      numGeoms, gtMem.ptr, liMem.ptr, ctMem.ptr, pmMem.ptr, flMem.ptr
    );

    freeBytes(gtMem.ptr, gtMem.bytes);
    freeBytes(liMem.ptr, liMem.bytes);
    freeBytes(ctMem.ptr, ctMem.bytes);
    freeBytes(pmMem.ptr, pmMem.bytes);
    freeBytes(flMem.ptr, flMem.bytes);

    if (!collision) throw new Error("pino_collision_model_create_geometries failed");
    return collision;
  }

  function disposeCollisionModel(collision) {
    w.pino_collision_model_free(collision);
  }

  function collisionMinDistance(model, collision, ws, q) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq) {
      throw new Error(`invalid q length: expected ${nq}, got ${q.length}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const distMem = writeF64Array(new Float64Array(1));
    const pairMem = writeI32Array(new Int32Array(2));

    const code = w.pino_collision_min_distance(model, collision, ws, qMem.ptr, distMem.ptr, pairMem.ptr);
    if (code !== 0) throw new Error(`pino_collision_min_distance failed: ${code}`);

    const distance = memoryF64()[distMem.ptr / 8];
    const i32 = memoryI32();
    const pair = [i32[pairMem.ptr / 4], i32[pairMem.ptr / 4 + 1]];

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(distMem.ptr, distMem.bytes);
    freeBytes(pairMem.ptr, pairMem.bytes);
    return { distance, pair };
  }

  function collisionQueryDetails(model, collision, ws, q, maxResults) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq) {
      throw new Error(`invalid q length: expected ${nq}, got ${q.length}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const countMem = writeI32Array(new Int32Array(1)); // usize on wasm32 = 4 bytes
    const pairMem = writeI32Array(new Int32Array(maxResults * 2));
    const distMem = writeF64Array(new Float64Array(maxResults));
    const normalMem = writeF64Array(new Float64Array(maxResults * 3));
    const paMem = writeF64Array(new Float64Array(maxResults * 3));
    const pbMem = writeF64Array(new Float64Array(maxResults * 3));
    const penMem = writeF64Array(new Float64Array(maxResults));
    const collMem = writeI32Array(new Int32Array(maxResults));

    const code = w.pino_collision_query_details(
      model, collision, ws, qMem.ptr, maxResults,
      countMem.ptr, pairMem.ptr, distMem.ptr, normalMem.ptr,
      paMem.ptr, pbMem.ptr, penMem.ptr, collMem.ptr
    );
    if (code !== 0) throw new Error(`pino_collision_query_details failed: ${code}`);

    const count = memoryI32()[countMem.ptr / 4];
    const f64 = memoryF64();
    const i32 = memoryI32();
    const results = [];
    for (let i = 0; i < count; i++) {
      results.push({
        pair: [i32[pairMem.ptr / 4 + 2 * i], i32[pairMem.ptr / 4 + 2 * i + 1]],
        distance: f64[distMem.ptr / 8 + i],
        normal: [f64[normalMem.ptr / 8 + 3 * i], f64[normalMem.ptr / 8 + 3 * i + 1], f64[normalMem.ptr / 8 + 3 * i + 2]],
        pointA: [f64[paMem.ptr / 8 + 3 * i], f64[paMem.ptr / 8 + 3 * i + 1], f64[paMem.ptr / 8 + 3 * i + 2]],
        pointB: [f64[pbMem.ptr / 8 + 3 * i], f64[pbMem.ptr / 8 + 3 * i + 1], f64[pbMem.ptr / 8 + 3 * i + 2]],
        penetration: f64[penMem.ptr / 8 + i],
        isColliding: i32[collMem.ptr / 4 + i] !== 0,
      });
    }

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(countMem.ptr, countMem.bytes);
    freeBytes(pairMem.ptr, pairMem.bytes);
    freeBytes(distMem.ptr, distMem.bytes);
    freeBytes(normalMem.ptr, normalMem.bytes);
    freeBytes(paMem.ptr, paMem.bytes);
    freeBytes(pbMem.ptr, pbMem.bytes);
    freeBytes(penMem.ptr, penMem.bytes);
    freeBytes(collMem.ptr, collMem.bytes);
    return results;
  }

  // ---------------------------------------------------------------------------
  // Centroidal
  // ---------------------------------------------------------------------------

  function centroidalMomentum(model, ws, q, qd) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq || qd.length !== nq) {
      throw new Error(`invalid state length: nq=${nq}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const qdMem = writeF64Array(new Float64Array(qd));
    const momMem = writeF64Array(new Float64Array(6));
    const comMem = writeF64Array(new Float64Array(3));

    const code = w.pino_centroidal_momentum(model, ws, qMem.ptr, qdMem.ptr, momMem.ptr, comMem.ptr);
    if (code !== 0) throw new Error(`pino_centroidal_momentum failed: ${code}`);

    const f64 = memoryF64();
    const momentum = {
      linear: [f64[momMem.ptr / 8], f64[momMem.ptr / 8 + 1], f64[momMem.ptr / 8 + 2]],
      angular: [f64[momMem.ptr / 8 + 3], f64[momMem.ptr / 8 + 4], f64[momMem.ptr / 8 + 5]],
    };
    const com = [f64[comMem.ptr / 8], f64[comMem.ptr / 8 + 1], f64[comMem.ptr / 8 + 2]];

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(momMem.ptr, momMem.bytes);
    freeBytes(comMem.ptr, comMem.bytes);
    return { momentum, com };
  }

  function centroidalMap(model, ws, q) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq) {
      throw new Error(`invalid q length: expected ${nq}, got ${q.length}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const outMem = writeF64Array(new Float64Array(6 * nq));

    const code = w.pino_centroidal_map(model, ws, qMem.ptr, outMem.ptr);
    if (code !== 0) throw new Error(`pino_centroidal_map failed: ${code}`);

    const flat = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + 6 * nq));
    const Ag = [];
    for (let r = 0; r < 6; r++) {
      Ag.push(flat.slice(r * nq, r * nq + nq));
    }

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return Ag;
  }

  function centroidalFullTerms(model, ws, q, qd, qdd) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq || qd.length !== nq || qdd.length !== nq) {
      throw new Error(`invalid state length: nq=${nq}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const qdMem = writeF64Array(new Float64Array(qd));
    const qddMem = writeF64Array(new Float64Array(qdd));
    const agMem = writeF64Array(new Float64Array(6 * nq));
    const dagMem = writeF64Array(new Float64Array(6 * nq * nq));
    const momMem = writeF64Array(new Float64Array(6));
    const hdotMem = writeF64Array(new Float64Array(6));

    const code = w.pino_centroidal_full_terms(
      model, ws, qMem.ptr, qdMem.ptr, qddMem.ptr,
      agMem.ptr, dagMem.ptr, momMem.ptr, hdotMem.ptr
    );
    if (code !== 0) throw new Error(`pino_centroidal_full_terms failed: ${code}`);

    const f64 = memoryF64();
    const agFlat = Array.from(f64.slice(agMem.ptr / 8, agMem.ptr / 8 + 6 * nq));
    const Ag = [];
    for (let r = 0; r < 6; r++) {
      Ag.push(agFlat.slice(r * nq, r * nq + nq));
    }
    const dagFlat = Array.from(f64.slice(dagMem.ptr / 8, dagMem.ptr / 8 + 6 * nq * nq));
    const dAg = [];
    for (let r = 0; r < 6 * nq; r++) {
      dAg.push(dagFlat.slice(r * nq, r * nq + nq));
    }
    const momentum = {
      linear: [f64[momMem.ptr / 8], f64[momMem.ptr / 8 + 1], f64[momMem.ptr / 8 + 2]],
      angular: [f64[momMem.ptr / 8 + 3], f64[momMem.ptr / 8 + 4], f64[momMem.ptr / 8 + 5]],
    };
    const momentumRate = {
      linear: [f64[hdotMem.ptr / 8], f64[hdotMem.ptr / 8 + 1], f64[hdotMem.ptr / 8 + 2]],
      angular: [f64[hdotMem.ptr / 8 + 3], f64[hdotMem.ptr / 8 + 4], f64[hdotMem.ptr / 8 + 5]],
    };

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(qddMem.ptr, qddMem.bytes);
    freeBytes(agMem.ptr, agMem.bytes);
    freeBytes(dagMem.ptr, dagMem.bytes);
    freeBytes(momMem.ptr, momMem.bytes);
    freeBytes(hdotMem.ptr, hdotMem.bytes);
    return { ag: Ag, dagDq: dAg, momentum, momentumRate };
  }

  // ---------------------------------------------------------------------------
  // Regressors
  // ---------------------------------------------------------------------------

  function inverseDynamicsRegressor(model, q, qd, qdd, g = [0, 0, -9.81]) {
    const nq = Number(w.pino_model_nq(model));
    const nl = Number(w.pino_model_nlinks(model));
    const p = 10 * nl;
    if (q.length !== nq || qd.length !== nq || qdd.length !== nq) {
      throw new Error(`invalid state length: nq=${nq}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const qdMem = writeF64Array(new Float64Array(qd));
    const qddMem = writeF64Array(new Float64Array(qdd));
    const gMem = writeF64Array(new Float64Array(g));
    const outMem = writeF64Array(new Float64Array(nq * p));

    const code = w.pino_inverse_dynamics_regressor(
      model, qMem.ptr, qdMem.ptr, qddMem.ptr, gMem.ptr, outMem.ptr
    );
    if (code !== 0) throw new Error(`pino_inverse_dynamics_regressor failed: ${code}`);

    const flat = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + nq * p));
    const Y = [];
    for (let r = 0; r < nq; r++) {
      Y.push(flat.slice(r * p, r * p + p));
    }

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(qddMem.ptr, qddMem.bytes);
    freeBytes(gMem.ptr, gMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return Y;
  }

  function kineticEnergyRegressor(model, q, qd) {
    const nq = Number(w.pino_model_nq(model));
    const nl = Number(w.pino_model_nlinks(model));
    const p = 10 * nl;
    if (q.length !== nq || qd.length !== nq) {
      throw new Error(`invalid state length: nq=${nq}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const qdMem = writeF64Array(new Float64Array(qd));
    const outMem = writeF64Array(new Float64Array(p));

    const code = w.pino_kinetic_energy_regressor(model, qMem.ptr, qdMem.ptr, outMem.ptr);
    if (code !== 0) throw new Error(`pino_kinetic_energy_regressor failed: ${code}`);

    const y = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + p));

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return y;
  }

  // ---------------------------------------------------------------------------
  // Derivatives
  // ---------------------------------------------------------------------------

  function rneaSecondOrderDerivatives(model, ws, q, qd, qdd, g = [0, 0, -9.81]) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq || qd.length !== nq || qdd.length !== nq) {
      throw new Error(`invalid state length: nq=${nq}`);
    }
    const cube = nq * nq * nq;

    const qMem = writeF64Array(new Float64Array(q));
    const qdMem = writeF64Array(new Float64Array(qd));
    const qddMem = writeF64Array(new Float64Array(qdd));
    const gMem = writeF64Array(new Float64Array(g));
    const d2qMem = writeF64Array(new Float64Array(cube));
    const d2vMem = writeF64Array(new Float64Array(cube));
    const d2uMem = writeF64Array(new Float64Array(cube));

    const code = w.pino_rnea_second_order_derivatives(
      model, ws, qMem.ptr, qdMem.ptr, qddMem.ptr, gMem.ptr,
      d2qMem.ptr, d2vMem.ptr, d2uMem.ptr
    );
    if (code !== 0) throw new Error(`pino_rnea_second_order_derivatives failed: ${code}`);

    const d2TauDq2 = Array.from(memoryF64().slice(d2qMem.ptr / 8, d2qMem.ptr / 8 + cube));
    const d2TauDqd2 = Array.from(memoryF64().slice(d2vMem.ptr / 8, d2vMem.ptr / 8 + cube));
    const d2TauDqdd2 = Array.from(memoryF64().slice(d2uMem.ptr / 8, d2uMem.ptr / 8 + cube));

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(qddMem.ptr, qddMem.bytes);
    freeBytes(gMem.ptr, gMem.bytes);
    freeBytes(d2qMem.ptr, d2qMem.bytes);
    freeBytes(d2vMem.ptr, d2vMem.bytes);
    freeBytes(d2uMem.ptr, d2uMem.bytes);
    return { d2TauDq2, d2TauDqd2, d2TauDqdd2 };
  }

  // ---------------------------------------------------------------------------
  // Constrained
  // ---------------------------------------------------------------------------

  function constrainedAbaLockedJoints(model, ws, q, qd, tau, lockedMask, g = [0, 0, -9.81]) {
    const nq = Number(w.pino_model_nq(model));
    if (q.length !== nq || qd.length !== nq || tau.length !== nq || lockedMask.length !== nq) {
      throw new Error(`invalid state length: nq=${nq}`);
    }

    const qMem = writeF64Array(new Float64Array(q));
    const qdMem = writeF64Array(new Float64Array(qd));
    const tauMem = writeF64Array(new Float64Array(tau));
    const maskMem = writeI32Array(new Int32Array(lockedMask));
    const gMem = writeF64Array(new Float64Array(g));
    const outMem = writeF64Array(new Float64Array(nq));

    const code = w.pino_constrained_aba_locked_joints(
      model, ws, qMem.ptr, qdMem.ptr, tauMem.ptr, maskMem.ptr, gMem.ptr, outMem.ptr
    );
    if (code !== 0) throw new Error(`pino_constrained_aba_locked_joints failed: ${code}`);

    const qdd = Array.from(memoryF64().slice(outMem.ptr / 8, outMem.ptr / 8 + nq));

    freeBytes(qMem.ptr, qMem.bytes);
    freeBytes(qdMem.ptr, qdMem.bytes);
    freeBytes(tauMem.ptr, tauMem.bytes);
    freeBytes(maskMem.ptr, maskMem.bytes);
    freeBytes(gMem.ptr, gMem.bytes);
    freeBytes(outMem.ptr, outMem.bytes);
    return qdd;
  }

  return {
    exports: w,

    // Model lifecycle
    createModelFromJson,
    createModelFromUrdf,
    createModelFromSdf,
    createModelFromMjcf,
    createModel,
    modelToJson,
    modelToUrdf,
    modelToSdf,
    modelToMjcf,
    modelNq,
    modelNlinks,
    newWorkspace,
    disposeModel,
    disposeWorkspace,

    // Core dynamics
    rnea,
    aba,
    crba,
    gravityTorques,
    coriolisTorques,

    // Kinematics & analysis
    frameJacobian,
    centerOfMass,
    energy,
    computeAllTerms,
    forwardKinematicsPoses,

    // Batch operations
    rneaBatch,
    abaBatch,
    crbaBatch,
    rolloutAbaEuler,

    // Contact dynamics
    contactConstrainedDynamics,
    applyContactImpulse,
    contactJacobianNormal,

    // Collision
    createCollisionModel,
    createCollisionModelGeometries,
    disposeCollisionModel,
    collisionMinDistance,
    collisionQueryDetails,

    // Centroidal
    centroidalMomentum,
    centroidalMap,
    centroidalFullTerms,

    // Regressors
    inverseDynamicsRegressor,
    kineticEnergyRegressor,

    // Derivatives
    rneaSecondOrderDerivatives,

    // Constrained
    constrainedAbaLockedJoints,

    // Memory helpers
    allocBytes,
    freeBytes,
    memoryU8,
    memoryF64,
    memoryI32,
    memoryU32,
  };
}
