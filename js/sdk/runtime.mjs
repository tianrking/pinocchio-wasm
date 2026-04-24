export function createRuntime(w) {
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
    const view = arr instanceof Float64Array ? arr : new Float64Array(arr);
    const bytes = view.length * 8;
    const ptr = allocBytes(bytes);
    memoryF64().set(view, ptr / 8);
    return { ptr, bytes };
  }

  function writeI32Array(arr) {
    const view = arr instanceof Int32Array ? arr : new Int32Array(arr);
    const bytes = view.length * 4;
    const ptr = allocBytes(bytes);
    memoryI32().set(view, ptr / 4);
    return { ptr, bytes };
  }

  function readF64Array(ptr, len) {
    return Array.from(memoryF64().slice(ptr / 8, ptr / 8 + len));
  }

  function withF64Array(arr, fn) {
    const mem = writeF64Array(arr);
    try {
      return fn(mem.ptr, mem);
    } finally {
      freeBytes(mem.ptr, mem.bytes);
    }
  }

  function readExportedString(outPtrPtr, outLenPtr) {
    const u32 = memoryU32();
    const strPtr = Number(u32[outPtrPtr / 4]);
    const strLen = Number(u32[outLenPtr / 4]);
    const bytes = memoryU8().slice(strPtr, strPtr + strLen);
    freeBytes(strPtr, strLen);
    return textDecoder.decode(bytes);
  }

  return {
    textEncoder,
    textDecoder,
    memoryU8,
    memoryF64,
    memoryI32,
    memoryU32,
    allocBytes,
    freeBytes,
    writeBytes,
    writeF64Array,
    writeI32Array,
    readF64Array,
    withF64Array,
    readExportedString,
  };
}
