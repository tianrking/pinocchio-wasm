import ctypes
from pathlib import Path


class PinocchioWasm:
    def __init__(self, lib_path: str | None = None):
        if lib_path is None:
            base = Path(__file__).resolve().parents[2]
            candidates = [
                base / "target" / "release" / "libpinocchio_wasm.dylib",
                base / "target" / "release" / "libpinocchio_wasm.so",
                base / "target" / "release" / "pinocchio_wasm.dll",
            ]
            lib_file = next((p for p in candidates if p.exists()), None)
            if lib_file is None:
                raise FileNotFoundError("build shared library first: cargo build --release")
            lib_path = str(lib_file)

        self.lib = ctypes.CDLL(lib_path)
        self.lib.pino_model_create_from_json.restype = ctypes.c_void_p
        self.lib.pino_workspace_new.restype = ctypes.c_void_p

    def model_from_json(self, json_str: str) -> int:
        b = json_str.encode("utf-8")
        ptr = self.lib.pino_model_create_from_json(b, len(b))
        if ptr == 0:
            raise RuntimeError("failed to create model")
        return int(ptr)

    def workspace_new(self, model: int) -> int:
        ptr = self.lib.pino_workspace_new(ctypes.c_void_p(model))
        if ptr == 0:
            raise RuntimeError("failed to create workspace")
        return int(ptr)

    def free_model(self, model: int) -> None:
        self.lib.pino_model_free(ctypes.c_void_p(model))

    def free_workspace(self, ws: int) -> None:
        self.lib.pino_workspace_free(ctypes.c_void_p(ws))

    def aba(self, model: int, ws: int, q: list[float], qd: list[float], tau: list[float], gravity: list[float]) -> list[float]:
        n = len(q)
        arr = ctypes.c_double * n
        garr = ctypes.c_double * 3
        out = arr()
        s = self.lib.pino_aba(
            ctypes.c_void_p(model),
            ctypes.c_void_p(ws),
            arr(*q),
            arr(*qd),
            arr(*tau),
            garr(*gravity),
            out,
        )
        if s != 0:
            raise RuntimeError(f"pino_aba failed: {s}")
        return [out[i] for i in range(n)]
