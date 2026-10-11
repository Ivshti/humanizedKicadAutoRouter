"""The C side: A* search and an exact distance transform, built on first use."""
import ctypes
import os
import subprocess

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def _load():
    src = os.path.join(HERE, "astar.c")
    out = os.path.join(HERE, "build", "libastar.so")
    if not os.path.exists(out) or os.path.getmtime(out) < os.path.getmtime(src):
        os.makedirs(os.path.dirname(out), exist_ok=True)
        subprocess.check_call(["cc", "-O3", "-shared", "-fPIC", "-o", out, src])
    k = ctypes.CDLL(out)
    P = ctypes.c_void_p
    k.astar.restype = ctypes.c_int
    k.astar.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, P, P, ctypes.c_float, P, P, P,
                        ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float,
                        ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, P, ctypes.c_int,
                        ctypes.c_int64, P, P, ctypes.c_float]
    k.edt2d.restype = None
    k.edt2d.argtypes = [P, ctypes.c_int, ctypes.c_int, P]
    return k


kernel = _load()


def edt(feature):
    """Distance in cells from every cell to the nearest True cell of `feature`."""
    feat = np.ascontiguousarray(feature, dtype=np.uint8)
    out = np.empty(feat.shape, np.float32)
    kernel.edt2d(feat.ctypes.data, feat.shape[0], feat.shape[1], out.ctypes.data)
    return np.sqrt(out, out=out)
