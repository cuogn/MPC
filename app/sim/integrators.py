from __future__ import annotations
import numpy as np
from typing import Callable

def rk4_step(f: Callable[[np.ndarray], np.ndarray], x: np.ndarray, dt: float) -> np.ndarray:
    k1 = f(x)
    k2 = f(x + 0.5*dt*k1)
    k3 = f(x + 0.5*dt*k2)
    k4 = f(x + dt*k3)
    return x + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)
