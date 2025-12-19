from __future__ import annotations

import numpy as np


def rmse(y: np.ndarray, yref: np.ndarray) -> float:
    y = np.asarray(y, dtype=float)
    yref = np.asarray(yref, dtype=float)
    return float(np.sqrt(np.mean((y - yref) ** 2)))


def ise(y: np.ndarray, yref: np.ndarray) -> float:
    y = np.asarray(y, dtype=float)
    yref = np.asarray(yref, dtype=float)
    return float(np.sum((y - yref) ** 2))


def itae(t: np.ndarray, y: np.ndarray, yref: np.ndarray) -> float:
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    yref = np.asarray(yref, dtype=float)
    return float(np.sum(t * np.abs(y - yref)))


def overshoot_percent(y: np.ndarray, yref_final: float) -> float:
    y = np.asarray(y, dtype=float)
    if yref_final == 0:
        return 0.0
    peak = float(np.max(y))
    return max(0.0, (peak - yref_final) / abs(yref_final) * 100.0)


def settling_time(t: np.ndarray, y: np.ndarray, yref_final: float, tol_pct: float = 2.0) -> float:
    """Settling time (s) using ±tol_pct% band around yref_final.

    If never settles, return inf.
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    if t.size == 0:
        return float("inf")
    band = abs(yref_final) * (tol_pct / 100.0)
    lo = yref_final - band
    hi = yref_final + band
    inside = (y >= lo) & (y <= hi)
    for i in range(t.size):
        if inside[i:].all():
            return float(t[i] - t[0])
    return float("inf")
