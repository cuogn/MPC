from __future__ import annotations
from dataclasses import dataclass

@dataclass
class PIDParams:
    kp: float
    ki: float
    kd: float = 0.0
    kaw: float = 0.0  # anti-windup back-calc gain

class PIDSpeed:
    """PID speed controller producing iq_ref.
    Anti-windup via back-calculation when saturation occurs.
    """
    def __init__(self, p: PIDParams, iq_min: float, iq_max: float):
        self.p = p
        self.iq_min = float(iq_min)
        self.iq_max = float(iq_max)
        self.i_int = 0.0
        self.e_prev = 0.0

    def reset(self):
        self.i_int = 0.0
        self.e_prev = 0.0

    def step(self, e: float, dt: float) -> tuple[float, float, float]:
        # derivative
        de = (e - self.e_prev) / dt if dt > 0 else 0.0
        self.e_prev = e

        # unsat output
        u_unsat = self.p.kp * e + self.i_int + self.p.kd * de

        # saturate
        u_sat = max(self.iq_min, min(self.iq_max, u_unsat))

        # anti-windup back-calculation
        aw = 0.0
        if self.p.kaw > 0.0:
            aw = (u_sat - u_unsat) * self.p.kaw

        # integrate (note: integrate error + aw)
        self.i_int += (self.p.ki * (e + aw) * dt)

        # recompute (optional) - keep u_sat as final command
        return u_sat, u_unsat, aw
