from __future__ import annotations
from dataclasses import dataclass
import numpy as np

@dataclass
class PIParams:
    kp: float
    ki: float
    kaw: float = 0.0  # anti-windup gain

class PIController:
    def __init__(self, p: PIParams):
        self.p = p
        self.int = 0.0

    def reset(self):
        self.int = 0.0

    def update_int(self, e: float, dt: float, u_unsat: float | None, u_sat: float | None):
        aw = 0.0
        if u_unsat is not None and u_sat is not None and self.p.kaw > 0:
            aw = (u_sat - u_unsat) * self.p.kaw
        self.int += (e + aw) * self.p.ki * dt

    def out(self, e: float) -> float:
        return self.p.kp * e + self.int

def sat_vec_mag(vd: float, vq: float, vmax: float):
    mag = float(np.hypot(vd, vq))
    if mag <= vmax or mag <= 1e-12:
        return vd, vq
    s = vmax / mag
    return vd*s, vq*s

class CurrentLoopPI:
    def __init__(self, Rs: float, Lsigma: float, pi_d: PIParams, pi_q: PIParams, vmax: float):
        self.Rs = float(Rs)
        self.Lsigma = float(Lsigma)
        self.vmax = float(vmax)
        self.pi_d = PIController(pi_d)
        self.pi_q = PIController(pi_q)

    def reset(self):
        self.pi_d.reset()
        self.pi_q.reset()

    def step(self, id_ref: float, iq_ref: float, id_meas: float, iq_meas: float, omega_e: float, dt: float):
        ed = id_ref - id_meas
        eq = iq_ref - iq_meas

        vd_pi = self.pi_d.out(ed)
        vq_pi = self.pi_q.out(eq)

        # Unsat command with simple decoupling
        vd_unsat = vd_pi + self.Rs*id_meas - omega_e*self.Lsigma*iq_meas
        vq_unsat = vq_pi + self.Rs*iq_meas + omega_e*self.Lsigma*id_meas

        vd_sat, vq_sat = sat_vec_mag(vd_unsat, vq_unsat, self.vmax)

        # Anti-windup update (uses sat error per axis)
        self.pi_d.update_int(ed, dt, vd_unsat, vd_sat)
        self.pi_q.update_int(eq, dt, vq_unsat, vq_sat)

        return {
            "vd": vd_sat, "vq": vq_sat,
            "vd_unsat": vd_unsat, "vq_unsat": vq_unsat,
            "ed": ed, "eq": eq
        }
