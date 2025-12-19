from __future__ import annotations
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class IMParams:
    Rs: float
    Rr: float
    Ls: float
    Lr: float
    Lm: float
    p: int
    J: float
    B: float

    def det(self) -> float:
        return self.Ls * self.Lr - self.Lm**2

class InductionMotorDQ:
    """IM model in synchronous dq with flux states.
    x = [psi_ds, psi_qs, psi_dr, psi_qr, omega_m]
    """
    def __init__(self, params: IMParams, x0: np.ndarray | None = None):
        self.p = params
        if params.det() <= 0:
            raise ValueError("Invalid inductances: Ls*Lr must be > Lm^2.")
        self.x = np.zeros(5, dtype=float) if x0 is None else np.array(x0, dtype=float)

    def currents_from_flux(self, psi_ds, psi_qs, psi_dr, psi_qr):
        p = self.p
        det = p.det()
        ids = (p.Lr * psi_ds - p.Lm * psi_dr) / det
        idr = (-p.Lm * psi_ds + p.Ls * psi_dr) / det
        iqs = (p.Lr * psi_qs - p.Lm * psi_qr) / det
        iqr = (-p.Lm * psi_qs + p.Ls * psi_qr) / det
        return ids, iqs, idr, iqr

    def torque_e(self, psi_ds, psi_qs, ids, iqs) -> float:
        return 1.5 * self.p.p * (psi_ds * iqs - psi_qs * ids)

    def f(self, x: np.ndarray, v_ds: float, v_qs: float, omega_e: float, T_L: float) -> np.ndarray:
        psi_ds, psi_qs, psi_dr, psi_qr, omega_m = x
        p = self.p
        omega_r_e = p.p * omega_m
        omega_sl = omega_e - omega_r_e

        ids, iqs, idr, iqr = self.currents_from_flux(psi_ds, psi_qs, psi_dr, psi_qr)

        dpsi_ds = v_ds - p.Rs * ids + omega_e * psi_qs
        dpsi_qs = v_qs - p.Rs * iqs - omega_e * psi_ds

        dpsi_dr = -p.Rr * idr + omega_sl * psi_qr
        dpsi_qr = -p.Rr * iqr - omega_sl * psi_dr

        Te = self.torque_e(psi_ds, psi_qs, ids, iqs)
        domega_m = (Te - T_L - p.B * omega_m) / p.J

        return np.array([dpsi_ds, dpsi_qs, dpsi_dr, dpsi_qr, domega_m], dtype=float)

    def outputs(self, x: np.ndarray) -> dict:
        psi_ds, psi_qs, psi_dr, psi_qr, omega_m = x
        ids, iqs, idr, iqr = self.currents_from_flux(psi_ds, psi_qs, psi_dr, psi_qr)
        Te = self.torque_e(psi_ds, psi_qs, ids, iqs)
        return dict(
            psi_ds=psi_ds, psi_qs=psi_qs, psi_dr=psi_dr, psi_qr=psi_qr,
            omega_m=omega_m, omega_r_e=self.p.p*omega_m,
            ids=ids, iqs=iqs, idr=idr, iqr=iqr, Te=Te
        )
