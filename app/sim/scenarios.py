from __future__ import annotations
from dataclasses import dataclass

@dataclass(frozen=True)
class OpenLoopScenario:
    omega_e: float
    v_ds: float
    v_qs: float
    T_L0: float = 0.0
    T_L_step: float = 0.0
    t_step: float = 1.5

    def load_torque(self, t: float) -> float:
        return self.T_L0 + (self.T_L_step if t >= self.t_step else 0.0)
