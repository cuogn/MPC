from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np
import pandas as pd
from PySide6.QtCore import QThread, Signal

from app.sim.motor_im_dq import InductionMotorDQ
from app.sim.integrators import rk4_step
from app.sim.foc_current_loop import CurrentLoopPI, PIParams
from app.control.pid_speed import PIDSpeed, PIDParams
from app.control.mpc_speed import SpeedMPC, MPCParams
from app.sim.simulator import default_motor_params
from app.metrics.metrics import rmse, ise, itae, overshoot_percent, settling_time


@dataclass
class SimConfig:
    omega_step_rpm: float
    t_omega_step: float
    tl_step: float
    t_tl_step: float
    iq_limit: float
    pid_kp: float
    pid_ki: float
    mpc_Np: int
    mpc_Q: float
    mpc_R: float
    mpc_Rd: float
    t_end: float = 3.0
    omega_profile: Optional[List[Tuple[float, float]]] = None
    tl_profile: Optional[List[Tuple[float, float]]] = None


class BatchWorker(QThread):
    """Batch simulation worker.

    Emits a single result when finished:
      {"pid": df|None, "mpc": df|None, "metrics": df_metrics}
    """

    sig_status = Signal(str)
    sig_result = Signal(dict)

    def __init__(self, cfg: SimConfig, mode: str = "BOTH"):
        super().__init__()
        self.cfg = cfg
        self.mode = mode.upper()  # PID, MPC, BOTH
        self.is_running = False

    def stop(self):
        self.is_running = False

    def run(self):
        self.is_running = True
        try:
            self.sig_status.emit("simulating...")
            df_pid = None
            df_mpc = None
            rows = []

            if self.mode in ("PID", "BOTH"):
                df_pid, met = self._simulate("PID")
                rows.append({"controller": "PID", **met})

            if self.mode in ("MPC", "BOTH"):
                df_mpc, met = self._simulate("MPC")
                rows.append({"controller": "MPC", **met})

            mdf = pd.DataFrame(rows) if rows else None
            self.sig_result.emit({"pid": df_pid, "mpc": df_mpc, "metrics": mdf})
        except Exception as e:
            self.sig_status.emit(f"error: {e}")
            self.sig_result.emit({"pid": None, "mpc": None, "metrics": None})
        finally:
            self.is_running = False

    def _simulate(self, controller: str):
        p = default_motor_params()
        motor = InductionMotorDQ(p)
        motor.x[:] = 0.0

        # inner loop current PI (FOC)
        Lsigma = p.Ls - (p.Lm**2) / p.Lr
        current_ctl = CurrentLoopPI(
            Rs=p.Rs,
            Lsigma=Lsigma,
            pi_d=PIParams(kp=20.0, ki=2000.0, kaw=0.002),
            pi_q=PIParams(kp=20.0, ki=2000.0, kaw=0.002),
            vmax=300.0,
        )

        iq_lim = float(self.cfg.iq_limit)
        pid = PIDSpeed(
            PIDParams(kp=self.cfg.pid_kp, ki=self.cfg.pid_ki, kd=0.0, kaw=0.2),
            iq_min=-iq_lim,
            iq_max=iq_lim,
        )
        mpc = SpeedMPC(
            MPCParams(
                Ts=1e-3,
                Np=int(self.cfg.mpc_Np),
                Q=float(self.cfg.mpc_Q),
                R=float(self.cfg.mpc_R),
                Rd=float(self.cfg.mpc_Rd),
                iq_min=-iq_lim,
                iq_max=iq_lim,
            )
        )
        pid.reset()
        mpc.reset(0.0)

        # multi-rate (electrical dt_e, outer Ts)
        dt_e = 1e-4
        Ts = 1e-3
        n_inner = max(1, int(round(Ts / dt_e)))
        Ts = n_inner * dt_e
        mpc.p.Ts = Ts

        eps_psi = 0.05
        k_slip = (p.Rr / p.Lr) * p.Lm

        def piecewise(profile: Optional[List[Tuple[float, float]]], default: float):
            if not profile:
                return lambda _: default
            prof = sorted(profile, key=lambda x: x[0])
            def f(t: float) -> float:
                val = default
                for tt, vv in prof:
                    if t >= tt:
                        val = vv
                    else:
                        break
                return val
            return f

        omega_ref_rpm = piecewise(self.cfg.omega_profile, 0.0)
        load_torque = piecewise(self.cfg.tl_profile, 0.0)
        if not self.cfg.omega_profile:
            omega_ref_rpm = lambda t: 0.0 if t < float(self.cfg.t_omega_step) else float(self.cfg.omega_step_rpm)
        if not self.cfg.tl_profile:
            load_torque = lambda t: 0.0 if t < float(self.cfg.t_tl_step) else float(self.cfg.tl_step)

        def id_ref(_: float) -> float:
            return 5.0

        max_profile_t = 0.0
        for prof in (self.cfg.omega_profile or []):
            max_profile_t = max(max_profile_t, float(prof[0]))
        for prof in (self.cfg.tl_profile or []):
            max_profile_t = max(max_profile_t, float(prof[0]))
        t_end = max(float(self.cfg.t_end), max_profile_t + 0.5)
        n_steps = int(t_end / Ts)
        t0 = 0.0
        iq_ref = 0.0
        omega_e = 0.0
        recs = []

        for _ in range(n_steps + 1):
            if not self.is_running:
                break

            y = motor.outputs(motor.x)
            omega_m = y["omega_m"]
            omega_ref = omega_ref_rpm(t0) * (2 * np.pi) / 60.0
            e = omega_ref - omega_m

            psi_r_mag = float(np.hypot(y["psi_dr"], y["psi_qr"]))
            Kt = 1.5 * p.p * (p.Lm / p.Lr) * psi_r_mag

            if controller.upper() == "PID":
                iq_ref, _, _ = pid.step(e, Ts)
            else:
                # Linearized 1st-order mech model around current Kt
                A = 1.0 - (p.B / p.J) * Ts
                B = (Kt / p.J) * Ts
                G = -(1.0 / p.J) * Ts
                wref_seq = np.full(mpc.p.Np, omega_ref, dtype=float)
                d_seq = np.full(mpc.p.Np, load_torque(t0), dtype=float)
                sol = mpc.step(w0=omega_m, wref_seq=wref_seq, A=A, B=B, G=G, d_seq=d_seq)
                iq_ref = sol["iq_ref"]

            # integrate electrical dynamics
            for j in range(n_inner):
                t = t0 + j * dt_e
                TL = load_torque(t)

                y_in = motor.outputs(motor.x)
                ids, iqs = y_in["ids"], y_in["iqs"]

                psi_r_mag_in = float(np.hypot(y_in["psi_dr"], y_in["psi_qr"]))
                omega_r_e = p.p * y_in["omega_m"]
                omega_sl = (k_slip * iqs / psi_r_mag_in) if psi_r_mag_in > eps_psi else 0.0
                omega_e = omega_r_e + omega_sl

                u = current_ctl.step(
                    id_ref=id_ref(t),
                    iq_ref=iq_ref,
                    id_meas=ids,
                    iq_meas=iqs,
                    omega_e=omega_e,
                    dt=dt_e,
                )
                vd, vq = u["vd"], u["vq"]

                def fx(xx):
                    return motor.f(xx, vd, vq, omega_e, TL)

                motor.x = rk4_step(fx, motor.x, dt_e)

            y2 = motor.outputs(motor.x)
            recs.append(
                {
                    "t": t0,
                    "omega_ref_rpm": omega_ref_rpm(t0),
                    "omega_rpm": y2["omega_m"] * 60.0 / (2 * np.pi),
                    "iq_ref": iq_ref,
                    "iq": y2["iqs"],
                    "Te": y2["Te"],
                    "T_L": load_torque(t0),
                }
            )

            t0 += Ts

        df = pd.DataFrame(recs)

        # metrics after speed step
        mask = df["t"].values >= float(self.cfg.t_omega_step)
        t = df.loc[mask, "t"].values
        y = df.loc[mask, "omega_rpm"].values
        yref = df.loc[mask, "omega_ref_rpm"].values

        met = {
            "rmse_rpm": rmse(y, yref),
            "ise": ise(y, yref),
            "itae": itae(t - t[0], y, yref) if t.size else float("nan"),
            "overshoot_%": overshoot_percent(y, float(yref[-1])) if yref.size else float("nan"),
            "settling_s": settling_time(t, y, float(yref[-1]), tol_pct=2.0) if yref.size else float("nan"),
            "final_speed_rpm": float(df["omega_rpm"].values[-1]) if len(df) else float("nan"),
        }
        return df, met
