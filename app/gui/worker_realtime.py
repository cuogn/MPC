from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple, Optional
import time
import numpy as np
import pandas as pd
from PySide6.QtCore import QThread, Signal

from app.sim.motor_im_dq import InductionMotorDQ
from app.sim.integrators import rk4_step
from app.sim.foc_current_loop import CurrentLoopPI, PIParams
from app.control.pid_speed import PIDSpeed, PIDParams
from app.control.mpc_speed import SpeedMPC, MPCParams
from app.control.hybrid_mpc_speed import HybridSpeedMPC  # THÊM IMPORT NÀY
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
    hybrid_lr: float = 0.05  # Thêm thông số Learning Rate
    t_end: float = 3.0
    omega_profile: Optional[List[Tuple[float, float]]] = None
    tl_profile: Optional[List[Tuple[float, float]]] = None

class RealtimeWorker(QThread):
    sig_status = Signal(str)
    sig_data = Signal(dict)
    sig_done = Signal(dict)

    def __init__(self, cfg: SimConfig, mode: str = "COMPARE", plot_hz: float = 60.0, window_s: float = 3.0):
        super().__init__()
        self.cfg = cfg
        self.mode = mode.upper()
        self.plot_hz = float(plot_hz)
        self.window_s = float(window_s)
        self.is_running = False

    def stop(self):
        self.is_running = False

    def _make_drive(self):
        p = default_motor_params()
        motor = InductionMotorDQ(p)
        motor.x[:] = 0.0
        Lsigma = p.Ls - (p.Lm**2)/p.Lr
        current_ctl = CurrentLoopPI(
            Rs=p.Rs, Lsigma=Lsigma,
            pi_d=PIParams(kp=20.0, ki=2000.0, kaw=0.002),
            pi_q=PIParams(kp=20.0, ki=2000.0, kaw=0.002),
            vmax=300.0
        )
        return p, motor, current_ctl

    def run(self):
        self.is_running = True
        self.sig_status.emit(f"running {self.mode} (realtime)")

        # Khởi tạo 3 hệ thống truyền động độc lập
        p1, motor_pid, cur_pid = self._make_drive()
        p2, motor_mpc, cur_mpc = self._make_drive()
        p3, motor_hyb, cur_hyb = self._make_drive()

        iq_lim = float(self.cfg.iq_limit)
        
        # Cài đặt bộ điều khiển
        pid_ctl = PIDSpeed(PIDParams(kp=self.cfg.pid_kp, ki=self.cfg.pid_ki, kd=0.0, kaw=0.2), iq_min=-iq_lim, iq_max=iq_lim)
        
        mpc_params = MPCParams(Ts=1e-3, Np=int(self.cfg.mpc_Np), Q=float(self.cfg.mpc_Q), R=float(self.cfg.mpc_R), Rd=float(self.cfg.mpc_Rd), iq_min=-iq_lim, iq_max=iq_lim)
        mpc_ctl = SpeedMPC(mpc_params)
        
        hyb_ctl = HybridSpeedMPC(mpc_params, learning_rate=self.cfg.hybrid_lr)

        pid_ctl.reset()
        mpc_ctl.reset(0.0)
        hyb_ctl.reset(0.0)

        dt_e = 1e-4
        Ts = 1e-3
        n_inner = max(1, int(round(Ts / dt_e)))
        Ts = n_inner * dt_e
        
        mpc_ctl.p.Ts = Ts
        hyb_ctl.mpc.p.Ts = Ts

        eps_psi = 0.05
        k_slip_1 = (p1.Rr / p1.Lr) * (p1.Lm)
        k_slip_2 = (p2.Rr / p2.Lr) * (p2.Lm)
        k_slip_3 = (p3.Rr / p3.Lr) * (p3.Lm)

        def piecewise(profile: Optional[List[Tuple[float, float]]], default: float):
            if not profile: return lambda _: default
            prof = sorted(profile, key=lambda x: x[0])
            def f(t: float) -> float:
                val = default
                for tt, vv in prof:
                    if t >= tt: val = vv
                    else: break
                return val
            return f

        omega_ref_rpm = piecewise(self.cfg.omega_profile, 0.0)
        load_torque = piecewise(self.cfg.tl_profile, 0.0)
        
        if not self.cfg.omega_profile: omega_ref_rpm = lambda t: 0.0 if t < self.cfg.t_omega_step else self.cfg.omega_step_rpm
        if not self.cfg.tl_profile: load_torque = lambda t: 0.0 if t < self.cfg.t_tl_step else self.cfg.tl_step

        def id_ref(_: float) -> float: return 5.0

        max_profile_t = 0.0
        for prof in (self.cfg.omega_profile or []): max_profile_t = max(max_profile_t, float(prof[0]))
        for prof in (self.cfg.tl_profile or []): max_profile_t = max(max_profile_t, float(prof[0]))
        t_end = max(float(self.cfg.t_end), max_profile_t + 0.5)
        steps = int(t_end / Ts)

        plot_period = 1.0 / max(1e-6, self.plot_hz)
        next_plot_t = 0.0
        max_points = int(self.window_s / Ts) + 5

        log_pid, log_mpc, log_hyb = [], [], []
        iq_ref_pid, iq_ref_mpc, iq_ref_hyb = 0.0, 0.0, 0.0
        t0 = 0.0

        for _ in range(steps + 1):
            if not self.is_running: break

            omega_ref = omega_ref_rpm(t0) * (2*np.pi) / 60.0
            TL_now = load_torque(t0)

            if self.mode in ("PID", "COMPARE"):
                y = motor_pid.outputs(motor_pid.x)
                e = omega_ref - y["omega_m"]
                iq_ref_pid, _, _ = pid_ctl.step(e, Ts)

            if self.mode in ("MPC", "COMPARE"):
                y = motor_mpc.outputs(motor_mpc.x)
                psi_r_mag = float(np.hypot(y["psi_dr"], y["psi_qr"]))
                Kt = 1.5 * p2.p * (p2.Lm / p2.Lr) * psi_r_mag
                A = 1.0 - (p2.B / p2.J) * Ts
                B = (Kt / p2.J) * Ts
                G = -(1.0 / p2.J) * Ts
                wref_seq = np.full(mpc_ctl.p.Np, omega_ref, dtype=float)
                d_seq = np.full(mpc_ctl.p.Np, TL_now, dtype=float)
                sol = mpc_ctl.step(w0=y["omega_m"], wref_seq=wref_seq, A=A, B=B, G=G, d_seq=d_seq)
                iq_ref_mpc = sol["iq_ref"]

            if self.mode in ("HYBRID", "COMPARE"):
                y = motor_hyb.outputs(motor_hyb.x)
                psi_r_mag = float(np.hypot(y["psi_dr"], y["psi_qr"]))
                Kt = 1.5 * p3.p * (p3.Lm / p3.Lr) * psi_r_mag
                A = 1.0 - (p3.B / p3.J) * Ts
                B = (Kt / p3.J) * Ts
                G = -(1.0 / p3.J) * Ts
                wref_seq = np.full(hyb_ctl.mpc.p.Np, omega_ref, dtype=float)
                d_seq = np.full(hyb_ctl.mpc.p.Np, TL_now, dtype=float)
                sol = hyb_ctl.step(w0=y["omega_m"], wref_seq=wref_seq, A=A, B=B, G=G, d_seq=d_seq)
                iq_ref_hyb = sol["iq_ref"]

            # Inner Integration loop
            for j in range(n_inner):
                t = t0 + j * dt_e
                TL = load_torque(t)

                if self.mode in ("PID", "COMPARE"):
                    y = motor_pid.outputs(motor_pid.x)
                    ids, iqs = y["ids"], y["iqs"]
                    psi_r_mag = float(np.hypot(y["psi_dr"], y["psi_qr"]))
                    omega_e = p1.p * y["omega_m"] + ((k_slip_1 * iqs / psi_r_mag) if psi_r_mag > eps_psi else 0.0)
                    u = cur_pid.step(id_ref=id_ref(t), iq_ref=iq_ref_pid, id_meas=ids, iq_meas=iqs, omega_e=omega_e, dt=dt_e)
                    def fx1(xx): return motor_pid.f(xx, u["vd"], u["vq"], omega_e, TL)
                    motor_pid.x = rk4_step(fx1, motor_pid.x, dt_e)

                if self.mode in ("MPC", "COMPARE"):
                    y = motor_mpc.outputs(motor_mpc.x)
                    ids, iqs = y["ids"], y["iqs"]
                    psi_r_mag = float(np.hypot(y["psi_dr"], y["psi_qr"]))
                    omega_e = p2.p * y["omega_m"] + ((k_slip_2 * iqs / psi_r_mag) if psi_r_mag > eps_psi else 0.0)
                    u = cur_mpc.step(id_ref=id_ref(t), iq_ref=iq_ref_mpc, id_meas=ids, iq_meas=iqs, omega_e=omega_e, dt=dt_e)
                    def fx2(xx): return motor_mpc.f(xx, u["vd"], u["vq"], omega_e, TL)
                    motor_mpc.x = rk4_step(fx2, motor_mpc.x, dt_e)
                    
                if self.mode in ("HYBRID", "COMPARE"):
                    y = motor_hyb.outputs(motor_hyb.x)
                    ids, iqs = y["ids"], y["iqs"]
                    psi_r_mag = float(np.hypot(y["psi_dr"], y["psi_qr"]))
                    omega_e = p3.p * y["omega_m"] + ((k_slip_3 * iqs / psi_r_mag) if psi_r_mag > eps_psi else 0.0)
                    u = cur_hyb.step(id_ref=id_ref(t), iq_ref=iq_ref_hyb, id_meas=ids, iq_meas=iqs, omega_e=omega_e, dt=dt_e)
                    def fx3(xx): return motor_hyb.f(xx, u["vd"], u["vq"], omega_e, TL)
                    motor_hyb.x = rk4_step(fx3, motor_hyb.x, dt_e)

            # Data Logging
            if self.mode in ("PID", "COMPARE"):
                y = motor_pid.outputs(motor_pid.x)
                log_pid.append({"t": t0, "omega_ref_rpm": omega_ref_rpm(t0), "omega_rpm": y["omega_m"] * 60.0 / (2*np.pi), "iq_ref": iq_ref_pid, "iq": y["iqs"], "Te": y["Te"], "T_L": TL_now})
            if self.mode in ("MPC", "COMPARE"):
                y = motor_mpc.outputs(motor_mpc.x)
                log_mpc.append({"t": t0, "omega_ref_rpm": omega_ref_rpm(t0), "omega_rpm": y["omega_m"] * 60.0 / (2*np.pi), "iq_ref": iq_ref_mpc, "iq": y["iqs"], "Te": y["Te"], "T_L": TL_now})
            if self.mode in ("HYBRID", "COMPARE"):
                y = motor_hyb.outputs(motor_hyb.x)
                log_hyb.append({"t": t0, "omega_ref_rpm": omega_ref_rpm(t0), "omega_rpm": y["omega_m"] * 60.0 / (2*np.pi), "iq_ref": iq_ref_hyb, "iq": y["iqs"], "Te": y["Te"], "T_L": TL_now})

            if t0 >= next_plot_t:
                next_plot_t += plot_period

                def recent(log):
                    if not log: return (np.array([]),)*6
                    r = log[-max_points:]
                    return (np.array([x["t"] for x in r], dtype=float), np.array([x["omega_rpm"] for x in r], dtype=float), np.array([x["omega_ref_rpm"] for x in r], dtype=float), 
                            np.array([x["iq_ref"] for x in r], dtype=float), np.array([x["Te"] for x in r], dtype=float), np.array([x["T_L"] for x in r], dtype=float))

                t_pid, w_pid, wref_pid, iqref_pid, te_pid, tl_pid = recent(log_pid)
                t_mpc, w_mpc, wref_mpc, iqref_mpc, te_mpc, tl_mpc = recent(log_mpc)
                t_hyb, w_hyb, wref_hyb, iqref_hyb, te_hyb, tl_hyb = recent(log_hyb)

                t_ref = t_pid if wref_pid.size else (t_mpc if wref_mpc.size else t_hyb)
                wref_plot = wref_pid if wref_pid.size else (wref_mpc if wref_mpc.size else wref_hyb)

                def last(a, default=0.0): return float(a[-1]) if getattr(a, "size", 0) else float(default)

                self.sig_data.emit({
                    "t_ref": t_ref, "omega_ref": wref_plot,
                    "t_pid": t_pid, "omega_pid": w_pid, "iqref_pid": iqref_pid,
                    "t_mpc": t_mpc, "omega_mpc": w_mpc, "iqref_mpc": iqref_mpc,
                    "t_hybrid": t_hyb, "omega_hybrid": w_hyb, "iqref_hybrid": iqref_hyb,
                    "te_pid_now": last(te_pid), "te_mpc_now": last(te_mpc), "te_hyb_now": last(te_hyb),
                    "tl_now": last(tl_hyb) if tl_hyb.size else last(tl_mpc),
                    "iq_limit": iq_lim, "mode": self.mode,
                })
                time.sleep(0.001)

            t0 += Ts

        df_pid = pd.DataFrame(log_pid) if log_pid else None
        df_mpc = pd.DataFrame(log_mpc) if log_mpc else None
        df_hyb = pd.DataFrame(log_hyb) if log_hyb else None

        def compute_metrics(df: pd.DataFrame | None):
            if df is None or len(df) == 0: return None
            mask = df["t"].values >= float(self.cfg.t_omega_step)
            t = df.loc[mask, "t"].values
            y = df.loc[mask, "omega_rpm"].values
            yref = df.loc[mask, "omega_ref_rpm"].values
            if t.size == 0: return None
            return {"rmse_rpm": rmse(y, yref), "ise": ise(y, yref), "itae": itae(t - t[0], y, yref), "overshoot_%": overshoot_percent(y, float(yref[-1])), "settling_s": settling_time(t, y, float(yref[-1]), tol_pct=2.0), "final_speed_rpm": float(df["omega_rpm"].values[-1])}

        rows = []
        if (met := compute_metrics(df_pid)): rows.append({"controller": "PID", **met})
        if (met := compute_metrics(df_mpc)): rows.append({"controller": "MPC", **met})
        if (met := compute_metrics(df_hyb)): rows.append({"controller": "HYBRID", **met})
        mdf = pd.DataFrame(rows) if rows else None

        self.sig_status.emit("done")
        self.sig_done.emit({"pid": df_pid, "mpc": df_mpc, "hybrid": df_hyb, "metrics": mdf})
        self.is_running = False