from __future__ import annotations

import json
import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from app.sim.motor_im_dq import IMParams, InductionMotorDQ
from app.sim.integrators import rk4_step
from app.sim.foc_current_loop import CurrentLoopPI, PIParams
from app.control.pid_speed import PIDSpeed, PIDParams
from app.control.mpc_speed import SpeedMPC, MPCParams
from app.metrics.metrics import rmse, ise, itae, overshoot_percent, settling_time


def _apply_thesis_plot_style() -> None:
    """White background + black text for thesis figures."""
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
            "text.color": "black",
            "axes.labelcolor": "black",
            "axes.edgecolor": "black",
            "xtick.color": "black",
            "ytick.color": "black",
            "font.size": 11,
            "axes.titlesize": 12,
            "axes.labelsize": 11,
            "legend.fontsize": 10,
            "lines.linewidth": 2.0,
            "grid.color": "#cccccc",
            "grid.linewidth": 0.6,
            "grid.alpha": 0.6,
        }
    )


def _load_step(t: float, cfg: Dict[str, Any]) -> float:
    t0 = float(cfg.get("t0", 1.5))
    val = float(cfg.get("value", 5.0))
    return 0.0 if t < t0 else val


def _omega_ref_step_rpm(t: float, omega_step_rpm: float, t_step: float) -> float:
    return 0.0 if t < t_step else float(omega_step_rpm)


def apply_motor_variation(p: IMParams, mult: Dict[str, float]) -> IMParams:
    """Return a new IMParams with param multipliers applied.

    Supported keys:
      - Rs, Rr, Lm: direct scaling
      - Lsigma: scales stator/rotor *leakage* inductances (Lls, Llr) while keeping Lm constant
    """
    Rs, Rr, Ls, Lr, Lm = p.Rs, p.Rr, p.Ls, p.Lr, p.Lm

    if "Rs" in mult:
        Rs *= float(mult["Rs"])
    if "Rr" in mult:
        Rr *= float(mult["Rr"])
    if "Lm" in mult:
        Lm *= float(mult["Lm"])

    if "Lsigma" in mult:
        f = float(mult["Lsigma"])
        # infer leakages from nominal
        Lls = max(0.0, Ls - p.Lm)
        Llr = max(0.0, Lr - p.Lm)
        Ls = Lm + Lls * f
        Lr = Lm + Llr * f

    # sanity: determinant must be positive (Ls*Lr - Lm^2 > 0)
    det = Ls * Lr - Lm * Lm
    if det <= 1e-10:
        # gently increase leakage until valid
        Ls = max(Ls, 1.2 * Lm)
        Lr = max(Lr, 1.2 * Lm)

    return IMParams(Rs=Rs, Rr=Rr, Ls=Ls, Lr=Lr, Lm=Lm, p=p.p, J=p.J, B=p.B)


@dataclass
class LoopCfg:
    dt_e: float
    Ts_speed: float
    t_end: float


@dataclass
class LimitsCfg:
    iq_limit: float
    Vmax: float


def simulate_one(
    controller: str,
    p: IMParams,
    loop: LoopCfg,
    limits: LimitsCfg,
    current_pi: Tuple[PIParams, PIParams],
    pid: PIDParams,
    mpc: MPCParams,
    omega_step_rpm: float,
    t_omega_step: float,
    load_cfg: Dict[str, Any],
    id_ref_const: float = 5.0,
) -> Tuple[pd.DataFrame, Dict[str, float]]:
    """Run one PID/MPC simulation with IM dq plant + inner current PI."""
    dt_e = float(loop.dt_e)
    Ts_speed = float(loop.Ts_speed)
    n_inner = max(1, int(round(Ts_speed / dt_e)))
    Ts = n_inner * dt_e

    # inner current loop
    Lsigma = p.Ls - (p.Lm ** 2) / p.Lr
    pi_d, pi_q = current_pi
    current_ctl = CurrentLoopPI(Rs=p.Rs, Lsigma=Lsigma, pi_d=pi_d, pi_q=pi_q, vmax=float(limits.Vmax))

    # outer loop
    iq_lim = float(limits.iq_limit)
    if controller.lower() == "pid":
        speed_ctl = PIDSpeed(pid, iq_min=-iq_lim, iq_max=iq_lim)
        speed_ctl.reset()
        mpc_ctl = None
    elif controller.lower() == "mpc":
        mpc_ctl = SpeedMPC(mpc)
        mpc_ctl.reset(0.0)
        speed_ctl = None
    else:
        raise ValueError("controller must be 'pid' or 'mpc'")

    motor = InductionMotorDQ(p)
    motor.x[:] = 0.0

    # slip computation (rotor-flux orientation)
    eps_psi = 0.05
    k_slip = (p.Rr / p.Lr) * (p.Lm)

    def load_torque(t: float) -> float:
        typ = str(load_cfg.get("type", "STEP")).upper()
        if typ == "STEP":
            return _load_step(t, load_cfg)
        # fallback
        return _load_step(t, load_cfg)

    rows = []
    iq_ref_prev = 0.0
    mpc_status = "na"
    omega_e = 0.0

    n_steps = int(loop.t_end / Ts)
    for ks in range(n_steps + 1):
        t0 = ks * Ts

        y = motor.outputs(motor.x)
        omega_m = float(y["omega_m"])
        omega_ref = _omega_ref_step_rpm(t0, omega_step_rpm, t_omega_step) * (2 * np.pi) / 60.0
        e = omega_ref - omega_m
        TL0 = float(load_torque(t0))

        # for MPC: estimate Kt from rotor flux magnitude
        psi_r_mag = float(np.hypot(y["psi_dr"], y["psi_qr"]))
        Kt = 1.5 * p.p * (p.Lm / p.Lr) * psi_r_mag

        if speed_ctl is not None:
            iq_ref, iq_unsat, aw = speed_ctl.step(e, Ts)
        else:
            # Discrete model for speed: ω[k+1] = A ω[k] + B iq[k] + G TL
            A = 1.0 - (p.B / p.J) * Ts
            B = (Kt / p.J) * Ts
            G = -(1.0 / p.J) * Ts

            wref_seq = np.full(mpc_ctl.p.Np, omega_ref, dtype=float)
            d_seq = np.full(mpc_ctl.p.Np, TL0, dtype=float)
            sol = mpc_ctl.step(w0=omega_m, wref_seq=wref_seq, A=A, B=B, G=G, d_seq=d_seq)
            iq_ref = float(sol["iq_ref"])
            mpc_status = str(sol.get("status", "ok"))
            iq_unsat = float(sol["u_seq"][0]) if sol.get("u_seq") is not None else iq_ref
            aw = 0.0

        # inner-loop integration
        last_u = None
        for j in range(n_inner):
            t = t0 + j * dt_e
            TL = float(load_torque(t))

            y_in = motor.outputs(motor.x)
            ids, iqs = float(y_in["ids"]), float(y_in["iqs"])

            psi_r_mag_in = float(np.hypot(y_in["psi_dr"], y_in["psi_qr"]))
            omega_r_e = p.p * float(y_in["omega_m"])
            omega_sl = (k_slip * iqs / psi_r_mag_in) if psi_r_mag_in > eps_psi else 0.0
            omega_e = omega_r_e + omega_sl

            u = current_ctl.step(
                id_ref=id_ref_const,
                iq_ref=iq_ref,
                id_meas=ids,
                iq_meas=iqs,
                omega_e=omega_e,
                dt=dt_e,
            )
            vd, vq = float(u["vd"]), float(u["vq"])

            def fx(xx):
                return motor.f(xx, vd, vq, omega_e, TL)

            motor.x = rk4_step(fx, motor.x, dt_e)
            last_u = u

        y2 = motor.outputs(motor.x)
        omega_rpm = float(y2["omega_m"]) * 60.0 / (2 * np.pi)

        du = float(iq_ref - iq_ref_prev)
        iq_ref_prev = float(iq_ref)

        rows.append(
            {
                "t": t0,
                "omega_ref_rpm": _omega_ref_step_rpm(t0, omega_step_rpm, t_omega_step),
                "omega_rpm": omega_rpm,
                "iq_ref": float(iq_ref),
                "iq_unsat": float(iq_unsat),
                "aw": float(aw),
                "Te": float(y2["Te"]),
                "T_L": float(TL0),
                "ids": float(y2["ids"]),
                "iqs": float(y2["iqs"]),
                "vd": float(last_u["vd"] if last_u else 0.0),
                "vq": float(last_u["vq"] if last_u else 0.0),
                "du": du,
                "sat_iq": 1.0 if abs(float(iq_ref)) >= (iq_lim - 1e-9) else 0.0,
                "mpc_status": mpc_status,
            }
        )

    df = pd.DataFrame(rows)

    mask = df["t"].values >= float(t_omega_step)
    t = df.loc[mask, "t"].values
    y = df.loc[mask, "omega_rpm"].values
    yref = df.loc[mask, "omega_ref_rpm"].values
    yfinal = float(yref[-1] if len(yref) else 0.0)

    met = {
        "rmse_rpm": rmse(y, yref),
        "ise": ise(y, yref),
        "itae": itae(t - t[0], y, yref),
        "overshoot_%": overshoot_percent(y, yfinal),
        "settling_s": settling_time(t, y, yfinal, tol_pct=2.0),
        "du2": float(np.sum(np.square(df.loc[mask, "du"].values))),
        "sat_pct": float(100.0 * np.mean(df.loc[mask, "sat_iq"].values)),
        "final_speed_rpm": float(df["omega_rpm"].iloc[-1]),
    }
    return df, met


def run_batch(catalog_path: str, out_dir: str | None = None) -> str:
    """Run PID vs MPC for all scenarios in a catalog and write tables/logs/plots.

    Returns the output directory path.
    """
    _apply_thesis_plot_style()

    catalog = json.loads(Path(catalog_path).read_text(encoding="utf-8"))
    nom = catalog["nominal"]
    scenarios = catalog["scenarios"]

    mp = nom["motor"]
    p_nom = IMParams(
        Rs=float(mp["Rs"]),
        Rr=float(mp["Rr"]),
        Ls=float(mp["Ls"]),
        Lr=float(mp["Lr"]),
        Lm=float(mp["Lm"]),
        p=int(mp["p"]),
        J=float(mp["J"]),
        B=float(mp["B"]),
    )

    loop = LoopCfg(
        dt_e=float(nom["loop"]["dt_plant"]),
        Ts_speed=float(nom["loop"]["Ts_speed"]),
        t_end=float(nom["loop"]["t_end"]),
    )
    limits = LimitsCfg(iq_limit=float(nom["limits"]["iq_limit"]), Vmax=float(nom["limits"]["Vmax"]))

    pi_d = PIParams(kp=float(nom["current_pi"]["kp"]), ki=float(nom["current_pi"]["ki"]), kaw=float(nom["current_pi"].get("kaw", 0.002)))
    pi_q = PIParams(kp=float(nom["current_pi"]["kp"]), ki=float(nom["current_pi"]["ki"]), kaw=float(nom["current_pi"].get("kaw", 0.002)))

    pid = PIDParams(kp=float(nom["pid"]["kp"]), ki=float(nom["pid"]["ki"]), kd=float(nom["pid"].get("kd", 0.0)), kaw=float(nom["pid"].get("kaw", 0.2)))

    # MPC params: Ts must be aligned with inner discretization
    Ts = max(loop.Ts_speed, loop.dt_e)
    mpc = MPCParams(
        Ts=float(Ts),
        Np=int(nom["mpc"]["Np"]),
        Q=float(nom["mpc"]["Q"]),
        R=float(nom["mpc"]["R"]),
        Rd=float(nom["mpc"]["Rd"]),
        iq_min=-float(limits.iq_limit),
        iq_max=float(limits.iq_limit),
    )

    omega_step_rpm = float(nom["scenario"]["omega_step_rpm"])
    t_omega_step = float(nom["scenario"]["t_omega_step"])

    if out_dir is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = str(Path("outputs") / f"batch_{stamp}")
    out = Path(out_dir)
    (out / "csv").mkdir(parents=True, exist_ok=True)
    (out / "plots").mkdir(parents=True, exist_ok=True)

    results = []

    for sc in scenarios:
        sid = sc["id"]
        p_sc = apply_motor_variation(p_nom, sc.get("mult", {}))
        load_cfg = sc.get("load", {"type": "STEP", "t0": 1.5, "value": 5.0})

        df_pid, met_pid = simulate_one(
            "pid",
            p_sc,
            loop,
            limits,
            (pi_d, pi_q),
            pid,
            mpc,
            omega_step_rpm=omega_step_rpm,
            t_omega_step=t_omega_step,
            load_cfg=load_cfg,
            id_ref_const=float(nom["current_pi"].get("id_ref", 5.0)),
        )
        df_mpc, met_mpc = simulate_one(
            "mpc",
            p_sc,
            loop,
            limits,
            (pi_d, pi_q),
            pid,
            mpc,
            omega_step_rpm=omega_step_rpm,
            t_omega_step=t_omega_step,
            load_cfg=load_cfg,
            id_ref_const=float(nom["current_pi"].get("id_ref", 5.0)),
        )

        df_pid.to_csv(out / "csv" / f"{sid}_PID.csv", index=False)
        df_mpc.to_csv(out / "csv" / f"{sid}_MPC.csv", index=False)

        results.append({"scenario": sid, "controller": "PID", **met_pid})
        results.append({"scenario": sid, "controller": "MPC", **met_mpc})

        # Plot per scenario (speed + iq* + torques)
        fig, axs = plt.subplots(3, 1, figsize=(9.2, 8.0), sharex=True)
        axs[0].plot(df_pid["t"], df_pid["omega_ref_rpm"], linestyle="--", label="ω* (rpm)")
        axs[0].plot(df_pid["t"], df_pid["omega_rpm"], linestyle="-.", label="PID ω")
        axs[0].plot(df_mpc["t"], df_mpc["omega_rpm"], label="MPC ω")
        axs[0].set_ylabel("rpm")
        axs[0].grid(True)
        axs[0].legend()
        axs[0].set_title(f"Kịch bản {sid}: ω và ω*")

        lim = float(limits.iq_limit)
        axs[1].plot(df_pid["t"], df_pid["iq_ref"], linestyle="-.", label="PID i_q*")
        axs[1].plot(df_mpc["t"], df_mpc["iq_ref"], label="MPC i_q*")
        axs[1].plot(df_pid["t"], np.full_like(df_pid["t"], lim), linestyle=":", label="+limit")
        axs[1].plot(df_pid["t"], np.full_like(df_pid["t"], -lim), linestyle=":", label="-limit")
        axs[1].set_ylabel("A")
        axs[1].grid(True)
        axs[1].legend(ncols=2)
        axs[1].set_title("Lệnh dòng i_q* và giới hạn")

        axs[2].plot(df_pid["t"], df_pid["T_L"], linestyle="--", label="T_L")
        axs[2].plot(df_pid["t"], df_pid["Te"], linestyle="-.", label="PID T_e")
        axs[2].plot(df_mpc["t"], df_mpc["Te"], label="MPC T_e")
        axs[2].set_xlabel("t (s)")
        axs[2].set_ylabel("N·m")
        axs[2].grid(True)
        axs[2].legend()
        axs[2].set_title("Mô-men tải và mô-men điện từ")

        fig.tight_layout()
        fig.savefig(out / "plots" / f"{sid}_compare.png", dpi=300, bbox_inches="tight")
        fig.savefig(out / "plots" / f"{sid}_compare.svg", bbox_inches="tight")
        plt.close(fig)

    rdf = pd.DataFrame(results)
    rdf.to_csv(out / "metrics_all.csv", index=False)

    pivot = rdf.pivot_table(
        index="scenario",
        columns="controller",
        values=["rmse_rpm", "overshoot_%", "settling_s", "itae", "du2", "sat_pct"],
        aggfunc="first",
    )
    pivot.to_csv(out / "metrics_pivot.csv")

    try:
        with pd.ExcelWriter(out / "metrics_tables.xlsx", engine="openpyxl") as w:
            rdf.to_excel(w, sheet_name="AllRows", index=False)
            pivot.to_excel(w, sheet_name="Pivot", merge_cells=False)
    except Exception:
        # If openpyxl is not available in some environments, CSVs are still produced.
        pass

    # Store a copy of the catalog for traceability
    (out / "scenarios_catalog.json").write_text(json.dumps(catalog, indent=2), encoding="utf-8")
    return str(out)
