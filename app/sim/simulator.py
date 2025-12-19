from __future__ import annotations
import os
import numpy as np
import matplotlib.pyplot as plt

from app.sim.motor_im_dq import IMParams, InductionMotorDQ
from app.sim.integrators import rk4_step
from app.sim.logger import SimLogger
from app.sim.scenarios import OpenLoopScenario
from app.sim.foc_current_loop import CurrentLoopPI, PIParams
from app.control.pid_speed import PIDSpeed, PIDParams
from app.control.mpc_speed import SpeedMPC, MPCParams
from app.metrics.metrics import rmse, ise, itae, overshoot_percent


def default_motor_params() -> IMParams:
    Rs = 0.435
    Rr = 0.816
    Lls = 0.002
    Llr = 0.002
    Lm = 0.0693
    Ls = Lls + Lm
    Lr = Llr + Lm
    p = 2
    J = 0.02
    B = 0.001
    return IMParams(Rs=Rs, Rr=Rr, Ls=Ls, Lr=Lr, Lm=Lm, p=p, J=J, B=B)


def run_open_loop_demo():
    os.makedirs("outputs", exist_ok=True)
    f = 50.0
    omega_e = 2*np.pi*f

    V_ll_rms = 400.0
    V_phase_rms = V_ll_rms / np.sqrt(3.0)
    V_phase_peak = np.sqrt(2.0) * V_phase_rms

    scenario = OpenLoopScenario(
        omega_e=omega_e, v_ds=V_phase_peak, v_qs=0.0,
        T_L0=0.0, T_L_step=5.0, t_step=1.5
    )

    motor = InductionMotorDQ(default_motor_params())
    dt = 1e-4
    t_end = 3.0
    n = int(t_end/dt)

    log = SimLogger()
    decim = 10

    for k in range(n+1):
        t = k*dt
        TL = scenario.load_torque(t)

        def fx(xx):
            return motor.f(xx, scenario.v_ds, scenario.v_qs, scenario.omega_e, TL)

        motor.x = rk4_step(fx, motor.x, dt)

        if k % decim == 0:
            y = motor.outputs(motor.x)
            omega_rpm = y["omega_m"] * 60.0 / (2*np.pi)
            omega_sync_rpm = (scenario.omega_e / motor.p.p) * 60.0 / (2*np.pi)
            log.append(
                t=t, T_L=TL, Te=y["Te"], v_ds=scenario.v_ds, v_qs=scenario.v_qs,
                omega_rpm=omega_rpm, omega_sync_rpm=omega_sync_rpm,
                ids=y["ids"], iqs=y["iqs"]
            )

    df = log.to_dataframe()
    df.to_csv("outputs/open_loop.csv", index=False)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(df["t"], df["omega_rpm"], label="omega (rpm)")
    ax.plot(df["t"], df["omega_sync_rpm"], label="sync (rpm)")
    ax.grid(True); ax.legend()
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Speed (rpm)")
    fig.tight_layout()
    fig.savefig("outputs/open_loop.png", dpi=160)
    plt.close(fig)

    last = df.iloc[-1]
    print("=== Open-loop demo finished ===")
    print(f"Final speed: {last['omega_rpm']:.1f} rpm (sync ~ {last['omega_sync_rpm']:.1f} rpm)")
    print(f"Final Te: {last['Te']:.2f} N*m, Final TL: {last['T_L']:.2f} N*m")
    print("Saved: outputs/open_loop.csv and outputs/open_loop.png")


def run_current_loop_demo():
    os.makedirs("outputs", exist_ok=True)
    p = default_motor_params()
    Lsigma = p.Ls - (p.Lm**2)/p.Lr

    Vmax = 300.0
    pi_d = PIParams(kp=20.0, ki=2000.0, kaw=0.002)
    pi_q = PIParams(kp=20.0, ki=2000.0, kaw=0.002)
    ctl = CurrentLoopPI(Rs=p.Rs, Lsigma=Lsigma, pi_d=pi_d, pi_q=pi_q, vmax=Vmax)

    omega_e = 0.0

    motor = InductionMotorDQ(p)
    motor.x[:] = 0.0

    dt = 1e-4
    t_end = 0.4
    n = int(t_end/dt)

    def id_ref(t): return 0.0 if t < 0.10 else 5.0
    def iq_ref(t): return 0.0 if t < 0.20 else 12.0
    def TL(t): return 0.0

    log = SimLogger()
    decim = 5

    for k in range(n+1):
        t = k*dt
        y = motor.outputs(motor.x)
        ids, iqs = y["ids"], y["iqs"]
        idr, iqr = id_ref(t), iq_ref(t)

        u = ctl.step(idr, iqr, ids, iqs, omega_e, dt)
        vd, vq = u["vd"], u["vq"]

        def fx(xx):
            return motor.f(xx, vd, vq, omega_e, TL(t))

        motor.x = rk4_step(fx, motor.x, dt)

        if k % decim == 0:
            y2 = motor.outputs(motor.x)
            omega_rpm = y2["omega_m"] * 60.0 / (2*np.pi)
            log.append(
                t=t,
                id_ref=idr, iq_ref=iqr,
                ids=y2["ids"], iqs=y2["iqs"],
                vd=vd, vq=vq,
                vd_unsat=u["vd_unsat"], vq_unsat=u["vq_unsat"],
                omega_rpm=omega_rpm,
            )

    df = log.to_dataframe()
    df.to_csv("outputs/current_loop.csv", index=False)

    fig = plt.figure(figsize=(8,6))
    ax1 = fig.add_subplot(211)
    ax1.plot(df["t"], df["ids"], label="i_d")
    ax1.plot(df["t"], df["iqs"], label="i_q")
    ax1.plot(df["t"], df["id_ref"], "--", label="i_d*")
    ax1.plot(df["t"], df["iq_ref"], "--", label="i_q*")
    ax1.set_ylabel("Current (A)")
    ax1.grid(True); ax1.legend()

    ax2 = fig.add_subplot(212)
    ax2.plot(df["t"], df["vd"], label="v_d (sat)")
    ax2.plot(df["t"], df["vq"], label="v_q (sat)")
    ax2.plot(df["t"], df["vd_unsat"], "--", label="v_d (unsat)")
    ax2.plot(df["t"], df["vq_unsat"], "--", label="v_q (unsat)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Voltage (V)")
    ax2.grid(True); ax2.legend()

    fig.tight_layout()
    fig.savefig("outputs/current_loop.png", dpi=160)
    plt.close(fig)

    last = df.iloc[-1]
    print("=== Current-loop PI demo finished ===")
    print(f"Final i_d: {last['ids']:.2f} A (ref {last['id_ref']:.2f})")
    print(f"Final i_q: {last['iqs']:.2f} A (ref {last['iq_ref']:.2f})")
    print(f"Final speed: {last['omega_rpm']:.1f} rpm")
    print("Saved: outputs/current_loop.csv and outputs/current_loop.png")


def _speed_control_sim(controller: str):
    """Shared simulation harness for speed_pid / speed_mpc."""
    p = default_motor_params()
    Lsigma = p.Ls - (p.Lm**2)/p.Lr

    # Inner current loop
    Vmax = 300.0
    pi_d = PIParams(kp=20.0, ki=2000.0, kaw=0.002)
    pi_q = PIParams(kp=20.0, ki=2000.0, kaw=0.002)
    current_ctl = CurrentLoopPI(Rs=p.Rs, Lsigma=Lsigma, pi_d=pi_d, pi_q=pi_q, vmax=Vmax)

    # References & disturbance
    def omega_ref_rpm(t: float) -> float:
        return 0.0 if t < 0.30 else 1500.0

    def load_torque(t: float) -> float:
        return 0.0 if t < 1.50 else 5.0

    def id_ref(t: float) -> float:
        return 5.0

    motor = InductionMotorDQ(p)
    motor.x[:] = 0.0

    dt_e = 1e-4
    Ts = 1e-3
    n_inner = int(round(Ts / dt_e))
    n_inner = max(1, n_inner)
    Ts = n_inner * dt_e

    t_end = 3.0
    n_steps = int(t_end / Ts)

    # Speed controller
    iq_lim = 20.0
    if controller == "pid":
        speed_ctl = PIDSpeed(PIDParams(kp=0.8, ki=10.0, kd=0.0, kaw=0.2), iq_min=-iq_lim, iq_max=iq_lim)
        speed_ctl.reset()
    elif controller == "mpc":
        mpc = SpeedMPC(MPCParams(Ts=Ts, Np=25, Q=1.0, R=0.02, Rd=0.2, iq_min=-iq_lim, iq_max=iq_lim))
        mpc.reset(0.0)
    else:
        raise ValueError("controller must be 'pid' or 'mpc'")

    # Slip computation (rotor-flux orientation)
    eps_psi = 0.05
    k_slip = (p.Rr / p.Lr) * (p.Lm)

    log = SimLogger()

    iq_ref = 0.0
    iq_unsat = 0.0
    aw = 0.0
    omega_e = 0.0
    mpc_status = "na"

    for ks in range(n_steps + 1):
        t0 = ks * Ts

        y = motor.outputs(motor.x)
        omega_m = y["omega_m"]
        omega_ref = omega_ref_rpm(t0) * (2*np.pi) / 60.0
        e = omega_ref - omega_m

        # Estimate Kt from rotor flux magnitude (for MPC)
        psi_r_mag = float(np.hypot(y["psi_dr"], y["psi_qr"]))
        Kt = 1.5 * p.p * (p.Lm / p.Lr) * psi_r_mag  # Te ≈ Kt * iq

        if controller == "pid":
            iq_ref, iq_unsat, aw = speed_ctl.step(e, Ts)
        else:
            # Discrete model params (Euler)
            A = 1.0 - (p.B / p.J) * Ts
            B = (Kt / p.J) * Ts
            G = -(1.0 / p.J) * Ts

            # horizon references / disturbance (hold constant)
            wref_seq = np.full(mpc.p.Np, omega_ref, dtype=float)
            d_seq = np.full(mpc.p.Np, load_torque(t0), dtype=float)
            sol = mpc.step(w0=omega_m, wref_seq=wref_seq, A=A, B=B, G=G, d_seq=d_seq)
            iq_ref = sol["iq_ref"]
            mpc_status = sol["status"]
            iq_unsat = float(sol["u_seq"][0]) if sol["u_seq"] is not None else iq_ref
            aw = 0.0

        last_u = None
        for j in range(n_inner):
            t = t0 + j * dt_e
            TL = load_torque(t)

            y_in = motor.outputs(motor.x)
            ids, iqs = y_in["ids"], y_in["iqs"]

            psi_dr = y_in["psi_dr"]
            psi_qr = y_in["psi_qr"]
            psi_r_mag_in = float(np.hypot(psi_dr, psi_qr))

            omega_r_e = p.p * y_in["omega_m"]
            omega_sl = (k_slip * iqs / psi_r_mag_in) if psi_r_mag_in > eps_psi else 0.0
            omega_e = omega_r_e + omega_sl

            u = current_ctl.step(
                id_ref=id_ref(t),
                iq_ref=iq_ref,
                id_meas=ids,
                iq_meas=iqs,
                omega_e=omega_e,
                dt=dt_e
            )
            vd, vq = u["vd"], u["vq"]

            def fx(xx):
                return motor.f(xx, vd, vq, omega_e, TL)

            motor.x = rk4_step(fx, motor.x, dt_e)
            last_u = u

        y2 = motor.outputs(motor.x)
        omega_rpm = y2["omega_m"] * 60.0 / (2*np.pi)

        log.append(
            t=t0,
            omega_ref_rpm=omega_ref_rpm(t0),
            omega_rpm=omega_rpm,
            omega_m=y2["omega_m"],
            omega_e=omega_e,
            ids=y2["ids"],
            iqs=y2["iqs"],
            id_ref=id_ref(t0),
            iq_ref=iq_ref,
            iq_unsat=iq_unsat,
            aw=aw,
            Te=y2["Te"],
            T_L=load_torque(t0),
            vd=last_u["vd"] if last_u else 0.0,
            vq=last_u["vq"] if last_u else 0.0,
            psi_r_mag=float(np.hypot(y2["psi_dr"], y2["psi_qr"])),
            mpc_status=0.0 if mpc_status in ("na", None) else 1.0,  # simple numeric flag
        )

    df = log.to_dataframe()

    # Metrics over interval after step (e.g., t>=0.3)
    mask = df["t"].values >= 0.3
    y = df.loc[mask, "omega_rpm"].values
    yref = df.loc[mask, "omega_ref_rpm"].values
    t = df.loc[mask, "t"].values

    met = {
        "rmse_rpm": rmse(y, yref),
        "ise": ise(y, yref),
        "itae": itae(t - t[0], y, yref),
        "overshoot_%": overshoot_percent(y, float(yref[-1] if len(yref) else 0.0)),
        "final_speed_rpm": float(df["omega_rpm"].values[-1]),
    }
    return df, met


def run_speed_pid_demo():
    os.makedirs("outputs", exist_ok=True)
    df, met = _speed_control_sim("pid")
    df.to_csv("outputs/speed_pid.csv", index=False)

    fig = plt.figure(figsize=(9,7))
    ax1 = fig.add_subplot(311)
    ax1.plot(df["t"], df["omega_rpm"], label="omega (rpm)")
    ax1.plot(df["t"], df["omega_ref_rpm"], "--", label="omega* (rpm)")
    ax1.set_ylabel("Speed (rpm)")
    ax1.grid(True); ax1.legend()

    ax2 = fig.add_subplot(312)
    ax2.plot(df["t"], df["iqs"], label="i_q")
    ax2.plot(df["t"], df["iq_ref"], "--", label="i_q*")
    ax2.plot(df["t"], df["ids"], label="i_d")
    ax2.plot(df["t"], df["id_ref"], "--", label="i_d*")
    ax2.set_ylabel("Current (A)")
    ax2.grid(True); ax2.legend(ncols=2)

    ax3 = fig.add_subplot(313)
    ax3.plot(df["t"], df["Te"], label="T_e")
    ax3.plot(df["t"], df["T_L"], "--", label="T_L")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Torque (N*m)")
    ax3.grid(True); ax3.legend()

    fig.tight_layout()
    fig.savefig("outputs/speed_pid.png", dpi=160)
    plt.close(fig)

    print("=== Speed PID demo finished ===")
    print(met)
    print("Saved: outputs/speed_pid.csv and outputs/speed_pid.png")


def run_speed_mpc_demo():
    os.makedirs("outputs", exist_ok=True)
    df, met = _speed_control_sim("mpc")
    df.to_csv("outputs/speed_mpc.csv", index=False)

    fig = plt.figure(figsize=(9,7))
    ax1 = fig.add_subplot(311)
    ax1.plot(df["t"], df["omega_rpm"], label="omega (rpm)")
    ax1.plot(df["t"], df["omega_ref_rpm"], "--", label="omega* (rpm)")
    ax1.set_ylabel("Speed (rpm)")
    ax1.grid(True); ax1.legend()

    ax2 = fig.add_subplot(312)
    ax2.plot(df["t"], df["iqs"], label="i_q")
    ax2.plot(df["t"], df["iq_ref"], "--", label="i_q* (MPC)")
    ax2.plot(df["t"], df["ids"], label="i_d")
    ax2.plot(df["t"], df["id_ref"], "--", label="i_d*")
    ax2.set_ylabel("Current (A)")
    ax2.grid(True); ax2.legend(ncols=2)

    ax3 = fig.add_subplot(313)
    ax3.plot(df["t"], df["Te"], label="T_e")
    ax3.plot(df["t"], df["T_L"], "--", label="T_L")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Torque (N*m)")
    ax3.grid(True); ax3.legend()

    fig.tight_layout()
    fig.savefig("outputs/speed_mpc.png", dpi=160)
    plt.close(fig)

    print("=== Speed MPC demo finished ===")
    print(met)
    print("Saved: outputs/speed_mpc.csv and outputs/speed_mpc.png")


def run_compare_pid_mpc_demo():
    os.makedirs("outputs", exist_ok=True)
    df_pid, met_pid = _speed_control_sim("pid")
    df_mpc, met_mpc = _speed_control_sim("mpc")

    # Save metrics
    import pandas as pd
    mdf = pd.DataFrame([
        {"controller": "PID", **met_pid},
        {"controller": "MPC", **met_mpc},
    ])
    mdf.to_csv("outputs/compare_metrics.csv", index=False)

    fig = plt.figure(figsize=(10,7))
    ax1 = fig.add_subplot(211)
    ax1.plot(df_pid["t"], df_pid["omega_rpm"], label="PID ω (rpm)")
    ax1.plot(df_mpc["t"], df_mpc["omega_rpm"], label="MPC ω (rpm)")
    ax1.plot(df_pid["t"], df_pid["omega_ref_rpm"], "--", label="ω* (rpm)")
    ax1.set_ylabel("Speed (rpm)")
    ax1.grid(True); ax1.legend()

    ax2 = fig.add_subplot(212)
    ax2.plot(df_pid["t"], df_pid["iq_ref"], label="PID i_q*")
    ax2.plot(df_mpc["t"], df_mpc["iq_ref"], label="MPC i_q*")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("i_q* (A)")
    ax2.grid(True); ax2.legend()

    fig.tight_layout()
    fig.savefig("outputs/compare_pid_mpc.png", dpi=160)
    plt.close(fig)

    print("=== Compare PID vs MPC finished ===")
    print(mdf)
    print("Saved: outputs/compare_metrics.csv and outputs/compare_pid_mpc.png")
