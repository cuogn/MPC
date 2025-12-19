from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from scipy.optimize import lsq_linear

@dataclass
class MPCParams:
    Ts: float
    Np: int
    Q: float
    R: float
    Rd: float
    iq_min: float
    iq_max: float
    max_iter: int = 50

class SpeedMPC:
    """Bounded least-squares MPC (no external QP solver needed).

    Model (scalar):
      w_{k+1} = A w_k + B u_k + G d_k
      u_k := i_q_ref, d_k := T_L

    Solve:
      min Σ Q (w - wref)^2 + Σ R u^2 + Σ Rd (Δu)^2
      s.t. iq_min <= u_k <= iq_max

    We rewrite as bounded least squares:
      min ||M u - b||_2^2  with bounds on u
    using scipy.optimize.lsq_linear.
    """

    def __init__(self, p: MPCParams):
        self.p = p
        self.u_prev = 0.0
        self.u_warm = np.zeros(self.p.Np, dtype=float)

    def reset(self, u0: float = 0.0):
        self.u_prev = float(u0)
        self.u_warm[:] = float(u0)

    def _build_prediction_mats(self, A: float, B: float, G: float, N: int):
        # Gamma: mapping u[0..N-1] -> w1..wN
        # Gamma_d: mapping d[0..N-1] -> w1..wN
        Gamma = np.zeros((N, N), dtype=float)
        Gamma_d = np.zeros((N, N), dtype=float)
        a_pows = np.ones(N+1, dtype=float)
        for k in range(1, N+1):
            a_pows[k] = a_pows[k-1] * A

        for k in range(N):  # row for w_{k+1}
            for j in range(k+1):
                Gamma[k, j] = a_pows[k-j] * B
                Gamma_d[k, j] = a_pows[k-j] * G
        Phi = a_pows[1:N+1]  # w contribution from w0
        return Phi, Gamma, Gamma_d

    def _diff_matrix(self, N: int):
        # du0 = u0 - u_prev, duk = u_k - u_{k-1}
        D = np.zeros((N, N), dtype=float)
        D[0, 0] = 1.0
        for k in range(1, N):
            D[k, k] = 1.0
            D[k, k-1] = -1.0
        return D

    def step(self, w0: float, wref_seq: np.ndarray, A: float, B: float, G: float, d_seq: np.ndarray):
        N = self.p.Np
        wref = np.asarray(wref_seq, dtype=float).reshape(-1)
        d = np.asarray(d_seq, dtype=float).reshape(-1)
        if wref.size != N or d.size != N:
            raise ValueError("wref_seq and d_seq must have length Np")

        Phi, Gamma, Gamma_d = self._build_prediction_mats(A, B, G, N)
        D = self._diff_matrix(N)

        # c = (Phi*w0 + Gamma_d*d) - wref
        # Note: Gamma_d is N x N lower-triangular, so multiply by d vector
        c = Phi * float(w0) + Gamma_d.dot(d) - wref

        # Build least squares: minimize ||sqrt(Q)*(Gamma u + c)||^2 + ||sqrt(R)*u||^2 + ||sqrt(Rd)*(D u + d0)||^2
        sqrtQ = np.sqrt(self.p.Q)
        sqrtR = np.sqrt(self.p.R)
        sqrtRd = np.sqrt(self.p.Rd)

        # d0 = [-u_prev, 0, 0, ...]
        d0 = np.zeros(N, dtype=float)
        d0[0] = -self.u_prev

        A_ls = np.vstack([
            sqrtQ * Gamma,
            sqrtR * np.eye(N),
            sqrtRd * D
        ])
        b_ls = np.concatenate([
            -sqrtQ * c,
            np.zeros(N, dtype=float),
            -sqrtRd * d0
        ])

        res = lsq_linear(
            A_ls, b_ls,
            bounds=(self.p.iq_min, self.p.iq_max),
            max_iter=self.p.max_iter,
            verbose=0,
            method="trf",
        )

        if not res.success:
            u_seq = np.clip(self.u_warm, self.p.iq_min, self.p.iq_max)
            status = "fallback"
        else:
            u_seq = res.x
            status = "ok"

        u0 = float(u_seq[0])
        self.u_prev = u0
        # warm shift
        self.u_warm[:-1] = u_seq[1:]
        self.u_warm[-1] = u_seq[-1]

        return {"iq_ref": u0, "u_seq": u_seq, "status": status, "nfev": getattr(res, "nfev", None)}
