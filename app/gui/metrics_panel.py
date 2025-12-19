from __future__ import annotations

from PySide6.QtWidgets import QWidget, QGridLayout, QLabel, QFrame
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
import pandas as pd


class MetricsPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._build()

    def _build(self):
        layout = QGridLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(6)

        title = QLabel("Metrics (after ω* step)")
        title.setFont(QFont("Segoe UI", 11, QFont.Bold))
        layout.addWidget(title, 0, 0, 1, 3)

        # headers
        self.h_metric = QLabel("Metric"); self.h_metric.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.h_pid = QLabel("PID"); self.h_pid.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.h_mpc = QLabel("MPC"); self.h_mpc.setFont(QFont("Segoe UI", 10, QFont.Bold))
        layout.addWidget(self.h_metric, 1, 0)
        layout.addWidget(self.h_pid, 1, 1)
        layout.addWidget(self.h_mpc, 1, 2)

        self.rows = []
        self.keys = ["rmse_rpm", "ise", "itae", "overshoot_%", "settling_s", "final_speed_rpm"]
        self.pretty = {
            "rmse_rpm": "RMSE (rpm)",
            "ise": "ISE",
            "itae": "ITAE",
            "overshoot_%": "Overshoot (%)",
            "settling_s": "Settling time 2% (s)",
            "final_speed_rpm": "Final speed (rpm)",
        }

        font_val = QFont("Consolas", 10)
        for i, k in enumerate(self.keys):
            r = 2 + i
            lab = QLabel(self.pretty.get(k, k))
            pid = QLabel("-"); pid.setFont(font_val)
            mpc = QLabel("-"); mpc.setFont(font_val)
            layout.addWidget(lab, r, 0)
            layout.addWidget(pid, r, 1)
            layout.addWidget(mpc, r, 2)
            self.rows.append((k, pid, mpc))

        # subtle separator
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setFrameShadow(QFrame.Sunken)
        layout.addWidget(sep, 2 + len(self.keys), 0, 1, 3)

        hint = QLabel("Tip: Run Both to populate PID and MPC metrics.")
        hint.setStyleSheet("color: gray;")
        layout.addWidget(hint, 3 + len(self.keys), 0, 1, 3)

        layout.setColumnStretch(0, 2)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 1)

        self.setMinimumHeight(220)

    @staticmethod
    def _fmt(v):
        if v is None:
            return "-"
        try:
            vv = float(v)
            if vv == float("inf"):
                return "inf"
            if abs(vv) >= 1e6:
                return f"{vv:.3e}"
            if abs(vv) >= 1000:
                return f"{vv:.1f}"
            if abs(vv) >= 10:
                return f"{vv:.3f}"
            return f"{vv:.4f}"
        except Exception:
            return str(v)

    def clear(self):
        for k, pid, mpc in self.rows:
            pid.setText("-")
            mpc.setText("-")

    def set_metrics(self, mdf: pd.DataFrame | None):
        self.clear()
        if mdf is None or len(mdf) == 0:
            return
        pid_row = mdf[mdf["controller"] == "PID"].iloc[0] if (mdf["controller"] == "PID").any() else None
        mpc_row = mdf[mdf["controller"] == "MPC"].iloc[0] if (mdf["controller"] == "MPC").any() else None

        for k, pid, mpc in self.rows:
            if pid_row is not None and k in pid_row:
                pid.setText(self._fmt(pid_row[k]))
            if mpc_row is not None and k in mpc_row:
                mpc.setText(self._fmt(mpc_row[k]))
