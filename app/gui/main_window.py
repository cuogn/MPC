from __future__ import annotations

from PySide6.QtCore import Qt, Slot
from PySide6.QtWidgets import (
    QWidget, QMainWindow, QHBoxLayout, QVBoxLayout, QFormLayout,
    QPushButton, QLabel, QDoubleSpinBox, QSpinBox, QGroupBox,
    QFileDialog, QMessageBox
)
import pyqtgraph as pg
import pandas as pd
import json
from datetime import datetime
import os

from app.gui.worker_realtime import RealtimeWorker, SimConfig
from app.gui.schematic_2d import Schematic2DView
from app.gui.metrics_panel import MetricsPanel


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IM Speed Control Demo — PID vs MPC (2D Dashboard)")

        self.worker: RealtimeWorker | None = None
        self.last_pid: pd.DataFrame | None = None
        self.last_mpc: pd.DataFrame | None = None
        self.last_metrics: pd.DataFrame | None = None

        self._build_ui()
        self._connect()
        self._update_enable()
        self.resize(1280, 760)

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)

        # ---------------- Left: Controls ----------------
        left = QVBoxLayout()
        layout.addLayout(left, 0)

        scen_box = QGroupBox("Scenario")
        scen_layout = QFormLayout(scen_box)

        self.sp_omega_step = QDoubleSpinBox(); self.sp_omega_step.setRange(0, 4000); self.sp_omega_step.setValue(1500.0); self.sp_omega_step.setSuffix(" rpm")
        self.sp_t_omega = QDoubleSpinBox(); self.sp_t_omega.setRange(0, 10); self.sp_t_omega.setValue(0.30); self.sp_t_omega.setSingleStep(0.05); self.sp_t_omega.setSuffix(" s")
        self.sp_tl_step = QDoubleSpinBox(); self.sp_tl_step.setRange(0, 50); self.sp_tl_step.setValue(5.0); self.sp_tl_step.setSuffix(" N·m")
        self.sp_t_tl = QDoubleSpinBox(); self.sp_t_tl.setRange(0, 10); self.sp_t_tl.setValue(1.50); self.sp_t_tl.setSingleStep(0.05); self.sp_t_tl.setSuffix(" s")
        self.sp_iq_lim = QDoubleSpinBox(); self.sp_iq_lim.setRange(1, 200); self.sp_iq_lim.setValue(20.0); self.sp_iq_lim.setSuffix(" A")

        scen_layout.addRow("ω* step", self.sp_omega_step)
        scen_layout.addRow("t(ω* step)", self.sp_t_omega)
        scen_layout.addRow("T_L step", self.sp_tl_step)
        scen_layout.addRow("t(T_L step)", self.sp_t_tl)
        scen_layout.addRow("|i_q| limit", self.sp_iq_lim)

        pid_box = QGroupBox("PID Speed (outer)")
        pid_layout = QFormLayout(pid_box)
        self.sp_kp = QDoubleSpinBox(); self.sp_kp.setRange(0, 50); self.sp_kp.setDecimals(3); self.sp_kp.setValue(0.8)
        self.sp_ki = QDoubleSpinBox(); self.sp_ki.setRange(0, 200); self.sp_ki.setDecimals(3); self.sp_ki.setValue(10.0)
        pid_layout.addRow("Kp", self.sp_kp)
        pid_layout.addRow("Ki", self.sp_ki)

        mpc_box = QGroupBox("MPC Speed (outer)")
        mpc_layout = QFormLayout(mpc_box)
        self.sp_np = QSpinBox(); self.sp_np.setRange(5, 100); self.sp_np.setValue(25)
        self.sp_Q = QDoubleSpinBox(); self.sp_Q.setRange(0, 1000); self.sp_Q.setDecimals(3); self.sp_Q.setValue(1.0)
        self.sp_R = QDoubleSpinBox(); self.sp_R.setRange(0, 1000); self.sp_R.setDecimals(5); self.sp_R.setValue(0.02)
        self.sp_Rd = QDoubleSpinBox(); self.sp_Rd.setRange(0, 1000); self.sp_Rd.setDecimals(3); self.sp_Rd.setValue(0.2)
        mpc_layout.addRow("Np", self.sp_np)
        mpc_layout.addRow("Q", self.sp_Q)
        mpc_layout.addRow("R", self.sp_R)
        mpc_layout.addRow("Rd(Δu)", self.sp_Rd)

        preset_box = QGroupBox("Presets")
        preset_layout = QHBoxLayout(preset_box)
        self.btn_save_preset = QPushButton("Save preset")
        self.btn_load_preset = QPushButton("Load preset")
        preset_layout.addWidget(self.btn_save_preset)
        preset_layout.addWidget(self.btn_load_preset)

        run_box = QGroupBox("Run (realtime)")
        run_layout = QVBoxLayout(run_box)
        self.btn_run_pid = QPushButton("Run PID")
        self.btn_run_mpc = QPushButton("Run MPC")
        self.btn_run_both = QPushButton("Run Both (compare)")
        self.btn_stop = QPushButton("Stop")
        self.btn_reset = QPushButton("Reset")
        self.btn_export = QPushButton("Export Bundle")
        self.lbl_status = QLabel("Status: ready")
        self.lbl_status.setWordWrap(True)

        for b in (self.btn_run_pid, self.btn_run_mpc, self.btn_run_both, self.btn_stop, self.btn_reset, self.btn_export):
            run_layout.addWidget(b)
        run_layout.addWidget(self.lbl_status)

        # NEW: readable metrics panel
        self.metrics_panel = MetricsPanel()

        left.addWidget(scen_box)
        left.addWidget(pid_box)
        left.addWidget(mpc_box)
        left.addWidget(preset_box)
        left.addWidget(run_box)
        left.addWidget(self.metrics_panel)
        left.addStretch(1)

        # Make left column a bit wider so labels and metrics are readable
        root.setMinimumWidth(1200)

        # ---------------- Right: Dashboard + Plots ----------------
        right = QVBoxLayout()
        layout.addLayout(right, 1)

        pg.setConfigOptions(antialias=True)

        self.schematic = Schematic2DView()
        right.addWidget(self.schematic, 0)

        self.plot_speed = pg.PlotWidget(title="Speed")
        self.plot_speed.setLabel("left", "rpm")
        self.plot_speed.setLabel("bottom", "time", units="s")
        self.plot_speed.showGrid(x=True, y=True)
        self.plot_speed.setDownsampling(auto=True, mode="peak")
        self.plot_speed.setClipToView(True)
        self.curve_omega_pid = self.plot_speed.plot([], [], name="PID ω")
        self.curve_omega_mpc = self.plot_speed.plot([], [], name="MPC ω")
        self.curve_omega_ref = self.plot_speed.plot([], [], pen=pg.mkPen(style=Qt.DashLine), name="ω*")

        self.plot_iq = pg.PlotWidget(title="i_q* command")
        self.plot_iq.setLabel("left", "A")
        self.plot_iq.setLabel("bottom", "time", units="s")
        self.plot_iq.showGrid(x=True, y=True)
        self.plot_iq.setDownsampling(auto=True, mode="peak")
        self.plot_iq.setClipToView(True)
        self.curve_iq_pid = self.plot_iq.plot([], [], name="PID i_q*")
        self.curve_iq_mpc = self.plot_iq.plot([], [], name="MPC i_q*")

        right.addWidget(self.plot_speed, 1)
        right.addWidget(self.plot_iq, 1)

    def _connect(self):
        self.btn_run_pid.clicked.connect(lambda: self._run("PID"))
        self.btn_run_mpc.clicked.connect(lambda: self._run("MPC"))
        self.btn_run_both.clicked.connect(lambda: self._run("BOTH"))
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_reset.clicked.connect(self.on_reset)
        self.btn_export.clicked.connect(self.on_export_bundle)
        self.btn_save_preset.clicked.connect(self.on_save_preset)
        self.btn_load_preset.clicked.connect(self.on_load_preset)

    def _update_enable(self):
        running = self.worker is not None and self.worker.is_running
        for b in (self.btn_run_pid, self.btn_run_mpc, self.btn_run_both, self.btn_save_preset, self.btn_load_preset, self.btn_reset, self.btn_export):
            b.setEnabled(not running)
        self.btn_stop.setEnabled(running)

    def _gather_config(self) -> SimConfig:
        return SimConfig(
            omega_step_rpm=float(self.sp_omega_step.value()),
            t_omega_step=float(self.sp_t_omega.value()),
            tl_step=float(self.sp_tl_step.value()),
            t_tl_step=float(self.sp_t_tl.value()),
            iq_limit=float(self.sp_iq_lim.value()),
            pid_kp=float(self.sp_kp.value()),
            pid_ki=float(self.sp_ki.value()),
            mpc_Np=int(self.sp_np.value()),
            mpc_Q=float(self.sp_Q.value()),
            mpc_R=float(self.sp_R.value()),
            mpc_Rd=float(self.sp_Rd.value()),
            t_end=3.0,
        )

    def _run(self, mode: str):
        if self.worker is not None and self.worker.is_running:
            return
        cfg = self._gather_config()
        # a touch smoother plot refresh
        self.worker = RealtimeWorker(cfg, mode=mode, plot_hz=60.0, window_s=3.0)
        self.worker.sig_status.connect(self.on_status)
        self.worker.sig_data.connect(self.on_data)
        self.worker.sig_done.connect(self.on_done)
        self.worker.start()
        self.on_status(f"running {mode} ...")
        self._update_enable()

    @Slot()
    def on_stop(self):
        if self.worker:
            self.worker.stop()
        self.on_status("stopping...")

    @Slot()
    def on_reset(self):
        if self.worker:
            self.worker.stop()
            self.worker = None
        self.last_pid = None
        self.last_mpc = None
        self.last_metrics = None
        for c in (self.curve_omega_pid, self.curve_omega_mpc, self.curve_omega_ref, self.curve_iq_pid, self.curve_iq_mpc):
            c.setData([], [])
        self.metrics_panel.clear()
        self.on_status("ready")
        self.schematic.update_state(mode="—", rpm_ref=0, rpm_pid=0, rpm_mpc=0, iq_ref_pid=0, iq_ref_mpc=0, te_pid=0, te_mpc=0, tl=0, iq_limit=float(self.sp_iq_lim.value()))
        self._update_enable()

    @Slot(str)
    def on_status(self, msg: str):
        self.lbl_status.setText(f"Status: {msg}")

    @Slot(dict)
    def on_data(self, payload: dict):
        # plots
        if payload.get("t_ref") is not None and len(payload["t_ref"]) > 0:
            self.curve_omega_ref.setData(payload["t_ref"], payload["omega_ref"])

        if len(payload.get("t_pid", [])) > 0:
            self.curve_omega_pid.setData(payload["t_pid"], payload["omega_pid"])
            self.curve_iq_pid.setData(payload["t_pid"], payload["iqref_pid"])
        else:
            self.curve_omega_pid.setData([], [])
            self.curve_iq_pid.setData([], [])

        if len(payload.get("t_mpc", [])) > 0:
            self.curve_omega_mpc.setData(payload["t_mpc"], payload["omega_mpc"])
            self.curve_iq_mpc.setData(payload["t_mpc"], payload["iqref_mpc"])
        else:
            self.curve_omega_mpc.setData([], [])
            self.curve_iq_mpc.setData([], [])

        # schematic: derive last PID/MPC values from arrays if present
        def last(arr, default=0.0):
            return float(arr[-1]) if hasattr(arr, "__len__") and len(arr) else float(default)

        rpm_ref = last(payload.get("omega_ref", []), 0.0)
        rpm_pid = last(payload.get("omega_pid", []), 0.0)
        rpm_mpc = last(payload.get("omega_mpc", []), 0.0)
        iqref_pid = last(payload.get("iqref_pid", []), 0.0)
        iqref_mpc = last(payload.get("iqref_mpc", []), 0.0)

        self.schematic.update_state(
            mode=str(payload.get("mode", "BOTH")),
            rpm_ref=rpm_ref,
            rpm_pid=rpm_pid,
            rpm_mpc=rpm_mpc,
            iq_ref_pid=iqref_pid,
            iq_ref_mpc=iqref_mpc,
            te_pid=float(payload.get("te_pid_now", 0.0)),
            te_mpc=float(payload.get("te_mpc_now", 0.0)),
            tl=float(payload.get("tl_now", 0.0)),
            iq_limit=float(payload.get("iq_limit", float(self.sp_iq_lim.value()))),
        )

    @Slot(dict)
    def on_done(self, payload: dict):
        self.last_pid = payload.get("pid")
        self.last_mpc = payload.get("mpc")
        self.last_metrics = payload.get("metrics")
        self.metrics_panel.set_metrics(self.last_metrics)
        self._update_enable()

    @Slot()
    def on_save_preset(self):
        cfg = self._gather_config().__dict__
        path, _ = QFileDialog.getSaveFileName(self, "Save preset", "preset.json", "JSON Files (*.json)")
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(cfg, f, indent=2)
            QMessageBox.information(self, "Preset", f"Saved:\n{path}")
        except Exception as e:
            QMessageBox.critical(self, "Preset", str(e))

    @Slot()
    def on_load_preset(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load preset", "", "JSON Files (*.json)")
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            self.sp_omega_step.setValue(float(cfg.get("omega_step_rpm", 1500.0)))
            self.sp_t_omega.setValue(float(cfg.get("t_omega_step", 0.30)))
            self.sp_tl_step.setValue(float(cfg.get("tl_step", 5.0)))
            self.sp_t_tl.setValue(float(cfg.get("t_tl_step", 1.50)))
            self.sp_iq_lim.setValue(float(cfg.get("iq_limit", 20.0)))
            self.sp_kp.setValue(float(cfg.get("pid_kp", 0.8)))
            self.sp_ki.setValue(float(cfg.get("pid_ki", 10.0)))
            self.sp_np.setValue(int(cfg.get("mpc_Np", 25)))
            self.sp_Q.setValue(float(cfg.get("mpc_Q", 1.0)))
            self.sp_R.setValue(float(cfg.get("mpc_R", 0.02)))
            self.sp_Rd.setValue(float(cfg.get("mpc_Rd", 0.2)))
            QMessageBox.information(self, "Preset", f"Loaded:\n{path}")
        except Exception as e:
            QMessageBox.critical(self, "Preset", str(e))

    @Slot()
    def on_export_bundle(self):
        if self.last_metrics is None or (self.last_pid is None and self.last_mpc is None):
            QMessageBox.information(self, "Export", "Run a simulation first (PID/MPC/Both).")
            return

        os.makedirs("outputs", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_dir = QFileDialog.getExistingDirectory(self, "Choose parent folder", "outputs")
        if not base_dir:
            return
        out_dir = os.path.join(base_dir, f"run_{ts}")
        os.makedirs(out_dir, exist_ok=True)

        cfg = self._gather_config().__dict__
        with open(os.path.join(out_dir, "config.json"), "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)

        if self.last_pid is not None:
            self.last_pid.to_csv(os.path.join(out_dir, "pid.csv"), index=False)
        if self.last_mpc is not None:
            self.last_mpc.to_csv(os.path.join(out_dir, "mpc.csv"), index=False)

        self.last_metrics.to_csv(os.path.join(out_dir, "metrics.csv"), index=False)

        try:
            from pyqtgraph.exporters import ImageExporter
            ImageExporter(self.plot_speed.plotItem).export(os.path.join(out_dir, "plot_speed.png"))
            ImageExporter(self.plot_iq.plotItem).export(os.path.join(out_dir, "plot_iq.png"))
        except Exception:
            pass

        QMessageBox.information(self, "Export", f"Exported bundle to:\n{out_dir}")
