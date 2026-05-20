from __future__ import annotations

from PySide6.QtCore import Qt, Slot, QSize
from PySide6.QtWidgets import (
    QWidget, QMainWindow, QHBoxLayout, QVBoxLayout, QFormLayout, QGridLayout,
    QPushButton, QLabel, QDoubleSpinBox, QSpinBox, QGroupBox, QComboBox,
    QFileDialog, QMessageBox, QScrollArea, QTableWidget, QTableWidgetItem,
    QHeaderView, QSlider, QButtonGroup, QCheckBox, QSizePolicy, QApplication
)
from PySide6.QtGui import QIcon, QFont
import pyqtgraph as pg
import pandas as pd
import numpy as np
import json
from datetime import datetime
import os

from app.gui.worker_realtime import RealtimeWorker, SimConfig

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IM Speed Control Research Dashboard")
        
        self.worker: RealtimeWorker | None = None
        self._thread_active: bool = False
        self.profile_ref: list[tuple[float, float]] | None = None
        self.profile_tl: list[tuple[float, float]] | None = None
        self.last_pid: pd.DataFrame | None = None
        self.last_mpc: pd.DataFrame | None = None
        self.last_metrics: pd.DataFrame | None = None
        
        self.font_pct = 100 

        pg.setConfigOption('background', '#0D162D') 
        pg.setConfigOption('foreground', '#E2E8F0')
        pg.setConfigOptions(antialias=True)

        self._build_ui()
        self._apply_qss() 
        self._connect()
        self._update_enable()
        self.resize(1600, 900) 

    def _apply_qss(self):
        sz_base = max(9, int(12 * self.font_pct / 100))
        sz_small = max(8, int(10 * self.font_pct / 100))
        sz_h1 = max(14, int(18 * self.font_pct / 100))
        sz_h2 = max(12, int(16 * self.font_pct / 100))
        sz_kpi = max(14, int(18 * self.font_pct / 100))
        sz_icon = max(18, int(22 * self.font_pct / 100))
        
        # Tìm đường dẫn tuyệt đối đến file SVG
        import os
        
        # Thử nhiều đường dẫn khác nhau
        possible_paths = [
            ("app/gui/up-arrow-svgrepo-com.svg", "app/gui/down-arrow-backup-2-svgrepo-com.svg"),
            (os.path.join(os.path.dirname(os.path.abspath(__file__)), "app/gui/up-arrow-svgrepo-com.svg"),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "app/gui/down-arrow-backup-2-svgrepo-com.svg")),
            (os.path.join(os.getcwd(), "app/gui/up-arrow-svgrepo-com.svg"),
            os.path.join(os.getcwd(), "app/gui/down-arrow-backup-2-svgrepo-com.svg")),
        ]
        
        up_icon_path = None
        down_icon_path = None
        
        for up_path, down_path in possible_paths:
            if os.path.exists(up_path) and os.path.exists(down_path):
                up_icon_path = up_path
                down_icon_path = down_path
                break
        
        # Nếu không tìm thấy, tạo thư mục và file SVG mẫu
        if up_icon_path is None or down_icon_path is None:
            # Tạo thư mục nếu chưa có
            gui_dir = os.path.join(os.getcwd(), "app", "gui")
            os.makedirs(gui_dir, exist_ok=True)
            
            up_icon_path = os.path.join(gui_dir, "up-arrow-svgrepo-com.svg")
            down_icon_path = os.path.join(gui_dir, "down-arrow-backup-2-svgrepo-com.svg")
            
            # Tạo file SVG mẫu cho mũi tên lên
            with open(up_icon_path, 'w') as f:
                f.write('''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="24" height="24">
                <path fill="#48CAE4" d="M12 4l-8 8h6v8h4v-8h6z"/>
                </svg>''')
            
            # Tạo file SVG mẫu cho mũi tên xuống
            with open(down_icon_path, 'w') as f:
                f.write('''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="24" height="24">
                <path fill="#48CAE4" d="M12 20l8-8h-6v-8h-4v8h-6z"/>
                </svg>''')
        
        # Chuyển đổi path cho QSS (dùng forward slashes)
        up_icon_path = up_icon_path.replace('\\', '/')
        down_icon_path = down_icon_path.replace('\\', '/')
        
        up_icon = f"url({up_icon_path})"
        down_icon = f"url({down_icon_path})"

        dark_qss = f"""
        QMainWindow, QWidget {{
            background-color: #060B19;
            color: #E2E8F0;
            font-family: 'Inter', 'Segoe UI', sans-serif;
            font-size: {sz_base}px;
        }}
        
        QLabel, QCheckBox, QSlider {{ background-color: transparent; }}
        
        QGroupBox {{
            background-color: #0D162D;
            border: 1px solid #1E2E5D;
            border-radius: 6px;
            margin-top: 12px; 
            padding-top: 18px; 
            font-weight: bold;
            color: #48CAE4;
        }}
        QGroupBox::title {{
            subcontrol-origin: padding;
            subcontrol-position: top left;
            left: 10px; 
            top: 0px; 
            padding: 2px;
            font-size: {sz_base}px;
        }}
        
        QPushButton {{
            background-color: #1E2E5D;
            border: 1px solid #2A3F7A;
            border-radius: 4px;
            padding: 6px;
            color: #E2E8F0;
        }}
        QPushButton:hover {{ background-color: #2A3F7A; }}
        QPushButton:disabled {{ background-color: #111B33; color: #555; border: 1px solid #111; }}
        
        QPushButton.FontBtn {{ background-color: #0D162D; border: 1px solid #48CAE4; font-weight: bold; padding: 2px 10px; }}
        QPushButton.FontBtn:hover {{ background-color: #1E2E5D; }}
        
        QPushButton#btnStart {{ background-color: #16A34A; color: white; font-weight: bold; border: 1px solid #14532d; }}
        QPushButton#btnStart:hover {{ background-color: #15803D; }}
        QPushButton#btnStop {{ background-color: #E74C3C; color: white; font-weight: bold; }}
        QPushButton#btnStop:hover {{ background-color: #c0392b; }}
        QPushButton#btnOriginalReset {{ background-color: #556075; color: white; font-weight: bold; }}
        
        QWidget#ModeBox {{ background-color: #0D162D; border: 1px solid #1E2E5D; border-radius: 12px; padding: 3px; }}
        QPushButton#btnModeActive {{ background-color: #16A34A; color: white; font-weight: bold; border-radius: 8px; margin: 0px 2px; }}
        QPushButton#btnMode {{ background-color: transparent; border: 1px solid transparent; border-radius: 8px; margin: 0px 2px; }}
        QPushButton#btnMode:hover {{ background-color: #1E2E5D; }}

        QLineEdit {{
            background-color: #060B19; 
            border: 1px solid #1E2E5D; 
            border-radius: 4px; 
            padding: 4px;
        }}
        
        /* Style cho SpinBox với nút dọc */
        QSpinBox, QDoubleSpinBox {{
            background-color: #060B19;
            border: 1px solid #1E2E5D;
            border-radius: 4px;
            padding: 4px;
            min-height: 28px;
            padding-right: 24px;
        }}
        
        /* Định dạng vùng chứa nút */
        QSpinBox::up-button, QDoubleSpinBox::up-button {{
            subcontrol-origin: border;
            subcontrol-position: top right;
            width: 20px;
            height: 14px;
            top: 1px;
            right: 1px;
            background-color: #1E2E5D;
            border: none;
            border-radius: 2px;
        }}
        
        QSpinBox::down-button, QDoubleSpinBox::down-button {{
            subcontrol-origin: border;
            subcontrol-position: bottom right;
            width: 20px;
            height: 14px;
            bottom: 1px;
            right: 1px;
            background-color: #1E2E5D;
            border: none;
            border-radius: 2px;
        }}
        
        /* Hiển thị SVG cho mũi tên */
        QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {{
            image: {up_icon};
            width: 12px;
            height: 12px;
        }}
        
        QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {{
            image: {down_icon};
            width: 12px;
            height: 12px;
        }}
        
        /* Hover effects */
        QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
        QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {{
            background-color: #2A3F7A;
        }}
        
        QCheckBox {{ spacing: 8px; }}
        QCheckBox::indicator {{ width: 18px; height: 18px; border: 1px solid #1E2E5D; border-radius: 4px; background-color: #060B19; }}
        QCheckBox::indicator:checked {{ background-color: #16A34A; border: 1px solid #14532d; }}
        
        QTableWidget {{ background-color: transparent; border: none; gridline-color: #1E2E5D; }}
        QHeaderView::section {{ background-color: #1E2E5D; color: white; padding: 5px; border: none; font-weight: bold; font-size: {sz_base}px; }}
        QScrollArea {{ border: none; }}
        
        QWidget#BorderBox {{ background-color: #0D162D; border: 1px solid #1E2E5D; border-radius: 6px; }}
        
        QLabel.LblH1 {{ font-size: {sz_h1}px; font-weight: bold; color: #FFFFFF; }}
        QLabel.LblH2 {{ font-size: {sz_h2}px; font-weight: bold; color: #16A34A; }}
        QLabel.LblSmall {{ font-size: {sz_small}px; color: #7F8C8D; }}
        QLabel.LblIcon {{ font-size: {sz_icon}px; }}
        
        QLabel.KPIName {{ color: #E2E8F0; font-size: {sz_small}px; font-weight: bold; }}
        QLabel.KPIHeaderPID {{ color: #E65F2B; font-size: {max(8, sz_small-1)}px; font-weight: bold; }}
        QLabel.KPIHeaderMPC {{ color: #48CAE4; font-size: {max(8, sz_small-1)}px; font-weight: bold; }}
        QLabel.KPIValPID {{ color: #E65F2B; font-size: {sz_kpi}px; font-weight: bold; }}
        QLabel.KPIValMPC {{ color: #48CAE4; font-size: {sz_kpi}px; font-weight: bold; }}
        QLabel.KPIImp {{ color: #7F8C8D; font-size: {sz_small}px; font-weight: bold; }}
        """
        self.setStyleSheet(dark_qss)
        
        sz_pt = max(8, int(10 * self.font_pct / 100))
        self.plot_speed.setTitle("Speed Reference vs Actual Speed", color="#E2E8F0", size=f"{sz_pt}pt")
        self.plot_err.setTitle("Tracking Error (Speed Error)", color="#E2E8F0", size=f"{sz_pt}pt")
        self.plot_iq.setTitle("Electromagnetic Torque (Command)", color="#E2E8F0", size=f"{sz_pt}pt")

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main_layout = QVBoxLayout(root)
        main_layout.setContentsMargins(15, 10, 15, 10)

        # ================= TẦNG 1: HEADER BAR =================
        header_layout = QHBoxLayout()
        
        lbl_icon = QLabel("📄"); lbl_icon.setProperty("class", "LblIcon")
        lbl_title = QLabel("IM Speed Control Research Dashboard"); lbl_title.setProperty("class", "LblH1")
        lbl_subtitle = QLabel("Comparison of PID and SFMPC for 3 Phase Induction Motor"); lbl_subtitle.setProperty("class", "LblSmall")
        
        header_left = QVBoxLayout()
        header_left.addWidget(lbl_title)
        header_left.addWidget(lbl_subtitle)
        
        header_layout.addWidget(lbl_icon)
        header_layout.addLayout(header_left)
        header_layout.addStretch()

        mode_container = QWidget(); mode_container.setObjectName("ModeBox")
        mode_layout = QHBoxLayout(mode_container)
        mode_layout.setContentsMargins(4, 2, 4, 2)
        mode_layout.setSpacing(0)
        mode_layout.addWidget(QLabel("Mode: "))
        
        self.mode_group = QButtonGroup(self)
        self.btn_mode_pid = QPushButton("PID Only")
        self.btn_mode_mpc = QPushButton("SFMPC Only")
        self.btn_mode_both = QPushButton("Compare (PID vs SFMPC)")
        
        for idx, btn in enumerate([self.btn_mode_pid, self.btn_mode_mpc, self.btn_mode_both]):
            btn.setCheckable(True); btn.setObjectName("btnMode")
            self.mode_group.addButton(btn, idx)
            mode_layout.addWidget(btn)
        self.btn_mode_both.setChecked(True)
        self.mode_group.buttonClicked.connect(self._update_mode_style)
        
        header_layout.addWidget(mode_container)
        header_layout.addStretch()

        self.header_status_box = QWidget(); self.header_status_box.setObjectName("BorderBox")
        self.header_status_box.setFixedWidth(360) 
        h_box_lay = QHBoxLayout(self.header_status_box)
        h_box_lay.setContentsMargins(15, 5, 15, 5)
        
        self.lbl_status_dot = QLabel("●"); self.lbl_status_dot.setStyleSheet("color: #16A34A; font-size: 16px;")
        self.lbl_top_status = QLabel("Status: READY"); self.lbl_top_status.setStyleSheet("color: #16A34A; font-weight: bold;")
        self.lbl_top_time = QLabel("Sim. Time: 0.00 s")
        lbl_clock = QLabel("⏱"); lbl_clock.setProperty("class", "LblIcon")
        lbl_sep = QLabel(" | "); lbl_sep.setStyleSheet("color: #1E2E5D; font-weight: bold;") 
        
        h_box_lay.addWidget(self.lbl_status_dot)
        h_box_lay.addWidget(self.lbl_top_status)
        h_box_lay.addStretch()
        h_box_lay.addWidget(lbl_sep)
        h_box_lay.addWidget(lbl_clock)
        h_box_lay.addWidget(self.lbl_top_time)

        header_layout.addWidget(self.header_status_box)
        main_layout.addLayout(header_layout)
        main_layout.addSpacing(5)

        # ================= TẦNG 2: WORKSPACE =================
        workspace = QHBoxLayout()
        main_layout.addLayout(workspace)

        # ---------------- CỘT TRÁI (Sidebar) - Tăng width và bỏ scroll ----------------
        left_widget = QWidget()
        left_widget.setFixedWidth(360)
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 10, 0)
        left_layout.setSpacing(8)

        grp_sim = QGroupBox("SIMULATION CONTROL")
        lay_sim = QVBoxLayout(grp_sim)
        lay_sim.setContentsMargins(8, 18, 8, 10)
        lay_sim.setSpacing(8)
        lay_sim_r1 = QHBoxLayout()
        self.btn_start = QPushButton("Start"); self.btn_start.setObjectName("btnStart")
        self.btn_stop = QPushButton("Stop"); self.btn_stop.setObjectName("btnStop")
        self.btn_reset = QPushButton("Reset"); self.btn_reset.setObjectName("btnOriginalReset")
        lay_sim_r1.addWidget(self.btn_start); lay_sim_r1.addWidget(self.btn_stop); lay_sim_r1.addWidget(self.btn_reset)
        
        lay_sim_r2 = QHBoxLayout()
        self.btn_load_preset = QPushButton("Load Config")
        self.btn_save_preset = QPushButton("Save Config")
        self.btn_export = QPushButton("Export Data")
        lay_sim_r2.addWidget(self.btn_load_preset); lay_sim_r2.addWidget(self.btn_save_preset); lay_sim_r2.addWidget(self.btn_export)
        
        lay_sim.addLayout(lay_sim_r1); lay_sim.addLayout(lay_sim_r2)
        left_layout.addWidget(grp_sim)

        grp_test = QGroupBox("TEST SCENARIO")
        lay_test = QFormLayout(grp_test)
        lay_test.setContentsMargins(8, 18, 8, 10)
        lay_test.setVerticalSpacing(8)
        lay_test.setHorizontalSpacing(15)
        
        self.sp_omega_step = QDoubleSpinBox(); self.sp_omega_step.setRange(0, 4000); self.sp_omega_step.setValue(1500.0); self.sp_omega_step.setSuffix(" rpm")
        self.sp_t_omega = QDoubleSpinBox(); self.sp_t_omega.setRange(0, 10); self.sp_t_omega.setValue(0.15); self.sp_t_omega.setSingleStep(0.05); self.sp_t_omega.setSuffix(" s")
        self.sp_tl_step = QDoubleSpinBox(); self.sp_tl_step.setRange(0, 50); self.sp_tl_step.setValue(8.0); self.sp_tl_step.setSuffix(" N·m")
        self.chk_disturbance = QCheckBox("ON"); self.chk_disturbance.setChecked(True)
        self.chk_param_mismatch = QCheckBox("ON"); self.chk_param_mismatch.setChecked(False)

        self.hidden_t_tl = 0.0
        self.hidden_iq_lim = 17.0
        
        lay_test.addRow("Reference Speed", self.sp_omega_step)
        lay_test.addRow("Speed Step Time", self.sp_t_omega)
        lay_test.addRow("Load Torque", self.sp_tl_step)
        lay_test.addRow("Disturbance", self.chk_disturbance)
        lay_test.addRow("Param Mismatch", self.chk_param_mismatch)
        left_layout.addWidget(grp_test)

        grp_pid = QGroupBox("PID PARAMETERS")
        lay_pid = QFormLayout(grp_pid)
        lay_pid.setContentsMargins(8, 18, 8, 10)
        lay_pid.setVerticalSpacing(8)
        lay_pid.setHorizontalSpacing(15)
        
        self.sp_kp = QDoubleSpinBox(); self.sp_kp.setRange(0, 50); self.sp_kp.setDecimals(3); self.sp_kp.setValue(0.8)
        self.sp_ki = QDoubleSpinBox(); self.sp_ki.setRange(0, 200); self.sp_ki.setDecimals(3); self.sp_ki.setValue(4.0)
        self.sp_kd = QDoubleSpinBox(); self.sp_kd.setRange(0, 50); self.sp_kd.setDecimals(3); self.sp_kd.setValue(8.0)
        lay_pid.addRow("Kp", self.sp_kp); lay_pid.addRow("Ki", self.sp_ki); lay_pid.addRow("Kd", self.sp_kd)
        left_layout.addWidget(grp_pid)

        grp_mpc = QGroupBox("SFMPC PARAMETERS")
        lay_mpc = QFormLayout(grp_mpc)
        lay_mpc.setContentsMargins(8, 18, 8, 10)
        lay_mpc.setVerticalSpacing(8)
        lay_mpc.setHorizontalSpacing(15)
        
        self.sp_np = QSpinBox()
        self.sp_np.setRange(5, 100)
        self.sp_np.setValue(19)
        self.sp_np.setMinimumHeight(30)
        self.sp_np.setButtonSymbols(QSpinBox.UpDownArrows)
        
        self.sp_nu = QSpinBox()
        self.sp_nu.setRange(1, 100)
        self.sp_nu.setValue(1)
        self.sp_nu.setMinimumHeight(30)
        self.sp_nu.setButtonSymbols(QSpinBox.UpDownArrows)
        
        self.sp_ts_mpc = QDoubleSpinBox()
        self.sp_ts_mpc.setRange(0.0001, 0.1)
        self.sp_ts_mpc.setDecimals(4)
        self.sp_ts_mpc.setValue(0.0010)
        self.sp_ts_mpc.setSingleStep(0.0001)
        self.sp_ts_mpc.setSuffix(" s")
        self.sp_ts_mpc.setMinimumHeight(30)
        self.sp_ts_mpc.setButtonSymbols(QDoubleSpinBox.UpDownArrows)
        
        self.sp_Q = QDoubleSpinBox()
        self.sp_Q.setRange(0, 1000)
        self.sp_Q.setDecimals(2)
        self.sp_Q.setValue(1.00)
        self.sp_Q.setMinimumHeight(30)
        self.sp_Q.setButtonSymbols(QDoubleSpinBox.UpDownArrows)
        
        self.sp_R = QDoubleSpinBox()
        self.sp_R.setRange(0, 1000)
        self.sp_R.setDecimals(2)
        self.sp_R.setValue(0.10)
        self.sp_R.setMinimumHeight(30)
        self.sp_R.setButtonSymbols(QDoubleSpinBox.UpDownArrows)
        
        self.sp_Rd = QDoubleSpinBox()
        self.sp_Rd.setRange(0, 1000)
        self.sp_Rd.setDecimals(2)
        self.sp_Rd.setValue(0.01)
        self.sp_Rd.setMinimumHeight(30)
        self.sp_Rd.setButtonSymbols(QDoubleSpinBox.UpDownArrows)
        
        lay_mpc.addRow("Prediction Horizon (Nc)", self.sp_np)
        lay_mpc.addRow("Control Horizon (Nu)", self.sp_nu)
        lay_mpc.addRow("Sampling Time (Ts)", self.sp_ts_mpc)
        lay_mpc.addRow("λ (Speed Error Weight)", self.sp_Q)
        lay_mpc.addRow("μ (Control Effort Weight)", self.sp_R)
        lay_mpc.addRow("Switching Penalty", self.sp_Rd)
        left_layout.addWidget(grp_mpc)

        grp_motor = QGroupBox("MOTOR & SIMULATION")
        lay_motor = QFormLayout(grp_motor)
        lay_motor.setContentsMargins(8, 18, 8, 10)
        lay_motor.setVerticalSpacing(8)
        lay_motor.setHorizontalSpacing(15)
        
        cbo_motor = QComboBox(); cbo_motor.addItems(["IM - 2.2kW", "IM - 5.0kW"])
        lay_motor.addRow("Motor Model", cbo_motor)
        lay_motor.addRow("Sampling Time", QLabel("0.0010 s"))
        lay_motor.addRow("Noise Level", QLabel("0.0004"))
        left_layout.addWidget(grp_motor)
        
        left_layout.addSpacing(10)

        self.bot_status_widget = QWidget()
        self.bot_status_widget.setObjectName("BorderBox")
        self.bot_status_widget.setFixedHeight(55)
        bot_lay = QHBoxLayout(self.bot_status_widget)
        bot_lay.setContentsMargins(15, 8, 15, 8)
        self.lbl_bot_dot = QLabel("●"); self.lbl_bot_dot.setStyleSheet("color: #16A34A; font-size: 24px;")
        self.lbl_bot_text = QLabel("Ready"); self.lbl_bot_text.setProperty("class", "LblH2")
        bot_lay.addWidget(self.lbl_bot_dot)
        bot_lay.addWidget(self.lbl_bot_text)
        bot_lay.addStretch()
        left_layout.addWidget(self.bot_status_widget)

        workspace.addWidget(left_widget, 22)

        # ---------------- CỘT GIỮA (Đồ thị + KPI) ----------------
        center_layout = QVBoxLayout()
        center_layout.setSpacing(8)
        
        grp_kpi = QGroupBox("SUMMARY INDICATORS (Compare PID vs SFMPC)")
        lay_kpi = QHBoxLayout(grp_kpi)
        
        self.kpi_labels = {}
        self.metrics_keys = ["RMSE (rpm)", "Overshoot (%)", "Settling Time (s)", "Steady-State Error", "Rise Time (s)", "Max Torque Ripple"]
        
        for name in self.metrics_keys[:5]: 
            card = QWidget()
            card.setStyleSheet("background-color: transparent; border-right: 1px solid #1E2E5D;")
            c_lay = QVBoxLayout(card)
            c_lay.setContentsMargins(5, 0, 5, 0)
            c_lay.setSpacing(2)
            
            lbl_name = QLabel(name); lbl_name.setProperty("class", "KPIName")
            lbl_name.setAlignment(Qt.AlignCenter)
            
            sub_hdr_lay = QHBoxLayout()
            lbl_hdr_pid = QLabel("PID"); lbl_hdr_pid.setProperty("class", "KPIHeaderPID")
            lbl_hdr_pid.setAlignment(Qt.AlignCenter)
            lbl_hdr_mpc = QLabel("SFMPC"); lbl_hdr_mpc.setProperty("class", "KPIHeaderMPC")
            lbl_hdr_mpc.setAlignment(Qt.AlignCenter)
            sub_hdr_lay.addWidget(lbl_hdr_pid); sub_hdr_lay.addWidget(lbl_hdr_mpc)
            
            val_lay = QHBoxLayout()
            lbl_pid = QLabel("0.0"); lbl_pid.setProperty("class", "KPIValPID")
            lbl_pid.setAlignment(Qt.AlignCenter)
            lbl_mpc = QLabel("0.0"); lbl_mpc.setProperty("class", "KPIValMPC")
            lbl_mpc.setAlignment(Qt.AlignCenter)
            val_lay.addWidget(lbl_pid); val_lay.addWidget(lbl_mpc)
            
            lbl_imp = QLabel("▼ 0.0%"); lbl_imp.setProperty("class", "KPIImp")
            lbl_imp.setAlignment(Qt.AlignCenter)
            
            c_lay.addWidget(lbl_name)
            c_lay.addLayout(sub_hdr_lay)
            c_lay.addLayout(val_lay)
            c_lay.addWidget(lbl_imp)
            
            self.kpi_labels[name] = {'pid': lbl_pid, 'mpc': lbl_mpc, 'imp': lbl_imp}
            lay_kpi.addWidget(card)
            
        center_layout.addWidget(grp_kpi)

        grp_realtime = QGroupBox("REALTIME MONITORING")
        lay_realtime = QVBoxLayout(grp_realtime)
        lay_realtime.setContentsMargins(5, 20, 5, 10) 
        lay_realtime.setSpacing(8)

        pen_pid = pg.mkPen(color='#E65F2B', width=2.5)   
        pen_mpc = pg.mkPen(color='#48CAE4', width=2.5)   
        pen_ref = pg.mkPen(color='#FFFFFF', width=2.0, style=Qt.DashLine) 
        pen_lim = pg.mkPen(color='#F1C40F', width=1.5, style=Qt.DashLine) 
        
        def wrap_plot(plot_widget):
            container = QWidget(); container.setObjectName("PlotContainer")
            lay = QVBoxLayout(container); lay.setContentsMargins(5, 5, 5, 5) 
            lay.addWidget(plot_widget)
            return container

        self.plot_speed = pg.PlotWidget()
        self.plot_speed.setLabel("left", "Speed (rpm)", **{"color": "#7F8C8D"})
        self.plot_speed.showGrid(x=True, y=True, alpha=0.15)
        self.plot_speed.getPlotItem().layout.setContentsMargins(5, 5, 95, 5)
        leg1 = pg.LegendItem(offset=(0,0))
        leg1.setParentItem(self.plot_speed.graphicsItem())
        leg1.anchor((1, 0), (1, 0), offset=(-5, 10))
        
        self.curve_omega_ref = self.plot_speed.plot([], [], pen=pen_ref)
        self.curve_omega_pid = self.plot_speed.plot([], [], pen=pen_pid)
        self.curve_omega_mpc = self.plot_speed.plot([], [], pen=pen_mpc)
        leg1.addItem(self.curve_omega_ref, "Reference")
        leg1.addItem(self.curve_omega_pid, "PID")
        leg1.addItem(self.curve_omega_mpc, "SFMPC")
        
        self.plot_err = pg.PlotWidget()
        self.plot_err.setLabel("left", "Error (rpm)", **{"color": "#7F8C8D"})
        self.plot_err.showGrid(x=True, y=True, alpha=0.15)
        self.plot_err.getPlotItem().layout.setContentsMargins(5, 5, 95, 5)
        leg2 = pg.LegendItem(offset=(0,0))
        leg2.setParentItem(self.plot_err.graphicsItem())
        leg2.anchor((1, 0), (1, 0), offset=(-5, 10))
        
        self.curve_err_pid = self.plot_err.plot([], [], pen=pen_pid)
        self.curve_err_mpc = self.plot_err.plot([], [], pen=pen_mpc)
        self.curve_err_band_p = self.plot_err.plot([], [], pen=pen_ref)
        self.curve_err_band_n = self.plot_err.plot([], [], pen=pen_ref)
        leg2.addItem(self.curve_err_pid, "PID Error")
        leg2.addItem(self.curve_err_mpc, "SFMPC Error")
        leg2.addItem(self.curve_err_band_p, "± 5 rpm")

        self.plot_iq = pg.PlotWidget()
        self.plot_iq.setLabel("left", "Torque (N·m)", **{"color": "#7F8C8D"})
        self.plot_iq.setLabel("bottom", "Time (s)", **{"color": "#7F8C8D"})
        self.plot_iq.showGrid(x=True, y=True, alpha=0.15)
        self.plot_iq.getPlotItem().layout.setContentsMargins(5, 5, 95, 5)
        leg3 = pg.LegendItem(offset=(0,0))
        leg3.setParentItem(self.plot_iq.graphicsItem())
        leg3.anchor((1, 0), (1, 0), offset=(-5, 10))
        
        self.curve_iq_pid = self.plot_iq.plot([], [], pen=pen_pid)
        self.curve_iq_mpc = self.plot_iq.plot([], [], pen=pen_mpc)
        self.curve_tl_load = self.plot_iq.plot([], [], pen=pen_lim)
        leg3.addItem(self.curve_iq_pid, "PID Torque")
        leg3.addItem(self.curve_iq_mpc, "SFMPC Torque")
        leg3.addItem(self.curve_tl_load, "Load Torque")

        lay_realtime.addWidget(wrap_plot(self.plot_speed), 1)
        lay_realtime.addWidget(wrap_plot(self.plot_err), 1)
        lay_realtime.addWidget(wrap_plot(self.plot_iq), 1)

        lay_slider = QHBoxLayout()
        lay_slider.addWidget(QLabel("Time Window"))
        self.slider_time = QSlider(Qt.Horizontal)
        self.slider_time.setRange(10, 100); self.slider_time.setValue(50) 
        lay_slider.addWidget(self.slider_time)
        self.lbl_time_val = QLabel("5.0 s")
        lay_slider.addWidget(self.lbl_time_val)
        
        lay_realtime.addLayout(lay_slider)
        center_layout.addWidget(grp_realtime, 1) 

        self.tip_widget = QWidget(); self.tip_widget.setObjectName("BorderBox")
        lay_tip = QHBoxLayout(self.tip_widget)
        lay_tip.setContentsMargins(12, 5, 12, 5)
        lbl_tip = QLabel(); lbl_tip.setProperty("class", "LblSmall")
        lbl_tip.setText("<font color='#48CAE4'>🔹</font> <i>Tip: SFMPC shows smaller overshoot, faster settling, and lower steady-state error.</i>")
        lay_tip.addWidget(lbl_tip)
        center_layout.addWidget(self.tip_widget)
        
        workspace.addLayout(center_layout, 58)

        # ---------------- CỘT PHẢI (Metrics + Status) ----------------
        right_layout = QVBoxLayout()
        right_layout.setSpacing(10)
        
        grp_metrics = QGroupBox("PERFORMANCE METRICS (PID vs SFMPC)")
        grp_metrics.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum) 
        lay_tbl = QVBoxLayout(grp_metrics)
        lay_tbl.setContentsMargins(5,15,5,5)
        
        self.tbl_metrics = QTableWidget(6, 4)
        self.tbl_metrics.setFixedHeight(240) 
        self.tbl_metrics.setHorizontalHeaderLabels(["Metric", "PID", "SFMPC", "Improvement"])
        
        item_pid_hdr = QTableWidgetItem("PID")
        item_pid_hdr.setForeground(pg.mkColor('#E65F2B'))
        self.tbl_metrics.setHorizontalHeaderItem(1, item_pid_hdr)
        
        item_mpc_hdr = QTableWidgetItem("SFMPC")
        item_mpc_hdr.setForeground(pg.mkColor('#48CAE4'))
        self.tbl_metrics.setHorizontalHeaderItem(2, item_mpc_hdr)
        
        self.tbl_metrics.verticalHeader().setVisible(False)
        self.tbl_metrics.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tbl_metrics.setEditTriggers(QTableWidget.NoEditTriggers)
        
        for row, name in enumerate(self.metrics_keys):
            self.tbl_metrics.setItem(row, 0, QTableWidgetItem(name))
            for col in range(1, 4):
                item = QTableWidgetItem("0.0"); item.setTextAlignment(Qt.AlignCenter)
                self.tbl_metrics.setItem(row, col, item)

        lay_tbl.addWidget(self.tbl_metrics)
        right_layout.addWidget(grp_metrics) 

        grp_status = QGroupBox("SYSTEM STATUS")
        grp_status.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        lay_status = QVBoxLayout(grp_status)
        lay_status.setContentsMargins(5,15,5,5)
        
        self.tbl_status = QTableWidget(10, 2)
        self.tbl_status.verticalHeader().setVisible(False)
        self.tbl_status.horizontalHeader().setVisible(False)
        self.tbl_status.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        self.tbl_status.verticalHeader().setContentsMargins(0,0,0,0)
        for row_idx in range(9):
            self.tbl_status.verticalHeader().setSectionResizeMode(row_idx, QHeaderView.ResizeToContents)
        self.tbl_status.verticalHeader().setSectionResizeMode(9, QHeaderView.Stretch) 
        self.tbl_status.setEditTriggers(QTableWidget.NoEditTriggers)
        self.tbl_status.setShowGrid(True)
        
        self.status_keys_list = [
            ("Controller Mode", "COMPARE (PID vs SFMPC)"), ("Simulation State", "READY"),
            ("Sim. Time", "0.00 s"), ("Sampling Time", "0.0010 s"),
            ("Reference Speed", "1500.0 rpm"), ("Actual Speed (PID)", "0.0 rpm"),
            ("Actual Speed (SFMPC)", "0.0 rpm"), ("Torque (Est.)", "0.0 N·m"),
            ("Worker State", "Idle"), ("Data Logging", "ON")
        ]
        
        for row, (k, v) in enumerate(self.status_keys_list):
            item_k = QTableWidgetItem(k); item_k.setForeground(pg.mkColor('#E2E8F0'))
            item_v = QTableWidgetItem(v)
            if v in ["READY", "RUNNING", "Active", "ON"]:
                item_v.setForeground(pg.mkColor('#16A34A'))
            else: 
                item_v.setForeground(pg.mkColor('#E2E8F0'))
            self.tbl_status.setItem(row, 0, item_k); self.tbl_status.setItem(row, 1, item_v)

        lay_status.addWidget(self.tbl_status)
        right_layout.addWidget(grp_status)
        
        self.footer_widget = QWidget(); self.footer_widget.setObjectName("BorderBox")
        lay_footer = QHBoxLayout(self.footer_widget)
        lay_footer.setContentsMargins(10, 5, 10, 5)
        
        lay_footer.addWidget(QLabel("Theme 🌙"))
        lay_footer.addStretch()
        
        lay_footer.addWidget(QLabel("Font Size:"))
        self.btn_font_minus = QPushButton("-"); self.btn_font_minus.setProperty("class", "FontBtn")
        self.lbl_font_val = QLabel("100%"); self.lbl_font_val.setFixedWidth(40); self.lbl_font_val.setAlignment(Qt.AlignCenter)
        self.btn_font_plus = QPushButton("+"); self.btn_font_plus.setProperty("class", "FontBtn")
        
        lay_footer.addWidget(self.btn_font_minus)
        lay_footer.addWidget(self.lbl_font_val)
        lay_footer.addWidget(self.btn_font_plus)
        
        right_layout.addWidget(self.footer_widget)
        workspace.addLayout(right_layout, 20) 
        
        self._update_mode_style()

    def _update_mode_style(self):
        for btn in [self.btn_mode_pid, self.btn_mode_mpc, self.btn_mode_both]:
            if btn.isChecked(): btn.setObjectName("btnModeActive")
            else: btn.setObjectName("btnMode")
            btn.style().unpolish(btn); btn.style().polish(btn)
        
        if hasattr(self, 'tbl_status'):
            mode_text = "COMPARE (PID vs SFMPC)"
            if self.btn_mode_pid.isChecked(): mode_text = "PID ONLY"
            elif self.btn_mode_mpc.isChecked(): mode_text = "SFMPC ONLY"
            self.tbl_status.item(0, 1).setText(mode_text)

    def _connect(self):
        self.btn_start.clicked.connect(self._on_start_clicked)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_reset.clicked.connect(self.on_reset)
        self.btn_export.clicked.connect(self.on_export_bundle)
        self.btn_save_preset.clicked.connect(self.on_save_preset)
        self.btn_load_preset.clicked.connect(self.on_load_preset)
        
        self.slider_time.valueChanged.connect(self._on_time_slider_changed)
        self.btn_font_minus.clicked.connect(lambda: self._change_font_size(-10))
        self.btn_font_plus.clicked.connect(lambda: self._change_font_size(10))

    def _on_time_slider_changed(self, val):
        self.lbl_time_val.setText(f"{val/10.0:.1f} s")

    def _change_font_size(self, delta):
        self.font_pct = max(80, min(150, self.font_pct + delta))
        self.lbl_font_val.setText(f"{self.font_pct}%")
        self._apply_qss() 

    def _on_start_clicked(self):
        mode = "BOTH"
        if self.btn_mode_pid.isChecked(): mode = "PID"
        elif self.btn_mode_mpc.isChecked(): mode = "MPC"
        self._run(mode)

    def _worker_active(self) -> bool:
        return bool(self._thread_active or (self.worker and self.worker.isRunning()))

    def _update_enable(self):
        running = self._worker_active()
        for b in (self.btn_start, self.btn_save_preset, self.btn_load_preset, self.btn_reset, self.btn_export):
            b.setEnabled(not running)
        self.btn_stop.setEnabled(running)
        
        state_txt = "RUNNING" if running else "READY"
        color = "#F1C40F" if running else "#16A34A" 
        
        self.lbl_top_status.setText(f"Status: {state_txt}")
        self.lbl_top_status.setStyleSheet(f"color: {color}; font-weight: bold;")
        self.lbl_status_dot.setStyleSheet(f"color: {color}; font-size: 16px;")
        
        self.lbl_bot_dot.setStyleSheet(f"color: {color}; font-size: 24px;")
        self.lbl_bot_text.setText(state_txt)
        self.lbl_bot_text.setStyleSheet(f"color: {color}; font-weight: bold;")
        
        if hasattr(self, 'tbl_status'):
            self.tbl_status.item(1, 1).setText(state_txt)
            self.tbl_status.item(1, 1).setForeground(pg.mkColor(color))
            w_state = "Active" if running else "Idle"
            self.tbl_status.item(8, 1).setText(w_state)
            self.tbl_status.item(8, 1).setForeground(pg.mkColor(color))

    def _gather_config(self) -> SimConfig:
        tl_val = float(self.sp_tl_step.value()) if self.chk_disturbance.isChecked() else 0.0
        
        kp_val = float(self.sp_kp.value())
        ki_val = float(self.sp_ki.value())
        if self.chk_param_mismatch.isChecked():
            kp_val *= 1.3
            ki_val *= 0.7

        return SimConfig(
            omega_step_rpm=float(self.sp_omega_step.value()),
            t_omega_step=float(self.sp_t_omega.value()),
            tl_step=tl_val, 
            t_tl_step=0.0,
            iq_limit=17.0,
            pid_kp=kp_val,
            pid_ki=ki_val,
            mpc_Np=int(self.sp_np.value()),
            mpc_Q=float(self.sp_Q.value()),
            mpc_R=float(self.sp_R.value()),
            mpc_Rd=float(self.sp_Rd.value()),
            t_end=self.slider_time.value() / 10.0,  
            omega_profile=None,
            tl_profile=None,
        )

    def _run(self, mode: str):
        if self._worker_active(): return
        cfg = self._gather_config()
        sim_duration = self.slider_time.value() / 10.0
        self.worker = RealtimeWorker(cfg, mode=mode, plot_hz=60.0, window_s=sim_duration)
        self.worker.sig_status.connect(self.on_status)
        self.worker.sig_data.connect(self.on_data)
        self.worker.sig_done.connect(self.on_done)
        self.worker.finished.connect(self._on_worker_finished)
        self._thread_active = True
        self.worker.start()
        self._update_enable()

    @Slot()
    def _on_worker_finished(self):
        self._thread_active = False; self.worker = None; self._update_enable()

    @Slot()
    def on_stop(self):
        if self.worker: self.worker.stop()
        self._update_enable()

    @Slot()
    def on_reset(self):
        if self.worker: self.worker.stop()
        self.last_pid = None; self.last_mpc = None; self.last_metrics = None
        for c in (self.curve_omega_pid, self.curve_omega_mpc, self.curve_omega_ref, 
                  self.curve_iq_pid, self.curve_iq_mpc, self.curve_tl_load,
                  self.curve_err_pid, self.curve_err_mpc, self.curve_err_band_p, self.curve_err_band_n):
            c.setData([], [])
        
        self.lbl_top_time.setText("Sim. Time: 0.00 s")
        self.tbl_status.item(2, 1).setText("0.00 s")
        self.tbl_status.item(4, 1).setText("0.0 rpm")
        self.tbl_status.item(5, 1).setText("0.0 rpm")
        self.tbl_status.item(6, 1).setText("0.0 rpm")
        self.tbl_status.item(7, 1).setText("0.0 N·m")
        
        self._reset_metrics_ui()
        self._update_enable()

    def _reset_metrics_ui(self):
        for name, lbls in self.kpi_labels.items():
            lbls['pid'].setText("0.0"); lbls['mpc'].setText("0.0"); lbls['imp'].setText("▼ 0.0%")
            lbls['imp'].setStyleSheet("color: #7F8C8D; font-weight: bold;")
        for row in range(self.tbl_metrics.rowCount()):
            for col in range(1, 4):
                item = self.tbl_metrics.item(row, col)
                if item: item.setText("0.0")

    @Slot(str)
    def on_status(self, msg: str): pass 

    def _calculate_realtime_metrics(self, payload: dict) -> pd.DataFrame:
        t_ref_arr = np.array(payload.get("t_ref", []))
        omega_ref_arr = np.array(payload.get("omega_ref", []))
        ref_val = float(self.sp_omega_step.value()) if len(omega_ref_arr) == 0 else float(omega_ref_arr[-1])
        if ref_val == 0: ref_val = 1500.0
        
        def calc_signals(t_arr, omega_arr, iqref_arr):
            t = np.array(t_arr); w = np.array(omega_arr); iq = np.array(iqref_arr)
            if len(t) == 0 or len(w) == 0 or len(t_ref_arr) == 0: return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            
            ref_interp = np.interp(t, t_ref_arr, omega_ref_arr)
            
            rmse = np.sqrt(np.mean((ref_interp - w)**2))
            max_w = np.max(w)
            overshoot = max(0.0, (max_w - ref_val) / ref_val * 100.0)
            err = np.abs(ref_interp - w)
            sse = float(err[-1]) if len(err) > 0 else 0.0
            
            within_band = np.where(err > 5.0)[0]
            ts_val = float(t[within_band[-1]]) if len(within_band) > 0 else 0.0
            
            t_10 = t[np.where(w >= 0.1 * ref_val)[0]]; t_90 = t[np.where(w >= 0.9 * ref_val)[0]]
            tr_val = float(t_90[0] - t_10[0]) if (len(t_10) > 0 and len(t_90) > 0 and t_90[0] > t_10[0]) else 0.0
            ripple = float(np.max(iq) - np.min(iq)) if len(iq) > 0 else 0.0
            return rmse, overshoot, ts_val, sse, tr_val, max(0.0, ripple)

        pid_m = calc_signals(payload.get("t_pid", []), payload.get("omega_pid", []), payload.get("iqref_pid", []))
        mpc_m = calc_signals(payload.get("t_mpc", []), payload.get("omega_mpc", []), payload.get("iqref_mpc", []))
        
        rows = []
        for idx, name in enumerate(self.metrics_keys): rows.append({"Metric": name, "PID": pid_m[idx], "MPC": mpc_m[idx]})
        return pd.DataFrame(rows)

    @Slot(dict)
    def on_data(self, payload: dict):
        t_ref_arr = np.array(payload.get("t_ref", []))
        omega_ref_arr = np.array(payload.get("omega_ref", []))
        
        if len(t_ref_arr) > 0: 
            self.curve_omega_ref.setData(t_ref_arr, omega_ref_arr)
            
        has_pid = len(payload.get("t_pid", [])) > 0
        has_mpc = len(payload.get("t_mpc", [])) > 0

        if has_pid:
            self.curve_omega_pid.setData(payload["t_pid"], payload["omega_pid"])
            self.curve_iq_pid.setData(payload["t_pid"], payload["iqref_pid"])
        if has_mpc:
            self.curve_omega_mpc.setData(payload["t_mpc"], payload["omega_mpc"])
            self.curve_iq_mpc.setData(payload["t_mpc"], payload["iqref_mpc"])

        t_candidates = []
        if has_pid and len(t_ref_arr) > 0:
            t_pid = np.array(payload["t_pid"])
            omega_pid = np.array(payload["omega_pid"])
            ref_interp_pid = np.interp(t_pid, t_ref_arr, omega_ref_arr)
            self.curve_err_pid.setData(t_pid, ref_interp_pid - omega_pid)
            t_candidates.append(float(t_pid[-1]))

        if has_mpc and len(t_ref_arr) > 0:
            t_mpc = np.array(payload["t_mpc"])
            omega_mpc = np.array(payload["omega_mpc"])
            ref_interp_mpc = np.interp(t_mpc, t_ref_arr, omega_ref_arr)
            self.curve_err_mpc.setData(t_mpc, ref_interp_mpc - omega_mpc)
            t_candidates.append(float(t_mpc[-1]))

        t_end_val = self.slider_time.value() / 10.0
        tmax = max(t_candidates) if t_candidates else t_end_val
        
        self.curve_err_band_p.setData([0.0, t_end_val], [5.0, 5.0])
        self.curve_err_band_n.setData([0.0, t_end_val], [-5.0, -5.0])
        tl_step = float(self.sp_tl_step.value()) if self.chk_disturbance.isChecked() else 0.0
        self.curve_tl_load.setData([0.0, t_end_val], [tl_step, tl_step])

        self.lbl_top_time.setText(f"Sim. Time: {tmax:.2f} s")
        self.tbl_status.item(2, 1).setText(f"{tmax:.2f} s")

        def last(arr): return float(arr[-1]) if hasattr(arr, "__len__") and len(arr) else 0.0
        rpm_ref = float(omega_ref_arr[-1]) if len(omega_ref_arr) > 0 else 0.0
        rpm_pid, rpm_mpc = last(payload.get("omega_pid", [])), last(payload.get("omega_mpc", []))
        
        self.tbl_status.item(4, 1).setText(f"{rpm_ref:.1f} rpm")
        self.tbl_status.item(5, 1).setText(f"{rpm_pid:.1f} rpm")
        self.tbl_status.item(6, 1).setText(f"{rpm_mpc:.1f} rpm")
        self.tbl_status.item(7, 1).setText(f"{max(last(payload.get('iqref_pid', [])), last(payload.get('iqref_mpc', []))):.2f} N·m")

        self._update_metrics_ui(self._calculate_realtime_metrics(payload))

    @Slot(dict)
    def on_done(self, payload: dict):
        self.last_pid = payload.get("pid"); self.last_mpc = payload.get("mpc"); self.last_metrics = payload.get("metrics")
        if self.last_metrics is not None and not self.last_metrics.empty: self._update_metrics_ui(self.last_metrics)
        self._thread_active = False; self.worker = None; self._update_enable()

    def _update_metrics_ui(self, df: pd.DataFrame):
        try:
            for i in range(min(df.shape[0], self.tbl_metrics.rowCount())):
                row_data = df.iloc[i]
                metric_name = str(row_data.get('Metric', self.metrics_keys[i] if i < len(self.metrics_keys) else ''))
                self.tbl_metrics.item(i, 0).setText(metric_name)

                pid_val_num = float(row_data.get('PID', 0.0))
                mpc_val_num = float(row_data.get('MPC', 0.0))
                pid_str = f"{pid_val_num:.2f}"; mpc_str = f"{mpc_val_num:.2f}"
                
                self.tbl_metrics.item(i, 1).setText(pid_str); self.tbl_metrics.item(i, 1).setForeground(pg.mkColor('#E65F2B'))
                self.tbl_metrics.item(i, 2).setText(mpc_str); self.tbl_metrics.item(i, 2).setForeground(pg.mkColor('#48CAE4'))

                color_imp = '#16A34A'
                arrow = "▼"
                if pid_val_num != 0:
                    imp_val = ((pid_val_num - mpc_val_num) / pid_val_num) * 100.0
                    if imp_val < 0: arrow = "▲"; color_imp = '#E74C3C'
                    imp_str = f"{arrow} {abs(imp_val):.1f}%"
                else:
                    imp_str = "0.0%" if mpc_val_num == 0 else "▲ 100%"
                    if mpc_val_num > 0: color_imp = '#E74C3C'

                self.tbl_metrics.item(i, 3).setText(imp_str)
                self.tbl_metrics.item(i, 3).setForeground(pg.mkColor(color_imp))

                for k_name, lbl_dict in self.kpi_labels.items():
                    if k_name.split(' ')[0].lower() in metric_name.lower():
                        lbl_dict['pid'].setText(pid_str); lbl_dict['mpc'].setText(mpc_str); lbl_dict['imp'].setText(imp_str)
                        lbl_dict['imp'].setStyleSheet(f"color: {color_imp}; font-weight: bold;")
        except Exception as e: pass

    @Slot()
    def on_save_preset(self): pass
    @Slot()
    def on_load_preset(self): pass
    @Slot()
    def on_export_bundle(self): pass