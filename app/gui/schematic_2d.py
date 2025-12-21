from __future__ import annotations

from dataclasses import dataclass
from PySide6.QtWidgets import QGraphicsView, QGraphicsScene
from PySide6.QtCore import Qt, QRectF, QPointF
from PySide6.QtGui import QPen, QBrush, QPolygonF, QFont, QPainter
import math


@dataclass
class SchematicState:
    # reference
    rpm_ref: float = 0.0

    # PID channel
    rpm_pid: float = 0.0
    iq_ref_pid: float = 0.0
    te_pid: float = 0.0

    # MPC channel
    rpm_mpc: float = 0.0
    iq_ref_mpc: float = 0.0
    te_mpc: float = 0.0

    # common / disturbance
    tl: float = 0.0
    iq_limit: float = 20.0
    mode: str = "BOTH"  # PID / MPC / BOTH


class Schematic2DView(QGraphicsView):
    """A cleaner 2D dashboard.

    - dark theme to match plots
    - dual needles for PID vs MPC when BOTH
    - current command bars (iq_ref) and torque bars
    - arrows: thickness encodes magnitude
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRenderHint(QPainter.Antialiasing, True)
        self.setRenderHint(QPainter.TextAntialiasing, True)
        self.setBackgroundBrush(QBrush(Qt.black))

        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setMinimumHeight(420)

        self.state = SchematicState()

        self.W, self.H = 980, 420
        self.scene.setSceneRect(0, 0, self.W, self.H)

        self._build()

    def _build(self):
        s = self.scene
        s.clear()

        # palette
        self.col_fg = Qt.white
        self.col_panel = Qt.darkGray
        self.col_wire = Qt.gray
        self.col_pid = Qt.cyan      # close to "blue" but visible on dark
        self.col_mpc = Qt.yellow    # visible on dark
        self.col_ref = Qt.green

        # blocks
        pen_box = QPen(self.col_fg, 2)
        brush_box = QBrush(self.col_panel)

        s.addRect(QRectF(40, 210, 170, 95), pen_box, brush_box)
        t = s.addText("INVERTER\n(vd, vq)")
        t.setDefaultTextColor(self.col_fg)
        t.setPos(75, 232)

        # motor: stator + rotor (centered lower to avoid text overlap)
        s.addEllipse(QRectF(330, 170, 160, 160), QPen(self.col_fg, 2), QBrush(Qt.black))
        s.addEllipse(QRectF(365, 205, 90, 90), QPen(self.col_wire, 2), QBrush(Qt.black))
        mt = s.addText("IM\nMotor")
        mt.setDefaultTextColor(self.col_fg)
        mt.setPos(395, 235)

        # shaft + load (adjusted for motor position)
        s.addLine(500, 250, 705, 250, QPen(self.col_wire, 4))
        s.addRect(QRectF(705, 205, 210, 95), pen_box, brush_box)
        lt = s.addText("LOAD\n(TL)")
        lt.setDefaultTextColor(self.col_fg)
        lt.setPos(785, 227)

        # arrows
        self.arrow_i_pid = s.addPolygon(QPolygonF(), QPen(self.col_pid, 1), QBrush(self.col_pid))
        self.arrow_i_mpc = s.addPolygon(QPolygonF(), QPen(self.col_mpc, 1), QBrush(self.col_mpc))
        self.arrow_t_pid = s.addPolygon(QPolygonF(), QPen(self.col_pid, 1), QBrush(self.col_pid))
        self.arrow_t_mpc = s.addPolygon(QPolygonF(), QPen(self.col_mpc, 1), QBrush(self.col_mpc))

        # labels for arrows (aligned to arrow midpoints)
        self.lbl_i = s.addText("i_q*")
        self.lbl_i.setDefaultTextColor(self.col_fg)
        self.lbl_i.setPos(245, 244)
        self.lbl_t = s.addText("T_e")
        self.lbl_t.setDefaultTextColor(self.col_fg)
        self.lbl_t.setPos(560, 272)

        # gauge (centered, larger gap from top panels)
        s.addEllipse(QRectF(270, 100, 280, 280), QPen(self.col_wire, 2))
        self.needle_ref = s.addLine(410, 240, 410, 120, QPen(self.col_ref, 2, Qt.DashLine))
        self.needle_pid = s.addLine(410, 240, 410, 140, QPen(self.col_pid, 3))
        self.needle_mpc = s.addLine(410, 240, 410, 150, QPen(self.col_mpc, 3))

        gl = s.addText("Speed (rpm)")
        gl.setDefaultTextColor(self.col_fg)
        gl.setPos(372, 330)

        # bars panel (Mode info - left top)
        s.addRect(QRectF(25, 15, 290, 110), QPen(self.col_wire, 1), QBrush(Qt.black))
        self.txt = s.addText("")
        self.txt.setDefaultTextColor(self.col_fg)
        self.txt.setFont(QFont("Consolas", 9))
        self.txt.setPos(38, 24)

        # Commands panel (right top) - same height as Mode panel
        s.addRect(QRectF(525, 15, 430, 130), QPen(self.col_wire, 1), QBrush(Qt.black))
        label = s.addText("Commands")
        label.setDefaultTextColor(self.col_fg)
        label.setFont(QFont("Consolas", 9))
        label.setPos(535, 22)

        self.bar_iq_pid = s.addRect(QRectF(535, 48, 0, 10), QPen(Qt.NoPen), QBrush(self.col_pid))
        self.bar_iq_mpc = s.addRect(QRectF(535, 64, 0, 10), QPen(Qt.NoPen), QBrush(self.col_mpc))
        self.txt_iq = s.addText("")
        self.txt_iq.setDefaultTextColor(self.col_fg)
        self.txt_iq.setFont(QFont("Consolas", 8))
        self.txt_iq.setPos(535, 80)

        self._redraw()

    def _arrow_poly(self, x1, y1, x2, y2, width=6.0):
        dx, dy = x2 - x1, y2 - y1
        L = math.hypot(dx, dy) + 1e-9
        ux, uy = dx / L, dy / L
        px, py = -uy, ux
        w = float(width)

        head_len = 18.0
        bx, by = x2 - ux * head_len, y2 - uy * head_len

        p1 = QPointF(x1 + px*w, y1 + py*w)
        p2 = QPointF(x1 - px*w, y1 - py*w)
        p3 = QPointF(bx - px*w, by - py*w)
        p4 = QPointF(bx - px*(2.2*w), by - py*(2.2*w))
        p5 = QPointF(x2, y2)
        p6 = QPointF(bx + px*(2.2*w), by + py*(2.2*w))
        p7 = QPointF(bx + px*w, by + py*w)
        return QPolygonF([p1, p2, p3, p4, p5, p6, p7])

    def _needle(self, item, rpm: float):
        rpm = max(0.0, min(2000.0, float(rpm)))
        ang = (-120.0 + (rpm/2000.0)*240.0) * math.pi/180.0
        cx, cy = 410.0, 240.0
        r = 120.0
        x2 = cx + r*math.cos(ang)
        y2 = cy + r*math.sin(ang)
        item.setLine(cx, cy, x2, y2)

    def _redraw(self):
        st = self.state
        # arrows thickness encodes magnitude
        def norm(val, scale):
            return min(1.0, abs(val) / max(1e-6, scale))

        iq_scale = max(1e-6, st.iq_limit)
        te_scale = 10.0

        w_i_pid = 2.0 + 10.0 * norm(st.iq_ref_pid, iq_scale)
        w_i_mpc = 2.0 + 10.0 * norm(st.iq_ref_mpc, iq_scale)
        w_t_pid = 2.0 + 10.0 * norm(st.te_pid, te_scale)
        w_t_mpc = 2.0 + 10.0 * norm(st.te_mpc, te_scale)

        # current arrows (aligned with inverter/motor height)
        self.arrow_i_pid.setPolygon(self._arrow_poly(210, 250, 320, 250, width=w_i_pid))
        self.arrow_i_mpc.setPolygon(self._arrow_poly(210, 270, 320, 270, width=w_i_mpc))
        # torque arrows (aligned with shaft)
        self.arrow_t_pid.setPolygon(self._arrow_poly(500, 250, 705, 250, width=w_t_pid))
        self.arrow_t_mpc.setPolygon(self._arrow_poly(500, 270, 705, 270, width=w_t_mpc))

        # needles
        self._needle(self.needle_ref, st.rpm_ref)
        self._needle(self.needle_pid, st.rpm_pid)
        self._needle(self.needle_mpc, st.rpm_mpc)

        # bars: map |iq_ref| into width (0..390)
        W = 390.0
        wpid = W * norm(st.iq_ref_pid, iq_scale)
        wmpc = W * norm(st.iq_ref_mpc, iq_scale)
        self.bar_iq_pid.setRect(QRectF(535, 48, wpid, 10))
        self.bar_iq_mpc.setRect(QRectF(535, 64, wmpc, 10))

        # text block (formatted to fit better with proper spacing)
        self.txt.setPlainText(
            f"Mode: {st.mode}\n"
            f"rpm_ref: {st.rpm_ref:7.1f}\n"
            f"PID rpm: {st.rpm_pid:7.1f}\n"
            f"MPC rpm: {st.rpm_mpc:7.1f}\n"
            f"TL: {st.tl:6.2f} N·m\n"
            f"|iq|lim: {st.iq_limit:5.1f} A"
        )
        self.txt_iq.setPlainText(
            f"PID iq*: {st.iq_ref_pid:6.2f} A\n"
            f"MPC iq*: {st.iq_ref_mpc:6.2f} A\n"
            f"PID Te:  {st.te_pid:6.2f} N·m\n"
            f"MPC Te:  {st.te_mpc:6.2f} N·m"
        )

        # hide/show based on mode
        if st.mode == "PID":
            self.arrow_i_mpc.hide(); self.arrow_t_mpc.hide(); self.needle_mpc.hide(); self.bar_iq_mpc.hide()
        elif st.mode == "MPC":
            self.arrow_i_pid.hide(); self.arrow_t_pid.hide(); self.needle_pid.hide(); self.bar_iq_pid.hide()
        else:
            self.arrow_i_pid.show(); self.arrow_t_pid.show(); self.needle_pid.show(); self.bar_iq_pid.show()
            self.arrow_i_mpc.show(); self.arrow_t_mpc.show(); self.needle_mpc.show(); self.bar_iq_mpc.show()

    def update_state(self, **kwargs):
        for k, v in kwargs.items():
            if hasattr(self.state, k):
                setattr(self.state, k, float(v) if k != "mode" else str(v))
        self._redraw()
