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
        self.setMinimumHeight(300)

        self.state = SchematicState()

        self.W, self.H = 980, 310
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

        s.addRect(QRectF(40, 110, 170, 95), pen_box, brush_box)
        t = s.addText("INVERTER\n(vd, vq)")
        t.setDefaultTextColor(self.col_fg)
        t.setPos(75, 132)

        # motor: stator + rotor
        s.addEllipse(QRectF(320, 65, 180, 180), QPen(self.col_fg, 2), QBrush(Qt.black))
        s.addEllipse(QRectF(360, 105, 100, 100), QPen(self.col_wire, 2), QBrush(Qt.black))
        mt = s.addText("IM\nMotor")
        mt.setDefaultTextColor(self.col_fg)
        mt.setPos(395, 135)

        # shaft + load
        s.addLine(500, 155, 705, 155, QPen(self.col_wire, 4))
        s.addRect(QRectF(705, 110, 210, 95), pen_box, brush_box)
        lt = s.addText("LOAD\n(TL)")
        lt.setDefaultTextColor(self.col_fg)
        lt.setPos(785, 132)

        # arrows
        self.arrow_i_pid = s.addPolygon(QPolygonF(), QPen(self.col_pid, 1), QBrush(self.col_pid))
        self.arrow_i_mpc = s.addPolygon(QPolygonF(), QPen(self.col_mpc, 1), QBrush(self.col_mpc))
        self.arrow_t_pid = s.addPolygon(QPolygonF(), QPen(self.col_pid, 1), QBrush(self.col_pid))
        self.arrow_t_mpc = s.addPolygon(QPolygonF(), QPen(self.col_mpc, 1), QBrush(self.col_mpc))

        # labels for arrows
        self.lbl_i = s.addText("i_q*")
        self.lbl_i.setDefaultTextColor(self.col_fg)
        self.lbl_i.setPos(235, 85)
        self.lbl_t = s.addText("T_e")
        self.lbl_t.setDefaultTextColor(self.col_fg)
        self.lbl_t.setPos(560, 200)

        # gauge
        s.addEllipse(QRectF(300, 10, 220, 220), QPen(self.col_wire, 2))
        self.needle_ref = s.addLine(410, 120, 410, 30, QPen(self.col_ref, 2, Qt.DashLine))
        self.needle_pid = s.addLine(410, 120, 410, 45, QPen(self.col_pid, 3))
        self.needle_mpc = s.addLine(410, 120, 410, 55, QPen(self.col_mpc, 3))

        gl = s.addText("Speed (rpm)")
        gl.setDefaultTextColor(self.col_fg)
        gl.setPos(382, 235)

        # bars panel
        s.addRect(QRectF(40, 15, 250, 80), QPen(self.col_wire, 1), QBrush(Qt.black))
        self.txt = s.addText("")
        self.txt.setDefaultTextColor(self.col_fg)
        self.txt.setFont(QFont("Consolas", 10))
        self.txt.setPos(55, 22)

        # iq bars
        s.addRect(QRectF(540, 20, 375, 70), QPen(self.col_wire, 1), QBrush(Qt.black))
        label = s.addText("Commands")
        label.setDefaultTextColor(self.col_fg)
        label.setPos(550, 25)

        self.bar_iq_pid = s.addRect(QRectF(550, 50, 0, 12), QPen(Qt.NoPen), QBrush(self.col_pid))
        self.bar_iq_mpc = s.addRect(QRectF(550, 70, 0, 12), QPen(Qt.NoPen), QBrush(self.col_mpc))
        self.txt_iq = s.addText("")
        self.txt_iq.setDefaultTextColor(self.col_fg)
        self.txt_iq.setFont(QFont("Consolas", 9))
        self.txt_iq.setPos(550, 88)

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
        cx, cy = 410.0, 120.0
        r = 90.0
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

        # current arrows (offset a bit so PID/MPC both visible)
        self.arrow_i_pid.setPolygon(self._arrow_poly(210, 150, 320, 150, width=w_i_pid))
        self.arrow_i_mpc.setPolygon(self._arrow_poly(210, 170, 320, 170, width=w_i_mpc))
        # torque arrows
        self.arrow_t_pid.setPolygon(self._arrow_poly(500, 175, 705, 175, width=w_t_pid))
        self.arrow_t_mpc.setPolygon(self._arrow_poly(500, 195, 705, 195, width=w_t_mpc))

        # needles
        self._needle(self.needle_ref, st.rpm_ref)
        self._needle(self.needle_pid, st.rpm_pid)
        self._needle(self.needle_mpc, st.rpm_mpc)

        # bars: map |iq_ref| into width (0..350)
        W = 350.0
        wpid = W * norm(st.iq_ref_pid, iq_scale)
        wmpc = W * norm(st.iq_ref_mpc, iq_scale)
        self.bar_iq_pid.setRect(QRectF(550, 50, wpid, 12))
        self.bar_iq_mpc.setRect(QRectF(550, 70, wmpc, 12))

        # text block
        self.txt.setPlainText(
            f"Mode: {st.mode}\n"
            f"rpm_ref: {st.rpm_ref:7.1f}\n"
            f"PID rpm: {st.rpm_pid:7.1f}   MPC rpm: {st.rpm_mpc:7.1f}\n"
            f"TL: {st.tl:6.2f} N·m   |iq|lim: {st.iq_limit:5.1f} A"
        )
        self.txt_iq.setPlainText(
            f"PID iq*: {st.iq_ref_pid:6.2f} A   MPC iq*: {st.iq_ref_mpc:6.2f} A\n"
            f"PID Te:  {st.te_pid:6.2f} N·m   MPC Te:  {st.te_mpc:6.2f} N·m"
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
