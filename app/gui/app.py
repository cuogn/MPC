from __future__ import annotations

"""GUI entrypoint.

Fixes common Qt crash:
  QWidget: Must construct a QApplication before a QWidget

Rule: create QApplication FIRST, then import/instantiate any QWidget.
"""

import sys


def run_gui() -> None:
    # IMPORTANT: QApplication must be created before importing any QWidget subclasses.
    from PySide6.QtWidgets import QApplication
    import pyqtgraph as pg

    app = QApplication(sys.argv)

    # Light theme: white background, black foreground (match thesis requirement)
    pg.setConfigOption("background", "w")
    pg.setConfigOption("foreground", "k")
    pg.setConfigOptions(antialias=True)

    from app.gui.main_window import MainWindow

    win = MainWindow()
    win.show()
    sys.exit(app.exec())
