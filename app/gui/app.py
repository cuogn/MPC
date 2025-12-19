from __future__ import annotations
import sys
from PySide6.QtWidgets import QApplication
from app.gui.main_window import MainWindow

def run_gui():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
