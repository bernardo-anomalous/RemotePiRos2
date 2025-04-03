from PyQt5.QtWidgets import QTextEdit
from PyQt5.QtCore import Qt

class ControlStatusField(QTextEdit):
    """
    A read-only text area that displays control data.
    Shows: PITCH COMMAND, ROLL COMMAND, CURRENT PITCH, CURRENT ROLL, and SERVO ANGLES.
    Each item is on its own line.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setAttribute(Qt.WA_StyledBackground, True)


        self.setMinimumHeight(150)
