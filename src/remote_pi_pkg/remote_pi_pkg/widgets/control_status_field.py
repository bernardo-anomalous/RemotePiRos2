from PyQt5.QtWidgets import QTextEdit

class ControlStatusField(QTextEdit):
    """
    A read-only text area that displays control data.
    Shows: PITCH COMMAND, ROLL COMMAND, CURRENT PITCH, CURRENT ROLL, and SERVO ANGLES.
    Each item is on its own line.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setStyleSheet("""
            QTextEdit {
                background-color: #222222;
                border: 2px solid #00FFFF;
                padding: 5px;
            }
        """)
        self.setMinimumHeight(150)
