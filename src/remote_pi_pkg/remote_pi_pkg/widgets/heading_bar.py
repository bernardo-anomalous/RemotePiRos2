from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPen, QColor, QFont
from PyQt5.QtCore import Qt

class HeadingBarWidget(QWidget):
    """
    A horizontal HUD-style heading bar.
    Draws a horizontal line with tick marks for cardinal directions and a vertical red marker.
    Also displays the exact heading value (in degrees) near the marker.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.heading = 0.0  # degrees (0 = NORTH)
        self.setMinimumHeight(80)

    def update_heading(self, heading):
        self.heading = heading % 360
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = self.rect()
        width = rect.width()
        height = rect.height()
        center_x = width // 2
        center_y = height // 2

        painter.fillRect(rect, QColor("#141414"))
        pen = QPen(QColor("#00FF00"), 2)
        painter.setPen(pen)
        painter.drawLine(0, center_y, width, center_y)

        tick_interval = 15
        pixels_per_degree = width / 90.0  # ±45° around center

        for deg in range(-45, 46, tick_interval):
            tick_x = center_x + int(deg * pixels_per_degree)
            painter.drawLine(tick_x, center_y - 10, tick_x, center_y + 10)
            label_deg = (self.heading + deg) % 360
            cardinal = ""
            if abs(label_deg - 0) < 7.5 or abs(label_deg - 360) < 7.5:
                cardinal = "N"
            elif abs(label_deg - 90) < 7.5:
                cardinal = "E"
            elif abs(label_deg - 180) < 7.5:
                cardinal = "S"
            elif abs(label_deg - 270) < 7.5:
                cardinal = "W"
            label = cardinal if cardinal else f"{int(label_deg)}°"
            painter.drawText(tick_x - 15, center_y + 30, label)

        marker_pen = QPen(QColor("#FF4500"), 3)
        painter.setPen(marker_pen)
        painter.drawLine(center_x, center_y - 20, center_x, center_y + 20)

        painter.setPen(QPen(QColor("#FF4500")))
        painter.setFont(QFont("Courier New", 14, QFont.Bold))
        heading_text = f"{self.heading:.1f}°"
        painter.drawText(center_x - 30, center_y - 25, heading_text)
