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
        self.setMinimumHeight(90)
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background: transparent;")
        self.heading = 0.0
        self.target_heading = 0.0

    def update_heading_target(self, heading):
        """Set the target heading, called by the GUI every frame."""
        self.target_heading = heading % 360
        
    def smooth_update_value(self, current, target, factor=0.2):
        # Shortest path around the circle (handles wrap-around at 360°)
        delta = (target - current + 540) % 360 - 180
        return (current + factor * delta) % 360


        
    def heading_to_cardinal(self, degrees):
        directions = [
            "N", "NNE", "NE", "ENE",
            "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW",
            "W", "WNW", "NW", "NNW"
        ]
        ix = int((degrees + 11.25) % 360 / 22.5)
        return directions[ix]


    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = self.rect()
        width = rect.width()
        height = rect.height()
        center_x = width // 2
        center_y = height // 2
        self.heading = self.smooth_update_value(self.heading, self.target_heading)

        painter.setPen(QPen(QColor("#00FF00"), 2))
        painter.drawLine(0, center_y, width, center_y)

        # === CONFIGURATION ===
        visual_range_deg = 90            # Total degrees shown across full widget width
        pixels_per_degree = width / visual_range_deg
        tick_interval = 15               # degrees between ticks
        total_ticks = int(visual_range_deg / tick_interval) + 2  # +2 for margin

        # === Compute base heading snapped to nearest tick ===
        center_heading = self.heading % 360
        base_heading = int(center_heading // tick_interval) * tick_interval
        heading_offset_deg = center_heading - base_heading
        offset_px = heading_offset_deg * pixels_per_degree

        # === Draw moving ticks ===
        for i in range(-total_ticks, total_ticks + 1):
            tick_heading = (base_heading + i * tick_interval) % 360
            tick_x = int(center_x + i * tick_interval * pixels_per_degree - offset_px)

            if tick_x < -40 or tick_x > width + 40:
                continue

            # Draw tick
            painter.drawLine(tick_x, center_y - 10, tick_x, center_y + 10)

            # Label
            label = ""
            if abs(tick_heading - 0) < 7.5 or abs(tick_heading - 360) < 7.5:
                label = "N"
            elif abs(tick_heading - 90) < 7.5:
                label = "E"
            elif abs(tick_heading - 180) < 7.5:
                label = "S"
            elif abs(tick_heading - 270) < 7.5:
                label = "W"
            else:
                label = f"{int(tick_heading)}°"

            painter.setFont(QFont("Courier New", 10))
            painter.drawText(tick_x - 15, center_y + 20, label)

        # === Red center marker ===
        painter.setPen(QPen(QColor("#FF4500"), 3))
        painter.drawLine(center_x, center_y - 20, center_x, center_y + 20)

        # === Center heading readout ===
        heading_text = f"{self.heading:.1f}°"
        cardinal_text = self.heading_to_cardinal(self.heading)

        painter.setPen(QPen(QColor("#FF4500")))
        painter.setFont(QFont("Courier New", 14, QFont.Bold))
        painter.drawText(center_x + 10, center_y - 20, heading_text)

        painter.setFont(QFont("Courier New", 12))
        painter.drawText(center_x - 50, center_y - 20, cardinal_text)
