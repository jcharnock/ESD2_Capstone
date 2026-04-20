import sys
import requests
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QListWidget,
    QListWidgetItem, QVBoxLayout, QHBoxLayout
)
from PySide6.QtGui import QPixmap
from PySide6.QtCore import Qt, QTimer

# Django API URL
API_URL = 'http://127.0.0.1:8000/api/entries/'

class TennisTracker(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Tennis Tracker')
        self.resize(1280, 720)

        # Build UI 
        self.setup_ui()

        # load site
        self.refresh_data()

        # 10 second refresh
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_data)
        self.timer.start(10000)

    def setup_ui(self):
        main = QVBoxLayout(self)

        # Tennis Tracker label
        title = QLabel('Tennis Tracker')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('font-size: 26px;')
        main.addWidget(title)

        # Top section
        top = QHBoxLayout()

        self.info = QLabel('Timestamp: --\nX Coordinate: --\nY Coordinate: --')
        top.addWidget(self.info)

        # blank court image info
        self.image = QLabel('Court Image Here')
        self.image.setFixedSize(210, 80)
        self.image.setAlignment(Qt.AlignCenter)
        self.image.setStyleSheet('background:#a8cc2d;')
        top.addWidget(self.image)

        main.addLayout(top)

        # Ruling
        self.ruling = QLabel('RULING: --')
        self.ruling.setAlignment(Qt.AlignCenter)
        self.ruling.setStyleSheet('font-size: 28px;')
        main.addWidget(self.ruling)

        # Scrollable entry list
        self.entries = QListWidget()
        main.addWidget(self.entries)

'''    def refresh_data(self):
        try:
            response = requests.get(API_URL, timeout=5)
            data = response.json()

            if not data:
                return

            latest = data[0] '''

'''    def entry_clicked(self):
        # check to see if entry was clicked

        # Update left info panel
            self.info.setText(
                f"Timestamp: {latest['timestamp']}\n"
                f"X Coordinate: {latest['x_coordinate']}\n"
                f"Y Coordinate: {latest['y_coordinate']}"
            )
        # Update Ruling
            self.ruling.setText(
                f"RULING: {latest['ruling']}"
            )
        # Update court image
        new_court = QPixmap("path/to/image.png") # FIX LATER
        scaled_court = new_court.scaled(210, 80, aspectMode=Qt.KeepAspectRatio)
        self.image.setPixmap(scaled_court) '''

sys.exit(app.exec())