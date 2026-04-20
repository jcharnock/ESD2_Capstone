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
        self.counter = 0
        self.resize(1280, 720)

        # Build UI 
        self.setup_ui()

        # pull entries & load site
        self.refresh_data()
        #DEBUG: load fake SQL list
        #self.debug_refresh_data()

        # 10 second refresh
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_data)
        self.timer.start(10000)

    def setup_ui(self):
        main = QVBoxLayout(self)

        # Tennis Tracker label
        title = QLabel('Tennis Tracker')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('font-size: 80px;')
        main.addWidget(title)

        # top label
        top = QHBoxLayout()

        self.info = QLabel('Timestamp: --\nX Coordinate: --\nY Coordinate: --')
        self.info.setStyleSheet('font-size: 30px;')
        top.addWidget(self.info)

        # blank court image info
        self.image = QLabel('Court Image Here')
        self.image.setFixedSize(480, 240)
        self.image.setAlignment(Qt.AlignCenter)
        self.image.setStyleSheet('background:#a8cc2d;')
        top.addWidget(self.image)

        main.addLayout(top)

        # Ruling
        self.ruling = QLabel('RULING: --')
        self.ruling.setAlignment(Qt.AlignCenter)
        self.ruling.setStyleSheet(
              'font-size: 60px;'
              'font-weight: bold;'
        )
        main.addWidget(self.ruling)

        # Scrollable entry list
        self.entries = QListWidget()
        self.entries.setStyleSheet(
                'font-size: 24px;'
                'padding: 8px;' 
            )
        main.addWidget(self.entries)
        self.entries.itemClicked.connect(self.load_selected_entry)

    # Load the clicked entry onto main GUI
    def load_selected_entry(self, item):
            row = item.data(Qt.UserRole)
            if row:
                self.entry_clicked(row)


    #DEBUG: refresh data w/ junk
    ''' def debug_refresh_data(self):
        self.counter += 1

        text = (
            f"Entry #{self.counter}\n"
            f"Timestamp: {self.counter}"
            )
        item = QListWidgetItem(text)
        self.entries.insertItem(0, item) '''

    # check for SQL data
    def refresh_data(self):
        try:
            response = requests.get(API_URL, timeout=5)
            data = response.json()

            if not data:
                return
            
            # store data into entries
            self.entries = data

            for row in data:
                # DEBUG: add data to rows
                text = (
                    f"Entry #{row['entry_number']}\n"
                    f"Timestamp: {row['timestamp']}"
                )
                item = QListWidgetItem(text)
                # Store full row in item
                item.setData(Qt.UserRole, row)
                # insert item at top of list
                self.entries.insertItem(0, item)

        except Exception as e:
            print("Refresh Error:", e)

    # run when list entry is clicked to update display
    def entry_clicked(self, row):
            # Update left info panel
            self.info.setText(
                f"Timestamp: {row['timestamp']}\n"
                f"X Coordinate: {row['x_coordinate']}\n"
                f"Y Coordinate: {row['y_coordinate']}"
            )
        # Update Ruling
            self.ruling.setText(
                f"RULING: {row['ruling']}"
            )
        # Update court image
            image_path = row.get("court_image")
            if image_path:
                pixmap = QPixmap(image_path)
                if not pixmap.isNull():
                    self.image_label.setPixmap(
                        pixmap.scaled( 
                            480, # length
                            240, # width
                            Qt.KeepAspectRatio,
                            Qt.SmoothTransformation
                        )
                    )
                    return
            # if image is missing set instead
            self.image_label.setText("No Image")
            self.image_label.setPixmap(QPixmap())

# DEBUG: run application in window
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TennisTracker()
    window.show()
    sys.exit(app.exec())

#sys.exit(app.exec())