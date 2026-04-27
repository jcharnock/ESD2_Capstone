import sys
import os
from datetime import datetime
import requests
import random
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QListWidget,
    QListWidgetItem, QVBoxLayout, QHBoxLayout
)
from PySide6.QtGui import QPixmap
from PySide6.QtCore import Qt, QTimer, QByteArray

class TennisTracker(QWidget):
    def __init__(self):
        super().__init__()
        self.counter = 0
        self.resize(1280, 720)

        # Build UI 
        self.setup_ui()

        # keep list of seen entries from SQL
        self.seen_entries = set()

        # pull entries & load site
        self.refresh_data()
        #DEBUG: load fake SQL list
        #self.debug_refresh_data()

        # 10 second refresh
        self.timer = QTimer()
        # DEBUG Refresh Data
        #self.timer.timeout.connect(self.debug_refresh_data)
        self.timer.timeout.connect(self.refresh_data)
        self.timer.start(5000)
        # check for clicked entry
        self.entries.itemClicked.connect(self.handle_item_click)

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
                'font-size: 28px;'
                'padding: 12px;' 
            )
        main.addWidget(self.entries)
        self.entries.itemClicked.connect(self.load_selected_entry)

    # click handler
    def handle_item_click(self, item):
        row = item.data(Qt.UserRole)
        self.entry_clicked(row)

    # Load the clicked entry onto main GUI
    def load_selected_entry(self, item):
        row = item.data(Qt.UserRole)
        if row:
            self.entry_clicked(row)

    # load the image from the SQL blob data to be displayed
    def convert_court_image(sql_blob):
        if sql_blob is None:
            return None
        
        image_bytes = QByteArray(sql_blob)
        good_image = QPixmap()
        good_image.loadFromData(image_bytes)
        return good_image


    #DEBUG: refresh data w/ junk
    def debug_refresh_data(self):
        self.counter += 1

        text = (
            f"Entry #{self.counter}             "
            f"Timestamp: {datetime.now().strftime("%H:%M:%S")}"
            )
        item = QListWidgetItem(text)
        
        # create dummy data
        row = {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "x_coordinate": round(random.random(), 3),
            "y_coordinate": round(random.random(), 3),
            "ruling": "IN",
            "court_image": "../images/tennis_court" + str(self.counter) + ".png"
        }

        item.setData(Qt.UserRole, row)
        self.entries.insertItem(0, item)

    # check for SQL data
    def refresh_data(self):
        try:
            response = requests.get("http://127.0.0.1:5000/get_new_entries", timeout=5)
            data = response.json()
            print(data)

            if not data:
                return

            for row in data:
                entry_no = int(float(row.get("entry_number", "N/A")))
                timestamp = row.get("timestamp", "N/A")
                unique_id = entry_no
                # if unique_id doesn't match seen entries add to list (fixes duplicates)
                if unique_id in self.seen_entries:
                    continue

                text = (
                    f"Entry #{entry_no}             Timestamp: {timestamp}"
                )            
                item = QListWidgetItem(text)
                # Storen row
                item.setData(Qt.UserRole, row)
                # insert item at top of list
                self.entries.insertItem(0, item)
                self.seen_entries.add(unique_id)

        except Exception as e:
            print("Refresh Error:", e)

    # run when list entry is clicked to update display
    def entry_clicked(self, row):
        # Update left info panel
        self.info.setText(
            f"Timestamp: {row['timestamp']}\n"
            f"X Coordinate: {float(row['x_coordinate']):.2f}\n"
            f"Y Coordinate: {float(row['y_coordinate']):.2f}"
         )
        # Update Ruling
        self.ruling.setText(
            f"RULING: {row['ruling']}"
        )
        # Update court image
        web_image = QLabel()
        web_image.convert_court_image(row['court_image'])
        if web_image:
            pixmap = QPixmap(web_image)
            if not pixmap.isNull():
                self.image.setPixmap(
                    pixmap.scaled( 
                        480, # length
                        240, # width
                        Qt.KeepAspectRatio,
                         Qt.SmoothTransformation
                    )
                 )
                return
        # if image is missing set instead
        self.image.setText("No Image")
        self.image.setPixmap(web_image)

# DEBUG: run application in window
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TennisTracker()
    window.show()
    sys.exit(app.exec())
