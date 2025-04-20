from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QPushButton, QComboBox, QLabel, QSpacerItem, QSizePolicy, QHBoxLayout
)
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtCore import Qt
from game_launcher import launch_project
import os

class Dashboard(QWidget):
    def __init__(self):
        super().__init__()
        self.dropdown = None
        self.start_button = None
        self.setWindowTitle("STM32 FreeRTOS Project Dashboard")
        self.setFixedSize(520, 420)
        self.setStyleSheet("""
            QWidget {
                background-color: #1e1e2f;
                color: white;
            }
            QComboBox {
                background-color: #2e2e3e;
                border-radius: 10px;
                padding: 6px 12px;
                font-size: 14px;
            }
            QPushButton {
                background-color: #0078D7;
                border: none;
                border-radius: 10px;
                padding: 10px 20px;
                color: white;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #005fa3;
            }
        """)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        layout.setSpacing(25)
        layout.setContentsMargins(40, 30, 40, 30)

        # Logo
        logo_path = os.path.join("assets", "logo.png")
        if os.path.exists(logo_path):
            logo_label = QLabel()
            pixmap = QPixmap(logo_path).scaledToWidth(100, Qt.SmoothTransformation)
            logo_label.setPixmap(pixmap)
            logo_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(logo_label)

        # Title
        title = QLabel("STM32 Game Projects")
        title.setFont(QFont("Segoe UI", 20, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Subtitle / dropdown label
        label = QLabel("Choose a Project to Launch")
        label.setFont(QFont("Segoe UI", 13))
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)

        # Dropdown
        self.dropdown = QComboBox()
        self.dropdown.setFont(QFont("Segoe UI", 13))
        self.dropdown.addItems(["snake_game", "door_lock "])
        self.dropdown.setFixedHeight(40)
        layout.addWidget(self.dropdown)

        # Launch button
        self.start_button = QPushButton("🚀 Launch Project")
        self.start_button.setFont(QFont("Segoe UI", 14))
        self.start_button.setFixedHeight(45)
        self.start_button.clicked.connect(self.start_selected_project)
        layout.addWidget(self.start_button)

        # Spacer
        layout.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        self.setLayout(layout)

    def start_selected_project(self):
        project = self.dropdown.currentText()
        self.hide()
        launch_project(project, parent_dashboard=self)
