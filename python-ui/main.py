# main.py
from PyQt5.QtWidgets import QApplication
from dashboard_ui import Dashboard

def main():
    app = QApplication([])
    window = Dashboard()
    window.show()
    app.exec_()

if __name__ == "__main__":
    main()
