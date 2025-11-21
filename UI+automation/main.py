import os
import sys

from PySide6.QtWidgets import QApplication
from PySide6.QtQml import QQmlApplicationEngine

from backend import QBackend


# Base directory = folder where main.py lives
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# Your QML lives in the "UI" subfolder
QML_FILE = os.path.join(BASE_DIR, "UI", "Main.qml")


def main():
    app = QApplication(sys.argv)

    backend = QBackend()
    engine = QQmlApplicationEngine()

    # Expose backend to QML as "backend"
    engine.rootContext().setContextProperty("backend", backend)

    # Open serial link to Arduino
    backend.open()

    # Load the QML file
    print("Loading QML from:", QML_FILE)
    engine.load(QML_FILE)

    if not engine.rootObjects():
        print("❌ Failed to load QML!")
        for err in engine.errors():
            print("QML error:", err.toString())
        sys.exit(-1)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
