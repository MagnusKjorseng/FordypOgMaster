import sys

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import(
    QApplication,
    QPushButton,
    QMainWindow,
    QHBoxLayout,
    QVBoxLayout,
    QWidget,
    QLabel,
    QButtonGroup,
    QGroupBox
)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        vlayout = QVBoxLayout()
        #label = QLabel("Mode select")
        #label.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        #vlayout.addWidget(label)
        buttons = [
            "Waypoint",
            "Hold Position",
            "Joystick"]

        buttonsBox = QGroupBox()
        buttonLayout = QHBoxLayout()
        buttonGroup = QButtonGroup()
        buttonGroup.setExclusive(True)
        # buttonsBox.setLayout(buttonLayout)

        for button in buttons:
            btn = QPushButton(button)

            buttonGroup.addButton(btn)
            # btn.setCheckable(True)
            buttonLayout.addWidget(btn)

        print(buttonGroup.buttons())
        vlayout.addLayout(buttonLayout)
        widget = QWidget()
        widget.setLayout(vlayout)

        self.setCentralWidget(widget)

app = QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec()
