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
    QGroupBox,
    QLineEdit,
    QSlider
)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        main_layout = QHBoxLayout()
        main_box = QGroupBox()

        usv_layout = QHBoxLayout()
        usv_box = QGroupBox("USV")
        usv_box.setAlignment(4)
        usv_layout.addWidget(self.create_usv_heading())
        usv_layout.addWidget(self.create_usv_position())

        usv_box.setLayout(usv_layout)

        rov_layout = QHBoxLayout()
        rov_box = QGroupBox("ROV")
        rov_box.setAlignment(4)
        rov_layout.addWidget(self.create_rov_depth())
        rov_layout.addWidget(self.create_rov_heading())

        rov_box.setLayout(rov_layout)


        main_layout.addWidget(usv_box)
        main_layout.addLayout(self.create_central_layout())
        main_layout.addWidget(rov_box)
        main_box.setLayout(main_layout)

        main_window = QGroupBox()
        window_layout = QVBoxLayout()
        window_layout.addLayout(self.create_alarm_layout())
        window_layout.addWidget(main_box)
        main_window.setLayout(window_layout)

        self.setCentralWidget(main_window)

    def create_usv_heading(self):
        layout = QVBoxLayout()
        group_box = QGroupBox("Heading")
        group_box.setAlignment(4)

        hold_button = QPushButton("Hold")
        hold_button.setCheckable(True)
        layout.addWidget(hold_button)

        set_layout = QHBoxLayout()
        set_box = QLineEdit()
        set_box.setPlaceholderText("Enter desired Heading")
        set_layout.addWidget(set_box)
        set_button = QPushButton("Set")
        set_layout.addWidget(set_button)

        layout.addLayout(set_layout)
        group_box.setLayout(layout)

        return group_box
        #TODO: Add dial for readout


    def create_usv_position(self):
        layout = QVBoxLayout()
        group_box = QGroupBox("Position")
        group_box.setAlignment(4)

        hold_button = QPushButton("Hold")
        hold_button.setCheckable(True)
        layout.addWidget(hold_button)

        group_box.setLayout(layout)
        return group_box

        #TODO: Add arrows for movement

    def create_central_layout(self):
        layout = QVBoxLayout()

        joystick_button = QPushButton("Joystick Control")
        joystick_button.setCheckable(True)
        layout.addWidget(joystick_button)

        waypoint_button = QPushButton("Waypoint mode")
        waypoint_button.setCheckable(True)
        layout.addWidget(waypoint_button)

        reverse_mode_button = QPushButton("Reverse Master/Slave")
        layout.addWidget(reverse_mode_button)

        return layout

    def create_rov_depth(self):
        layout = QVBoxLayout()
        group_box = QGroupBox("Depth")
        group_box.setAlignment(4)
        hold_button = QPushButton("Hold")
        hold_button.setCheckable(True)
        layout.addWidget(hold_button)

        retract_button = QPushButton("Retract")
        layout.addWidget(retract_button)

        depth_slider = QSlider()
        depth_slider.setRange(0,100)
        depth_slider.setTickPosition(QSlider.TickPosition.TicksBothSides)
        depth_slider.setInvertedAppearance(True)
        layout.addWidget(depth_slider)

        group_box.setLayout(layout)
        return group_box

    def create_rov_heading(self):
        layout = QVBoxLayout()
        group_box = QGroupBox("Heading")
        group_box.setAlignment(4)
        hold_button = QPushButton("Hold")
        hold_button.setCheckable(True)
        layout.addWidget(hold_button)
        #TODO: Add arrow array here

        group_box.setLayout(layout)
        return group_box

    def create_alarm_layout(self):
        layout = QHBoxLayout()

        abort_button = QPushButton("ABORT")
        layout.addWidget(abort_button)

        e_stop_button = QPushButton("E-Stop")
        layout.addWidget(e_stop_button)

        return layout

app = QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec()
