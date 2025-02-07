""" GUI to drive the drivetrain through wifi
"""

import socket
from threading import Thread
from time import sleep, time
from typing import Tuple
from PySide6.QtWidgets import *
from PySide6.QtCore import *
from PySide6.QtGui import *
import commands2
from wpilib import RobotState

from cu_hal.drivetrain.drivetrain_wifi import DrivetrainWifi
from subsystems.drivetrain import Drivetrain, DrivetrainConfig

# Define the server address and port
HOST = "192.168.1.100"  # The server's hostname or IP address
PORT = 8080  # The port used by the server

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

sock.connect((HOST, PORT))

drivetrain = Drivetrain(DrivetrainConfig(), DrivetrainWifi(sock))

cur_ref_x_vel = 0
cur_ref_y_vel = 0
cur_ref_omega = 0

target_ref_x_vel = 0
target_ref_y_vel = 0
target_ref_omega = 0

slew_rate_xy = 0.5
slew_rate_theta = 3.0

def update():
    drivetrain.drive_raw_local(target_ref_x_vel, target_ref_y_vel, target_ref_omega)


drive_command = commands2.cmd.run(update, drivetrain).ignoringDisable(True)
print(drive_command)
drivetrain.setDefaultCommand(drive_command)

def update_thread():
    t = time()
    dt = 0.05
    commands2.CommandScheduler.getInstance().enable()

    while True:
        target_t = 0.02 - dt
        if target_t > 0:
            sleep(target_t)
        commands2.CommandScheduler.getInstance().run()
        print(f"{dt * 1000:.1f}")
        t_new = time()
        dt = t_new - t
        t = t_new


class Window(QWidget):

    def update_monitors(self):
        cur_x_vel, cur_y_vel, cur_omega = drivetrain.get_local_vel()
        odom_x, odom_y, odom_theta = (
            drivetrain.pose_x[0],
            drivetrain.pose_x[1],
            drivetrain.pose_theta,
        )

        self.target_vel_values_label.setText(
            f"X: {target_ref_x_vel:.2F} Y: {target_ref_y_vel:.2F} W: {target_ref_omega:.2F}"
        )
        self.cur_vel_values_label.setText(
            f"X: {cur_x_vel:.2F} Y: {cur_y_vel:.2F} W: {cur_omega:.2F}"
        )
        self.odom_values_label.setText(
            f"X: {odom_x:.2F} Y: {odom_y:.2F} T: {odom_theta:.2F}"
        )

    def __init__(self, parent=None):
        super().__init__(parent=parent)
        grid = QGridLayout(self)
        self.layout = grid

        self.target_vel_label = QLabel("Target Velocities: ", self)
        self.target_vel_values_label = QLabel("", self)
        self.cur_vel_label = QLabel("Current Velocities: ", self)
        self.cur_vel_values_label = QLabel("", self)
        self.odom_label = QLabel("Estimated pose: ", self)
        self.odom_values_label = QLabel("", self)

        grid.addWidget(self.target_vel_label, 0, 0)
        grid.addWidget(self.target_vel_values_label, 0, 1)
        grid.addWidget(self.cur_vel_label, 1, 0)
        grid.addWidget(self.cur_vel_values_label, 1, 1)
        grid.addWidget(self.odom_label, 2, 0)
        grid.addWidget(self.odom_values_label, 2, 1)

        # Create timer to update widgets
        self.update_timer = QTimer(self)
        self.connect(self.update_timer, SIGNAL("timeout()"), self.update_monitors)
        self.update_timer.start(50)

        self.resize(400, 400)
        self.eventFilter = KeyHandler(parent=self)
        self.installEventFilter(self.eventFilter)


class KeyHandler(QObject):
    keys = {chr(char): False for char in range(127)}

    def eventFilter(self, widget, event):
        if event.type() == QEvent.KeyPress:
            if event.isAutoRepeat():
                return False
            text = event.text()
            self.keys[text.lower()] = True
            self.update_speeds()

        elif event.type() == QEvent.KeyRelease:
            if event.isAutoRepeat():
                return False
            text = event.text()
            self.keys[text.lower()] = False
            self.update_speeds()
        return False

    def update_speeds(self):
        global target_ref_x_vel, target_ref_y_vel, target_ref_omega
        cx = 0.5
        cy = 0.5
        cw = 3.0
        if self.keys["w"]:
            target_ref_x_vel = cx * 1
        elif self.keys["s"]:
            target_ref_x_vel = cx * -1
        else:
            target_ref_x_vel = 0

        if self.keys["a"]:
            target_ref_y_vel = cy * 1
        elif self.keys["d"]:
            target_ref_y_vel = cy * -1
        else:
            target_ref_y_vel = 0

        if self.keys["q"]:
            target_ref_omega = cw * 1
        elif self.keys["e"]:
            target_ref_omega = cw * -1
        else:
            target_ref_omega = 0


# Create timer for updating
Thread(target=update_thread, daemon=True).start()
app = QApplication([])
window = Window()
window.show()
app.exec()
