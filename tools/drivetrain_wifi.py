""" GUI to drive the drivetrain through wifi
"""

from math import pi
import socket
from threading import Thread
from time import sleep, time
from PySide6.QtWidgets import QWidget, QGridLayout, QLabel, QApplication
from PySide6.QtCore import QTimer, SIGNAL, QObject, QEvent
import commands2

from control.drivetrain.move_to_pose import TrapezoidalMove
from cu_hal.drivetrain.drivetrain_wifi import DrivetrainWifi
from cu_hal.drivetrain.drivetrain_sim import DrivetrainSim
from subsystems.drivetrain import Drivetrain, DrivetrainConfig
from subsystems.vision import Vision, VisionConfig

# Define the server address and port
SHOULD_USE_VISION = True
HOST = "192.168.1.100"  # The server's hostname or IP address
PORT = 8080  # The port used by the server

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

sock.connect((HOST, PORT))

drivetrain = Drivetrain(DrivetrainConfig(), DrivetrainWifi(sock))
if SHOULD_USE_VISION:
    vision_config = VisionConfig()
    vision_config.should_display = True
    vision_config.dev_index = 4
    vision = Vision(vision_config)
    vision.add_pose2d_callback = drivetrain.pose_estimator.add_vision_pose
#drivetrain = Drivetrain(DrivetrainConfig(), DrivetrainSim())

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
drivetrain.setDefaultCommand(drive_command)

move_pose_command = TrapezoidalMove(drivetrain, 0.794, 0.58, -pi / 2) \
                .andThen(TrapezoidalMove(drivetrain, 1.9, 0.58, 0)) \
                .andThen(TrapezoidalMove(drivetrain, 0.7, 0.58, 0)) \
                .andThen(TrapezoidalMove(drivetrain, 0.7, 1.2,  0)) \
                .andThen(commands2.InstantCommand(lambda: drivetrain.reset_odom(0.7, 1.0, 0), drivetrain).ignoringDisable(True)) \
                .andThen(TrapezoidalMove(drivetrain, 0.3, 1.0, 0)) \


def update_thread():
    t = time()
    dt = 0.05
    commands2.CommandScheduler.getInstance().enable()

    while True:
        target_t = 0.02 - dt
        if target_t > 0:
            sleep(target_t)
        commands2.CommandScheduler.getInstance().run()
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
        if self.keys["g"] and not move_pose_command.isScheduled():
            move_pose_command.schedule()


# Create timer for updating
Thread(target=update_thread, daemon=True).start()
app = QApplication([])
window = Window()
window.show()
app.exec()
