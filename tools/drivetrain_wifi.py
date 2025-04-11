""" GUI to drive the drivetrain through wifi
"""
from cu_hal.auxilliary.auxilliary_hw import AuxilliaryHW
from cu_hal.drivetrain.drivetrain_i2c import DrivetrainI2C
from smbus2 import SMBus
from math import pi
import socket
from threading import Thread
from time import sleep, time
from PySide6.QtWidgets import QWidget, QGridLayout, QLabel, QApplication
from PySide6.QtCore import QTimer, SIGNAL, QObject, QEvent
import commands2

from control.drivetrain.move_to_pose import move_to_inches
from cu_hal.drivetrain.drivetrain_wifi import DrivetrainWifi
from cu_hal.drivetrain.drivetrain_sim import DrivetrainSim
from subsystems.beacon import Beacon
from subsystems.drivetrain import Drivetrain, DrivetrainConfig
from subsystems.dumper import Dumper
from subsystems.intake import Intake
from subsystems.mover import Mover
from subsystems.safety import Safety
from subsystems.vision import Vision, VisionConfig

from control.beacon import extend_beacon, retract_beacon, travel_beacon
from control.dumper import extend_dumper
from control.intake import start_intake, stop_intake
from control.mover import grab_box, mover_off, release_box
from control.safety import wait_for_armed, wait_for_led_on


bus = SMBus(1)
drivetrain_hal = DrivetrainI2C(bus)
drivetrain = Drivetrain(DrivetrainConfig(), drivetrain_hal)
drivetrain.drive_raw_local(0, 0, 0)
drivetrain.periodic()
drivetrain.reset_odom_inches(32.25, 8, 0)

aux_hal = AuxilliaryHW(bus)
beacon = Beacon(aux_hal)
intake = Intake(aux_hal)
dumper = Dumper(aux_hal)
mover = Mover(aux_hal)
safety = Safety(aux_hal)


vision_config = VisionConfig()
vision_config.should_display = False
vision_config.dev_index = -1
#vision_config.should_display = True
vision = Vision(vision_config)
vision.add_pose2d_callback = drivetrain.pose_estimator.add_vision_pose

cur_ref_x_vel = 0
cur_ref_y_vel = 0
cur_ref_omega = 0

target_ref_x_vel = 0
target_ref_y_vel = 0
target_ref_omega = 0

slew_rate_xy = 0.5
slew_rate_theta = 3.0

led_on = False
armed = False

def update():
    drivetrain.drive_raw_local(target_ref_x_vel, target_ref_y_vel, target_ref_omega)


drive_command = commands2.cmd.run(update, drivetrain).ignoringDisable(True)
drivetrain.setDefaultCommand(drive_command)



def update_thread():
    global led_on, armed
    t = time()
    dt = 0.02
    commands2.CommandScheduler.getInstance().enable()

    while True:
        t = time()
        commands2.CommandScheduler.getInstance().run()
        t_new = time()
        #print(t_new - t)
        to_sleep = dt - (t_new - t)
        if to_sleep > 0:
            sleep(to_sleep)
        led_on = safety.is_light_on()
        armed = safety.is_armed()


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
        self.led_values_label.setText(led_on)
        self.armed_values_label.setText(armed)

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
        self.led_label = QLabel("LED on: ", self)
        self.led_values_label = QLabel("", self)
        self.armed_label = QLabel("Armed: ", self)
        self.armed_values_label = QLabel("", self)

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
            if text.lower() == "i":
                retract_beacon(beacon).schedule()
            if text.lower() == "o":
                travel_beacon(beacon).schedule()
            if text.lower() == "p":
                extend_beacon(beacon).schedule()

            if text.lower() == "f":
                extend_dumper(dumper).schedule()

            if text.lower() == "1":
                start_intake(intake).schedule()

            if text.upper() == "2":
                stop_intake(intake).schedule()

            if text.lower() == "b":
                mover_off(mover).schedule()
            if text.lower() == "n":
                grab_box(mover).schedule()
            if text.lower() == "m":
                release_box(mover).schedule()            

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
