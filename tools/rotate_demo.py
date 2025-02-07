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
import numpy as np
from pupil_apriltags import Detector
import cv2
from wpimath.controller import PIDController


from cu_hal.drivetrain.drivetrain_wifi import DrivetrainWifi
from subsystems.drivetrain import Drivetrain, DrivetrainConfig

# Define the server address and port
HOST = "192.168.1.100"  # The server's hostname or IP address
PORT = 8080  # The port used by the server

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

sock.connect((HOST, PORT))

drivetrain = Drivetrain(DrivetrainConfig(), DrivetrainWifi(sock))

# cur_ref_x_vel = 0
# cur_ref_y_vel = 0
# cur_ref_omega = 0

target_ref_x_vel = 0
target_ref_y_vel = 0
target_ref_omega = 0

# slew_rate_xy = 0.5
# slew_rate_theta = 3.0


# def update():
#     drivetrain.drive_raw_local(target_ref_x_vel, target_ref_y_vel, target_ref_omega)


# drive_command = commands2.cmd.run(update, drivetrain).ignoringDisable(True)
# print(drive_command)
# drivetrain.setDefaultCommand(drive_command)


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

detector = Detector(families="tag36h11")
camera = cv2.VideoCapture(0)

FOCAL_LENGTh = 450
IMG_WIDTH = 640
class RotateCommand(commands2.Command):
    def __init__(self):
        self.requirements.add(drivetrain)
        self.pidcontroller = PIDController(0.066, 0, 0)
        self.pidcontroller.setSetpoint(0)


    def initialize(self):
        super().initialize()
        
    def execute(self):
        # read image from camera
        __, image = camera.read()

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray)
        # show image
        cv2.imshow('Camera Feed:', image)
        cv2.waitKey(1)

        # If no tag is detected, wait
        if (len(detections) == 0): 
            drivetrain.drive_raw_local(0, 0, 0)
            return

        for detection in detections:
            print("AprilTag detected!\n")
            print(f"Tag ID: {detection.tag_id}\n")
            print(f"Corners:\n{detection.corners}\n")

            match detection.tag_id:
                case 0 | 1 | 2 | 3 | 4:
                    print(f"Rendevous pad {detection.tag_id}!\n")
                case 5:
                    print("We are facing the North wall\n")
                case 6:
                    print("We are facing the South wall\n")
                case 7:
                    print("We are facing the East wall\n")
                case _:
                    print(f"Tag detected: {detection.tag_id}\n")
            
            tag_center_x = detection.center[0]
            img_center_x = IMG_WIDTH / 2
            x_offset = tag_center_x - img_center_x

            angle_rad = np.arctan(x_offset / FOCAL_LENGTh)
            angle_deg = np.degrees(angle_rad)

            drivetrain.drive_raw_local(0, 0, self.pidcontroller.calculate(angle_deg))
    def runsWhenDisabled(self):
        return True
    
drivetrain.setDefaultCommand(RotateCommand().ignoringDisable(True))

update_thread()