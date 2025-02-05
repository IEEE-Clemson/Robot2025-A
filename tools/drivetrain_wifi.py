""" GUI to drive the drivetrain through wifi
"""
import socket
from threading import Thread
from time import sleep, time
from typing import Tuple
from PySide6.QtWidgets import *
from PySide6.QtCore import *
from PySide6.QtGui import *

def readline(sock):
    """Read a line from a socket until a newline is encountered."""
    data = b""
    while True:
        chunk = sock.recv(1)
        if not chunk:
            break  # Connection closed
        data += chunk
        if data.endswith(b"\n"):
            return data.decode("utf-8").rstrip("\n")

def set_speeds(s: socket.socket, x, y, omega) -> Tuple[float, float, float]:
    message = f"{x:.2f},{y:.2f},{omega:.2f}\n"
    s.sendall(message.encode())
    line = readline(s)
    return tuple(float(x) for x in line.split(','))

cur_ref_x_vel = 0
cur_ref_y_vel = 0
cur_ref_omega = 0

target_ref_x_vel = 0
target_ref_y_vel = 0
target_ref_omega = 0

cur_x_vel = 0
cur_y_vel = 0
cur_omega = 0

slew_rate_xy = 1.0
slew_rate_theta = 4.0

def update(dt):
    global cur_ref_x_vel, cur_ref_y_vel, cur_ref_omega
    global cur_x_vel, cur_y_vel, cur_omega
    if abs(target_ref_x_vel - cur_ref_x_vel) < slew_rate_xy * dt:
        cur_ref_x_vel = target_ref_x_vel
    else:
        cur_ref_x_vel += slew_rate_xy * dt if target_ref_x_vel > cur_ref_x_vel else -slew_rate_xy * dt

    if abs(target_ref_y_vel - cur_ref_y_vel) < slew_rate_xy * dt:
        cur_ref_y_vel = target_ref_y_vel
    else:
        cur_ref_y_vel += slew_rate_xy * dt if target_ref_y_vel > cur_ref_y_vel else -slew_rate_xy * dt

    if abs(target_ref_omega - cur_ref_omega) < slew_rate_theta * dt:
        cur_ref_omega = target_ref_omega
    else:
        cur_ref_omega += slew_rate_theta * dt if target_ref_omega > cur_ref_omega else -slew_rate_theta * dt

    t = time()
    cur_x_vel, cur_y_vel, cur_omega = set_speeds(sock, cur_ref_x_vel, cur_ref_y_vel, cur_ref_omega)
    print(f"Latency: {(time()-t)*1000:.1f}ms")
    print(cur_ref_x_vel, cur_y_vel, cur_ref_omega)

def update_thread():
    t = time()
    dt = 0.1
    while True:
        update(dt)
        sleep(0.1)
        t_new = time()
        dt = t_new - t
        t = t_new

class Window(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.layout = QHBoxLayout(self)
        self.label1 = QLabel("", self)
        self.label2 = QLabel("Key Pressed: ", self)
        self.layout.addWidget(self.label2)
        self.layout.addWidget(self.label1)
        self.resize(400,400)
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
            widget.label1.setText(text)
            self.update_speeds()

        elif event.type() == QEvent.KeyRelease:
            if event.isAutoRepeat():
                return False
            text = event.text()
            self.keys[text.lower()] = False
            widget.label1.setText(text + " Released")
            self.update_speeds()
        return False

    def update_speeds(self):
        global target_ref_x_vel, target_ref_y_vel, target_ref_omega
        cx = 0.5
        cy = 0.5
        cw = 3.0
        if self.keys['w']:
            target_ref_x_vel = cx * 1
        elif self.keys['s']:
            target_ref_x_vel = cx * -1
        else:
            target_ref_x_vel = 0

        if self.keys['a']:
            target_ref_y_vel = cy * 1
        elif self.keys['d']:
            target_ref_y_vel = cy * -1
        else:
            target_ref_y_vel = 0

        if self.keys['q']:
            target_ref_omega = cw * 1
        elif self.keys['e']:
            target_ref_omega = cw * -1
        else:
            target_ref_omega = 0
    
# Define the server address and port
HOST = '192.168.1.100'  # The server's hostname or IP address
PORT = 8080        # The port used by the server

# Create a socket object
sock = None
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    # Connect to the server
    s.connect((HOST, PORT))

    sock = s
    # Create timer for updating
    Thread(target = update_thread).start() 
    app = QApplication([])
    window = Window()
    window.show()
    app.exec()
