""" GUI to drive the drivetrain through wifi
"""
import socket
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

def set_speeds(s: socket.socket, x, y, omega):
    message = f"{x:.2f},{y:.2f},{omega:.2f}\n"
    s.sendall(message.encode())
    line = readline(s)
    print(line)

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
        x = 0
        y = 0
        w = 0
        if self.keys['w']:
            x = 1
        if self.keys['s']:
            x = -1
        if self.keys['a']:
            y = 1
        if self.keys['d']:
            y = -1
        if self.keys['q']:
            w = 1
        if self.keys['e']:
            w = -1
        cx = 1.0
        cy = 1.0
        cw = 1.0
        set_speeds(sock, x * cx, y * cy, w * cw)
        
# Define the server address and port
HOST = '192.168.1.100'  # The server's hostname or IP address
PORT = 8080        # The port used by the server

# Create a socket object
sock = None
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    # Connect to the server
    s.connect((HOST, PORT))
    s.timeout
    sock = s
    app = QApplication([])
    window = Window()
    window.show()
    app.exec()
