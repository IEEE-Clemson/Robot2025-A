import socket
from typing import Tuple
from cu_hal.interfaces import DrivetrainHAL


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

class DrivetrainWifi(DrivetrainHAL):
    """Implementation of the drivetrain HAL for physical hardware using wifi
    """
    def __init__(self, socket: socket.socket):
        super().__init__()
        self._socket = socket

    def get_local_velocity(self) -> Tuple[float, float, float]:
        return 0, 0, 0

    def set_target_wheel_velocities(self, vx: float, vy: float, omega: float) -> Tuple[float, float, float]:
        message = f"{vx:.2f},{vy:.2f},{omega:.2f}\n"
        self._socket.sendall(message.encode())
        line = readline(self._socket)
        return tuple(float(x) for x in line.split(','))
    
    
    