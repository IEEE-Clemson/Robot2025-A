from abc import abstractmethod
from typing import List, Protocol, Tuple

class DrivetrainHAL(Protocol):
    """Hardware abstraction layer for the meccanum drivetrain
    """

    @abstractmethod
    def get_local_velocity(self) -> Tuple[float, float, float]:
        """Gets estimated local velocity in meters per second and angular velocity in rads
        Positive X is front of robot and positive Y is left side of robot (see robot_geometry.md)

        Returns:
            Tuple[float]: Tuple of vx [m/s], vy [m/s], omega [rad/s]
        """
        ...
    
    @abstractmethod
    def set_target_wheel_velocities(self, vx: float, vy: float, omega: float) -> Tuple[float, float, float]:
        """Sets estimated local velocity in meters per second and angular velocity in rads
        Returns current velocity and angular velocity
        Positive X is front of robot and positive Y is left side of robot (see robot_geometry.md)

        Args:
            vx (float): Estimated velocity in x direction relative to robot [m/s]
            vy (float): Estimated velocity in y direction relative to robot [m/s]
            omega (float): Estimated angular velocity [rad/s]

        Returns:
            Tuple[float, float, float]:  Tuple of vx [m/s], vy [m/s], omega [rad/s]
        """
        ...


class AuxilliaryHAL(Protocol):
    """Hardware abstraction layer for the auxilliary functions of the robot
    """

    @abstractmethod
    def idle_box(self):
        ...

    @abstractmethod
    def grab_box(self):
        ...

    @abstractmethod
    def release_box(self):
        ...

    @abstractmethod
    def travel_beacon(self):
        ...

    @abstractmethod
    def extend_beacon(self):
        ...

    @abstractmethod
    def retract_beacon(self):
        ...

    @abstractmethod
    def extend_dropper(self):
        ...

    @abstractmethod
    def is_dropper_running(self) -> bool:
        ...

    @abstractmethod
    def set_intake_speed(self, speed: float):
        ...

    @abstractmethod
    def get_intake_speed(self) -> float:
        ...

    @abstractmethod
    def is_armed(self) -> bool:
        ...

    @abstractmethod
    def is_led_on(self) -> bool:
        ...
