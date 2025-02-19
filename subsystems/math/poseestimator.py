"""Pose estimator heavily based WPILib implementation
https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java

"""

from math import pi, sqrt
from typing import List, Tuple
import numpy as np
from .pose import pose_add, pose_exp, pose_log, pose_sub
from wpimath.geometry import Pose2d


def clamp(x, x_min, x_max):
    return max(x_min, min(x, x_max))


def floorkey_buf(buf: List, t: float) -> int | None:
    """Gets the index of the element with the timestamp just before the given time

    Args:
        buf (List[Tuple[float, np.ndarray, float]]): Buffer to sample from
        t (float): Timestamp to find

    Returns:
        int|None: index of element
    """
    # Buf is sorted by time
    # Search for first timestamp in buffer that is lower than current timestamp
    # TODO: replace with binary search
    lower_i = None
    for i, entry in reversed(list(enumerate(buf))):
        if entry[0] < t:
            lower_i = i
            break

    return lower_i


def sample_timestamp_buf(
    buf: List[Tuple[float, np.ndarray, float]], t: float
) -> Tuple[np.ndarray, float] | None:
    """Samples a buffer that is labeled with timestamps
    Note: Cannot sample before the first element in the buffer

    Args:
        buf (List[Tuple[float, np.ndarray, float]]): Buffer to sample
        t (float): Time stamp to sample from

    Returns:
        Tuple[np.ndarray, float]|None: Tuple of interpolated pos and theta
    """
    # Buf is sorted by time
    # Search for first timestamp in buffer that is lower than current timestamp
    # TODO: replace with binary search
    lower_i = floorkey_buf(buf, t)
    if lower_i == None:
        return None
    lower_bound = buf[lower_i]

    if lower_i < len(buf) - 1:
        upper_i = lower_i + 1
        upper_bound = buf[upper_i]
    else:
        return lower_bound[1], lower_bound[2]

    lower_t, lower_x, lower_theta = lower_bound
    upper_t, upper_x, upper_theta = upper_bound

    interp = (t - lower_t) / (upper_t - lower_t)

    x_interp = (upper_x - lower_x) * interp + lower_x
    theta_interp = (upper_theta - lower_theta) * interp + lower_theta
    return x_interp, theta_interp


class VisionUpdate:
    """Vision update record
    Records the vision compensated pose as well as the odometry pose at the time of the vision pose
    """

    def __init__(
        self,
        vision_x: np.ndarray,
        vision_theta: float,
        odom_x: np.ndarray,
        odom_theta: float,
    ):
        self.vision_x = vision_x
        "Position from the vision results"
        self.vision_theta = vision_theta
        "Rotation from the vision results"
        self.odom_x = odom_x
        "Position of the drivetrain odometry at the time of the vision pose"
        self.odom_theta = odom_theta
        "Rotation of the drivetrain odometry at the time of the vision pose"

    def compensate(
        self, pos_x: np.ndarray, pos_theta: float
    ) -> Tuple[np.ndarray, float]:
        """Compensates the vision results based on the current drivetrain odometry

        Args:
            pos_x (np.ndarray): Position of the drivetrain odometry
            pos_theta (float): Rotation from the drivetrain odometry

        Returns:
            Tuple[np.ndarray, float]: Tuple of the compensated position and rotation
        """
        delta_x, delta_theta = pose_sub(pos_x, pos_theta, self.odom_x, self.odom_theta)
        return pose_add(self.vision_x, self.vision_theta, delta_x, delta_theta)


# Pose estimation code based on wpilib:
class PoseEstimator:
    def __init__(self, state_stddev: np.ndarray|List = [0.05, 0.05, 0.05], vision_stddev: np.ndarray|List = [0.03, 0.03, 0.03]):
        state_stddev = np.array(state_stddev).reshape(-1, 1)
        vision_stddev = np.array(vision_stddev).reshape(-1, 1)

        self.sample_limit = 0.5  # Retain samples from up to 0.5 seconds ago
        "Limit on the age of odometry samples"

        self._k_vision = np.zeros((3, 3))
        "Covariance matrix for vision results"

        self._odom_buf: List[Tuple[float, np.ndarray, float]] = []
        "Buffer containing stamped poses from odometry"
        self._vision_buf: List[Tuple[float, VisionUpdate]] = []
        "Buffer containing stamped poses from vision"

        self._x_est = np.zeros(2)
        "Estimated position based on odometry and vision data"
        self._theta_est = 0
        "Estimated rotation based on odometry and vision data"

        self._q: np.ndarray = state_stddev[:, 0] ** 2
        "Covariance matrix of the state used when adding vision results"

        self.set_vision_stddev(vision_stddev)

    def set_vision_stddev(self, vision_stddev: np.ndarray):
        """Sets the covariance for new vision results

        Args:
            vision_stddev (np.ndarray): Array of shape (3,1) where (0,0) -> X stddev [m], (1,0) -> Y stddev [m], (2,0) -> Theta stddev [rad]
        """

        r = (vision_stddev[:, 0] ** 2).flatten()
        q = self._q.flatten()
        # Closed form kalman gain
        self._k_vision = np.diag(q / (q + np.sqrt(q * r)))

    def reset_pose(self, x: np.ndarray, theta: float):
        """Resets the pose of the robot, clearing the odometry and vision pose buffers

        Args:
            x (np.ndarray): New position of the robot
            theta (float): New rotation of the robot
        """
        self._odom_buf = []
        self._vision_buf = []

        self._x_est = x
        self._theta_est = theta

    @property
    def x_est(self) -> np.ndarray:
        """Estimated position based on both vision and odometry

        Returns:
            np.ndarray: Estimated position
        """
        return self._x_est

    @property
    def theta_est(self) -> float:
        """Estimated rotation based on both vision and odometry

        Returns:
            float: Estimated rotation
        """
        return self._theta_est

    def sample_at(self, t: float) -> Tuple[np.ndarray, float] | None:
        """Returns the pose at a given time if the odom buffer is not empty

        Args:
            t (float): Timestamp in seconds

        Returns:
            Tuple[np.ndarray, float]|None: Tuple of pos and theta
        """
        if len(self._odom_buf) == 0:
            return None

        oldest_t = self._odom_buf[0][0]
        newest_t = self._odom_buf[-1][0]
        t = clamp(t, oldest_t, newest_t)

        # Only used odometry data if no vision data is present
        if len(self._vision_buf) == 0 or t < self._vision_buf[0][0]:
            return sample_timestamp_buf(self._odom_buf, t)

        # WPILIB uses floorkey instead of direct buffer, see if necessary
        i = floorkey_buf(self._vision_buf, t)
        if i == None:
            return None
        _, vision_update = self._vision_buf[i]

        odom = sample_timestamp_buf(self._odom_buf, t)
        if odom == None:
            return None
        odom_x, odom_theta = odom

        return vision_update.compensate(odom_x, odom_theta)

    def _remove_stale_vision_samples(self):
        """Removes vision samples older than the oldest odometry pose
        """
        if len(self._odom_buf) == 0:
            return
        oldest_t = self._odom_buf[0][0]

        if len(self._vision_buf) == 0 or self._vision_buf[0][0] > oldest_t:
            return

        i = floorkey_buf(self._vision_buf, oldest_t)
        if i == None:
            return
        self._vision_buf = self._vision_buf[i:]

    def add_vision_measurements(self, pos_x: np.ndarray, pos_theta: float, t: float):
        """Adds a vision measurement to the pose estimator
        Note the vision pose should be transformed to have the 
        same coordinate system as the center of the robot

        Args:
            pos_x (np.ndarray): Position of the vision pose
            pos_theta (float): Rotation of the vision pose
            t (float): Timestamp of the vision pose
        """
        if len(self._odom_buf) == 0 or self._odom_buf[0][0] - self.sample_limit > t:
            return
        self._remove_stale_vision_samples()

        odom_sample = sample_timestamp_buf(self._odom_buf, t)
        if odom_sample == None:
            return
        odom_sample_x, odom_sample_theta = odom_sample

        vision_sample = self.sample_at(t)
        if vision_sample == None:
            return
        vision_sample_x, vision_sample_theta = vision_sample

        # Compute the twist and scale it by the confidence value for the vision procedure
        twist_x, twist_theta = pose_log(
            vision_sample_x, vision_sample_theta, pos_x, pos_theta
        )
        scaled_twist = (
            self._k_vision @ np.append(twist_x, twist_theta).reshape((-1, 1))
        ).flatten()
        scaled_twist_x = scaled_twist[:2]
        scaled_twist_theta = scaled_twist[2]

        # Calculate vision update
        vision_pos_x, vision_pos_theta = pose_exp(
            vision_sample_x, vision_sample_theta, scaled_twist_x, scaled_twist_theta
        )
        vision_update = VisionUpdate(
            vision_pos_x, vision_pos_theta, odom_sample_x, odom_sample_theta
        )
        self._vision_buf.append([t, vision_update])

        # Use vision result compensated with latest odometry
        self._x_est, self._theta_est = vision_update.compensate(
            self._odom_buf[-1][1], self._odom_buf[-1][2]
        )

    def update_with_time(
        self, odom_x: np.ndarray, odom_theta: float, t: float
    ) -> Tuple[np.ndarray, float]:
        """Updates the odometry for the pose estimator
        This should be called with a relatively fixed delta time and
        as soon as new odometry information is received

        Args:
            odom_x (np.ndarray): Position of the odometry
            odom_theta (float): Rotation of the odometry
            t (float): Timestamp of the odometry

        Returns:
            Tuple[np.ndarray, float]: Compensated pose from the pose estimator
        """

        self._odom_buf.append([t, odom_x, odom_theta])

        # Remove out of date samples
        for i, sample in enumerate(self._odom_buf):
            if sample[0] > t - self.sample_limit:
                break
        self._odom_buf = self._odom_buf[i:]

        if len(self._vision_buf) == 0:
            self._x_est = odom_x
            self._theta_est = odom_theta
        else:
            vision_update = self._vision_buf[-1][1]
            self._x_est, self._theta_est = vision_update.compensate(odom_x, odom_theta)

    def add_vision_pose(self, pose: Pose2d, t: float):
        self.add_vision_measurements(pose.translation().toVector(), pose.rotation().radians(), t)
