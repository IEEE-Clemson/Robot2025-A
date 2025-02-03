#https://file.tavsys.net/control/controls-engineering-in-frc.pdf

from math import pi, sqrt
from typing import Tuple
import numpy as np
def pose_add(a_x: np.ndarray, a_theta: float, b_x: np.ndarray, b_theta: float) -> Tuple[np.ndarray, float]:
    """Applys a transform B to a I.e if a were in local coordinate of b, bring to world space

    Args:
        a_x (np.ndarray): Position of A
        a_theta (float): Rotation of A
        b_x (np.ndarray): Position of B
        b_theta (float): Rotation of B

    Returns:
        Tuple[np.ndarray, float]: Tuple of position and rotation
    """
    # Equivalent to A * B in matrix form
    theta = a_theta + b_theta
    rot_mat = np.array([[np.cos(a_theta), -np.sin(a_theta)],
                        [np.sin(a_theta),  np.cos(a_theta)]])
    x = rot_mat.dot(b_x.reshape((-1, 1))).flatten() + a_x
    return x, theta

def pose_sub(a_x: np.ndarray, a_theta: float, b_x: np.ndarray, b_theta: float) -> Tuple[np.ndarray, float]:
    """Computes a transformation T such that pose_add(b, T) = a

    Args:
        a_x (np.ndarray): Position of A
        a_theta (float): Rotation of A
        b_x (np.ndarray): Position of B
        b_theta (float): Rotation of B

    Returns:
        Tuple[np.ndarray, float]: Tuple of position and rotation
    """
    # Equivalent to A * B^-1 in matrix form
    theta = a_theta - b_theta
    rot_mat = np.array([[np.cos(-b_theta), -np.sin(-b_theta)],
                        [np.sin(-b_theta),  np.cos(-b_theta)]])
    x = rot_mat.dot((a_x - b_x).reshape((-1, 1))).flatten()
    return x, theta

def pose_log(a_x: np.ndarray, a_theta: float, b_x: np.ndarray, b_theta: float) -> Tuple[np.ndarray, float]:
    """Given two poses A and B, compute a twist T such that pose_exp(A, T) = B

    Args:
        a_x (np.ndarray): Position of A
        a_theta (float): Rotation of A
        b_x (np.ndarray): Position of B
        b_theta (float): Rotation of B

    Returns:
        Tuple[np.ndarray, float]: Tuple of linear and angular components of twist
    """
    dx, dtheta = pose_sub(b_x, b_theta, a_x, a_theta)
    half_theta = dtheta / 2
    cos_m_one = np.cos(dtheta) - 1

    if abs(cos_m_one) < 1e-9:
        c = 1.0 - 1.0 / 12.0 * dtheta * dtheta
    else:
        c = -(half_theta * np.sin(dtheta)) / cos_m_one
    total_theta = np.atan2(-half_theta, c)
    scale = np.hypot(half_theta, c)

    rot_mat = np.array([[np.cos(total_theta), -np.sin(total_theta)],
                        [np.sin(total_theta),  np.cos(total_theta)]])
    x = rot_mat.dot(dx.reshape((-1, 1))).flatten() * scale
    return x, dtheta

def pose_exp(a_x: np.ndarray, a_theta: float, t_x: np.ndarray, t_theta: float) -> Tuple[np.ndarray, float]:
    """Applies the given twist T to pose A

    Args:
        a_x (np.ndarray): Position of A
        a_theta (float): Rotation of A
        t_x (np.ndarray): Linear component of twist
        t_theta (float): Angular component of twist

    Returns:
        Tuple[np.ndarray, float]: Pose with twist applied
    """
    if abs(t_theta) < 1e-9:
        s = 1.0 - 1.0 / 6.0 * t_theta * t_theta
        c = 0.5 * t_theta
    else:
        s = np.sin(t_theta) / t_theta
        c = (1 - np.cos(t_theta)) / t_theta
    
    rot_mat = np.array([[s, -c],
                        [c,  s]])
    dx =  (rot_mat @ t_x.reshape((-1, 1))).flatten()
    return pose_add(a_x, a_theta, dx, t_theta)
    
if __name__ == "__main__":
    # Poor mans unit tests
    # Test minus
    a_x = np.array([0, 0])
    a_theta = pi/4
    b_x = np.array([5, 5])
    b_theta = pi/4
    c_x, c_theta = pose_sub(b_x, b_theta, a_x, a_theta)
    np.testing.assert_allclose(c_x, np.array([5*sqrt(2), 0]), rtol=1e-5, atol=1e-5)
    np.testing.assert_allclose(c_theta, 0, rtol=1e-5, atol=1e-5)


    a_x = np.array([-1, 0])
    a_theta = 0 #pi / 4
    b_x = np.array([0, 1])
    b_theta = pi / 2

    # b + (a - b) == a
    g_x, g_theta = pose_sub(a_x, a_theta, b_x, b_theta)
    f_x, f_theta = pose_add(b_x, b_theta, g_x, g_theta)

    np.testing.assert_allclose(f_x, a_x, rtol=1e-5, atol=1e-5)
    np.testing.assert_allclose(f_theta, a_theta, rtol=1e-5, atol=1e-5)

    # Twist log
    a_x = np.array([0, 0])
    a_theta = 0
    b_x = np.array([5, 5])
    b_theta = pi / 2
    t_x, t_theta = pose_log(a_x, a_theta, b_x, b_theta)
    np.testing.assert_allclose(t_x, np.array([5.0 / 2 * pi, 0]), rtol=1e-5, atol=1e-5)
    np.testing.assert_allclose(t_theta, pi / 2, rtol=1e-5, atol=1e-5)

    a_x = np.array([-1, 0])
    a_theta = 0 #pi / 4
    b_x = np.array([0, 1])
    b_theta = pi / 2
    # Twist
    t_x, t_theta = pose_log(a_x, a_theta, b_x, b_theta)
    h_x, h_theta = pose_exp(a_x, a_theta, t_x, t_theta)
    np.testing.assert_allclose(h_x, b_x, rtol=1e-5, atol=1e-5)
    np.testing.assert_allclose(h_theta, b_theta, rtol=1e-5, atol=1e-5)
