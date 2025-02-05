from math import pi, sqrt
import numpy as np
from numpy.testing import assert_allclose

from subsystems.math.pose import pose_add, pose_exp, pose_log, pose_sub


def test_pose_sub():
    a_x = np.array([0, 0])
    a_theta = pi / 4
    b_x = np.array([5, 5])
    b_theta = pi / 4
    c_x, c_theta = pose_sub(b_x, b_theta, a_x, a_theta)
    assert_allclose(c_x, np.array([5 * sqrt(2), 0]), rtol=1e-5, atol=1e-5)
    assert_allclose(c_theta, 0, rtol=1e-5, atol=1e-5)


def test_pose_sub_add():
    a_x = np.array([-1, 0])
    a_theta = pi / 4
    b_x = np.array([0, 1])
    b_theta = pi / 2

    g_x, g_theta = pose_sub(a_x, a_theta, b_x, b_theta)
    f_x, f_theta = pose_add(b_x, b_theta, g_x, g_theta)

    assert_allclose(f_x, a_x, rtol=1e-5, atol=1e-5)
    assert_allclose(f_theta, a_theta, rtol=1e-5, atol=1e-5)


def test_pose_log():
    # Twist log
    a_x = np.array([0, 0])
    a_theta = 0
    b_x = np.array([5, 5])
    b_theta = pi / 2
    t_x, t_theta = pose_log(a_x, a_theta, b_x, b_theta)
    assert_allclose(t_x, np.array([5.0 / 2 * pi, 0]), rtol=1e-5, atol=1e-5)
    assert_allclose(t_theta, pi / 2, rtol=1e-5, atol=1e-5)


def test_pose_exp_log():
    a_x = np.array([-1, 0])
    a_theta = pi / 4
    b_x = np.array([0, 1])
    b_theta = pi / 2
    # Twist
    t_x, t_theta = pose_log(a_x, a_theta, b_x, b_theta)
    h_x, h_theta = pose_exp(a_x, a_theta, t_x, t_theta)
    assert_allclose(h_x, b_x, rtol=1e-5, atol=1e-5)
    assert_allclose(h_theta, b_theta, rtol=1e-5, atol=1e-5)
