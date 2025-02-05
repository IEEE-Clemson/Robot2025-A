from subsystems.math.poseestimator import PoseEstimator, floorkey_buf, sample_timestamp_buf
import numpy as np

def test_samplers():
    pose_estimator = PoseEstimator([0.02, 0.02, 0.02], [0.05, 0.05, 0.05])
    pose_estimator.sample_limit = 5
    pose_estimator.reset_pose(np.array([0, 0]), 0)
    pose_estimator.update_with_time(np.array([0.0, 0.0]), 0.0, 0)
    pose_estimator.update_with_time(np.array([0.4, 0.4]), 0.2, 1.0)
    pose_estimator.update_with_time(np.array([1.0, 1.0]), 0.3, 2.0)

    pos, theta = sample_timestamp_buf(pose_estimator._odom_buf, 0.5)
    assert np.allclose(pos, np.array([0.2, 0.2]))
    assert np.isclose(theta, 0.1)

    pos, theta = sample_timestamp_buf(pose_estimator._odom_buf, 1.5)
    assert np.allclose(pos, np.array([0.7, 0.7]))
    assert np.isclose(theta, 0.25)


def test_sample_at():
    pose_estimator = PoseEstimator([0.02, 0.02, 0.02], [0.05, 0.05, 0.05])
    pose_estimator.sample_limit = 5
    pose_estimator.reset_pose(np.array([0, 0]), 0)

    # Test with only vision data
    pose_estimator.update_with_time(np.array([0.0, 0.0]), 0.0, 0)
    pose_estimator.update_with_time(np.array([0.4, 0.4]), 0.2, 1.0)
    pose_estimator.update_with_time(np.array([1.0, 1.0]), 0.3, 2.0)
    
    pos, theta = pose_estimator.sample_at(0.5)
    assert np.allclose(pos, np.array([0.2, 0.2]))
    assert np.isclose(theta, 0.1)

    pos, theta = pose_estimator.sample_at(1.5)
    assert np.allclose(pos, np.array([0.7, 0.7]))
    assert np.isclose(theta, 0.25)


def test_sample_limit():
    pose_estimator = PoseEstimator([0.02, 0.02, 0.02], [0.05, 0.05, 0.05])
    pose_estimator.sample_limit = 0.5
    pose_estimator.reset_pose(np.array([0, 0]), 0)
    pose_estimator.update_with_time(np.array([0.1, 0.1]), 0.1, 0)
    pose_estimator.update_with_time(np.array([0.2, 0.2]), 0.2, 0.6)
    pose_estimator.update_with_time(np.array([0.3, 0.3]), 0.3, 1.0)

    assert len(pose_estimator._odom_buf) == 2

def test_poseestimator():
    pose_estimator = PoseEstimator([0.05, 0.05, 0.05], [0.03, 0.03, 0.03])
    pose_estimator.reset_pose(np.array([0, 0]), 0)
    pose_estimator.update_with_time(np.array([0.1, 0.1]), 0.1, 0)
    pose_estimator.update_with_time(np.array([0.2, 0.2]), 0.2, 0.6)
    pose_estimator.update_with_time(np.array([0.3, 0.3]), 0.3, 1.0)

    # Verify pose
    assert np.allclose(pose_estimator.x_est, np.array([0.3, 0.3]))
    assert np.isclose(pose_estimator.theta_est, 0.3)

    # Add vision measurements
    pose_estimator.add_vision_measurements(np.array([0.4, 0.4]), 0.4, 1.0)
    # Verify vision update has been added
    assert len(pose_estimator._vision_buf) == 1

    
    # Add more vision measurements and verify they converge
    for i in range(5):
        pose_estimator.update_with_time(np.array([0.3, 0.3]), 0.3, 1.05 + 0.1 * i)
        pose_estimator.add_vision_measurements(np.array([0.4, 0.4]), 0.4, 1.0 + 0.1 * i)
    # Verify pose
    assert np.allclose(pose_estimator.x_est, np.array([0.4, 0.4]), atol=0.01, rtol=0.01)
    assert np.isclose(pose_estimator.theta_est, 0.4, atol=0.01, rtol=0.01)