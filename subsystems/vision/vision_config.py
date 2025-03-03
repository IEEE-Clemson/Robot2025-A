from wpimath.geometry import Pose3d, Rotation3d, Transform3d, Translation3d, Pose2d

class VisionConfig:
    dev_index = 0
    should_display = False
    fx = 674.92
    fy = 676.63
    cx = 438.86
    cy = 297.29

    tag_size = 0.08
    camera_pose = Transform3d(Translation3d(-0.115, 0.10, 0.235), Rotation3d.fromDegrees(0, 20, 180))