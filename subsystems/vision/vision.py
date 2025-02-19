from math import pi
from multiprocessing import Process, Queue
from typing import List

from commands2 import Subsystem

from wpimath.geometry import Pose3d, Rotation3d, Transform3d, Translation3d, Pose2d
from .apriltag_detector import apriltag_detector_main, ApriltagResult
from .vision_config import VisionConfig

TAG_HEIGHT = 0.05
id_to_pose = {
    0: Pose3d(0, 0.572, TAG_HEIGHT, Rotation3d(0, 0, pi )), # West wall
    1: Pose3d(0, 0.572, TAG_HEIGHT, Rotation3d(0, 0, pi )), # West wall
    2: Pose3d(0, 0.572, TAG_HEIGHT, Rotation3d(0, 0, pi )), # West wall
    3: Pose3d(0, 0.572, TAG_HEIGHT, Rotation3d(0, 0, pi )), # West wall
    4: Pose3d(0, 0.572, TAG_HEIGHT, Rotation3d(0, 0, pi )), # West wall

    5: Pose3d(0.812, 1.143, TAG_HEIGHT, Rotation3d(0, 0, pi / 2)), # North wall
    6: Pose3d(1.050, 0, TAG_HEIGHT, Rotation3d(0, 0, pi/2)), # South wall
    7: Pose3d(2.354, 0.572, TAG_HEIGHT, Rotation3d(0, 0, -pi)), # East wall
}

class Vision(Subsystem):
    def __init__(self, config: VisionConfig):
        self._config = config
        self._queue = Queue(maxsize=4096)
        self.process = Process(target=apriltag_detector_main, args=(config, self._queue))
        self.cur_tags: List[ApriltagResult] = []
        self.cur_pose2d = []
        self.add_pose2d_callback = None
        
        self.process.start()
    
    def periodic(self):
        self.cur_tags = []
        self.cur_pose2d = []
        while not self._queue.empty():
            res = self._queue.get()
            self.cur_tags.append(res)

        t = 0
        # Calculate pose2d from tags
        for tag in self.cur_tags:
            t = tag.t
            if tag.id not in id_to_pose:
                continue
            # 3D pose, Ideally should have z = 0, and only yaw rotation
            robot_in_field: Pose3d = id_to_pose[tag.id] \
                .transformBy(Transform3d.fromMatrix(tag.pose).inverse())\
                .transformBy(self._config.camera_pose.inverse())
            # TODO: Reproject onto XY plane
            pose2d = Pose2d(
                robot_in_field.translation().x,
                robot_in_field.translation().y,
                robot_in_field.rotation().Z()
            )
            self.cur_pose2d.append(pose2d)
            #print(robot_in_field.translation())
            
            if self.add_pose2d_callback is not None:
                self.add_pose2d_callback(t, pose2d)