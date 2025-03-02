from multiprocessing import Queue
from time import time
from typing import List, Tuple
import numpy as np
import cv2
from pupil_apriltags import Detector, Detection


from wpimath.geometry import Pose3d, Rotation3d, Transform3d, Translation3d

from .vision_config import VisionConfig


class ApriltagResult:
    n = 0
    id = 0
    t = 0.0
    pose = np.array([])


def get_april_tag_poses(config: VisionConfig, image: np.ndarray, detector: Detector) -> List[Tuple[int, Transform3d]]:
    """Detects apriltags in the given BGR image and returns the pose and ID 

    Args:
        image (np.ndarray): BGR image to find the apriltags in
        detector (Detector): Detector configured for the camera of the given image

    Returns:
        List[Tuple[int, Transform3d]]: List of tuples of ID and transform of apriltag relative to camera
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detections: List[Detection] = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=(config.fx, config.fy, config.cx, config.cy),
        tag_size=config.tag_size)
    
    poses = []
    for detection in detections:
        # Get the pose of the tag
        swap_xy = np.array([[ 0,  0, 1],
                            [-1,  0, 0],
                            [ 0, -1, 0]])
        translation = swap_xy @ detection.pose_t
        rotation = swap_xy @ detection.pose_R @ np.linalg.inv(swap_xy)
        # Convert to pose so that it is easier to work with
        tag_in_cam = Transform3d(Translation3d(translation), Rotation3d(rotation))
        if detection.pose_err < 1e-7 and np.linalg.norm(detection.pose_t) < 1.0:
            poses.append((detection.tag_id, tag_in_cam))
    return np.array(poses)



def apriltag_detector_main(config: VisionConfig, queue: Queue):
    cap = cv2.VideoCapture(config.dev_index)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
    cap.set(cv2.CAP_PROP_FPS, 100)
    cap.set(cv2.CAP_PROP_EXPOSURE, 200)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, 500)
    
    detector = Detector(families="tag36h11")
    n = 0
    while True:
        t = time()
        ret, image = cap.read()
        if image is None:
            continue
        
        try:
            tags = get_april_tag_poses(config, image, detector)
            for id, pose in tags:
                res = ApriltagResult()
                res.id = id
                res.pose = pose.toMatrix()
                res.n = n
                res.t = t

                queue.put_nowait(res)

            if config.should_display:
                cv2.imshow("Camera", image)
                cv2.waitKey(1)
        except Exception as e:
            print("Exception in camera process: ", e)