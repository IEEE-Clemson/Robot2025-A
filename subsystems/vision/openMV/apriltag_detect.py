# apriltag_detect.py

import sensor
import time
import math

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

clock = time.clock()

FRAME_WIDTH = 320
FOCAL_LENGTH = 2.8 # mm
SENSOR_WIDTH = 3.99
PIXEL_SIZE = SENSOR_WIDTH / FRAME_WIDTH

while True:
    clock.tick()
    img = sensor.snapshot()
    for tag in img.find_apriltags():
        center_x = tag.cx
        center_y = tag.cy

        offset_x_mm = (center_x - (FRAME_WIDTH / 2)) * PIXEL_SIZE
        horizontal_angle = math.degrees(math.atan(offset_x_mm / FOCAL_LENGTH))

        img.draw_rectangle(tag.rect, color = (0, 0, 255))
        img.draw_cross(tag.cx, tag.cy, color = (0, 255, 0))
        print(f"Tag ID: {tag.id}")
        print("Angle to Tag Center:", horizontal_angle)
    print(clock.fps())
