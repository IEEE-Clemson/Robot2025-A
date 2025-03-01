# apriltag_detect.py

import sensor
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    for tag in img.find_apriltags():
        img.draw_rectangle(tag.rect, color = (0, 0, 255))
        img.draw_cross(tag.cx, tag.cy, color = (0, 255, 0))
        print(f"Tag ID: {tag.id}")
    print(clock.fps())
