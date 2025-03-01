# line_detect.py

import sensor
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

clock = time.clock()

min_degree = 0
max_degree = 180


while True:
    clock.tick()
    img = sensor.snapshot()
    for line in img.find_lines(threshold = 1000, theta_margin=25, rho_margin=25):
        if (min_degree <= line.theta()) and (max_degree >= line.theta()):
            img.draw_line(line.line(), color=(255, 0, 0))
    print(clock.fps())
