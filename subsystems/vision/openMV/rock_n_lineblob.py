'''
This script is a "fork" of the multi_color_blob_tracking example created by MIT
Purpose: track purple rocks and white lines
Suggestions: use the region of interest (roi) parameter when finding those objects,
        so as to reduce the portion of the image for where the objects are considered.
        This would limit interference of similar colors outside the field whilst also
        reducing decision paralysis for the robot.
'''


# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Multi Color Blob Tracking Example
#
# This example shows off multi color blob tracking using the OpenMV Cam.

import sensor
import time
import math

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green things. You may wish to tune them...
thresholds = [
    (0, 57, -22, 57, -56, -15), # rocks normal, thresh 1
    # (30, 100, -64, -8, -32, 32), # green stuff, thresh 2
    (100, 100, -5, 19, -6, 127), # thresh 3
]  # generic_blue_thresholds
# You may pass up to 16 thresholds above. However, it's not really possible to segment any
# scene with 16 thresholds before color thresholds start to overlap heavily.

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()

# my code
# isCaveCentered = 0 # is the robot oriented such to enter the cave safely?
# # when cavecentered, the bounds for the lines should be similar
# # when closer to one line, the bounding box and edge box
# roi_lowhalf = (80,100,230,130)


# def get_rock_data(blob_rect, fc, rock_width: float)->tuple:
#     '''blob_rect: (x,y,w,h)
#     fc - focal|xy, center|xy
#     rock_width - cm
#     distance - same units as rock_width
#     angle - from axis parallel to camera normal; add 90

#     Return: (0,0) or (distance, angle)
#     '''
#     p = 0
#     theta = 0
#     print("hello")
#     if blob_rect[2]*blob_rect[3] > 0:
#         px = rock_width * fc[0] / blob_rect[2]
#         py = rock_width * fc[1] / blob_rect[3]
#         p = math.sqrt(px**2+py**2)
#         if not( fc[2]-blob_rect[0] == 0):
#             theta = math.atan(p/(fc[2]-blob_rect[0]))
#     return (p,theta)



# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. Don't set "merge=True" because that will merge blobs which we don't want here.

while True:
    clock.tick()
    img = sensor.snapshot()
    #cx = img.width()/2
    #cy = img.height()/2
    #fx = (2.8 / 3.984) * 656
    #fy = (2.8 / 2.952) * 488
    for blob in img.find_blobs(thresholds, pixels_threshold=200, area_threshold=200):
        # These values depend on the blob not being circular - otherwise they will be shaky.
        if blob.elongation() > 0.5:
            img.draw_edges(blob.min_corners(), color=(255, 0, 0))
            img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
            img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
        # These values are stable all the time.
        img.draw_rectangle(blob.rect())
        #print(get_rock_data(blob.rect(), (fx,fy,cx,cy), 3))
        img.draw_cross(blob.cx(), blob.cy())
        # Note - the blob rotation is unique to 0-180 only.
        img.draw_keypoints(
            [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
        )
    #print(clock.fps())
