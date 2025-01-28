
import asyncio
from dataclasses import Field
import platform
from turtle import right
from pyray import *
import pymunk
import raylib
import math

from sim import field
from sim.differential_chassis import DifferentialChassis
from sim.mecanum_chassis import MeccanumChassis
     
screen_width = 2000
screen_height = 2000


def draw_chassis(c: DifferentialChassis):
    r = c.body
    rl_push_matrix()
    rl_translatef(r.position[0], r.position[1], 0)
    rl_rotatef(math.degrees(r.angle), 0, 0, 1)

    draw_circle_v(Vector2(0, 0), 0.01, RED)
    draw_line_v(Vector2(-c.wheel_dist / 2, -c.wheel_radius), Vector2(-c.wheel_dist / 2, c.wheel_radius), RED)
    draw_line_v(Vector2(c.wheel_dist / 2, -c.wheel_radius), Vector2(c.wheel_dist / 2, c.wheel_radius), RED)
    draw_rectangle_lines_ex(Rectangle(-c.width / 2, -c.height / 2, c.width, c.height), 0.01, LIGHTGRAY)

    rl_pop_matrix()

dt = 0.02

def mecanum_drive(chassis: MeccanumChassis, x, y, rx):
    denominator = max(abs(x) + abs(y) + abs(rx), 1);
    frontLeftPower = (x + y + rx) / denominator
    backLeftPower = (x - y + rx) / denominator
    frontRightPower = (x - y - rx) / denominator
    backRightPower = (x + y - rx) / denominator

    v = 12
    chassis.set_motors(frontLeftPower * v, -frontRightPower * v, backLeftPower * v, -backRightPower * v)

def arcade_drive(rotate, drive, chassis: MeccanumChassis):
    global chassis_rigid_body
    """Drives the robot using arcade drive."""
    # variables to determine the quadrants
    maximum = max(abs(drive), abs(rotate))
    total, difference = drive + rotate, drive - rotate

    # set speed according to the quadrant that the values are in
    if drive >= 0:
        if rotate >= 0:  # I quadrant
            left_motor = maximum
            right_motor = difference
        else:            # II quadrant
            left_motor = total
            right_motor = maximum
    else:
        if rotate >= 0:  # IV quadrant
            left_motor = total
            right_motor = -maximum
        else:            # III quadrant
            left_motor = -maximum
            right_motor = difference

    chassis.set_motors(left_motor * 12, right_motor * -12)


async def main():
    space = pymunk.Space()
    # There aren't that many collisions so the number of iterations should be small
    space.iterations = 3
    space.sleep_time_threshold = 0.5
    space.gravity = 0, 0
    space.collision_bias = 0.001
    space.collision_slop = 0.001
    space.damping = 0.01
    

    sim_field = field.SimField(space)
    # Create segments around the edge of the screen.
    chassis = MeccanumChassis(space)
    chassis.body.position = -0.385, -0.43
    asyncio.create_task(main_robot_task(chassis))
    set_config_flags(raylib.FLAG_MSAA_4X_HINT)
    init_window(screen_width, screen_height, "Test")
    while not window_should_close():
        throttle = 0
        strafe = 0
        steer = 0
        if is_key_down(raylib.KEY_RIGHT):
            strafe = -1
        if is_key_down(raylib.KEY_LEFT):
            strafe = 1
        if is_key_down(raylib.KEY_UP):
            throttle = -1
        if is_key_down(raylib.KEY_DOWN):
            throttle = 1
        if is_key_down(raylib.KEY_Q):
            steer = 1
        if is_key_down(raylib.KEY_E):
            steer = -1
        #mecanum_drive(chassis, throttle, strafe, steer)
        iters = 3
        for i in range(3):
            chassis.sim_step(dt / iters)
            space.step(dt / iters)

        begin_drawing()
        rl_disable_backface_culling()
        clear_background(GRAY)
        draw_text(f"Vel {chassis.body.velocity.length :.3f}", 190, 200, 20, LIGHTGRAY)

        # draw 2m patch
        rl_push_matrix()
        
        viewport_height = 2.5
        scale = screen_height / viewport_height
        rl_scalef(scale, -scale, 1)
        rl_push_matrix()
        rl_rotatef(90, 1, 0, 0)
        draw_grid(20, 0.25)
        rl_pop_matrix()
        rl_translatef(viewport_height / 2, -viewport_height / 2, 0)

        sim_field.draw()
        chassis.draw()
        rl_pop_matrix()
        end_drawing()
        await asyncio.sleep(dt) # You MUST call this in your main loop
    close_window()

async def main_robot_task(chassis: MeccanumChassis):
    await asyncio.sleep(2)
    mecanum_drive(chassis, 0, -1, 0)
    await asyncio.sleep(1)
    mecanum_drive(chassis, -1, 0, 0)
    await asyncio.sleep(4)
    mecanum_drive(chassis, 0, 1, 0)
    await asyncio.sleep(4)
    mecanum_drive(chassis, 0, 0, 0)

asyncio.run(main())

