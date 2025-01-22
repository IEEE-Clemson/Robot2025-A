
import asyncio
import platform
from pyray import *
import pymunk
import raylib
import math

from sim import field
from sim.differential_chassis import DifferentialChassis
     
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

chassis = DifferentialChassis()
dt = 0.02

def arcade_drive(rotate, drive):
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
    

    # Create segments around the edge of the screen.
    field.create_field_hitbox(space)

    space.add(chassis.body, chassis.shape)

    set_config_flags(raylib.FLAG_MSAA_4X_HINT)
    init_window(screen_width, screen_height, "Hello")
    while not window_should_close():
        throttle = 0
        steer = 0
        if is_key_down(raylib.KEY_RIGHT):
            steer = 1
        if is_key_down(raylib.KEY_LEFT):
            steer = -1
        if is_key_down(raylib.KEY_UP):
            throttle = 1
        if is_key_down(raylib.KEY_DOWN):
            throttle = -1
        arcade_drive(steer, throttle)

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

        field.draw_field()
        draw_chassis(chassis)
        rl_pop_matrix()
        end_drawing()
        await asyncio.sleep(dt) # You MUST call this in your main loop
    close_window()

asyncio.run(main())