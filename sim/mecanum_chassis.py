from math import cos, pi, sin
import math
import numpy as np
import pymunk
from pyray import *

from sim.dcmotor import DCMotorConfig, DCMotorEncoderSim

G = 9.81

class MecanumWheel:
    def __init__(self, parent: pymunk.Body, rel_x, rel_y, theta, roller_theta):
        self.radius = 0.04
        self.roller_mu_s = 1.2
        self.roller_mu_rn = 0.0002
        self.roller_mu_rt = 0.0002
        self.roller_theta = roller_theta
        self.applied_force = pymunk.Vec2d(0, 0)

        self.rel_x = rel_x
        self.rel_y = rel_y
        self.theta = theta

        self.motor = DCMotorEncoderSim(DCMotorConfig())
        self.parent = parent

    def sim_step(self, dt: float):
        # TODO: Implement wheel slipping
        # TODO: Implement rolling friction
        # TODO: Implement proper encoder count

        # backpropagate previously applied forces from other constraints
        v = self.get_roller_local_velocity()
        self.motor.sim_set_velocity(v[1] / self.radius * cos(self.roller_theta))

        # calculate speed
        moment = self.parent.mass * (self.rel_x**2 + self.rel_y**2) + self.parent.moment
        self.motor.sim_apply_moment(moment * math.sqrt(2))
        
        applied_force_mag = self.motor.sim_get_torque() * cos(self.roller_theta) / self.radius
        self.applied_force = pymunk.Vec2d(1, 0).rotated(self.theta - self.roller_theta + pi/2).scale_to_length(applied_force_mag)
        self.parent.apply_force_at_local_point(self.applied_force, (self.rel_x, self.rel_y))

    def get_local_velocity(self):
        v_local = self.parent.velocity_at_local_point((self.rel_x, self.rel_y))
        return v_local.rotated(-self.parent.angle - self.theta)

    def get_roller_local_velocity(self):
        return self.get_local_velocity().rotated(self.roller_theta)

    def draw(self):
        rl_push_matrix()
        rl_translatef(self.rel_x, self.rel_y, 0)
        rl_rotatef(math.degrees(self.theta), 0, 0, 1)
        v_local = self.get_local_velocity()

        radius = self.radius
        draw_line_v(Vector2(0, 0), Vector2(*v_local), RED)
        draw_line_v(Vector2(0, 0), self.applied_force * 0.01, GREEN)

        for i in range(3):
            t = -self.roller_theta
            r = self.radius
            s = 0.5 * r
            off = 0.03 * (i-1)
            draw_line_v(Vector2(-sin(t) * s, cos(t) * s + off) , Vector2(-sin(t) * -s, cos(t) * -s + off), DARKGREEN)
            
        rl_pop_matrix()


class MeccanumChassis:
    def __init__(self, space: pymunk.Space):
        self.wheel_dist = 0.25
        self.width = 0.25
        self.length = 0.25

        self.body = pymunk.Body(body_type=pymunk.Body.DYNAMIC)
        self.body.position = 0, 0
        self.body.mass = 10
        self.shape = pymunk.Poly.create_box(self.body, (self.width, self.length))
        self.shape.mass = self.body.mass
        self.shape.body = self.body
        self.body.moment = self.shape.moment

        space.add(self.body, self.shape)

        self.tl_wheel = MecanumWheel(
            self.body, -self.wheel_dist / 2, self.wheel_dist / 2, pi, pi / 4
        )
        self.tr_wheel = MecanumWheel(
            self.body, self.wheel_dist / 2, self.wheel_dist / 2, 0, -pi / 4
        )
        self.bl_wheel = MecanumWheel(
            self.body, -self.wheel_dist / 2, -self.wheel_dist / 2, pi, -pi / 4
        )
        self.br_wheel = MecanumWheel(
            self.body, self.wheel_dist / 2,  -self.wheel_dist / 2, 0, pi / 4
        )

    def sim_step(self, dt: float):
        self.tl_wheel.sim_step(dt)
        self.tr_wheel.sim_step(dt)
        self.bl_wheel.sim_step(dt)
        self.br_wheel.sim_step(dt)
    
    def set_motors(self, v_tl: float, v_tr: float, v_bl: float, f_tr: float):
        self.tl_wheel.motor.set_voltage(v_tl)
        self.tr_wheel.motor.set_voltage(v_tr)
        self.bl_wheel.motor.set_voltage(v_bl)
        self.br_wheel.motor.set_voltage(f_tr)

    def draw(self):
        rl_push_matrix()
        rl_translatef(self.body.position[0], self.body.position[1], 0)
        rl_rotatef(math.degrees(self.body.angle), 0, 0, 1)
        draw_rectangle_lines_ex(Rectangle(-self.width / 2, -self.length / 2, self.width, self.length), 0.01, LIGHTGRAY)
        draw_line_v((0, 0), (0, 0.1), RED)
        self.tl_wheel.draw()
        self.tr_wheel.draw()
        self.bl_wheel.draw()
        self.br_wheel.draw()
        rl_pop_matrix()
        