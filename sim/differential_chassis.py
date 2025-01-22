from turtle import left, right
import pymunk
from sim.dcmotor import DCMotorEncoderSim, DCMotorConfig
import numpy as np
from numpy import array, ndarray
from math import pi, sin, cos

from sim.masses import SquareMass

G = 9.81

def clamp(x, minimum, maximum):
    return max(min(x, maximum), minimum)

class DifferentialChassis:
    def __init__(self):
        self.width = 0.3
        self.height = 0.3

        self.masses = [SquareMass([self.width, self.height], [0.0, 0.0], 10.0)]
        self.wheel_dist = 0.2
        self.wheel_radius = 0.04
        self.wheel_mu_s = 1.2
        
        self.wheel_mu_r = 0.01

        self.left_motor = DCMotorEncoderSim(DCMotorConfig())
        self.right_motor = DCMotorEncoderSim(DCMotorConfig())

        self.body = pymunk.Body(body_type=pymunk.Body.DYNAMIC)
        self.body.position = 0, 0
        self.body.mass = sum([mass.get_mass() for mass in self.masses])
        self.body.moment = sum([mass.get_moment_2d() for mass in self.masses])
        self.shape = pymunk.Poly.create_box(self.body, (self.width, self.height))
        self.shape.body = self.body

    def set_motors(self, left_voltage: float, right_voltage: float):
        self.left_motor.set_voltage(left_voltage)
        self.right_motor.set_voltage(right_voltage)

    def sim_step(self, dt: float):
        # force from wheels will be interpreted as external force
        # then results from constraint solving will be backpropagated to the motors
        # Note: this assumes that the wheels are infinitely thin

        forward = self.body.local_to_world((0, 1)) - self.body.position
        left_wheel_pos = (-self.wheel_dist / 2, 0)
        right_wheel_pos = (self.wheel_dist / 2, 0)

        v_l = self.body.velocity_at_local_point(left_wheel_pos).dot(forward)
        v_r = self.body.velocity_at_local_point(right_wheel_pos).dot(forward)

        raw_torque_left = self.left_motor.sim_get_torque()
        raw_torque_right = -self.right_motor.sim_get_torque() # invert because rotation is inverted relative to chassis

        max_rolling_friction = self.wheel_mu_r * G * self.body.mass
        max_static_friction = self.wheel_mu_s * G * self.body.mass
        force_left = clamp(raw_torque_left / self.wheel_radius, -max_static_friction, max_static_friction)
        force_right = clamp(raw_torque_right / self.wheel_radius, -max_static_friction, max_static_friction)

        friction_force_left = np.sign(v_l) * max_rolling_friction
        if max_rolling_friction * dt > abs(v_l):
            if(abs(v_l) > 0.001):
                self.body.apply_impulse_at_local_point((0, -v_l * self.body.mass), left_wheel_pos)
        else:
            self.body.apply_force_at_local_point((0, -friction_force_left), left_wheel_pos)

        friction_force_right = np.sign(v_r) * max_rolling_friction
        if max_rolling_friction * dt > abs(v_r):
            if(abs(v_r) > 0.001):
                self.body.apply_impulse_at_local_point((0, -v_r * self.body.mass), right_wheel_pos)
        else:
            self.body.apply_force_at_local_point((0, -friction_force_right), right_wheel_pos)

        local_velocity = self.body.velocity.rotated(-self.body.angle)
        local_forward_vel = local_velocity.dot((0, 1))
        slip_vel = local_velocity - local_forward_vel * pymunk.Vec2d(0, 1)
        slip_vel_mag = np.linalg.norm(slip_vel)
        if slip_vel_mag < max_static_friction * dt:
            if slip_vel_mag > 0.001:
                self.body.apply_impulse_at_local_point(-slip_vel * self.body.mass)
        else:
            self.body.apply_force_at_local_point(-max_static_friction * slip_vel / slip_vel_mag)

        self.body.apply_force_at_local_point((0, force_left), left_wheel_pos)
        self.body.apply_force_at_local_point((0, force_right), right_wheel_pos)


        self.left_motor.sim_set_velocity(-v_l / self.wheel_radius)
        self.right_motor.sim_set_velocity(v_r / self.wheel_radius)
        