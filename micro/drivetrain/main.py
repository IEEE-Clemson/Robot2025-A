""" Main control file for drivetrain
Kinematics based on https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
"""

from encoder import Encoder
from pi_motor import PIMotor
import utime
import comms
import sys
from machine import Timer, Pin
import config

# Disable autorun because cancelling it through vscode is glitchy
devel = True
if devel:
    sys.exit()


led = Pin("LED") # Heartbeat LED
motor_fl = PIMotor(
    0, config.fl_pin_a, config.fl_pin_b, config.fl_pin_f, config.fl_pin_r
)
motor_fr = PIMotor(
    1, config.fr_pin_a, config.fr_pin_b, config.fr_pin_f, config.fr_pin_r
)
motor_bl = PIMotor(
    2, config.bl_pin_a, config.bl_pin_b, config.bl_pin_f, config.bl_pin_r
)
motor_br = PIMotor(
    3, config.br_pin_a, config.br_pin_b, config.br_pin_f, config.br_pin_r
)
motor_fl.invert_motor = config.fl_inverted
motor_fr.invert_motor = config.fr_inverted
motor_bl.invert_motor = config.bl_inverted
motor_br.invert_motor = config.br_inverted

target_x_vel = 0
target_y_vel = 0
target_omega = 0

local_x_vel = 0
local_y_vel = 0
omega = 0

def update_local_vel():
    global local_x_vel, local_y_vel, omega

    w_fl = motor_fl.cur_vel
    w_fr = motor_fr.cur_vel
    w_bl = motor_bl.cur_vel
    w_br = motor_br.cur_vel
    a = config.wheel_radius / 4
    b = config.wheel_radius / (4 *(config.wheel_dist_x + config.wheel_dist_y))
    local_x_vel = (w_fl + w_fr + w_bl + w_br) * a
    local_y_vel = (-w_fl + w_fr + w_bl - w_br) * a
    omega = (-w_fl +w_fr - w_bl + w_br) * b

def set_target_vel():
    vx = target_x_vel
    vy = target_y_vel

    a = 1 / config.wheel_radius
    b = (config.wheel_dist_x + config.wheel_dist_y) * target_omega

    motor_fl.target_velocity = a * (vx - vy - b)
    motor_fr.target_velocity = a * (vx + vy + b)
    motor_bl.target_velocity = a * (vx + vy - b)
    motor_br.target_velocity = a * (vx - vy + b)


def comm_error_handler():
    #TODO: Blink LED until fault is cleared
    # Can't use PWM since LED is done through wifi chip
    pass

comm = comms.Comm('NAVIGATION', comm_error_handler)

# Timer for PI
control_tim = Timer()
def control_update(timer):
    dt = 1.0 / config.freq
    motor_fl.update(dt)
    motor_fr.update(dt)
    motor_bl.update(dt)
    motor_br.update(dt)
    update_local_vel()

control_tim.init(freq=config.freq, mode=Timer.PERIODIC, callback=control_update)


def handle_control_request(speeds):
    global target_x_vel, target_y_vel, target_omega
    if len(speeds) != 3:
        comm.send('CONTROL', 'STATUS', False, local_x_vel, local_y_vel, omega)
    
    target_x_vel = speeds[0]
    target_y_vel = speeds[1]
    target_omega = speeds[2]

    comm.send('CONTROL', 'STATUS', True, local_x_vel, local_y_vel, omega)


# Comm loop
while True:
    #TODO: Ask to increase baud rate from 9600bps to 115200bps
    # If average packet size is ~16 bytes, we can only send 60 packets per second vs 720 packets per second
    try:
        packet = comm.receive()
        if packet is None:
            continue
        
        # TODO: Update packet names
        if packet.command_str == 'MOVE FORWARD':
            # This can be 4 16 bit ints in the future, but use floats for simplicity atm
            handle_control_request(*packet.arguments)
        
        if packet.command_str == 'STATUS':
            comm.send('CONTROL', 'STATUS', True, local_x_vel, local_y_vel, omega)
    except Exception as e:
        print("Warning: ", e)