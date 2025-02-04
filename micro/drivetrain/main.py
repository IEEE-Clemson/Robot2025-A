""" Main control file for drivetrain
Kinematics based on https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
"""

from encoder import Encoder
from pi_motor import PIMotor
import utime
import sys
from machine import Timer, Pin
import config
import network
import socket

# Disable autorun because cancelling it through vscode is glitchy
devel = False
use_wifi = True
if devel:
    sys.exit()


en1 = Pin(2, Pin.OUT)
en2 = Pin(3, Pin.OUT)
en3 = Pin(4, Pin.OUT)
en4 = Pin(5, Pin.OUT)
en1.on()
en2.on()
en3.on()
en4.on()

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

    motor_fl.target_velocity = a * (vx + vy - b)
    motor_fr.target_velocity = a * (vx - vy + b)
    motor_bl.target_velocity = a * (vx - vy - b)
    motor_br.target_velocity = a * (vx + vy + b)


def comm_error_handler():
    #TODO: Blink LED until fault is cleared
    # Can't use PWM since LED is done through wifi chip
    pass


prev_time = utime.time_ns()
measured_delta = 0
# Timer for PI
control_tim = Timer()
def control_update(timer: Timer):
    global measured_delta, prev_time
    start = utime.time_ns()
    dt = (start - prev_time) * 1e-9
    prev_time = start
    set_target_vel()
    motor_fl.update(dt)
    motor_fr.update(dt)
    motor_bl.update(dt)
    motor_br.update(dt)
    end = utime.time_ns()
    update_local_vel()
    measured_delta = end - start

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
if not use_wifi:
    import comms
    comm = comms.Comm('NAVIGATION', comm_error_handler)
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
else:
    static_ip = '192.168.1.100'  # Replace with your desired static IP
    subnet_mask = '255.255.255.0'
    gateway_ip = '192.168.1.254'
    dns_server = '8.8.8.8'

    print("Configuring network")
    ap = network.WLAN(network.AP_IF)
    ap.deinit()
    ap.config(ssid="Drivetrain", password="test", security=0)
    ap.active(True)

    while not ap.active():
        utime.sleep_ms(10)
    print("Network activated")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('', 8080))
    s.listen(5)
    ap.ifconfig((static_ip, subnet_mask, gateway_ip, dns_server))



    while True:
        target_x_vel = 0
        target_y_vel = 0
        target_omega = 0
        conn, addr = s.accept()
        conn: socket.socket
        try:
            
            print("Connection accepted")
            while True:
                try:
                    line: str = conn.readline().decode('utf-8')
                    speeds = [float(x) for x in line.split(',')]
                    target_x_vel = speeds[0]
                    target_y_vel = speeds[1]
                    target_omega = speeds[2]
                    conn.write(f"{local_x_vel:.2f},{local_y_vel:.2f},{omega:.2f}\n")
                except ValueError:
                    target_x_vel = 0
                    target_y_vel = 0
                    target_omega = 0
                    conn.write(f"{local_x_vel:.2f},{local_y_vel:.2f},{omega:.2f}\n")
        except OSError:
            target_x_vel = 0
            target_y_vel = 0
            target_omega = 0
        
    