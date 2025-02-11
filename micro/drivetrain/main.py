""" Main control file for drivetrain
Kinematics based on https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
"""

import network
import socket
import utime
import sys
from machine import Timer, Pin, PWM

from pi_motor import PIMotor
import config


# Disable autorun because cancelling it through vscode is glitchy
devel = False
use_wifi = True
if devel:
    sys.exit()


en1 = Pin(2, Pin.OUT)
en2 = Pin(3, Pin.OUT)
en3 = Pin(4, Pin.OUT)
en4 = Pin(5, Pin.OUT)

en_intake = Pin(22, Pin.OUT)

# pwm_intake = PWM(Pin(27, Pin.OUT))
# pwm_intake.freq(1000)
en_intake.off()
# pwm_intake.duty_u16(65000)

en1.on()
en2.on()
en3.on()
en4.on()


led = Pin("LED")  # Heartbeat LED
motor_fl = PIMotor(
    0, config.FL_PIN_A, config.FL_PIN_B, config.FL_PIN_F, config.FL_PIN_R
)
motor_fr = PIMotor(
    1, config.FR_PIN_A, config.FR_PIN_B, config.FR_PIN_F, config.FR_PIN_R
)
motor_bl = PIMotor(
    2, config.BL_PIN_A, config.BL_PIN_B, config.BL_PIN_F, config.BL_PIN_R
)
motor_br = PIMotor(
    3, config.BR_PIN_A, config.BR_PIN_B, config.BR_PIN_F, config.BR_PIN_R
)
motor_fl.invert_motor = config.FL_INVERTED
motor_fr.invert_motor = config.FR_INVERTED
motor_bl.invert_motor = config.BL_INVERTED
motor_br.invert_motor = config.BR_INVERTED

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
    a = config.WHEEL_RADIUS / 4
    b = config.WHEEL_RADIUS / (4 * (config.WHEEL_DIST_X + config.WHEEL_DIST_Y))
    local_x_vel = (w_fl + w_fr + w_bl + w_br) * a
    local_y_vel = (-w_fl + w_fr + w_bl - w_br) * a
    omega = (-w_fl + w_fr - w_bl + w_br) * b


def set_target_vel():
    vx = target_x_vel
    vy = target_y_vel

    a = 1 / config.WHEEL_RADIUS
    b = (config.WHEEL_DIST_X + config.WHEEL_DIST_Y) * target_omega

    motor_fl.target_velocity = a * (vx - vy - b)
    motor_fr.target_velocity = a * (vx + vy + b)
    motor_bl.target_velocity = a * (vx + vy - b)
    motor_br.target_velocity = a * (vx - vy + b)


def comm_error_handler():
    # TODO: Blink LED until fault is cleared
    # Can't use PWM since LED is done through wifi chip
    pass


prev_time = utime.time_ns()
measured_delta = 0
# Timer for PI
control_tim = Timer()


def control_update(timer: Timer):
    # pwm_intake.duty_u16(65000)

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


control_tim.init(freq=config.FREQ, mode=Timer.PERIODIC, callback=control_update)


def handle_control_request(speeds):
    global target_x_vel, target_y_vel, target_omega
    if len(speeds) != 3:
        comm.send("CONTROL", "STATUS", False, local_x_vel, local_y_vel, omega)

    target_x_vel = speeds[0]
    target_y_vel = speeds[1]
    target_omega = speeds[2]

    comm.send("CONTROL", "STATUS", True, local_x_vel, local_y_vel, omega)


# Comm loop
if not use_wifi:
    import comms

    comm = comms.Comm("NAVIGATION", comm_error_handler)
    while True:
        # TODO: Ask to increase baud rate from 9600bps to 115200bps
        # If average packet size is ~16 bytes, we can only send 60 packets per second vs 720 packets per second
        try:
            packet = comm.receive()
            if packet is None:
                continue

            # TODO: Update packet names
            if packet.command_str == "MOVE FORWARD":
                # This can be 4 16 bit ints in the future, but use floats for simplicity atm
                handle_control_request(*packet.arguments)

            if packet.command_str == "STATUS":
                comm.send("CONTROL", "STATUS", True, local_x_vel, local_y_vel, omega)
        except Exception as e:
            print("Warning: ", e)
else:
    static_ip = "192.168.1.100"  # Replace with your desired static IP
    subnet_mask = "255.255.255.0"
    gateway_ip = "192.168.1.254"
    dns_server = "8.8.8.8"

    print("Configuring network")
    ap = network.WLAN(network.AP_IF)
    ap.deinit()
    ap.config(ssid="Drivetrain", password="test", security=0)
    ap.active(True)

    while not ap.active():
        utime.sleep_ms(10)
    print("Network activated")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("", 8080))
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
                    line: str = conn.readline().decode("utf-8")
                    speeds = [float(x) for x in line.split(",")]
                    target_x_vel = speeds[0]
                    target_y_vel = speeds[1]
                    target_omega = speeds[2]
                    conn.write(f"{local_x_vel:.2f},{local_y_vel:.2f},{omega:.2f}\n")
                except ValueError as e:
                    target_x_vel = 0
                    target_y_vel = 0
                    target_omega = 0
                    conn.write(f"{local_x_vel:.2f},{local_y_vel:.2f},{omega:.2f}\n")
                    print(e)

        except OSError as e:
            target_x_vel = 0
            target_y_vel = 0
            target_omega = 0
            print(e)
