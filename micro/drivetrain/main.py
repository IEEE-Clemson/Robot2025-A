""" Main control file for drivetrain
Kinematics based on https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
"""

import network
import socket
import utime
import sys
from machine import Timer, Pin, PWM, I2C, mem32, ADC
import _thread

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
    update_light_measures()
    #print("Light detected", is_light_on)
    update_local_vel()
    measured_delta = end - start


control_tim.init(freq=config.FREQ, mode=Timer.PERIODIC, callback=control_update)

PR_PIN = 3 		# ADC Photoresistor Pin 
PCT_INC = 1.03	# % increase from avg for detection
POLL_RATE = 0.5	# # of times/second values poll
is_light_on = False
avg = 0
n = 1
lightsum = 0

photoresistor = ADC(PR_PIN)
def update_light_measures():
    global is_light_on, avg, n, lightsum
    light = photoresistor.read_u16()        
    if ((light > (avg*PCT_INC)) and (n > 1)):
        is_light_on = True
        return
    
    lightsum = lightsum + light
    avg = lightsum / n
    n = n + 1


# Micropython api is horribly limited for some reason
class i2c_slave:
    I2C0_BASE = 0x40044000
    I2C1_BASE = 0x40048000
    IO_BANK0_BASE = 0x40014000
    
    mem_rw =  0x0000
    mem_xor = 0x1000
    mem_set = 0x2000
    mem_clr = 0x3000
    
    IC_CON = 0
    IC_TAR = 4
    IC_SAR = 8
    IC_DATA_CMD = 0x10
    IC_RAW_INTR_STAT = 0x34
    IC_RX_TL = 0x38
    IC_TX_TL = 0x3C
    IC_CLR_INTR = 0x40
    IC_CLR_RD_REQ = 0x50
    IC_CLR_TX_ABRT = 0x54
    IC_CLR_STOP_DET = 0x60
    IC_CLR_START_DET = 0x64

    IC_ENABLE = 0x6c
    IC_STATUS = 0x70
    
    def write_reg(self, reg, data, method=0):
        mem32[ self.i2c_base | method | reg] = data
        
    def set_reg(self, reg, data):
        self.write_reg(reg, data, method=self.mem_set)
        
    def clr_reg(self, reg, data):
        self.write_reg(reg, data, method=self.mem_clr)
                
    def __init__(self, i2cID = 0, sda=0,  scl=1, slaveAddress=0x41):
        self.scl = scl
        self.sda = sda
        self.slaveAddress = slaveAddress
        self.i2c_ID = i2cID
        if self.i2c_ID == 0:
            self.i2c_base = self.I2C0_BASE
        else:
            self.i2c_base = self.I2C1_BASE
        
        # 1 Disable DW_apb_i2c
        self.clr_reg(self.IC_ENABLE, 1)
        # 2 set slave address
        # clr bit 0 to 9
        # set slave address
        self.clr_reg(self.IC_SAR, 0x1ff)
        self.set_reg(self.IC_SAR, self.slaveAddress &0x1ff)
        # 3 write IC_CON  7 bit, enable in slave-only
        self.clr_reg(self.IC_CON, 0b01001001)
        # set SDA PIN
        mem32[ self.IO_BANK0_BASE | self.mem_clr |  ( 4 + 8 * self.sda) ] = 0x1f
        mem32[ self.IO_BANK0_BASE | self.mem_set |  ( 4 + 8 * self.sda) ] = 3
        # set SLA PIN
        mem32[ self.IO_BANK0_BASE | self.mem_clr |  ( 4 + 8 * self.scl) ] = 0x1f
        mem32[ self.IO_BANK0_BASE | self.mem_set |  ( 4 + 8 * self.scl) ] = 3
        # 4 enable i2c 
        self.set_reg(self.IC_ENABLE, 1)


    def anyRead(self):
        status = mem32[ self.i2c_base | self.IC_RAW_INTR_STAT] & 0x20
        if status :
            return True
        return False

    def put(self, data):
        # reset flag       
        self.clr_reg(self.IC_CLR_TX_ABRT,1)
        status = mem32[ self.i2c_base | self.IC_CLR_RD_REQ]
        mem32[ self.i2c_base | self.IC_DATA_CMD] = data  & 0xff

    def any(self):
        # get IC_STATUS
        status = mem32[ self.i2c_base | self.IC_STATUS]
        # check RFNE receive fifio not empty
        if (status &  8) :
            return True
        return False
    
    def get(self):
        while not self.any():
            pass
        return mem32[ self.i2c_base | self.IC_DATA_CMD] & 0xff
    
    def started(self):
        stopped = False
        # TX ABRT:
        if mem32[self.i2c_base | self.IC_RAW_INTR_STAT ] & (1 << 6) != 0:
            self.clr_reg(self.IC_CLR_TX_ABRT,1)
            stopped = True
        # START DET
        if mem32[self.i2c_base | self.IC_RAW_INTR_STAT ] & (1 << 10) != 0:
            self.clr_reg(self.IC_CLR_START_DET,1)
            stopped = True
        # STOP DET
        if mem32[self.i2c_base | self.IC_RAW_INTR_STAT ] & (1 << 9) != 0:
            self.clr_reg(self.IC_CLR_STOP_DET,1)
            #stopped = True

        return stopped


s_i2c = i2c_slave(0,sda=0,scl=1,slaveAddress=0x41)

buf_size = 32
data_buf = [i for i in range(buf_size)]


# need to run i2c on second core so it wont get interrupted
def i2c_task():
    try:
        addr = 0
        addr_read = False
        try:
            while True:
                if s_i2c.any():
                    if not addr_read:
                        addr = s_i2c.get()
                        addr_read = True
                    else:
                        data_buf[addr] = s_i2c.get()
                        addr = (addr + 1) % buf_size
                if s_i2c.anyRead():
                    s_i2c.put(data_buf[addr])
                    addr = (addr + 1) % buf_size
                if s_i2c.started():
                    addr_read = False
        except Exception:
            pass
    except KeyboardInterrupt:
        pass

_thread.start_new_thread(i2c_task, ())

while True:
    utime.sleep_ms(500)
    led.toggle()

def handle_control_request(speeds):
    global target_x_vel, target_y_vel, target_omega
    if len(speeds) != 3:
        comm.send("CONTROL", "RESPONSE", False, local_x_vel, local_y_vel, omega)
        return

    target_x_vel = speeds[0]
    target_y_vel = speeds[1]
    target_omega = speeds[2]

    comm.send("CONTROL", "RESPONSE", bool(True), float(local_x_vel), float(local_y_vel), float(omega))




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
                t = utime.ticks_us()
                #handle_control_request(packet.arguments)
                print(utime.ticks_us() - t)
                
            if packet.command_str == "STATUS":
                comm.send("CONTROL", "RESPONSE", bool(True), float(local_x_vel), float(local_y_vel), float(omega))
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
