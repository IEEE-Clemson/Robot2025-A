import machine
from machine import Pin
import utime
import struct
import math

led = Pin("LED", mode = Pin.OUT)

i2c = machine.I2C(1, scl=Pin(3, mode=Pin.ALT_I2C), sda=Pin(2, mode=Pin.ALT_I2C), freq=400000)
i2c.scan()

def init_mag():
    i2c.writeto_mem(0xD, 0x9, (0x19).to_bytes(1, 'little'))

def read_raw_data():
    buf = i2c.readfrom_mem(0xD, 0x0, 0x6)
    x, y, z = struct.unpack("<hhh", buf)
    return x, y, z

def get_mag():
    x, y, z = read_raw_data()
    mag = math.sqrt(x**2 + y**2 + z**2)
    return mag

init_mag()
prev_mag = get_mag()
while True:
    cur_mag = get_mag()
    if abs(prev_mag - cur_mag) > 500:
        led.on()
        utime.sleep_ms(1000)
        led.off()
        cur_mag = get_mag()
    prev_mag = cur_mag
    utime.sleep_ms(100)