from machine import Pin, PWM, I2C
from registerDefinitions import *
from BMI270 import BMI270
import math




class IMU:
    def __init__(self, sclpin=1, sdapin=0, sample_rate=100) -> None:
        self.i2c = I2C(0, scl=Pin(sclpin), sda=Pin(sdapin))
        self.address = I2C_PRIM_ADDR

        self.BMI270 = BMI270(serialDevice=self.i2c)
        self.accel = [0,0,0]
        self.gyro = [0,0,0]
        self.yaw = 0
        self.sample_rate = sample_rate

        
    def update(self):
        self.accel = self.BMI270.get_acc_data()
        self.gyro = self.BMI270.get_gyr_data()