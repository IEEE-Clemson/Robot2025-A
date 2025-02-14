from machine import Pin, PWM, I2C
from registerDefinitions import *
from BMI270 import BMI270
import math




class IMU:
    def __init__(self, sclpin=1, sdapin=0, sample_rate=100) -> None:
        self.i2c = I2C(I2C_BUS, scl=Pin(sclpin), sda=Pin(sdapin))
        self.address = I2C_PRIM_ADDR

        self.BMI270 = BMI270(serialDevice=self.i2c)
        self.accel = [0,0,0]
        self.gyro = [0,0,0]
        self.yaw = 0
        self.sample_rate = sample_rate

        self.filter = ButterworthFilter(cutoff_freq=1, sample_rate=sample_rate)

        
    def update(self):
        self.accel = self.BMI270.getAccel()
        self.gyro = self.BMI270.getGyro()
        self.gyro[2] = self.filter.filter(self.gyro[2])
        yaw += self.gyro[2] * (1/self.sample_rate)





class ButterworthFilter:
    def __init__(self, cutoff_freq, sample_rate, order=2):
        # Normalized cutoff frequency (cutoff frequency / Nyquist frequency)
        nyquist = 0.5 * sample_rate
        normal_cutoff = cutoff_freq / nyquist
        
        # Calculate filter coefficients using the bilinear transform method
        self.b, self.a = self.butterworth_coefficients(normal_cutoff, order)
        
        self.x = [0] * (len(self.b) - 1)  # Input values (previous)
        self.y = [0] * (len(self.a) - 1)  # Output values (previous)

    def butterworth_coefficients(self, cutoff, order):
        """Calculate the Butterworth filter coefficients."""
        c = math.sqrt(2)  # Critical damping
        theta_c = math.tan(math.pi * cutoff)

        # Coefficients for 2nd-order filter
        b0 = 1 / (1 + c * theta_c + theta_c**2)
        b1 = 2 * b0
        b2 = b0
        a1 = 2 * (theta_c**2 - 1) * b0
        a2 = (1 - c * theta_c + theta_c**2) * b0

        return [b0, b1, b2], [1, a1, a2]

    def filter(self, input_value):
        """Filter the input value using the Butterworth filter."""
        # Shift previous input values
        self.x = [input_value] + self.x[:-1]

        # Calculate the filtered output
        output_value = sum(b * x for b, x in zip(self.b, self.x)) - sum(a * y for a, y in zip(self.a[1:], self.y))

        # Shift previous output values
        self.y = [output_value] + self.y[:-1]

        return output_value