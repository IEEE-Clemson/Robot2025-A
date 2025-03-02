from machine import Pin, PWM, I2C # type: ignore
from newFiles.registerDefinitions import * 
from newFiles.BMI270 import BMI270
from math import cos, sin, radians
from newFiles.filter import * 




class IMU:
    def __init__(self, sclpin=1, sdapin=0, sample_rate=100) -> None:
        self.i2c = I2C(0, scl=Pin(sclpin), sda=Pin(sdapin))
        self.address = I2C_PRIM_ADDR

        self.BMI270 = BMI270(serialDevice=self.i2c)
        self.accel = [0,0,0]
        self.gyro = [0,0,0]
        self.yaw = 0
        self.sample_rate = sample_rate
        self.angle = 0.0
        self.matrix_z = ((1, 0, 0), (0, 1, 0), (0, 0, 1))

        self.filter = Fusion() 
        return None 
    
    def update_accel(self):
        self.accel = self.BMI270.get_acc_data()
        self.gyro = self.BMI270.get_gyr_data()
        self.filter.update(self.accel, self.gyro, dt=0.01)
        # print(self.filter.yaw)

        a_x : float = 0.1972763704
        a_y : float = 0.008317349556
        a_z : float = 9.719187625

        print(self.accel[0]-a_x, self.accel[1]-a_y, self.accel[2]-a_z)




    def update_psi(self, dt: float=0.01) -> None:

        omega_psi : float = self.BMI270.get_gyr_data()[2] * dt 
        self.angle += omega_psi 

        return None 
    

    def update_matrix(self, dt: float=0.01) -> None:


        self.update_psi(dt=dt)

        cos_psi : float = cos(radians(self.angle))
        sin_psi : float = sin(radians(self.angle))

        r_one   : tuple = (cos_psi, -1*sin_psi, 0)
        r_two   : tuple = (sin_psi, cos_psi, 0)
        r_three : tuple = (0, 0, 1)

        R = (r_one, r_two, r_three)

        self.matrix_z = R 

        return None 
    


    def test_matrix_update(self) -> None:

        self.update_matrix()

        row_one = self.matrix_z[0]
        row_two = self.matrix_z[1]
        row_three = self.matrix_z[2]


        print(f"[{row_one[0]} {row_one[1]} {row_one[2]}]")
        print(f"[{row_two[0]} {row_two[1]} {row_two[2]}]")
        print(f"[{row_three[0]} {row_three[1]} {row_three[2]}]")




        return None 