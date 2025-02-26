

from machine import Pin, PWM, I2C # type: ignore 
from BMI270 import BMI270 
from math import sin, cos, tan, asin, acos, atan, degrees, radians, pi
from time import sleep, sleep_ms, sleep_us, time_ns




class Quaternion:
    def __init__(self, linked_gyro: BMI270) -> None:

        """

        Name Axes Greek
        
        Roll  X   Phi 
        Pitch Y   Theta
        Yaw   Z   Psi 



        """


        self.angle_phi:   float = 0
        self.angle_theta: float = 0
        self.angle_psi:   float = 0


        self.omega_phi:   float = 0
        self.angle_theta: float = 0 
        self.angle_psi:   float = 0


        self.q_w : float = 0
        self.q_x : float = 0
        self.q_y : float = 0
        self.q_z : float = 0


        self.axes_x: float = 0
        self.axes_y: float = 0
        self.axes_y: float = 0
        self.axes_theta: float = 0


        self.linked_gyro = linked_gyro

        self.last_read_ts = 0




        return None 


    def update_omegas(self) -> None:


        new_omegas = self.linked_gyro.get_gyr_data()

        self.omega_phi   = new_omegas[0]
        self.omega_theta = new_omegas[1]
        self.omega_psi   = new_omegas[2]

        return None
    


    def approx_new_quaternion(self, dt: float) -> None:

        """
        Using this link: https://gamedev.stackexchange.com/questions/108920/applying-angular-velocity-to-quaternion

        Assumptions:

        We express the angular velocity as a quaternion vector (0, w_x, w_y, w_z)

        To update the quaternion of each component respectively, we use the hadamard product (element by element product) in order to compute
        the "delta q" given the readings


        We add this "delta q" to the current quaternion in order to set the new quaternion based on the data.
        
        """


        self.update_omegas()


        omega_phi   = (self.omega_phi   * 0.5 * dt)
        omega_theta = (self.omega_theta * 0.5 * dt)
        omega_psi   = (self.omega_psi   * 0.5 * dt) 

        updated_q_x = self.q_x * (omega_phi   + 1)
        updated_q_y = self.q_y * (omega_theta + 1)
        updated_q_z = self.q_z * (omega_psi   + 1)


        self.q_x = updated_q_x
        self.q_y = updated_q_y 
        self.q_z = updated_q_z


        return None 
    






    ''' STATIC METHODS / CALCULATION UTILS GO HERE '''
    

    # Link I used for the derivations for a lot of these equations: 
    # https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html

    @staticmethod 
    def euler_to_quaternion(angle_phi: float, angle_theta: float, angle_psi: float) -> tuple[float]:


        angle_phi   = radians(angle_phi)   * 0.5 # Roll
        angle_theta = radians(angle_theta) * 0.5 # Pitch
        angle_psi   = radians(angle_psi)   * 0.5 # Yaw

        cos_phi = cos(angle_phi)
        sin_phi = sin(angle_phi) 
        
        cos_theta = cos(angle_theta) 
        sin_theta = sin(angle_theta) 

        cos_psi = cos(angle_psi)
        sin_psi = sin(angle_psi)


        q_zero_calc  = (cos_phi * cos_theta * cos_psi) + (sin_phi * sin_theta * sin_psi) 
        q_one_calc   = (sin_phi * cos_theta * cos_psi) - (cos_phi * sin_theta * sin_psi)
        q_two_calc   = (cos_phi * sin_theta * cos_psi) + (sin_phi * cos_theta * sin_psi) 
        q_three_calc = (cos_phi * cos_theta * sin_psi) - (sin_phi * sin_theta * cos_psi) 


        q_full = (q_zero_calc, q_one_calc, q_two_calc, q_three_calc)


        return q_full


    @staticmethod
    def quaternion_to_euler(q_zero: float, q_one: float, q_two: float, q_three: float, q_four: float) -> tuple[float]:


        # X / Roll
        phi_calc = ( (2 * (q_zero*q_one + q_two*q_three) ) / (q_zero**2 - q_one**2 - q_two**2 + q_three**2) )
        phi_calc = atan(phi_calc) 

        # Y / Pitch 
        theta_calc = (2 * (q_zero*q_two - q_one*q_one)) 
        theta_calc = asin(theta_calc) 
        

        # Z / Yaw
        psi_calc = (2 * (q_zero*q_three + q_one*q_two)) / (q_one**2 + q_one**2 - q_two**2 - q_three**2)
        psi_calc = atan(psi_calc)





        converted_eulers = (phi_calc, theta_calc, psi_calc)



        return converted_eulers 
