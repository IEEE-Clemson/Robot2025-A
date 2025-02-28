

from machine import Pin, PWM, I2C # type: ignore 
from time import sleep, sleep_ms, sleep_us
from math import cos, acos, sin, asin, tan, atan, atan2, radians, degrees, pi
from BMI270 import BMI270




class Matrix:
    def __init__(self, linked_gyro: BMI270) -> None:

        """ 
        KEY:

        X == Phi   == Roll
        Y == Theta == Pitch
        Z == Psi   == Yaw

        """


        self.linked_gyro = linked_gyro
        
    
        self.angle_phi   : float = 0
        self.angle_theta : float = 0
        self.angle_psi   : float = 0 
        '''
        angle_phi   -> The angle of the rigid body in respect to the x-axis 
        angle_theta -> The angle of the rigid body in respect to the y-axis 
        angle_psi   -> The angle of the rigid body in respect to the z-axis 


        '''



        self.omega_phi   : float = 0 
        self.omega_theta : float = 0
        self.omega_psi   : float = 0
        '''
        omega_phi   -> Angular acceleration of angle_phi (time derivative of phi)
        omega_theta -> Angular acceleration of angle_theta (time derivative of theta)
        omega_psi   -> Angular acceleration of angle_psi (time derivatiev of psi)

        '''


        self.matrix_X    : tuple = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
        self.matrix_Y    : tuple = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
        self.matrix_Z    : tuple = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
        '''

        These are the rotation matrices for each respective axes. These are updated and called upon when integrating the 



        '''

        self.matrix_R    : tuple = ((0, 0, 0), (0, 0, 0), (0, 0, 0))
        self.matrix_O    : tuple = ((0, 0, 0), (0, 0, 0), (0, 0, 0))




        return None 



    def update_omegas(self) -> None:

        new_omegas : list = self.linked_gyro.get_gyr_data()

        new_phi   : float = new_omegas[0] 
        new_theta : float = new_omegas[1]
        new_psi   : float = new_omegas[2]


        self.matrix_O = Matrix.derive_matrix_O(new_phi, new_theta, new_psi)




        return None 


    


    def update_axes_matrix(theta: float, mode: int) -> tuple:
        """
        FUNCTION 


        PURPOSE


        ARGUMENTS


        RETURNS


        """

        self.update_omegas()


        R = self.matrix_R 
        O = self.matrix_O



        # Individual Components of R

        r_11 : float = R[0][0]
        r_12 : float = R[0][1]
        r_13 : float = R[1][0]

        r_21 : float = R[1][0]
        r_22 : float = R[1][1]
        r_23 : float = R[1][2]

        r_31 : float = R[2][0]
        r_32 : float = R[2][1]
        r_33 : float = R[2][2]



        # Individual Components of O 

        o_11 : float = O[0][0]
        o_12 : float = O[0][1] 
        o_13 : float = O[0][2] 

        o_21 : float = O[1][0]
        o_22 : float = O[1][1]
        o_23 : float = O[1][2]

        o_31 : float = O[2][0]
        o_32 : float = O[2][1]
        o_33 : float = O[2][2]

     

        # Deriving M:






        return None 


 
    @staticmethod
    def derive_rotation_matrix(theta: float, mode: int) -> tuple:
        """
        FUNCTION  



        PURPOSE 



        ARGUMENTS



        RETURNS 
        
        
        """

        cos_theta : float = cos(radians(theta))
        sin_theta   : float = sin(radians(theta))


        if(mode == 0): # X axes rotation selected
            row_one   : tuple = (1, 0, 0)
            row_two   : tuple = (0, cos_theta, -1*sin_theta)
            row_three : tuple = (0, sin_theta, cos_theta)

            R = (row_one, row_two, row_three)
        
        elif(mode == 1): # Y axes rotation selected

            row_one   : tuple = (cos_theta, 0, sin_theta)
            row_two   : tuple = (0, 1, 0)
            row_three : tuple = (-1*sin_theta, 0, cos_theta)

            R = (row_one, row_two, row_three)
        
        elif(mode == 2): # Z axes rotation selected

            row_one : tuple = (cos_theta, -1*sin_theta, 0)
            row_two : tuple = (0, 1, 0)
            row_three : tuple = (-1*sin_theta, 0, cos_theta)

            R = (row_one, row_two, row_three)

        else:

            row_one   : tuple = (1, 0, 0)
            row_two   : tuple = (0, 1, 0) 
            row_three : tuple = (0, 0, 1)

            R = (row_one, row_two, row_three)



        return R 



    @staticmethod 
    def derive_matrix_O(omega_phi: float, omega_theta: float, omega_psi: float) -> tuple:
        """
        FUNCTION  



        PURPOSE 



        ARGUMENTS



        RETURNS 
        
        
        """

        r_11 : float = 0
        r_12 : float = (-1 * omega_psi) 
        r_13 : float = omega_theta

        r_21 : float = omega_psi 
        r_22 : float = 0
        r_23 : float = (-1 * omega_phi) 


        r_31 : float = (-1 * omega_theta)
        r_32 : float = omega_phi 
        r_33 : float = 0

        matrix_omega = ((r_11, r_12, r_13), (r_21, r_22, r_23), (r_31, r_32, r_33)) 


        return matrix_omega
    
    @staticmethod
    def derive_matrix_R(angle_phi: float, angle_theta: float, angle_psi: float) -> tuple:
        """
        FUNCTION  



        PURPOSE 



        ARGUMENTS



        RETURNS 
        
        
        """

        cos_phi : float = cos(radians(angle_phi))
        sin_phi : float = sin(radians(angle_phi))

        cos_theta : float = cos(radians(angle_theta))
        sin_theta : float = sin(radians(angle_theta))


        cos_psi  : float = cos(radians(angle_psi))
        sin_psi  : float = sin(radians(angle_psi))

        r_11 : float  = (cos_theta * cos_psi)
        r_12 : float  = (sin_phi * sin_theta * cos_psi) - (cos_phi * sin_psi)
        r_13 : float  = (cos_phi * sin_theta * cos_psi) + (sin_phi * sin_psi)

        r_21 : float  = (cos_theta * sin_psi) 
        r_22 : float  = (sin_phi * sin_theta * sin_psi) + (cos_phi * cos_psi) 
        r_23 : float  = (cos_phi * sin_theta * sin_psi) - (sin_phi * cos_psi)
        
        r_31 : float  = (-1 * sin_theta) 
        r_32 : float  = (sin_phi * cos_theta)
        r_33 : float  = (cos_phi * cos_theta) 


        row_one   : tuple = (r_11, r_12, r_13)
        row_two   : tuple = (r_21, r_22, r_23)
        row_three : tuple = (r_31, r_32, r_33)


        R = (row_one, row_two, row_three)


        return R 


    @staticmethod
    def derive_euler_angles(rotation_matrix: tuple) -> tuple:
        

        r_11 : float = rotation_matrix[0][0]
        r_12 : float = rotation_matrix[0][1]
        r_13 : float = rotation_matrix[0][2]

        r_21 : float = rotation_matrix[1][0]

        r_31 : float = rotation_matrix[2][0]
        r_32 : float = rotation_matrix[2][1]
        r_33 : float = rotation_matrix[2][2]

        if((r_31 != 1) & (r_31 != -1)):

            theta_one : float = -1 * asin(r_31)
            theta_two : float = (pi - theta_one)

            phi_one   : float = atan2((r_32 / cos(theta_one)), (r_33 / cos(theta_one)))
            phi_two   : float = atan2((r_32 / cos(theta_two)), (r_33 / cos(theta_two)))

            psi_one   : float = atan2((r_21 / cos(theta_one)), (r_11 / cos(theta_one)))
            psi_two   : float = atan2((r_21 / cos(theta_two)), (r_22 / cos(theta_two)))

        else:

            psi_one : float = 0
            psi_two : float = 0

            if(r_31 == -1):
                
                theta_one : float = (pi / 2)
                theta_two : float = theta_one

                phi_one   : float = psi_one + atan2(r_12, r_13)
                phi_two   : float = phi_one

            else:
                theta_one : float = -1 * (pi / 2)
                theta_two : float = theta_one

                phi_one   : float = (-1 * psi_one) + atan2(-1*r_12, -1*r_13)
                phi_two   : float = phi_one 



        euler_set_one : tuple = (phi_one, theta_one, psi_one)
        euler_set_two : tuple = (phi_two, theta_two, psi_two)


        full_set = (euler_set_one, euler_set_two)


        return full_set 
