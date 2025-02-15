from machine import Pin, PWM, I2C # type: ignore 
from registerDefinitions import * # type: ignore 



class Gyroscope:
    def __init__(self, serial_device: I2C) -> None:
        
        
        self.x_data = 0
        self.y_data = 0
        self.z_data= 0
        
        
        self.output_data_rate = 0
        self.bandwidth_preference = 0
        self.noise_setting = 0
        self.filter_setting = 0
        
        
        self.ois_range = 0
        self.gyro_range = 0    
        
        
        self.serial_device = serial_device 
        
        
        
        return None 
    
    
    def read_register(self, register_address: int) -> int:
        """
        Simple utility function that reads registers. Plan to add more 'safety' features in the future to make this more useful, but 
        for now it remains a humble wrapper for a a micropython class method.
        """
        
        read_register = self.serial_device.readfrom_mem(I2C_PRIM_ADDR, register_address, 1)
        
        read_register = int.from_bytes(read_register)
    
        return read_register
    
    
        
    
    
    def updateCoordinates(self) -> None:
        """
        Function that updates the read coordinate data of the gyroscope's positional registers. For each coordinate plane I use bit-wise operations 
        to 'merge' the upper and lower bytes together. 
        """
        
        x_nibble_one = self.read_register(register_address=GYR_X_7_0)
        x_nibble_two = self.read_register(register_address=GYR_X_15_8)
        
        x_nibble_two = (x_nibble_two << 8) | (x_nibble_one)
        
        self.x_data = x_nibble_two
        
        
        y_nibble_one = self.read_register(register_address=GYR_Y_7_0)
        y_nibble_two = self.read_register(register_address=GYR_Y_15_8)
        
        y_nibble_two = (y_nibble_two << 8) | (y_nibble_one)
        
        self.y_data = y_nibble_two
        
        
        
        z_nibble_one = self.read_register(register_address=GYR_Z_7_0)
        z_nibble_two = self.read_register(register_address=GYR_Z_15_8)
        
        z_nibble_two = (z_nibble_two << 8) | (z_nibble_one)
        
        self.z_data = z_nibble_two
        
        
    
        return None 
    
    
    def update_odr(self, new_odr: int) -> None:
        """
        Function that changes the output data rate (ODR) of the gyroscope's associated register(s).
        """
        
        old_register = self.read_register(register_address=GYR_CONF)
        old_register &= MSB_MASK_8BIT
        
        all_odr_options = {GYR_ODR_3200, GYR_ODR_1600, GYR_ODR_800, GYR_ODR_400, GYR_ODR_200, GYR_ODR_100, GYR_ODR_50, GYR_ODR_25}
        
        if(new_odr in all_odr_options):
            old_register |= new_odr 
        else:
            return None 
        
        self.output_data_rate = new_odr 
        self.serial_device.writeto_mem(addr=I2C_PRIM_ADDR, memaddr=GYR_CONF, buf=old_register, addrsize=8)
        
        
        return None 
    
    
    def update_bandwidth(self, new_band: int) -> None:
        
        """
        Function that updates the 
        
        """
        
        old_register = self.read_register(register_address=GYR_CONF)
        old_register &= ~(BIT_5 | BIT_4)
        
        
        if(new_band == GYR_BWP_OSR4):
            old_register |= (GYR_BWP_OSR4 << 3)
        elif(new_band == GYR_BWP_OSR2):
            old_register |= (GYR_BWP_OSR2 << 3)
        elif(new_band == GYR_BWP_NORMAL):
            old_register |= (GYR_BWP_NORMAL << 3)
        else:
            return None 
        
        
        self.bandwidth_preference = new_band 
        self.serial_device.writeto_mem(addr=I2C_PRIM_ADDR, memaddr=GYR_CONF, buf=old_register, addrsize=8)
        
        
        return None 
    
    
    def update_noise_preference(self, new_noise: int) -> None:
        """
        Function that edits the performance mode of the gryoscope's noise function(s).\n
        Power Optimized: 0x00\n
        Performance Optimized: 0x01\n
        """
        
        old_register = self.read_register(register_address=GYR_CONF)
        old_register &= ~BIT_6
        
        if((new_noise >= 0x00) & (new_noise <= 0x01)):
            old_register |= (new_noise << 6)
        else:
            return None 
        
        
        self.noise_setting = new_noise 
        self.serial_device.writeto_mem(addr=I2C_PRIM_ADDR, memaddr=GYR_CONF, buf=old_register, addrsize=8)
        
        
        return None 
    
    
    def update_filter_preference(self, new_filter: int) -> None:
        """
        Function that edits the performance mode of the gyroscope's filter.\n
        Power Optimized: 0x00\n 
        Performance Optimized: 0x01\n
        """
        
        old_register = self.read_register(register_address=GYR_CONF)
        old_register &= ~BIT_7 
        
        if((new_filter >= 0x00) & (new_filter <= 0x01)):
            old_register |= (new_filter << 7)
        else:
            return None 
    
        self.filter_setting = new_filter 
        self.serial_device.writeto_mem(addr=I2C_PRIM_ADDR, memaddr=GYR_CONF, buf=old_register, addrsize=8)
        
        
        return None 
    
    
    def update_gyr_range(self, new_range: int) -> None:
        """
        Function that updates the gyroscope's angular rate measurement range. 
        """

        old_register = self.read_register(register_address=GYR_RANGE)
        old_register &= ~(BIT_2 | BIT_1 | BIT_0) 
        
        
        all_gyr_range_options = {GYR_RANGE_2000, GYR_RANGE_1000, GYR_RANGE_500, GYR_RANGE_250, GYR_RANGE_125}
        
        
        if(new_range in all_gyr_range_options):
            old_register |= new_range
        else:
            return None 
  
        
        self.gyro_range = new_range
        self.serial_device.writeto_mem(addr=I2C_PRIM_ADDR, memaddr=GYR_RANGE, buf=old_register, addrsize=8)
    
        
        return None 
    
    
    
    def update_ois_range(self, new_ois: int) -> None:
        """
        Function that updates the OIS angular rate measurement range. 
        """
        
        old_register = self.read_register(register_address=GYR_RANGE)
        old_register &= ~BIT_4 
        
        if((new_ois >= 0x00) & (new_ois <= 0x01)):
            old_register |= (new_ois << 3)
        else:
            return None 
        
        
        self.ois_range = new_ois 
        self.serial_device.writeto_mem(addr=I2C_PRIM_ADDR, memaddr=GYR_RANGE, buf=old_register, addrsize=8)
        
        
        return None 
    
    
    
