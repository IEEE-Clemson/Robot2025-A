from machine import Pin, PWM, I2C # type: ignore 
from registerDefinitions import * # type: ignore 




class PWR_Configure:
    def __init__(self, serial_device: I2C) -> None:
        
        
        self.adv_power_save = 0x00
        self.fifo_self_wake_up =0x00
        self.fup_en = 0x00
    
        
        
        self.serial_device = serial_device
        
        return None
    
    def read_regsiter(self) -> int:
        
        current_val = self.serial_device.readfrom_mem(I2C_PRIM_ADDR, PWR_CONF, 1)
        current_val = int.from_bytes(current_val)
        
        
        return current_val
    
    
    def adv_power_toggle(self, value: int) -> None:
        
        current_register = self.read_regsiter()
        
        if(value == 0x00):
            current_register &= ~BIT_0
        elif(value == 0x01):
            current_register |= BIT_0 
        else:
            return None 
        
        
        self.adv_power_save = value 
        self.serial_device.writeto_mem(I2C_PRIM_ADDR, PWR_CONF, current_register)
        
        
        return None 

    def fifo_toggle(self, value: int) -> None:
        
        current_register = self.read_regsiter()
        if(value == 0x00):
            current_register &= ~BIT_1 
        elif(value == 0x01):
            current_register |= BIT_1 
        else:
            return None 
        
        
        self.fifo_self_wake_up = value 
        self.serial_device.writeto_mem(I2C_PRIM_ADDR, PWR_CONF, current_register)
        
        
        return None 
    
    
    
    def fup_toggle(self, value: int) -> None:
        
        current_register = self.read_regsiter()
        if(value == 0x00):
            current_register &= ~BIT_2 
        elif(value == 0x01):
            current_register |= BIT_2 
        else:
            return None 
        
        
        self.fup_en = value 
        self.serial_device.writeto_mem(I2C_PRIM_ADDR, PWR_CONF, current_register)
        
        
        
        
        return None 


class PWR_Control:
    def __init__(self, serial_device: I2C) -> None:
        
        
        self.aux_en = 0x00
        self.gyro_en = 0x00
        self.acc_en = 0x00
        self.temp_en = 0x00
        
        
        self.serial_device = serial_device 
        
        
        
        return None 
    
    def read_regsiter(self, address: int) -> int:
        
        current_val = self.serial_device.readfrom_mem(I2C_PRIM_ADDR, address, 1)
        current_val = int.from_bytes(current_val)
        
        
        return current_val
    
    
    def auxillary_enable(self, value: int) -> None:
        
        current_register = self.read_regsiter(PWR_CTRL)
        
        if(value == 0x00):
            current_register &= ~BIT_0
            
        elif(value == 0x01):
            current_register |= BIT_0
            
        else:
            return None 
        
        
        
        self.aux_en = value 
        self.serial_device.writeto_mem(I2C_PRIM_ADDR, PWR_CTRL, current_register)
        
        return None 
    
    
    def gyroscope_enable(self, value: int) -> None:
        
        current_register = self.read_regsiter(PWR_CTRL)
        
        if(value == 0x00):
            current_register &= ~BIT_1
        elif(value == 0x01):
            current_register |= BIT_1 
        else:
            return None 
        
        self.gyro_en = value 
        self.serial_device.writeto_mem(I2C_PRIM_ADDR, PWR_CTRL, current_register)
        
        return None 
    
    
    def accel_enable(self, value: int) -> None:
        
        current_register = self.read_regsiter(PWR_CTRL)
        
        if(value == 0x00):
            current_register &= ~BIT_2 
        elif(value == 0x01):
            current_register |= BIT_2 
        else:
            return None 
        
        
        
        self.acc_en = value 
        self.serial_device.writeto_mem(I2C_PRIM_ADDR, PWR_CTRL, current_register)
        
        
        return None 
    
    
    def temperature_enable(self, value: int) -> None:
        
        current_register = self.read_regsiter(PWR_CTRL)
        if(value == 0x00):
            current_register &= ~BIT_3
        elif(value == 0x01):
            current_register |= BIT_3 
        else:
            return None 
        
        
        
        self.temp_en = value 
        self.serial_device.writeto_mem(I2C_PRIM_ADDR, PWR_CTRL, current_register)
        
        
        return None 
    
    
class Command_Register:
    def __init__(self, serial_device: I2C) -> None:
        
        self.serial_device = serial_device 
        
        return None 
    
    
    def g_trigger(self) -> None:
        
        self.serial_device.writeto_mem(I2C_PRIM_ADDR, CMD, 0x02)
        
        return None 
    
    
    def usr_gain(self) -> None:
        
        self.serial_device.writeto_mem(I2C_PRIM_ADDR, CMD, 0x03)
        
        return None 
    
    
    def nvm_prog(self) -> None:
        
        self.serial_device.writeto_mem(I2C_PRIM_ADDR, CMD, 0xa0)
        
        return None 
    
    def fifo_flush(self) -> None:
        
        self.serial_device.writeto_mem(I2C_PRIM_ADDR, CMD, 0xb0)
        
        return None 
    
    
    def softreset(self) -> None:
        
        self.serial_device.writeto_mem(I2C_PRIM_ADDR, CMD, 0xb6)
        
        return None 
