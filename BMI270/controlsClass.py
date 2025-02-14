

from machine import Pin, PWM, I2C # type: ignore 
from registerDefinitions import * # type: ignore 




class PowerControl:
    def __init__(self, serialDevice: I2C) -> None:
        
        
        self.auxStatus = 0
        self.gyroStatus = 0
        self.accelStatus = 0
        self.tempStatus = 0
        
        self.serialDevice = serialDevice
        
        
        return None 
    

    def readRegister(self, registerAddress: int) -> int:
        """
        Simple utility function that returns the integer value of a read register value on the IMU.
        """
        
        currentRegister = self.serialDevice.readfrom_mem(I2C_PRIM_ADDR, registerAddress, 1)
        currentRegister = int.from_bytes(currentRegister)
        
        return currentRegister
    
    
    
    def auxToggle(self) -> None:
        """
        Toggles whether the auxillary sensor is enabled ON/OFF
        """
        currentRegister = self.readRegister(registerAddress=PWR_CTRL, serialDevice=self.serialDevice)
                
        if(self.auxStatus == 1):
            self.auxStatus = 0
            currentRegister &= ~BIT_0
        else:
            self.auxStatus = 1 
            currentRegister |= BIT_0 
            
        
        self.serialDevice.writeto_mem(I2C_PRIM_ADDR, PWR_CTRL, currentRegister)
        
        
        return None 
    
    
    def gyroToggle(self) -> None:
        """
        Toggles whether the gyroscope is enabled ON/OFF
        """
        currentRegister = self.readRegister(registerAddress=PWR_CTRL, serialDevice=self.serialDevice)
        
        if(self.gyroStatus == 1):
            self.gyroStatus = 0
            currentRegister &= ~BIT_1
        else:
            self.gyroStatus = 1 
            currentRegister |= BIT_1 
            
        self.serialDevice.writeto_mem(I2C_PRIM_ADDR, PWR_CTRL, currentRegister)
        
        
        return None 
    
    
    def accelToggle(self) -> None:
        """
        Toggles whether the acceleration is enabled ON/OFF
        """
        
        currentRegister = self.readRegister(registerAddress=PWR_CTRL, serialDevice=self.serialDevice)
        
        if(self.accelStatus == 1):
            self.accelStatus = 0
            currentRegister &= ~BIT_2 
        else:
            self.accelStatus = 1 
            currentRegister |= BIT_2
        
                
        self.serialDevice.writeto_mem(I2C_PRIM_ADDR, PWR_CTRL, currentRegister)
        
        
        return None 
    
    
    def tempToggle(self) -> None:
        """
        Toggles whether the temperature sensor is enabled ON/OFF
        """
        
        currentRegister = self.readRegister(registerAddress=PWR_CTRL, serialDevice=self.serialDevice)
        
        if(self.tempToggle == 1):
            self.tempStatus = 0 
            currentRegister &= ~BIT_3 
        else:
            self.tempStatus = 1 
            currentRegister |= BIT_3 
        

        self.serialDevice.writeto_mem(I2C_PRIM_ADDR, PWR_CTRL, currentRegister)
        
        return None 
    
    
    
