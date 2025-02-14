


from machine import Pin, PWM, I2C # type: ignore 
from registerDefinitions import * # type: ignore 


class Gyroscope:
    def __init__(self, serialDevice: I2C) -> None:
        
        self.xData = 0
        self.yData = 0
        self.zData = 0 
        
        
        self.outputDataRate = 0
        self.bandwidthPreferences = 0
        
        
        self.OISRange = 0
        self.gyroRange = 0
        
        
        self.filterPerformance = 0
        self.noisePerformance = 0
        
        self.serialDevice = serialDevice 
        
        
        
        
        return None 
    
    
    @staticmethod
    def readRegister(registerAddress: int, serialDevice: I2C) -> int:
        """
        Simple utility function that returns the integer value of a read register value on the IMU.
        """
        
        currentRegister = serialDevice.readfrom_mem(I2C_PRIM_ADDR, registerAddress, 1)
        currentRegister = int.from_bytes(currentRegister)
        
        return currentRegister
    
    
    def writeRegister(self, registerAddress: int, valueWriting: int) -> None:
        """
        Simple utility function that writes 'valueWriting' value into the address of 'registerAddress' 
        """
        
        self.serialDevice.writeto_mem(I2C_PRIM_ADDR, registerAddress, valueWriting)        
        
        return None 
    
    
    def updateCoordinates(self, serialDevice: I2C) -> None:
        """
        Updates the coordinate attributes of the Accelerometer class by reading the MSB and LSB registers.\n
        NOTE:
        This function 'merges' the MSB and LSB of these coordinate registers together using bitwise operations as seen below.\n 
        """
        
        x_nibble_one = self.readRegister(registerAddress=GYR_X_7_0, serialDevice=serialDevice)
        x_nibble_two = self.readRegister(registerAddress=GYR_X_15_8, serialDevice=serialDevice)
        
        x_nibble_two <<= 8 
        x_nibble_two |= x_nibble_one 
        
        self.xData = x_nibble_two
        
        
        y_nibble_one = self.readRegister(registerAddress=GYR_Y_7_0, serialDevice=serialDevice)
        y_nibble_two = self.readRegister(registerAddress=GYR_X_15_8, serialDevice=serialDevice)
        
        y_nibble_two <<= 8 
        y_nibble_two |= y_nibble_one 

        self.yData = y_nibble_two
        
        z_nibble_one = self.readRegister(registerAddress=GYR_Z_7_0, serialDevice=serialDevice)
        z_nibble_two = self.readRegister(registerAddress=GYR_Z_15_8, serialDevice=serialDevice)
        
        z_nibble_two <<= 8 
        z_nibble_two |= z_nibble_one 
        
        self.zData = z_nibble_two 
        

        return None 
    
    
    def updateOutputDataRate(self, newRate: int) -> None:
        
        currentRegister = self.readRegister(registerAddress=GYR_CONF, serialDevice=self.serialDevice)
        currentRegister &= MSB_MASK_8BIT
        
        if((newRate >= GYR_ODR_25) & (newRate <= GYR_ODR_3200)):
            currentRegister |= newRate 
        else:
            return None 
        
        self.gyroRange = newRate 
        self.writeRegister(registerAddress=GYR_CONF, valueWriting=currentRegister, serialDevice=self.serialDevice)
        
            
        return None 
    
    def updateBandwidthPreferences(self, newBand: int) -> None:
        
        currentRegister = self.readRegister(registerAddress=GYR_CONF, serialDevice=self.serialDevice)
        currentRegister &= ~(BIT_5 | BIT_4)
        
        if((newBand >= GYR_BWP_OSR4) & (newBand <= GYR_BWP_NORMAL)):
            currentRegister |= (newBand << 4)
        else:
            return None
        
        
        
        
        self.bandwidthPreferences = newBand 
        self.writeRegister(registerAddress=GYR_CONF, valueWriting=currentRegister, serialDevice=self.serialDevice)
        
        
        
        return None 
    
    
    
    def updateNoisePerformance(self, newSetting: int) -> None:
        
        currentRegister = self.readRegister(registerAddress=GYR_CONF, serialDevice=self.serialDevice)
        currentRegister &= ~BIT_6 
        
        if((newSetting >= 0X00) & (newSetting <= 0x01)):
            currentRegister |= (newSetting << 6)
        else:
            return None 
        
        
        self.noisePerformance = newSetting
        self.writeRegister(registerAddress=GYR_CONF, valueWriting=currentRegister, serialDevice=self.serialDevice)
        
        
        return None 
    
    def updateFilterPerformance(self,  newSetting: int) -> None:
        
        currentRegister = self.readRegister(registerAddress=GYR_CONF, serialDevice=self.serialDevice)
        currentRegister &= ~BIT_7 
        
        if((newSetting >= 0x00) & (newSetting <+ 0x01)):
            currentRegister |= (newSetting << 7)
        else:
            return None 
        
        
        self.filterPerformance = newSetting
        self.writeRegister(registerAddress=GYR_CONF, valueWriting=currentRegister, serialDevice=self.serialDevice)
        
        
        return None 
    
    
    
    def updateGyroRange(self,  newRange: int) -> None:
        
        currentRegister = self.readRegister(registerAddress=GYR_RANGE, serialDevice=self.serialDevice)
        currentRegister &= ~FIRST_3_BITS
        
        if((newRange >= GYR_RANGE_2000) & (newRange <= GYR_RANGE_125)):
            currentRegister |= newRange 
        else:
            return None 
        
        
        
        self.gyroRange = newRange 
        self.writeRegister(registerAddress=GYR_RANGE, valueWriting=currentRegister, serialDevice=self.serialDevice)    
            
        
        
        return None 
    
    
    def updateOISRange(self, newRange: int ) -> None:
        
        currentRegister = self.readRegister(registerAddress=GYR_RANGE, serialDevice=self.serialDevice)
        
        if(newRange == 0X00):
            currentRegister &= ~BIT_3 
        elif(newRange == 0x01):
            currentRegister |= BIT_3
        else:
            return None 
        
        
        
        self.OISRange = currentRegister
        self.writeRegister(registerAddress=GYR_RANGE, valueWriting=currentRegister, serialDevice=self.serialDevice)

            
        
        
        return None 