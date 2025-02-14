from machine import Pin, PWM, I2C # type: ignore 
from registerDefinitions import * # type: ignore 




class Accelerometer:
    def __init__(self, serialDevice: I2C) -> None:
        
        
        self.xData = 0
        self.yData = 0
        self.zData = 0
        
        self.serialDevice = serialDevice

        
        self.readingRange = 0 
        '''
        This is the range of the acceleromter's output. (+/- n*gravity)\n
        Possible options are:\n
        0x00: +/- 2g\n
        0x01: +/- 4g\n
        0x02: +/- 8g\n
        0x03: +/- 16g\n
        '''
        self.bandwidthParameter = 0
        '''
        The bandwith / resolution of the IMU's readings.\n 
        NOTE: The sampling rate and averaging of the signals is determined ALSO by the filterPreference variable!\n
        '''
        
        
        self.outputDataRate = 0
        '''
        The hertz in the rate of which new data travels down the serial bus.\n
        NOTE: The ODR is indepedent of the power mode of the sensor!\n
        '''
        
        
        self.filterPreference = 0
        '''
        Determines the power consumption for the accelerometer (uA consumed).
        0x00 is powersaving mode 
        0x01 is performance mode 
        '''
        
                
        return None 
    
    @staticmethod
    def fetchRegister(registerInt: int) -> int:
        """
        Simple utility function that returns the integer value of a read register value on the IMU.
        """
        
        currentRegister = self.serialDevice.readfrom_mem(I2C_PRIM_ADDR, registerInt, 1)
        currentRegister = int.from_bytes(currentRegister)
        
        return currentRegister
    

    def updateCoordinates(self) -> None:
        """
        Updates the coordinate attributes of the Accelerometer class by reading the MSB and LSB registers.\n
        NOTE:
        This function 'merges' the MSB and LSB of these coordinate registers together using bitwise operations as seen below.\n 
        """
        
    
        x_nibble_one = self.fetchRegister(registerInt=ACC_X_7_0)
        x_nibble_two = self.fetchRegister(registerInt=ACC_X_15_8)
        
        x_nibble_two <<= 8 
        x_nibble_two |= x_nibble_one 

        self.xData = x_nibble_two

        
        y_nibble_one = self.fetchRegister(registerInt=ACC_Y_7_0)
        y_nibble_two = self.fetchRegister(registerInt=ACC_Y_15_8)
        
        y_nibble_two <<= 8 
        y_nibble_two |= y_nibble_one 
        
  
        self.yData = y_nibble_two 
        
        
        z_nibble_one = self.fetchRegister(registerInt=ACC_Z_7_0)
        z_nibble_two = self.fetchRegister(registerInt=ACC_Z_15_8) 
       
        z_nibble_two <<= 8
        z_nibble_two |= z_nibble_one
        
        
        self.zData = z_nibble_two
        
        
                
        return None 
    
   
    def updateBandwidthParameter(self, newBandwidth: int) -> None:
        """
        Updates the sampling method / bandwidth paramter for the IMU. Note that for each register, the filter performance mode DOES make a
        difference -- not only in the sampling methods themselves, but also in which registers are allowed to be accessed / configured by software, too.\n
        Hopefully I've made how this works out somewhat clear below. I also would *highly* reccommend looking at the datasheet for any foreign
        terminology.
        
        """
        
        currentRegister = self.fetchRegister(ACC_CONF)
        currentRegister &= 0x8F

        
        if(newBandwidth == 0x00):
            currentRegister |= (ACC_BWP_OSR4 << 4) # OSR4 Sampling Method if performance mode ON || No averaging if OFF
        elif(newBandwidth == 0x01):
            currentRegister |= (ACC_BWP_OSR2 << 4) # OSR2 Sampling Method if performance mode ON || Average 2 samples if OFF
        elif(newBandwidth == 0x02):
            currentRegister |= (ACC_BWP_NORMAL << 4) # "Normal" Sampling Method if performance mode ON || Average 4 samples if OFF 
        elif(newBandwidth == 0x03):
            currentRegister |= (ACC_BWP_CIC << 4) # "CIC" Sampling Method if performance mode ON || Average 8 samples if OFF 
        elif((newBandwidth >= 0x04) & (newBandwidth <= 0x07)): 
            
            if(self.filterPreference != 0x00): # Checks if performance mode is ON
                return None # If performance mode is OFF, these next four are reserved and should NOT be manipulated by software.
            
            else:
                
                if(newBandwidth == 0x04):
                    currentRegister |= (ACC_BWP_RES16 << 4) # Average of last 16 samples
                elif(newBandwidth == 0x05):
                    currentRegister |= (ACC_BWP_RES32 << 4) # Average of last 32 samples
                elif(newBandwidth == 0x06):
                    currentRegister |= (ACC_BWP_RES64 << 4) # Average of last 64 samples
                elif(newBandwidth == 0x07):
                    currentRegister |= (ACC_BWP_CIC << 4 ) # Average of last 128 samples
                    
        else:
            return None # If invalid option is chosen, we make no changes to the hardware.
        
        
        self.serialDevice.writeto_mem(I2C_PRIM_ADDR, ACC_CONF, currentRegister) # If valid entry selected, we write new data.
        
        
    
        return None 
    
    
    def updateODR(self, newRate: int) -> None:
        """
        Updates the output data rate (ODR) of the accelerometer. The relative Hz next to each command are commented for clarification. 
        """
                
        currentRegister = self.fetchRegister(ACC_CONF)
        currentRegister &= MSB_MASK_8BIT
        
        if(newRate == 0x0C):
            currentRegister |= ACC_ODR_1600 # 1600 Hz
        elif(newRate == 0x0B):
            currentRegister |= ACC_ODR_800 # 800 Hz
        elif(newRate == 0X0A):
            currentRegister |= ACC_ODR_400 # 400 Hz
        elif(newRate == 0X09):
            currentRegister |= ACC_ODR_200 # 200 Hz
        elif(newRate == 0x08):
            currentRegister |= ACC_ODR_100 # 100 Hz
        elif(newRate == 0x07):
            currentRegister |= ACC_ODR_50 # 50 Hz
        elif(newRate == 0x06):
            currentRegister |= ACC_ODR_25 # 25 Hz 
        elif(newRate == 0x05):
            currentRegister |= ACC_ODR_12P5 # 12.5Hz
        elif(newRate == 0x04):
            currentRegister |= ACC_ODR_6P25 # 6.5 Hz
        elif(newRate == 0x03):
            currentRegister |= ACC_ODR_3P1 # 3.1 Hz 
        elif(newRate == 0x02):
            currentRegister |= ACC_ODR_1P5 # 1.5 Hz
        elif(newRate == 0x01):
            currentRegister |= ACC_ODR_0P78 # 0.78 Hz
        else:
            return None # Prevents from writing to reserved registers
        
        
        self.outputDataRate = newRate 
        self.serialDevice.writeto_mem(I2C_PRIM_ADDR, ACC_CONF, currentRegister) # Writes new data if valid entry
        
        
        return None 
    
    
    def updateRange(self, newRange: int) -> None:
        """
        Updates the reading range of the device to a new value 'n' wherein the range is from -n -> +n. It's relatively
        self-explanatory.
        
        """
        
        currentRegister = self.fetchRegister(ACC_RANGE)
        currentRegister &= ~FIRST_2_BITS
        
        if(newRange == 0x00):
            currentRegister |= ACC_RANGE_2G # +/- 2g
        elif(newRange == 0x01):
            currentRegister |= ACC_RANGE_4G # +/- 4g
        elif(newRange == 0x02):
            currentRegister |= ACC_RANGE_8G # +/- 8g
        elif(newRange == 0x03):
            currentRegister |= ACC_RANGE_16G # +/- 16g
        else:
            return None 
        
        self.readingRange = newRange 
        self.serialDevice.writeto_mem(I2C_PRIM_ADDR, ACC_RANGE, currentRegister)
        
    
        return None 
    
    
    def updateFilterMode(self, newFilter: int) -> None:
        """
        Edits the bandwidth filtering-mode for the accelerometer. As mentioned before, this DOES have some serious affects to the nature of
        the sampling methods for the accelerometer data, so please be careful when using this!
        """
        
        currentRegister = self.fetchRegister(ACC_CONF)
        
        if(newFilter == 0x00): # Power-saving mode 
            currentRegister |= BIT_6
        elif(newFilter == 0x01): # Performance mode
            currentRegister &= ~BIT_6 
        else:
            return None 
        
        self.filterPreference = newFilter 
        self.serialDevice.writeto_mem(I2C_PRIM_ADDR, ACC_RANGE, currentRegister)
        
        
        return None 
