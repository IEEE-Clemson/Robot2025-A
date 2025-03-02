


from machine import Pin, PWM, I2C # type: ignore 
from OldFiles.register_definitions import *
from newFiles.config_file import bmi270_config_file
from time import sleep




class BMI270:
    def __init__(self, serialDevice: I2C) -> None:
        self.i2c = serialDevice
        self.acc_range        = 2 * GRAVITY
        self.acc_odr          = 100
        self.gyr_range        = 1000
        self.gyr_odr          = 200

        self.load_config_file()
        self.set_acc_range(ACC_RANGE_2G)
        # self.set_acc_odr(ACC_ODR_100)
        self.set_gyr_range(GYR_RANGE_1000)
        # self.set_gyr_odr(GYR_ODR_200)
        self.set_acc_bwp(ACC_BWP_NORMAL)
        self.set_gyr_bwp(GYR_BWP_NORMAL)
        self.set_mode("performance")
        self.enable_acc()
        self.enable_gyr()
    
    def load_config_file(self) -> None:
        if (self.read_register(INTERNAL_STATUS) == 0x01):
            return
        else:
            self.write_register(PWR_CONF, 0x00)
            sleep(0.00045)
            self.write_register(INIT_CTRL, 0x00)
            for i in range(256):
                self.write_register(INIT_ADDR_0, 0x00)
                self.write_register(INIT_ADDR_1, i)
                self.write_i2c_block_data(I2C_PRIM_ADDR, INIT_DATA, bmi270_config_file[i*32:(i+1)*32])
                sleep(0.000020)
            self.write_register(INIT_CTRL, 0x01)
            sleep(0.02)



    def write_i2c_block_data(self, addr, register, data):
        """ Write multiple bytes of data to register of device at addr
            Returns None """
        # writeto_mem() expects something it can treat as a buffer
        if not isinstance(data, bytes):
            if not isinstance(data, list):
                data = [data]
            data = bytes(data)

        register_size = 8
        if isinstance(register, list):
            temp = 0
            register_size = 0
            for r in bytes(register):
                temp <<= 8
                temp |= r
                register_size += 8
            register = temp

        return self.i2c.writeto_mem(addr, register, data, addrsize=register_size)

    def read_register(self, address: int) -> int:
        currentRegister = self.i2c.readfrom_mem(I2C_PRIM_ADDR, address, 1)
        currentRegister = int.from_bytes(currentRegister, 'little')
        
        return currentRegister
        
    
    def write_register(self, address: int, value: int) -> None:
        buf = bytearray(int.to_bytes(value, 1, 'little'))
        self.i2c.writeto_mem(I2C_PRIM_ADDR, address, buf)
    
    def __unsignedToSigned__(self, n, byte_count) -> int:
        return int.from_bytes(n.to_bytes(byte_count, 'little', signed=False), 'little', signed=True)

    def __signedToUnsigned__(self, n, byte_count) -> int:
        return int.from_bytes(n.to_bytes(byte_count, 'little', signed=True), 'little', signed=False)

    def set_mode(self, mode="performance") -> None:
        if (mode == "low_power"):
            self.write_register(PWR_CTRL, 0x04)
            self.write_register(ACC_CONF, 0x17)
            self.write_register(GYR_CONF, 0x28)
            self.write_register(PWR_CONF, 0x03)
            self.acc_odr = 50
            self.gyr_odr = 100
        elif (mode == "normal"):
            self.write_register(PWR_CTRL, 0x0E)
            self.write_register(ACC_CONF, 0xA8)
            self.write_register(GYR_CONF, 0xA9)
            self.write_register(PWR_CONF, 0x02)
            self.acc_odr = 100
            self.gyr_odr = 200
        elif (mode == "performance"):
            self.write_register(PWR_CTRL, 0x0E)
            self.write_register(ACC_CONF, 0xA8)
            self.write_register(GYR_CONF, 0xE9)
            self.write_register(PWR_CONF, 0x02)
            self.acc_odr = 100
            self.gyr_odr = 200

    
    def enable_aux(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) | BIT_0))

    def disable_aux(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) & ~BIT_0))

    def enable_gyr(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) | BIT_1))

    def disable_gyr(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) & ~BIT_1))

    def enable_acc(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) | BIT_2))

    def disable_acc(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) & ~BIT_2))

    def enable_temp(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) | BIT_3))

    def disable_temp(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) & ~BIT_3))

    def enable_fifo_header(self) -> None:
        self.write_register(FIFO_CONFIG_1, (self.read_register(FIFO_CONFIG_1) | BIT_4))

    def disable_fifo_header(self) -> None:
        self.write_register(FIFO_CONFIG_1, (self.read_register(FIFO_CONFIG_1) & ~BIT_4))

    def enable_data_streaming(self) -> None:
        self.write_register(FIFO_CONFIG_1, (self.read_register(FIFO_CONFIG_1) | LAST_3_BITS))

    def disable_data_streaming(self) -> None:
        self.write_register(FIFO_CONFIG_1, (self.read_register(FIFO_CONFIG_1) & ~LAST_3_BITS))

    def enable_acc_filter_perf(self) -> None:
        self.write_register(ACC_CONF, (self.read_register(ACC_CONF) | BIT_7))

    def disable_acc_filter_perf(self) -> None:
        self.write_register(ACC_CONF, (self.read_register(ACC_CONF) & ~BIT_7))

    def enable_gyr_noise_perf(self) -> None:
        self.write_register(GYR_CONF, (self.read_register(GYR_CONF) | BIT_6))

    def disable_gyr_noise_perf(self) -> None:
        self.write_register(GYR_CONF, (self.read_register(GYR_CONF) & ~BIT_6))

    def enable_gyr_filter_perf(self) -> None:
        self.write_register(GYR_CONF, (self.read_register(GYR_CONF) | BIT_7))

    def disable_gyr_filter_perf(self) -> None:
        self.write_register(GYR_CONF, (self.read_register(GYR_CONF) & ~BIT_7))

    def set_acc_range(self, range=ACC_RANGE_2G) -> None:
        if (range == ACC_RANGE_2G):
            self.write_register(ACC_RANGE, ACC_RANGE_2G)
            self.acc_range = 2 * GRAVITY
        elif (range == ACC_RANGE_4G):
            self.write_register(ACC_RANGE, ACC_RANGE_4G)
            self.acc_range = 4 * GRAVITY
        elif (range == ACC_RANGE_8G):
            self.write_register(ACC_RANGE, ACC_RANGE_8G)
            self.acc_range = 8 * GRAVITY
        elif (range == ACC_RANGE_16G):
            self.write_register(ACC_RANGE, ACC_RANGE_16G)
            self.acc_range = 16 * GRAVITY

    def set_gyr_range(self, range=GYR_RANGE_2000) -> None:
        if (range == GYR_RANGE_2000):
            self.write_register(GYR_RANGE, GYR_RANGE_2000)
            self.gyr_range = 2000
        elif (range == GYR_RANGE_1000):
            self.write_register(GYR_RANGE, GYR_RANGE_1000)
            self.gyr_range = 1000
        elif (range == GYR_RANGE_500):
            self.write_register(GYR_RANGE, GYR_RANGE_500)
            self.gyr_range = 500
        elif (range == GYR_RANGE_250):
            self.write_register(GYR_RANGE, GYR_RANGE_250)
            self.gyr_range = 250
        elif (range == GYR_RANGE_125):
            self.write_register(GYR_RANGE, GYR_RANGE_125)
            self.gyr_range = 125

    def set_acc_odr(self, odr=ACC_ODR_200) -> None:
        if (odr == ACC_ODR_1600):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_1600))
            self.acc_odr = 1600
        elif (odr == ACC_ODR_800):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_800))
            self.acc_odr = 800
        elif (odr == ACC_ODR_400):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_400))
            self.acc_odr = 400
        elif (odr == ACC_ODR_200):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_200))
            self.acc_odr = 200
        elif (odr == ACC_ODR_100):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_100))
            self.acc_odr = 100
        elif (odr == ACC_ODR_50):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_50))
            self.acc_odr = 50
        elif (odr == ACC_ODR_25):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_25))
            self.acc_odr = 25

    def set_gyr_odr(self, odr=GYR_ODR_200) -> None:
        if (odr == GYR_ODR_3200):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_3200))
            self.gyr_odr = 3200
        elif (odr == GYR_ODR_1600):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_1600))
            self.gyr_odr = 1600
        elif (odr == GYR_ODR_800):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_800))
            self.gyr_odr = 800
        elif (odr == GYR_ODR_400):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_400))
            self.gyr_odr = 400
        elif (odr == GYR_ODR_200):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_200))
            self.gyr_odr = 200
        elif (odr == GYR_ODR_100):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_100))
            self.gyr_odr = 100
        elif (odr == GYR_ODR_50):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_50))
            self.gyr_odr = 50
        elif (odr == GYR_ODR_25):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_25))
            self.gyr_odr = 25

    def set_acc_bwp(self, bwp=ACC_BWP_NORMAL) -> None:
        if (bwp == ACC_BWP_OSR4):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_OSR4 << 4)))
        elif (bwp == ACC_BWP_OSR2):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_OSR2 << 4)))
        elif (bwp == ACC_BWP_NORMAL):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_NORMAL << 4)))
        elif (bwp == ACC_BWP_CIC):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_CIC << 4)))
        elif (bwp == ACC_BWP_RES16):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_RES16 << 4)))
        elif (bwp == ACC_BWP_RES32):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_RES32 << 4)))
        elif (bwp == ACC_BWP_RES64):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_RES64 << 4)))
        elif (bwp == ACC_BWP_RES128):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_RES128 << 4)))

    def set_gyr_bwp(self, bwp=GYR_BWP_NORMAL) -> None:
        if (bwp == GYR_BWP_OSR4):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & LSB_MASK_8BIT_8) | (GYR_BWP_OSR4 << 4)))
        elif (bwp == GYR_BWP_OSR2):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & LSB_MASK_8BIT_8) | (GYR_BWP_OSR2 << 4)))
        elif (bwp == GYR_BWP_NORMAL):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & LSB_MASK_8BIT_8) | (GYR_BWP_NORMAL << 4)))

    def get_sensor_time(self) -> int:
        sensortime_0 = self.read_register(SENSORTIME_0)
        sensortime_1 = self.read_register(SENSORTIME_1)
        sensortime_2 = self.read_register(SENSORTIME_2)

        return (sensortime_2 << 16) | (sensortime_1 << 8) | sensortime_0

    def get_raw_acc_data(self):
        acc_value_x_lsb = self.read_register(ACC_X_7_0)
        acc_value_x_msb = self.read_register(ACC_X_15_8)
        acc_value_x = (acc_value_x_msb << 8) | acc_value_x_lsb

        acc_value_y_lsb = self.read_register(ACC_Y_7_0)
        acc_value_y_msb = self.read_register(ACC_Y_15_8)
        acc_value_y = (acc_value_y_msb << 8) | acc_value_y_lsb

        acc_value_z_lsb = self.read_register(ACC_Z_7_0)
        acc_value_z_msb = self.read_register(ACC_Z_15_8)
        acc_value_z = (acc_value_z_msb << 8) | acc_value_z_lsb

        return [acc_value_x, acc_value_y, acc_value_z]

    def get_raw_gyr_data(self):
        gyr_value_x_lsb = self.read_register(GYR_X_7_0)
        gyr_value_x_msb = self.read_register(GYR_X_15_8)
        gyr_value_x = (gyr_value_x_msb << 8) | gyr_value_x_lsb

        gyr_value_y_lsb = self.read_register(GYR_Y_7_0)
        gyr_value_y_msb = self.read_register(GYR_Y_15_8)
        gyr_value_y = (gyr_value_y_msb << 8) | gyr_value_y_lsb

        gyr_value_z_lsb = self.read_register(GYR_Z_7_0)
        gyr_value_z_msb = self.read_register(GYR_Z_15_8)
        gyr_value_z = (gyr_value_z_msb << 8) | gyr_value_z_lsb

        return [gyr_value_x, gyr_value_y, gyr_value_z]
    
    def get_raw_temp_data(self) -> int:
        temp_value_lsb = self.read_register(TEMP_7_0)
        temp_value_msb = self.read_register(TEMP_15_8)
        temp_value = (temp_value_msb << 8) | temp_value_lsb

        return self.__unsignedToSigned__(temp_value, 2)
    
    def get_acc_data(self):
        raw_acc_data = self.get_raw_acc_data()
        for i in range(3):
            if (raw_acc_data[i] > 32767):
                raw_acc_data[i] -= 65536
        acceleration = [x / 32768 * self.acc_range for x in raw_acc_data]

        return acceleration

    def get_gyr_data(self):
        raw_gyr_data = self.get_raw_gyr_data()
        for i in range(3):
            if (raw_gyr_data[i] > 32767):
                raw_gyr_data[i] -= 65536
        # angular_velocity = [DEG2RAD * x / 32768 * self.gyr_range for x in raw_gyr_data]
        angular_velocity = [1.2 * (x / 32768) * self.gyr_range for x in raw_gyr_data]

        return angular_velocity
    
    def get_temp_data(self) -> float:
        raw_data = self.get_raw_temp_data()
        if (raw_data > 32767):
            raw_data -= 65536
        temp_celsius = raw_data * 0.001952594 + 23.0
        
        return temp_celsius