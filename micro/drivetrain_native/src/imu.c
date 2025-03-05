#include "imu.h"

#include "madgwick_filter.h"
#include "hardware/gpio.h"
#include "bno055.h"


#define GRAVITY_EARTH  (9.80665f)

#define ADDR           UINT8_C(0x28)
#define TIMEOUT_MS      UINT8_C(5000)

static int i2c_read_data_verbose(i2c_inst_t* i2c, uint8_t reg_addr, uint8_t *reg_data, uint32_t len, bool silent);
static int i2c_read_data(i2c_inst_t* i2c, uint8_t reg_addr, uint8_t *reg_data, uint32_t len);
static int i2c_write_data(i2c_inst_t* i2c, uint8_t reg_addr, const uint8_t *reg_data, uint32_t len);

static int i2c_read_data_verbose(i2c_inst_t* i2c, uint8_t reg_addr, uint8_t *reg_data, uint32_t len, bool silent)
{
    int res;
    absolute_time_t start, timeout;

    start = get_absolute_time();
    timeout = delayed_by_ms(start, TIMEOUT_MS);

    res = i2c_write_blocking_until(i2c, ADDR, &reg_addr, 1, true, timeout);
    if(res != 1) {
        if(!silent)
            printf("Write addr fail\n");

        return -2;
    }
    res = i2c_read_blocking_until(i2c, ADDR, reg_data, len, false, timeout);
    if(res != (int)len) {
        if(!silent)
            printf("Read data fail\n");
        return -2;
    }
    return 0;
}

static int i2c_read_data(i2c_inst_t* i2c, uint8_t reg_addr, uint8_t *reg_data, uint32_t len)
{
    return i2c_read_data_verbose(i2c, reg_addr, reg_data, len, false);
}

static int i2c_write_data(i2c_inst_t* i2c, uint8_t reg_addr, const uint8_t *reg_data, uint32_t len)
{
    int res;
    absolute_time_t start, timeout;
    uint32_t i;

    start = get_absolute_time();
    timeout = delayed_by_ms(start, TIMEOUT_MS);

    for(i = 0; i < len; i++) {
        start = get_absolute_time();
        timeout = delayed_by_ms(start, TIMEOUT_MS);
        res = i2c_write_burst_blocking(i2c, ADDR, &reg_addr, 1);
        res = i2c_write_blocking_until(i2c, ADDR, reg_data + i, 1, false, timeout);
        if(res != 1) {
            printf("Write data fail\n");
            return -2;
        }
        sleep_us(450);
    }
    return 0;
}

static int read_register(i2c_inst_t* i2c, uint8_t reg, uint8_t* val)
{
    int res;
    res = i2c_read_data(i2c, reg, val, 1);
    if(res < 0) {
        printf("Failed to read register\n");
        return -1;
    }
    return 0;
}

static int write_register(i2c_inst_t* i2c, uint8_t reg, uint8_t data)
{
    int res;
    res = i2c_write_data(i2c, reg, &data, 1);
    if(res < 0) {
        printf("Failed to write register\n");
        return -1;
    }
    return 0;
}


void bno_init(struct BNO055 *bno, i2c_inst_t *i2c, uint8_t sda, uint8_t scl)
{
    i2c_init(i2c, 400000);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);

    bno->i2c = i2c;

    write_register(bno->i2c, OPR_MODE, OPR_MODE_ACCGYRO); // Enable
    write_register(bno->i2c, UNIT_SEL, UNIT_SEL_RAD | UNIT_SEL_RADS | UNIT_SEL_MPS2);
}


void bno_get_raw_gyr_data(struct BNO055* bno, int16_t *gx, int16_t* gy, int16_t*gz)
{
    uint8_t gx1, gx0, gy1, gy0, gz1, gz0;
    read_register(bno->i2c, GYR_DATA_X_7_0, &gx0);
    read_register(bno->i2c, GYR_DATA_X_15_8, &gx1);
    *gx = ((uint16_t)gx1 << 8) | (uint16_t)gx0;


    read_register(bno->i2c, GYR_DATA_Y_7_0, &gy0);
    read_register(bno->i2c, GYR_DATA_Y_15_8, &gy1);
    *gy = ((uint16_t)gy1 << 8) | (uint16_t)gy0;

    read_register(bno->i2c, GYR_DATA_Z_7_0, &gz0);
    read_register(bno->i2c, GYR_DATA_Z_15_8, &gz1);
    *gz = ((uint16_t)gz1 << 8) | (uint16_t)gz0;
}

void bno_get_raw_acc_data(struct BNO055* bno, int16_t *ax, int16_t* ay, int16_t*az)
{
    uint8_t ax1, ax0, ay1, ay0, az1, az0;
    read_register(bno->i2c, ACC_DATA_X_7_0, &ax0);
    read_register(bno->i2c, ACC_DATA_X_15_8, &ax1);
    *ax = ((uint16_t)ax1 << 8) | (uint16_t)ax0;


    read_register(bno->i2c, ACC_DATA_Y_7_0, &ay0);
    read_register(bno->i2c, ACC_DATA_Y_15_8, &ay1);
    *ay = ((uint16_t)ay1 << 8) | (uint16_t)ay0;

    read_register(bno->i2c, ACC_DATA_Z_7_0, &az0);
    read_register(bno->i2c, ACC_DATA_Z_15_8, &az1);
    *az = ((uint16_t)az1 << 8) | (uint16_t)az0;
}

void bno_gyro_init(struct BNO055_GYRO *bno, i2c_inst_t *i2c, uint8_t sda, uint8_t scl)
{
    i2c_init(i2c, 400000);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);

    bno->i2c = i2c;

    write_register(bno->i2c, OPR_MODE, OPR_MODE_IMU); // Enable
    write_register(bno->i2c, UNIT_SEL, UNIT_SEL_RAD | UNIT_SEL_RADS | UNIT_SEL_MPS2);
}

int bno_gyro_get_euler_angles_raw(struct BNO055_GYRO* bno, int16_t *roll, int16_t* pitch, int16_t* yaw)
{
    uint8_t roll1, roll0, pitch1, pitch0, yaw1, yaw0;
    int res = 0;
    res |= read_register(bno->i2c, EUL_DATA_X_7_0, &roll0);
    res |= read_register(bno->i2c, EUL_DATA_X_15_8, &roll1);
    *roll = ((uint16_t)roll1 << 8) | (uint16_t)roll0;


    res |= read_register(bno->i2c, EUL_DATA_Y_7_0, &pitch0);
    res |= read_register(bno->i2c, EUL_DATA_Y_15_8, &pitch1);
    *pitch = ((uint16_t)pitch1 << 8) | (uint16_t)pitch0;

    res |= read_register(bno->i2c, EUL_DATA_Z_7_0, &yaw0);
    res |= read_register(bno->i2c, EUL_DATA_Z_15_8, &yaw1);
    *yaw = ((uint16_t)yaw1 << 8) | (uint16_t)yaw0;

    return res;
}

void bno_gyro_reset(struct BNO055_GYRO* bno)
{
    uint8_t data = 1;
    int res = -1;

    write_register(bno->i2c, SYS_TRIGGER, SYS_TRIGGER_RST_SYS);
    while(res != 0 || data != 0) 
        res = i2c_read_data_verbose(bno->i2c, SYS_STATUS, &data, 1, true);

    // Enter config mode to reset
    write_register(bno->i2c, OPR_MODE, OPR_MODE_CONFIGMODE);
    write_register(bno->i2c, UNIT_SEL, UNIT_SEL_RAD | UNIT_SEL_RADS | UNIT_SEL_MPS2);

    write_register(bno->i2c, OPR_MODE, OPR_MODE_IMU); // Enable
}
