#include "imu.h"

#include "madgwick_filter.h"
#include "bmi270.h"
#include "hardware/gpio.h"

#define GRAVITY_EARTH  (9.80665f)

#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)
/*!
 * @brief This internal API is used to set configurations for accel and gyro.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 2, bmi);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_200HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel and gyro configurations. */
        rslt = bmi2_set_sensor_config(config, 2, bmi);
    }

    return rslt;
}

static BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    struct IMU *imu = (struct IMU *)intf_ptr;

    i2c_read_blocking(imu->i2c, reg_addr, reg_data, len, false);
    return BMI2_INTF_RET_SUCCESS;
}

static BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    struct IMU *imu = (struct IMU *)intf_ptr;

    i2c_write_blocking(imu->i2c, reg_addr, reg_data, len, false);
    return BMI2_INTF_RET_SUCCESS;
}


/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

void imu_init(struct IMU *imu, i2c_inst_t *i2c, uint8_t sda, uint8_t scl)
{
    int res;
    uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    struct quaternion unit_quat = {1, 0, 0, 0};
    
    // Init i2c
    i2c_init(i2c, 400000);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);

    // Init gyro
    imu->bmi.intf = BMI2_I2C_INTF;
    imu->bmi.write = bmi2_i2c_write;
    imu->bmi.read = bmi2_i2c_read;
    imu->bmi.intf_ptr = imu;

    bmi270_init(&imu->bmi);
    set_accel_gyro_config(&imu->bmi);
    bmi2_sensor_enable(sensor_list, 2, &imu->bmi);

    // Init rotation as unit quaternion
    imu->q = unit_quat;
}

void imu_update(struct IMU *imu, float dt)
{
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    struct bmi2_sens_data sensor_data = { { 0 } };
    uint8_t resolution;

    bmi2_get_sensor_data(&sensor_data, &imu->bmi);
    resolution = imu->bmi.resolution;
    
    /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
    ax = lsb_to_mps2(sensor_data.acc.x, (float)2, resolution);
    ay = lsb_to_mps2(sensor_data.acc.y, (float)2, resolution);
    az = lsb_to_mps2(sensor_data.acc.z, (float)2, resolution);

    /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
    gx = lsb_to_dps(sensor_data.gyr.x, (float)2000, resolution);
    gy = lsb_to_dps(sensor_data.gyr.y, (float)2000, resolution);
    gz = lsb_to_dps(sensor_data.gyr.z, (float)2000, resolution);

    imu_filter(&imu->q, dt, ax, ay, az, gx, gy, gz);
}

float imu_get_z_radians(struct IMU *imu) {
    struct quaternion q = imu->q;
    float roll, pitch, yaw;
    eulerAngles(q, &roll, &pitch, &yaw);
    return yaw;
}