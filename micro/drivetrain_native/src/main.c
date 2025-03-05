#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/critical_section.h"
#ifdef PICO_W
#include "pico/cyw43_arch.h"
#endif

#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico/i2c_slave.h"

#include "config.h"
#include "pi_motor.h"
#include "pid_control.h"
#include "imu.h"
#include "comm.h"
#include "madgwick_filter.h"
#include <stdlib.h>

// Buff written to by i2c
static volatile uint8_t i2c_mem[BUFF_SIZE];
static volatile uint8_t i2c_mem_addr;
static volatile bool i2c_addr_written;

static int16_t roll_raw, pitch_raw, yaw_raw;
static float theta_filtered;

// Layout for usage in program
static volatile struct I2CMemLayout *mem = (volatile struct I2CMemLayout*)(i2c_mem);

static struct PIDParams pid_params;
static struct PIMotorMod10A motor_fl, motor_fr, motor_bl, motor_br;
static struct BNO055_GYRO imu;

static void __not_in_flash_func(i2c_slave_handler)(i2c_inst_t *i2c, i2c_slave_event_t event) {
    uint8_t byte;
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        if (!i2c_addr_written) {
            // writes always start with the memory address
            i2c_mem_addr = i2c_read_byte_raw(i2c);
            i2c_addr_written = true;
        } else {
            // save into memory
            byte = i2c_read_byte_raw(i2c);
            if(i2c_mem_addr < BUFF_SIZE)
                i2c_mem[i2c_mem_addr] = byte;
            i2c_mem_addr++;
        }
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        // load from memory
        if(i2c_mem_addr < BUFF_SIZE)
            i2c_write_byte_raw(i2c, i2c_mem[i2c_mem_addr]);
        else
            i2c_write_byte_raw(i2c, 0);
        i2c_mem_addr++;
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        i2c_addr_written = false;
        break;
    default:
        break;
    }
}

static void setup_slave() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}


static void init_motors() {
    encoder_init_pio(pio0);
    pid_create_params(&pid_params, 0.1f, 0.1f, 0.0f);
    pi_motor_mod10a_init(&motor_fl, &pid_params, pio0, 0, FL_PIN_A, FL_PIN_B, FL_PIN_DIR, FL_PIN_PWM, FL_INVERTED);
    pi_motor_mod10a_init(&motor_fr, &pid_params, pio0, 1, FR_PIN_A, FR_PIN_B, FR_PIN_DIR, FR_PIN_PWM, FR_INVERTED);
    pi_motor_mod10a_init(&motor_bl, &pid_params, pio0, 2, BL_PIN_A, BL_PIN_B, BL_PIN_DIR, BL_PIN_PWM, BL_INVERTED);
    pi_motor_mod10a_init(&motor_br, &pid_params, pio0, 3, BR_PIN_A, BR_PIN_B, BR_PIN_DIR, BR_PIN_PWM, BR_INVERTED);
}

static void update_local_vel() {
    float w_fl, w_fr, w_bl, w_br;
    float a, b;
    float local_vx, local_vy, omega;

    w_fl = motor_fl.cur_vel;
    w_fr = motor_fr.cur_vel;
    w_bl = motor_bl.cur_vel;
    w_br = motor_br.cur_vel;

    a = WHEEL_RADIUS / 4.0f;
    b = WHEEL_RADIUS / (4.0f * (WHEEL_DIST_X + WHEEL_DIST_Y));

    local_vx = (w_fl + w_fr + w_bl + w_br) * a;
    local_vy = (-w_fl + w_fr + w_bl - w_br) * a;
    omega = (-w_fl + w_fr - w_bl + w_br) * b;

    mem->cur_vx = (int16_t)(local_vx / MAX_VXY * INT16_MAX);
    mem->cur_vy = (int16_t)(local_vy / MAX_VXY * INT16_MAX);
    mem->cur_omega = (int16_t)(omega / MAX_OMEGA * INT16_MAX);
}

static void update_target_vel() {
    float vx, vy, omega;
    float a, b;

    vx = (float)(mem->target_vx) * MAX_VXY / INT16_MAX;
    vy = (float)(mem->target_vy) * MAX_VXY / INT16_MAX;
    omega = (float)(mem->target_omega) * MAX_OMEGA / INT16_MAX;
    
    a = 1 / WHEEL_RADIUS;
    b = (WHEEL_DIST_X + WHEEL_DIST_Y) * omega;

    motor_fl.setpoint = a * (vx - vy - b);
    motor_fr.setpoint = a * (vx + vy + b);
    motor_bl.setpoint = a * (vx + vy - b);
    motor_br.setpoint = a * (vx - vy + b);
}

static const int LED_PIN_PICO = 25;
int main() {
    float dt;
    absolute_time_t time, time_to_sleep;
    int i = 0;
    bool led = false;
    int count;

    // Disable pause on debug when sleeping
    timer_hw->dbgpause = 0;

    stdio_init_all();
    if (watchdog_enable_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        return 0;
    }
    watchdog_enable(100, 1);
    
    // Unfortunately, we have to load the entire network stack to use LED on pico W
#ifdef PICO_W
    if(cyw43_arch_init()) {
        printf("Wi-Fi init failed");
    }
#else
    gpio_init(LED_PIN_PICO);
    gpio_set_dir(LED_PIN_PICO, true);
#endif

    
    setup_slave();
    init_motors();
    bno_gyro_init(&imu, IMU_I2C_INST, IMU_SDA_PIN, IMU_SCL_PIN);
    bno_gyro_reset(&imu, 0, 0, 0);
    watchdog_update();
    
    dt = 1.0f / FREQ;
    
    float avgx = 0, avgy = 0, avgz = 0;
    int count = 1;
    while(true) {
        time = get_absolute_time();
        time_to_sleep = delayed_by_ms(time, 1000 / FREQ);
        pi_motor_mod10a_update(&motor_fl, dt);
        pi_motor_mod10a_update(&motor_fr, dt);
        pi_motor_mod10a_update(&motor_bl, dt);
        pi_motor_mod10a_update(&motor_br, dt);
        
        bno_gyro_get_euler_angles_raw(&imu, &roll_raw, &pitch_raw, &yaw_raw);
        
        update_local_vel();
        update_target_vel();
        mem->theta = roll_raw;
        
        busy_wait_until(time_to_sleep);

        // Blink led for heartbeat
        i = (i + 1) % 100;
        if(i == 0) {
            // This is a no-op if usb is disconnected
            printf("Rotation: %f %f %f\n", roll_raw * REG2RAD, pitch_raw * REG2RAD, yaw_raw * REG2RAD);    
            printf("Filtered theta: %f\n", theta_filtered);    
            printf("Encoder Count: %ld %ld %ld %ld\n", 
                encoder_get_count(&motor_fl.encoder), 
                encoder_get_count(&motor_bl.encoder), 
                encoder_get_count(&motor_fr.encoder), 
                encoder_get_count(&motor_br.encoder));
            printf("Motor Outs: %.1f %.1f %.1f %.1f\n", motor_fl.out, motor_bl.out, motor_fr.out, motor_br.out);
            printf("Encoder Vel: %.1f %.1f %.1f %.1f\n", motor_fl.cur_vel, motor_bl.cur_vel, motor_fr.cur_vel, motor_br.cur_vel);
            led = !led;
#ifdef PICO_W
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);
#else
            gpio_put(LED_PIN_PICO, led);
#endif
        }
        count++; 
        watchdog_update();
    }
}