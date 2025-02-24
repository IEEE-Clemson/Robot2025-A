#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/critical_section.h"
#include "pico/cyw43_arch.h"

#include "hardware/adc.h"
#include "hardware/i2c.h"

#include "config.h"
#include "i2c_slave.h"
#include "i2c_fifo.h"
#include "pi_motor.h"
#include "pid_control.h"


struct __attribute__((__packed__)) I2CMemLayout {
    int16_t target_vx;
    int16_t target_vy;
    int16_t target_omega;

    int16_t cur_vx;
    int16_t cur_vy;
    int16_t cur_omega;
};

// Buff written to by i2c
uint8_t i2c_mem[BUFF_SIZE];
uint8_t i2c_mem_addr;
bool i2c_addr_written;

// Layout for usage in program
struct I2CMemLayout *mem = (I2CMemLayout*)i2c_mem;  

struct PIDParams pid_params;
struct PIMotor motor_fl, motor_fr, motor_bl, motor_br;

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    uint8_t byte;
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        if (!i2c_addr_written) {
            // writes always start with the memory address
            i2c_mem_addr = i2c_read_byte(i2c);
            i2c_addr_written = true;
        } else {
            // save into memory
            byte = i2c_read_byte(i2c);
            if(i2c_mem_addr < BUFF_SIZE)
                i2c_mem[i2c_mem_addr] = byte;
            i2c_mem_addr++;
        }
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        // load from memory
        if(i2c_mem_addr < BUFF_SIZE)
            i2c_write_byte(i2c, i2c_mem[i2c_mem_addr]);
        else
            i2c_write_byte(i2c, 0);
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
    pi_motor_init(&motor_fl, &pid_params, pio0, 0, FL_PIN_A, FL_PIN_B, FL_PIN_F, FL_PIN_R, FL_INVERTED);
    pi_motor_init(&motor_fr, &pid_params, pio0, 1, FR_PIN_A, FR_PIN_B, FR_PIN_F, FR_PIN_R, FR_INVERTED);
    pi_motor_init(&motor_bl, &pid_params, pio0, 2, BL_PIN_A, BL_PIN_B, BL_PIN_F, BL_PIN_R, BL_INVERTED);
    pi_motor_init(&motor_br, &pid_params, pio0, 3, BR_PIN_A, BR_PIN_B, BR_PIN_F, BR_PIN_R, BR_INVERTED);
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

    mem->cur_vx = (int16_t)(local_vx / MAX_VXY * 32768);
    mem->cur_vy = (int16_t)(local_vy / MAX_VXY * 32768);
    mem->cur_omega = (int16_t)(omega / MAX_OMEGA * 32768);
}

static void update_target_vel() {
    float vx, vy, omega;
    float a, b;

    vx = (float)(mem->target_vx) * MAX_VXY / 32768.0f;
    vy = (float)(mem->target_vy) * MAX_VXY / 32768.0f;
    omega = (float)(mem->target_omega) * MAX_OMEGA / 32768.0f;

    a = 1 / WHEEL_RADIUS;
    b = (WHEEL_DIST_X + WHEEL_DIST_Y) * omega;

    motor_fl.setpoint = a * (vx - vy - b);
    motor_fr.setpoint = a * (vx + vy + b);
    motor_bl.setpoint = a * (vx + vy - b);
    motor_br.setpoint = a * (vx - vy + b);
}


int main() {
    float dt;
    absolute_time_t time, time_to_sleep;
    int i = 0;
    bool led = false;

    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        return -1;
    }

    printf("Starting\n");

    setup_slave();
    init_motors();
    
    dt = 1.0f / FREQ;
    while(true) {
        time = get_absolute_time();
        time_to_sleep = delayed_by_ms(time, 1000 / FREQ);
        pi_motor_update(&motor_fl, dt);
        pi_motor_update(&motor_fr, dt);
        pi_motor_update(&motor_bl, dt);
        pi_motor_update(&motor_br, dt);

        update_local_vel();
        update_target_vel();
        
        busy_wait_until(time_to_sleep);

        // Blink led for heartbeat
        i = (i + 1) % 100;
        if(i == 0) {
            led = !led;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);
            printf("Here %f\n", motor_bl.cur_vel);
        }
    }
}