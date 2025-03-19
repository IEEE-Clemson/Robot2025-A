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
#include "comm.h"
#include "accel_stepper.h"
#include <stdlib.h>

// Buff written to by i2c
static volatile uint8_t i2c_mem[BUFF_SIZE];
static volatile uint8_t i2c_mem_addr;
static volatile bool i2c_addr_written;
static volatile absolute_time_t i2c_last_heartbeat;

// Layout for usage in program
static volatile struct I2CMemLayout *mem = (volatile struct I2CMemLayout*)(i2c_mem);

// INTAKE
static struct PIDParams pid_params;
static struct PIMotor motor_intake;

// DUMPER
AccelStepper dumper_stepper(AccelStepper::DRIVER, DUMPER_PIN_STEP, DUMPER_PIN_DIR);
static int dumper_state = 0; // 0 -> Idle, 1 -> Lifting, 2 -> Shaking, 3 -> Retracting
static absolute_time_t shake_delay;
static bool shaking_up;

static void __not_in_flash_func(i2c_slave_handler)(i2c_inst_t *i2c, i2c_slave_event_t event) {
    uint8_t byte;
    i2c_last_heartbeat = get_absolute_time();

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

static bool is_heartbeat_alive() {
    return absolute_time_diff_us(get_absolute_time(), i2c_last_heartbeat) > 500'000;
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


static void init_intake() {
    encoder_init_pio(pio0);
    pid_create_params(&pid_params, 0.1f, 0.1f, 0.0f);
    pi_motor_init(&motor_intake, &pid_params, pio0, 0, INTAKE_PIN_A, INTAKE_PIN_B, INTAKE_PIN_R, INTAKE_PIN_F, INTAKE_INVERTED);
}

static void update_local_vel() {
    mem->cur_intake_v = (int16_t)(motor_intake.cur_vel / MAX_V * INT16_MAX);
}

static void update_target_vel() {
    motor_intake.setpoint = (float)(mem->target_intake_v) * MAX_V / INT16_MAX;
}

static void init_dumper() {
    dumper_stepper.disableOutputs();
    dumper_stepper.setAcceleration(DUMPER_ACCEL);
    dumper_stepper.setAcceleration(DUMPER_MAX_VEL);
}

static void update_dumper() {
    absolute_time_t time = get_absolute_time();
    if(dumper_state == 0) {
        // IDLE STATE
        if(mem->dumper_deploy == RTRUE) {
            mem->dumper_deploy = 0;
            dumper_stepper.enableOutputs();
            dumper_state = 1;
        }
    } else if(dumper_state == 1) {
        // LIFTING STATE
        dumper_stepper.run();
        if(dumper_stepper.distanceToGo() == 0) {
            // Transition to shake state
            shake_delay = delayed_by_ms(time, 4000);
            dumper_state = 2;
        }
    } else if(dumper_state == 2) {
        // SHAKE STATE

        dumper_stepper.run();
        if(dumper_stepper.distanceToGo() == 0) {
            if(time > shake_delay) {
                dumper_state = 3;
                dumper_stepper.moveTo(0);
            } else if(shaking_up) {
                shaking_up = !shaking_up;
                dumper_stepper.moveTo(DUMPER_TARGET_POS - DUMPER_TARGET_POS);
            } else {
                shaking_up = !shaking_up;
                dumper_stepper.moveTo(DUMPER_TARGET_POS + DUMPER_TARGET_POS);
            }
        }
    } else if (dumper_state == 3) {
        // DESCENDING STATE

        dumper_stepper.run();
        if(dumper_stepper.distanceToGo() == 0) {
            dumper_state = 0;
            dumper_stepper.disableOutputs();
        }
    } else {
        dumper_state = 0;
    }
    mem->dumper_active = dumper_state != 0;
}

static void init_servos() {

}

static void update_servos() {

}


int main() {
    float dt;
    absolute_time_t time, time_to_sleep;
    int i = 0;
    bool led = false;
    int count;

    // Disable pause on debug when sleeping
    timer_hw->dbgpause = 0;

    stdio_init_all();
    printf("Starting\n");
    if (watchdog_enable_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
    }
    watchdog_enable(1000, 1);
    
    // Unfortunately, we have to load the entire network stack to use LED on pico W
#ifdef PICO_W
    if(cyw43_arch_init()) {
        printf("Wi-Fi init failed");
    }
#else
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, true);
#endif
    setup_slave();
    printf("Motor init\n");
    init_intake();
    init_dumper();
    init_servos();

    watchdog_update();
    
    dt = 1.0f / FREQ;

    time = get_absolute_time();
    time_to_sleep = delayed_by_ms(time, 1000 / FREQ);
    while(true) {
        time = get_absolute_time();

        // Reset if heartbeat expired
        if(!is_heartbeat_alive()) {
            mem->dumper_deploy = 0;
            mem->target_intake_v = 0;
            mem->beacon_pos = 0;
            mem->box_mover_pos = 0;
        }

        if(time_to_sleep > time) {
            time_to_sleep = delayed_by_ms(time, 1000 / FREQ);
            pi_motor_update(&motor_intake, dt);
            
            update_local_vel();
            update_target_vel();

            // Blink led for heartbeat
            i = (i + 1) % 100;
            if(i == 0) {
                // This is a no-op if usb is disconnected
                printf("Motor Outs: %.1f\n", motor_intake.out);
                printf("Encoder Vel: %.1f\n", motor_intake.cur_vel);
                led = !led;
        }
        update_dumper();
        update_servos();
        
#ifdef PICO_W
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);
#else
            gpio_put(PICO_DEFAULT_LED_PIN, led);
#endif
        }

        count++; 
        watchdog_update();
        busy_wait_until(time_to_sleep);
    }
}