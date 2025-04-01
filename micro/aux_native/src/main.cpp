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
#include <hardware/pwm.h>
#include <hardware/adc.h>
#include <pico/multicore.h>

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

// BOX MOVER
uint box_mover_pwm;
uint beacon_pwm;
// BEACON PLACER

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
    return true;
    //return absolute_time_diff_us(get_absolute_time(), i2c_last_heartbeat) > 500'000;
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
    motor_intake.setpoint = ((float)(mem->target_intake_v)) * MAX_V / INT16_MAX;
}

static void init_dumper() {
    dumper_stepper.disableOutputs();
    dumper_stepper.setEnablePin(DUMPER_PIN_EN);
    dumper_stepper.setPinsInverted(false, false, true);
    dumper_stepper.setAcceleration(DUMPER_ACCEL);
    dumper_stepper.setMaxSpeed(DUMPER_MAX_VEL);
}

static void update_dumper() {
    absolute_time_t time = get_absolute_time();
    if(dumper_state == 0) {
        // IDLE STATE
        if(mem->dumper_deploy == RTRUE) {
            mem->dumper_deploy = 0;
            dumper_stepper.setAcceleration(DUMPER_ACCEL);
            dumper_stepper.enableOutputs();
            dumper_state = 1;
            dumper_stepper.moveTo(DUMPER_TARGET_POS);
        }
    } else if(dumper_state == 1) {
        // LIFTING STATE
        dumper_stepper.run();
        if(dumper_stepper.distanceToGo() == 0) {
            // Transition to shake state
            dumper_stepper.setAcceleration(60000);
            dumper_stepper.setMaxSpeed(20000);
            shake_delay = delayed_by_ms(time, 4000);
            dumper_state = 2;
        }
    } else if(dumper_state == 2) {
        // SHAKE STATE

        dumper_stepper.run();
        if(dumper_stepper.distanceToGo() == 0) {
            if(time > shake_delay) {
                dumper_state = 3;
                dumper_stepper.setMaxSpeed(DUMPER_MAX_VEL);
                dumper_stepper.setAcceleration(DUMPER_ACCEL);
                dumper_stepper.moveTo(0);
            } else if(shaking_up) {
                shaking_up = !shaking_up;
                dumper_stepper.moveTo(DUMPER_TARGET_POS - DUMPER_SHAKE_STEPS);
            } else {
                shaking_up = !shaking_up;
                dumper_stepper.moveTo(DUMPER_TARGET_POS + DUMPER_SHAKE_STEPS);
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
    pwm_config config;

    gpio_set_function(BOX_MOVER_PIN_PWM, GPIO_FUNC_PWM);
    gpio_set_function(BEACON_PIN_PWM, GPIO_FUNC_PWM);
    
    mem->box_mover_pos = 3;
    mem->beacon_pos = 0;
    box_mover_pwm = pwm_gpio_to_slice_num(BOX_MOVER_PIN_PWM);
    beacon_pwm = pwm_gpio_to_slice_num(BEACON_PIN_PWM);

    config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 38.0f);
    pwm_init(box_mover_pwm, &config, true);
    pwm_init(beacon_pwm, &config, true);
}

static uint16_t beacon_level = BEACON_STOWED_POS;
static uint16_t box_mover_level = BOX_MOVER_GRIP_POS;
static uint16_t cur_beacon_level = BEACON_STOWED_POS;
static void update_servos() {
    uint16_t slew = 10;

    switch(mem->beacon_pos) {
    case 0:
        beacon_level = BEACON_STOWED_POS;
        break;
    case 1:
        beacon_level = BEACON_TRAVEL_POS;
        break;
    case 2:
        beacon_level = BEACON_DEPLOY_POS;
        break;
    default:
        beacon_level = BEACON_STOWED_POS;
        break;
    }

    switch(mem->box_mover_pos) {
    case 0:
        box_mover_level = BOX_MOVER_GRIP_POS;
        break;
    case 1:
        box_mover_level = BOX_MOVER_OPEN_POS;
        break;
    case 2:
        box_mover_level = BOX_MOVER_IDLE_POS;
        break;
    case 3:
        box_mover_level = BOX_MOVER_OFF_POS;
        break;
    default:
        box_mover_level = BOX_MOVER_OFF_POS;
        break;
    }
    if(cur_beacon_level < beacon_level) {
        if(cur_beacon_level + slew > beacon_level) {
            cur_beacon_level = beacon_level;
        } else {
            cur_beacon_level += slew;
        }
    } else if(cur_beacon_level > beacon_level) {
        if(cur_beacon_level - slew < beacon_level) {
            cur_beacon_level = beacon_level;
        } else {
            cur_beacon_level -= slew;
        }
    }
    pwm_set_gpio_level(BEACON_PIN_PWM, cur_beacon_level);
    pwm_set_gpio_level(BOX_MOVER_PIN_PWM, box_mover_level);
}

static void init_armed_switch() {
    gpio_init(PIN_ARMED);
    gpio_set_dir(PIN_ARMED, false);
    gpio_set_pulls(PIN_ARMED, true, false);

    adc_init();
    adc_gpio_init(START_PIN_SENSE);
}

static void update_start_led() {
    uint16_t led_adc;
    adc_select_input(2);
    led_adc = adc_read();
    
    mem->light_on = led_adc > 1300;
}

void stepper_core_thread() {
    while(true) {
        update_dumper();
    }
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
    multicore_reset_core1();
    sleep_ms(100);
    multicore_launch_core1(&stepper_core_thread);
    init_servos();
    init_armed_switch();

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
            mem->box_mover_pos = 3;
            //mem->armed = 0;
        } else {
            mem->armed = gpio_get(PIN_ARMED);
        }

        if(time_to_sleep < time) {
            time_to_sleep = delayed_by_ms(time, 1000 / FREQ);
            pi_motor_update(&motor_intake, dt);
            
            update_local_vel();
            update_target_vel();

            update_servos();
            update_start_led();
            // Blink led for heartbeat
            i = (i + 1) % 100;
            if(i == 0) {
                // This is a no-op if usb is disconnected
                printf("Motor Outs: %.1f\n", motor_intake.out);
                printf("Encoder Vel: %.1f\n", motor_intake.cur_vel);
                led = !led;
            }            
            count++; 
            
        }
        watchdog_update();
        
#ifdef PICO_W
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);
#else
            gpio_put(PICO_DEFAULT_LED_PIN, led);
#endif
    }
}