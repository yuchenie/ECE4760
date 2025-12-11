#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "pwm.pio.h"
#include "gate_drive.pio.h"
#include "pt_cornell_rp2040_v1_4.h"

#define OUT_PINS 10
#define PWM_PIN 25

// the rotor position as defined by hall input values. 
// note that this value does not change incrementally as the motor moves!
volatile int state = 0;

#define TAU 1e6 // time constant in us for low-pass filter

// controls, velocity feedback, and volts per hertz values
volatile int dir = 0;
volatile float throttle = 0;
volatile float motor_rpm = 0.0f;
const float RATED_MOTOR_RPM = 3000.0f;
const float RATED_MOTOR_VOLTAGE = 12.0f;
const float MAX_VOLTAGE_AT_STALL = 6.0f;
#define MIN_RPM 25.0f
#define WRAPVAL 255

// if the motor is stationary, then the interrupt needs to be called
// periodically to avoid the interrupt from never being called
// and the motor_rpm from never being updated
struct repeating_timer timer;

// time of last irq
volatile uint32_t irq_prev_time = 0;

// commutation table
const uint8_t shift[2][6][3] = {
    // clockwise
    {
        {1,2,3},
        {3,4,5},
        {1,4,5},
        {5,0,1},
        {5,2,3},
        {3,0,1},
    },
    // counter-clockwise
    {
        {3,0,1},
        {5,2,3},
        {5,0,1},
        {1,4,5},
        {3,4,5},
        {1,2,3},
    }
};

// PWM PIO state machine
PIO pwm_pio = pio0;
int pwm_sm = 0;

// GATE DRIVE PIO state machine
PIO gd_pio = pio1;
int gd_sm = 1;

// hall inputs
#define NUM_INPUTS 3
const uint input_pins[NUM_INPUTS] = {16, 17, 18};

// current sensing
const uint adc_pins[NUM_INPUTS] = {26, 27, 28};
#define SHUNT_RESISTANCE 0.025
#define INA_GAIN 20
#define LOGIC_LEVEL 3.3
#define ADC_MAX 4095
const float adc_to_current = (LOGIC_LEVEL) / (ADC_MAX * INA_GAIN * SHUNT_RESISTANCE);

#define abs(a) ((a>0) ? a:-a)

// Constrain duty cycle & map to wrap value.
int duty_cycle_to_level(float duty_cycle)
{
    if (duty_cycle < 0.0f)
        duty_cycle = 0.0f;
    if (duty_cycle >= 1.0f) {
        duty_cycle = 1.0f;
    }

    return (int)(duty_cycle * WRAPVAL);
}

// Write `period` to the input shift register
void pio_pwm_set_period(PIO pio, uint sm, uint32_t period) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put_blocking(pio, sm, period);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pio, sm, true);
}

// Write `level` to TX FIFO. State machine will copy this into X.
void pio_pwm_set_level(PIO pio, uint sm, uint32_t level) {
    pio_sm_put_blocking(pio, sm, level);
}

// Update values to drive the phases.
void update_control() {
    int pwm = gpio_get(PWM_PIN);
    uint32_t data = 0b000000 | (1 << shift[dir][state][0]) | (pwm << shift[dir][state][1]) | (!pwm << shift[dir][state][2]);    
    uint32_t test = 0b000000 | (1 << shift[dir][state][0]);
    pio_sm_put_blocking(gd_pio, gd_sm, test);
    pio_sm_put_blocking(gd_pio, gd_sm, data);
}

bool timer_callback(struct repeating_timer *t)
{
    // no velocity on boot
    if (irq_prev_time == 0) {
        motor_rpm = 0.0f;
    }

    int timer_current_time = time_us_64();
    float timer_period = (float)(timer_current_time - irq_prev_time);
    
    // low-pass filter
    float raw_rpm = 2.5e6f / timer_period; 
    float alpha = timer_period / (TAU + timer_period);
    motor_rpm = alpha * raw_rpm + (1.0f - alpha) * motor_rpm;
    
    // minimum measurable rpm is 25 rpm
    if (motor_rpm < MIN_RPM)
        motor_rpm = 0.0f;

    // update volts per hertz control with new speed
    float duty = throttle * (motor_rpm / RATED_MOTOR_RPM + MAX_VOLTAGE_AT_STALL / RATED_MOTOR_VOLTAGE);
    pio_pwm_set_level(pwm_pio, pwm_sm, duty_cycle_to_level(duty));
    
    cancel_repeating_timer(&timer);
    add_repeating_timer_ms(100, (repeating_timer_callback_t)timer_callback, NULL, &timer);
    return true;
}

void irq_handler(uint gpio, uint32_t events) {
    // Update rotor position & gate outputs.
    int a = gpio_get(input_pins[0]);
    int b = gpio_get(input_pins[1]);
    int c = gpio_get(input_pins[2]);
    state = ((a << 2) | (b << 1) | (c)) - 1;
    update_control();

    // Velocity feedback
    int irq_current_time = time_us_64();

    // time between steps in microseconds
    float step_period = (float)(irq_current_time - irq_prev_time);
    if (step_period <= 800.0f) { // invalid period, ignore
        return;
    }

    irq_prev_time = irq_current_time;

    // step/us * 1 elec. rev/6 steps * 1 mech. rev/4 elec. rev * 1e6 us/s * 60 s/min
    // = 2.5e6 rpm
    float raw_rpm = 2.5e6f / step_period; 

    // low-pass filter
    float alpha = step_period / (TAU + step_period);
    motor_rpm = alpha * raw_rpm + (1.0f - alpha) * motor_rpm;

    // update volts per hertz control with new speed
    float duty = throttle * (motor_rpm / RATED_MOTOR_RPM + MAX_VOLTAGE_AT_STALL / RATED_MOTOR_VOLTAGE);
    pio_pwm_set_level(pwm_pio, pwm_sm, duty_cycle_to_level(duty));

    // reset the timer to call this function again in 100ms if no step is detected
    cancel_repeating_timer(&timer);
    add_repeating_timer_ms(100, (repeating_timer_callback_t)timer_callback, NULL, &timer);
}

// Update controls on rising & falling edges of PWM signal. 
void pwm_irq0() {
    pio_interrupt_clear(pwm_pio, 0);
    update_control();
}

// Current sensing thread.
// Does not work with board :(
static PT_THREAD (current_sense(struct pt *pt)) {
    PT_BEGIN(pt) ;
    while(1) {
        
        // Determine the sensor to read depending on the state of the controls.
        if (dir == 1) {
            if (state == 1 || state == 2) {
                adc_select_input(0);
            } else if (state == 0 || state == 4) {
                adc_select_input(1);  
            } else if (state == 3 || state == 5) {
                adc_select_input(2);       
            }
        } else {
            if (state == 3 || state == 4) {
                adc_select_input(0);
            } else if (state == 1 || state == 5) {
                adc_select_input(1);  
            } else if (state == 0 || state == 2) {
                adc_select_input(2);       
            }
        }

        printf("Current: %f\n", adc_read() * adc_to_current);

        PT_YIELD_usec(10000) ;
    }
    PT_END(pt) ;
}

// User input thread. User can change throttle.
static PT_THREAD (serial_input(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char classifier ;
    static float throttle_ ;
    while(1) {
        sprintf(pt_serial_out_buffer, "input a command: ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        
        sscanf(pt_serial_in_buffer,"%f", &throttle_) ;

        if (throttle_ < 0) {
            dir = 1;
        } else {
            dir = 0;
        }

        throttle = abs(throttle_) ;
    }
    PT_END(pt) ;
}

int main() {
    // overclocking, be aware that changing the system clock affects the switching dead time!
    set_sys_clock_khz(150000, true) ;
    stdio_init_all();

    // PWM PIO state machine
    uint pwm_offset = pio_add_program(pwm_pio, &pwm_program);    
    pwm_program_init(pwm_pio, pwm_sm, pwm_offset, PWM_PIN);
  
    irq_set_exclusive_handler(PIO0_IRQ_0, pwm_irq0);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_set_irq0_source_enabled(pwm_pio, pis_interrupt0, true);

    pio_pwm_set_period(pwm_pio, pwm_sm, WRAPVAL);
    pio_pwm_set_level(pwm_pio, pwm_sm, 0);

    // GATE DRIVE PIO state machine
    uint gd_offset = pio_add_program(gd_pio, &gate_drive_program);
    gate_drive_program_init(gd_pio, gd_sm, gd_offset, OUT_PINS);
    pio_sm_set_enabled(gd_pio, gd_sm, true);

    // hall effect sensors
    gpio_set_irq_enabled_with_callback(input_pins[0], GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &irq_handler);
    for (int i = 0; i < NUM_INPUTS; i++)
    {
        gpio_init(input_pins[i]);
        gpio_set_dir(input_pins[i], GPIO_IN);
        gpio_pull_up(input_pins[i]);
        gpio_set_irq_enabled(input_pins[i], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    }

    // add a timer that will call the irq_handler if no step has been detected for 100ms
    add_repeating_timer_ms(100, (repeating_timer_callback_t)timer_callback, NULL, &timer);    

    // configure adc for current sense
    adc_init();
    adc_select_input(0);
    for (int i = 0; i < NUM_INPUTS; i++)
    {
        gpio_init(adc_pins[i]);
        gpio_set_dir(adc_pins[i], GPIO_IN);
        adc_gpio_init(adc_pins[i]);
    }

    // pt_add_thread(current_sense);
    pt_add_thread(serial_input);
    pt_schedule_start ;
}
