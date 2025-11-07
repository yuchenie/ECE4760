#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "vga16_graphics_v2.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"

volatile float duty_cycle = 0;
volatile fix15 setpoint = int2fix15(10);
volatile fix15 accel_angle, gyro_angle, gyro_angle_delta, complementary_angle;

// default tuned constants
volatile float kP = 0.016;
volatile float kI = 0.00005;
volatile float kD = 0.01;
volatile fix15 max_accumulator = int2fix15(18000);
volatile fix15 min_accumulator = int2fix15(-18000);
volatile fix15 fix_error_accumulator = 0;

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// character array
char screentext[40];

// draw speed
int threshold = 10 ;

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphores
static struct pt_sem vga_semaphore ;
static struct pt_sem playback_semaphore ; 

// Button parameters
#define BUTTON_PIN 15 

// Debouncer State Machine variables
int button_state = 0;
int prev_button_state = 0;
int prev_prev_button_state = 0;

// Predefined orientations
fix15 fix_0_deg = int2fix15(0);
fix15 fix_60_deg = int2fix15(60);
fix15 fix_90_deg = int2fix15(90);
fix15 fix_120_deg = int2fix15(120);

// Some paramters for PWM
#define WRAPVAL 6000
#define CLKDIV  25.0
#define MAX_PWM 2000
uint slice_num ;

float constrain(float value, float min, float max) {
    if (value > max) {
        return max;
    } else if (value < min) {
        return min;
    } else {
        return value;
    }
}

fix15 fix15constrain(fix15 value, fix15 min, fix15 max) {
    if (value > max) {
        return max;
    } else if (value < min) {
        return min;
    } else {
        return value;
    }
}

// Interrupt service routine
void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);
    accel_angle = multfix15(float2fix15(atan2(acceleration[2], acceleration[1])), oneeightyoverpi);
    gyro_angle_delta = multfix15(-gyro[0], zeropt001);
    complementary_angle = multfix15(complementary_angle + gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);

    fix15 fix_error_current = setpoint - complementary_angle;
    fix_error_accumulator += fix_error_current;
    fix_error_accumulator = fix15constrain(fix_error_accumulator, min_accumulator, max_accumulator);
    
    float raw_duty_cycle = kP * fix2float15(fix_error_current) + kI * fix2float15(fix_error_accumulator) + kD * fix2float15(gyro[0]);
    duty_cycle = duty_cycle + ((raw_duty_cycle - duty_cycle) / 16.0) ;
    int duty = (int) (constrain(duty_cycle, 0.0, 1.0) * MAX_PWM);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, duty);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty);

    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    PT_BEGIN(pt) ;

    // We will start drawing at column 81
    static int xcoord = 81 ;
    
    // Rescale the measurements for display
    static float OldRange = 500. ; // (+/- 250)
    static float NewRange = 150. ; // (looks nice on VGA)
    static float OldMin = -250. ;
    static float OldMax = 250. ;

    // Control rate of drawing
    static int throttle ;

    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "+2") ;
    setCursor(50, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "-2") ;
    setCursor(50, 425) ;
    writeString(screentext) ;

    // Draw top plot
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "+250") ;
    setCursor(45, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "-250") ;
    setCursor(45, 225) ;
    writeString(screentext) ;
    
    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1 ;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) { 
            // Zero drawspeed controller
            throttle = 0 ;

            // Erase a column
            drawVLine(xcoord, 0, 480, BLACK) ;

            // Draw bottom plot for duty cycle (multiply by 120 to scale from +/-2 to +/-250)
            drawPixel(xcoord, 430 - (int)(NewRange*((float)((duty_cycle*120.0)-OldMin)/OldRange)), WHITE) ;

            // Draw top plot for recorded and setpoint angles
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(complementary_angle))-OldMin)/OldRange)), WHITE) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(setpoint))-OldMin)/OldRange)), GREEN) ;

            // Text for values of PID constants, error integral, recorded and setpoint angles. 
            setTextColor2(WHITE, BLACK) ;
            setTextSize(1) ;
            char buffer[50];
            
            setCursor(10, 10) ;
            sprintf(buffer, "RECORDED ANGLE: %f", fix2float15(complementary_angle));
            writeString(buffer) ;

            setCursor(10, 20) ;
            sprintf(buffer, "SETPOINT ANGLE: %f", fix2float15(setpoint));
            writeString(buffer) ;

            setCursor(10, 30) ;
            // sprintf(buffer, "DUTY: %f", duty_cycle);
            sprintf(buffer, "ACCUMULATED ERROR: %d", fix2int15(fix_error_accumulator));
            writeString(buffer) ;
            
            setCursor(10, 40) ;
            sprintf(buffer, "kP: %f", kP);
            writeString(buffer) ;
            
            setCursor(10, 50) ;
            sprintf(buffer, "kI: %f", kI);
            writeString(buffer) ;
            
            setCursor(10, 60) ;
            sprintf(buffer, "kD: %f", kD);
            writeString(buffer) ;

            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
            }
        }
        
        /*
        Read the button and trigger playback if pressed 
        (not sure about timing here, given this runs at 1khz or around there)
        */ 

        // Get button state
        button_state = gpio_get(BUTTON_PIN);
        
        // Debouncer State Machine
        if (button_state == 1 && button_state == prev_button_state && prev_button_state != prev_prev_button_state) {
            // If button pressed, signal playback thread
            PT_SEM_SIGNAL(pt, &playback_semaphore);
        }
        // update state machine variables
        prev_prev_button_state = prev_button_state;
        prev_button_state = button_state;

    }
    // Indicate end of thread
    PT_END(pt);
}

// Playback thread, triggered on semaphore from button press in VGA thread (currently on core 1, not sure abt this)
static PT_THREAD (protothread_playback(struct pt *pt))
{
    PT_BEGIN(pt);
    
    while(true) {
        // Wait on semaphore from VGA thread
        PT_SEM_WAIT(pt, &playback_semaphore);

        // Set arm to horizontal
        setpoint = fix_90_deg;

        // Yield for 5 sec
        PT_YIELD_usec(5000000);

        // Set to 30 deg above horizontal (120)
        setpoint = fix_120_deg;

        // Yield for 5 sec
        PT_YIELD_usec(5000000);

        // Set to 30 deg below horizontal (60)
        setpoint = fix_60_deg;

        // Yield for 5 sec
        PT_YIELD_usec(5000000);

        // Set arm to horizontal
        setpoint = fix_90_deg;
    }

    PT_END(pt);
}

// User input thread. User can change draw speed
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char classifier ;
    static float test_setpoint ;
    static float test_kP ;
    static float test_kI ;
    static float test_kD ;
    static float float_in ;
    while(1) {
        sprintf(pt_serial_out_buffer, "input a command: ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        
        sscanf(pt_serial_in_buffer,"%f %f %f %f", &test_setpoint, &test_kP, &test_kI, &test_kD) ;
        setpoint = float2fix15(constrain(test_setpoint, 2.0, 150.0)) ;
        kP = test_kP ;
        kI = test_kI ;
        kD = test_kD ;
    }
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_add_thread(protothread_playback) ; 
    pt_schedule_start ;
}

int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    // set up gpio for button input
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_down(BUTTON_PIN);


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;

    // Pullup resistors on breakout board, don't need to turn on internals
    // gpio_pull_up(SDA_PIN) ;
    // gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;
}
