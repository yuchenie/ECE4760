// ECE 4760 Lab 1 Code
// Erica Jiang (ej289)
// Max Trager (mt687)

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"

// VGA graphics library
#include "vga16_graphics_v2.h"
#include "pt_cornell_rp2040_v1_4.h"

// Keypad pin configurations
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

#define LED             25

unsigned int keycodes[12] = {   0x28, 0x11, 0x21, 0x41, 0x12,
                                0x22, 0x42, 0x14, 0x24, 0x44,
                                0x18, 0x48} ;
unsigned int scancodes[4] = {   0x01, 0x02, 0x04, 0x08} ;
unsigned int button = 0x70 ;


char keytext[40];

// State machine variables
int prev_key = -1;
int prev_prev_key = -1;

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

//Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0  // 2^32 (a constant)
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)

// the DDS units - core 0

// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs ;

// Frequency output to synthesize for input to DDS
volatile float frequency = 0;

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size] ;

// Values output to DAC
int DAC_output_0 ;
int DAC_output_1 ;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1) ;    // maximum amplitude
fix15 attack_inc ;                      // rate at which sound ramps up
fix15 decay_inc ;                       // rate at which sound ramps down
fix15 current_amplitude_0 = 0 ;         // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0 ;         // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME             250
#define DECAY_TIME              250
#define SUSTAIN_TIME            10000

// Sound Primitive timing parameter (6500 interrupts)
#define BEEP_DURATION           6500

// State machine variables
volatile unsigned int STATE_0 = 0 ;
volatile unsigned int RECORD_STATE = 0 ;
volatile unsigned int count_0 = 0 ;

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define LED      25
#define SPI_PORT spi0

//GPIO for timing the ISR
#define ISR_GPIO 2

// Playback thread variables and semaphore definition
semaphore_t playback_go ;
int record[100] ;
int record_index = 0;

// This timer ISR is called on core 0
static void alarm_irq(void) {

    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1) ;

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

    // If we want to generate a noise (or silence) ...
    if (STATE_0 != 0) {
        
        // Calculate swoop frequency
        if (STATE_0 == 1) {
            frequency = (-0.00002462) * count_0 * count_0 + 0.16 * count_0 + 1740;
        } 
        
        // Calculate chirp frequency
        else if (STATE_0 == 2) {
            frequency = (0.000118343) * count_0 * count_0 + 2000;
        } 
        
        // Silence
        else if (STATE_0 == 3) {
            frequency = 0;
        }
        
        // Increment DDS and send to DAC
        phase_incr_main_0 = (frequency*two32)/Fs;
        phase_accum_main_0 += phase_incr_main_0  ;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0>>24])) + 2048 ;

        // Ramp up amplitude
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
        }
        // Ramp down amplitude
        else if (count_0 > BEEP_DURATION - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        count_0 += 1 ;

        // State transition?
        if (count_0 == BEEP_DURATION) {
            STATE_0 = 0 ;
            count_0 = 0 ;
            y = 0.0;
        }
    }

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0) ;

}

// This thread runs on core 0 - playback mode
static PT_THREAD (protothread_playback(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;
    while(1) {
        // Wait for signal from record/keypad thread
        PT_SEM_SDK_WAIT(pt, &playback_go) ;
        
        // Set index to 0, set current to first sound to play
        int playback_index = 0;
        int current = record[playback_index];

        // While there are still sound primitives to play..
        while (current != 0) {
            
            // Play the current sound primitive in the array
            current_amplitude_0 = 0 ;
            STATE_0 = current ;
            count_0 = 0 ;
            
            // Let the sound play out (each primitive lasts 130ms)
            sleep_ms(130);

            // Go to the next element in the record array
            playback_index++;
            current = record[playback_index];
        }
        
    }
    // Indicate thread end
    PT_END(pt) ;
}

// This thread runs on core 0 - normal play mode and record mode.
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    // Initialize variables
    static int i = -1;
    static uint32_t keypad ;

    while(1) {

        gpio_put(LED, !gpio_get(LED)) ;

        // Scan the keypad!
        for (i=0; i<KEYROWS; i++) {
            // Set a row high
            gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                            (scancodes[i] << BASE_KEYPAD_PIN)) ;
            // Small delay required
            sleep_us(1) ; 
            // Read the keycode
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
            // Break if button(s) are pressed
            if (keypad & button) break ;
        }
        // If we found a button . . .
        if (keypad & button) {
            // Look for a valid keycode.
            for (i=0; i<NUMKEYS; i++) {
                if (keypad == keycodes[i]) break ;
            }
            // If we don't find one, report invalid keycode
            if (i==NUMKEYS) (i = -1) ;
        }
        // Otherwise, indicate invalid/non-pressed buttons
        else (i=-1) ;

        
        // Debouncer logic 
        if (i != -1 && i == prev_key && prev_key != prev_prev_key) {

            // If one of buttons 1, 2, 3 pressed ... 
            if (i <= 3) {
                
                // Set state to corresponding press, to play the desired sound
                current_amplitude_0 = 0 ;
                STATE_0 = i ;
                count_0 = 0 ;
                
                // If we are in record mode ... 
                if (RECORD_STATE) {
                    // Keep track of each press and increment for next possible element
                    record[record_index] = i;
                    record_index++;
                }
            } 
            
            // If button 4 is pressed, we want to record
            else if (i == 4) {
                
                // Indicate we are in record state
                RECORD_STATE = 1;
                
                // Clear the previous array
                for (int j = 0; j < 100; j++) {
                    record[j] = 0;
                }

                // Index at the start of the array
                record_index = 0;
            } 
            
            // If button 5 is pressed, we want to playback the recording (if there is one)
            else if (i == 5) {
                // We aren't recording anymore
                RECORD_STATE = 0;
                
                // Yield the thread 
                PT_YIELD(pt);
                
                // Sent a semaphore signal so that the playback thread is scheduled
                PT_SEM_SDK_SIGNAL(pt, &playback_go) ;
                
            }
        }
        
        // Set state machine variables for next keypad scan
        prev_prev_key = prev_key;
        prev_key = i ;

        // Run every 30msec
        PT_YIELD_usec(30000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}


int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    ////////////////// KEYPAD INITS ///////////////////////
    
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN)) ;
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)) ;
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0x0 << BASE_KEYPAD_PIN)) ;
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_down((BASE_KEYPAD_PIN + 4)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 5)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 6)) ;

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    spi_set_format(SPI_PORT, 16, 0, 0, 0);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC) ;
    gpio_set_dir(LDAC, GPIO_OUT) ;
    gpio_put(LDAC, 0) ;

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO) ;
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0) ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME)) ;
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME)) ;

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = float2fix15(2047*sin((float)ii*6.283/(float)sine_table_size));
    }

    // Initialize a semaphore for thread signaling
    sem_init(&playback_go, 0, 1) ;

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM) ;
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq) ;
    irq_set_enabled(ALARM_IRQ, true) ;
    
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

    // Add protothreads and start them
    pt_add_thread(protothread_core_0) ;
    pt_add_thread(protothread_playback) ;
    pt_schedule_start ;

}
