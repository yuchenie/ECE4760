
/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
  - GPIO 16 ---> VGA Hsync
  - GPIO 17 ---> VGA Vsync
  - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
  - GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
  - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
  - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
  - RP2040 GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "vga16_graphics_v2.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))

// Wall detection
#define hitBottom(b) (b>int2fix15(323))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(0))
#define hitRight(a) (a>int2fix15(640))

#define VERTICAL_SPACING 19
#define HORIZONTAL_SPACING 38
#define NUM_ROWS 16
#define NUM_PEGS ((NUM_ROWS * (NUM_ROWS + 1)) / 2)

// Data structures for pegs and balls
typedef struct
{
    int x;
    int y;
} peg;

typedef struct
{
    fix15 x;
    fix15 y;
    fix15 vx;
    fix15 vy;
    fix15 last_peg_y;
} ball;

#define MAX_BALLS 4095
volatile int NUM_BALLS = 0;

peg peg_array[136] ;
ball ball_array[MAX_BALLS] ;

// uS per frame
#define FRAME_RATE 33000

// the color of the ball
char color = WHITE ;

// ball on core 0
fix15 ball0_x ;
fix15 ball0_y ;
fix15 ball0_vx ;
fix15 ball0_vy ;
fix15 fix15_gravity = float2fix15(0.6) ;
fix15 fix15_bounciness = float2fix15(0.5);
fix15 fix15_0 = int2fix15(0) ;
fix15 fix15_2 = int2fix15(2) ;
fix15 fix15_160 = int2fix15(160);
fix15 fix15_neg2 = int2fix15(-2) ;
fix15 fix15_0point25 = float2fix15(0.25) ;
fix15 fix15_0point5 = float2fix15(0.5);
fix15 fix15_10 = int2fix15(10) ;
fix15 fix15_38 = int2fix15(38);

fix15 peg0_x = int2fix15(320);
fix15 peg0_y = int2fix15(19);

// ball on core 1
fix15 ball1_x ;
fix15 ball1_y ;
fix15 ball1_vx ;
fix15 ball1_vy ;

// Debouncer State Machine variables
int button_state = 0;
int prev_button_state = 0;
int prev_prev_button_state = 0;


// State 0: Reset / Update Parameters
// State 1: Ball Number Modulation
// State 2: Bounciness Modulation
int modulation_state = 0;

// Store num of balls before gets locked in
int temp_NUM_BALLS = 0;

// Store bounciness before gets locked in
int bounciness_reading;
fix15 fix15_reading;
fix15 fix15_temp_bounciness;

// Bounciness for display on VGA
float bounciness = 0.0;

// Number of samples per period in sine table
#define sine_table_size 256

// Sine table
int raw_sin[sine_table_size] ;

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size] ;

// Pointer to the address of the DAC data table
unsigned short * dma_pointer = &DAC_data[0] ;

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

//SPI configurations
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size ;

int ctrl_chan ;
int data_chan ;

#define ADC_PIN 26


// Number of slots balls could fall through on 16-row galton
#define SLOT_NUM 17
#define LAST_ROW_PEG_NUM 17

// maximum height of a box on the histogram (i.e how much space in pixels do we have to work with)
#define MAX_HEIGHT 140

fix15 fix_MAX_HEIGHT = int2fix15(MAX_HEIGHT);

// Keeps track of how many balls fall in each
int histogram[SLOT_NUM] ={0};
int histogram_total = 0;

// X-values of pegs in the last row
int last_row_x_vals[] = {16, 54, 92, 130, 168, 206, 244, 282, 320, 358, 396, 434, 472, 510, 548, 586, 624};

// Variables to determine if complete clearing is necesary
int max_value = 0;
int prev_max_value = 0;

// determine the index into the peg array corresponding with the first peg in each row
int peg_indexer[19] = {0, 0, 1, 3, 6, 10, 15, 21, 28, 36, 45, 55, 66, 78, 91, 105, 120, 136, 136};

/*

Histogram based on x-value when ball hits bottom:

The last row of the galton board is as follows

X-val:            16   54   92  130  168  206  244  282  320  358  396  434  472  510  548  586  624   

Row:              o    o    o    o    o    o    o    o    o    o    o    o    o    o    o    o    o

Index:          0   1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17  

The first peg in the last row is at x value (320 - 38(8)) = 16

So, if a ball hits bottom with x between 0 and 15 (shouldn't phase thru peg), it would go into index 0
    if a ball hits bottom with x between 17 and 53 (shouldn't phase thru peg), it would go into index 0

If we shift up by 22, then we can take the mod of the adjusted x value to get the proper index to increment. 
The "zero index" would range from 0 to 37, the "one index" would range from 39 to 75


We can perform a mod by performing fixed width division and converting to int

*/ 

// Updates histogram array, runs when a ball hits the bottom
void updateHistogramVals(fix15* x) 
{
  // Adjust x so taking mod alligns with index
  
  fix15 adjusted_x = *x + 22;
  

  // Take mod of adjusted x and 38 to get proper index to update
  int index_to_update = fix2int15(divfix(adjusted_x, fix15_38));

  // Increment corresponding index
  histogram[index_to_update]++;

}

// Reset histogram back to intial states
void resetHistogramVals() 
{
  
  clearLowFrame(323, BLACK);
  max_value = 0;
  histogram_total = 0;

  // Set all entries back to zero
  for (int i=0; i<SLOT_NUM; i++) {
    histogram[i] = 0;
  }
}

// Draw histogram boxes 
void displayHistogramVals() 
{
  // Figure out what the maximum balls contained in one "slot" is
  for (int i=0; i<SLOT_NUM; i++) {
    if (histogram[i] > max_value) {
      max_value = histogram[i];
    }
  };

  // If we need to rescale the boxes
  if (max_value != prev_max_value) {
    clearLowFrame(323, BLACK);
    prev_max_value = max_value;
  }

  // Only draw rectangles if balls have fallen through
  if (max_value != 0) {

    // Convert max value to fix15 for scale factor calc
    fix15 fix_max_value = int2fix15(max_value);
    
    // Gives a scale factor (Pixels per Value [num of balls]) for us to scale the histogram boxes dynamically
    fix15 height_scaler = divfix(fix_MAX_HEIGHT, fix_max_value);

    // Rectangle parameters for each box
    int rect_x; // Top left vertex x position
    int rect_y; // Top left vertex y position
    int rect_width;
    int rect_height;

    // For every slot
    for (int i=0; i<SLOT_NUM; i++) {
      
      // Only draw a box if non-zero value in slot
      if (histogram[i] > 0) {
        
        // Height (in pixels) for each slot = height_scale_factor x num of balls in slot
        fix15 fix_rect_height = multfix15(height_scaler, int2fix15(histogram[i]));

        // Height as an int, for input into draw function
        rect_height = fix2int15(fix_rect_height);

        // Slots on the left most edge (Slot 0)
        if (i==0) {
          rect_width   = 33;
          rect_x  = 0; 
        }

        // Slot on the right most edge (Slot 17)
        else if (i == 17) {
          rect_width = 15;
          rect_x = (last_row_x_vals[17] + 19);
        }
        
        // Otherwise standard spacing (minus 2 to give space between histogram boxes)
        else {
          rect_width = 36; 
          rect_x = (last_row_x_vals[i-1] + 19);
        }

        // Pixel # increase as you go down, so the y-pixel where you start the box is (480 - height of box)
        rect_y = 480 - rect_height;

        // Draw the rectangle for the given parameters
        drawRect(rect_x, rect_y, rect_width, rect_height, WHITE);
      }
    }
  }
}

// Spawn/respawn a ball
void spawnBall(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction)
{
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(0) ;
  // Choose left or right
  // if (direction) *vx = int2fix15(3) ;
  // else *vx = int2fix15(-3) ;
  *vx = (fix15) ((rand() & 0xFFFF) - int2fix15(1));
  *vx = *vx >> 1;
  // Moving down
  *vy = int2fix15(0) ;
}

// Detect wallstrikes, update velocity and position
void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy, ball* ball)
{
  // If a ball hits the bottom...
  if (hitBottom(*y)) {
    // Update the histogram display
    updateHistogramVals(&ball->x);
    displayHistogramVals();
    histogram_total++;

    // Respawn the ball
    spawnBall(&ball->x, &ball->y, &ball->vx, &ball->vy, 0);
  } 
  
  // If a ball hits the right wall...
  if (hitRight(*x)) {
    *vx = (-*vx) ;
    *x  = (*x - int2fix15(5)) ;
  }
  
  // If a ball hits the left wall...
  if (hitLeft(*x)) {
    *vx = (-*vx) ;
    *x  = (*x + int2fix15(5)) ;
  } 

}

// Collision physics
bool collide(fix15* ball_x, fix15* ball_y, fix15* ball_vx, fix15* ball_vy, fix15* int_peg_x, fix15* int_peg_y, fix15* last_peg_y) 
{
  fix15 peg_x = int2fix15(*int_peg_x);
  fix15 peg_y = int2fix15(*int_peg_y);
  fix15 dx = *ball_x - peg_x ;
  fix15 dy = *ball_y - peg_y ;
  fix15 abs_dx = absfix15(dx) ;
  fix15 abs_dy = absfix15(dy) ;
  fix15 dist ;

  // square root approximation
  if (abs_dx > abs_dy) {
    dist = abs_dx + multfix15(fix15_0point25, abs_dy);
  } else {
    dist = abs_dy + multfix15(fix15_0point25, abs_dx);
  }

  if (absfix15(dist) < fix15_10) {
    fix15 normal_x = divfix(dx, dist);
    fix15 normal_y = divfix(dy, dist);

    fix15 intermediate_term = multfix15(fix15_neg2, (multfix15(normal_x, *ball_vx) + multfix15(normal_y, *ball_vy)));
   
    if (intermediate_term > fix15_0) {
      *ball_x = peg_x + multfix15(normal_x, (dist+fix15_2));
      *ball_y = peg_y + multfix15(normal_y, (dist+fix15_2));

      *ball_vx = *ball_vx + multfix15(normal_x, intermediate_term);
      *ball_vy = *ball_vy + multfix15(normal_y, intermediate_term);
      
      if (*last_peg_y != peg_y) {
        dma_start_channel_mask(1u << ctrl_chan) ;
        *ball_vx = multfix15(fix15_bounciness, *ball_vx); 
        *ball_vy = multfix15(fix15_bounciness, *ball_vy);
      }
      
    }
  }

  return absfix15(dist) < fix15_10;
}

// Updating ball position and adding gravity
void moveBall(fix15* ball_x, fix15* ball_y, fix15* ball_vx, fix15* ball_vy) 
{
  *ball_vy = *ball_vy + fix15_gravity ;
  *ball_x = *ball_x + *ball_vx ;
  *ball_y = *ball_y + *ball_vy ;
}

// Animation on core 0
static PT_THREAD (protothread_anim0(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;

    // ################## BEGIN AI-GENERATED CODE ##################
    int start_x = 320;
    int start_y = 19;
    int peg_index = 0;

    for (int row = 0; row < NUM_ROWS; row++) {
        int y = start_y + row * VERTICAL_SPACING;

        for (int col = 0; col <= row; col++) {
            // center pegs in each row under start_x
            int x = start_x - (row * HORIZONTAL_SPACING) / 2 + col * HORIZONTAL_SPACING;

            peg_array[peg_index].x = x;
            peg_array[peg_index].y = y;
            peg_index++;
        }
    }
    // ################### END AI-GENERATED CODE ###################

    // initialize all balls first
    for (int i = 0; i < MAX_BALLS; i++) {
      ball_array[i].x = int2fix15(320) ;
      ball_array[i].y = int2fix15(0) ;
      ball_array[i].vx = int2fix15(0) ;
      ball_array[i].vy = int2fix15(0) ;
      spawnBall(&ball_array[i].x, &ball_array[i].y, &ball_array[i].vx, &ball_array[i].vy, 0);
    }

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;    

      for (int i = NUM_BALLS; i < MAX_BALLS; i+=2) {
        drawPixel(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), BLACK);
      }

      for (int i = 0; i < NUM_BALLS; i+=2) {
        drawPixel(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), BLACK);
        wallsAndEdges(&ball_array[i].x, &ball_array[i].y, &ball_array[i].vx, &ball_array[i].vy, &ball_array[i]) ;

        int test = fix2int15(ball_array[i].y) / 19;
                
        // only check nearest three rows
        for (int j = peg_indexer[test]; j < peg_indexer[test+2]; j++) {
          if (collide(&ball_array[i].x, &ball_array[i].y, &ball_array[i].vx, &ball_array[i].vy, &peg_array[j].x, &peg_array[j].y,&ball_array[i].last_peg_y)) {
            break;
          }
        }
        moveBall(&ball_array[i].x, &ball_array[i].y, &ball_array[i].vx, &ball_array[i].vy);
        drawPixel(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), WHITE);
      }
      
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
      // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

// Animation on core 1
static PT_THREAD (protothread_anim1(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;    

      for (int i = NUM_BALLS+1; i < MAX_BALLS; i+=2) {
        drawPixel(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), BLACK);
      }

      for (int i = 1; i < NUM_BALLS; i+=2) {
        drawPixel(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), BLACK);
        wallsAndEdges(&ball_array[i].x, &ball_array[i].y, &ball_array[i].vx, &ball_array[i].vy, &ball_array[i]) ;

        int test = fix2int15(ball_array[i].y) / 19;

        // only check nearest three rows
        for (int j = peg_indexer[test]; j < peg_indexer[test+2]; j++) {
          if (collide(&ball_array[i].x, &ball_array[i].y, &ball_array[i].vx, &ball_array[i].vy, &peg_array[j].x, &peg_array[j].y,&ball_array[i].last_peg_y)) {
            break;
          }
        }
        moveBall(&ball_array[i].x, &ball_array[i].y, &ball_array[i].vx, &ball_array[i].vy);
        drawPixel(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), WHITE);
      }
      
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

// Button and potentiometer input thread
static PT_THREAD (protothread_user_input_and_display(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;
    static int ms_30 = 0;

    while(1) {
      
      for (int i = 0; i < 136; i++) {
        drawCircle(peg_array[i].x, peg_array[i].y, 6, WHITE);
      }
      
      // Get button state
      button_state = gpio_get(28);
      
      // Debouncer State Machine
      if (button_state == 1 && button_state == prev_button_state && prev_button_state != prev_prev_button_state) {
        modulation_state++;
        
        // Wrapping behavior (States go 0, 1, 2, 0, 1, 2 etc)
        if (modulation_state == 3){
          modulation_state = 0;
        }
      }
      
      // State 0: Initial reset state, and also "update" state where parameters get updated from the adjusted values
      if (modulation_state == 0) {
        
        // If parameters have changed...
        if (NUM_BALLS != temp_NUM_BALLS || fix15_bounciness != fix15_temp_bounciness) {
          // Reset the histogram
          resetHistogramVals();
        }
        
        // Set simulation values using adjusted values by user
        NUM_BALLS = temp_NUM_BALLS;
        fix15_bounciness = fix15_temp_bounciness;
      }

      // State 1: Modulating the number of balls
      else if (modulation_state == 1) {
        temp_NUM_BALLS = (adc_read() >> 4) << 4;  
      }

      // State 2: Modulating the bounciness parameter
      else if (modulation_state == 2) {
        // Scale bounciness with 128 resolution from 0 to 1
        bounciness_reading = adc_read() >> 5;
        fix15_reading = int2fix15(bounciness_reading);

        // Update temp bounciness to be set in state 0
        fix15_temp_bounciness = divfix(fix15_reading, fix15_160);
        
        // Update bounciness for display on VGA
        bounciness = fix2float15(fix15_temp_bounciness);
      }

      // Write text to screen
      setTextColor2(WHITE, BLACK) ;
      setTextSize(1) ;
      char buffer[50];
      
      setCursor(10, 10) ;
      sprintf(buffer, "ACTIVE BALL COUNT: %04d", NUM_BALLS);
      writeString(buffer) ;
      
      // setCursor(10, 10) ;
      // sprintf(buffer, "TOTAL BALLS DROPPED: %d", histogram_total);
      // writeString(buffer) ;

      setCursor(10, 20) ;
      sprintf(buffer, "TIME: %d s", ms_30 / 30);
      writeString(buffer) ;

      setCursor(10, 30) ;
      sprintf(buffer, "STATE: %d ", modulation_state);
      writeString(buffer) ;

      setCursor(10, 40) ;
      sprintf(buffer, "DESIRED BALLS: %04d", temp_NUM_BALLS);
      writeString(buffer) ;

      setCursor(10, 50) ;
      sprintf(buffer, "DESIRED BOUNCINESS: %f", bounciness);
      writeString(buffer) ;

      // Update state machine variables
      prev_prev_button_state = prev_button_state;
      prev_button_state = button_state;

      ms_30++;
      
      PT_YIELD_usec(30000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}

// Set up core 1 thread
void core1_main(){
  // Add animation thread
  pt_add_thread(protothread_anim1);
  // Start the scheduler
  pt_schedule_start ;

}

int main(){
  set_sys_clock_khz(250000, true) ;
  // initialize stio
  stdio_init_all() ;

  // initialize VGA
  initVGA() ;

  sleep_ms(5000);

  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // Initialize SPI channel (channel, baud rate set to 20MHz)
  spi_init(SPI_PORT, 20000000) ;

  // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
  spi_set_format(SPI_PORT, 16, 0, 0, 0);

  // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

  // Build sine table and DAC data table
  int i ;
  for (i=0; i<(sine_table_size); i++){
      raw_sin[i] = (int)(2047 * sin((float)i*6.283/(float)sine_table_size) + 2047); //12 bit
      DAC_data[i] = DAC_config_chan_A | (raw_sin[i] & 0x0fff) ;
  }

  // Select DMA channels
  data_chan = dma_claim_unused_channel(true);;
  ctrl_chan = dma_claim_unused_channel(true);;

  // Setup the control channel
  dma_channel_config c = dma_channel_get_default_config(ctrl_chan);   // default configs
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);             // 32-bit txfers
  channel_config_set_read_increment(&c, false);                       // no read incrementing
  channel_config_set_write_increment(&c, false);                      // no write incrementing
  channel_config_set_chain_to(&c, data_chan);                         // chain to data channel

  dma_channel_configure(
      ctrl_chan,                          // Channel to be configured
      &c,                                 // The configuration we just created
      &dma_hw->ch[data_chan].read_addr,   // Write address (data channel read address)
      &dma_pointer,                   // Read address (POINTER TO AN ADDRESS)
      1,                                  // Number of transfers
      false                               // Don't start immediately
  );

  // Setup the data channel
  dma_channel_config c2 = dma_channel_get_default_config(data_chan);  // Default configs
  channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);            // 16-bit txfers
  channel_config_set_read_increment(&c2, true);                       // yes read incrementing
  channel_config_set_write_increment(&c2, false);                     // no write incrementing
  // (X/Y)*sys_clk, where X is the first 16 bytes and Y is the second
  // sys_clk is 125 MHz unless changed in code. Configured to ~44 kHz
  dma_timer_set_fraction(0, 0x0017, 0xffff) ;
  // 0x3b means timer0 (see SDK manual)
  channel_config_set_dreq(&c2, 0x3b);                                 // DREQ paced by timer 0


  dma_channel_configure(
      data_chan,                  // Channel to be configured
      &c2,                        // The configuration we just created
      &spi_get_hw(SPI_PORT)->dr,  // write address (SPI data register)
      DAC_data,                   // The initial read address
      sine_table_size,            // Number of transfers
      false                       // Don't start immediately.
  );

  gpio_init(ADC_PIN);
  gpio_set_dir(ADC_PIN, GPIO_IN);
  adc_init();
  adc_gpio_init(ADC_PIN);
  adc_select_input(0);

  // set up gpio for button input
  gpio_init(28);
  gpio_set_dir(28, GPIO_IN);
  gpio_pull_down(28);

  // add threads
  pt_add_thread(protothread_anim0);
  pt_add_thread(protothread_user_input_and_display);

  // start scheduler
  pt_schedule_start ;
} 