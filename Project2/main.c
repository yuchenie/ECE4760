
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
#define hitBottom(b) (b>int2fix15(480))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))

typedef struct
{
    fix15 x;
    fix15 y;
} peg;

typedef struct
{
    fix15 x;
    fix15 y;
    fix15 vx;
    fix15 vy;
} ball;

peg peg_array[15] ;
ball ball_array[15] ;

// uS per frame
#define FRAME_RATE 33000

// the color of the ball
char color = WHITE ;

// ball on core 0
fix15 ball0_x ;
fix15 ball0_y ;
fix15 ball0_vx ;
fix15 ball0_vy ;
fix15 peg0_x = int2fix15(320);
fix15 peg0_y = int2fix15(50);

// ball on core 1
fix15 ball1_x ;
fix15 ball1_y ;
fix15 ball1_vx ;
fix15 ball1_vy ;

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

// Create a ball
void spawnball(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction)
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
void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // Reverse direction if we've hit a wall
  if (hitTop(*y)) {
    *vy = (-*vy) ;
    *y  = (*y + int2fix15(5)) ;
  }
  if (hitBottom(*y)) {
    *vy = (-*vy) ;
    *y  = (*y - int2fix15(5)) ;
  } 
  if (hitRight(*x)) {
    *vx = (-*vx) ;
    *x  = (*x - int2fix15(5)) ;
  }
  if (hitLeft(*x)) {
    *vx = (-*vx) ;
    *x  = (*x + int2fix15(5)) ;
  } 

  // Update position using velocity
  *x = *x + *vx ;
  *y = *y + *vy ;
}

void death(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  if (hitBottom(*y)) {
    spawnball(&ball0_x, &ball0_y, &ball0_vx, &ball0_vy, 0);
  }
}

void drawBoard() {

}

bool collide(fix15* ball_x, fix15* ball_y, fix15* ball_vx, fix15* ball_vy, fix15* peg_x, fix15* peg_y) {
  fix15 dx = *ball_x - *peg_x ;
  fix15 dy = *ball_y - *peg_y ;
  fix15 dist ;

  if (absfix15(dx) > absfix15(dy)) {
    dist = absfix15(dx) + multfix15(float2fix15(0.25), absfix15(dy));
  } else {
    dist = absfix15(dy) + multfix15(float2fix15(0.25), absfix15(dx));
  }

  if (absfix15(dist) < int2fix15(10)) { 
    fix15 normal_x = divfix(dx, dist);
    fix15 normal_y = divfix(dy, dist);

    fix15 intermediate_term = multfix15(int2fix15(-2), (multfix15(normal_x, *ball_vx) + multfix15(normal_y, *ball_vy)));
   
    if (intermediate_term > int2fix15(0)) {
      *ball_x = *peg_x + multfix15(normal_x, (dist+int2fix15(2)));
      *ball_y = *peg_y + multfix15(normal_y, (dist+int2fix15(2)));

      *ball_vx = *ball_vx + multfix15(normal_x, intermediate_term);
      *ball_vy = *ball_vy + multfix15(normal_y, intermediate_term);
      
      // start the control channel
      dma_start_channel_mask(1u << ctrl_chan) ;
    }
  }
  
  *ball_vy = *ball_vy + float2fix15(0.32) ; // gravity
  *ball_x = *ball_x + *ball_vx ;
  *ball_y = *ball_y + *ball_vy ;

  return absfix15(dist) < int2fix15(10);
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input ;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write ;
      while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input a number in the range 1-15: ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input) ;
        // update ball color
        if ((user_input > 0) && (user_input < 16)) {
          color = (char)user_input ;
        }
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;

    // Spawn a ball
    spawnball(&ball0_x, &ball0_y, &ball0_vx, &ball0_vy, 0);

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;      
      // erase ball
      drawCircle(fix2int15(ball0_x), fix2int15(ball0_y), 4, BLACK);
      // update ball's position and velocity
      // wallsAndEdges(&ball0_x, &ball0_y, &ball0_vx, &ball0_vy) ;
      death(&ball0_x, &ball0_y, &ball0_vx, &ball0_vy) ;
      collide(&ball0_x, &ball0_y, &ball0_vx, &ball0_vy, &peg0_x, &peg0_y); 
      
      // draw the ball at its new position
      drawCircle(fix2int15(ball0_x), fix2int15(ball0_y), 4, WHITE);
      
      drawCircle(320, 50, 6, WHITE);
      // draw the boundaries
      // drawArena() ;
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

    // Spawn a ball
    spawnball(&ball1_x, &ball1_y, &ball1_vx, &ball1_vy, 1);

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;      
      // erase ball
      drawRect(fix2int15(ball1_x), fix2int15(ball1_y), 2, 2, BLACK);
      // update ball's position and velocity
      // wallsAndEdges(&ball1_x, &ball1_y, &ball1_vx, &ball1_vy) ;
      // draw the ball at its new position
      drawRect(fix2int15(ball1_x), fix2int15(ball1_y), 2, 2, color); 
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

void core1_main(){
  // Add animation thread
  pt_add_thread(protothread_anim1);
  // Start the scheduler
  // pt_schedule_start ;

}

int main(){
  set_sys_clock_khz(150000, true) ;
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

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 
