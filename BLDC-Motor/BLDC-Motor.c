/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pwm.pio.h"
#include "gate_drive.pio.h"

#define OUT_PINS 10
#define PWM_PIN 25

volatile int state = 0;
uint32_t prev_data = 0b000000; 

struct repeating_timer timer;

const uint8_t shift[6][3] = {
    {3,0,1},
    {5,2,3},
    {5,0,1},
    {1,4,5},
    {3,4,5},
    {1,2,3},
};

// TODO adjust wiring onto pico because it's wrong
#define NUM_INPUTS 3
const uint input_pins[NUM_INPUTS] = {16, 17, 18};

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

void irq_handler(uint gpio, uint32_t events) {
    int a = gpio_get(input_pins[0]);
    int b = gpio_get(input_pins[1]);
    int c = gpio_get(input_pins[2]);
    state = (a << 2) | (b << 1) | (c);
}

int main() {
    stdio_init_all();

    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);
    gpio_put(2, 0);

    gpio_init(3);
    gpio_set_dir(3, GPIO_OUT);
    gpio_put(3, 0);

    // PWM PIO state machine
    PIO pwm_pio = pio0;
    int pwm_sm = 0;
    uint pwm_offset = pio_add_program(pwm_pio, &pwm_program);
    pwm_program_init(pwm_pio, pwm_sm, pwm_offset, PWM_PIN);

    pio_pwm_set_period(pwm_pio, pwm_sm, 255);
    pio_pwm_set_level(pwm_pio, pwm_sm, 100);

    // GATE DRIVE PIO state machine
    PIO gd_pio = pio1;
    int gd_sm = 1;
    uint gd_offset = pio_add_program(gd_pio, &gate_drive_program);
    gate_drive_program_init(gd_pio, gd_sm, gd_offset, OUT_PINS);
    pio_sm_set_enabled(gd_pio, gd_sm, true);

    // hall effect sensors
    // gpio_set_irq_enabled_with_callback(input_pins[0], GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &irq_handler);
    for (int i = 0; i < NUM_INPUTS; i++)
    {
        gpio_init(input_pins[i]);
        gpio_set_dir(input_pins[i], GPIO_IN);
        gpio_pull_up(input_pins[i]);
        gpio_set_irq_enabled(input_pins[i], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    }

    while (true) {
        int a = gpio_get(input_pins[0]);
        int b = gpio_get(input_pins[1]);
        int c = gpio_get(input_pins[2]);
        state = ((a << 2) | (b << 1) | (c)) - 1;

        printf("data: %d\n", state);

        int pwm = gpio_get(PWM_PIN);
        uint32_t data = 0b000000 | (1 << shift[state][0]) | (pwm << shift[state][1]) | (!pwm << shift[state][2]);
        
        if (prev_data != data) { 
            uint32_t test = 0b000000 | (1 << shift[state][0]);
            pio_sm_put_blocking(gd_pio, gd_sm, test);
            pio_sm_put_blocking(gd_pio, gd_sm, data);
            prev_data = data;
        }        
    }
}
