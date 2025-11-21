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

volatile int state = 0;

struct repeating_timer timer;

const uint8_t shift[6][3] = {
    {2,1,0},
    {4,3,2},
    {4,1,0},
    {0,5,4},
    {2,5,4},
    {0,3,2},
};

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

bool timer_callback(struct repeating_timer *t)
{
    state += 1;
    state = state % 6;
    cancel_repeating_timer(&timer);
    add_repeating_timer_ms(100, (repeating_timer_callback_t)timer_callback, NULL, &timer);
    return true;
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
    pwm_program_init(pwm_pio, pwm_sm, pwm_offset, PICO_DEFAULT_LED_PIN);

    pio_pwm_set_period(pwm_pio, pwm_sm, 255);
    pio_pwm_set_level(pwm_pio, pwm_sm, 128);

    // GATE DRIVE PIO state machine
    PIO gd_pio = pio1;
    int gd_sm = 1;
    uint gd_offset = pio_add_program(gd_pio, &gate_drive_program);
    gate_drive_program_init(gd_pio, gd_sm, gd_offset, OUT_PINS);
    pio_sm_set_enabled(gd_pio, gd_sm, true);

    add_repeating_timer_ms(100, (repeating_timer_callback_t)timer_callback, NULL, &timer);

    while (true) {
        int pwm = gpio_get(PICO_DEFAULT_LED_PIN);
        uint32_t data = 0b000000 | (1 << shift[state][0]) | (pwm << shift[state][1]) | (!pwm << shift[state][2]);
        pio_sm_put_blocking(gd_pio, gd_sm, data);
    }
}
