// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ----- //
// clock //
// ----- //

#define clock_wrap_target 2
#define clock_wrap 8

static const uint16_t clock_program_instructions[] = {
    0x80a0, //  0: pull   block                      
    0x6040, //  1: out    y, 32                      
            //     .wrap_target
    0xa022, //  2: mov    x, y                       
    0xe001, //  3: set    pins, 1                    
    0xc000, //  4: irq    nowait 0                   
    0x0045, //  5: jmp    x--, 5                     
    0xa022, //  6: mov    x, y                       
    0xe000, //  7: set    pins, 0                    
    0x0048, //  8: jmp    x--, 8                     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program clock_program = {
    .instructions = clock_program_instructions,
    .length = 9,
    .origin = -1,
};

static inline pio_sm_config clock_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + clock_wrap_target, offset + clock_wrap);
    return c;
}

// this is a raw helper function for use by the user which sets up the GPIO output, and configures the SM to output on a particular pin
void clock_program_init(PIO pio, uint sm, uint offset, uint pin) {
   pio_gpio_init(pio, pin);
   pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
   pio_sm_config c = clock_program_get_default_config(offset);
   sm_config_set_set_pins(&c, pin, 1);
   pio_sm_init(pio, sm, offset, &c);
}

#endif
