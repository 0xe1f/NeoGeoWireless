/**
 * Copyright (c) Akop Karapetyan - All Rights Reserved
 *
 * Unauthorized copying and distribution of this file, via any medium is
 * strictly prohibited.
 *
 * See file "LICENSE" for additional information.
 **/

#include "compat.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// tick per 64 clock cycles; overflow for every 256 ticks
#define CYCLES_PER_US (F_CPU / 1000000L)
#define US_PER_T0_OVF ((64 * 256) / CYCLES_PER_US)
#define MILLIS_INC (US_PER_T0_OVF / 1000)
#define FRACT_INC ((US_PER_T0_OVF % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

static uint8_t compat_initialized = 0;
static volatile uint32_t timer0_overflow_count = 0;
static volatile uint32_t timer0_millis = 0;
static volatile uint8_t timer0_fract = 0;

uint8_t compat_map_mask[] = {
    _BV(0), // PD0 (0)
    _BV(1), // PD1 (1)
    _BV(2), // PD2 (2)
    _BV(3), // PD3 (3)
    _BV(4), // PD4 (4)
    _BV(5), // PD5 (5)
    _BV(6), // PD6 (6)
    _BV(7), // PD7 (7)
    _BV(0), // PB0 (8)
    _BV(1), // PB1 (9)
    _BV(2), // PB2 (10)
    _BV(3), // PB3 (11)
    _BV(4), // PB4 (12)
    _BV(5), // PB5 (13)
    _BV(0), // PC0 (14)
    _BV(1), // PC1 (15)
    _BV(2), // PC2 (16)
    _BV(3), // PC3 (17)
    _BV(4), // PC4 (18)
    _BV(5), // PC5 (19)
    _BV(6), // PB6 (20)
    _BV(7), // PB7 (21)
    _BV(6), // PC6 (22)
};

volatile uint8_t *compat_map_ddr[] = {
    &DDRD, // PD0 (0)
    &DDRD, // PD1 (1)
    &DDRD, // PD2 (2)
    &DDRD, // PD3 (3)
    &DDRD, // PD4 (4)
    &DDRD, // PD5 (5)
    &DDRD, // PD6 (6)
    &DDRD, // PD7 (7)
    &DDRB, // PB0 (8)
    &DDRB, // PB1 (9)
    &DDRB, // PB2 (10)
    &DDRB, // PB3 (11)
    &DDRB, // PB4 (12)
    &DDRB, // PB5 (13)
    &DDRC, // PC0 (14)
    &DDRC, // PC1 (15)
    &DDRC, // PC2 (16)
    &DDRC, // PC3 (17)
    &DDRC, // PC4 (18)
    &DDRC, // PC5 (19)
    &DDRB, // PB6 (20)
    &DDRB, // PB7 (21)
    &DDRC, // PC6 (22)
};

volatile uint8_t *compat_map_port[] = {
    &PORTD, // PD0 (0)
    &PORTD, // PD1 (1)
    &PORTD, // PD2 (2)
    &PORTD, // PD3 (3)
    &PORTD, // PD4 (4)
    &PORTD, // PD5 (5)
    &PORTD, // PD6 (6)
    &PORTD, // PD7 (7)
    &PORTB, // PB0 (8)
    &PORTB, // PB1 (9)
    &PORTB, // PB2 (10)
    &PORTB, // PB3 (11)
    &PORTB, // PB4 (12)
    &PORTB, // PB5 (13)
    &PORTC, // PC0 (14)
    &PORTC, // PC1 (15)
    &PORTC, // PC2 (16)
    &PORTC, // PC3 (17)
    &PORTC, // PC4 (18)
    &PORTC, // PC5 (19)
    &PORTB, // PB6 (20)
    &PORTB, // PB7 (21)
    &PORTC, // PC6 (22)
};

volatile uint8_t *compat_map_pin[] = {
    &PIND, // PD0 (0)
    &PIND, // PD1 (1)
    &PIND, // PD2 (2)
    &PIND, // PD3 (3)
    &PIND, // PD4 (4)
    &PIND, // PD5 (5)
    &PIND, // PD6 (6)
    &PIND, // PD7 (7)
    &PINB, // PB0 (8)
    &PINB, // PB1 (9)
    &PINB, // PB2 (10)
    &PINB, // PB3 (11)
    &PINB, // PB4 (12)
    &PINB, // PB5 (13)
    &PINC, // PC0 (14)
    &PINC, // PC1 (15)
    &PINC, // PC2 (16)
    &PINC, // PC3 (17)
    &PINC, // PC4 (18)
    &PINC, // PC5 (19)
    &PINB, // PB6 (20)
    &PINB, // PB7 (21)
    &PINC, // PC6 (22)
};

// timer0, responsible for tracking elapsed time
ISR(TIMER0_OVF_vect) {
    uint32_t m = timer0_millis;
    uint8_t f = timer0_fract;

    m += MILLIS_INC;
    f += FRACT_INC;
    if (f >= FRACT_MAX) {
        f -= FRACT_MAX;
        m += 1;
    }

    timer0_fract = f;
    timer0_millis = m;
    timer0_overflow_count++;
}

void pinMode(uint8_t pin, uint8_t mode) {
    volatile uint8_t *ddr = pin_to_ddr(pin);
    if (ddr == CPT_BAD_REG) {
        return;
    }
    uint8_t mask = pin_to_mask(pin);

    if (mode == CPT_INPUT) {
        volatile uint8_t *port = pin_to_port(pin);
        *ddr &= ~mask;
        *port &= ~mask;
    } else if (mode == CPT_OUTPUT) {
        *ddr |= mask;
    } else if (mode == CPT_INPUT_PULLUP) {
        volatile uint8_t *port = pin_to_port(pin);
        *ddr &= ~mask;
        *port |= mask;
    }
}

void digitalWrite(uint8_t pin, uint8_t val) {
    volatile uint8_t *port = pin_to_port(pin);
    if (port == CPT_BAD_REG) {
        return;
    }
    uint8_t mask = pin_to_mask(pin);

    if (val == CPT_LOW) {
        *port &= ~mask;
    } else {
        *port |= mask;
    }
}

uint8_t digitalRead(uint8_t pin) {
    volatile uint8_t *port = pin_to_pin(pin);
    if (port == CPT_BAD_REG) {
        return CPT_LOW;
    }
    uint8_t mask = pin_to_mask(pin);

    if (*port & mask) {
        return CPT_HIGH;
    } else {
        return CPT_LOW;
    }
}

uint32_t compat_ms() {
    uint8_t oldSREG = SREG;

    cli();
    uint32_t m = timer0_millis;
    SREG = oldSREG;

    return m;
}

uint32_t compat_us() {
    uint8_t oldSREG = SREG;

    cli();
    uint32_t m = timer0_overflow_count;
    uint8_t t = TCNT0;

    if ((TIFR0 & _BV(TOV0)) && (t < 255)) {
        m++;
    }

    SREG = oldSREG;

    return ((m << 8) + t) * (64 / CYCLES_PER_US);
}

inline void compat_delay_ms(uint32_t ms) {
    for (; ms--;) { _delay_ms(1); }
}

inline void compat_delay_us(uint32_t us) {
    // This seems to drift from the true value by about .5ms per 1000ms,
    // but I guess it's good enough for now
    compat_delay_ms(us / 1000);
    for (us %= 1000; us--;) { _delay_us(1); }
}

void compat_init() {
    if (compat_initialized) {
        return;
    }

    TCCR0B = _BV(CS01) | _BV(CS00); // Clk/64
    TIMSK0 = _BV(TOIE0); // enable timer0 overflow interrupt
    sei();

    compat_initialized = 1;
}
