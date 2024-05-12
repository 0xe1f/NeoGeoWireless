// Copyright 2021-2024, Akop Karapetyan
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stddef.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// SNES timing
#define CLOCK_CYCLE_COUNT 16
#define LATCH_DELAY_US    12
#define CLOCK_DELAY_US    6

// States
#define STATE_AUTOFIRE 0x01

// Hotkeys
#define HOTKEY_MACRO    0x02
#define HOTKEY_AUTOFIRE 0x01

// EEPROM
#define MAGIC_MASK        0xd34d0000
#define EEPROM_ADDR_STATE ((uint32_t *) 0)

#define STATUS_LED_ON  0x7f

#ifdef PAL
#define FPS 50 // frames per second
#define AUTOFIRE_INTERVAL(b) (((current_frame - b->pressed_frames) / 5) & 1)
#else
#define FPS 60 // frames per second
#define AUTOFIRE_INTERVAL(b) (((current_frame - b->pressed_frames) / 6) & 1)
#endif

#define ELAPSED(x) (current_frame - (x))

// in frames
#define STATUS_AF_BLINK_PERIOD   FPS
#define STATUS_FAST_BLINK_PERIOD (FPS / 4)
#define STATUS_FAST_DURATION     FPS // 1 sec
#define HOTKEY_THRESHOLD         ((int)(FPS * 1.5f)) // 1.5 sec

struct pin {
    volatile uint8_t *ddr;
    volatile uint8_t *port;
    uint8_t mask;
};

struct button {
    uint8_t state;
    uint32_t pressed_frames;
};

struct hotkey {
    uint16_t button_mask;
    uint8_t type;
    struct button *autofire;
    uint16_t macro_mask;
    uint32_t pressed_frames;
};

// SNES button indices (clock cycle order)
#define SN_INDEX_B   0
#define SN_INDEX_Y   1
#define SN_INDEX_SEL 2
#define SN_INDEX_STA 3
#define SN_INDEX_UP  4
#define SN_INDEX_DN  5
#define SN_INDEX_LT  6
#define SN_INDEX_RT  7
#define SN_INDEX_A   8
#define SN_INDEX_X   9
#define SN_INDEX_LB  10
#define SN_INDEX_RB  11

static struct button buttons[] = {
    { 0 }, // B
    { 0 }, // Y
    { 0 }, // SELECT
    { 0 }, // START
    { 0 }, // UP
    { 0 }, // DOWN
    { 0 }, // LEFT
    { 0 }, // RIGHT
    { 0 }, // A
    { 0 }, // X
    // N/A below here
    { 0 }, // LB
    { 0 }, // RB
    { 0 }, // ...
    { 0 }, // ...
    { 0 }, // ...
    { 0 }, // ...
};

// SNES button masks (clock cycle order)
#define SN_MASK_B   0x0001
#define SN_MASK_Y   0x0002
#define SN_MASK_SEL 0x0004
#define SN_MASK_STA 0x0008
#define SN_MASK_UP  0x0010
#define SN_MASK_DN  0x0020
#define SN_MASK_LT  0x0040
#define SN_MASK_RT  0x0080
#define SN_MASK_A   0x0100
#define SN_MASK_X   0x0200
#define SN_MASK_LB  0x0400
#define SN_MASK_RB  0x0800

#define SN_MASK_ABXY (SN_MASK_A | SN_MASK_B | SN_MASK_X | SN_MASK_Y)

static struct hotkey hotkeys[] = {
    // Macro
    { SN_MASK_SEL | SN_MASK_LB, HOTKEY_MACRO },
    { SN_MASK_SEL | SN_MASK_RB, HOTKEY_MACRO },
    // Autofire
    { SN_MASK_SEL | SN_MASK_A, HOTKEY_AUTOFIRE, &buttons[SN_INDEX_A] },
    { SN_MASK_SEL | SN_MASK_B, HOTKEY_AUTOFIRE, &buttons[SN_INDEX_B] },
    { SN_MASK_SEL | SN_MASK_X, HOTKEY_AUTOFIRE, &buttons[SN_INDEX_X] },
    { SN_MASK_SEL | SN_MASK_Y, HOTKEY_AUTOFIRE, &buttons[SN_INDEX_Y] },
    // N/A
    { 0 }
};

// SNES signal registers
// v0.1 - latch: pc0; data: pb3
#define LATCH_DDR  DDRC
#define LATCH_PORT PORTC
#define LATCH_MASK _BV(PC1)
#define CLOCK_DDR  DDRC
#define CLOCK_PORT PORTC
#define CLOCK_MASK _BV(PC2)
#define DATA_DDR   DDRC
#define DATA_PORT  PORTC
#define DATA_PIN   PINC
#define DATA_MASK  _BV(PC0)

// LED registers
#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_MASK _BV(PB3)
#define LED_OCR  OCR2A

// SNES button state map; updated by interrupt - do not read
static volatile uint16_t ir_button_states = 0;
// Frames elapsed since start; updated by interrupt - do not read
static volatile uint32_t ir_frames_elapsed = 0;
// Controller state
static uint16_t controller_state = 0;
// Main loop's current frame
static uint32_t current_frame = 0;
// Main loop's current button states
static uint16_t current_button_states = 0;
// While > current_frames, fast blink status LED
static uint32_t fast_blink_frames = 0;

// Pins for PCB v0.1
// SNES clock ordering
// Neo Geo pinout; initial map based on NGCD controller
static volatile struct pin output_pins[] = {
    { &DDRD, &PORTD, _BV(PD4) }, // B / A
    { &DDRB, &PORTB, _BV(PB7) }, // Y / C
    { &DDRD, &PORTD, _BV(PD6) }, // SELECT / SELECT
    { &DDRD, &PORTD, _BV(PD5) }, // START / START
    { &DDRD, &PORTD, _BV(PD0) }, // UP / UP
    { &DDRD, &PORTD, _BV(PD1) }, // DOWN / DOWN
    { &DDRD, &PORTD, _BV(PD2) }, // LEFT / LEFT
    { &DDRD, &PORTD, _BV(PD3) }, // RIGHT / RIGHT
    { &DDRB, &PORTB, _BV(PB6) }, // A / B
    { &DDRD, &PORTD, _BV(PD7) }, // X / D
    // N/A below here
    { 0, 0, 0 }, // L bumper
    { 0, 0, 0 }, // R bumper
    { 0, 0, 0 }, // always high
    { 0, 0, 0 }, // ..
    { 0, 0, 0 }, // ..
    { 0, 0, 0 }, // ..
};

static void init_pins() {
    // LATCH: normally low
    // CLOCK: normally high
    LATCH_DDR |= LATCH_MASK;
    LATCH_PORT &= ~LATCH_MASK;
    CLOCK_DDR |= CLOCK_MASK;
    CLOCK_PORT |= CLOCK_MASK;
    // DATA:  high during clock cycle by default; low when pressed
    DATA_DDR &= ~DATA_MASK;
    DATA_PORT &= ~DATA_MASK;
    // LED
    LED_DDR |= LED_MASK;
    LED_PORT &= LED_MASK;

    // NG output pins
    for (volatile struct pin *p = output_pins; p->ddr; p++) {
        *p->ddr |= p->mask;
        *p->port |= p->mask;
    }
}

static void init_interrupts() {
    // https://eleccelerator.com/avr-timer-calculator/
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS10) | _BV(CS12); // CTC, 1024 prescaler
#ifdef PAL
    OCR1A = 156; // approx. 50Hz
#else // PAL
    OCR1A = 130; // approx. 60Hz
#endif // PAL
    TIMSK1 = _BV(OCIE1A); // TIMER1 compare interrupt vector

    // status LED
    // OC2A
    // https://gist.github.com/JesusIslam/0b921543a8f962cf05a34034f1645eca
    TCCR2A |= _BV(COM2A1) | _BV(WGM20); // mode1; pwm phase correct
    TCCR2B |= _BV(CS20); // no prescale
    LED_OCR = 0x00;

    sei();
}

// Called every ~16.67ms (60Hz)
ISR(TIMER1_COMPA_vect) {
    ir_frames_elapsed++;
    // latch signal
    LATCH_PORT |= LATCH_MASK;
    _delay_us(LATCH_DELAY_US);
    LATCH_PORT &= ~LATCH_MASK;

    uint16_t states = 0;
    for (int i = 0; i < CLOCK_CYCLE_COUNT; i++) {
        // clock cycle
        _delay_us(CLOCK_DELAY_US);
        CLOCK_PORT &= ~CLOCK_MASK;
        // read data; will be low if pressed
        uint8_t data = DATA_PIN & DATA_MASK;
        states |= (data == 0) << i;
        _delay_us(CLOCK_DELAY_US);
        CLOCK_PORT |= CLOCK_MASK;
    }
    ir_button_states = states;
}

static void read_hotkeys() {
    struct button *last_autofire = NULL;
    uint8_t active_type = 0;
    uint8_t update_state = 0;
    for (struct hotkey *h = hotkeys; h->button_mask; h++) {
        if ((current_button_states & h->button_mask) == h->button_mask) {
            // Don't process more than one type of hotkey at a time
            // This is to prevent macro programming from also triggering
            // autofire
            if (active_type == 0) {
                active_type = h->type;
            } else if (active_type != h->type) {
                h->pressed_frames = 0;
                continue;
            }
            // Check hotkey
            if (h->pressed_frames == 0) {
                h->pressed_frames = current_frame;
            } else if (h->pressed_frames > current_frame) {
                // Do nothing; suppressed
            } else if (ELAPSED(h->pressed_frames) >= HOTKEY_THRESHOLD) {
                switch (h->type) {
                case HOTKEY_AUTOFIRE:
                    if (last_autofire != NULL) {
                        h->autofire->state &= ~STATE_AUTOFIRE;
                        h->autofire->state |= last_autofire->state & STATE_AUTOFIRE;
                    } else {
                        h->autofire->state ^= STATE_AUTOFIRE;
                        last_autofire = h->autofire;
                    }
                    break;
                case HOTKEY_MACRO:
                    h->macro_mask = current_button_states & SN_MASK_ABXY;
                    break;
                }
                update_state = 1;
                h->pressed_frames = UINT32_MAX;
            }
        } else {
            h->pressed_frames = 0;
        }
    }
    if (update_state) {
        fast_blink_frames = current_frame + STATUS_FAST_DURATION;
#ifdef SAVE_STATE
        write_state();
#endif // SAVE_STATE
    }
}

#ifdef SAVE_STATE
static void read_state() {
    eeprom_busy_wait();
    uint32_t state = eeprom_read_dword(EEPROM_ADDR_STATE);
    if ((state & 0xffff0000) != MAGIC_MASK) {
        return;
    }
    // Restore autofire
    buttons[SN_INDEX_A].state |= (state & 0x1) ? STATE_AUTOFIRE : 0;
    buttons[SN_INDEX_B].state |= (state & 0x2) ? STATE_AUTOFIRE : 0;
    buttons[SN_INDEX_X].state |= (state & 0x4) ? STATE_AUTOFIRE : 0;
    buttons[SN_INDEX_Y].state |= (state & 0x8) ? STATE_AUTOFIRE : 0;
}

static void write_state() {
    uint32_t state = MAGIC_MASK;
    state |= (buttons[SN_INDEX_A].state & STATE_AUTOFIRE) ? 0x1 : 0;
    state |= (buttons[SN_INDEX_B].state & STATE_AUTOFIRE) ? 0x2 : 0;
    state |= (buttons[SN_INDEX_X].state & STATE_AUTOFIRE) ? 0x4 : 0;
    state |= (buttons[SN_INDEX_Y].state & STATE_AUTOFIRE) ? 0x8 : 0;
    cli();
    eeprom_busy_wait();
    eeprom_write_dword(EEPROM_ADDR_STATE, state);
    sei();
}
#endif // SAVE_STATE

int main() {
    init_pins();
#ifdef SAVE_STATE
    read_state();
#endif // SAVE_STATE
    init_interrupts();

    for (;;) {
        current_button_states = ir_button_states;
        current_frame = ir_frames_elapsed;

        read_hotkeys();

        // Unset global autofire state (will update if set in button loop)
        controller_state &= ~STATE_AUTOFIRE;

        // Update buttons
        uint16_t button_mask = 1;
        volatile struct pin *p;
        struct button *b;

        // FIXME: static reference to array index
        if (current_button_states & SN_MASK_LB) {
            current_button_states |= hotkeys[0].macro_mask;
        }
        if (current_button_states & SN_MASK_RB) {
            current_button_states |= hotkeys[1].macro_mask;
        }

        for (p = output_pins, b = buttons; p->ddr; p++, b++) {
            // Update global autofire state
            controller_state |= b->state & STATE_AUTOFIRE;
            // write data to output pins
            if (current_button_states & button_mask) {
                if (b->pressed_frames == 0) {
                    b->pressed_frames = current_frame;
                }
                if ((b->state & STATE_AUTOFIRE) == 0
                    || AUTOFIRE_INTERVAL(b) == 0) {
                    *p->port &= ~p->mask;
                } else {
                    *p->port |= p->mask;
                }
            } else {
                b->pressed_frames = 0;
                *p->port |= p->mask;
            }
            button_mask <<= 1;
        }

        // Status LED
        if (fast_blink_frames > current_frame
            && (current_frame % STATUS_FAST_BLINK_PERIOD < STATUS_FAST_BLINK_PERIOD / 2)) {
            LED_OCR = STATUS_LED_ON;
        } else if ((controller_state & STATE_AUTOFIRE)
             && (current_frame % STATUS_AF_BLINK_PERIOD < STATUS_AF_BLINK_PERIOD / 2)) {
            LED_OCR = STATUS_LED_ON;
        } else {
            LED_OCR = 0x00;
        }
    }

    return 0;
}
