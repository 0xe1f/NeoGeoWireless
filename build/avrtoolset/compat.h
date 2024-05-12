/**
 * Copyright (c) Akop Karapetyan - All Rights Reserved
 *
 * Unauthorized copying and distribution of this file, via any medium is
 * strictly prohibited.
 *
 * See file "LICENSE" for additional information.
 **/

#ifndef COMPAT_H
#define COMPAT_H

#include <stdint.h>

extern uint8_t map_mask[];
extern volatile uint8_t *map_ddr[];
extern volatile uint8_t *map_port[];
extern volatile uint8_t *map_pin[];

#define SPI_MOSI      PB3
#define SPI_MOSI_PORT PORTB
#define SPI_MOSI_PIN  PINB
#define SPI_MOSI_DDR  DDRB

#define SPI_MISO      PB4
#define SPI_MISO_PORT PORTB
#define SPI_MISO_PIN  PINB
#define SPI_MISO_DDR  DDRB

#define SPI_SCK       PB5
#define SPI_SCK_PORT  PORTB
#define SPI_SCK_PIN   PINB
#define SPI_SCK_DDR   DDRB

#define CPT_PB0 8
#define CPT_PB1 9
#define CPT_PB2 10
#define CPT_PB3 11
#define CPT_PB4 12
#define CPT_PB5 13
#define CPT_PB6 20
#define CPT_PB7 21
#define CPT_PC0 14
#define CPT_PC1 15
#define CPT_PC2 16
#define CPT_PC3 17
#define CPT_PC4 18
#define CPT_PC5 19
#define CPT_PC6 22
#define CPT_PD0 0
#define CPT_PD1 1
#define CPT_PD2 2
#define CPT_PD3 3
#define CPT_PD4 4
#define CPT_PD5 5
#define CPT_PD6 6
#define CPT_PD7 7

#define CPT_PMAX CPT_PC6
#define CPT_BAD_REG ((uint8_t*)0)

#define CPT_INPUT        0
#define CPT_OUTPUT       1
#define CPT_INPUT_PULLUP 2

#define CPT_LOW  0
#define CPT_HIGH 1

void compat_init();
uint32_t compat_ms();
uint32_t compat_us();
void compat_delay_ms(uint32_t ms);
void compat_delay_us(uint32_t us);

#define pin_to_mask(pin) (((uint8_t)pin <= CPT_PMAX) ? compat_map_mask[(uint8_t)pin] : 0)
#define pin_to_ddr(pin) (((uint8_t)pin <= CPT_PMAX) ? compat_map_ddr[(uint8_t)pin] : CPT_BAD_REG)
#define pin_to_port(pin) (((uint8_t)pin <= CPT_PMAX) ? compat_map_port[(uint8_t)pin] : CPT_BAD_REG)
#define pin_to_pin(pin) (((uint8_t)pin <= CPT_PMAX) ? compat_map_pin[(uint8_t)pin] : CPT_BAD_REG)

// Arduino-ish

#define A0 CPT_PC0
#define A1 CPT_PC1
#define A2 CPT_PC2
#define A3 CPT_PC3
#define A4 CPT_PC4
#define A5 CPT_PC5

#define INPUT        CPT_INPUT
#define OUTPUT       CPT_OUTPUT
#define INPUT_PULLUP CPT_INPUT_PULLUP

#define LOW  CPT_LOW
#define HIGH CPT_HIGH

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
uint8_t digitalRead(uint8_t pin);

#define millis() compat_ms()
#define micros() compat_us()
#define delay(x) compat_delay_ms(x)
#define delayMicroseconds(x) compat_delay_us(x)

#endif // COMPAT_H
