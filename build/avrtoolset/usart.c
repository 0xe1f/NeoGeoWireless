/**
 * Copyright (c) Akop Karapetyan - All Rights Reserved
 *
 * Unauthorized copying and distribution of this file, via any medium is
 * strictly prohibited.
 *
 * See file "LICENSE" for additional information.
 **/

#include "usart.h"

#include <avr/io.h>
#include <util/setbaud.h>

void usart_init() {
    UBRR0H = UBRRH_VALUE; // defined in setbaud.h
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~_BV(U2X0);
#endif
    UCSR0B = _BV(TXEN0) | _BV(RXEN0); // enable transmitter/receiver
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8 data bits, 1 stop bit
}

void usart_send_byte(uint8_t b) {
    loop_until_bit_is_set(UCSR0A, UDRE0); // wait until UDRE0 is set
    UDR0 = b;
}

void usart_send_string(const char *str) {
    for (const char *s = str; *s; s++) {
        usart_send_byte(*s);
    }
}
