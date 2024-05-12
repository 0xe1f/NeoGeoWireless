/**
 * Copyright (c) Akop Karapetyan - All Rights Reserved
 *
 * Unauthorized copying and distribution of this file, via any medium is
 * strictly prohibited.
 *
 * See file "LICENSE" for additional information.
 **/

#include "spi.h"

#include <avr/io.h>
#include "compat.h"

void spi_init() {
    SPI_MOSI_DDR |= _BV(SPI_MOSI);  // output on MOSI
    SPI_MISO_PORT |= _BV(SPI_MISO); // pullup on MISO
    SPI_SCK_DDR |= _BV(SPI_SCK);    // output on SCK
    SPCR |= _BV(SPR1) | _BV(MSTR) | _BV(SPE); // div 64; clkmaster; enable
}

uint8_t spi_transfer(uint8_t b) {
    SPDR = b;
    loop_until_bit_is_set(SPSR, SPIF);
    return SPDR;
}
