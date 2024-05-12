/**
 * Copyright (c) Akop Karapetyan - All Rights Reserved
 *
 * Unauthorized copying and distribution of this file, via any medium is
 * strictly prohibited.
 *
 * See file "LICENSE" for additional information.
 **/

#ifndef COMPAT_SPI_H
#define COMPAT_SPI_H

#include <stdint.h>

void spi_init();
uint8_t spi_transfer(uint8_t b);

#endif // COMPAT_SPI_H
