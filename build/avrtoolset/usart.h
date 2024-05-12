/**
 * Copyright (c) Akop Karapetyan - All Rights Reserved
 *
 * Unauthorized copying and distribution of this file, via any medium is
 * strictly prohibited.
 *
 * See file "LICENSE" for additional information.
 **/

#ifndef COMPAT_USART_H
#define COMPAT_USART_H

#include <stdint.h>

void usart_init();
void usart_send_byte(uint8_t b);
void usart_send_string(const char *str);

#endif // COMPAT_USART_H
