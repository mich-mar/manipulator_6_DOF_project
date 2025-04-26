#ifndef UART_UTILS_H
#define UART_UTILS_H

#include "stm32l4xx_hal.h"     // Dostosuj do odpowiedniej serii STM32
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "i2c.h"
#include "usart.h"

// Funkcja pomocnicza do wysyłania tekstu przez UART2
static inline void uart_print(char *str)
{
    if (str != NULL) {
        HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
    }
}

#endif // UART_UTILS_H
