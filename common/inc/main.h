#ifndef _MAIN_H
#define _MAIN_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>

// Memory addresses
#define BOOT_ADDR       0x08000000
#define LOADER_ADDR     0x08004000
#define UPDATER_ADDR    0x08008000
#define APP_ADDR        0x08020000
#define IMAGE_HDR_SIZE  0x200

// LED Pins (STM32F4-Discovery)
#define LED_GREEN_PIN   GPIO_PIN_12  // PD12
#define LED_ORANGE_PIN  GPIO_PIN_13  // PD13
#define LED_RED_PIN     GPIO_PIN_14  // PD14
#define LED_BLUE_PIN    GPIO_PIN_15  // PD15
#define LED_PORT        GPIOD

// UART definitions
#define UART_TX_PIN     GPIO_PIN_2   // PA2
#define UART_RX_PIN     GPIO_PIN_3   // PA3
#define UART_PORT       GPIOA
#define UART_INSTANCE   USART2

// Timeouts
#define BOOT_TIMEOUT_MS       10000
#define UART_TIMEOUT_MS       1000
#define XMODEM_TIMEOUT_MS     5000

// Utility macros
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#endif // _MAIN_H