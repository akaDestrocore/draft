#ifndef _MAIN_H
#define _MAIN_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_gpio.h"
#include <stdint.h>

// Memory addresses
#define BOOT_ADDR       0x08000000
#define LOADER_ADDR     0x08004000
#define UPDATER_ADDR    0x08008000
#define APP_ADDR        0x08020000
#define IMAGE_HDR_SIZE  0x200

// LED Pins
#define GREEN_Pin LL_GPIO_PIN_12
#define GREEN_GPIO_Port GPIOD
#define ORANGE_Pin LL_GPIO_PIN_13
#define ORANGE_GPIO_Port GPIOD
#define RED_Pin LL_GPIO_PIN_14
#define RED_GPIO_Port GPIOD
#define BLUE_Pin LL_GPIO_PIN_15
#define BLUE_GPIO_Port GPIOD

// UART definitions
#define UART_TX_PIN     GPIO_PIN_2   // PA2
#define UART_RX_PIN     GPIO_PIN_3   // PA3
#define UART_PORT       GPIOA
#define UART_INSTANCE   USART2

// Timeouts
#define BOOT_TIMEOUT_MS       10000
#define UART_TIMEOUT_MS       1000
#define XMODEM_TIMEOUT_MS     5000

#endif // _MAIN_H