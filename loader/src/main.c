/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for the Loader component
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "image.h"
#include "bootloader.h"
#include "crc.h"
#include "flash.h"
#include "transport.h"
#include "uart_transport.h"
#include "xmodem.h"
#include "ring_buffer.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Time in which Enter key press is ignored (because 
// ExtraPuTTY sends Enter after XMODEM)
#define ENTER_BLOCK_AFTER_UPDATE_MS 3000
static uint32_t ENTER_BLOCKED_UNTIL = 0;

// Added state for PostXmodem recovery
typedef enum {
    POST_XMODEM_INITIAL,
    POST_XMODEM_RECOVERING,
    POST_XMODEM_COMPLETE
} PostXmodemState_t;

// Timeouts
#define BOOT_TIMEOUT_MS 10000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Flash memory addresses
#define BOOT_ADDR       0x08000000
#define LOADER_ADDR     0x08004000
#define UPDATER_ADDR    0x08008000
#define APP_ADDR        0x08020000
#define IMAGE_HDR_SIZE  0x200

// Boot options
typedef enum {
    BOOT_OPTION_NONE,
    BOOT_OPTION_APPLICATION,
    BOOT_OPTION_UPDATER,
    BOOT_OPTION_LOADER,
    BOOT_OPTION_SELECT_UPDATE_TARGET
} BootOption_t;

// Shared memory structure
typedef struct {
    uint8_t target_image_type;
    bool update_requested;
    uint8_t reserved[62];
} SharedMemory_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Shared memory
#define SHARED_MEMORY_SECTION __attribute__((section(".shared_memory")))
SHARED_MEMORY_SECTION SharedMemory_t shared_memory = {0};

// Image header
#define IMAGE_HEADER_SECTION __attribute__((section(".image_hdr")))
IMAGE_HEADER_SECTION ImageHeader_t loader_header = {
    .image_magic = IMAGE_MAGIC_LOADER,
    .image_hdr_version = IMAGE_VERSION_CURRENT,
    .image_type = IMAGE_TYPE_LOADER,
    .version_major = 1,
    .version_minor = 0,
    .version_patch = 0,
    ._padding = 0,
    .vector_addr = LOADER_ADDR + IMAGE_HDR_SIZE,
    .crc = 0,  // Will be calculated during build
    .data_size = 0, // Will be calculated during build
};

// Firmware size symbol from linker
extern uint32_t __firmware_size;

// Transport structure
Transport_t transport;

// XMODEM manager
XmodemManager_t xmodem_manager;

// Boot banner and options strings
static const char* BOOT_BANNER = "\r\n\
\x1B[96mxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\r\n\
xxxxxxxx  xxxxxxxxxxxxxxxxxxxx  xxxxxxxxx\r\n\
xxxxxxxxxx  xxxxxxxxxxxxxxxxx  xxxxxxxxxx\r\n\
xxxxxx  xxx  xxxxxxxxxxxxxxx  xx   xxxxxx\r\n\
xxxxxxxx  xx  xxxxxxxxxxxxx  xx  xxxxxxxx\r\n\
xxxx  xxx   xxxxxxxxxxxxxxxxx  xxx  xxxxx\r\n\
xxxxxx    xxxx  xxxxxxxx  xxx     xxxxxxx\r\n\
xxxxxxxx xxxxx xx      xx xxxx  xxxxxxxxx\r\n\
xxxx     xxxxx   xx  xx   xxxxx     xxxxx\r\n\
xxxxxxxx xxxxxxxxxx  xxxxxxxxxx  xxxxxxxx\r\n\
xxxxx    xxxxxx  xx  xx  xxxxxx    xxxxxx\r\n\
xxxxxxxx  xxxx xxxx  xxxx xxxxx xxxxxxxxx\r\n\
xxxxxxx    xxx  xxx  xxx  xxx    xxxxxxxx\r\n\
xxxxxxxxxx   xxxxxx  xxxxxx   xxxxxxxxxxx\r\n\
xxxxxxxxxxxxxx             xxxxxxxxxxxxxx\r\n\
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\x1B[0m\r\n\r\n";

static const char* BOOT_OPTIONS_STR = "\x1B[96mPress \x1B[93m'Enter'\x1B[0m\x1B[96m to boot application\r\n\
Press \x1B[93m'I'\x1B[0m\x1B[96m to get information about system state\r\n\
Press \x1B[93m'F'\x1B[0m\x1B[96m to update firmware using XMODEM(CRC)\r\n\
Press \x1B[93m'U'\x1B[0m\x1B[96m to enter updater\x1B[0m\r\n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
// UI functions
void clear_screen(void);
void display_menu(void);
char* itoa(uint32_t value);
char* to_hex(uint32_t value);
bool check_application_valid(void);
bool check_updater_valid(void);
bool is_enter_blocked(uint32_t current_time);
void block_enter_temporarily(uint32_t current_time);
void send_cancel_sequence(void);
uint32_t recover_from_xmodem(void);

// Boot functions
void boot_to_application(void);
void boot_to_updater(void);

// System functions
void setup_system_clock(void);
void enable_peripherals(void);
void setup_gpio_pins(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// UI functions
void clear_screen(void) {
    const char* cmd = "\x1B[2J\x1B[1;1H";
    transport_send(&transport, (const uint8_t*)cmd, strlen(cmd));
    
    for (int i = 0; i < 5; i++) {
        transport_process(&transport);
    }
}

void display_menu(void) {
    clear_screen();
    
    transport_send(&transport, (const uint8_t*)BOOT_BANNER, strlen(BOOT_BANNER));

    const char* header_part1 = "\x1B[91m-- Loader v";
    transport_send(&transport, (const uint8_t*)header_part1, strlen(header_part1));
    transport_send(&transport, (const uint8_t*)itoa(loader_header.version_major), strlen(itoa(loader_header.version_major)));
    transport_send(&transport, (const uint8_t*)".", 1);
    transport_send(&transport, (const uint8_t*)itoa(loader_header.version_minor), strlen(itoa(loader_header.version_minor)));
    transport_send(&transport, (const uint8_t*)".", 1);
    transport_send(&transport, (const uint8_t*)itoa(loader_header.version_patch), strlen(itoa(loader_header.version_patch)));
    transport_send(&transport, (const uint8_t*)" --\x1B[0m\r\n\r\n", 11);
    
    transport_send(&transport, (const uint8_t*)BOOT_OPTIONS_STR, strlen(BOOT_OPTIONS_STR));
    
    const char* timeout_msg = "\x1B[92mWill boot automatically in 10 seconds\x1B[0m\r\n";
    transport_send(&transport, (const uint8_t*)timeout_msg, strlen(timeout_msg));
    
    for (int i = 0; i < 150; i++) {
        transport_process(&transport);
    }
}

char* itoa(uint32_t value) {
    static char buffer[16];
    
    if (value == 0) {
        buffer[0] = '0';
        buffer[1] = '\0';
        return buffer;
    }
    
    int i = 0;
    while (value > 0 && i < 15) {
        buffer[i++] = '0' + (value % 10);
        value /= 10;
    }
    
    // Reverse the string
    int j = 0;
    int k = i - 1;
    while (j < k) {
        char temp = buffer[j];
        buffer[j] = buffer[k];
        buffer[k] = temp;
        j++;
        k--;
    }
    
    buffer[i] = '\0';
    return buffer;
}

char* to_hex(uint32_t value) {
    static char buffer[16];
    
    if (value == 0) {
        buffer[0] = '0';
        buffer[1] = '\0';
        return buffer;
    }
    
    int i = 0;
    while (value > 0 && i < 15) {
        uint8_t digit = value & 0xF;
        buffer[i++] = digit < 10 ? '0' + digit : 'a' + (digit - 10);
        value >>= 4;
    }
    
    // Reverse the string
    int j = 0;
    int k = i - 1;
    while (j < k) {
        char temp = buffer[j];
        buffer[j] = buffer[k];
        buffer[k] = temp;
        j++;
        k--;
    }
    
    buffer[i] = '\0';
    return buffer;
}

bool check_application_valid(void) {
    BootConfig_t boot_config = {
        .app_addr = APP_ADDR,
        .updater_addr = UPDATER_ADDR,
        .loader_addr = LOADER_ADDR,
        .image_hdr_size = IMAGE_HDR_SIZE
    };
    
    return is_firmware_valid(APP_ADDR, &boot_config);
}

bool check_updater_valid(void) {
    BootConfig_t boot_config = {
        .app_addr = APP_ADDR,
        .updater_addr = UPDATER_ADDR,
        .loader_addr = LOADER_ADDR,
        .image_hdr_size = IMAGE_HDR_SIZE
    };
    
    return is_firmware_valid(UPDATER_ADDR, &boot_config);
}

bool is_enter_blocked(uint32_t current_time) {
    return ENTER_BLOCKED_UNTIL > current_time;
}

void block_enter_temporarily(uint32_t current_time) {
    ENTER_BLOCKED_UNTIL = current_time + ENTER_BLOCK_AFTER_UPDATE_MS;
}

void send_cancel_sequence(void) {
    uint8_t cancel_bytes[3] = {XMODEM_CAN, XMODEM_CAN, XMODEM_CAN};
    transport_send(&transport, cancel_bytes, 3);

    while (!uart_transport_is_tx_complete()) {
        transport_process(&transport);
    }

    uint32_t start_time = HAL_GetTick();
    while (HAL_GetTick() - start_time < 1000) {
        transport_process(&transport);
    }

    for (int i = 0; i < 30; i++) {
        transport_process(&transport);
    }
}

uint32_t recover_from_xmodem(void) {
    uart_transport_clear_rx();
    uint32_t start_time = HAL_GetTick();
    while (HAL_GetTick() - start_time < 3000) {
        transport_process(&transport);
    }
    
    // Clear screen
    clear_screen();
    HAL_Delay(500);

    for (int i = 0; i < 150; i++) {
        transport_process(&transport);
    }
    
    // Block 'Enter' for proper functionality in ExtraPuTTY
    block_enter_temporarily(HAL_GetTick());
    
    // Reset autoboot timer
    uint32_t new_autoboot_time = HAL_GetTick();
    
    // Display main menu
    display_menu();
    
    for (int i = 0; i < 150; i++) {
        transport_process(&transport);
    }
    
    while (!uart_transport_is_tx_complete()) {
        transport_process(&transport);
    }
    
    // Return new time for timeout
    return new_autoboot_time;
}

// Boot functions
void boot_to_application(void) {
    BootConfig_t boot_config = {
        .app_addr = APP_ADDR,
        .updater_addr = UPDATER_ADDR,
        .loader_addr = LOADER_ADDR,
        .image_hdr_size = IMAGE_HDR_SIZE
    };
    
    // Wait for UART to finish sending
    while (!uart_transport_is_tx_complete()) {
        transport_process(&transport);
    }
    
    uart_transport_clear_rx();
    
    // Boot to application
    boot_application(&boot_config);
    
    // Should never reach here
    Error_Handler();
}

void boot_to_updater(void) {
    BootConfig_t boot_config = {
        .app_addr = APP_ADDR,
        .updater_addr = UPDATER_ADDR,
        .loader_addr = LOADER_ADDR,
        .image_hdr_size = IMAGE_HDR_SIZE
    };
    
    // Wait for UART to finish sending
    while (!uart_transport_is_tx_complete()) {
        transport_process(&transport);
    }
    
    uart_transport_clear_rx();
    
    // Boot to updater
    boot_updater(&boot_config);
    
    // Should never reach here
    Error_Handler();
}

// System functions
void setup_system_clock(void) {
    // Enable PWR clock
    __HAL_RCC_PWR_CLK_ENABLE();
    
    // Set voltage scale for maximum frequency
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    
    // Configure flash latency
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_5);
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
    __HAL_FLASH_DATA_CACHE_ENABLE();
    
    // Enable HSE
    __HAL_RCC_HSE_CONFIG(RCC_HSE_ON);
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET) {
        // Wait for HSE
    }
    
    // Configure PLL (90 MHz)
    RCC_PLLInitTypeDef pll;
    pll.PLLState = RCC_PLL_ON;
    pll.PLLSource = RCC_PLLSOURCE_HSE;
    pll.PLLM = 4;
    pll.PLLN = 90;
    pll.PLLP = RCC_PLLP_DIV2;
    pll.PLLQ = 4;
    HAL_RCC_OscConfig((RCC_OscInitTypeDef*)&pll);
    
    // Configure buses
    RCC_ClkInitTypeDef clk;
    clk.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

void enable_peripherals(void) {
    // Enable GPIO clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    
    // Enable USART2 clock
    __HAL_RCC_USART2_CLK_ENABLE();
    
    // Enable CRC clock
    __HAL_RCC_CRC_CLK_ENABLE();
}

void setup_gpio_pins(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Configure UART pins (PA2=TX, PA3=RX)
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Configure LED pins (PD12-PD15)
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

// UART interrupt handler
void USART2_IRQHandler(void) {
    uart_transport_irq_handler();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  
  /* USER CODE BEGIN 2 */
  // Initialize firmware size in the header
  uint32_t firmware_size = (uint32_t)&__firmware_size;
  loader_header.data_size = firmware_size;
  
  // Initialize UART transport
  UARTTransport_Config_t uart_config = {
    .usart = USART2,
    .baudrate = 115200,
    .timeout = 1000,
    .use_xmodem = 1,
    .app_addr = APP_ADDR,
    .updater_addr = UPDATER_ADDR,
    .loader_addr = LOADER_ADDR,
    .image_hdr_size = IMAGE_HDR_SIZE
  };
  
  // Initialize transport layer with UART
  if (transport_init(&transport, TRANSPORT_UART, &uart_config) != 0) {
    Error_Handler();
  }
  
  // Initialize XMODEM
  XmodemConfig_t xmodem_config = {
    .app_addr = APP_ADDR,
    .updater_addr = UPDATER_ADDR,
    .loader_addr = LOADER_ADDR,
    .image_hdr_size = IMAGE_HDR_SIZE
  };
  xmodem_init(&xmodem_manager, &xmodem_config);
  
  // USART2 interrupt
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  
  // Display boot menu
  display_menu();
  
  // Main variables
  BootOption_t boot_option = BOOT_OPTION_NONE;
  uint32_t autoboot_timer = HAL_GetTick();
  bool update_in_progress = false;
  uint32_t firmware_target = APP_ADDR;
  uint32_t led_toggle_time = HAL_GetTick();
  PostXmodemState_t post_xmodem_state = POST_XMODEM_COMPLETE;
  bool xmodem_error_occurred = false;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Process transport data
    transport_process(&transport);
    
    // Blink LED
    uint32_t current_time = HAL_GetTick();
    if (current_time - led_toggle_time >= 500) {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // Toggle green LED
        led_toggle_time = current_time;
    }
    
    // Post-XMODEM recovery
    if (post_xmodem_state == POST_XMODEM_RECOVERING) {
        autoboot_timer = recover_from_xmodem();
        post_xmodem_state = POST_XMODEM_COMPLETE;
        update_in_progress = false;
        boot_option = BOOT_OPTION_NONE;
        continue;
    }
    
    // Process user input if not in update mode
    if (!update_in_progress) {
        uint8_t byte = 0;
        if (transport_receive(&transport, &byte, 1) > 0) {
            // Reset autoboot timer on any key press
            autoboot_timer = current_time;
            
            switch (byte) {
                case 'U':
                case 'u':
                    // Boot to updater
                    if (check_updater_valid()) {
                        const char* msg = "\x1B[36m\r\n Booting updater...\x1B[0m\r\n";
                        transport_send(&transport, (const uint8_t*)msg, strlen(msg));
                        boot_option = BOOT_OPTION_UPDATER;
                    } else {
                        const char* msg = "\x1B[31m\r\nValid updater not found!\x1B[0m\r\n";
                        transport_send(&transport, (const uint8_t*)msg, strlen(msg));
                        HAL_Delay(1500);
                        display_menu();
                    }
                    break;
                    
                case 'F':
                case 'f':
                    // Start firmware update
                    clear_screen();
                    const char* update_msg = "\r\n\x1B[92mUpdate firmware using XMODEM - select target:\x1B[0m\r\n\
\x1B[93m[\x1B[32m1\x1B[93m] \x1B[96m- Updater\x1B[0m\r\n\
\x1B[93m[\x1B[32m2\x1B[93m] \x1B[96m- Application\x1B[0m\r\n";
                    transport_send(&transport, (const uint8_t*)update_msg, strlen(update_msg));
                    boot_option = BOOT_OPTION_SELECT_UPDATE_TARGET;
                    break;
                    
                case '1':
                    if (boot_option == BOOT_OPTION_SELECT_UPDATE_TARGET) {
                        // Update updater
                        clear_screen();
                        const char* updater_msg = "\x1B[92mUpdating updater...\x1B[0m\r\n\r\n\
\x1B[96mSend file using XMODEM protocol with CRC-16. \x1B[0m\r\n\
\x1B[33mIf menu doesn't load after update is over, please press \x1B[96m'Esc'\x1B[0m\r\n";
                        transport_send(&transport, (const uint8_t*)updater_msg, strlen(updater_msg));
                        
                        firmware_target = UPDATER_ADDR;
                        uart_transport_xmodem_receive(firmware_target);
                        update_in_progress = true;
                        boot_option = BOOT_OPTION_NONE;
                        
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   // Green - system alive
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);   // Orange - XMODEM active
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // Red - no error
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); // Blue - no data yet
                    } else {
                        display_menu();
                    }
                    break;
                    
                case '2':
                    if (boot_option == BOOT_OPTION_SELECT_UPDATE_TARGET) {
                        // Update application
                        clear_screen();
                        const char* app_msg = "\x1B[92mUpdating application...\x1B[0m\r\n\r\n\
\x1B[96mSend file using XMODEM protocol with CRC-16. \x1B[0m\r\n\
\x1B[33mIf menu doesn't load after update is over, please press \x1B[96m'Esc'\x1B[0m\r\n";
                        transport_send(&transport, (const uint8_t*)app_msg, strlen(app_msg));
                        
                        firmware_target = APP_ADDR;
                        uart_transport_xmodem_receive(firmware_target);
                        update_in_progress = true;
                        boot_option = BOOT_OPTION_NONE;
                        
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   // Green - system alive
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);   // Orange - XMODEM active
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // Red - no error
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); // Blue - no data yet
                    } else {
                        display_menu();
                    }
                    break;
                    
                case 'I':
                case 'i':
                    // Show system information
                    clear_screen();
                    const char* info_header = "\x1B[91m\r\n--- System Information ---\x1B[0m\r\n\r\n";
                    transport_send(&transport, (const uint8_t*)info_header, strlen(info_header));
                    
                    // Loader
                    const char* loader_label = "\x1B[96mLoader (this image) : \x1B[0m";
                    transport_send(&transport, (const uint8_t*)loader_label, strlen(loader_label));
                    
                    ImageHeader_t header;
                    BootConfig_t boot_config = {
                        .app_addr = APP_ADDR,
                        .updater_addr = UPDATER_ADDR,
                        .loader_addr = LOADER_ADDR,
                        .image_hdr_size = IMAGE_HDR_SIZE
                    };
                    
                    if (get_firmware_header(LOADER_ADDR, &boot_config, &header)) {
                        const char* valid_msg = "\x1B[32mValid\x1B[0m\r\n";
                        transport_send(&transport, (const uint8_t*)valid_msg, strlen(valid_msg));
                        
                        // Version
                        const char* version_label = "\x1B[92m  Version: \x1B[93m";
                        transport_send(&transport, (const uint8_t*)version_label, strlen(version_label));
                        transport_send(&transport, (const uint8_t*)itoa(header.version_major), strlen(itoa(header.version_major)));
                        transport_send(&transport, (const uint8_t*)".", 1);
                        transport_send(&transport, (const uint8_t*)itoa(header.version_minor), strlen(itoa(header.version_minor)));
                        transport_send(&transport, (const uint8_t*)".", 1);
                        transport_send(&transport, (const uint8_t*)itoa(header.version_patch), strlen(itoa(header.version_patch)));
                        transport_send(&transport, (const uint8_t*)"\x1B[0m\r\n", 7);
                        
                        // Vector table
                        const char* vector_label = "\x1B[92m  Vector table: \x1B[93m0x";
                        transport_send(&transport, (const uint8_t*)vector_label, strlen(vector_label));
                        transport_send(&transport, (const uint8_t*)to_hex(header.vector_addr), strlen(to_hex(header.vector_addr)));
                        transport_send(&transport, (const uint8_t*)"\x1B[0m\r\n", 7);
                        
                        // Data size
                        const char* size_label = "\x1B[92m  Data size: \x1B[93m";
                        transport_send(&transport, (const uint8_t*)size_label, strlen(size_label));
                        transport_send(&transport, (const uint8_t*)itoa(header.data_size), strlen(itoa(header.data_size)));
                        transport_send(&transport, (const uint8_t*)" bytes\x1B[0m\r\n", 14);
                        
                        // CRC
                        const char* crc_label = "\x1B[92m  CRC: \x1B[93m0x";
                        transport_send(&transport, (const uint8_t*)crc_label, strlen(crc_label));
                        transport_send(&transport, (const uint8_t*)to_hex(header.crc), strlen(to_hex(header.crc)));
                        transport_send(&transport, (const uint8_t*)"\x1B[0m\r\n", 7);
                        
                        // Address
                        const char* addr_label = "\x1B[92m  Address: \x1B[93m0x";
                        transport_send(&transport, (const uint8_t*)addr_label, strlen(addr_label));
                        transport_send(&transport, (const uint8_t*)to_hex(LOADER_ADDR), strlen(to_hex(LOADER_ADDR)));
                        transport_send(&transport, (const uint8_t*)"\x1B[0m\r\n", 7);
                    } else {
                        const char* invalid_msg = "\x1B[31mInvalid or Not Found\x1B[0m\r\n";
                        transport_send(&transport, (const uint8_t*)invalid_msg, strlen(invalid_msg));
                    }
                    
                    transport_send(&transport, (const uint8_t*)"\r\n", 2);
                    
                    // Application
                    const char* app_label = "\x1B[96mApplication: \x1B[0m";
                    transport_send(&transport, (const uint8_t*)app_label, strlen(app_label));
                    
                    if (get_firmware_header(APP_ADDR, &boot_config, &header)) {
                        const char* valid_msg = "\x1B[32mValid\x1B[0m\r\n";
                        transport_send(&transport, (const uint8_t*)valid_msg, strlen(valid_msg));
                        
                        // Version
                        const char* version_label = "\x1B[92m  Version: \x1B[93m";
                        transport_send(&transport, (const uint8_t*)version_label, strlen(version_label));
                        transport_send(&transport, (const uint8_t*)itoa(header.version_major), strlen(itoa(header.version_major)));
                        transport_send(&transport, (const uint8_t*)".", 1);
                        transport_send(&transport, (const uint8_t*)itoa(header.version_minor), strlen(itoa(header.version_minor)));
                        transport_send(&transport, (const uint8_t*)".", 1);
                        transport_send(&transport, (const uint8_t*)itoa(header.version_patch), strlen(itoa(header.version_patch)));
                        transport_send(&transport, (const uint8_t*)"\x1B[0m\r\n", 7);
                        
                        // Vector table
                        const char* vector_label = "\x1B[92m  Vector table: \x1B[93m0x";
                        transport_send(&transport, (const uint8_t*)vector_label, strlen(vector_label));
                        transport_send(&transport, (const uint8_t*)to_hex(header.vector_addr), strlen(to_hex(header.vector_addr)));
                        transport_send(&transport, (const uint8_t*)"\x1B[0m\r\n", 7);
                        
                        // Data size
                        const char* size_label = "\x1B[92m  Data size: \x1B[93m";
                        transport_send(&transport, (const uint8_t*)size_label, strlen(size_label));
                        transport_send(&transport, (const uint8_t*)itoa(header.data_size), strlen(itoa(header.data_size)));
                        transport_send(&transport, (const uint8_t*)" bytes\x1B[0m\r\n", 14);
                        
                        // CRC
                        const char* crc_label = "\x1B[92m  CRC: \x1B[93m0x";
                        transport_send(&transport, (const uint8_t*)crc_label, strlen(crc_label));
                        transport_send(&transport, (const uint8_t*)to_hex(header.crc), strlen(to_hex(header.crc)));
                        transport_send(&transport, (const uint8_t*)"\x1B[0m\r\n", 7);
                        
                        // Address
                        const char* addr_label = "\x1B[92m  Address: \x1B[93m0x";
                        transport_send(&transport, (const uint8_t*)addr_label, strlen(addr_label));
                        transport_send(&transport, (const uint8_t*)to_hex(APP_ADDR), strlen(to_hex(APP_ADDR)));
                        transport_send(&transport, (const uint8_t*)"\x1B[0m\r\n", 7);
                    } else {
                        const char* invalid_msg = "\x1B[31mInvalid or Not Found\x1B[0m\r\n";
                        transport_send(&transport, (const uint8_t*)invalid_msg, strlen(invalid_msg));
                    }
                    
                    transport_send(&transport, (const uint8_t*)"\r\n", 2);
                    
                    // Updater
                    const char* updater_label = "\x1B[96mUpdater: \x1B[0m";
                    transport_send(&transport, (const uint8_t*)updater_label, strlen(updater_label));
                    
                    if (get_firmware_header(UPDATER_ADDR, &boot_config, &header)) {
                        const char* valid_msg = "\x1B[32mValid\x1B[0m\r\n";
                        transport_send(&transport, (const uint8_t*)valid_msg, strlen(valid_msg));
                        
                        // Version
                        const char* version_label = "\x1B[92m  Version: \x1B[93m";
                        transport_send(&transport, (const uint8_t*)version_label, strlen(version_label));
                        transport_send(&transport, (const uint8_t*)itoa(header.version_major), strlen(itoa(header.version_major)));
                        transport_send(&transport, (const uint8_t*)".", 1);
                        transport_send(&transport, (const uint8_t*)itoa(header.version_minor), strlen(itoa(header.version_minor)));
                        transport_send(&transport, (const uint8_t*)".", 1);
                        transport_send(&transport, (const uint8_t*)itoa(header.version_patch), strlen(itoa(header.version_patch)));
                        transport_send(&transport, (const uint8_t*)"\x1B[0m\r\n", 7);
                        
                        // Vector table
                        const char* vector_label = "\x1B[92m  Vector table: \x1B[93m0x";
                        transport_send(&transport, (const uint8_t*)vector_label, strlen(vector_label));
                        transport_send(&transport, (const uint8_t*)to_hex(header.vector_addr), strlen(to_hex(header.vector_addr)));
                        transport_send(&transport, (const uint8_t*)"\x1B[0m\r\n", 7);
                        
                        // Data size
                        const char* size_label = "\x1B[92m  Data size: \x1B[93m";
                        transport_send(&transport, (const uint8_t*)size_label, strlen(size_label));
                        transport_send(&transport, (const uint8_t*)itoa(header.data_size), strlen(itoa(header.data_size)));
                        transport_send(&transport, (const uint8_t*)" bytes\x1B[0m\r\n", 14);
                        
                        // CRC
                        const char* crc_label = "\x1B[92m  CRC: \x1B[93m0x";
                        transport_send(&transport, (const uint8_t*)crc_label, strlen(crc_label));
                        transport_send(&transport, (const uint8_t*)to_hex(header.crc), strlen(to_hex(header.crc)));
                        transport_send(&transport, (const uint8_t*)"\x1B[0m\r\n", 7);
                        
                        // Address
                        const char* addr_label = "\x1B[92m  Address: \x1B[93m0x";
                        transport_send(&transport, (const uint8_t*)addr_label, strlen(addr_label));
                        transport_send(&transport, (const uint8_t*)to_hex(UPDATER_ADDR), strlen(to_hex(UPDATER_ADDR)));
                        transport_send(&transport, (const uint8_t*)"\x1B[0m\r\n", 7);
                    } else {
                        const char* invalid_msg = "\x1B[31mInvalid or Not Found\x1B[0m\r\n";
                        transport_send(&transport, (const uint8_t*)invalid_msg, strlen(invalid_msg));
                    }
                    
                    transport_send(&transport, (const uint8_t*)"\r\n", 2);
                    
                    // System info
                    const char* sys_info = "\x1B[96mSystem Info:\x1B[0m\r\n";
                    transport_send(&transport, (const uint8_t*)sys_info, strlen(sys_info));
                    
                    const char* boot_timeout = "\x1B[92m  Boot timeout: \x1B[93m";
                    transport_send(&transport, (const uint8_t*)boot_timeout, strlen(boot_timeout));
                    transport_send(&transport, (const uint8_t*)itoa(BOOT_TIMEOUT_MS / 1000), strlen(itoa(BOOT_TIMEOUT_MS / 1000)));
                    transport_send(&transport, (const uint8_t*)" seconds\r\n", 10);
                    
                    const char* uptime = "\x1B[92m  System uptime: \x1B[93m";
                    transport_send(&transport, (const uint8_t*)uptime, strlen(uptime));
                    transport_send(&transport, (const uint8_t*)itoa(HAL_GetTick() / 1000), strlen(itoa(HAL_GetTick() / 1000)));
                    transport_send(&transport, (const uint8_t*)" seconds\x1B[0m\r\n", 14);
                    
                    const char* esc_prompt = "\r\n\x1B[96mPress \x1B[93m'Esc'\x1B[0m \x1B[96mto return to menu...\x1B[0m\r\n";
                    transport_send(&transport, (const uint8_t*)esc_prompt, strlen(esc_prompt));
                    
                    while (1) {
                        transport_process(&transport);
                        uint8_t key;
                        if (transport_receive(&transport, &key, 1) > 0) {
                            if (key == 27) { // ESC
                                autoboot_timer = HAL_GetTick();
                                break;
                            }
                            if (key != 0) {
                                // Reset autoboot on any key
                                autoboot_timer = HAL_GetTick();
                            }
                        }
                    }
                    
                    display_menu();
                    break;
                    
                case '\r':
                case '\n':
                    if (!is_enter_blocked(current_time)) {
                        if (check_application_valid()) {
                            const char* boot_msg = "\x1B[92m\r\n Booting application...\x1B[0m\r\n";
                            transport_send(&transport, (const uint8_t*)boot_msg, strlen(boot_msg));
                            boot_option = BOOT_OPTION_APPLICATION;
                        } else {
                            const char* not_found_msg = "\x1B[31m\r\nValid application not found!\x1B[0m\r\n";
                            transport_send(&transport, (const uint8_t*)not_found_msg, strlen(not_found_msg));
                            HAL_Delay(1500);
                            display_menu();
                        }
                    }
                    break;
                    
                case 27: // ESC
                    clear_screen();
                    display_menu();
                    break;
                    
                default:
                    if (boot_option == BOOT_OPTION_SELECT_UPDATE_TARGET) {
                        boot_option = BOOT_OPTION_NONE;
                        display_menu();
                    }
                    break;
            }
        }
    } else {
        // Handle XMODEM update - use the uart_transport_process
        transport_process(&transport);
        
        // Check XMODEM state
        XmodemState_t xmodem_state = uart_transport_xmodem_state();
        
        if (xmodem_state == XMODEM_STATE_COMPLETE) {
            update_in_progress = false;
            xmodem_error_occurred = false;
            
            // Small delay for flash
            HAL_Delay(100);
            
            // CRC verification
            if (!verify_firmware_crc(firmware_target, IMAGE_HDR_SIZE)) {
                const char* crc_fail_msg = "\r\n\x1B[31mCRC verification failed! Invalidating firmware.\x1B[0m\r\n";
                transport_send(&transport, (const uint8_t*)crc_fail_msg, strlen(crc_fail_msg));
                
                if (invalidate_firmware(firmware_target)) {
                    const char* invalidated_msg = "\r\n\x1B[93mFirmware invalidated successfully.\x1B[0m\r\n";
                    transport_send(&transport, (const uint8_t*)invalidated_msg, strlen(invalidated_msg));
                } else {
                    const char* invalid_fail_msg = "\r\n\x1B[31mFailed to invalidate firmware!\x1B[0m\r\n";
                    transport_send(&transport, (const uint8_t*)invalid_fail_msg, strlen(invalid_fail_msg));
                }
                
                xmodem_error_occurred = true;
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Red LED on
            } else {
                const char* crc_ok_msg = "\r\n\x1B[32mCRC verification successful!\x1B[0m\r\n";
                transport_send(&transport, (const uint8_t*)crc_ok_msg, strlen(crc_ok_msg));
            }
            
            post_xmodem_state = POST_XMODEM_RECOVERING;
        } else if (xmodem_state == XMODEM_STATE_ERROR && post_xmodem_state == POST_XMODEM_COMPLETE) {
            const char* error_msg = "\r\n\x1B[31mXMODEM transfer error. Aborting.\x1B[0m\r\n";
            transport_send(&transport, (const uint8_t*)error_msg, strlen(error_msg));
            
            xmodem_error_occurred = true;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Red LED on
            post_xmodem_state = POST_XMODEM_RECOVERING;
        }
    }
    
    // Handle boot options
    switch (boot_option) {
        case BOOT_OPTION_APPLICATION:
            if (check_application_valid()) {
                boot_to_application();
            } else {
                const char* validation_fail = "\r\n\x1B[31mApplication validation failed just before boot\x1B[0m\r\n";
                transport_send(&transport, (const uint8_t*)validation_fail, strlen(validation_fail));
                boot_option = BOOT_OPTION_NONE;
                HAL_Delay(1500);
                display_menu();
            }
            break;
            
        case BOOT_OPTION_UPDATER:
            if (check_updater_valid()) {
                boot_to_updater();
            } else {
                const char* validation_fail = "\r\n\x1B[31mUpdater validation failed just before boot\x1B[0m\r\n";
                transport_send(&transport, (const uint8_t*)validation_fail, strlen(validation_fail));
                boot_option = BOOT_OPTION_NONE;
                HAL_Delay(1500);
                display_menu();
            }
            break;
            
        default:
            break;
    }
    
    // Auto-boot timeout check
    if (!update_in_progress && 
        boot_option == BOOT_OPTION_NONE && 
        !is_enter_blocked(current_time) && 
        post_xmodem_state == POST_XMODEM_COMPLETE) {
        
        if (current_time - autoboot_timer >= BOOT_TIMEOUT_MS) {
            if (check_application_valid()) {
                const char* auto_boot_msg = "\x1B[93m\r\n Auto-boot timeout reached. Booting application...\x1B[0m\r\n";
                transport_send(&transport, (const uint8_t*)auto_boot_msg, strlen(auto_boot_msg));
                boot_option = BOOT_OPTION_APPLICATION;
            } else {
                const char* not_found_msg = "\x1B[31m\r\n Auto-boot timeout reached but valid application not found!\x1B[0m\r\n";
                transport_send(&transport, (const uint8_t*)not_found_msg, strlen(not_found_msg));
                autoboot_timer = current_time;
                HAL_Delay(1500);
                display_menu();
            }
        }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
 void SystemClock_Config(void)
 {
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
 
   /** Configure the main internal regulator output voltage
   */
   __HAL_RCC_PWR_CLK_ENABLE();
   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
 
   /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLM = 4;
   RCC_OscInitStruct.PLL.PLLN = 90;
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
   RCC_OscInitStruct.PLL.PLLQ = 4;
   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
   {
     Error_Handler();
   }
 
   /** Initializes the CPU, AHB and APB buses clocks
   */
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
 
   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
   {
     Error_Handler();
   }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
 static void MX_USART2_UART_Init(void)
 {
 
   /* USER CODE BEGIN USART2_Init 0 */
 
   /* USER CODE END USART2_Init 0 */
 
   LL_USART_InitTypeDef USART_InitStruct = {0};
 
   LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
 
   /* Peripheral clock enable */
   LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
 
   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
   /**USART2 GPIO Configuration
   PA2   ------> USART2_TX
   PA3   ------> USART2_RX
   */
   GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
   GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
   GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
   GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
   GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
   LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
   /* USART2 interrupt Init */
   NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
   NVIC_EnableIRQ(USART2_IRQn);
 
   /* USER CODE BEGIN USART2_Init 1 */
 
   /* USER CODE END USART2_Init 1 */
   USART_InitStruct.BaudRate = 115200;
   USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
   USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
   USART_InitStruct.Parity = LL_USART_PARITY_NONE;
   USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
   USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
   USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
   LL_USART_Init(USART2, &USART_InitStruct);
   LL_USART_ConfigAsyncMode(USART2);
   LL_USART_Enable(USART2);
   /* USER CODE BEGIN USART2_Init 2 */
    LL_USART_EnableIT_ERROR(USART2);
    LL_USART_EnableIT_RXNE(USART2);
   /* USER CODE END USART2_Init 2 */
 
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
 static void MX_CRC_Init(void)
 {
 
   /* USER CODE BEGIN CRC_Init 0 */
 
   /* USER CODE END CRC_Init 0 */
 
   /* Peripheral clock enable */
   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
 
   /* USER CODE BEGIN CRC_Init 1 */
 
   /* USER CODE END CRC_Init 1 */
   /* USER CODE BEGIN CRC_Init 2 */
 
   /* USER CODE END CRC_Init 2 */
 
 }

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
 static void MX_GPIO_Init(void)
 {
   LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
   /* USER CODE BEGIN MX_GPIO_Init_1 */
 
   /* USER CODE END MX_GPIO_Init_1 */
 
   /* GPIO Ports Clock Enable */
   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
 
   /**/
   LL_GPIO_ResetOutputPin(GPIOD, GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin);
 
   /**/
   GPIO_InitStruct.Pin = GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin;
   GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
   GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
   GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
   GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
   LL_GPIO_Init(GPIOD, &GPIO_InitStruct);
 
   /* USER CODE BEGIN MX_GPIO_Init_2 */
 
   /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // Toggle red LED
    HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */