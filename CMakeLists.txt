cmake_minimum_required(VERSION 3.22)  

include("cmake/gcc-arm-none-eabi.cmake")

# Define main project
project(stm32f4_bootloader)
enable_language(C ASM)

# Find Python directly
find_program(PYTHON_EXECUTABLE 
    NAMES python3 python
    REQUIRED
)

message("Build type: " ${CMAKE_BUILD_TYPE})
message("Using Python: ${PYTHON_EXECUTABLE}")

# Common settings for all projects
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# MCU settings
set(MCU_FAMILY STM32F4xx)
set(MCU_MODEL STM32F407xx)

# Define startup files
set(BOOT_STARTUP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boot/startup/startup_stm32f407vgtx.s")
set(LOADER_STARTUP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/loader/startup/startup_stm32f407vgtx.s")
set(UPDATER_STARTUP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/updater/startup/startup_stm32f407vgtx.s")
set(APP_STARTUP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/application/startup/startup_stm32f407vgtx.s")

# Common CPU parameters
set(cpu_PARAMS 
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# Common compiler flags
set(COMMON_FLAGS
    ${cpu_PARAMS}
    -Wall
    -Wextra
    -Wno-unused-parameter
    -ffunction-sections
    -fdata-sections
    -fstack-usage
    -fno-builtin
    $<$<CONFIG:Debug>:-O0 -g3 -ggdb>
    $<$<CONFIG:Release>:-Os -g0>
)

# Common linker flags
set(COMMON_LINKER_FLAGS
    ${cpu_PARAMS}
    -Wl,-gc-sections
    -Wl,--print-memory-usage
    -Wl,--start-group
    -lc
    -lm
    -Wl,--end-group
    -Wl,-z,max-page-size=8
)

# Common include directories
set(COMMON_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/common/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/${MCU_FAMILY}_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/${MCU_FAMILY}_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/CMSIS/Device/ST/${MCU_FAMILY}/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/CMSIS/Include
)

# Common driver sources
set(COMMON_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c
)

# Common library sources
set(COMMON_LIB_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/common/src/flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/common/src/crc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/common/src/image.c
    ${CMAKE_CURRENT_SOURCE_DIR}/common/src/xmodem.c
)

# Create executables for each component
add_executable(boot)
add_executable(loader)
add_executable(updater)
add_executable(application)

# Function to configure common settings
function(configure_target target)
    message("Configuring target: ${target}")
    target_compile_options(${target} PRIVATE ${COMMON_FLAGS})
    target_include_directories(${target} PRIVATE ${COMMON_INCLUDES})
    
    # Base definitions
    target_compile_definitions(${target} PRIVATE
        "USE_HAL_DRIVER"
        ${MCU_MODEL}
    )

    # Target-specific definitions
    if(${target} STREQUAL "boot")
        target_compile_definitions(${target} PRIVATE "BOOT_PROJECT")
    elseif(${target} STREQUAL "loader")
        target_compile_definitions(${target} PRIVATE 
            "LOADER_PROJECT"
            "MBEDTLS_CONFIG_FILE=<mbedtls_config.h>"
        )
    elseif(${target} STREQUAL "updater")
        target_compile_definitions(${target} PRIVATE 
            "UPDATER_PROJECT"
            "MBEDTLS_CONFIG_FILE=<mbedtls_config.h>"
        )
    endif()

    target_link_options(${target} PRIVATE ${COMMON_LINKER_FLAGS})
endfunction()

# Configure all targets
configure_target(boot)
configure_target(loader)
configure_target(updater)
configure_target(application)

# Add mbedTLS and JANPATCH src
file(GLOB_RECURSE MBEDTLS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/drivers/ThirdParty/mbedTLS/library/*.c"
)

file(GLOB_RECURSE JANPATCH_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/drivers/ThirdParty/JANPATCH/*.c"
)

# Set specific includes for each target
target_include_directories(boot PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/boot/inc)
target_include_directories(loader PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/loader/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/MBEDTLS/App
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ThirdParty/mbedTLS/include
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ThirdParty/JANPATCH
)
target_include_directories(updater PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/updater/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/MBEDTLS/App
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ThirdParty/mbedTLS/include
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ThirdParty/JANPATCH
)
target_include_directories(application PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/application/inc)

# Set linker scripts and generate map files
set_target_properties(boot PROPERTIES LINK_FLAGS 
    "-T${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F407VGTx_FLASH_BOOT.ld -Wl,-Map=${CMAKE_BINARY_DIR}/boot.map")
set_target_properties(loader PROPERTIES LINK_FLAGS 
    "-T${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F407VGTx_FLASH_LOADER.ld -Wl,-Map=${CMAKE_BINARY_DIR}/loader.map")
set_target_properties(updater PROPERTIES LINK_FLAGS 
    "-T${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F407VGTx_FLASH_UPDATER.ld -Wl,-Map=${CMAKE_BINARY_DIR}/updater.map")
set_target_properties(application PROPERTIES LINK_FLAGS 
    "-T${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F407VGTx_FLASH_APP.ld -Wl,-Map=${CMAKE_BINARY_DIR}/application.map")

# Add sources to targets
target_sources(boot PRIVATE
    ${BOOT_STARTUP_FILE}
    ${COMMON_SOURCES}
    ${COMMON_LIB_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/boot/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/boot/src/system_stm32f4xx.c
)

target_sources(loader PRIVATE
    ${LOADER_STARTUP_FILE}
    ${COMMON_SOURCES}
    ${COMMON_LIB_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/loader/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/loader/src/system_stm32f4xx.c
    ${MBEDTLS_SOURCES}
)

target_sources(updater PRIVATE
    ${UPDATER_STARTUP_FILE}
    ${COMMON_SOURCES}
    ${COMMON_LIB_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/updater/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/updater/src/system_stm32f4xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/updater/src/updater_flash.c
    ${MBEDTLS_SOURCES}
    ${JANPATCH_SOURCES}
)

target_sources(application PRIVATE
    ${APP_STARTUP_FILE}
    ${COMMON_SOURCES}
    ${COMMON_LIB_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/system_stm32f4xx.c
)

# Post-build commands
foreach(TARGET boot loader updater application)
    add_custom_command(TARGET ${TARGET} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> ${TARGET}.bin
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET}>
    )
endforeach()

# Patch headers and merge bin
add_custom_target(patch_and_merge ALL
    DEPENDS boot loader updater application
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/merge_images.py 
        boot.bin loader.bin updater.bin application.bin
    COMMAND cp merged_firmware.bin final_firmware.bin
    COMMENT "Creating final firmware by merging all components"
)

# Flash targets
add_custom_target(flash_boot
    DEPENDS boot
    COMMAND STM32_Programmer.sh -c port=swd mode=UR -d boot.bin 0x08000000 -v -hardRst
)

add_custom_target(flash_full
    DEPENDS patch_and_merge
    COMMAND STM32_Programmer.sh -c port=swd mode=UR -d merged_firmware.bin 0x08000000 -v -hardRst
)