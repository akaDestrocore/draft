cmake_minimum_required(VERSION 3.22)  

include("cmake/gcc-arm-none-eabi.cmake")

# Define main project
project(stm32f4_bootloader)
enable_language(C ASM)

# Find Python
find_program(PYTHON_EXECUTABLE 
    NAMES python3 python
    REQUIRED
)

message("Build type: " ${CMAKE_BUILD_TYPE})
message("Using Python: ${PYTHON_EXECUTABLE}")

#############################################################
#### COMMON SETTINGS FOR ALL COMPONENTS
#############################################################

# Common settings for all projects
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# MCU settings
set(MCU_FAMILY STM32F4xx)
set(MCU_MODEL STM32F407xx)

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

# Basic HAL/CMSIS drivers needed by all components
set(DRIVER_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/${MCU_FAMILY}_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/${MCU_FAMILY}_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/CMSIS/Device/ST/${MCU_FAMILY}/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/CMSIS/Include
)

# Create executable only for boot
add_executable(boot)
add_executable(loader)
# add_executable(updater)
# add_executable(application)

#############################################################
#### BOOT SETTINGS
#############################################################

# Boot project settings
set(BOOT_STARTUP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boot/startup/startup_stm32f407vgtx.s")

set(BOOT_INCLUDES
    ${DRIVER_INCLUDES}
    ${CMAKE_CURRENT_SOURCE_DIR}/boot/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/common/inc
)

set(BOOT_SOURCES
    ${BOOT_STARTUP_FILE}
    ${CMAKE_CURRENT_SOURCE_DIR}/boot/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/boot/src/system_stm32f4xx.c
)

target_compile_options(boot PRIVATE ${COMMON_FLAGS})
target_include_directories(boot PRIVATE ${BOOT_INCLUDES})
target_compile_definitions(boot PRIVATE "USE_HAL_DRIVER" ${MCU_MODEL} "BOOT_PROJECT")
target_link_options(boot PRIVATE ${COMMON_LINKER_FLAGS})
set_target_properties(boot PROPERTIES LINK_FLAGS 
    "-T${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F407VGTx_FLASH_BOOT.ld -Wl,-Map=${CMAKE_BINARY_DIR}/boot.map")
target_sources(boot PRIVATE ${BOOT_SOURCES})

#############################################################
#### НАСТРОЙКИ ДЛЯ LOADER И UPDATER (ЗАКОММЕНТИРОВАНО)
#############################################################

# Loader and updater project settings
set(LOADER_STARTUP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/loader/startup/startup_stm32f407vgtx.s")
set(UPDATER_STARTUP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/updater/startup/startup_stm32f407vgtx.s")

set(LOADER_UPDATER_INCLUDES
    ${DRIVER_INCLUDES}
    ${CMAKE_CURRENT_SOURCE_DIR}/common/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/MBEDTLS/App
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/Third_Party/mbedTLS/include/mbedtls
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/Third_Party/mbedTLS/include
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/Third_Party/JANPATCH
)

# Общие исходные файлы для loader и updater
file(GLOB COMMON_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/common/src/*.c
)

# Настройки для loader
target_compile_options(loader PRIVATE ${COMMON_FLAGS})
target_include_directories(loader PRIVATE 
    ${LOADER_UPDATER_INCLUDES} 
    ${CMAKE_CURRENT_SOURCE_DIR}/loader/inc
)
target_compile_definitions(loader PRIVATE 
    "USE_HAL_DRIVER" 
    ${MCU_MODEL} 
    "LOADER_PROJECT" 
    "MBEDTLS_CONFIG_FILE=<mbedtls_config.h>"
)
target_link_options(loader PRIVATE ${COMMON_LINKER_FLAGS})
set_target_properties(loader PROPERTIES LINK_FLAGS 
    "-T${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F407VGTx_FLASH_LOADER.ld -Wl,-Map=${CMAKE_BINARY_DIR}/loader.map")
target_sources(loader PRIVATE
    ${LOADER_STARTUP_FILE}
    ${COMMON_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/loader/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/boot/src/system_stm32f4xx.c
)

# # Настройки для updater
# target_compile_options(updater PRIVATE ${COMMON_FLAGS})
# target_include_directories(updater PRIVATE 
#     ${LOADER_UPDATER_INCLUDES} 
#     ${CMAKE_CURRENT_SOURCE_DIR}/updater/inc
# )
# target_compile_definitions(updater PRIVATE 
#     "USE_HAL_DRIVER" 
#     ${MCU_MODEL} 
#     "UPDATER_PROJECT" 
#     "MBEDTLS_CONFIG_FILE=<mbedtls_config.h>"
# )
# target_link_options(updater PRIVATE ${COMMON_LINKER_FLAGS})
# set_target_properties(updater PROPERTIES LINK_FLAGS 
#     "-T${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F407VGTx_FLASH_UPDATER.ld -Wl,-Map=${CMAKE_BINARY_DIR}/updater.map")
# target_sources(updater PRIVATE
#     ${UPDATER_STARTUP_FILE}
#     ${COMMON_SOURCES}
#     ${CMAKE_CURRENT_SOURCE_DIR}/updater/src/main.c
# )

#############################################################
#### НАСТРОЙКИ ДЛЯ APPLICATION (ЗАКОММЕНТИРОВАНО)
#############################################################

# Закомментированные настройки для application - раскомментируйте, когда будет нужно
# set(APP_STARTUP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/application/startup/startup_stm32f407vgtx.s")
# 
# set(APP_INCLUDES
#     ${DRIVER_INCLUDES}
#     ${CMAKE_CURRENT_SOURCE_DIR}/common/inc
#     ${CMAKE_CURRENT_SOURCE_DIR}/application/inc
# )
# 
# target_compile_options(application PRIVATE ${COMMON_FLAGS})
# target_include_directories(application PRIVATE ${APP_INCLUDES})
# target_compile_definitions(application PRIVATE "USE_HAL_DRIVER" ${MCU_MODEL} "APP_PROJECT")
# target_link_options(application PRIVATE ${COMMON_LINKER_FLAGS})
# set_target_properties(application PROPERTIES LINK_FLAGS 
#     "-T${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F407VGTx_FLASH_APP.ld -Wl,-Map=${CMAKE_BINARY_DIR}/application.map")
# target_sources(application PRIVATE
#     ${APP_STARTUP_FILE}
#     ${COMMON_SOURCES}
#     ${CMAKE_CURRENT_SOURCE_DIR}/application/src/main.c
# )

#############################################################
#### POST-BUILD COMMANDS И ЦЕЛИ ДЛЯ ПРОШИВКИ
#############################################################

# Post-build command только для boot
add_custom_command(TARGET boot POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:boot> boot.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:boot>
)

# Цель для прошивки boot
add_custom_target(flash_boot
    DEPENDS boot
    COMMAND STM32_Programmer.sh -c port=swd mode=UR -d boot.bin 0x08000000 -v -hardRst
)

# Закомментированные цели для полной прошивки
# add_custom_target(patch_and_merge ALL
#     DEPENDS boot loader updater
#     COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/merge_images.py 
#         boot.bin loader.bin updater.bin application.bin
#     COMMAND cp merged_firmware.bin final_firmware.bin
#     COMMENT "Creating final firmware by merging all components"
# )
# 
# add_custom_target(flash_full
#     DEPENDS patch_and_merge
#     COMMAND STM32_Programmer.sh -c port=swd mode=UR -d merged_firmware.bin 0x08000000 -v -hardRst
# )