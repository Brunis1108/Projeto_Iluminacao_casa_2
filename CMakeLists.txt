
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 1.5.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Controle_Iluminacao_2 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

include_directories(${CMAKE_SOURCE_DIR}/lib)

# Add executable. Default name is the project name, version 0.1

add_executable(Controle_Iluminacao_2 Controle_Iluminacao_2.c lib/ssd1306.c)

pico_set_program_name(Controle_Iluminacao_2 "Controle_Iluminacao_2")
pico_set_program_version(Controle_Iluminacao_2 "0.1")

pico_generate_pio_header(Controle_Iluminacao_2 ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Controle_Iluminacao_2 0)
pico_enable_stdio_usb(Controle_Iluminacao_2 1)

# Add the standard library to the build
target_link_libraries(Controle_Iluminacao_2
        pico_stdlib
        hardware_gpio
        hardware_adc
        hardware_i2c
        hardware_pio
        hardware_pwm
        pico_cyw43_arch_lwip_threadsafe_background
)

# Add the standard include files to the build
target_include_directories(Controle_Iluminacao_2 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/lib/lwip/src/include
    ${PICO_SDK_PATH}/lib/lwip/src/include/arch
    ${PICO_SDK_PATH}/lib/lwip/src/include/lwip
)

target_sources(Controle_Iluminacao_2 PRIVATE
    ${PICO_SDK_PATH}/lib/lwip/src/apps/http/httpd.c
    ${PICO_SDK_PATH}/lib/lwip/src/apps/http/fs.c
)


# Add any user requested libraries

pico_add_extra_outputs(Controle_Iluminacao_2)
