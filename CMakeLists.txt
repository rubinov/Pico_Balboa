# Generated Cmake Pico project file
# Updated: November 21, 2024, 6:59 PM CST

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(Pico_Balboa C CXX ASM)

pico_sdk_init()

add_executable(Pico_Balboa 
    Pico_Balboa.c
)

pico_set_program_name(Pico_Balboa "Pico_Balboa")
pico_set_program_version(Pico_Balboa "0.1")

# Generate PIO header from encoders.pio
pico_generate_pio_header(Pico_Balboa ${CMAKE_CURRENT_LIST_DIR}/encoders.pio)

pico_enable_stdio_uart(Pico_Balboa 0)
pico_enable_stdio_usb(Pico_Balboa 1)

# --- UPDATED: Added Bluetooth and Wireless Libraries ---
target_link_libraries(Pico_Balboa
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_pio
    hardware_timer
    hardware_gpio
    hardware_dma
    # Bluetooth / Wireless support
    pico_cyw43_arch_none 
    pico_btstack_classic
    pico_btstack_cyw43
    btstack_run_loop
)

target_include_directories(Pico_Balboa PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(Pico_Balboa)