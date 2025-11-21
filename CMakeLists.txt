# Generated Cmake Pico project file

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
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Pico_Balboa C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Pico_Balboa Pico_Balboa.c )

pico_set_program_name(Pico_Balboa "Pico_Balboa")
pico_set_program_version(Pico_Balboa "0.1")

# Generate PIO header
pico_generate_pio_header(Pico_Balboa ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Pico_Balboa 0)
pico_enable_stdio_usb(Pico_Balboa 1)

# Add the standard library to the build
target_link_libraries(Pico_Balboa
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Pico_Balboa PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(Pico_Balboa 
        hardware_spi
        hardware_i2c
        hardware_pio
        hardware_timer
        
        )

pico_add_extra_outputs(Pico_Balboa)

# ====================================================================================
# ENCODER TEST EXECUTABLE (Diagnostic Tool)
# ====================================================================================

add_executable(encoder_test
    encoder_test.c
)

pico_set_program_name(encoder_test "Encoder_Test")
pico_set_program_version(encoder_test "0.1")

# Enable USB output for printf
pico_enable_stdio_uart(encoder_test 0)
pico_enable_stdio_usb(encoder_test 1)

# Add the standard library
target_link_libraries(encoder_test
    pico_stdlib
    hardware_gpio
)

# Add include directories
target_include_directories(encoder_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Create UF2 file
pico_add_extra_outputs(encoder_test)