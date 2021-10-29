cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(wm8731_i2s_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(wm8731_i2s
	main.c
	wm8731.c
)

pico_enable_stdio_uart(wm8731_i2s 1)

pico_generate_pio_header(wm8731_i2s ${CMAKE_CURRENT_LIST_DIR}/i2s_fulldup.pio)

target_link_libraries(wm8731_i2s
	pico_stdlib
	hardware_pio
	hardware_dma
	hardware_i2c
)

pico_add_extra_outputs(wm8731_i2s)
