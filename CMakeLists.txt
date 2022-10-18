cmake_minimum_required(VERSION 3.12)

# Change your executable name to something creative!
set(NAME ov2640) # <-- Name your project/executable here!

include(pimoroni_pico_import.cmake)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

set(PICO_BOARD pico_w)
set(PICO_CYW43_ARCH_POLL 1)

# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(${NAME}
	ov2640.c
	main.c
)

pico_set_program_name(${NAME} "ov2640")
pico_set_program_version(${NAME} "0.1")

pico_enable_stdio_uart(${NAME} 0)
pico_enable_stdio_usb(${NAME} 1)

pico_generate_pio_header(${NAME} ${CMAKE_CURRENT_LIST_DIR}/image.pio)

target_link_libraries(${NAME}
	pico_stdlib
	pico_multicore
	hardware_dma
	hardware_i2c
	hardware_pio
	hardware_pwm
	pico_cyw43_arch_lwip_poll
)

target_include_directories(${NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR} )

pico_add_extra_outputs(${NAME})
