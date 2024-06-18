cmake_minimum_required(VERSION 3.12)

add_executable(pico_wifi_smart_grid
    src/main.c
)

# include directories
target_include_directories(pico_wifi_smart_grid PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
    )

# libraries
target_link_libraries(pico_wifi_smart_grid 
    hardware_adc
    hardware_dma
    pico_multicore
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_mbedtls
    pico_lwip_mbedtls
    pico_lwip_mqtt
    )

# enable usb output, disable uart output
pico_enable_stdio_usb(pico_wifi_smart_grid 0)
pico_enable_stdio_uart(pico_wifi_smart_grid 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico_wifi_smart_grid)
