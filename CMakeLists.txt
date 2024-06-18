cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_wifi C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)

# set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)

# Initialize the SDK
pico_sdk_init()

# Add smart_grid example
add_subdirectory(smart_grid)
# add_subdirectory(tls_client)
