# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)


# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# Tell CMake where to find the executable source file
add_executable(main 
    main.cpp
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(main)

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(main 
    pico_stdlib
)
