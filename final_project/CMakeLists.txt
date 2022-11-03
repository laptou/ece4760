add_executable(final_project)

# must match with pio filename and executable name from above
pico_generate_pio_header(final_project ${CMAKE_CURRENT_LIST_DIR}/vga/hsync.pio)
pico_generate_pio_header(final_project ${CMAKE_CURRENT_LIST_DIR}/vga/vsync.pio)
pico_generate_pio_header(final_project ${CMAKE_CURRENT_LIST_DIR}/vga/rgb.pio)

target_include_directories(final_project PRIVATE "${PROJECT_SOURCE_DIR}")
add_subdirectory("vga")
add_subdirectory("fpmath")
add_subdirectory("fft")

# must match with executable name and source file names
target_sources(final_project PRIVATE main.cpp)

# must match with executable name
target_link_libraries(final_project PRIVATE pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_pio hardware_dma hardware_adc hardware_irq)

# must match with executable name
pico_add_extra_outputs(final_project)
