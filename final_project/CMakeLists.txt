add_executable(final_project)

# must match with pio filename and executable name from above
# pico_generate_pio_header(fft ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
# pico_generate_pio_header(fft ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
# pico_generate_pio_header(fft ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# must match with executable name and source file names
target_sources(final_project PRIVATE main.c)

# must match with executable name
target_link_libraries(final_project PRIVATE pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_pio hardware_dma hardware_adc hardware_irq)

# must match with executable name
pico_add_extra_outputs(final_project)
