target_sources(final_project
  PRIVATE vga/vga.cpp
  PUBLIC FILE_SET HEADERS
  BASE_DIRS ${PROJECT_SOURCE_DIR}
  FILES vga.h)
