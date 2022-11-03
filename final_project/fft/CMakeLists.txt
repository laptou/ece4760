target_sources(final_project
  PRIVATE fft/fft.cpp
  PUBLIC FILE_SET HEADERS
  BASE_DIRS ${PROJECT_SOURCE_DIR}
  FILES fft.h)
