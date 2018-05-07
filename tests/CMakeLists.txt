set(LIBADA_TEST_RESOURCES_PATH "${PROJECT_SOURCE_DIR}/tests/resources/")
configure_file(
  "${CMAKE_SOURCE_DIR}/tests/config.hpp.in"
  "${CMAKE_BINARY_DIR}/tests/config.hpp"
  @ONLY
)
include_directories("${CMAKE_SOURCE_DIR}/tests/")
include_directories("${CMAKE_BINARY_DIR}/tests/")

if(NOT DART_utils_FOUND)
  message(STATUS "Looking for DART component 'dart-utils' - NOT found, "
      "to build libada tests, please install dart-utils (libdart6-utils-dev)")
  return()
endif()

