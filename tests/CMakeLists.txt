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

# libhebi_add_test(test_Grab test_Grab.cpp)
# target_link_libraries(test_Grab "${PROJECT_NAME}")

# libherb_add_test(test_planOutOfJointLimits test_planOutOfJointLimits.cpp)
# target_link_libraries(test_planOutOfJointLimits "${PROJECT_NAME}")

# libherb_add_test(test_planToEndEffectorOffset
#    test_planToEndEffectorOffset.cpp)
# target_link_libraries(test_planToEndEffectorOffset "${PROJECT_NAME}")
