cmake_minimum_required(VERSION 2.8.12)
project(libada)
set(library_VERSION 0.2.0)

option(LIBADA_TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#================================================================================
# CodeCov setup
#
option(CODECOV "Enable CodeCov support" OFF)
if(CODECOV)
  include(CodeCoverage)
  setup_target_for_coverage(${PROJECT_NAME}_coverage ctest coverage)
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage"
  )
endif()

#================================================================================
# Dependencies
#

include(ExternalProject)

find_package(DART 6.6.0 REQUIRED
  OPTIONAL_COMPONENTS utils
)

find_package(aikido 0.3.0 REQUIRED
  COMPONENTS
    common
    constraint
    control
    control_ros
    distance
    planner
    planner_kunzretimer
    planner_ompl
    planner_parabolic
    planner_vectorfield
    planner_kunzretimer
    robot
    statespace
    io
)

find_package(ada_description REQUIRED)

find_package(Boost REQUIRED)

find_package(controller_manager_msgs REQUIRED)

find_package(roslib REQUIRED)

find_package(srdfdom REQUIRED)

find_package(urdf REQUIRED)


#================================================================================
# Compiler settings
#
add_compile_options(-Wall -Wextra -Wpedantic)
if(LIBADA_TREAT_WARNINGS_AS_ERRORS)
  add_compile_options(-Werror)
endif()

#================================================================================
# Library
#

set(sources
  src/AdaFingerKinematicSimulationPositionCommandExecutor.cpp
  src/AdaHandKinematicSimulationPositionCommandExecutor.cpp
  src/AdaHand.cpp
  src/Ada.cpp
  src/util.cpp
)

add_library(libada SHARED ${sources})

target_include_directories(libada PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(libada SYSTEM
  PUBLIC
    ${DART_INCLUDE_DIRS}
    ${aikido_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${controller_manager_msgs_INCLUDE_DIRS}
  PRIVATE
    ${roslib_INCLUDE_DIRS}
    ${srdfdom_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
)

set_property(TARGET libada PROPERTY VERSION ${library_VERSION})

# Don't name shared library "liblibada"
set_property(TARGET libada PROPERTY PREFIX "")

target_link_libraries(libada
  PUBLIC
    ${DART_LIBRARIES}
    ${aikido_LIBRARIES}
  PRIVATE
    ${roslib_LIBRARIES}
    ${srdfdom_LIBRARIES}
    ${urdf_LIBRARIES}
)

#================================================================================
# Testing
#

set_property(GLOBAL PROPERTY LIBADA_TESTS)
function(libada_add_test target_name)
  add_executable("${target_name}" ${ARGN})
  add_test("${target_name}" "${target_name}")

  target_link_libraries("${target_name}" gtest gtest_main)

  set_property(GLOBAL APPEND PROPERTY LIBADA_TESTS "${target_name}")
endfunction()

option(LIBADA_BUILD_TESTS "Build libada unit tests" ON)
if(LIBADA_BUILD_TESTS)
  enable_testing()
  include(cmake/External_GTest.cmake)
  include_directories(${GTEST_INCLUDE_DIRS})
  add_subdirectory(tests)
endif()

# Targets to mimic a Catkin package.
get_property(all_tests GLOBAL PROPERTY LIBADA_TESTS)
add_custom_target(tests DEPENDS ${all_tests})
add_custom_target(run_tests COMMAND "${CMAKE_CTEST_COMMAND}")

#================================================================================
# Installation
#

install(TARGETS libada EXPORT libadaConfig
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# This makes the project importable from the install directory
install(EXPORT libadaConfig DESTINATION share/libada/cmake)

# This makes the project importable from the build directory
export(TARGETS libada FILE libadaConfig.cmake)

# Install the package.xml file (to satisfy REP-136).
install(FILES "package.xml"
  DESTINATION "share/libada")

#install(DIRECTORY resources DESTINATION share)


#================================================================================
# Formatting
#

include(ClangFormat)
clang_format_setup(VERSION 3.8)

if(CLANG_FORMAT_EXECUTABLE)

  file(GLOB_RECURSE ALL_SOURCE_FILES
    LIST_DIRECTORIES false
    include/*.h include/*.hpp src/*.c src/*.cpp tests/*.cpp tests/*.hpp)

  clang_format_add_sources(${ALL_SOURCE_FILES})

  clang_format_add_targets()
endif()

# This custom target doesn't affect building libada but is only for
# displaying the header files in IDEs.
FILE(GLOB_RECURSE libada_headers "include/*.hpp")
add_custom_target(headers SOURCES ${libada_headers})
