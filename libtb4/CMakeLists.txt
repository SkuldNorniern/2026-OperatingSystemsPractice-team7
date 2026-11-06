# cmake -S libtb4 -B build/libtb4 && cmake --build build/libtb4 && ctest --test-dir build/libtb4
cmake_minimum_required(VERSION 3.16)
project(libtb4 C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(TB4_BUILD_TESTS "Build libtb4 unit tests" ${PROJECT_IS_TOP_LEVEL})
option(TB4_SANITIZE "Build with AddressSanitizer + UBSan" OFF)

add_library(tb4 STATIC
  src/geom.c
  src/grid.c
  src/tf.c
  src/explore.c
  src/markers.c)
add_library(tb4::tb4 ALIAS tb4)
target_include_directories(tb4 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(tb4 PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
target_compile_definitions(tb4 PUBLIC _DEFAULT_SOURCE)
target_link_libraries(tb4 PUBLIC m)
set_target_properties(tb4 PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(TB4_SANITIZE)
  target_compile_options(tb4 PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(tb4 PUBLIC -fsanitize=address,undefined)
endif()

if(TB4_BUILD_TESTS)
  enable_testing()
  add_executable(test_tb4 tests/test_tb4.c)
  target_link_libraries(test_tb4 PRIVATE tb4)
  add_test(NAME test_tb4 COMMAND test_tb4)
endif()
