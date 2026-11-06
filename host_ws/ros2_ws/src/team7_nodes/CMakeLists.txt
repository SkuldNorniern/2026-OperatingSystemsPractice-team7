cmake_minimum_required(VERSION 3.16)
project(team7_nodes C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_action REQUIRED)
find_package(rclc REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)

set(TB4_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../../libtb4 ${CMAKE_CURRENT_BINARY_DIR}/libtb4)

add_library(team7_common STATIC src/common.c)
target_include_directories(team7_common PUBLIC src)
target_link_libraries(team7_common PUBLIC tb4::tb4)
ament_target_dependencies(team7_common PUBLIC
  rcl rclc rcutils rcl_yaml_param_parser rosidl_runtime_c geometry_msgs nav_msgs tf2_msgs visualization_msgs)

set(TEAM7_WARNINGS -Wall -Wextra -Wshadow -Wno-unused-parameter)
target_compile_options(team7_common PRIVATE ${TEAM7_WARNINGS})

add_executable(marker_mapper src/marker_mapper.c)
target_link_libraries(marker_mapper team7_common)
ament_target_dependencies(marker_mapper aruco_opencv_msgs nav_msgs visualization_msgs)

add_executable(explorer src/explorer.c)
target_link_libraries(explorer team7_common)
ament_target_dependencies(explorer rcl_action nav2_msgs action_msgs nav_msgs std_msgs visualization_msgs)

add_executable(mission src/mission.c)
target_link_libraries(mission team7_common)
ament_target_dependencies(mission action_msgs std_msgs visualization_msgs)

foreach(t marker_mapper explorer mission)
  target_compile_options(${t} PRIVATE ${TEAM7_WARNINGS})
endforeach()

install(TARGETS marker_mapper explorer mission DESTINATION lib/${PROJECT_NAME})

ament_package()
