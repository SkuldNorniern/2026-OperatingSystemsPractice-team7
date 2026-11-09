cmake_minimum_required(VERSION 3.16)
project(team7_nodes NONE)

find_package(ament_cmake REQUIRED)
# Fail before Cargo runs if required generated ROS interfaces are missing.
find_package(nav2_msgs REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)

find_program(CARGO cargo HINTS "$ENV{HOME}/.cargo/bin" REQUIRED)

set(CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo")
set(CARGO_PROFILE colcon)
set(TEAM7_BINARIES marker_mapper explorer mission scan_filter navigator slam aruco_detector velocity_guard)

add_custom_target(cargo_build ALL
  # cargo cant use make jobserver from here
  COMMAND ${CMAKE_COMMAND} -E env --unset=MAKEFLAGS --unset=MFLAGS "CARGO_TARGET_DIR=${CARGO_TARGET_DIR}"
          ${CARGO} build --profile ${CARGO_PROFILE}
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  USES_TERMINAL
  VERBATIM)

foreach(bin ${TEAM7_BINARIES})
  install(PROGRAMS "${CARGO_TARGET_DIR}/${CARGO_PROFILE}/${bin}"
          DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
