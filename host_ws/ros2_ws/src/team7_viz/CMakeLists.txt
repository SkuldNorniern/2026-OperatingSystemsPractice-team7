cmake_minimum_required(VERSION 3.16)
project(team7_viz NONE)

find_package(ament_cmake REQUIRED)

find_program(CARGO cargo HINTS "$ENV{HOME}/.cargo/bin" REQUIRED)

set(CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo")
set(CARGO_PROFILE colcon)

add_custom_target(cargo_build ALL
  # cargo cant use make jobserver from here
  COMMAND ${CMAKE_COMMAND} -E env --unset=MAKEFLAGS --unset=MFLAGS "CARGO_TARGET_DIR=${CARGO_TARGET_DIR}"
          ${CARGO} build --profile ${CARGO_PROFILE}
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  USES_TERMINAL
  VERBATIM)

install(PROGRAMS "${CARGO_TARGET_DIR}/${CARGO_PROFILE}/team7_viz" DESTINATION lib/${PROJECT_NAME})

ament_package()
