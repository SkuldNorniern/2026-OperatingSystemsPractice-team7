cmake_minimum_required(VERSION 3.16)
project(team7_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
        DESTINATION share/${PROJECT_NAME})

ament_package()
