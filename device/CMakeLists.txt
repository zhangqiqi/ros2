cmake_minimum_required (VERSION 3.16)

project(device VERSION 1.0)

SET(CMAKE_INSTALL_PREFIX ../libs/)

SET(DEVICE_DIR .)

include_directories(./include)
include(./lidar/lidar.cmake)


