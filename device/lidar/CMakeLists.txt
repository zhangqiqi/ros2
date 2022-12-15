cmake_minimum_required (VERSION 3.16)

project(lidar VERSION 1.0)

SET(CMAKE_CXX_STANDARD 17)

aux_source_directory(./wheeltec-N10 LIDAR_SRC)

add_executable(N10_test ${LIDAR_SRC})
