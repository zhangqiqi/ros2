cmake_minimum_required (VERSION 3.16)

project(lidar VERSION 1.0)

SET(CMAKE_CXX_STANDARD 17)

SET(CMAKE_INSTALL_PREFIX ../../../)

include_directories(../include)
include_directories(./wheeltec-N10/include)

aux_source_directory(./wheeltec-N10 LIDAR_SRC)

add_executable(N10_test ${LIDAR_SRC})

add_library(wheeltec_lidar STATIC ./wheeltec-N10/wheeltec_n10_protocol.c)

install(TARGETS wheeltec_lidar 
		DESTINATION libs/lib
)

install(DIRECTORY wheeltec-N10/include/
		DESTINATION libs/include/lidar/wheeltec-N10
)

