cmake_minimum_required (VERSION 3.8.2)

project(libmotor VERSION 1.0)

include_directories(./include)
include_directories(./src/include)
include_directories(./PID)
include_directories(../counter/include)
include_directories(../counter/src/include)

aux_source_directory(./src/ LIBMOTOR_SRC)
aux_source_directory(../counter/src/ LIBMOTOR_SRC)

aux_source_directory(./test/ LIBMOTOR_TEST_SRC)

add_library(motor SHARED ${LIBMOTOR_SRC} ./PID/motor_pid.c)

add_executable(motor_test ${LIBMOTOR_TEST_SRC})

target_link_libraries(motor_test motor)
