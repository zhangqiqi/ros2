cmake_minimum_required (VERSION 3.16)

project(device VERSION 1.0)

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/libs/)

add_subdirectory(common/)
add_subdirectory(device/)
add_subdirectory(opensource/)

add_custom_target(
    ros2_pkgs
    COMMAND colcon build --cmake-args -DCOMMON_LIB_PATH=${CMAKE_INSTALL_PREFIX}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ros2_pkgs/
)


