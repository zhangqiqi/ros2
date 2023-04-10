#!/bin/sh

colcon build --cmake-args -DCOMMON_LIB_PATH=~/ros2/libs/ -DCMAKE_EXPORT_COMPILE_COMMANDS=on

