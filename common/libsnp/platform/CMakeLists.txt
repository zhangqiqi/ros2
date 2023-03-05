cmake_minimum_required (VERSION 3.8.2)

project(libsnp_platforms VERSION 1.0)

SET(CMAKE_BUILD_TYPE "Debug")

link_directories(../libs/lib)

include_directories(./include)
include_directories(../libs/include/)

aux_source_directory(. PLATFORMS_SRC)

add_library(snp_platforms SHARED ${PLATFORMS_SRC})

target_link_libraries(snp_platforms snp event_core)
