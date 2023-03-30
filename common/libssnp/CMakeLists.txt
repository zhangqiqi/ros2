cmake_minimum_required (VERSION 3.8.2)

project(libssnp VERSION 1.0)


SET(CMAKE_BUILD_TYPE "Debug")

include_directories(./include)

aux_source_directory(./src/ LIBSSNP_SRC)

add_library(snp SHARED ${LIBSSNP_SRC})


