cmake_minimum_required (VERSION 3.8.2)

project(libsnp VERSION 1.0)


SET(CMAKE_BUILD_TYPE "Debug")

link_directories(./libs/lib)

include_directories(./include)
include_directories(./src/include)
include_directories(./platform/include)

aux_source_directory(./src/ LIBSNP_SRC)

aux_source_directory(./test/ LIBSNP_TEST_SRC)

add_library(snp SHARED ${LIBSNP_SRC})

add_subdirectory(./platform/)

add_executable(snp_test ${LIBSNP_TEST_SRC})

target_link_libraries(snp_test snp snp_platforms pthread)
