cmake_minimum_required (VERSION 3.16)

project(cJSON VERSION 1.0)

include_directories(.)

aux_source_directory(. CJSON_SRC)

add_library(cJSON STATIC ${CJSON_SRC})

install(TARGETS cJSON
        DESTINATION lib
)

install(FILES cJSON.h
        DESTINATION include
)
