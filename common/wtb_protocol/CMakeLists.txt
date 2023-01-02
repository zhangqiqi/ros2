cmake_minimum_required (VERSION 3.16)

project(wtb_protocol VERSION 1.0)

include_directories(.)

aux_source_directory(. WTB_PROTOCOL_SRC)

add_library(wtb_protocol STATIC ${WTB_PROTOCOL_SRC})

install(TARGETS wtb_protocol
        DESTINATION lib
)

install(FILES wtb_protocol.h
        DESTINATION include
)
