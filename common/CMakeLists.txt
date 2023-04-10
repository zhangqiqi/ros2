cmake_minimum_required (VERSION 3.16)

project(common VERSION 1.0)

include_directories(./include/)
aux_source_directory(./src/ LIBCOMMON_SRC)

add_subdirectory(cJSON/)
add_subdirectory(libssnp/)

add_library(common SHARED ${LIBCOMMON_SRC})

target_compile_features(common PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17

install(TARGETS common
	DESTINATION lib		
)

install(DIRECTORY ./include/
	DESTINATION include/common/		
)


