SET(LIDAR_DIR ${DEVICE_DIR}/lidar)

include_directories(${LIDAR_DIR}/wheeltec-N10/include)
aux_source_directory(${LIDAR_DIR}/wheeltec-N10 LIDAR_SRC)

add_library(wheeltec_lidar STATIC ${LIDAR_SRC})

install(TARGETS wheeltec_lidar 
		DESTINATION lib
)

install(DIRECTORY ${LIDAR_DIR}/wheeltec-N10/include/
		DESTINATION include/lidar/wheeltec-N10
)
