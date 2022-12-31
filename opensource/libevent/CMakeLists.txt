cmake_minimum_required (VERSION 3.16)

project(libevent VERSION 1.0)

if (UNIX)

install(DIRECTORY unix/bin DESTINATION ./) 
install(DIRECTORY unix/include DESTINATION ./)
install(DIRECTORY unix/lib DESTINATION ./)

endif()

 