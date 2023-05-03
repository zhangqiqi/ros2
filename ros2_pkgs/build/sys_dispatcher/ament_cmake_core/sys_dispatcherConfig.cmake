# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sys_dispatcher_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sys_dispatcher_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sys_dispatcher_FOUND FALSE)
  elseif(NOT sys_dispatcher_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sys_dispatcher_FOUND FALSE)
  endif()
  return()
endif()
set(_sys_dispatcher_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sys_dispatcher_FIND_QUIETLY)
  message(STATUS "Found sys_dispatcher: 0.0.0 (${sys_dispatcher_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sys_dispatcher' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${sys_dispatcher_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sys_dispatcher_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sys_dispatcher_DIR}/${_extra}")
endforeach()
