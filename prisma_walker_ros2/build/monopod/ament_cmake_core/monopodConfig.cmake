# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_monopod_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED monopod_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(monopod_FOUND FALSE)
  elseif(NOT monopod_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(monopod_FOUND FALSE)
  endif()
  return()
endif()
set(_monopod_CONFIG_INCLUDED TRUE)

# output package information
if(NOT monopod_FIND_QUIETLY)
  message(STATUS "Found monopod: 0.0.0 (${monopod_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'monopod' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${monopod_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(monopod_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${monopod_DIR}/${_extra}")
endforeach()
