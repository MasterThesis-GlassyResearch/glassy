# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_glassy_joystick_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED glassy_joystick_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(glassy_joystick_FOUND FALSE)
  elseif(NOT glassy_joystick_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(glassy_joystick_FOUND FALSE)
  endif()
  return()
endif()
set(_glassy_joystick_CONFIG_INCLUDED TRUE)

# output package information
if(NOT glassy_joystick_FIND_QUIETLY)
  message(STATUS "Found glassy_joystick: 0.0.0 (${glassy_joystick_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'glassy_joystick' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${glassy_joystick_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(glassy_joystick_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${glassy_joystick_DIR}/${_extra}")
endforeach()
