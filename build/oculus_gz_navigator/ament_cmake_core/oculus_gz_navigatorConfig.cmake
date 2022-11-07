# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_oculus_gz_navigator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED oculus_gz_navigator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(oculus_gz_navigator_FOUND FALSE)
  elseif(NOT oculus_gz_navigator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(oculus_gz_navigator_FOUND FALSE)
  endif()
  return()
endif()
set(_oculus_gz_navigator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT oculus_gz_navigator_FIND_QUIETLY)
  message(STATUS "Found oculus_gz_navigator: 0.0.0 (${oculus_gz_navigator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'oculus_gz_navigator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${oculus_gz_navigator_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(oculus_gz_navigator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${oculus_gz_navigator_DIR}/${_extra}")
endforeach()
