# Copyright(c)2016, NVIDIA CORPORATION.All rights reserved.

# - Try to find Driveworks
# Once done, this will define
#
#  Driveworks_FOUND - system has Driveworks
#  Driveworks_INCLUDE_DIRS - the Driveworks include directories
#  Driveworks_LIBRARIES - link these to use Driveworks

#include(LibFindMacros)
#include(ArchConfiguration)

# Use pkg-config to get hints about paths
#libfind_pkg_check_modules(Driveworks_PKGCONF driveworks)

# Use provided driveworks location hints
if(DRIVEWORKS)
  if(IS_ABSOLUTE ${DRIVEWORKS})
    set(DRIVEWORKS_DIR ${DRIVEWORKS} CACHE STRING "Path to the driveworks library location" FORCE)
  else()
    if("${CMAKE_HOST_SYSTEM}" MATCHES ".*Windows.*")
        set(sep "\\")
    else()
        set(sep "/")
    endif()
    get_filename_component(temp "${SDK_BINARY_DIR}${sep}${DRIVEWORKS}" ABSOLUTE)
    set(DRIVEWORKS_DIR ${temp} CACHE STRING "Path to the driveworks library location" FORCE)
  endif()
    message(STATUS "Driveworks location forced to: ${DRIVEWORKS_DIR}")
else()
    set(DRIVEWORKS_DIR "" CACHE STRING "Path to the driveworks library location")
endif()

# Make sure we support cross-compilation out of the default root path
SET(CMAKE_FIND_ROOT_PATH ${CMAKE_FIND_ROOT_PATH} ${DRIVEWORKS_DIR})

# Include dir
find_path(Driveworks_INCLUDE_DIR
  NAMES dw/core/Version.h
  HINTS ${DRIVEWORKS_DIR}/targets/${CMAKE_SYSTEM_PROCESSOR}-linux/include
        ${Driveworks_PKGCONF_INCLUDE_DIRS}/../targets/${CMAKE_SYSTEM_PROCESSOR}-linux/include
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../targets/${CMAKE_SYSTEM_PROCESSOR}-linux/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${DRIVEWORKS_DIR}/include
)

# Finally the library itself
find_library(Driveworks_LIBRARY
  NAMES driveworks
  HINTS ${DRIVEWORKS_DIR}/targets/${CMAKE_SYSTEM_PROCESSOR}-linux/lib
        ${Driveworks_PKGCONF_LIBRARY_DIRS}/../targets/${CMAKE_SYSTEM_PROCESSOR}-linux/lib
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../targets/${CMAKE_SYSTEM_PROCESSOR}-linux/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/../lib
        ${DRIVEWORKS_DIR}/lib
)

if (NOT DRIVEWORKS AND Driveworks_INCLUDE_DIR AND DRIVEWORKS_DIR STREQUAL "")
    get_filename_component(DRIVEWORKS_FROM_INCLUDE ${Driveworks_INCLUDE_DIR} DIRECTORY)
    message(STATUS "Driveworks found at: ${DRIVEWORKS_FROM_INCLUDE}")
    set(DRIVEWORKS_DIR ${DRIVEWORKS_FROM_INCLUDE} CACHE STRING "Path to the driveworks library location" FORCE)
endif()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Driveworks_PROCESS_INCLUDES Driveworks_INCLUDE_DIR)
set(Driveworks_PROCESS_LIBS Driveworks_LIBRARY)
#libfind_process(Driveworks)
