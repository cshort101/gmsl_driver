# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

set(CMAKE_SYSTEM_NAME "Linux")
set(CMAKE_SYSTEM_VERSION 1)
set(VIBRANTE_BUILD ON)       #flags for the CMakeList.txt
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# need that one here, because this is a toolchain file and hence executed before
# default cmake settings are set
set(CMAKE_FIND_LIBRARY_PREFIXES "lib")
set(CMAKE_FIND_LIBRARY_SUFFIXES ".a" ".so")

# check that Vibrante PDK must be set
if (NOT DEFINED VIBRANTE_PDK)
    if (DEFINED ENV{VIBRANTE_PDK})
        message(STATUS "VIBRANTE_PDK = ENV : $ENV{VIBRANTE_PDK}")
        set(VIBRANTE_PDK $ENV{VIBRANTE_PDK} CACHE STRING "Path to the vibrante-XXX-linux path for cross-compilation" FORCE)
    endif()
else()
     message(STATUS "VIBRANTE_PDK = ${VIBRANTE_PDK}")
endif()

if(DEFINED VIBRANTE_PDK)
  if(NOT IS_ABSOLUTE ${VIBRANTE_PDK})
      get_filename_component(VIBRANTE_PDK "${SDK_BINARY_DIR}/${VIBRANTE_PDK}" ABSOLUTE)
  endif()
endif()

set(ARCH "aarch64")
set(VIBRANTE TRUE)
set(VIBRANTE_V4L TRUE)
add_definitions(-DVIBRANTE -DVIBRANTE_V4L)

set(TOOLCHAIN "${VIBRANTE_PDK}/../toolchains/tegra-4.9-nv")

set(CMAKE_CXX_COMPILER "${TOOLCHAIN}/usr/bin/aarch64-gnu-linux/aarch64-gnu-linux-g++")
set(CMAKE_C_COMPILER "${TOOLCHAIN}/usr/bin/aarch64-gnu-linux/aarch64-gnu-linux-gcc")
set(GCC_COMPILER_VERSION "4.9" CACHE STRING "GCC Compiler version")

# setup compiler for cross-compilation
set(CMAKE_CXX_FLAGS           "-fPIC"               CACHE STRING "c++ flags")
set(CMAKE_C_FLAGS             "-fPIC"               CACHE STRING "c flags")
set(CMAKE_SHARED_LINKER_FLAGS ""                    CACHE STRING "shared linker flags")
set(CMAKE_MODULE_LINKER_FLAGS ""                    CACHE STRING "module linker flags")
set(CMAKE_EXE_LINKER_FLAGS    ""                    CACHE STRING "executable linker flags")

set(LD_PATH ${VIBRANTE_PDK}/lib-target)
set(LD_PATH_EXTRA ${VIBRANTE_PDK}/targetfs/lib/aarch64-linux-gnu)


# Please, be carefull looks like "-Wl,-unresolved-symbols=ignore-in-shared-libs" can lead to silent "ld" problems
set(CMAKE_SHARED_LINKER_FLAGS   "-L${LD_PATH} -L${LD_PATH_EXTRA} -Wl,-rpath,${LD_PATH} ${CMAKE_SHARED_LINKER_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS      "-L${LD_PATH} -L${LD_PATH_EXTRA} -Wl,-rpath,${LD_PATH} ${CMAKE_EXE_LINKER_FLAGS}")

# Set cmake root path. If there is no "/usr/local" in CMAKE_FIND_ROOT_PATH then FinCUDA.cmake doesn't work
SET(CMAKE_FIND_ROOT_PATH ${VIBRANTE_PDK} ${VIBRANTE_PDK}/targetfs/usr/local/ ${VIBRANTE_PDK}/targetfs/ /usr/local)

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# set system default include dir
include_directories(BEFORE SYSTEM ${VIBRANTE_PDK}/include)

# determine target device and pdk branch
if (NOT DEFINED VIBRANTE_PDK_DEVICE AND VIBRANTE_PDK)
    if(${VIBRANTE_PDK} MATCHES "vibrante-(.+)-linux")
        set(VIBRANTE_PDK_DEVICE ${CMAKE_MATCH_1} CACHE STRING "Cross-compilation target device")
        message(STATUS "VIBRANTE_PDK_DEVICE = ${VIBRANTE_PDK_DEVICE}")
    else()
        message(FATAL_ERROR "Can't determine target device for PDK: ${VIBRANTE_PDK}")
    endif()
endif()

if (NOT DEFINED VIBRANTE_PDK_BRANCH AND VIBRANTE_PDK)
    if(EXISTS "${VIBRANTE_PDK}/lib-target/version-nv-pdk.txt")
        set(VIBRANTE_PDK_FILE "${VIBRANTE_PDK}/lib-target/version-nv-pdk.txt")
    elseif(EXISTS "${VIBRANTE_PDK}/lib-target/version-nv-sdk.txt")
        set(VIBRANTE_PDK_FILE "${VIBRANTE_PDK}/lib-target/version-nv-sdk.txt")
    endif()

    if(VIBRANTE_PDK_FILE)
       file(READ ${VIBRANTE_PDK_FILE} version-nv-pdk)
       if(${version-nv-pdk} MATCHES "^(.+)-[0123456789]+")
           set(VIBRANTE_PDK_BRANCH ${CMAKE_MATCH_1} CACHE STRING "Cross-compilation PDK branch name")
           message(STATUS "VIBRANTE_PDK_BRANCH = ${VIBRANTE_PDK_BRANCH}")
       else()
           message(FATAL_ERROR "Can't determine PDK branch for PDK ${VIBRANTE_PDK}")
       endif()
    else()
       message(FATAL_ERROR "Can't open ${VIBRANTE_PDK}/lib-target/version-nv-(pdk/sdk).txt for PDK branch detection")
    endif()
endif()

if (DEFINED VIBRANTE_PDK_BRANCH)
  string(REPLACE "." ";" PDK_VERSION_LIST ${VIBRANTE_PDK_BRANCH})

  # Some PDK's have less than three version numbers. Pad the list so we always
  # have at least three numbers, allowing pre-existing logic depending on major,
  # minor, patch versioning to work without modifications
  list(LENGTH PDK_VERSION_LIST _PDK_VERSION_LIST_LENGTH)
  while(_PDK_VERSION_LIST_LENGTH LESS 3)
    list(APPEND PDK_VERSION_LIST 0)
    math(EXPR _PDK_VERSION_LIST_LENGTH "${_PDK_VERSION_LIST_LENGTH} + 1")
  endwhile()

  set(VIBRANTE_PDK_PATCH 0)
  set(VIBRANTE_PDK_BUILD 0)

  list(LENGTH PDK_VERSION_LIST PDK_VERSION_LIST_LENGTH)

  list(GET PDK_VERSION_LIST 0 VIBRANTE_PDK_MAJOR)
  list(GET PDK_VERSION_LIST 1 VIBRANTE_PDK_MINOR)
  if (PDK_VERSION_LIST_LENGTH GREATER 2)
    list(GET PDK_VERSION_LIST 2 VIBRANTE_PDK_PATCH)
  endif()

  if (PDK_VERSION_LIST_LENGTH GREATER 3)
    list(GET PDK_VERSION_LIST 3 VIBRANTE_PDK_BUILD)
  endif()

  add_definitions(-DVIBRANTE_PDK_MAJOR=${VIBRANTE_PDK_MAJOR})
  add_definitions(-DVIBRANTE_PDK_MINOR=${VIBRANTE_PDK_MINOR})
  add_definitions(-DVIBRANTE_PDK_PATCH=${VIBRANTE_PDK_PATCH})
  add_definitions(-DVIBRANTE_PDK_BUILD=${VIBRANTE_PDK_BUILD})
endif()
