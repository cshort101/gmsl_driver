# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Platform selection
#-------------------------------------------------------------------------------

if(VIBRANTE)
    message(STATUS "Cross Compiling for Vibrante")
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
    set(WINDOWS TRUE)
    add_definitions(-DWINDOWS)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        set(LINUX TRUE)
        add_definitions(-DLINUX)
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
        set(VIBRANTE TRUE)
        add_definitions(-DVIBRANTE))
    else()
        message(FATAL_ERROR "Unsupported Linux CPU architecture ${CMAKE_SYSTEM_PROCESSOR}.")
    endif()
else()
    message(FATAL_ERROR "Cannot identify OS")
endif()

# Position independent code enforce for 64bit systems and armv7l
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
   set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

#-------------------------------------------------------------------------------
# Architecture selection
#-------------------------------------------------------------------------------
if(MSVC)
    # Windows
    if(CMAKE_CL_64)
        set(ARCH_DIR "Win64")
    else()
        set(ARCH_DIR "Win32")
        message(FATAL_ERROR "Only Win64 builds are supported \
                Please make sure that you build the project with 64 bit compiler.")
    endif()
    if(MSVC_VERSION EQUAL 1800)
        set(ARCH_DIR "${ARCH_DIR}-vc12")
    elseif(MSVC_VERSION EQUAL 1900)
        set(ARCH_DIR "${ARCH_DIR}-vc14")
    endif()
else()
    # Linux (either VIBRANTE or PC)
    if(VIBRANTE)
        # Select device architecture
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(ARCH_DIR "linux-aarch64")
        else()
            set(ARCH_DIR "linux-armv7l")
        endif()
    else()
        set(ARCH_DIR "linux")
    endif()
endif()
unset(SDK_ARCH_DIR CACHE)
set(SDK_ARCH_DIR ${ARCH_DIR} CACHE INTERNAL "")
