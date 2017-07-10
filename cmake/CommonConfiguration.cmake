# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Enable C++11
#-------------------------------------------------------------------------------
if(CMAKE_VERSION VERSION_GREATER 3.1)
    set(CMAKE_CXX_STANDARD 11)
else()
    if(LINUX OR VIBRANTE)
        include(CheckCXXCompilerFlag)
        CHECK_CXX_COMPILER_FLAG(-std=c++11 COMPILER_SUPPORTS_CXX11)
        CHECK_CXX_COMPILER_FLAG(-std=c++0x COMPILER_SUPPORTS_CXX0X)
        if(COMPILER_SUPPORTS_CXX11)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
        elseif(COMPILER_SUPPORTS_CXX0X)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
        else()
            message(ERROR "Compiler ${CMAKE_CXX_COMPILER} has no C++11 support")
        endif()
    endif()
endif()

#-------------------------------------------------------------------------------
# Dependencies
#-------------------------------------------------------------------------------
find_package(Threads REQUIRED)

if(LINUX)
    find_package(X11 REQUIRED)
    if (NOT X11_Xrandr_FOUND)
        message(FATAL_ERROR "Missing X11_Xrandr library")
    endif()
    if (NOT X11_Xcursor_FOUND)
        message(FATAL_ERROR "Missing X11_Xcursor library")
    endif()
    if (NOT X11_xf86vmode_FOUND)
        message(FATAL_ERROR "Missing X11_xf86vmode library")
    endif()
    if (NOT X11_Xinerama_FOUND)
        message(FATAL_ERROR "Missing X11_Xinerama library")
    endif()
    if (NOT X11_Xi_FOUND)
        message(FATAL_ERROR "Missing X11_Xi library")
    endif()
endif()
