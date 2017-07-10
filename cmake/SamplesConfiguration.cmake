# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# CUDA dependency (needs to be after defining all configurations)
#-------------------------------------------------------------------------------
find_package(CUDA REQUIRED)
set(CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})
include_directories(${CUDA_TOOLKIT_INCLUDE})

#-------------------------------------------------------------------------------
# Build flags
#-------------------------------------------------------------------------------
if(MSVC)
    if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX")

    if(CMAKE_GENERATOR MATCHES "NMake Makefiles JOM")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /FS")
    endif()

    # Microsoft Visual Studio 2013 gives wrong warning about valid C++11 uniform(bracket) initialization C4351 (http://stackoverflow.com/questions/24122006/vs2013-default-initialization-vs-value-initialization)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4351")

    # Parallel compilation
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")

    # Ignore warnings on missing .pdb's for debug builds
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /ignore:4099")

    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${SDK_BINARY_DIR}/runtime")
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wunused -Wunused-value -Wunused-parameter")

    if(NOT VIBRANTE)
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined -Wl,--as-needed")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-undefined -Wl,--as-needed")
    endif()
endif()

#-------------------------------------------------------------------------------
# CUDA configuration
#-------------------------------------------------------------------------------
set(CUDA_NVCC_FLAGS "-arch=sm_30;-lineinfo;-std=c++11;")

#-------------------------------------------------------------------------------
# Configured headers
#-------------------------------------------------------------------------------
include_directories(${SDK_BINARY_DIR}/configured)
