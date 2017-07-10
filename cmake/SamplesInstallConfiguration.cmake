# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Samples Installation configuration
#-------------------------------------------------------------------------------
set(SDK_SAMPLE_DESTINATION  "bin")
set(SDK_LIBRARY_DESTINATION "lib")
set(SDK_ARCHIVE_DESTINATION "lib")

function(sdk_install_samples SAMPLES)
    install(TARGETS ${SAMPLES}
        COMPONENT samples
        RUNTIME DESTINATION ${SDK_SAMPLE_DESTINATION}
        LIBRARY DESTINATION ${SDK_LIBRARY_DESTINATION}
        ARCHIVE DESTINATION ${SDK_ARCHIVE_DESTINATION}
    )
endfunction(sdk_install_samples)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${SDK_BINARY_DIR}/install" CACHE PATH "Default install path" FORCE)
endif()
message(STATUS "Driveworks Samples install dir: ${CMAKE_INSTALL_PREFIX}")
