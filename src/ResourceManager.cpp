/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2015-2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ResourceManager.hpp"

ResourceManager::~ResourceManager()
{
    releaseSAL();

    releaseRenderer();

    releaseDriveworks();

    releaseSampleApp();

    delete window;
}

dwStatus ResourceManager::initDriveworks()
{
    dwStatus res = dwLogger_initialize(getConsoleLoggerCallback(true));
    if (DW_SUCCESS != res) return res;

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = dwContextParameters();

#ifdef VIBRANTE
    //sdkParams.eglDisplay = gWindow->getEGLDisplay();
    memset(&sdkParams, 0, sizeof(dwContextParameters));
#endif

    (void) window;

    return dwInitialize(&m_SDKHandle, DW_VERSION, &sdkParams);
}

dwStatus ResourceManager::initSAL()
{
    return dwSAL_initialize(&m_salHandle, m_SDKHandle);
}

void ResourceManager::releaseSAL()
{
    dwSAL_release(&m_salHandle);
}

dwStatus ResourceManager::initRenderer()
{
    dwStatus result;

    result = dwRenderer_initialize(&m_rendererHandle, m_SDKHandle);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init renderer: ") +
                                 dwGetStatusName(result));

    // Set some renderer defaults
    dwRect rect;
    rect.width   = window->width();
    rect.height  = window->height();
    rect.x = 0;
    rect.y = 0;

    dwRenderer_setRect(rect, m_rendererHandle);

    float32_t rasterTransform[9];
    rasterTransform[0] = 1.0f;
    rasterTransform[3] = 0.0f;
    rasterTransform[6] = 0.0f;

    rasterTransform[1] = 0.0f;
    rasterTransform[4] = 1.0f;
    rasterTransform[7] = 0.0f;

    rasterTransform[2] = 0.0f;
    rasterTransform[5] = 0.0f;
    rasterTransform[8] = 1.0f;

    result = dwRenderer_set2DTransform(rasterTransform, m_rendererHandle);
    if (DW_SUCCESS != result) return result;

    result = dwRenderer_setPointSize(4.0f, m_rendererHandle);
    if (DW_SUCCESS != result) return result;

    result = dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, m_rendererHandle);
    if (DW_SUCCESS != result) return result;

    result = dwRenderer_setFont(DW_RENDER_FONT_VERDANA_16, m_rendererHandle);

    return result;
}

void ResourceManager::releaseDriveworks()
{
    dwRelease(&m_SDKHandle);
    dwLogger_release();
}

void ResourceManager::releaseRenderer()
{
    dwRenderer_release(&m_rendererHandle);
}

std::string ResourceManager::getArgument(const char *name) const
{
    return gArguments.get(name);
}

dwStatus ResourceManager::initializeResources(int argc,
                                              const char *argv[],
                                              const ProgramArguments* arguments,
                                              void (*userKeyPressCallback)(int))
{
    try {
        initSampleApp(argc, argv, arguments, userKeyPressCallback, 960, 604);
    } catch (const std::runtime_error& e) {
        std::cerr << e.what();
        return DW_INTERNAL_ERROR;
    }

    dwStatus res = initDriveworks();
    if(DW_SUCCESS != res) {
        std::cerr << "Cannot initialize DriveWorks" << std::endl;
        return res;
    }

    //res = initRenderer();
    //if(DW_SUCCESS != res) {
    //    std::cerr << "Cannot initialize DriveWorks' renderer" << std::endl;
    //    return res;
    //}

    res = initSAL();
    if(DW_SUCCESS != res) {
        std::cerr << "Cannot initialize SAL" << std::endl;
        return res;
    }


    return DW_SUCCESS;
}

void ResourceManager::setWindowSize(const int32_t width, const int32_t height) {
    window->setWindowSize(width, height);

    dwRect rect{0, 0, width, height};
    dwRenderer_setRect(rect, m_rendererHandle);
}
