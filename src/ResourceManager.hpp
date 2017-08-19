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

#ifndef DRIVEWORKSSDK_RESOURCEMANAGER_HPP__
#define DRIVEWORKSSDK_RESOURCEMANAGER_HPP__

#include <memory>

#include <dw/core/Context.h>
#include <dw/image/ImageStreamer.h>
#include <dw/renderer/Renderer.h>

#include <Window.hpp>
#include <WindowEGL.hpp>
#include <WindowGLFW.hpp>
#include <ProgramArguments.hpp>
#include <dw/sensors/Sensors.h>

// Sample Framework
#include <SampleFramework.hpp>

/**
 * RAII Manager for the resources that are not the object of this sample
 **/

class ResourceManager
{
  private:
    dwContextHandle_t m_SDKHandle;
    dwRendererHandle_t m_rendererHandle;
    dwSALHandle_t m_salHandle;
    
  protected:
    dwStatus initDriveworks();
    dwStatus initSAL();
    dwStatus initRenderer();

    void releaseRenderer();
    void releaseSAL();
    void releaseDriveworks();

  public:
    ResourceManager()
            : m_SDKHandle(DW_NULL_HANDLE)
            , m_rendererHandle(DW_NULL_HANDLE)
            , m_salHandle(DW_NULL_HANDLE)
    {}

    ~ResourceManager();

    dwStatus initializeResources(int argc,
                                 const char *argv[],
                                 const ProgramArguments* arguments,
                                 void (*userKeyPressCallback)(int)) ;

    const dwContextHandle_t getSDK() const
    {
        return m_SDKHandle;
    }

    const dwRendererHandle_t getRenderer() const
    {
        return m_rendererHandle;
    }

    const dwSALHandle_t getSAL() const
    {
        return m_salHandle;
    }

    WindowBase * getWindow()
    {
	return window;
    }

    std::string getArgument(const char *name) const;

    void setWindowSize(const int32_t width, const int32_t height);
    WindowBase *window;

};

#endif //DRIVEWORKSSDK_RESOURCEMANAGER_HPP__
