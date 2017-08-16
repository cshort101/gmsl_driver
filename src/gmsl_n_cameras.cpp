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
#include <signal.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <memory>

#ifdef LINUX
#include <execinfo.h>
#include <unistd.h>
#endif

#include <cstring>
#include <functional>
#include <list>
#include <iomanip>
#include <thread>

#include <chrono>
#include <mutex>
#include <condition_variable>
//#include <lodepng.h>


#include <ros/ros.h>


// SAMPLE COMMON
#include <Checks.hpp>
#include <WindowGLFW.hpp>
#include <WindowEGL.hpp>
#include <ProgramArguments.hpp>
#include <ConsoleColor.hpp>
#include <ResourceManager.hpp>
#include <RawPipeline.hpp>


// CORE
#include <dw/core/Context.h>
#include <dw/core/Logger.h>

// RENDERER
#include <dw/renderer/Renderer.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>
#include <dw/sensors/camera/Camera.h>

// IMAGE
#include <dw/image/FormatConverter.h>
#include <dw/image/ImageStreamer.h>



#include "cv_connection.hpp"


//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------
static volatile bool g_run = true;
static bool gTakeScreenshot = false;
static int gScreenshotCount = 0;

// 1KB should be plenty for data lines from any sensor
// Actual size is returned during runtime
#define MAX_EMBED_DATA_SIZE (1024 * 1024)
NvMediaISCEmbeddedData sensorData;

// Program arguments
ProgramArguments g_arguments(
    {
	ProgramArguments::Option_t("num-cameras", "1"),
        ProgramArguments::Option_t("camera-type", "ar0231-rccb"),
        ProgramArguments::Option_t("csi-port", "ab"),
        ProgramArguments::Option_t("offscreen", "0"),
        ProgramArguments::Option_t("write-file", ""),
        ProgramArguments::Option_t("serialize-type", "h264"),
        ProgramArguments::Option_t("serialize-bitrate", "8000000"),
        ProgramArguments::Option_t("serialize-framerate", "30"),
        ProgramArguments::Option_t("slave", "0"),
        ProgramArguments::Option_t("fifo-size", "3"),

    });

// Resources
ResourceManager gResources;

//------------------------------------------------------------------------------
// Method declarations
//------------------------------------------------------------------------------
int main(int argc, const char **argv);
void parseArguments(int argc, const char **argv);
void initGL(WindowBase **window);
void initRenderer(dwRendererHandle_t *renderer,
                  dwContextHandle_t context, WindowBase *window);
void initSdk(dwContextHandle_t *context, WindowBase *window);
void initSensors(dwSALHandle_t *sal, dwSensorHandle_t *camera,
                 uint32_t *imageWidth, uint32_t *imageHeight, dwImageType *cameraImageType,
                 dwContextHandle_t context);

void runNvMedia_pipeline(WindowBase *window, dwRendererHandle_t renderer, dwContextHandle_t sdk,
                         dwSensorHandle_t camera, int cameraWidth, int cameraHeight);

void sig_int_handler(int sig);
void keyPressCallback(int key);

//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{

    if (DW_SUCCESS != gResources.initializeResources(argc, argv, &g_arguments, nullptr)) {
	std::cerr << "Sample cannot acquire resources" << std::endl;	
    }

    
    // Cameras
    dwSensorHandle_t cameraSensors[std::stoi(gResources.get("num-cameras"))];   
 
    // Set up linux signal handlers
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = sig_int_handler;

    sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
    sigaction(SIGINT, &action, NULL);  // Ctrl-C
    sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
    sigaction(SIGABRT, &action, NULL); // abort() called.
    sigaction(SIGTERM, &action, NULL); // kill command

    //Init
    g_run = true;

    //initGL(&window);
    //initRenderer(&renderer, sdk, window);

    // create HAL and camera
    uint32_t imageWidth;
    uint32_t imageHeight;
    dwImageType cameraImageType;
    initSensors(gResources.getSAL(), cameraSensors, &imageWidth, &imageHeight, &cameraImageType, gResources.getSDK());

    if(cameraImageType != DW_IMAGE_NVMEDIA)
    {
        std::cerr << "Error: Expected nvmedia image type, received "
                  << cameraImageType << " instead." << std::endl;
        exit(-1);
    }

    // Allocate buffer for parsed embedded data
    sensorData.top.data    = new uint8_t[MAX_EMBED_DATA_SIZE];
    sensorData.bottom.data = new uint8_t[MAX_EMBED_DATA_SIZE];
    sensorData.top.bufferSize    = MAX_EMBED_DATA_SIZE;
    sensorData.bottom.bufferSize = MAX_EMBED_DATA_SIZE;


    runNvMedia_pipeline(gResources.getWindow(), gResources.getRenderer(), gResources.getSDK(), cameraSensors, imageWidth, imageHeight);

    // release used objects in correct order
    dwSAL_releaseSensor(&cameraSensor);
    dwSAL_release(&sal);

    // todo - render release code has been commented out, since that one results in a stall
    //        of the GMSL (nvmedia) pipeline. The issue is known and will be fixed in the future.
    //dwRenderer_release(&renderer);


    delete[] sensorData.top.data;
    delete[] sensorData.bottom.data;

    return 0;
}


//------------------------------------------------------------------------------
void initGL(WindowBase **window)
{
    bool offscreen = atoi(g_arguments.get("offscreen").c_str()) != 0;
    //#ifdef VIBRANTE
      //  if (offscreen)
        //    *window = new WindowOffscreenEGL(1280, 800);
    //#else
        (void)offscreen;
    //#endif

    if(!*window)
        *window = new WindowGLFW(1280, 800);

    (*window)->makeCurrent();
    (*window)->setOnKeypressCallback(keyPressCallback);
}

//------------------------------------------------------------------------------
void initSensors(dwSALHandle_t *sal, dwSensorHandle_t cameras[],
                 uint32_t *imageWidth, uint32_t *imageHeight,
		 dwImageType *cameraImageType,  dwContextHandle_t context)
{
    dwStatus result;

    result = dwSAL_initialize(sal, context);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot initialize SAL: "
                    << dwGetStatusName(result) << std::endl;
        exit(1);
    }


    dwSensorParams params;
    std::string parameterString = g_arguments.parameterString();
    parameterString             += ",output-format=yuv";
    params.parameters           = parameterString.c_str();
    params.protocol             = "camera.gmsl";



    for (int i = 0; i < std::stoi(g_arguments.get("num-cameras")); i++) {
	cameras[i] = DW_NULL_HANDLE;
	// create GMSL Camera interface
        result = dwSAL_createSensor(cameras[i], params, *sal);
        if (result != DW_SUCCESS) {
            std::cerr << "Cannot create driver: camera.gmsl with params: "
            	    << params.parameters << std::endl
            	    << "Error: " << dwGetStatusName(result) << std::endl;
            exit(2);
        }
    } 
    
    dwImageProperties cameraImageProperties;
    dwSensorCamera_getImageProperties(&cameraImageProperties,
                                     DW_CAMERA_PROCESSED_IMAGE,
                                    cameras[0]);
    *imageWidth = cameraImageProperties.width;
    *imageHeight = cameraImageProperties.height;
    *cameraImageType = cameraImageProperties.type;

    dwCameraProperties cameraProperties;
    dwSensorCamera_getSensorProperties(&cameraProperties, cameras[0]);

    std::cout << "Camera image with " << *imageWidth << "x" << *imageHeight
              << " at " << cameraProperties.framerate << " FPS" << std::endl;
}

//------------------------------------------------------------------------------


void runNvMedia_pipeline(WindowBase *window, dwRendererHandle_t renderer, dwContextHandle_t sdk,
                         dwSensorHandle_t cameraSensors[], int cameraWidth, int cameraHeight)
{
    // RGBA image pool for conversion from YUV camera output
    NvMediaDevice *nvmedia;
    dwContext_getNvMediaDevice(&nvmedia, sdk);

    std::vector<dwImageNvMedia *> rgbaImagePool;
    for (int i = 0; i < 4; ++i) {
        NvMediaImageAdvancedConfig advConfig;
        memset(&advConfig, 0, sizeof(advConfig));
        dwImageNvMedia *rgbaImage = new dwImageNvMedia();
        NvMediaImage *rgbaNvMediaImage;

        rgbaNvMediaImage = NvMediaImageCreate(nvmedia, NvMediaSurfaceType_Image_RGBA,
                                              NVMEDIA_IMAGE_CLASS_SINGLE_IMAGE, 1,
                                              cameraWidth, cameraHeight,
                                              0,
                                              &advConfig);
        dwImageNvMedia_setFromImage(rgbaImage, rgbaNvMediaImage);

        rgbaImagePool.push_back(rgbaImage);
    }
    // format converter
    dwImageProperties cameraImageProperties;
    dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE, cameraSensors[0]);
    dwImageProperties displayImageProperties = cameraImageProperties;
    displayImageProperties.pxlFormat = DW_IMAGE_RGBA;
    displayImageProperties.planeCount = 1;

    dwImageFormatConverterHandle_t yuv2rgba = DW_NULL_HANDLE;
    dwStatus status                         = dwImageFormatConverter_initialize(&yuv2rgba,
                                                        &cameraImageProperties,
                                                        &displayImageProperties,
                                                        sdk);
    if (status != DW_SUCCESS) {
        std::cerr << "Cannot initialize pixel format converter" << std::endl;
        exit(1);
    }

    // image API translator
    //dwImageStreamerHandle_t nvm2gl = DW_NULL_HANDLE;
    //dwImageStreamer_initialize(&nvm2gl, &displayImageProperties, DW_IMAGE_GL, sdk);

    //! [nv_docs_popoulate_params_and_call_initializer]
    dwSensorSerializerHandle_t serializer;
    dwSerializerParams serializerParams;
    serializerParams.parameters = "";
    bool recordCamera = !g_arguments.get("write-file").empty();
    if (recordCamera) {
        std::string newParams = "";
        if (g_arguments.has("serialize-type")) {
            newParams +=
                std::string("format=") + std::string(g_arguments.get("serialize-type"));
        }
        if (g_arguments.has("serialize-bitrate")) {
            newParams +=
                std::string(",bitrate=") + std::string(g_arguments.get("serialize-bitrate"));
        }
        if (g_arguments.has("serialize-framerate")) {
            newParams +=
                std::string(",framerate=") + std::string(g_arguments.get("serialize-framerate"));
        }
        newParams += std::string(",type=disk,file=") + std::string(g_arguments.get("write-file"));

        serializerParams.parameters = newParams.c_str();
        serializerParams.onData     = nullptr;

        dwSensorSerializer_initialize(&serializer, &serializerParams, cameraSensor);
        dwSensorSerializer_start(serializer);

    //! [nv_docs_popoulate_params_and_call_initializer]
    }
    // Start Sensor and Processing
    dwCameraProperties cameraProperties;
    dwSensorCamera_getSensorProperties(&cameraProperties, cameraSensor);

    g_run = dwSensor_start(cameraSensor) == DW_SUCCESS;

std::cout << "A" << std::endl;

    int argc; char** argv;
    ros::init(argc, argv, "image_publisher");

    OpenCVConnector cv;

    // Message msg;
    while (g_run) {
        dwCameraFrameHandle_t frameHandle;
        dwImageNvMedia *frame = nullptr;
        uint32_t camera = 0u;
        dwStatus status = dwSensorCamera_readFrame(&frameHandle, camera, 1000000, cameraSensor);
        if (status != DW_SUCCESS) {
            std::cout << "\n ERROR readFrame: " << dwGetStatusName(status) << std::endl;
            continue;
        }

        if( cameraProperties.outputTypes & DW_CAMERA_PROCESSED_IMAGE) {
            status = dwSensorCamera_getImageNvMedia(&frame, DW_CAMERA_PROCESSED_IMAGE, frameHandle);
            if( status != DW_SUCCESS ) {
                std::cout << "\n ERROR getImageNvMedia " << dwGetStatusName(status) << std::endl;
            }
        }
std::cout << "B" << std::endl;
        // get embedded lines
        if( cameraProperties.outputTypes & DW_CAMERA_DATALINES) {
            const dwCameraDataLines* dataLines = nullptr;
            status = dwSensorCamera_getDataLines(&dataLines, frameHandle);
            // parse the data
            if( status == DW_SUCCESS ) {
                status = dwSensorCamera_parseDataNvMedia(&sensorData, dataLines, cameraSensor);
                if( status == DW_SUCCESS ) {
                    std::cout << "Exposure Time (s): " << sensorData.exposureMidpointTime << "\r";// std::endl;
                } else {
                    std::cout << "Could not parse embedded data: " << dwGetStatusName(status) << "\r"; //std::endl;
                }
            } else {
                std::cout << "Error getting datalines: " << dwGetStatusName(status) << "\r"; //std::endl;
            }
        }

std::cout << "C" << std::endl;
        if (frame && recordCamera ) {
            dwSensorSerializer_serializeCameraFrameAsync(frameHandle, serializer);
        }

        // log message
        //std::cout << frame->timestamp_us;
        //std::cout << " IMAGE SIZE " << frame->img->width << "x" << frame->img->height;
        //std::cout << std::endl;

        // Convert from YUV to RGBA
        if (frame && rgbaImagePool.size() > 0) {
            dwImageNvMedia *rgbaImage = rgbaImagePool.back();
            rgbaImagePool.pop_back();

            //std::cout << " CONVERSION YUV->RGBA\n";
            status = dwImageFormatConverter_copyConvertNvMedia(rgbaImage, frame, yuv2rgba);
            if (status != DW_SUCCESS) {
                std::cout << "\n ERROR copyConvert: " << dwGetStatusName(status) << std::endl;
                rgbaImagePool.push_back(rgbaImage);

            } else {

                // take screenshot if requested
                if (true)
                {
                    NvMediaImageSurfaceMap surfaceMap;
                    if (NvMediaImageLock(rgbaImage->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
                    {
                       
 				cv.WriteToOpenCV((unsigned char*)surfaceMap.surface[0].mapping, rgbaImage->prop.width, rgbaImage->prop.height);

                        /*char fname[128];
                        sprintf(fname, "screenshot_%04d.png", gScreenshotCount++);
                        lodepng_encode32_file(fname, (unsigned char*)surfaceMap.surface[0].mapping, rgbaImage->prop.width, rgbaImage->prop.height);
                        NvMediaImageUnlock(rgbaImage->img);
                        gTakeScreenshot = false;
                        std::cout << "SCREENSHOT TAKEN to " << fname << "\n";*/
				NvMediaImageUnlock(rgbaImage->img);
                        rgbaImagePool.push_back(rgbaImage);
                          
                    }else
                    {
                        std::cout << "CANNOT LOCK NVMEDIA IMAGE - NO SCREENSHOT\n";
                    }
                }

 /*                // Send via ImageStreamer to get GL image back
                status = dwImageStreamer_postNvMedia(rgbaImage, nvm2gl);
                if (status != DW_SUCCESS) {
                    std::cout << "\n ERROR postNvMedia: " << dwGetStatusName(status) << std::endl;
                } else {
                    dwImageGL *frameGL = nullptr;
                    status             = dwImageStreamer_receiveGL(&frameGL, 60000, nvm2gl);

                    if (status == DW_SUCCESS && frameGL) {
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                        // render received texture
                        dwRenderer_renderTexture(frameGL->tex, frameGL->target, renderer);

                        dwImageStreamer_returnReceivedGL(frameGL, nvm2gl);
                   }
                }

                // any image returned back, we put back into the pool
                dwImageNvMedia *retimg = nullptr;
                dwImageStreamer_waitPostedNvMedia(&retimg, 33000, nvm2gl);

                if (retimg)
                    rgbaImagePool.push_back(retimg);
 */            }
        }

        dwSensorCamera_returnFrame(&frameHandle);

        if (window)
            window->swapBuffers();
    }
    if (recordCamera) {
        dwSensorSerializer_stop(serializer);
        dwSensorSerializer_release(&serializer);
    }

    dwSensor_stop(cameraSensor);
   // dwImageStreamer_release(&nvm2gl);

    for (auto frame : rgbaImagePool) {
        NvMediaImageDestroy(frame->img);
        delete frame;
    }

    dwImageFormatConverter_release(&yuv2rgba);
}

//------------------------------------------------------------------------------
void sig_int_handler(int sig)
{
    (void)sig;

    g_run = false;
}

//------------------------------------------------------------------------------
void keyPressCallback(int key)
{
}
