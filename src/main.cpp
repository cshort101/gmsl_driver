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
#include <vector>

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
#include <SampleFramework.hpp>


#include "cv_connection.hpp"

#include "Camera.hpp"

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
        ProgramArguments::Option_t("type-ab", "ar0231-rccb"),
	ProgramArguments::Option_t("type-cd", "ar0231-rccb"),
	ProgramArguments::Option_t("type-ef", "ar0231-rccb"),
	ProgramArguments::Option_t("selector-mask", "1111"),
        ProgramArguments::Option_t("csi-port", "ab"),
	ProgramArguments::Option_t("cross-csi-sync", "0"),
        ProgramArguments::Option_t("offscreen", "0"),
        ProgramArguments::Option_t("write-file", ""),
        ProgramArguments::Option_t("serialize-type", "h264"),
        ProgramArguments::Option_t("serialize-bitrate", "8000000"),
        ProgramArguments::Option_t("serialize-framerate", "30"),
        ProgramArguments::Option_t("slave", "0"),
        ProgramArguments::Option_t("fifo-size", "3"),

    });

// Resource Manager
ResourceManager gResources;
uint32_t g_numCameras;

//------------------------------------------------------------------------------
// Method declarations
//------------------------------------------------------------------------------
int main(int argc, const char **argv);
void initGL(WindowBase **window);
void initSensors(dwSALHandle_t sal, std::vector<Camera> &cameras);

void runNvMedia_pipeline(WindowBase *window, dwRendererHandle_t renderer, dwContextHandle_t sdk,
                         std::vector<Camera> &cameras);

void sig_int_handler(int sig);
void keyPressCallback(int key);

//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{

    gResources.initializeResources(argc, argv, &g_arguments, keyPressCallback);
    std::vector<Camera> cameras;

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

    // create HAL and camera
    uint32_t imageWidth;
    uint32_t imageHeight;
    dwImageType cameraImageType;

    initSensors(gResources.getSAL(), cameras);
    
 
    for (auto &camera : cameras) {
	    if(camera.imageType != DW_IMAGE_NVMEDIA)
	    {
		std::cerr << "Error: Expected nvmedia image type, received "
			  << cameraImageType << " instead." << std::endl;
		exit(-1);
	    }
    }

    // Allocate buffer for parsed embedded data
    sensorData.top.data    = new uint8_t[MAX_EMBED_DATA_SIZE];
    sensorData.bottom.data = new uint8_t[MAX_EMBED_DATA_SIZE];
    sensorData.top.bufferSize    = MAX_EMBED_DATA_SIZE;
    sensorData.bottom.bufferSize = MAX_EMBED_DATA_SIZE;


    runNvMedia_pipeline(gResources.getWindow(), gResources.getRenderer(), gResources.getSDK(), cameras);

    // release used objects in correct order
    for (auto &camera : cameras)
    	dwSAL_releaseSensor(&camera.sensor);

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

    if(!gResources.getWindow())
        gResources.window = new WindowGLFW(1280, 800);

    (gResources.window)->makeCurrent();
    (gResources.window)->setOnKeypressCallback(keyPressCallback);
}


//------------------------------------------------------------------------------
void initSensors(dwSALHandle_t sal, std::vector<Camera> &cameras)
{

    std::string selector = g_arguments.get("selector-mask");

    dwStatus result;

    // identify active ports
    int idx             = 0;
    int cnt[3]          = {0, 0, 0};
    std::string port[3] = {"ab", "cd", "ef"};
    for (size_t i = 0; i < selector.length() && i < 12; i++, idx++) {
        const char s = selector[i];
        if (s == '1') {
            cnt[idx / 4]++;
        }
    }



    // how many cameras selected in a port
    g_numCameras = 0;
    for (size_t p = 0; p < 3; p++) {
        if (cnt[p] > 0) {
            std::string params;

            params += std::string("csi-port=") + port[p];
            params += ",camera-type=" + g_arguments.get((std::string("type-") + port[p]).c_str());
            params += ",camera-count=4"; // when using the mask, just ask for all cameras, mask will select properly

            if (selector.size() >= p*4) {
                params += ",camera-mask="+ selector.substr(p*4, std::min(selector.size() - p*4, size_t{4}));
            }

            params += ",slave="  + g_arguments.get("slave");
            params += ",cross-csi-sync="  + g_arguments.get("cross-csi-sync");
            params += ",fifo-size="  + g_arguments.get("fifo-size");
	    params += ",output-format=yuv";

            dwSensorHandle_t salSensor = DW_NULL_HANDLE;
            dwSensorParams salParams;
            salParams.parameters = params.c_str();
            salParams.protocol = "camera.gmsl";
	   
	    Camera *cam = new Camera(salSensor, salParams, gResources.getSAL());
            cameras.push_back(*cam);
	    (g_numCameras) += cam->numSiblings;
        }    
    }
}



//------------------------------------------------------------------------------


void runNvMedia_pipeline(WindowBase *window, dwRendererHandle_t renderer, dwContextHandle_t sdk,
                         std::vector<Camera> &cameras)
{


    bool recordCamera = !g_arguments.get("write-file").empty();
    

    // RGBA image pool for conversion from YUV camera output
    for (auto &camera : cameras) {

	NvMediaDevice *nvmedia;
	dwContext_getNvMediaDevice(&nvmedia, sdk);

	for (int i = 0; i < 4; ++i) {
		NvMediaImageAdvancedConfig advConfig;
		memset(&advConfig, 0, sizeof(advConfig));
		dwImageNvMedia *rgbaImage = new dwImageNvMedia();
		NvMediaImage *rgbaNvMediaImage;

		rgbaNvMediaImage = NvMediaImageCreate(nvmedia, NvMediaSurfaceType_Image_RGBA,
						      NVMEDIA_IMAGE_CLASS_SINGLE_IMAGE, 1,
						      camera.width, camera.height,
						      0,
						      &advConfig);
		dwImageNvMedia_setFromImage(rgbaImage, rgbaNvMediaImage);


		camera.rgbaImagePool.push_back(rgbaImage);
	}


	std::cout << "Image pool created" << std::endl;
   

	// format converter

	dwImageProperties cameraImageProperties;
	dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE, camera.sensor);
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
	else {
		camera.converter = yuv2rgba;
		std::cout << "Pixel format converter created" << std::endl;
	}

	// image API translator
	//dwImageStreamerHandle_t nvm2gl = DW_NULL_HANDLE;
	//dwImageStreamer_initialize(&nvm2gl, &displayImageProperties, DW_IMAGE_GL, sdk);

	//! [nv_docs_popoulate_params_and_call_initializer]
	dwSensorSerializerHandle_t serializer;
	dwSerializerParams serializerParams; 
	serializerParams.parameters = "";
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
			
		dwSensorSerializer_initialize(&serializer, &serializerParams, camera.sensor);
		camera.serializer = serializer;
		dwSensorSerializer_start(camera.serializer);
	} 
	//! [nv_docs_popoulate_params_and_call_initializer]

	g_run &= dwSensor_start(camera.sensor) == DW_SUCCESS;
	std::cout << "Serializers created" << std::endl;

    }


    std::cout << "A" << std::endl;

    int argc; char** argv;
    ros::init(argc, argv, "image_publisher");

    std::vector<OpenCVConnector *> cv_connectors;
    for (int i = 0; i < std::stoi(g_arguments.get("num-cameras")); i++) {
	cv_connectors.push_back(new OpenCVConnector("camera/" + std::to_string(i) + "/image"));
    }
    std::cout << "CV connectors created" << std::endl;

    // Message msg;
    while (g_run) {
	for (int i = 0; i < cameras.size(); i++) {
		Camera camera = cameras[i];

		//Get Camera properties
		dwCameraProperties cameraProperties;
		dwSensorCamera_getSensorProperties(&cameraProperties, camera.sensor);
		
		dwCameraFrameHandle_t frameHandle;
		dwImageNvMedia *frame = nullptr;
		dwStatus status = dwSensorCamera_readFrame(&frameHandle, camera.numSiblings, 1000000, camera.sensor);
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
			status = dwSensorCamera_parseDataNvMedia(&sensorData, dataLines, camera.sensor);
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
		    dwSensorSerializer_serializeCameraFrameAsync(frameHandle, camera.serializer);
		}

		// log message
		//std::cout << frame->timestamp_us;
		//std::cout << " IMAGE SIZE " << frame->img->width << "x" << frame->img->height;
		//std::cout << std::endl;

		// Convert from YUV to RGBA
		if (frame && camera.rgbaImagePool.size() > 0) {
		    dwImageNvMedia *rgbaImage = camera.rgbaImagePool.back();
		    camera.rgbaImagePool.pop_back();

		    //std::cout << " CONVERSION YUV->RGBA\n";
		    status = dwImageFormatConverter_copyConvertNvMedia(rgbaImage, frame, camera.converter);
		    if (status != DW_SUCCESS) {
			std::cout << "\n ERROR copyConvert: " << dwGetStatusName(status) << std::endl;
			camera.rgbaImagePool.push_back(rgbaImage);

		    } else {

			// take screenshot if requested
			if (true)
			{
			    NvMediaImageSurfaceMap surfaceMap;
			    if (NvMediaImageLock(rgbaImage->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
			    {
					std::cout << "Writing to Open CV" << std::endl;
			       
					cv_connectors[i]->WriteToOpenCV((unsigned char*)surfaceMap.surface[0].mapping, rgbaImage->prop.width, rgbaImage->prop.height);

				/*char fname[128];
				sprintf(fname, "screenshot_%04d.png", gScreenshotCount++);
				lodepng_encode32_file(fname, (unsigned char*)surfaceMap.surface[0].mapping, rgbaImage->prop.width, rgbaImage->prop.height);
				NvMediaImageUnlock(rgbaImage->img);
				gTakeScreenshot = false;
				std::cout << "SCREENSHOT TAKEN to " << fname << "\n";*/
					NvMediaImageUnlock(rgbaImage->img);
				camera.rgbaImagePool.push_back(rgbaImage);
				  
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
    }
}

////------------------------------------------------------------------------------
//void sig_int_handler(int sig)
//{
//    (void)sig;
//
//    g_run = false;
//}
//
////------------------------------------------------------------------------------
//void keyPressCallback(int key)
//{
//}
