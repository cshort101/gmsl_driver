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


#include <Camera.hpp>

#include <iostream>
#include <cstring>





/*--------------------------------------------------------------------------*/


Camera::Camera(dwSensorHandle_t &sensor, dwSensorParams salParams, dwSALHandle_t sal, dwContextHandle_t sdk, ProgramArguments arguments,  bool record)
{
	// Init dwHandles
	this->sensor     = DW_NULL_HANDLE;
	streamer   = DW_NULL_HANDLE;
	serializer = DW_NULL_HANDLE;
	

	// Are we recording the camera?
	this->record = false || record;
	
	//Initialize sensors
	dwStatus result = dwSAL_createSensor(&sensor, salParams, sal);

	// Initialize camera and image properties
	if (result == DW_SUCCESS) {

		this->sensor = sensor;

		dwImageProperties cameraImageProperties;
		dwSensorCamera_getImageProperties(&cameraImageProperties,
				DW_CAMERA_PROCESSED_IMAGE,
				this->sensor);

		dwCameraProperties cameraProperties;
		dwSensorCamera_getSensorProperties(&cameraProperties, this->sensor);

		width = cameraImageProperties.width;
		height = cameraImageProperties.height;
		imageType = cameraImageProperties.type;
		numSiblings = cameraProperties.siblings;
		std::cout << "Camera siblings: " << numSiblings <<  std::endl;
	}
	else {
		std::cerr << "Cannot create driver: " << salParams.protocol
		    << " with params: " << salParams.parameters << std::endl
		    << "Error: " << dwGetStatusName(result) << std::endl;
		if (result == DW_INVALID_ARGUMENT) {
		    std::cerr << "It is possible the given camera is not supported. " << std::endl;
		}
	}
	
	initImagePool(sdk);
	initFormatter(sdk);
	if (record)  initSerializer(arguments);
}

/*--------------------------------------------------------------------------*/


/*

Initialize RGBA Image Pool for each Camera Port

*/
void Camera::initImagePool(dwContextHandle_t sdk)
{

	NvMediaDevice *nvmedia;
	dwContext_getNvMediaDevice(&nvmedia, sdk);


        for (int i = 0; i < numSiblings; ++i) {

                NvMediaImageAdvancedConfig advConfig;
                memset(&advConfig, 0, sizeof(advConfig));
                dwImageNvMedia *rgbaImage = new dwImageNvMedia();
                NvMediaImage *rgbaNvMediaImage;


                rgbaNvMediaImage = NvMediaImageCreate(nvmedia, NvMediaSurfaceType_Image_RGBA,
                                                      NVMEDIA_IMAGE_CLASS_SINGLE_IMAGE, 1,
                                                      width, height,
                                                      0,
                                                      &advConfig);
                dwImageNvMedia_setFromImage(rgbaImage, rgbaNvMediaImage);


                rgbaImagePool.push_back(rgbaImage);
        }


        std::cout << "Image pool created" << std::endl;

}

/*--------------------------------------------------------------------------*/



/* Initialize the image formatter */

void Camera::initFormatter(dwContextHandle_t sdk) {

	dwImageProperties cameraImageProperties;
        dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE, sensor);
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
                converter = yuv2rgba;
                std::cout << "Pixel format converter created" << std::endl;
        }
}

/*--------------------------------------------------------------------------*/


/* Initialize a data serializer for the camera */
void Camera::initSerializer(ProgramArguments arguments) {
	
        dwSerializerParams serializerParams;
        serializerParams.parameters = "";
        if (record) {
                std::string newParams = "";
                if (arguments.has("serialize-type")) {
                        newParams +=
                        std::string("format=") + std::string(arguments.get("serialize-type"));
                }
                if (arguments.has("serialize-bitrate")) {
                        newParams +=
                        std::string(",bitrate=") + std::string(arguments.get("serialize-bitrate"));
                }
                if (arguments.has("serialize-framerate")) {
                        newParams +=
                        std::string(",framerate=") + std::string(arguments.get("serialize-framerate"));
                }
                newParams += std::string(",type=disk,file=") + std::string(arguments.get("write-file"));

                serializerParams.parameters = newParams.c_str();
                serializerParams.onData     = nullptr;

                dwSensorSerializer_initialize(&serializer, &serializerParams, sensor);
              }
        //! [nv_docs_popoulate_params_and_call_initializer]

       std::cout << "Serializer created" << std::endl;
}

/*--------------------------------------------------------------------------*/


/* Read frames from every camera on the csi-port and add them to the image pool */




/*--------------------------------------------------------------------------*/


/* Start camera and serialization */
bool Camera::start() 
{	
	if (record) {
		dwSensorSerializer_start(serializer);
	} 
	return dwSensor_start(sensor) == DW_SUCCESS;
}

/*--------------------------------------------------------------------------*/

/* Stop the camera sensor and release all resources */
void Camera::stop_camera() 
{
	if (record) {
		dwSensorSerializer_stop(serializer);
		dwSensorSerializer_release(&serializer);
	}

	dwSensor_stop(sensor);

	for (auto frame : rgbaImagePool) {
		NvMediaImageDestroy(frame->img);
		delete frame;
	}

	dwImageFormatConverter_release(&converter);
}


Camera::~Camera() {}
	


