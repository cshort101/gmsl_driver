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

Camera::Camera(dwSensorHandle_t *sensor, dwSensorParams salParams, dwSALHandle_t sal)
{
	
	dwStatus result = dwSAL_createSensor(&salSensor, salParams, sal);

	if (result == DW_SUCCESS) {

		cam.sensor = sensor;

		dwImageProperties cameraImageProperties;
		dwSensorCamera_getImageProperties(&cameraImageProperties,
				DW_CAMERA_PROCESSED_IMAGE,
				this->sensor);

		dwCameraProperties cameraProperties;
		dwSensorCamera_getSensorProperties(&cameraProperties, this->sensor);

		width = cameraImageProperties.width;
		height = cameraImageProperties.height;
		imageType = cameraImageProperties.imageType;
		numSiblings = cameraProperties.siblings;
	}
	else {
		std::cerr << "Cannot create driver: " << salParams.protocol
		    << " with params: " << salParams.parameters << std::endl
		    << "Error: " << dwGetStatusName(result) << std::endl;
		if (result == DW_INVALID_ARGUMENT) {
		    std::cerr << "It is possible the given camera is not supported. "
		}
	}
}

Camera::~Camera()
{
	if (serializer) {
		dwSensorSerializer_stop(serializer);
		dwSensorSerializer_release(&serializer);
    	}
	dwSensor_stop(sensor)
	for (auto frame : rgbaImagePool) {
		NvMediaImageDestroy(frame->img);
		delete frame;
	}
	if (converter) {
		dwImageFormatConverter_release(&converter);
	}
}
	


