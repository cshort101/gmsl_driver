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

#include <ProgramArguments.hpp>

#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/ImageStreamer.h>
#include <dw/image/FormatConverter.h> 

#include <string>
#include <vector>

class Camera 
{
	
	public:
		Camera(dwSensorHandle_t &sensor, dwSensorParams sensorParams, dwSALHandle_t sal, dwContextHandle_t sdk, ProgramArguments arguments,  bool record);
		~Camera();
		void stop_camera();
		bool start();
		//void capture_camera();
		dwSensorHandle_t getSensor() { return sensor;}
	
		
		dwSensorHandle_t sensor;
		uint32_t numSiblings;
		uint32_t width;
		uint32_t height;
		dwImageStreamerHandle_t streamer;
		dwImageFormatConverterHandle_t converter;
		std::vector<dwImageNvMedia *> rgbaImagePool;
		dwSensorSerializerHandle_t serializer;
		dwImageType imageType;
		bool record;
	

	protected:
		void initImagePool(dwContextHandle_t sdk);
		void initFormatter(dwContextHandle_t sdk);
		void initSerializer(ProgramArguments arguments);

};
