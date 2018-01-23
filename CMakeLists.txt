cmake_minimum_required(VERSION 2.8.3)
project(gmsl_driver_nodelet)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
         set(VIBRANTE TRUE)
         add_definitions(-DVIBRANTE)

find_package(CUDA REQUIRED)
find_package(Threads REQUIRED)
find_package(GLEW REQUIRED)
find_package(Driveworks REQUIRED)
find_package(OpenCV REQUIRED )




find_package(catkin REQUIRED COMPONENTS nodelet 
	roscpp 
        gmsl_frame_msg
	std_msgs 
	nav_msgs
	geometry_msgs
  	cv_bridge
  	image_transport
	sensor_msgs)


## Setup include directories
include_directories(${catkin_INCLUDE_DIRS},
        src)

catkin_package(
  LIBRARIES gmsl_driver_nodelet
  CATKIN_DEPENDS nodelet
      roscpp
      gmsl_frame_msg
      std_msgs
      nav_msgs
      geometry_msgs
      cv_bridge
      image_transport
      sensor_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Driveworks_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/driveworks-src
  ${GLEW_INCLUDE_DIR}
)
include_directories(/usr/share/visionworks/sources/3rdparty/nvmedia/
/usr/share/visionworks/VisionWorks-1.6-Samples/3rdparty/glfw3/include/
)



## Create the nodelet tutorial library
add_library(gmsl_driver_nodelet src/main.cpp
        src/OpenCVWrapper.cpp
	src/ConsoleColor.cpp
  	src/DriveNet.cpp
        src/common.cpp
)

target_link_libraries(gmsl_driver_nodelet 
	${catkin_LIBRARIES}
	nvmedia  
  	${GLEW_LIBRARY}
  	${Driveworks_LIBRARY}
  	GLESv2
  	drm
  	${CUDA_LIBRARY}
  	${OpenCV_LIBS})

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(gmsl_driver_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()

## Mark the nodelet library for installations
install(TARGETS gmsl_driver_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_definition.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
