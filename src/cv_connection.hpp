
#ifndef OPEN_CV_CONNECTOR
#define OPEN_CV_CONNECTOR

#include <ros/ros.h>
#include <image_transport/image_transport.h>



class OpenCVConnector {

public:
   OpenCVConnector();

   void WriteToOpenCV(unsigned char*, int, int);


   ros::NodeHandle nh;
   image_transport::ImageTransport it;
   image_transport::Publisher pub;

   unsigned int counter;

};



#endif

