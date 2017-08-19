
#ifndef OPEN_CV_CONNECTOR
#define OPEN_CV_CONNECTOR

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <string>


class OpenCVConnector {

public:
   OpenCVConnector(std::string topic_name);

   void WriteToOpenCV(unsigned char*, int, int);


   ros::NodeHandle nh;
   image_transport::ImageTransport it;
   image_transport::Publisher pub;
   std::string topic_name;

   unsigned int counter;

};



#endif

