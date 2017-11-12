# gmsl_driver
Require opencv and ROS installed on PX2. 
Clone all into your ros catkin_ws/src 
After first cmake, need to manually change the directories for driveworks and driveworks lib. I used ccmake to do it.  
Require image_transport and cv_bridge in ROS
Add email address to package.xml

Thanks to Colin Weinshenker for his work on multiple camera driver. 


Update current status: Our multiple camera driver will dramatically drop frames when record a rosbag. So the solution is we save multiple cameras frames to h264 files and only publish timestamp and frame_id to ROS for post processing. This repo is no longer maintained and will leave it at a base work for anyone working on PX2.
