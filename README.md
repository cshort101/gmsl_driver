# We've changed the driver to be a ROS nodelet so you can `catkin_make` in your ros ws. However, due to NVIDIA confidential requirement, we could only share the CMakeFile, nodelet template etc. All other src files, please modify it to meet your own requirements. Also, we manage it to record videos as h264 files, and only store timestamp and frame info to ROS. For post processing, you can write a h264 publisher to publish images in h264 to ROS, which will work like a ROSbag. 


## ROS driver for NVIDIA gmsl camera(s)

    rosrun gmsl_driver gmsl_driver_node --PARAMETER=VALUE

###### Parameters for camera type and parameters

        --type-ab (default=ar0231-rccb)
        --type-cd (default=ar0231-rccb)
        --type-ef (default=ar0231-rccb)
        --selector-mask (default=11110011)
        
Selector-mask is for camera index for ports ab (A), cd (B), and/or ef (C). 1 is enable 0 is disable. 

The order is A3A2A1A0B3B2B1B0C3C2C1C0. The default value 11110011 is to enable six cameras A0-A3 and B0-B1.

        --offscreen (default=0)
        --slave (default=0)
        --fifo-size (default=3)
    
Camera buffer size, at least to be 3

        --cross-csi-sync (default=1)
        
Default setting: the frames are synchronized over the span of all csi-ports and a latency of 1ms on average (worst case observed was 5 ms) 

         --downsample-rate (default=3)
        
Downsample the ROS publish rate. Default value 3 will publish ROS message at rate of 10Hz

        --broadcast-ros-images (default=1)

Parameter to enable/disable the ROS broadcast of the camera images. If 1, the images are broadcast using a separate topic for each camera (/camera/image/[camera_id]. If 0, no images are broadcast.

## Parameters for writing video stream(s) to disk

        --write-file (default="")
        
The location to save video files. Setting example: --write-file=/media/hard-drive/video-

        --serialize-type (default=h264)
        
The format of the video with two possible options: 'h264' or 'uncompressed'

        --serialize-bitrate (default=8000000)
        --serialize-framerate (default=30)
        
The frame rate of the recorded video can be overwritte by this setting

