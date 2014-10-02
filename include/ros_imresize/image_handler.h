/*
* image_handler.h
* Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
* Created on: 10/01/2014
* Author: Jéremie Deray
*/

#ifndef IMAGE_HANDLER_H_
#define IMAGE_HANDLER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>


class SingleImageHandler
{

public:

    SingleImageHandler();
    ~SingleImageHandler();

    bool isImageReceived();

protected:

    void setCameraInfo(const sensor_msgs::CameraInfoConstPtr&);
    void topicCallback(const sensor_msgs::ImageConstPtr& received_image);

    sensor_msgs::CameraInfo _infoCam;

    ros::NodeHandle _nh;

    image_transport::ImageTransport _it;

    image_transport::Subscriber _sub_img;
    image_transport::Publisher _pub_img;

    ros::Publisher _pub_info;

    cv::Mat _K;
    cv::Mat _dist;

    bool _infoReceived;
    bool _undistord;

    int _width;
    int _height;
};


#endif  // IMAGE_HANDLER_H_
