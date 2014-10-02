/*
* image_handler.h
* Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
* Created on: 10/01/2014
* Author: Jéremie Deray
*/

#include "ros_imresize/image_handler.h"

#include <ros/ros.h>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "ros_imresize");

    SingleImageHandler resize;

    ros::spin();

    return 0;
}
