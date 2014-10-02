/*
* image_handler.h
* Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
* Created on: 10/01/2014
* Author: Jéremie Deray
*/

#include "ros_imresize/image_handler.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/callback_queue.h>
#include <opencv2/imgproc/imgproc.hpp>


////////////////////////////////////////////////////////////////////////////
////////////                                                    ////////////
////////////                   SingleImageHandler               ////////////
////////////                                                    ////////////
////////////////////////////////////////////////////////////////////////////


SingleImageHandler::SingleImageHandler() :
_infoReceived(false),
_nh("/ros_imresize"),
_width(0),
_height(0),
_it(_nh)
{
    std::string imgTopicName;
    std::string infoTopicName;

    ros::Rate wrait(10);

    ROS_INFO("Retrieving parameters ...");

    while(!(_nh.getParam("topic_crop", imgTopicName) &&
          _nh.getParam("camera_info", infoTopicName)) &&
	  ros::ok())
    {
	ros::spinOnce();
	wrait.sleep();
    }

    ROS_INFO("Parameters retrieved ...");

    _nh.param("resize_width", _width, (int)640);
    _nh.param("resize_height", _height, (int)480);

    _nh.param("undistord", _undistord, false);

    ros::Subscriber sub_info = _nh.subscribe(infoTopicName, 1, &SingleImageHandler::setCameraInfo, this);

    ROS_INFO("WAITING for ROS camera calibration!\n");
    ros::Rate rate(10);
    while (!_infoReceived  && ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("RECEIVED ROS camera calibration!\n");

    sub_info.shutdown();

    _sub_img = _it.subscribe(imgTopicName, 1, &SingleImageHandler::topicCallback, this);

    _pub_img = _it.advertise(imgTopicName + "_crop", 1);

    _pub_info = _nh.advertise<sensor_msgs::CameraInfo>(infoTopicName + "_crop", 1);

    ROS_INFO("Running\n");
}

SingleImageHandler::~SingleImageHandler()
{
}

void SingleImageHandler::topicCallback(const sensor_msgs::ImageConstPtr& received_image)
{
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(received_image, sensor_msgs::image_encodings::BGR8);
       
    cv::Mat undist;
 
    if (_undistord)
    {
       cv::undistort(cvPtr->image, undist, _K, _dist);
    }
    else
    {
       undist = cvPtr->image;
    }   

    cv::resize(undist, cvPtr->image, cv::Size(_width, _height),
               0, 0, cv::INTER_LINEAR);

    _pub_img.publish(cvPtr->toImageMsg());
    _pub_info.publish(_infoCam);
}

void SingleImageHandler::setCameraInfo(const sensor_msgs::CameraInfoConstPtr &received_info)
{
    _infoCam = *received_info;

    float scale_x = (float)(_width) / (float)(_infoCam.width);
    float scale_y = (float)(_height) / (float)(_infoCam.height);

    _infoCam.K[0] *= scale_x;
    _infoCam.K[2] *= scale_x;

    _infoCam.K[4] *= scale_y;
    _infoCam.K[5] *= scale_y;

    ROS_INFO_STREAM("Previous camera info :\n" << *received_info << "\n");

    if (_undistord)
    {
        _K = cv::Mat::eye(3, 3, CV_32F);

        _K.at<float>(0) = _infoCam.K[0];
        _K.at<float>(2) = _infoCam.K[2];

        _K.at<float>(4) = _infoCam.K[4];
        _K.at<float>(5) = _infoCam.K[5];

        if (_infoCam.distortion_model == "plumb_bob")
        {
            _dist = cv::Mat(_infoCam.D);
        }
        else
        {
            _dist = cv::Mat::zeros(5, 1, CV_32F);
            //TODO : check for other model
        }

        _infoCam.distortion_model = "";
        _infoCam.D.clear();

        _infoCam.D.clear();

        ROS_INFO_STREAM("Undistortion active with param :\n" << _dist << "\n");
        ROS_INFO_STREAM("Undistortion active with param :\n" << _K << "\n");
    }

    _infoCam.width = _width;
    _infoCam.height = _height;

    ROS_INFO_STREAM("New camera info :\n" << _infoCam << "\n");

    _infoReceived = true;

}
