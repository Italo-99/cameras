/*
 * Software License Agreement (Apache Licence 2.0)
 *
 *  Copyright (c) [2024], [italo Almirante]
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *   3. The name of the author may not be used to endorse or promote
 *      products derived from this software without specific prior
 *      written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: [Italo Almirante]
 *  Created on: [2024-05-22]
*/

/* CODE DESCRIPTION
    The code in this file implements a class which helps to read camera images
    and save some of them into a specified folder path.
    Launch the parameters using "image_saver.launch" and "image_saver_params.yaml".
*/

#include "image_saver/ImageSaver.h"

// Public constructor
ImageSaver::ImageSaver()
{
    node_name_ = nh_.getNamespace();
    ROS_INFO("Starting node : %s", node_name_.c_str());

    setParams();
}

// ROS spinner
void ImageSaver::spinner()
{
    ros::Rate loop_rate_(frequency_);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate_.sleep();
    }
}

// Set user params
void ImageSaver::setParams()
{
    // Folders path to save images
    if (!nh_.getParam(node_name_+"/depth_path", depth_image_path_))
    {
        ROS_ERROR("Depth path not defined");
        ros::shutdown();
        return;
    }

    if (!nh_.getParam(node_name_+"/color_path", color_image_path_))
    {
        ROS_ERROR("Color path not defined");
        ros::shutdown();
        return;
    }

    // Limit of saving images
    counter_color_ = 0;
    counter_depth_ = 0;

    if (!nh_.getParam(node_name_+"/counter_limit_color", counter_limit_color_))
    {
        ROS_WARN("Counter limit for color images not specified: set to 10 as default");
        counter_limit_color_ = 10;
    }

    if (!nh_.getParam(node_name_+"/counter_limit_depth", counter_limit_depth_))
    {
        ROS_WARN("Counter limit for depth images not specified: set to 10 as default");
        counter_limit_depth_ = 10;
    }

    // Subscribers declarations
    bool depth = false;
    if (!nh_.getParam(node_name_+"/depth", depth))
    {
        ROS_WARN("Depth acquisition non specified: set to false as default");
    }

    if (!nh_.getParam(node_name_+"/depth_topic", depth_topic_))
    {
        ROS_WARN("Depth images topic not defined");
    }

    if (!nh_.getParam(node_name_+"/color_topic", color_topic_))
    {
        ROS_WARN("Color images topic not defined, set to false as default");
    }

    if (depth)
    {
        sub_d_ = nh_.subscribe(depth_topic_, 1, &ImageSaver::depthimageCallback, this);
    }
    sub_c_ = nh_.subscribe(color_topic_, 1, &ImageSaver::colorimageCallback, this);

    // Set image reading frequency
    double frequency = 1.0;
    if (!nh_.getParam(node_name_+"/frequency", frequency_))
    {
        ROS_WARN("Images acquisition frequency not defined, set 1 Hz as default");
    }
}

// Read depth images
void ImageSaver::depthimageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    uint32_t width = msg->width;
    uint32_t height = msg->height;

    ROS_INFO(" ");
    ROS_INFO("Received image with size: %dx%d", width, height);
    ROS_INFO("Received image with encoding: %s", msg->encoding.c_str());
    ROS_INFO("Received image step in bytes: %d", msg->step);
    ROS_INFO("Received image data with size: %lu=%dx%lu", 
                msg->data.size(), msg->step, msg->data.size()/msg->step);

    if (counter_depth_ < counter_limit_depth_)
    {
        std::string path = depth_image_path_ + "/depth/" + std::to_string(counter_depth_) + ".jpg";
        // cv::imwrite(path, msg->data); // Uncomment this when OpenCV is integrated
        counter_depth_++;
    }
}

// Read color images
void ImageSaver::colorimageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    uint32_t width = msg->width;
    uint32_t height = msg->height;

    ROS_INFO(" ");
    ROS_INFO("Received image with size: %dx%d", width, height);
    ROS_INFO("Received image with encoding: %s", msg->encoding.c_str());

    if (counter_color_ < counter_limit_color_)
    {
        std::string path = color_image_path_ + "/color/" + std::to_string(counter_color_) + ".jpg";
        // cv::imwrite(path, msg->data); // TODO: uncomment this when OpenCV is integrated
        counter_color_++;
    }
}