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

#ifndef IMAGE_SAVER_H
#define IMAGE_SAVER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

class ImageSaver
{
    public:
        ImageSaver();
        void spinner(void);

    private:
        // void setParams(const std::string& depth_topic, const std::string& color_topic, 
        //                const std::string& depth_path,  const std::string& color_path, 
        //                int frequency, unsigned int counter_limit);
        void setParams(void);

        void depthimageCallback(const sensor_msgs::Image::ConstPtr& msg);
        void colorimageCallback(const sensor_msgs::Image::ConstPtr& msg);

        ros::NodeHandle nh_;
        std::string node_name_;

        ros::Subscriber sub_d_;
        ros::Subscriber sub_c_;

        int counter_limit_color_;
        int counter_limit_depth_;
        int counter_color_;
        int counter_depth_;

        float frequency_;

        std::string depth_image_path_;
        std::string color_image_path_;

        std::string color_topic_;
        std::string depth_topic_;
};

#endif // IMAGE_SAVER_H