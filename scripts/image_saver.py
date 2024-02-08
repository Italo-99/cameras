#!/usr/bin/env python

"""
*
 * Software License Agreement (Apache Licence 2.0)
 *
 *  Copyright (c) [2024], [Andrea Pupa] [italo Almirante]
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
 *  Author: [Andrea Pupa] [Italo Almirante]
 *  Created on: [2024-01-17]
*
"""


import rospy,rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class image_saver:

    def __init__(self):
        
        # Counter to save only 10 depth images
        self.counter = 0

        # Path file for color images
        self.rospack_color       = rospkg.RosPack()
        self.package_path_color  = self.rospack_color.get_path('cables_detection')
        self.script_path_color   = self.package_path_color + "/scripts/figures_test/fastdlo/real_images/"

        # Path file for depth images
        self.rospack_depth       = rospkg.RosPack()
        self.package_path_depth  = self.rospack_depth.get_path('cameras')
        self.script_path_depth   = self.package_path_depth + "/images/cables_depth/"

        # Initialize the ROS node
        rospy.init_node('image_saver')

        # Create a subscriber for the depth image topic
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)

        # Create a subscriber for the color image topic
        rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback)

        # Run ROS spinner
        spin_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            spin_rate.sleep()

    def depth_image_callback(self,msg):

        # Extract width and height from the received message
        width = msg.width
        height = msg.height

        # # Print the size information
        # rospy.loginfo(" ")
        # rospy.loginfo("Received image with size: %dx%d", width, height)

        # # Print encoding of pixels
        # rospy.loginfo("Received image with encoding: %s", msg.encoding)      

        # Define the path where you want to save the image
        if self.counter < 10:
            path = self.script_path_depth+"cables" + str(self.counter) + ".jpg"
            # Save the image to the specified path
            cv_image = CvBridge().imgmsg_to_cv2(msg, msg.encoding)
            cv2.imwrite(path, cv_image)
            self.counter += 1

        if self.counter == 10:
            rospy.loginfo("All depth images saved")
            self.counter += 1

    def color_image_callback(self,msg):
        # Extract width and height from the received message
        width = msg.width
        height = msg.height

        # # Print the size information
        # rospy.loginfo(" ")
        # rospy.loginfo("Received image with size: %dx%d", width, height)

        # # Print encoding of pixels 'rgb8'
        # rospy.loginfo("Received image with encoding: %s", msg.encoding)

        # Define the path where you want to save the image
        path = self.script_path_color+"cables.jpg"

        # Save the image to the specified path
        cv_image = CvBridge().imgmsg_to_cv2(msg, msg.encoding)
        cv2.imwrite(path, cv_image)

if __name__ == '__main__':
    
    image_saver()
