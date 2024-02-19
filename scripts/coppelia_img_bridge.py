#!/usr/bin/env python

""" LICENSE
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
 *"""

# The following code reads images from CoppealiSim scene

import rospy, sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class CoppeliaVisionSensor:

    def __init__(self):

        # Init ROS node which interacts with CoppeliaSim
        rospy.init_node('coppelia_img_bridge')
        
        # Define ROS publishers for depth and color images
        self.depth_pub = rospy.Publisher('/coppelia_depth_image', Image, queue_size=1)
        self.color_pub = rospy.Publisher('/coppelia_color_image', Image, queue_size=1)

        # Setup node params
        self.color_vision_sensor    = rospy.get_param('~color_vision_sensor')
        self.depth_vision_sensor    = rospy.get_param('~depth_vision_sensor')
        self.loop_sleep_time        = 1/rospy.get_param('~fps')
        self.img_width              = rospy.get_param('~img_width')
        self.img_height             = rospy.get_param('~img_height')

        # Get vision sensor
        self.client         = RemoteAPIClient()
        self.sim            = self.client.require('sim')
        self.simVision      = self.client.require('simVision')
        self.color_sensor   = self.sim.getObject(self.color_vision_sensor)
        self.depth_sensor   = self.sim.getObject(self.depth_vision_sensor)
        
        # Create a CvBridge object
        self.bridge = CvBridge()
        
        # Main loop
        while not rospy.is_shutdown():

            # Get image info from CoppeliaSim
            depth_img, color_img = self.get_vision_sensor_images()
            
            # Publish depth image
            depth_msg = self.convert_image_to_ros_msg(depth_img)
            self.depth_pub.publish(depth_msg)
            
            # Publish color image
            color_msg = self.convert_image_to_ros_msg(color_img)
            self.color_pub.publish(color_msg)
            
            rospy.sleep(self.loop_sleep_time)

        sys.exit()
    
    def get_vision_sensor_images(self):
        
        # Get depth image
        depth_img = self.get_depth_image()  # 16UC1
        
        # Get color image
        color_img = self.get_color_image()  # RGB8
        
        return depth_img, color_img
    
    def get_depth_image(self):
        
        depth_data, img_dims = self.sim.getVisionSensorDepth(self.depth_sensor)
        print(depth_data)
        depth_img = np.array(depth_data)
        print(len(depth_img))
        depth_img = np.reshape(depth_img, (img_dims[0],img_dims[1]))
        depth_img = (depth_img * 255).astype(np.uint8)
        return depth_img
    
    def get_color_image(self):

        image, resolution = self.simVision.rgbVisionSensorHandle(self.color_sensor)

        print("color image")
        print(image)
        print("Resolution")
        print(resolution) # self.img_width,self.img_height

        image_integers = [b for b in image]
        image_array = np.array(image_integers, dtype=np.uint8).reshape((resolution[1], resolution[0], 3))
        color_img = Image.fromarray(image_array, 'RGB')

        # color_img = np.array(color_data, dtype=np.uint8)
        # color_img = np.reshape(color_img, (resolution[1], resolution[0], 3))
        # color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
        return color_img
    
    def convert_image_to_ros_msg(self, img):

        ros_img_msg = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
        return ros_img_msg

if __name__ == '__main__':
    try:
        CoppeliaVisionSensor()
    except rospy.ROSInterruptException:
        pass
