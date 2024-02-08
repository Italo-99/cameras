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

# FAKE CAMERA INFO PUBLISHER TO TEST DEPTH IMAGE MEASUREMENT

import rospy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

def publish_camera_info():
    rospy.init_node('camera_info_publisher', anonymous=True)
    pub = rospy.Publisher('/usb_cam/camera_info', CameraInfo, queue_size=10)
    rate = rospy.Rate(100)  # 10 Hz

    camera_info_msg = CameraInfo()
    camera_info_msg.header = Header(seq=1, stamp=rospy.Time.now(), frame_id="usb_cam")
    camera_info_msg.height = 480
    camera_info_msg.width = 640
    camera_info_msg.distortion_model = 'plumb_bob'
    camera_info_msg.D = [0.000000, 0.000000, 0.000000, 0.000000, 0.000000]
    camera_info_msg.K = [609.9419555664062,               0.0, 324.0304870605469, 
                                        0.0, 608.6234130859375, 246.2322540283203, 
                                        0.0,               0.0,               1.0]
    camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info_msg.P = [609.9419555664062, 0.0, 324.0304870605469, 0.0, 0.0,
                         608.6234130859375, 246.2322540283203, 0.0, 0.0, 0.0, 1.0, 0.0]
    camera_info_msg.binning_x = 0
    camera_info_msg.binning_y = 0
    camera_info_msg.roi.x_offset = 0
    camera_info_msg.roi.y_offset = 0
    camera_info_msg.roi.height = 0
    camera_info_msg.roi.width = 0
    camera_info_msg.roi.do_rectify = False

    while not rospy.is_shutdown():

        pub.publish(camera_info_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_info()
    except rospy.ROSInterruptException:
        pass
