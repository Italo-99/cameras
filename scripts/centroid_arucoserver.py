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


""" CENTROID ARUCO SERVER
The following code has the only aim to compute the centroid of
a fiducial marker detected.
"""

from cameras.srv import Aruco2D_Poses
from cv_bridge import CvBridge
import cv2
from fiducial_msgs.msg import FiducialArray
from geometry_msgs.msg import PoseArray,Pose
import numpy as np
import rospy
from sensor_msgs.msg import Image


class Centroid_ArucoServer:

    def __init__(self):

        # Initialize global class variables
        self.image = FiducialArray()

        # Initialize ROS node
        rospy.init_node('centroid_arucoserver')
        rospy.Subscriber('/fiducial_vertices', FiducialArray, self.fiducial_vertices_callback)
        rospy.Service('centroid_aruco', Aruco2D_Poses, self.handle_centroid_aruco)
        rospy.loginfo("Ready to compute centroids of the fiducial vertices.")

        # Run ROS spinner
        spin_rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            spin_rate.sleep()

    def fiducial_vertices_callback(self,msg):

        self.image = msg

    def handle_centroid_aruco(self,req):

        centroids = PoseArray()

        for fiducial in self.image.fiducials:

            xc = (fiducial.x0+fiducial.x1+fiducial.x2+fiducial.x3)/4
            yc = (fiducial.y0+fiducial.y1+fiducial.y2+fiducial.y3)/4
            new_point = Pose()
            new_point.position.x = xc
            new_point.position.y = yc
            centroids.poses.append(new_point)
            fiducial_id = fiducial.fiducial_id
            rospy.loginfo("Fiducial ID: %d\nCentroid: %s", fiducial_id, centroids.poses[-1].position)

        return centroids
    
if __name__ == "__main__":
    
    Centroid_ArucoServer()
