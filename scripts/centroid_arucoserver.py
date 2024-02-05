#!/usr/bin/env python

""" CENTROID ARUCO SERVER
The following code has the only aim to compute the centroid of
a fiducial marker detected.
"""

from cables_detection.srv import Cables2D_Poses
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
        rospy.Service('centroid_aruco', Cables2D_Poses, self.handle_centroid_aruco)
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
