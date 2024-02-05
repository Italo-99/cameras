#!/usr/bin/env python

""" CENTROID ARUCO SERVER
The following code has the only aim to compute the centroid of
a fiducial marker detected.
"""

import rospy
from sensor_msgs.msg import Image
from fiducial_msgs.msg import FiducialArray
from cables_detection.srv import Cable2D_Poses
from cv_bridge import CvBridge
import cv2

class Centroid_ArucoServer:

    def __init__(self):

        # Initialize global class variables
        self.image = FiducialArray()

        # Initialize ROS node
        rospy.init_node('centroid_arucoserver')
        rospy.Subscriber('/fiducial_vertices', FiducialArray, self.fiducial_vertices_callback)
        rospy.Service('centroid_aruco', Cable2D_Poses, self.handle_centroid_aruco)
        rospy.loginfo("Ready to compute centroids of the fiducial vertices.")

        # Run ROS spinner
        spin_rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            spin_rate.sleep()

    def fiducial_vertices_callback(self,msg):

        self.image = msg

    def handle_centroid_aruco(self,req):

        self.centroids = []

        for fiducial in self.image.fiducials:

            xc = (fiducial.x0+fiducial.x1+fiducial.x2+fiducial.x3)/4
            yc = (fiducial.y0+fiducial.y1+fiducial.y2+fiducial.y3)/4
            self.centroids.append([xc,yc])
            # fiducial_id = fiducial.fiducial_id
            # rospy.loginfo("Fiducial ID: %d, Centroid: %s", fiducial_id, self.centroids[-1])

        return self.centroids

if __name__ == "__main__":
    
    Centroid_ArucoServer()
