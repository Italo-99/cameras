#!/usr/bin/env python3

from operator import truediv
from tokenize import Single
import rospy
import math
import numpy
import ar_track_alvar
from ar_track_alvar_msgs.msg import AlvarMarker
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point, PointStamped, Transform
from std_msgs.msg import Int32
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#float variables of orientation
roll  = 0.0
pitch = 0.0
yaw   = 0.0

class Landmark(object):
    """ ARTag Landmark Converter"""

    # Initialization function
    def __init__(self):
        super(Landmark, self).__init__()
        rospy.init_node("artag_detector")


    # Callback function (get landmark relative position)
    def callback(self, data):

        if len(data.markers) == 0:
            print("No artag detected")
            id = 0

        print("Len_data")
        print(len(data.markers))

        for i in range(0,len(data.markers)):
            id = data.markers[i].id
            
            """
            print("id")
            print(id)
            print(type(id))
            print(id>0)
            """
            
            if ((id > 0) and (id < 16)):
                x = data.markers[i].pose.pose.position.x
                y = data.markers[i].pose.pose.position.y
                z = data.markers[i].pose.pose.position.z
                rx = data.markers[i].pose.pose.orientation.x
                ry = data.markers[i].pose.pose.orientation.y
                rz = data.markers[i].pose.pose.orientation.z
                w  = data.markers[i].pose.pose.orientation.w
                [roll, pitch, yaw] = euler_from_quaternion([rx,ry,rz,w])
                
                # Calcolo distanza dal landmark (range)
                dist = math.sqrt(x*x + y*y + z*z)           # range

                print("Artag ", id, " detected!")
                print("Relative x: ", x)
                print("Relative y: ", y)
                print("Relative z: ", z)
                print("Distance: ", dist)
                print("Orient x: ", roll)
                print("Orient y: ", pitch)  
                print("Orient z: ", yaw)      
                print() 


def main():
        marker = Landmark()
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, marker.callback)
        rospy.spin()


if __name__ == '__main__':
    main()