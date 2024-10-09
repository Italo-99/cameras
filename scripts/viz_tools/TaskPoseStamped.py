#!/usr/bin/env python3

from operator import truediv
from tokenize import Single
import rospy
import math
import numpy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PointStamped, Transform, PoseStamped
from std_msgs.msg import Int32
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Pose_Stamped(object):
    """ Pose Stamped Visualizer"""

    # Initialization function
    def __init__(self):
        super(Pose_Stamped, self).__init__()
        rospy.init_node("trajectory_visualizer")

        waypoint_publisher = rospy.Publisher("TrajectoryVisualizer", Path, queue_size=1)

        self.waypoint_publisher = waypoint_publisher

        self.msg = Path()
        self.msg.header.frame_id = "odom"

        self.init_time = rospy.get_time()

    # Callback function (prtin rover poses into a path)
    def callback(self, data):
        
        time2 = rospy.get_time()

        if time2 - self.init_time > 1.0 :
        
            self.msg.header.stamp = rospy.Time.now()
            
            pose = PoseStamped()
            pose.pose.position  = data.pose.pose.position
            pose.pose.orientation = data.pose.pose.orientation
            self.msg.poses.append(pose)

            self.waypoint_publisher.publish(self.msg)
            rospy.loginfo("Published {} waypoints.".format(len(self.msg.poses)))  

            print("Rover position")
            print("X: ", pose.pose.position.x)
            print("Y: ", pose.pose.position.y)
            print("Z: ", pose.pose.position.z)

            self.init_time = rospy.get_time()


def main():
        Pose = Pose_Stamped()
        rospy.Subscriber("odometry/filtered", Odometry, Pose.callback)
        rospy.spin()

if __name__ == '__main__':
    main()