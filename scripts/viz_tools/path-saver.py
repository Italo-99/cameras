#! /usr/bin/env python

import rospy
from nav_msgs.msg import Path
import csv


def path_callback(msg):

	#rospy.loginfo(msg)
    try:
        with open('navigation-path.csv', 'w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)

            for pose_stamped in msg.poses:
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y
                z = pose_stamped.pose.position.z
                quat_x = pose_stamped.pose.orientation.x
                quat_y = pose_stamped.pose.orientation.y
                quat_z = pose_stamped.pose.orientation.z
                quat_w = pose_stamped.pose.orientation.w

                # Scrittura dati nel file csv
                csv_writer.writerow([x, y, z, quat_x, quat_y, quat_z, quat_w])

        rospy.loginfo("Dati salvati")
    
    except:
        rospy.logerr("Si è verificato un errore")



if __name__ == '__main__':

    rospy.init_node('path-msg-saver')
    sub = rospy.Subscriber('/rover_path', Path, callback=path_callback)     # ottiene il messaggio pubbilcato dal nodo "path_plotter" 
    rospy.spin()