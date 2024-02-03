#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

def publish_camera_info():
    rospy.init_node('camera_info_publisher', anonymous=True)
    pub = rospy.Publisher('/usb_cam/camera_info', CameraInfo, queue_size=10)
    rate = rospy.Rate(100)  # 10 Hz

    while not rospy.is_shutdown():
        camera_info_msg = CameraInfo()
        camera_info_msg.header = Header(seq=1, stamp=rospy.Time.now(), frame_id="usb_cam")
        camera_info_msg.height = 480
        camera_info_msg.width = 640
        camera_info_msg.distortion_model = ''
        camera_info_msg.D = [0.000000, 0.000000, 0.000000, 0.000000, 0.000000]
        camera_info_msg.K = [609.9419555664062,               0.0, 324.0304870605469, 
                                           0.0, 608.6234130859375, 246.2322540283203, 
                                           0.0,               0.0,               1.0]
        camera_info_msg.R = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info_msg.P = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info_msg.binning_x = 0
        camera_info_msg.binning_y = 0
        camera_info_msg.roi.x_offset = 0
        camera_info_msg.roi.y_offset = 0
        camera_info_msg.roi.height = 0
        camera_info_msg.roi.width = 0
        camera_info_msg.roi.do_rectify = False

        pub.publish(camera_info_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_info()
    except rospy.ROSInterruptException:
        pass
