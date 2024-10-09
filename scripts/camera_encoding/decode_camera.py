#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class Decoder:
    def __init__(self):
        rospy.init_node('jpg_to_raw', anonymous=True)

        # Get the topic names from the launch file or parameters
        self.compressed_topic = rospy.get_param('~compressed_topic', '/usb_cam/compressed')  # default topic
        self.raw_topic = rospy.get_param('~raw_topic', '/usb_cam/decoded_image_raw')  # default output topic

        self.bridge = CvBridge()
        self.raw_pub = rospy.Publisher(self.raw_topic, Image, queue_size=1)
        rospy.Subscriber(self.compressed_topic, CompressedImage, self.decode_callback)

    def decode_callback(self, data):
        # Convert the CompressedImage data to a numpy array
        np_arr = np.frombuffer(data.data, dtype=np.uint8)

        # Decode the JPEG image into an OpenCV image
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is not None:
            # Convert the CV2 image to a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.raw_pub.publish(ros_image)

    def run(self):
        rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    decoder = Decoder()

    try:
        decoder.run()
    except rospy.ROSInterruptException:
        pass
