#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2

class Encoder:
    def __init__(self):
        rospy.init_node('raw_to_jpg', anonymous=True)

        # Get the topic names from the launch file or parameters
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')  # default if not provided
        self.compressed_topic = rospy.get_param('~compressed_topic', '/usb_cam/compressed')

        self.bridge = CvBridge()
        self.compressed_pub = rospy.Publisher(self.compressed_topic, CompressedImage, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_callback)

    def image_callback(self, data):
        # Convert the ROS Image message to a CV2 image
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Encode the frame into JPEG format
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]  # Adjust quality if needed
        ret, encoded_image = cv2.imencode('.jpg', cv_image, encode_param)

        if ret:
            self.publish_compressed(encoded_image.tobytes())

    def publish_compressed(self, encoded_frame):
        # Create a CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = encoded_frame

        self.compressed_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    encoder = Encoder()

    try:
        encoder.run()
    except rospy.ROSInterruptException:
        pass
