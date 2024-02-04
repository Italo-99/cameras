#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Callback functions to handle incoming messages
def image_Aldepth_callback(msg):

    # Print the size information
    rospy.loginfo(" ")
    rospy.loginfo("Received aligned depth image with size: %dx%d", msg.width, msg.height)

    # Print encoding of pixels
    rospy.loginfo("Encoding: %s", msg.encoding)

    # Print step size
    rospy.loginfo("Image steps in bytes: %d", msg.step)

    # Print data size
    rospy.loginfo("Data format with size: %d=%dx%d", 
                    len(msg.data), msg.step, len(msg.data)/msg.step)

    # This data should be interpreted in this way:
    # 16UC1: 16 bits (2 bytes) compose a 16-bit unsigned int value 
    #        for a single pixel to express distance in mm

    # Convert ROS Image message to OpenCV image
    img = CvBridge().imgmsg_to_cv2(msg, msg.encoding)
    # cv2.imshow("Image window",img)
    # cv2.waitKey(3)

    if (msg.encoding == "16UC1"):

        # Return image 16UC1 size and one sample depth information 
        rospy.loginfo("Img size: %dx%d",len(img[0]),len(img))

        rospy.loginfo("Central pixel point is at a distance of %d mm",
                    img[int(msg.width/2)][int(msg.height/2)])
        
            # Save cv image
        file_path = "src/cameras/images/Aldepth.jpg"
        cv2.imwrite(file_path, img)
        
    return img

def image_color_callback(msg):

    # Print the size information
    rospy.loginfo(" ")
    rospy.loginfo("Received Color image with size: %dx%d", msg.width, msg.height)

    # Print encoding of pixels
    rospy.loginfo("Encoding: %s", msg.encoding)

    # Print step size
    rospy.loginfo("Image steps in bytes: %d", msg.step)

    # Print data size
    rospy.loginfo("Data format with size: %d=%dx%d", 
                    len(msg.data), msg.step, len(msg.data)/msg.step)

    # This data should be interpreted in this way:
    # rgb8: uint8 for each color info (red, green and blue) 
    #        for a single pixel to express its color

    # Convert ROS Image message to OpenCV image
    img = CvBridge().imgmsg_to_cv2(msg, msg.encoding)
    # cv2.imshow("Image window",img)
    # cv2.waitKey(3)

    if (msg.encoding == "rgb8"):

        # Return image 16UC1 size and one sample depth information 
        rospy.loginfo("Img size: %dx%dx%d",len(img[0]),len(img),len(img[0][0]))

        rospy.loginfo("Central pixel point has a red quantity of %d",
                    img[int(msg.width/2)][int(msg.height/2)][0])
        
        rospy.loginfo("Central pixel point has a green quantity of %d",
                    img[int(msg.width/2)][int(msg.height/2)][1])
        
        rospy.loginfo("Central pixel point has a blue quantity of %d",
                    img[int(msg.width/2)][int(msg.height/2)][2])
        
        # Save cv image
        file_path = "src/cameras/images/color.jpg"
        cv2.imwrite(file_path, img)
        
    return img

def image_depth_callback(msg):

    # Print the size information
    rospy.loginfo(" ")
    rospy.loginfo("Received depth image with size: %dx%d", msg.width, msg.height)

    # Print encoding of pixels
    rospy.loginfo("Encoding: %s", msg.encoding)

    # Print step size
    rospy.loginfo("Image steps in bytes: %d", msg.step)

    # Print data size
    rospy.loginfo("Data format with size: %d=%dx%d", 
                    len(msg.data), msg.step, len(msg.data)/msg.step)

    # This data should be interpreted in this way:
    # 16UC1: 16 bits (2 bytes) compose a 16-bit unsigned int value 
    #        for a single pixel to express distance in mm

    # Convert ROS Image message to OpenCV image
    img = CvBridge().imgmsg_to_cv2(msg, msg.encoding)
    # cv2.imshow("Image window",img)
    # cv2.waitKey(3)

    if (msg.encoding == "16UC1"):

        # Return image 16UC1 size and one sample depth information 
        rospy.loginfo("Img size: %dx%d",len(img[0]),len(img))

        rospy.loginfo("Central pixel point is at a distance of %d mm",
                    img[int(msg.width/2)][int(msg.height/2)])
        
        # Save cv image
        file_path = "src/cameras/images/depth.jpg"
        cv2.imwrite(file_path, img)
        
    return img

# Main example function
def main():
    # Initialize the ROS node
    rospy.init_node('ex_image_saver_cv')

    # Create a subscriber for the image aligned depth topic
    sub_al_depth = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, image_Aldepth_callback)

    # Create a subscriber for the image aligned depth topic
    sub_color = rospy.Subscriber('/camera/color/image_raw', Image, image_color_callback)

    # Create a subscriber for the image depth rectified raw topic
    sub_depth = rospy.Subscriber('/camera/depth/image_rect_raw', Image, image_depth_callback)

    # Spin to keep the program alive and execute the callback when messages arrive
    rospy.spin()

if __name__ == '__main__':

    main()

# TODO:
    # 1) write a launcher for the camera d435i with minimal args
    # 2) write above functions in a class
    # 3) import fastdlo/core libraries in a menu node python script
    # 4) in this menu, choices are: make a photo, classify cables, get spline distances