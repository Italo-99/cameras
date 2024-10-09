#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import pickle
from std_srvs.srv import TriggerResponse
from sirio_utilities.srv import SaveImages, SaveImagesResponse  # Custom service import
import roslib.packages

class CameraImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_storage = {}

        # Get the path to your ROS package
        package_path = roslib.packages.get_pkg_dir('sirio_utilities')

        # Directory to store images and counters
        self.save_directory = os.path.join(package_path, 'saved_camera_feeds')
        self.counter_file = os.path.join(self.save_directory, "counters.pkl")

        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)

        # Load or initialize image counters
        if os.path.exists(self.counter_file):
            with open(self.counter_file, 'rb') as f:
                self.image_counters = pickle.load(f)
        else:
            self.image_counters = {
                "d435": 0,
                "d435i": 0,
                "t265_1": 0,
                "t265_2": 0,
                "log_1": 0,
                "log_2": 0,
                "zed_right": 0,
                "zed_left": 0
            }

        # Define subscribers for each camera
        rospy.Subscriber("/d435/color/image_raw",               Image, self.image_callback, "d435")
        rospy.Subscriber("/d435i/color/image_raw",              Image, self.image_callback, "d435i")
        rospy.Subscriber("/t265/t265/fisheye1/image_raw",       Image, self.image_callback, "t265_1")
        rospy.Subscriber("/t265/t265/fisheye2/image_raw",       Image, self.image_callback, "t265_2")
        rospy.Subscriber("/log_1/image_raw",                    Image, self.image_callback, "log_1")
        rospy.Subscriber("/log_2/image_raw",                    Image, self.image_callback, "log_2")
        rospy.Subscriber("/zed/right/image_raw",                Image, self.image_callback, "zed_right")
        rospy.Subscriber("/zed/left/image_raw",                 Image, self.image_callback, "zed_left")

        # Service to save images on demand
        self.save_service = rospy.Service('/save_images', SaveImages, self.save_images)  # Change to custom service

    def image_callback(self, data, camera_name):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Store the latest image in the dictionary
        self.image_storage[camera_name] = cv_image

    def save_images(self, req):
        camera_name = req.camera_name.lower()

        print(camera_name)
        if camera_name == "all":
            cameras_to_save = self.image_storage.keys()
        else:
            cameras_to_save = [camera_name] if camera_name in self.image_storage else []

        if not cameras_to_save:
            return SaveImagesResponse(success=False, message=f"Camera '{camera_name}' not found or no image available.")

        # Save the images as PNG files
        for cam_name in cameras_to_save:
            image = self.image_storage[cam_name]

            # Generate the filename with the counter
            image_filename = f"{cam_name}_{self.image_counters[cam_name]}.png"
            
            # Increment the counter for the next image
            self.image_counters[cam_name] += 1
            
            # Save the image as a .png file
            cv2.imwrite(os.path.join(self.save_directory, image_filename), image)
            
            rospy.loginfo(f"Saved image as {image_filename}")

        # Save the updated counters
        with open(self.counter_file, 'wb') as f:
            pickle.dump(self.image_counters, f)

        return SaveImagesResponse(success=True, message=f"Images from {', '.join(cameras_to_save)} saved successfully.")

    def run(self):
        
        rate = rospy.Rate(10)  # 10hz
    
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('camera_image_saver', anonymous=True)
    saver = CameraImageSaver()
    saver.run()
