# CAMERAS
This repository contains some useful ROS files to handle smart cameras, such as RealSense and Logitech, carried out by Italo Almirante within the Ars Control Laboratory, with the collaboration of the professor Cristian Secchi and the post-doctoral researcher Andrea Pupa.

# Installation guide

Clone the RealSense repository, as suggested at this link: 
    
    https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy

Clone the Usb Camera repository, as suggested at this link: 

    https://wiki.ros.org/usb_cam

# User guide

IMAGE SAVER:

1) To use the image saver function on the Intel RealSense Camera d435, launch the following command:

    roslaunch cameras d435_image_saver.launch

  Remember to check the   


# Notes

A ROS2 Humble version of this packages is under development.

Use the following terminal command to see the connected devices to your laptop:

        v4l2-ctl --list-devices
