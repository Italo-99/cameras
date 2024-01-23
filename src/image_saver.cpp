#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

// Callback function to handle incoming messages
void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // Extract width and height from the received message
    uint32_t width = msg->width;
    uint32_t height = msg->height;

    // Print the size information
    ROS_INFO(" ");
    ROS_INFO("Received image with size: %dx%d", width, height);

    // // Print encoding of pixels
    ROS_INFO("Received image with encoding: %s", msg->encoding.c_str());

    // Print step size
    ROS_INFO("Received image step in bytes: %d", msg->step);

    // Print data size
    ROS_INFO("Received image data with size: %lu=%dx%lu", 
                msg->data.size(),msg->step,msg->data.size()/msg->step);

    // This data should be interpreted in this way:
    // 16UC1: 16 bites (2 bytes) compose a 16 bit unsigned int value 
    //        for a single pixel to express distance in mm
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "image_saver");
    
    // Create a ROS NodeHandle
    ros::NodeHandle nh;

    // Create a subscriber for the image topic
    ros::Subscriber sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, imageCallback);

    // Spin to keep the program alive and execute the callback when messages arrive
    ros::spin();

    return 0;
}
