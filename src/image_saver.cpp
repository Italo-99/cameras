#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

// Counter to save only 10 depth images
unsigned int counter = 0;

void depthimageCallback(const sensor_msgs::Image::ConstPtr& msg)
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
    // Define the path where you want to save the image
    if (counter < 10){
    std::string path = "/src/cameras/images/cables_depth_images/cables"
                        +std::to_string(counter)+".jpg";}

    // Save the image to the specified path
    // cv::imwrite(path, msg->data);
}

void colorimageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // Extract width and height from the received message
    uint32_t width = msg->width;
    uint32_t height = msg->height;

    // Print the size information
    ROS_INFO(" ");
    ROS_INFO("Received image with size: %dx%d", width, height);

    // // Print encoding of pixels 'rgb8'
    ROS_INFO("Received image with encoding: %s", msg->encoding.c_str());

    // Define the path where you want to save the image
    std::string path = "/src/cables_detection/scripts/figures_test/fastdlo/real_images/cables.jpg";

    // Save the image to the specified path
    // cv::imwrite(path, msg->data);
}


int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "image_saver");
    
    // Create a ROS NodeHandle
    ros::NodeHandle nh;

    // Create a subscriber for the depth image topic
    ros::Subscriber sub_d = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depthimageCallback);

    // Create a subscriber for the color image topic
    ros::Subscriber sub_c = nh.subscribe("/camera/color/image_raw", 1, colorimageCallback);

    // Spin to keep the program alive and execute the callback when messages arrive
    ros::Rate loop_rate(10);
    while (ros::ok()) {ros::spinOnce();loop_rate.sleep();}

    return 0;
}
