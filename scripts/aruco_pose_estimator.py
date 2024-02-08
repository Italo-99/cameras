#!/usr/bin/env python

"""
*
 * Software License Agreement (Apache Licence 2.0)
 *
 *  Copyright (c) [2024], [Andrea Pupa] [italo Almirante]
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *   3. The name of the author may not be used to endorse or promote
 *      products derived from this software without specific prior
 *      written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: [Andrea Pupa] [Italo Almirante]
 *  Created on: [2024-01-17]
*
"""


""" CAMERA ESTEEMATOR NODE
This node executes three main activities:
1 - it's a subscriber to camera images, color and aligned depth
2 - it's a client to CentroidAruco service, which return pixel coordinates (u,v) 
    of the center of the detected fiducial marker
3 - it's a server which receives a camera pose and return (x,y,z) value of the 
    center of the detected fiducial marker

The evaluation with camera depth info is compared with fiducial transforms.
"""

from    cameras.srv             import Aruco2D_Poses
from    cameras.srv             import Aruco3D_Pose
from    cv_bridge               import CvBridge
import  cv2
from    fiducial_msgs.msg       import FiducialArray, FiducialTransformArray
from    geometry_msgs.msg       import PoseArray,Pose
import  math
import  numpy as np
import  rospy
from    std_msgs.msg            import Float32
from    sensor_msgs.msg         import Image

class aruco_estimator:

    def __init__(self):

        # Initialize global class variables and params
        self.depth_img_   = Image()         # depth image
        self.color_img_   = Image()         # color image
        self.gt_poses_    = np.empty((0,3)) # ground truth poses received by aruco pkg
        self.depth_topic_ = '/camera/aligned_depth_to_color/image_raw'  # PARAM
        self.color_topic_ = '/camera/color/image_raw'                   # PARAM
        self.freq_rate_   = 50                                          # PARAM
        self.fx           = 609.9419555664062                           # PARAM
        self.fy           = 608.6234130859375                           # PARAM
        self.cx           = 324.0304870605469                           # PARAM
        self.cy           = 246.2322540283203                           # PARAM
        self.cam_to_opt_x = 0.  # x offset from base_link to camera_color_optical_frame PARAM
        self.cam_to_opt_y = 0.  # y offset from base_link to camera_color_optical_frame PARAM
        self.cam_to_opt_z = 0.  # z offset from base_link to camera_color_optical_frame PARAM
        
        # Initialize the ROS node
        rospy.init_node('aruco_estimator_node')

        # Create a subscriber for the image aligned depth topic PARAM
        self.sub_al_depth_ = rospy.Subscriber(self.depth_topic_, Image, self.image_Aldepth_callback)

        # Create a subscriber for the image aligned depth topic PARAM
        self.sub_color_ = rospy.Subscriber(self.color_topic_, Image, self.image_color_callback)

        # Subscribe to the fiducial_transforms topic
        rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.fiducial_transforms_callback)

        # Connect as Cable2D poses client to get aruco centroids values
        rospy.wait_for_service('/centroid_aruco')
        self.service_proxy = rospy.ServiceProxy('/centroid_aruco', Aruco2D_Poses)
        rospy.loginfo('Connected to centroid_aruco server')

        # Create a server that receives
        rospy.Service('aruco_estimator', Aruco3D_Pose, self.handle_3d_poses)
        rospy.loginfo('Ready to compute aruco 3D poses')

    # Depth distance model computation
    def poses_depth_model(self,centroids,dist2D_pixels):

        poses = np.empty((0,3))
        index = 0
        # Iterate over recevied centroids
        for centroid in centroids.poses:
            rospy.loginfo("Centroids received: [%s , %s]", centroid.position.x, centroid.position.y)
            # Pixel coordinate of the centroid
            u = centroid.position.x
            v = centroid.position.y
            # Get the distance of that centroid
            d = float(dist2D_pixels[int(u),int(v)]/1000)
            print(d)
            # Compute direction vector
            tx = (u-self.cx)/self.fx
            ty = (v-self.cy)/self.fy
            tz = 1
            # Normalize the vector to get the direction versor
            norm = math.sqrt(tx*tx+ty*ty+tz*tz)
            # Compute the poses -> REFERRED TO CAMERA_LINK ORIENTATION
            x = +d*tz/norm
            y = -d*tx/norm
            z = -d*ty/norm
            new_pose = np.array([x,y,z]).reshape(1,3)
            poses = np.append(poses,new_pose,axis=0)
            rospy.loginfo("Pose computed by the model: x = %s, y = %s, z = %s", x,y,z)
            rospy.loginfo("Pose computed by aruco pkg: x = %s, y = %s, z = %s", 
                          self.gt_poses_[index][2],-self.gt_poses_[index][0],-self.gt_poses_[index][1])
            index += 1

        return poses
    
    # Server function to compute 3D poses of centroids
    def handle_3d_poses(self,req):

        # Compute the distances of each pixel
        # This data should be interpreted in this way:
            # 16UC1: 16 bits (2 bytes) compose a 16-bit unsigned int value 
            #        for a single pixel to express distance in mm
        depth_img = self.depth_img_
        dist2D_pixels = CvBridge().imgmsg_to_cv2(depth_img, depth_img.encoding)

        # Create a client to Cable2D_Poses service to get aruco centroids pixels
        color_img = self.color_img_
        response = self.service_proxy(color_img)

        # Compute the position of the detected points referred to camera optical frame
        poses = self.poses_depth_model(response.centroid,dist2D_pixels)

        # Get camera pose
        cam_x = req.camera_pose.pose.position.x
        cam_y = req.camera_pose.pose.position.y
        cam_z = req.camera_pose.pose.position.z

        # Return poses response
        response3D = PoseArray()
        for k in range(len(poses)):
            response_pose = Pose()
            response_pose.position.x = poses[k][0] + cam_x + self.cam_to_opt_x
            response_pose.position.y = poses[k][1] + cam_y + self.cam_to_opt_y
            response_pose.position.z = poses[k][2] + cam_z + self.cam_to_opt_z
            response3D.poses.append(response_pose)

        return response3D,response.centroid

    # Callback to transform callback of aruco detect pkg
    def fiducial_transforms_callback(self,msg):

        # Clear previous (x,y,z) poses
        self.gt_poses_ = np.empty((0,3))

        # Add estimated poses
        for transform in msg.transforms:
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            new_fiducial = np.array([x,y,z]).reshape(1,3)
            self.gt_poses_ = np.append(self.gt_poses_,new_fiducial, axis=0)
            
            # orientation = transform.transform.rotation
            # fiducial_id = transform.fiducial_id
            # rospy.loginfo("Fiducial ID: %d, Pose: [%f, %f, %f], Orientation: [%f, %f, %f, %f]",
            #             fiducial_id,
            #             pose.x, pose.y, pose.z,
            #             orientation.x, orientation.y, orientation.z, orientation.w)
 
    # Callback to depth images
    def image_Aldepth_callback(self,msg):

        self.depth_img_ = msg

        #     # Print the size information
        #     rospy.loginfo(" ")
        #     rospy.loginfo("Received aligned depth image with size: %dx%d", msg.width, msg.height)

        #     # Print encoding of pixels
        #     rospy.loginfo("Encoding: %s", msg.encoding)

        #     # Print step size
        #     rospy.loginfo("Image steps in bytes: %d", msg.step)

        #     # Print data size
        #     rospy.loginfo("Data format with size: %d=%dx%d", 
        #                     len(msg.data), msg.step, len(msg.data)/msg.step)

        #     # This data should be interpreted in this way:
        #     # 16UC1: 16 bits (2 bytes) compose a 16-bit unsigned int value 
        #     #        for a single pixel to express distance in mm

        #     # Convert ROS Image message to OpenCV image
        #     img = CvBridge().imgmsg_to_cv2(msg, msg.encoding)
        #     # cv2.imshow("Image window",img)
        #     # cv2.waitKey(3)

        #     if (msg.encoding == "16UC1"):

        #         # Return image 16UC1 size and one sample depth information 
        #         rospy.loginfo("Img size: %dx%d",len(img[0]),len(img))

        #         rospy.loginfo("Central pixel point is at a distance of %d mm",
        #                     img[int(msg.width/2)][int(msg.height/2)])
                
        #             # Save cv image
        #         file_path = "src/cameras/images/Aldepth.jpg"
        #         cv2.imwrite(file_path, img)
                
        #     return img

    # Callback to color images
    def image_color_callback(self,msg):

        self.color_img_ = msg

        #     # Print the size information
        #     rospy.loginfo(" ")
        #     rospy.loginfo("Received Color image with size: %dx%d", msg.width, msg.height)

        #     # Print encoding of pixels
        #     rospy.loginfo("Encoding: %s", msg.encoding)

        #     # Print step size
        #     rospy.loginfo("Image steps in bytes: %d", msg.step)

        #     # Print data size
        #     rospy.loginfo("Data format with size: %d=%dx%d", 
        #                     len(msg.data), msg.step, len(msg.data)/msg.step)

        #     # This data should be interpreted in this way:
        #     # rgb8: uint8 for each color info (red, green and blue) 
        #     #        for a single pixel to express its color

        #     # Convert ROS Image message to OpenCV image
        #     img = CvBridge().imgmsg_to_cv2(msg, msg.encoding)
        #     # cv2.imshow("Image window",img)
        #     # cv2.waitKey(3)

        #     if (msg.encoding == "rgb8"):

        #         # Return image 16UC1 size and one sample depth information 
        #         rospy.loginfo("Img size: %dx%dx%d",len(img[0]),len(img),len(img[0][0]))

        #         rospy.loginfo("Central pixel point has a red quantity of %d",
        #                     img[int(msg.width/2)][int(msg.height/2)][0])
                
        #         rospy.loginfo("Central pixel point has a green quantity of %d",
        #                     img[int(msg.width/2)][int(msg.height/2)][1])
                
        #         rospy.loginfo("Central pixel point has a blue quantity of %d",
        #                     img[int(msg.width/2)][int(msg.height/2)][2])
                
        #         # Save cv image
        #         file_path = "src/cameras/images/color.jpg"
        #         cv2.imwrite(file_path, img)
                
        #     return img

    # ROS python spinner
    def spinner(self):

        # Set loop rate
        spin_rate = rospy.Rate(self.freq_rate_)
        # Run ROS spinner
        while not rospy.is_shutdown():
            spin_rate.sleep()

if __name__ == '__main__':

    a = aruco_estimator()
    a.spinner()
