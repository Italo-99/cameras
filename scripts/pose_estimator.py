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
2 - it's a client to fastdlo detector service, which returns pixel coordinates (u,v) 
    of the splines of the detected cables
3 - it's a server which receives a camera pose and return (x,y,z) value of the 
    splines of the detected cables, together with received pixel coordinates (u,v)
"""

from    cables_detection.srv    import Cables2D_Poses
from    cameras.srv             import Obj3D_Poses
from    cv_bridge               import CvBridge
import  cv2
from    geometry_msgs.msg       import PoseArray,PoseStamped,Pose,TransformStamped
import  math
import  numpy as np
import  rospy,rospkg
from    scipy.interpolate       import interp1d
from    sensor_msgs.msg         import Image
import  tf2_ros

class pose_estimator:

    def __init__(self):

        # Initialize the ROS node
        rospy.init_node('pose_estimator_node')

        # Initialize global class variables and params
        self.depth_img_     = Image()
        self.color_img_     = Image()
        # self.depth_message_queue = []
        self.depth_topic_   = rospy.get_param('~depth_topic')
        self.color_topic_   = rospy.get_param('~color_topic')
        loop_rate           = rospy.get_param('~loop_rate')
        self.fx             = rospy.get_param('~fx')
        self.fy             = rospy.get_param('~fy')
        self.cx             = rospy.get_param('~cx')
        self.cy             = rospy.get_param('~cy')
        self.optical_frame  = rospy.get_param('~optical_frame')
        self.base_link_cam  = rospy.get_param('~base_link_cam')
        self.base_link      = rospy.get_param('~base_link')
        detector            = rospy.get_param('~detector')
        self.cal_factor     = rospy.get_param('~cal_factor')
        self.bias_factor    = rospy.get_param('~bias_factor')
        
        # Get transform from base_link_cam to optical_frame
        tf_rec = False
        tf_buffer = tf2_ros.Buffer()
        listener  = tf2_ros.TransformListener(tf_buffer)
        transform = TransformStamped()
        time_now  = rospy.Time.now()
        while (not tf_rec) and (not rospy.is_shutdown()):
            try:
                transform = tf_buffer.lookup_transform(self.base_link_cam, self.optical_frame, rospy.Time())
            except:
                rospy.loginfo("Waiting optical camera transfrom")
            if transform.header.frame_id:
                rospy.loginfo("Optical camera transform found")
                tf_rec = True
            rospy.sleep(0.1)
                
        # Store pose transform from base camera link to optical frame
        self.cam_to_opt_x = transform.transform.translation.x
        self.cam_to_opt_y = transform.transform.translation.y
        self.cam_to_opt_z = transform.transform.translation.z
        
        # Pose array publisher for RVIZ visualization purposes
        self.pub_pa = rospy.Publisher('/detection3D_poses', PoseArray,  queue_size=1)
        # self.pub_ps = rospy.Publisher('/grabbing_pose',     PoseStamped,queue_size=1)

        # Create a subscriber for the image aligned depth topic
        self.sub_al_depth_ = rospy.Subscriber(self.depth_topic_, Image, self.image_Aldepth_callback)

        # Create a subscriber for the image aligned depth topic
        self.sub_color_ = rospy.Subscriber(self.color_topic_, Image, self.image_color_callback)

        # Switch choice for image object detector
        if detector == "cables":
            # Connect as Cable2D poses client to get aruco centroids values
            rospy.wait_for_service('/fastdlo')
            self.service_proxy = rospy.ServiceProxy('/fastdlo', Cables2D_Poses)
            rospy.loginfo('Connected to fastdlo server')

        else:
            rospy.logerror("No image detector specified")
            return

        # Create a server that receives
        rospy.Service('pose3D_estimator', Obj3D_Poses, self.handle_3d_poses)
        rospy.loginfo('Ready to compute 3D poses')

        # ROS python spinner
        spin_rate = rospy.Rate(loop_rate)
        while not rospy.is_shutdown():
            spin_rate.sleep()

    # Depth distance model computation
    def poses_depth_model(self,centroids,dist2D_pixels):

        poses = np.empty((0,3))
        # Iterate over recevied centroids
        for centroid in centroids.poses:
            # rospy.loginfo("Centroids received: [%s , %s]", centroid.position.x, centroid.position.y)
            # Pixel coordinate of the centroid
            u = centroid.position.x
            v = centroid.position.y
            # Get the distance of that centroid
            d = float(dist2D_pixels[int(u),int(v)]/1000)*self.cal_factor+self.bias_factor
            # Compute direction vector
            tx = (u-self.cx)/self.fx
            ty = (v-self.cy)/self.fy
            tz = 1
            # Normalize the vector to get the direction versor
            norm = math.sqrt(tx*tx+ty*ty+tz*tz)
            # Compute the poses -> REFERRED TO CAMERA_BASE_LINK ORIENTATION
            x = +d*tz/norm
            y = -d*ty/norm
            z = -d*tx/norm
            # x = d*tx/norm
            # y = d*ty/norm
            # z = d*tz/norm
            # Add the new pose, check if is not [0.,0.,0.] -> x distance is sufficient for this check
            new_pose = np.array([x,y,z]).reshape(1,3)
            if new_pose[0][0] < 0.01 and (len(poses)!=0):
                new_pose[0] = poses[-1]
                poses = poses[:-1]
            poses = np.append(poses,new_pose,axis=0)
            # rospy.loginfo("Centroid 3D pose: x = %s, y = %s, z = %s", x,y,z)

            # # Create an interpolation function for each dimension
            # interp_func_x = interp1d(poses[0], poses[1], kind='cubic')
            # interp_func_y = interp1d(poses[0], poses[1], kind='cubic')
            # interp_func_z = interp1d(poses[0], poses[2], kind='cubic')

            # # Define new x values for smoother curve
            # x_smooth = np.linspace(min(poses[0]), max(poses[0]), int(len(poses)))

            # # Interpolate y and z values using the new x values
            # y_smooth = interp_func_y(x_smooth)
            # z_smooth = interp_func_z(x_smooth)

            # # Now, y_smooth and z_smooth contain the smoothed values

        # Check if first elements are not 0
        for k in range(len(poses)):
            if poses[0][0] < 0.01:
                poses = poses[1:]
            else:
                break

        # Return results
        return poses
    
    # Server function to compute 3D poses of centroids
    def handle_3d_poses(self,req):

        rospy.loginfo("Begin detection")
        
        # Measure computational time
        start_time_depth_reading = rospy.get_time()
        
        # # Is it possible to uncomment the following code to filter last 5 depth images
        # depth_img = Image()

        # for i in range(len(self.depth_img_.data)):
        #     for j in range(len(self.depth_img_.data)[0]):
        #         for k in range(len(self.message_queue)):
        #             depth_img[i][j] += self.message_queue[k].data[i][j]/len(self.message_queue)        

        # Compute the distances of each pixel
        # This data should be interpreted in this way:
            # 16UC1: 16 bits (2 bytes) compose a 16-bit unsigned int value 
            #        for a single pixel to express distance in mm

        depth_img = self.depth_img_
        dist2D_pixels = CvBridge().imgmsg_to_cv2(depth_img, depth_img.encoding)

        # Print depth reading time
        depth_reading_time = rospy.get_time()-start_time_depth_reading
        rospy.loginfo("Depth reading time: %s",depth_reading_time)

        # Create a client to Image Poses Detector service to get (u,v) coords
        color_img = self.color_img_
        response = self.service_proxy(color_img)

        # Measure init 3d pose computation time
        poses3D_time_start = rospy.get_time()

        # Iterate over detected cables
        poses = []

        for k in range(len(response.cables)):

            # Compute the position of the detected points referred to camera optical frame
            poses_cable = self.poses_depth_model(response.cables[k],dist2D_pixels)

            # Get camera pose
            cam_x = req.camera_pose.pose.position.x
            cam_y = req.camera_pose.pose.position.y
            cam_z = req.camera_pose.pose.position.z

            # Return poses response
            cable_3D = PoseArray()
            cable_3D.header.frame_id = self.base_link
            # Fill cable poses
            for k in range(len(poses_cable)):
                response_pose = Pose()
                response_pose.position.x = poses_cable[k][0] + cam_x + self.cam_to_opt_x
                response_pose.position.y = poses_cable[k][1] + cam_y + self.cam_to_opt_y
                response_pose.position.z = poses_cable[k][2] + cam_z + self.cam_to_opt_z
                cable_3D.poses.append(response_pose)
            
            # Add the cable poses to the list of all cables 
            poses.append(cable_3D)

        # Visualize the first cable on RViz
        if len(poses) > 0:
            self.pub_pa.publish(poses[0])
        # goal_pose = PoseStamped()
        # goal_pose.header.frame_id = self.base_link_cam
        # goal_pose.pose = poses[0].poses[-1]
        # self.pub_ps.publish(goal_pose)

        # Print 3D poses computation time
        poses3D_time = rospy.get_time()-poses3D_time_start
        rospy.loginfo("Pose 3D computation time: %s",poses3D_time)

        rospy.loginfo("End detection")

        return poses,response.cables

    # Callback to depth images
    def image_Aldepth_callback(self,msg):

        self.depth_img_ = msg

        # # Add the incoming message to the queue
        # self.message_queue.append(msg)
        # # Keep only the latest 5 messages in the queue
        # self.message_queue = self.message_queue[-5:]

    # Callback to color images
    def image_color_callback(self,msg):

        self.color_img_ = msg

if __name__ == '__main__':

    pose_estimator()
