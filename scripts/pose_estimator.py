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
from    geometry_msgs.msg       import Point,PoseArray,PoseStamped,Pose,TransformStamped
import  math
import  numpy as np
import  quaternion
import  rospy,rospkg
from    scipy.interpolate       import interp1d,NearestNDInterpolator
from    scipy.spatial.transform import Rotation as R
from    sensor_msgs.msg         import Image
from    tf.transformations      import quaternion_from_euler
import  tf2_ros

class pose_estimator:

    def __init__(self):

        # Initialize the ROS node
        rospy.init_node('pose_estimator_node')

        # Initialize global class variables and params
        self.depth_buffer_  = []
        self.depth_img_     = Image()
        self.color_img_     = Image()
        self.depth_topic_   = rospy.get_param('~depth_topic')
        self.color_topic_   = rospy.get_param('~color_topic')
        loop_rate           = rospy.get_param('~loop_rate')
        self.fx             = rospy.get_param('~fx')
        self.fy             = rospy.get_param('~fy')
        self.cx             = rospy.get_param('~cx')
        self.cy             = rospy.get_param('~cy')
        self.optical_frame  = rospy.get_param('~optical_frame')
        self.base_link_cam  = rospy.get_param('~base_link_cam')
        detector            = rospy.get_param('~detector')
        self.cal_factor     = rospy.get_param('~cal_factor')
        self.bias_factor    = rospy.get_param('~bias_factor')
        self.coppelia       = rospy.get_param('~coppelia')
        self.cam_bound_low  = rospy.get_param('~cam_bound_low')
        self.cam_bound_up   = rospy.get_param('~cam_bound_up')
        
        # Get transform from base_link_cam to optical_frame
        tf_rec = False
        tf_buffer = tf2_ros.Buffer()
        listener  = tf2_ros.TransformListener(tf_buffer)
        transform = TransformStamped()
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
        self.pub_ps = rospy.Publisher('/example_grab_pose', PoseStamped,queue_size=1)

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

        rospy.signal_shutdown('Shutdown node')

    # Depth distance model computation
    def poses_depth_model(self,centroids,dist2D_pixels):

        poses = np.empty((0,3))

        # Iterate over recevied centroids
        for centroid in centroids.poses:

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
            x = +d*tz/norm  # x = d*tx/norm if optical frame is oriented as camera_base_link
            y = -d*ty/norm  # y = d*ty/norm if optical frame is oriented as camera_base_link
            z = -d*tx/norm  # z = d*tz/norm if optical frame is oriented as camera_base_link

            # Add the new pose, check if is not [0.,0.,0.] -> x distance is sufficient for this check
            new_pose = np.array([x,y,z]).reshape(1,3)
            if new_pose[0][0] < 0.01 and (len(poses)!=0):
                new_pose[0] = poses[-1]
                poses = poses[:-1]
            poses = np.append(poses,new_pose,axis=0)

        """ INTERPOLATION: not working because it's on one axis
        # Create an interpolation function for each dimension
        # interp_func_z = interp1d(poses[:,1], poses[:,2], kind='cubic') -> GOOD
        # interp_func_x = interp1d(poses[:,1], poses[:,0], kind='cubic') -> GOOD
        # interp_func_y = interp1d(poses[:,0], poses[:,1], kind='cubic') -> NOT GOOD
        # interp_func_z = interp1d(poses[:,0], poses[:,2], kind='cubic') -> NOT GOOD

        # Define new x values for smoother curve
        # x_smooth = np.linspace(min(poses[:,0]), max(poses[:,0]), int(len(poses))) -> NOT GOOD
        # y_smooth = np.linspace(min(poses[:,1]), max(poses[:,1]), int(len(poses))) # -> GOOD

        # # Interpolate y and z values using the new x values
        # poses[:,1] = interp_func_y(x_smooth) -> NOT GOOD
        # poses[:,2] = interp_func_z(x_smooth) -> NOT GOOD
        # poses[:,2] = interp_func_z(y_smooth)    # -> GOOD
        # poses[:,0] = interp_func_x(y_smooth)    # -> GOOD
        #"""
  
        # Interpolate along (y,z) for a smoother shape along x
        # interp_func = NearestNDInterpolator(poses[:,1:],poses[:,0])
        # z_smooth    = np.linspace(min(poses[:,2]), max(poses[:,2]), int(len(poses)))
        # y_smooth    = np.linspace(min(poses[:,1]), max(poses[:,1]), int(len(poses)))
        # poses[:,0]  = interp_func(y_smooth,z_smooth)

        # Check if first elements are not 0
        for k in range(len(poses)):
            if poses[0][0] < 0.01:
                poses = poses[1:]
            else:
                break

        # Return results
        return poses
    
    # Transform a position from the camera_base_link frame to world frame reference
    def ref_to_world(self,position,camera_pose):

        # Create a Pose message for tool0
        position_world = Point()

        # Compute the quaternion of child frame orientation
        q = np.array(([ camera_pose.pose.orientation.w,
                        camera_pose.pose.orientation.x,
                        camera_pose.pose.orientation.y,
                        camera_pose.pose.orientation.z]))

        # Compute the corresponding rotation matrix
        rot_mat = R.from_quat(q).as_matrix()

        # Compute the translation vector
        camera_pos = np.array(([ camera_pose.pose.position.x,
                                 camera_pose.pose.position.y,
                                 camera_pose.pose.position.z,1]))

        # Compute the transform
        transform = np.zeros((4,4))
        transform[0:3,0:3] = rot_mat
        transform[0:4,3]   = camera_pos

        # Get the numpy array of the vector to transform
        pos_rel   = np.array([position.x,position.y,position.z,1]).reshape((-1, 1))

        # Get the numpy array of the vector transformed
        pos_world = np.dot(transform,pos_rel)
        pos_world = pos_world[0:3]

        # Fill position msg referred to the world
        position_world.x = pos_world[0]
        position_world.y = pos_world[1]
        position_world.z = pos_world[2]

        return position

    # Server function to compute 3D poses of centroids
    def handle_3d_poses(self,req):

        rospy.loginfo("Begin detection")
        
        # Measure computational time
        start_time_depth_reading = rospy.get_time()

        """ Compute the distances of each pixel
            This data should be interpreted in this way:
            16UC1: 16 bits (2 bytes) compose a 16-bit unsigned int value 
                   for a single pixel to express distance in mm
        """

        # If sim setup on Coppelia is on, convert depth image as range map
        if self.coppelia:
            # depth_img     = self.depth_buffer_[-1]
            dist2D_pixels = CvBridge().imgmsg_to_cv2(self.depth_img_, self.depth_img_.encoding)
            dist2D_pixels = 1000*(dist2D_pixels[:,:,2]/255*(self.cam_bound_up-self.cam_bound_low)+self.cam_bound_low)
        # If real setup is on, convert depth image as 16UC1 format    
        else:
            # dist2D_pixels_np = np.zeros((self.depth_buffer_[-1].height,
            #                          self.depth_buffer_[-1].width,
            #                          len(self.depth_buffer_)))
            # for k in range(len(self.depth_buffer_)):
            #     dist2D_pixels_np[:,:,k] = CvBridge().imgmsg_to_cv2(
            #                             self.depth_buffer_[k],
            #                             self.depth_buffer_[k].encoding)
            # dist2D_pixels = np.sum(dist2D_pixels_np,axis=2)/len(self.depth_buffer_)
            dist2D_pixels = CvBridge().imgmsg_to_cv2(
                                        self.depth_img_,
                                        self.depth_img_.encoding)

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

            # Return poses response
            cable_3D = PoseArray()
            cable_3D.header.frame_id = self.base_link_cam
            # Fill cable poses
            for k in range(len(poses_cable)):
                response_pose = Pose()
                response_pose.orientation.w = 1
                response_pose.position.x = poses_cable[k][0] + self.cam_to_opt_x
                response_pose.position.y = poses_cable[k][1] + self.cam_to_opt_y
                response_pose.position.z = poses_cable[k][2] + self.cam_to_opt_z
                # response_pose.position   = self.ref_to_world(response_pose.position,
                #                                             req.camera_pose)
                cable_3D.poses.append(response_pose)
            
            # Add the cable poses to the list of all cables 
            poses.append(cable_3D)

        # Visualize the first cable on RViz
        if len(poses) > 0:

            self.pub_pa.publish(poses[0])
        
            # # Show a sample goal pose stamped value on rviz
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = self.base_link_cam
            goal_pose.pose = poses[0].poses[-1]
            self.pub_ps.publish(goal_pose)

        # Print 3D poses computation time
        poses3D_time = rospy.get_time()-poses3D_time_start
        rospy.loginfo("Pose 3D computation time: %s",poses3D_time)

        rospy.loginfo("End detection")

        return poses,response.cables

    # Callback to depth images
    def image_Aldepth_callback(self,msg):

        self.depth_img_ = msg

        """ BUFFER EXPERIMENT NOT WORKING
            # Keep only the latest 5 messages in the buffer
            self.depth_buffer_ = self.depth_buffer_[-4:]
            # Add the incoming message to the queue
            self.depth_buffer_.append(msg)
        """

    # Callback to color images
    def image_color_callback(self,msg):

        self.color_img_ = msg

if __name__ == '__main__':

    pose_estimator()
