#!/usr/bin/env python

""" CAMERA ESTEEMATOR NODE
This node executes three main activities:
1 - it's a subscriber to camera images, color and aligned depth
2 - it's a client to CentroidAruco service, which return pixel coordinates (u,v) 
    of the center of the detected fiducial marker
3 - it's a server which receives a camera pose and return (x,y,z) value of the 
    center of the detected fiducial marker

The evaluation with camera depth info is compared with fiducial transforms.
"""

from    cables_detection.srv    import Cables2D_Poses
from    cameras.srv             import Obj3D_Poses
from    cv_bridge               import CvBridge
import  cv2
from    fiducial_msgs.msg       import FiducialArray, FiducialTransformArray
from    geometry_msgs.msg       import PoseArray,Pose
import  math
import  numpy as np
import  rospy,rospkg
from    std_msgs.msg            import Float32
from    sensor_msgs.msg         import Image

class pose_estimator:

    def __init__(self):

        # Initialize global class variables and params
        self.depth_img_   = Image()         # depth image
        self.color_img_   = Image()         # color image
        self.depth_topic_ = '/camera/aligned_depth_to_color/image_raw'  # PARAM
        self.color_topic_ = '/camera/color/image_raw'                   # PARAM
        loop_rate         = 10                                          # Set loop rate PARAM
        self.fx           = 609.9419555664062                           # PARAM
        self.fy           = 608.6234130859375                           # PARAM
        self.cx           = 324.0304870605469                           # PARAM
        self.cy           = 246.2322540283203                           # PARAM
        self.cam_to_opt_x = 0.  # x offset from base_link to camera_color_optical_frame PARAM
        self.cam_to_opt_y = 0.  # y offset from base_link to camera_color_optical_frame PARAM
        self.cam_to_opt_z = 0.  # z offset from base_link to camera_color_optical_frame PARAM
        detector          = "cables"  # PARAM: what is the detector to activate
        self.frame_id     = "camera_color_optical_frame"    # PARAM: frame_id of the detection
        
        # Initialize the ROS node
        rospy.init_node('pose_estimator_node')
        spin_rate         = rospy.Rate(loop_rate)

        # Pose array publisher for RVIZ visualization purposes
        self.pub = rospy.Publisher('/pose_array_topic', PoseArray, queue_size=1)

        # Create a subscriber for the image aligned depth topic PARAM
        self.sub_al_depth_ = rospy.Subscriber(self.depth_topic_, Image, self.image_Aldepth_callback)

        # Create a subscriber for the image aligned depth topic PARAM
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
            d = float(dist2D_pixels[int(u),int(v)]/1000)
            # Compute direction vector
            tx = (u-self.cx)/self.fx
            ty = (v-self.cy)/self.fy
            tz = 1
            # Normalize the vector to get the direction versor
            norm = math.sqrt(tx*tx+ty*ty+tz*tz)
            # Compute the poses -> REFERRED TO CAMERA_LINK ORIENTATION
            # x = +d*tz/norm
            # y = -d*tx/norm
            # z = -d*ty/norm
            x = d*tx/norm
            y = d*ty/norm
            z = d*tz/norm
            new_pose = np.array([x,y,z]).reshape(1,3)
            poses = np.append(poses,new_pose,axis=0)
            # rospy.loginfo("Centroid 3D pose: x = %s, y = %s, z = %s", x,y,z)

        return poses
    
    # Server function to compute 3D poses of centroids
    def handle_3d_poses(self,req):

        # Compute the distances of each pixel
        # This data should be interpreted in this way:
            # 16UC1: 16 bits (2 bytes) compose a 16-bit unsigned int value 
            #        for a single pixel to express distance in mm
        depth_img = self.depth_img_
        dist2D_pixels = CvBridge().imgmsg_to_cv2(depth_img, depth_img.encoding)

        # Create a client to Image Poses Detector service to get (u,v) coords
        color_img = self.color_img_
        response = self.service_proxy(color_img)

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
            cable_3D.header.frame_id = self.frame_id
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
        self.pub.publish(poses[0])

        return [poses]

    # Callback to depth images
    def image_Aldepth_callback(self,msg):

        self.depth_img_ = msg

    # Callback to color images
    def image_color_callback(self,msg):

        self.color_img_ = msg

if __name__ == '__main__':

    pose_estimator()
