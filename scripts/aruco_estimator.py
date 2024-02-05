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

from    cables_detection.srv    import Cable2D_Poses
from    cameras.srv             import Cable3D_Poses
from    cv_bridge               import CvBridge
import  cv2
from    fiducial_msgs.msg       import FiducialArray, FiducialTransformArray
import  math
import  rospy
from    std_msgs.msg            import String
from    sensor_msgs.msg         import Image

class aruco_estimator:

    def __init__(self):

        # Initialize global class variables and params
        self.depth_img_   = Image()         # depth image
        self.color_img_   = Image()         # color image
        self.gt_poses_    = []              # ground truth poses received by aruco pkg
        self.depth_topic_ = '/camera/aligned_depth_to_color/image_raw'  # PARAM
        self.color_topic_ = '/camera/color/image_raw'                   # PARAM
        self.freq_rate_   = 10                                          # PARAM
        self.fx           = 609.9419555664062                           # PARAM
        self.fy           = 608.6234130859375                           # PARAM
        self.cx           = 324.0304870605469                           # PARAM
        self.cy           = 246.2322540283203                           # PARAM
        
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
        self.service_proxy = rospy.ServiceProxy('/centroid_aruco', Cable2D_Poses)
        rospy.loginfo('Connected to centroid_aruco server')

        # Create a server that receives
        rospy.Service('aruco_estimator', Cable3D_Poses, self.handle_3d_poses)
        rospy.loginfo('Ready to compute aruco 3D poses')

    # Depth distance model computation
    def poses_depth_model(self,centroids,dist2D_pixels):

        poses = []
        # Iterate over recevied centroids
        for centroid in centroids:
            rospy.loginfo("Centroids received: [%s , %s]", centroid[0], centroid[1])
            # Pixel coordinate of the centroid
            u = centroid[0]
            v = centroid[1]
            # Get the distance of that centroid
            d = dist2D_pixels[int(u),int(v)]
            # Compute direction vector
            tx = (u-self.cx)/self.fx
            ty = (v-self.cy)/self.fy
            tz = 1
            # Normalize the vector to get the direction versor
            norm = math.sqrt(tx^2+ty^2+tz^2)
            # Compute the poses
            [x,y,z] = d*[tz,-tx,-ty]/norm
            poses.append([x,y,z])
            rospy.loginfo("Pose computed by the model: x = %s, y = %s, z = %s", x,y,z)
            rospy.loginfo("Pose computed by aruco pkg: x = %s, y = %s, z = %s", 
                          self.gt_poses_.x,self.gt_poses_.y,self.gt_poses_.z)

        return poses
    
    # Server function to compute 3D poses of centroids
    def handle_3d_poses(self,req):

        # Create a client to Cable2D_Poses service to get aruco centroids pixels
        color_img = self.color_img_
        response = self.service_proxy(color_img)

        # Compute the distances of each pixel
        # This data should be interpreted in this way:
            # 16UC1: 16 bits (2 bytes) compose a 16-bit unsigned int value 
            #        for a single pixel to express distance in mm
        depth_img = self.depth_img_
        dist2D_pixels = CvBridge().imgmsg_to_cv2(depth_img, depth_img.encoding)

        # Process the response
        poses = self.poses_depth_model(response.centroids,dist2D_pixels)

        # Return poses response
        return poses

    # Callback to transform callback of aruco detect pkg
    def fiducial_transforms_callback(self,msg):

        # Clear previous (x,y,z) poses
        self.gt_poses_ = []

        # Add estimated poses
        for transform in msg.transforms:
            
            self.gt_poses_.append(transform.transform.translation)
            
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

# TODO:
    # 1) write a launcher for the camera d435i with minimal args
    # 2) write above functions in a class
    # 3) import fastdlo/core libraries in a menu node python script
    # 4) in this menu, choices are: make a photo, classify cables, get spline distances