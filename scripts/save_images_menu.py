#!/usr/bin/env python3
import rospy
from sirio_utilities.srv import SaveImages, SaveImagesRequest

class SaveImagesMenu:

    def __init__(self):
        rospy.init_node('save_images_menu', anonymous=True)
        rate = rospy.Rate(10)

        if not rospy.is_shutdown():
            rate.sleep()
            rospy.wait_for_service('/save_images')

        while not rospy.is_shutdown():
            print('\nSelect which camera to take a photo')
            print('1: d435')
            print('2: d435i')
            print('3: t265_1')
            print('4: t265_2')
            print('5: log_1')
            print('6: log_2')
            print('7: zed_right')
            print('8: zed_left')
            print('0: EXIT')

            choice = input("Enter your choice: ")

            if choice == '1':
                self.call_save_images_service('d435')
            elif choice == '2':
                self.call_save_images_service('d435i')
            elif choice == '3':
                self.call_save_images_service('t265_1')
            elif choice == '4':
                self.call_save_images_service('t265_2')
            elif choice == '5':
                self.call_save_images_service('log_1')
            elif choice == '6':
                self.call_save_images_service('log_2')
            elif choice == '7':
                self.call_save_images_service('zed_right')
            elif choice == '8':
                self.call_save_images_service('zed_left')
            elif choice == '0':
                break
            else:
                print("Invalid choice. Please try again.")

            rate.sleep()

    def call_save_images_service(self,camera_name):
        try:
            # Create a proxy for the save_images service
            save_images = rospy.ServiceProxy('/save_images', SaveImages)
        
            # Prepare the request
            req = SaveImagesRequest()
            req.camera_name = camera_name  # Set the camera name

            # Call the service
            resp = save_images(req)
        
            # Check the response (this depends on what the service returns)
            if resp.success:  # Assume there's a success field in the response
                print(f"Image saved for camera: {camera_name}")
            else:
                print(f"Failed to save image for camera: {camera_name}")

        except rospy.ServiceException as e:
            print(f"Service call for {camera_name} failed: {e}")


if __name__ == '__main__':
    try:
        menu = SaveImagesMenu()
    except rospy.ROSInterruptException:
        pass