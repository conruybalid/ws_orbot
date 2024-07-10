import rclpy
from rclpy.node import Node

from custom_interfaces.srv import GetLocation
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
from get_pic.VideoQueue import VideoStreamHandler


import pyzed.sl as sl

import numpy as np


class ZedPublisher(Node):
    def __init__(self):
        super().__init__('zed_location_service_node')
        self.distancepublisher_ = self.create_service(GetLocation, 'zed_location_service', self.process_image_callback)
        self.maskpublisher = self.create_publisher(Image, 'zed_mask_topic', 10)
        self.get_logger().info('Distance publisher node has been initialized')


        init = sl.InitParameters(depth_mode=sl.DEPTH_MODE.ULTRA,
                                 coordinate_units=sl.UNIT.MILLIMETER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)

        self.zed = sl.Camera()
        status = self.zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            


        # Create a ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.sdk_verbose = 1 # Enable verbose logging
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE # Set the depth mode to performance (fastest)
        init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print("Error {}, exit program".format(err)) # Display the error
            

            # Capture 50 images and depth, then stop
        
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()
        self.runtime_parameters = sl.RuntimeParameters()
        


        



    def process_image_callback(self,  request, response):
        # Grab an image
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # A new image is available if grab() returns sl.ERROR_CODE.SUCCESS

            # Retrieve the image, depth, and point cloud
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT) # Get the left image
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA) # Retrieve colored point cloud

            # Convert the image to HSV color space
            hsv_image = cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGR2HSV)

            # Define the lower and upper bounds for the red color in HSV
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])

            # Create a mask for the red color
            mask = cv2.inRange(hsv_image, lower_red, upper_red)

            # Find the contours of the red mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Find the center of the largest contour
            if len(contours) > 0:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])

                    x, y, z, something = self.point_cloud.get_value(center_x, center_y)[1]

                    self.get_logger().info(f'point cloud: {x}, {y}, {z}, {something}')

                    # # Convert the center coordinates to meters
                    x_distance = (-x + 509) / 1000
                    y_distance = (-y + 840) / 1000
                    z_distance = (z - 175) / 1000

                    self.get_logger().info(f'x_distance: {x_distance}, y_distance: {y_distance}, z_distance: {z_distance}')

                    # print("Distance from left of the image (x-axis): {} meters".format(x_distance))
                    # print("Distance from top of the image (y-axis): {} meters".format(y_distance))
                    response.apple_coordinates.x = x_distance
                    response.apple_coordinates.y = y_distance
                    response.apple_coordinates.z = z_distance
            
                    mask_msg = CvBridge().cv2_to_imgmsg(mask)
                    self.maskpublisher.publish(mask_msg)

                    return response
                else:
                    self.get_logger().info("No red mask found. (Too Small)")
                    response.apple_coordinates.x = 0.0
                    response.apple_coordinates.y = 0.0
                    response.apple_coordinates.z = 0.0

            else:
                self.get_logger().info("No red mask found.")
                response.apple_coordinates.x = 0.0
                response.apple_coordinates.y = 0.0
                response.apple_coordinates.z = 0.0

            mask_msg = CvBridge().cv2_to_imgmsg(mask)
            self.maskpublisher.publish(mask_msg)

        else:
            self.get_logger().info("Could not access the camera.")
            response.apple_coordinates.x = 0.0
            response.apple_coordinates.y = 0.0
            response.apple_coordinates.z = 0.0




        return response
    
    
                

def main(args=None):
    rclpy.init(args=args)
    zed_service = ZedPublisher()
    # Check if the stream is opened successfully  
    rclpy.spin(zed_service)
    zed_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
