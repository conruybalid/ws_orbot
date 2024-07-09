import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
from get_pic.VideoQueue import VideoStreamHandler


import pyzed.sl as sl

import numpy as np


class ZedPublisher(Node):
    def __init__(self):
        super().__init__('zed_distance_publisher')
        self.distancepublisher_ = self.create_publisher(Float32, 'zed_distance_topic', 10)
        self.maskpublisher = self.create_publisher(Image, 'zed_mask_topic', 10)
        self.timer_ = self.create_timer(1.0, self.publish_distance)
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
        


        



    def publish_distance(self):
        # Grab an image
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # A new image is available if grab() returns sl.ERROR_CODE.SUCCESS
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT) # Get the left image
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH) # Retrieve depth matrix. Depth is aligned on the left RGB image
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA) # Retrieve colored point cloud
            

                    # Get and print distance value in mm at the center of the image
            # We measure the distance camera - object using Euclidean distance
            width = self.image.get_width()
            height = self.image.get_height()
            x = round(self.image.get_width() / 2)
            y = round(self.image.get_height() / 2)
            err, point_cloud_value = self.point_cloud.get_value(x, y)
            # distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
            #                      point_cloud_value[1] * point_cloud_value[1] +
            #                      point_cloud_value[2] * point_cloud_value[2])
            # print("Distance to Camera at ({0}, {1}): {2} mm".format(x, y, distance), end="\r")  

            # Retrieve the image, depth, and point cloud
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT) # Get the left image
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH) # Retrieve depth matrix. Depth is aligned on the left RGB image
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

                    # Get the depth value at the center of the mask
                    depth_value = self.depth.get_value(center_x, center_y)

                    # # Convert the center coordinates from pixels to meters
                    # x_distance = center_x * depth_value[1] / width
                    # y_distance = center_y * depth_value[1] /height
                
                    depth_value = depth_value[1] - 200 # Get the depth value in mm
                    depth_value = depth_value/1000 # Convert the depth value to meters
                    print("Depth at center of red mask: {} m".format(depth_value))
                    # print("Distance from left of the image (x-axis): {} meters".format(x_distance))
                    # print("Distance from top of the image (y-axis): {} meters".format(y_distance))
                    depth_msg = Float32()
                    depth_msg.data = depth_value
                    self.distancepublisher_.publish(depth_msg)
                    mask_msg = CvBridge().cv2_to_imgmsg(mask)
                    self.maskpublisher.publish(mask_msg)
                else:
                    print("No red mask found. (Too Small)")

            else:
                print("No red mask found.")



        return
    
    
                

def main(args=None):
    rclpy.init(args=args)
    zed_publisher = ZedPublisher()
    # Check if the stream is opened successfully  
    rclpy.spin(zed_publisher)
    zed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()