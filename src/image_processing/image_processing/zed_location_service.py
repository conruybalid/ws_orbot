import rclpy
from rclpy.node import Node

from custom_interfaces.srv import GetLocation
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2


import cv2
from cv_bridge import CvBridge

import pyzed.sl as sl

import numpy as np


class ZedPublisher(Node):
    def __init__(self):
        super().__init__('zed_location_service_node')
        self.distancepublisher_ = self.create_service(GetLocation, 'zed_location_service', self.process_image_callback)
        self.zed_rgb_sub = self.create_subscription(Image, 'zed_image_topic', self.zed_image_callback, 10)
        self.zed_image = None
        self.zed_pc_sub = self.create_subscription(PointCloud2, 'zed_pointcloud_topic', self.zed_pointcloud_callback, 10)
        self.zed_pointcloud = None
        self.maskpublisher = self.create_publisher(Image, 'zed_mask_topic', 10)
        self.get_logger().info('Distance publisher node has been initialized')
        


    def zed_image_callback(self, msg):
        self.zed_image = CvBridge().imgmsg_to_cv2(msg)
        self.get_logger().info('ZED Image received')

    def zed_pointcloud_callback(self, msg):
        self.get_logger().info('ZED Point Cloud received')
        self.zed_pointcloud = self.ros_point_cloud2_to_zed_point_cloud(msg)

    def ros_point_cloud2_to_zed_point_cloud(self, ros_point_cloud):
        # Assuming ros_point_cloud is a PointCloud2 message
        # Extract fields and data from the PointCloud2 message
        height, width = ros_point_cloud.height, ros_point_cloud.width
        point_step = ros_point_cloud.point_step
        row_step = ros_point_cloud.row_step
        data = ros_point_cloud.data

        # Convert data to a NumPy array of bytes, then to float32
        data_np = np.frombuffer(bytearray(data), dtype=np.float32)

        # Reshape the array to have the correct dimensions: (height, width, 3)
        points_array = np.reshape(data_np, (height, width, 3))

        return points_array




    def process_image_callback(self,  request, response):
        # Grab an image
        if self.zed_image is not None and self.zed_pointcloud is not None:
            
            # Convert the image to HSV color space
            hsv_image = cv2.cvtColor(self.zed_image, cv2.COLOR_BGR2HSV)

            # Define the lower and upper bounds for the red color in HSV
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])

            # Create a mask for the red color
            mask = cv2.inRange(hsv_image, lower_red, upper_red)

            # Find the contours of the red mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Create Viewable Mask
            viewing_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)        
                viewing_mask = cv2.rectangle(viewing_mask, (x, y), (x + w, y + h), (255, 0, 0), 2)


            # Find the center of the largest contour
            if len(contours) > 0:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])

                    x, y, z = self.zed_pointcloud[center_y, center_x]

                    self.get_logger().info(f'point cloud: {x}, {y}, {z}')

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

                    x, y, w, h = cv2.boundingRect(largest_contour)        
                    viewing_mask = cv2.rectangle(viewing_mask, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    mask_msg = CvBridge().cv2_to_imgmsg(viewing_mask)
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
                response.error_status = 1

            mask_msg = CvBridge().cv2_to_imgmsg(mask)
            self.maskpublisher.publish(mask_msg)

        else:
            self.get_logger().info("Could not access the camera.")
            response.apple_coordinates.x = 0.0
            response.apple_coordinates.y = 0.0
            response.apple_coordinates.z = 0.0
            response.error_status = 3




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
