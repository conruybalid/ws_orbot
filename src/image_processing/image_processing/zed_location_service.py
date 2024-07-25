import rclpy
from rclpy.node import Node

from custom_interfaces.srv import GetLocation
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

from image_processing.AI_model import AI_model

import cv2
from cv_bridge import CvBridge

import pyzed.sl as sl

import numpy as np



class ZedLocation(Node):
    """
    This class is a ROS2 node that provides a service for locating apples in an image. 
    The node subscribes to an image topic, processes the image, and returns the location of the apples in the image when called.

    Attributes:

        Service Clients:
            locate_apple_service (Service): A service for locating apples in an image.

        Subscribers:
            zed_rgb_sub (Subscriber): A subscriber for the RGB image topic from the zed camera.

        publishers:
            maskpublisher (Publisher): A publisher for the mask created during processing (for debug).

        Other Attributes:
            zed_image (numpy.ndarray): The RGB image received from the zed camera.
            zed_pointcloud (numpy.ndarray): The point cloud received from the zed camera.

    """

    def __init__(self):
        super().__init__('zed_location_service_node')
        self.zed_location_service = self.create_service(GetLocation, 'zed_location_service', self.process_image_callback)
        self.zed_rgb_sub = self.create_subscription(Image, 'zed_image_topic', self.zed_image_callback, 10)
        self.zed_image = None
        self.zed_pc_sub = self.create_subscription(PointCloud2, 'zed_pointcloud_topic', self.zed_pointcloud_callback, 10)
        self.zed_pointcloud = None
        self.maskpublisher = self.create_publisher(Image, 'zed_mask_topic', 10)

        self.AI = AI_model()

        self.get_logger().info('Distance publisher node has been initialized')
        

    """
    CALLBACK FUNCTIONS
    """

    def zed_image_callback(self, msg):
        self.zed_image = CvBridge().imgmsg_to_cv2(msg)
        self.get_logger().debug('ZED Image received')

    def zed_pointcloud_callback(self, msg):
        self.get_logger().debug('ZED Point Cloud received')
        self.zed_pointcloud = self.ros_point_cloud2_to_zed_point_cloud(msg)

    def ros_point_cloud2_to_zed_point_cloud(self, ros_point_cloud):
        """
        Used to convert a ROS PointCloud2 message to a NumPy array of points. (only works for xyz)
        """
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
        """
        when called, applies a red mask to the rgb image and locates the apple in the image.
        If an apple is found, returns the location of the apple as reported in the point cloud.
        Else, returns a failed_coordinates object with error_status 1.
        """
        # Grab an image
        if self.zed_image is not None and self.zed_pointcloud is not None:
            
            pixels = self.AI.GetAppleCoordinates(self.zed_image, confidence_threshold=0.5)
            
            # Find the center of the largest contour
            if len(pixels) > 0:
                    x1_p, y1_p, x2_p, y2_p = pixels[0]

                    center_x = (x1_p, x2_p) / 2
                    center_y = (y1_p, y2_p) / 2

                    x, y, z = self.zed_pointcloud[center_y, center_x]

                    self.get_logger().debug(f'point cloud: {x}, {y}, {z}')

                    # # Convert the center coordinates to meters
                    x_distance = (-x + 509) / 1000
                    y_distance = (-y + 840) / 1000
                    z_distance = (z - 175) / 1000

                    self.get_logger().debug(f'x_distance: {x_distance}, y_distance: {y_distance}, z_distance: {z_distance}')

                    # print("Distance from left of the image (x-axis): {} meters".format(x_distance))
                    # print("Distance from top of the image (y-axis): {} meters".format(y_distance))
                    response.apple_coordinates.x = x_distance
                    response.apple_coordinates.y = y_distance
                    response.apple_coordinates.z = z_distance
        
                    viewing_mask = cv2.rectangle(self.zed_image, (x1_p, y1_p), (x2_p, y2_p), (0, 0, 255), 2)
                    mask_msg = CvBridge().cv2_to_imgmsg(viewing_mask)
                    self.maskpublisher.publish(mask_msg)

                    return response

            else:
                self.get_logger().info("No apples found.")
                response.apple_coordinates.x = 0.0
                response.apple_coordinates.y = 0.0
                response.apple_coordinates.z = 0.0
                response.error_status = 1

            mask_msg = CvBridge().cv2_to_imgmsg(self.zed_image)
            self.maskpublisher.publish(mask_msg)

        else:
            self.get_logger().error("No Image from Camera")
            response.apple_coordinates.x = 0.0
            response.apple_coordinates.y = 0.0
            response.apple_coordinates.z = 0.0
            response.error_status = 3


        return response
    
    
                

def main(args=None):
    rclpy.init(args=args)
    zed_service = ZedLocation()
    # Check if the stream is opened successfully  
    rclpy.spin(zed_service)
    zed_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
