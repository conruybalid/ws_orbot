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

import math


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
            AI (AI_model): An instance of the AI_model class for processing images.

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

    def process_image_callback(self,  request, response):

        """
        when called, applies a red mask to the rgb image and locates the apple in the image.
        If an apple is found, returns the location of the apple as reported in the point cloud.
        Else, returns a failed_coordinates object with error_status 1.
        """
        # If the image and point cloud are available, process the image
        if self.zed_image is not None and self.zed_pointcloud is not None:
            
            image = cv2.cvtColor(self.zed_image, cv2.COLOR_BGR2RGB) # Swap Red and Blue for input into the AI model
            pixels = self.AI.GetAppleCoordinates(image, confidence_threshold=0.6) # Get the apple coordinates from the AI model (only ones above the confidence threshold)

            # Create a "mask" image with the blue boxes drawn around the apples along with their confidence
            viewing_mask = self.zed_image
            for x1_p, y1_p, x2_p, y2_p, conf in pixels:
                viewing_mask = cv2.rectangle(viewing_mask, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 5)
                viewing_mask = self.drawText(viewing_mask, f"{conf:.3f}", x1_p, y1_p - 3)
            
            # If apples are found
            if len(pixels) > 0:
                # Go through each apple (ordered in descending confidence)
                # Get the coordinates of the apple from the point cloud
                # If the apple is within reach, box in red and return that one
                # Else, box in green and move onto next apple
                foundValidApple = False
                for pixel in pixels:
                    x1_p, y1_p, x2_p, y2_p, *_ = pixel

                    # Get the center of the apple in the image
                    center_x = int((x1_p + x2_p) / 2)
                    center_y = int((y1_p + y2_p) / 2)

                    # Get the coordinates of the apple from the point cloud
                    x, y, z = np.mean(self.zed_pointcloud[center_y - 2: center_y + 2, center_x - 2: center_x + 2, :], axis=(0, 1))

                    self.get_logger().debug(f'point cloud: {x}, {y}, {z}')

                    # Check if the point cloud values are invalid, box in green and move onto next apple
                    if math.isnan(x) or math.isnan(y) or math.isnan(z) or math.isinf(x) or math.isinf(y) or math.isinf(z):
                        self.get_logger().info(f'Invalid point cloud values: {x}, {y}, {z}')
                        viewing_mask = cv2.rectangle(self.zed_image, (x1_p, y1_p), (x2_p, y2_p), (0, 255, 0), 5)
                        continue
                    

                    # Convert the coordinates to meters in reference to arm base
                    x_distance = (-x - 361) / 1000
                    y_distance = (-y + 785) / 1000
                    z_distance = (z - 175) / 1000

                    self.get_logger().info(f'x_distance: {x_distance}, y_distance: {y_distance}, z_distance: {z_distance}')

                    # Check if the apple is within reach. If not box in green and move onto next apple
                    euclidean_distance = math.sqrt(x_distance**2 + y_distance**2 + z_distance**2)
                    if (euclidean_distance > 1.0 or euclidean_distance < 0.65):
                        viewing_mask = cv2.rectangle(self.zed_image, (x1_p, y1_p), (x2_p, y2_p), (0, 255, 0), 5)
                        self.get_logger().info(f'Euclidean distance: {euclidean_distance} unattainable')
                        
                    elif not foundValidApple:    
                        # If the apple is within reach, box in red and return the coordinates
                        foundValidApple = True

                        # Box in red and publish "mask"
                        viewing_mask = cv2.rectangle(self.zed_image, (x1_p, y1_p), (x2_p, y2_p), (0, 0, 255), 5)
                        mask_msg = CvBridge().cv2_to_imgmsg(viewing_mask)
                        self.maskpublisher.publish(mask_msg)
                
                        # Set response values
                        response.apple_coordinates.x = x_distance
                        response.apple_coordinates.y = y_distance
                        response.apple_coordinates.z = z_distance


                if foundValidApple:
                    # Reset the image and point cloud
                    self.zed_image = None
                    self.zed_pointcloud = None
                    return response
                
                else:
                    # If no valid apples are found, return all zeros and error_status 1
                    self.get_logger().info("No valid apples found.")
                    response.apple_coordinates.x = 0.0
                    response.apple_coordinates.y = 0.0
                    response.apple_coordinates.z = 0.0
                    response.error_status = 1

            else: # If no apples are found, return all zeros and error_status 1
                self.get_logger().info("No apples found.")
                response.apple_coordinates.x = 0.0
                response.apple_coordinates.y = 0.0
                response.apple_coordinates.z = 0.0
                response.error_status = 1

            # Publish the "mask" image 
            mask_msg = CvBridge().cv2_to_imgmsg(viewing_mask)
            self.maskpublisher.publish(mask_msg)

        else: # If no image or point cloud is available, return all zeros and error_status 3
            self.get_logger().error("No Image from Camera")
            response.apple_coordinates.x = 0.0
            response.apple_coordinates.y = 0.0
            response.apple_coordinates.z = 0.0
            response.error_status = 3

        # Reset the image and point cloud
        self.zed_image = None
        self.zed_pointcloud = None

        return response
    

    """
    HELPER FUNCTIONS
    """

    def ros_point_cloud2_to_zed_point_cloud(self, ros_point_cloud: PointCloud2) -> np.ndarray:
        """
        Used to convert a ROS PointCloud2 message to a NumPy array of points. (only works for xyz)
        """
        # Assuming ros_point_cloud is a PointCloud2 message
        # Extract fields and data from the PointCloud2 message
        height, width = ros_point_cloud.height, ros_point_cloud.width # height and width of the image
        point_step = ros_point_cloud.point_step
        row_step = ros_point_cloud.row_step
        data = ros_point_cloud.data # Actual point cloud data

        # Convert data to a NumPy array of bytes, then to float32
        data_np = np.frombuffer(bytearray(data), dtype=np.float32) # This is a 1D array of all the data in the point cloud

        # Reshape the array to have the correct dimensions: (height, width, 3)
        points_array = np.reshape(data_np, (height, width, 3))

        return points_array


    def drawText(self, image: np.ndarray, text: str, x: int, y: int) -> np.ndarray:
        """
        Draws text on an image at a given location.
        """
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.5
        color = (255, 255, 255)
        thickness = 3
        # Get the text size
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        
        # Background rectangle coordinates
        rect_start = (x, y - text_height - baseline)
        rect_end = (x + text_width, y + 3)
        
        # Draw the background rectangle
        image = cv2.rectangle(image, rect_start, rect_end, (0, 0, 255), cv2.FILLED)

        # Draw the text
        image = cv2.putText(image, text, (x, y), font, font_scale, color, thickness)
        
        return image
    
                

def main(args=None):
    rclpy.init(args=args)
    zed_service = ZedLocation()
    rclpy.spin(zed_service)
    zed_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
