import rclpy
from rclpy.node import Node

from custom_interfaces.srv import GetLocation
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np
from typing import Tuple
import math

from image_processing.ImageProcess import processImage

from image_processing.AI_model import AI_model

SCALE_X = 480/1280
OFFSET_X = 20
SCALE_Y = 270/720
OFFSET_Y = 50

class ImageProcessingService(Node):
    """
    This class is a ROS2 node that provides a service for locating apples in an image. 
    The node subscribes to an image topic, processes the image, and returns the location of the apples in the image when called.

    Attributes:

        Service Clients:
            locate_apple_service (Service): A service for locating apples in an image.

        Subscribers:
            arm_rgb_sub (Subscriber): A subscriber for the RGB image topic from the arm.
            arm_depth_sub (Subscriber): A subscriber for the depth image topic from the arm.

        publishers:
            Masked_publisher (Publisher): A publisher for the mask created during processing (for debug).

        Other Attributes:
            rgb_image (numpy.ndarray): The RGB image received from the arm camera.
            depth_image (numpy.ndarray): The depth image received from the arm camera.
            AI (AI_model): An instance of the AI_model class for processing images.

    """


    def __init__(self):
        super().__init__('Arm_Locate_Apple_Service')
        self.locate_apple_service = self.create_service(GetLocation, 'Arm_Locate_Apple_Service', self.process_image_callback)
        self.arm_rgb_sub = self.create_subscription(Image, 'image_topic', self.image_callback, 10)
        self.arm_depth_sub = self.create_subscription(Image, 'camera/depth/image_raw', self.depth_image_callback, 10)
        self.Masked_publisher = self.create_publisher(Image, 'masked_image_topic', 10)

        self.rgb_image = None
        self.depth_image = None

        self.AI = AI_model()

        self.get_logger().info('Arm_Locate_Apple_Service has been Initialized')

    """
    CALLBACK FUNCTIONS
    """

    def image_callback(self, msg):
        self.rgb_image = CvBridge().imgmsg_to_cv2(msg)
        self.get_logger().debug('Image received')

    def depth_image_callback(self, msg):
        self.depth_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.get_logger().debug('Depth image received')
    
    def process_image_callback(self, request, response):
        """
        When called, uses the AI model to locate the apples.
        Draws blue boxes around all the apples in the image.
        Sorted by confidence, the first apple is boxed in red.
        Returns the coordinates of the first apple (highest confidence).
        If apples are not found, returns a failed_coordinates object with error_status 1.
        """
        failed_coordinates = GetLocation.Response().apple_coordinates
        failed_coordinates.x = 99.0
        failed_coordinates.y = 99.0
        failed_coordinates.z = 99.0

        self.get_logger().info('Incoming Arm Process request')
        
        # Check if both images are available to be processed
        if self.rgb_image is not None and self.depth_image is not None:
            try:
                image = image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB) # Swap Red and Blue for input into the AI model
                pixels = self.AI.GetAppleCoordinates(image, confidence_threshold=0.70) # Get the apple coordinates from the AI model (only ones above the confidence threshold)
                self.get_logger().info('Arm image processed successfully')

                # Create a "mask" image with the boxes drawn around the apples
                viewing_mask = self.rgb_image
                for i, (x1_p, y1_p, x2_p, y2_p, conf) in enumerate(pixels):
                    # if i == 0:
                    #     viewing_mask = cv2.rectangle(viewing_mask, (x1_p, y1_p), (x2_p, y2_p), (0, 0, 255), 5) # Draw the first apple in red (highest confidence)
                    # else:
                    viewing_mask = cv2.rectangle(viewing_mask, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 5) # Draw the other apples in blue

                    viewing_mask = self.drawText(viewing_mask, f"{conf:.3f}", x1_p, y1_p - 3) # Draw the confidence of the apple

                # Publish the "mask" image
                mask_msg = CvBridge().cv2_to_imgmsg(viewing_mask)
                self.Masked_publisher.publish(mask_msg)

                # If apples were found, return the coordinates of the first apple (highest confidence)
                if len(pixels) > 0:
                        bestx = 1000
                        besty = 1000
                        bestyP1 = 0
                        bestxP1 = 0
                        bestyP2 = 0
                        bestxP2 = 0
                        bestCenterx = 0
                        bestCentery = 0
                        for pixel in pixels:
                            x1_p, y1_p, x2_p, y2_p, *_ = pixel

                            center_x = int((x1_p + x2_p) / 2)
                            center_y = int((y1_p + y2_p) / 2)

                            response.error_status = 0

                            dist_x, dist_y = self.pixel_scale(center_x,center_y)
                            if (math.sqrt(center_x**2 + center_y**2) < math.sqrt(bestx**2 + besty**2)):
                                bestx = dist_x
                                besty = dist_y
                                bestxP1 = x1_p
                                bestyP1 = y1_p
                                bestxP2 = x2_p
                                bestyP2 = y2_p
                                bestCenterx = center_x
                                bestCentery = center_y

                        
                        viewing_mask = cv2.rectangle(viewing_mask, (bestxP1, bestyP1), (bestxP2, bestyP2), (0, 0, 255), 5) # Draw the best apple in red

                        # convert pixel coordinates to relative arm movement coordinates
                        response.apple_coordinates.x, response.apple_coordinates.y = float(bestx), float(besty)

                        # Find the matching pixel coordinate in the depth image
                        depth_y = int(bestCentery * SCALE_Y + OFFSET_Y)
                        if depth_y > 270: # Make sure the depth_y is within the bounds of the depth image
                            depth_y = 265
                        depth_x = int(bestCenterx * SCALE_X + OFFSET_X)
                        if depth_x > 480: # Make sure the depth_x is within the bounds of the depth image
                            depth_x = 475

                        # Get the average depth of the surrounding pixels in the depth image
                        average_depth = np.mean(self.depth_image[depth_y - 2: depth_y + 2, depth_x - 2: depth_x + 2])

                        response.apple_coordinates.z = average_depth / 1000.0 - 0.12 # Convert the depth from mm to m and subtract distance from camera to gripper

                else: # If no apples were found, return the failed_coordinates object and error_status 1
                    response.apple_coordinates = failed_coordinates
                    response.error_status = 1

                self.rgb_image = None
                return response

            except Exception as e: # If an error occurs in processing the image, return the failed_coordinates object and error_status 2
                self.get_logger().error('image processing failed: {}'.format(str(e)))
                response.error_status = 2
                self.rgb_image = None

        else: # If no images are available, return the failed_coordinates object and error_status 3
            self.get_logger().error('no image available for processing')
            response.error_status = 3

        response.apple_coordinates = failed_coordinates

        return response


    """
    HELPER FUNCTIONS
    """
    
    def drawText(self, image: np.ndarray, text: str, x: int, y: int) -> np.ndarray:
        """
        Draws text on an image at a given location.
        """
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 2.5
        color = (255, 255, 255)
        thickness = 4

        # Get the text size
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        
        # Background rectangle coordinates
        rect_start = (x, y - text_height - baseline)
        rect_end = (x + text_width, y - 3)
        
        # Draw the background rectangle
        image = cv2.rectangle(image, rect_start, rect_end, (0, 0, 255), cv2.FILLED)

        # Draw the text
        image = cv2.putText(image, text, (x, y), font, font_scale, color, thickness)
        
        return image
    
    
    def pixel_scale(self, x: int, y: int) -> Tuple[float, float]:
        """
        Transforms pixel coordinates to relative arm movement coordinates.
        """
        xDesire = 640 
        xDist = xDesire - x
        xChange = 0.00025*xDist

        yDesire = 420
        yDist = yDesire - y
        yChange = 0.00025*yDist

        return xChange, yChange
    
            

def main(args=None):
    rclpy.init(args=args)
    image_processing_service = ImageProcessingService()
    rclpy.spin(image_processing_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
