import rclpy
from rclpy.node import Node

from custom_interfaces.srv import GetLocation
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np

from image_processing.ImageProcess import processImage


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

        publishers:
            Masked_publisher (Publisher): A publisher for the mask created during processing (for debug).

        Other Attributes:
            rgb_image (numpy.ndarray): The RGB image received from the arm camera.

    """


    def __init__(self):
        super().__init__('Arm_Locate_Apple_Service')
        self.locate_apple_service = self.create_service(GetLocation, 'Arm_Locate_Apple_Service', self.process_image_callback)
        self.arm_rgb_sub = self.create_subscription(Image, 'image_topic', self.image_callback, 10)
        self.arm_depth_sub = self.create_subscription(Image, 'camera/depth/image_raw', self.depth_image_callback, 10)
        self.Masked_publisher = self.create_publisher(Image, 'masked_image_topic', 10)

        self.rgb_image = None
        self.depth_image = None

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

    def drawText(self, image, text, x, y):
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

        image = cv2.putText(image, text, (x, y), font, font_scale, color, thickness)
        return image

    
    def process_image_callback(self, request, response):
        """
        When called, uses the ImageProcess function (imported) to apply a red mask to the image and locate the apples.
        If apples are not found, returns a failed_coordinates object with error_status 1.
        """
        failed_coordinates = GetLocation.Response().apple_coordinates
        failed_coordinates.x = 99.0
        failed_coordinates.y = 99.0
        failed_coordinates.z = 99.0

        self.get_logger().info('Incoming Arm Process request')
        
        if self.rgb_image is not None and self.depth_image is not None:
            try:
                largest_contour_index, apple_points, viewing_mask = processImage(self.rgb_image)
                self.get_logger().info('Arm image processed successfully')

                
                mask_msg = CvBridge().cv2_to_imgmsg(viewing_mask)
                self.Masked_publisher.publish(mask_msg)

                if len(apple_points) > 0:
                        pixels = apple_points[largest_contour_index]

                        response.error_status = 0

                        response.apple_coordinates.x, response.apple_coordinates.y = self.pixel_scale(pixels.x, pixels.y)
                        # print(self.depth_image.shape)
                        # print(self.rgb_image.shape)
                        # print(center_y * SCALE_Y + OFFSET_Y, center_x * SCALE_X + OFFSET_X)
                        depth_y = int(pixels.y * SCALE_Y + OFFSET_Y)
                        if depth_y > 270:
                            depth_y = 265
                        depth_x = int(pixels.x * SCALE_X + OFFSET_X)
                        if depth_x > 480:
                            depth_x = 475

                        average_depth = np.mean(self.depth_image[depth_y - 2: depth_y + 2, depth_x - 2: depth_x + 2])

                        response.apple_coordinates.z = average_depth / 1000.0 - 0.15

                else:
                    response.apple_coordinates = failed_coordinates
                    response.error_status = 1

                self.rgb_image = None
                return response

            except Exception as e:
                self.get_logger().error('image processing failed: {}'.format(str(e)))
                response.error_status = 2
                self.rgb_image = None

        else:
            self.get_logger().error('no image available for processing')
            response.error_status = 3

        num_apples = -1
        response.apple_coordinates = failed_coordinates

        return response
    
    
    def pixel_scale(self, x, y):
        """
        Transforms pixel coordinates to relative arm movement coordinates.
        """
        xDesire = 640; 
        xDist = xDesire - x
        xChange = 0.00025*xDist

        yDesire = 420; #was 420
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