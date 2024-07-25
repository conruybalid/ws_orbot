import rclpy
from rclpy.node import Node

from custom_interfaces.srv import GetLocation
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

from image_processing.ImageProcess import processImage

from image_processing.AI_model import AI_model



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
        self.Masked_publisher = self.create_publisher(Image, 'masked_image_topic', 10)

        self.rgb_image = None

        self.AI = AI_model()

        self.get_logger().info('Arm_Locate_Apple_Service has been Initialized')

    """
    CALLBACK FUNCTIONS
    """

    def image_callback(self, msg):
        self.rgb_image = CvBridge().imgmsg_to_cv2(msg)
        self.get_logger().debug('Image received')

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
        
        if self.rgb_image is not None:
            try:
                image = image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
                pixels = self.AI.GetAppleCoordinates(image, confidence_threshold=0.85)
                self.get_logger().info('Arm image processed successfully')

                viewing_mask = self.rgb_image
                for i, (x1_p, y1_p, x2_p, y2_p) in enumerate(pixels):
                    if i == 0:
                        viewing_mask = cv2.rectangle(viewing_mask, (x1_p, y1_p), (x2_p, y2_p), (0, 0, 255), 5)
                    else:
                        viewing_mask = cv2.rectangle(viewing_mask, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 5)
                
                mask_msg = CvBridge().cv2_to_imgmsg(viewing_mask)
                self.Masked_publisher.publish(mask_msg)

                if len(pixels) > 0:
                        x1_p, y1_p, x2_p, y2_p = pixels[0]

                        center_x = int((x1_p + x2_p) / 2)
                        center_y = int((y1_p + y2_p) / 2)

                        response.error_status = 0

                        response.apple_coordinates.x, response.apple_coordinates.y = self.pixel_scale(center_x, center_y)
                        response.apple_coordinates.z = 0.0

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