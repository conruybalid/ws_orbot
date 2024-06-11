import rclpy
from rclpy.node import Node

from custom_interfaces.srv import ImageRequest

from cv_bridge import CvBridge
#from ImageProcess import processImage
from image_processing.ImageProcess import processImage

class ImageProcessingService(Node):
    def __init__(self):
        super().__init__('red_mask')
        self.service = self.create_service(ImageRequest, 'rgb_image_processing', self.process_image_callback)
        self.imageNum = 0

    def process_image_callback(self, request, response):
        # Process the image here
        image_msg = request.image
        self.get_logger().info('Incoming image request')
        # Image processing logic goes here
        image = CvBridge().imgmsg_to_cv2(image_msg)

        # try:
        response.num_apples, response.apple_coordinates = processImage(image, self.imageNum)
        self.get_logger().info('image processed successfully')
        self.imageNum += 1
        return response

        # except:
        #     self.get_logger().info('image processing failed')
        #     response.num_apples = -1
        #     response.region_coordinates = [-1.0, -1.0]
        #     return response
            

def main(args=None):
    rclpy.init(args=args)
    image_processing_service = ImageProcessingService()
    rclpy.spin(image_processing_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()