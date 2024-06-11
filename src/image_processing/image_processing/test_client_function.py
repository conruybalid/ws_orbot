import os
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import ImageRequest
import cv2
from cv_bridge import CvBridge

class ImageClient(Node):
    def __init__(self):
        super().__init__('image_client')
        self.client = self.create_client(ImageRequest, 'rgb_image_processing')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request_image()

    def request_image(self):
        request = ImageRequest.Request()
        image_path = os.path.expanduser('~/apple2.JPG')  # Replace with your image path
    
        cv_bridge = CvBridge()
        image = cv2.imread(image_path)
        request.image = cv_bridge.cv2_to_imgmsg(image)
        
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.callback)
        rclpy.spin_until_future_complete(self, self.future)
        
        

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('found %.d apples' % response.num_apples)
            for apple in response.apple_coordinates:
                self.get_logger().info('Location: %.2f, %.2f' % (apple.x, apple.y))
            return response
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % e)
            return None
        
    

def main(args=None):
    rclpy.init(args=args)
    image_client = ImageClient()
    #rclpy.spin(image_client)
    response = image_client.request_image()
    if response is not None:
        print('Number of apples: %d' % response.num_apples)

    image_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()