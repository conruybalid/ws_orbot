import os
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import ImageProcess
import cv2
from cv_bridge import CvBridge

class ImageClient(Node):
    def __init__(self):
        super().__init__('image_client')
        self.client = self.create_client(ImageProcess, 'rgb_image_processing')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        #self.request_image()
        self.req = ImageProcess.Request()

    def request_image(self, filePath):
        
        cv_bridge = CvBridge()
        image = cv2.imread(filePath)
        self.req.image = cv_bridge.cv2_to_imgmsg(image)
        
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
        

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
    image_path = os.path.expanduser('~/ws_orbot/apple2.JPG')
    response = image_client.request_image(image_path)
    if response is not None:
        image_client.get_logger().info('Number of apples: %d' % response.num_apples)
    else:
        image_client.get_logger().info('response is None')

    image_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()