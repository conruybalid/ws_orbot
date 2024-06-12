import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

import os

from custom_interfaces.action import pick_apple
from custom_interfaces.srv import ImageProcess
from custom_interfaces.srv import ImageRequest

from sensor_msgs.msg import Image






class PickAppleServer(Node):

    def __init__(self):
        super().__init__('pick_apple_action_server')
        self._action_server = ActionServer(
            self,
            pick_apple,
            'pick_apple',
            self.execute_callback)
        
        self.client = self.create_client(ImageRequest, 'get_image')

        self.client = self.create_client(ImageProcess, 'rgb_image_processing')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Request image
        image_msg = self.request_image()

        # Request image processing
        self.request_process_image(image_msg)
        
        # Publish Arm Movement


        result = pick_apple.Result()
        result.result = 'Success'
        return result




    def request_image(self):
        image_path = os.path.expanduser('~/ws_orbot/apple2.JPG')  # Replace with your image path
    
        cv_bridge = CvBridge()
        image = cv2.imread(image_path)
        
        return cv_bridge.cv2_to_imgmsg(image)
        

    def request_process_image(self, image_msg):
        request = ImageRequest.Request()
        request.image = image_msg
        
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.image_callback)
        rclpy.spin_until_future_complete(self, self.future)



    def image_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('found %.d apples' % response.num_apples)
            for apple in response.apple_coordinates:
                self.get_logger().info('Location: %.2f, %.2f' % (apple.x, apple.y))
            return response
        except Exception as e:
            self.get_logger().error('Image Process Service call failed: %s' % e)
            return None
        
    def publish_arm_movement(self):
        pass




def main(args=None):
    rclpy.init(args=args)

    pick_apple_action_server = PickAppleServer()

    rclpy.spin(pick_apple_action_server)

    pick_apple_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()