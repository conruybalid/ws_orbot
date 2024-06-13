import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

import os

from custom_interfaces.action import PickApple
from custom_interfaces.srv import ImageProcess
from custom_interfaces.srv import ImageRequest

from pick_apple.ImageProcess import processImage

from sensor_msgs.msg import Image






class PickAppleServer(Node):

    def __init__(self):
        super().__init__('pick_apple_action_server')
        self._action_server = ActionServer(
            self,
            PickApple,
            'pick_apple',
            self.execute_callback)
        
        self.image_msg = Image()

        self.image_sub = self.create_subscription(Image, 'image_topic', self.image_callback, 10)

        self.process_client = self.create_client(ImageProcess, 'rgb_image_processing')
        self.process_request = ImageProcess.Request()

        self.imageNum = 0

    def image_callback(self, msg):
        self.image_msg = msg

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = PickApple.Feedback()

        # Wait for image
        

        
        feedback_msg.feedback = 'Waiting For Image' 
        goal_handle.publish_feedback(feedback_msg)
        while self.image_msg is None:
            pass 
        feedback_msg.feedback = 'Got Image'  
        goal_handle.publish_feedback(feedback_msg)
        

#---------------------Service Call for image processing---------------------#
        # Request image processing
        # apple_info = self.request_process_image(image_msg)
        # self.get_logger().info('found %.d apples' % apple_info.num_apples)
        # for apple in apple_info.apple_coordinates:
        #     self.get_logger().info('Location: %.2f, %.2f' % (apple.x, apple.y))


#---------------------Fine I'll do it myself---------------------#

        image = CvBridge().imgmsg_to_cv2(self.image_msg)
        num_apples, apple_coordinates = processImage(image, self.imageNum)
        self.get_logger().info('found %.d apples' % num_apples)
        for apple in apple_coordinates:
            self.get_logger().info('Location: %.2f, %.2f' % (apple.x, apple.y))

        feedback_msg.feedback = 'Processed Image'
        goal_handle.publish_feedback(feedback_msg)


 
        # Publish Arm Movement
        self.publish_arm_movement(apple_coordinates[0])
        feedback_msg.feedback = f'moved to {apple_coordinates[0].x}, {apple_coordinates[0].y}'  
        goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        self.get_logger().info('Success')

        result = PickApple.Result()
        result.result = 'Success'
        return result




    def test_image(self):
        image_path = os.path.expanduser('~/ws_orbot/apple2.JPG')  # Replace with your image path
        cv_bridge = CvBridge()
        image = cv2.imread(image_path)
        return cv_bridge.cv2_to_imgmsg(image)
        
            
        

    def request_process_image(self, image_msg):
        self.process_request.image = image_msg
        
        self.future = self.process_client.call_async(self.process_request)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        self.future = None
        return response
        



    # def image_service_callback(self, future):
    #     try:
    #         response = future.result()
    #         self.get_logger().info('found %.d apples' % response.num_apples)
    #         for apple in response.apple_coordinates:
    #             self.get_logger().info('Location: %.2f, %.2f' % (apple.x, apple.y))
    #         return response
    #     except Exception as e:
    #         self.get_logger().error('Image Process Service call failed: %s' % e)
    #         return None
        
    def publish_arm_movement(self, msg):
        pass
        # self.move_publisher.publish(msg)
        # self.get_logger().info('Published Arm Movement')

        




def main(args=None):
    rclpy.init(args=args)

    pick_apple_action_server = PickAppleServer()

    rclpy.spin(pick_apple_action_server)

    pick_apple_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()