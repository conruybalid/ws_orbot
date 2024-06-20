import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

import os
import time

from custom_interfaces.action import PickApple
from custom_interfaces.srv import ImageProcess
from custom_interfaces.srv import ImageRequest
from custom_interfaces.msg import ArmControl
from typing import Union, List

from pick_apple.ImageProcess import processImage
from pick_apple.GetDepth import GetDepth

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float32






class PickAppleServer(Node):

    def __init__(self):
        super().__init__('pick_apple_action_server')
        self._action_server = ActionServer(
            self,
            PickApple,
            'pick_apple',
            self.execute_callback)
        
        self.image_msg = Image()
        self.depth_msg = Image()

        self.image_sub = self.create_subscription(Image, 'image_topic', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, 'depth_topic', self.depth_callback, 10)
        self.distance_sub = self.create_subscription(Float32, 'zed_distance_topic', self.distance_callback, 10)
        self.image_msg = None
        self.depth_msg = None
        self.distance_msg = None

        self.process_client = self.create_client(ImageProcess, 'rgb_image_processing')
        self.process_request = ImageProcess.Request()

        self.imageNum = 0

        self.move_publisher = self.create_publisher(ArmControl, 'arm_move', 10)
        self.absolute_move_publisher = self.create_publisher(ArmControl, 'absolute_arm_move', 10)
        self.Masked_publisher = self.create_publisher(Image, 'masked_image_topic', 10)

    def image_callback(self, msg):
        self.image_msg = msg

    def depth_callback(self, msg):
        self.depth_msg = msg

    def distance_callback(self, msg):
        self.distance_msg = msg


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = PickApple.Feedback()

        # Wait for image
        

        
        
        while self.image_msg is None:
            feedback_msg.feedback = 'Waiting For Image' 
            goal_handle.publish_feedback(feedback_msg)
            pass 
        feedback_msg.feedback = 'Got rgb Image'  
        goal_handle.publish_feedback(feedback_msg)
        

#---------------------Service Call for image processing---------------------#
        # Request image processing
        # apple_info = self.request_process_image(image_msg)
        # self.get_logger().info('found %.d apples' % apple_info.num_apples)
        # for apple in apple_info.apple_coordinates:
        #     self.get_logger().info('Location: %.2f, %.2f' % (apple.x, apple.y))


#---------------------Fine I'll do it myself---------------------#
        for i in range(0, 1):

            print(i)

            image = CvBridge().imgmsg_to_cv2(self.image_msg)
            num_apples, apple_coordinates, maskedImage = processImage(image, self.imageNum)
            self.get_logger().info('found %.d apples' % num_apples)
            for apple in apple_coordinates:
                self.get_logger().info('Location: %.2f, %.2f' % (apple.x, apple.y))

            feedback_msg.feedback = 'Processed Image'
            goal_handle.publish_feedback(feedback_msg)

            mask_msg = CvBridge().cv2_to_imgmsg(cv2.multiply(maskedImage,255))
            self.Masked_publisher.publish(mask_msg)

            if num_apples == 0:
                result = PickApple.Result()
                result.result = 'No Apples Found'
                goal_handle.abort()
                self.get_logger().info('No apples found')
    
                return result


            # scale the coordinates
            
            yDesire = 640; 
            yDist = yDesire - apple_coordinates[0].x
            yChange = 0.00040*yDist
            
            zDesire = 420; #was 420 
            zDist = zDesire - apple_coordinates[0].y
            zChange = 0.00025*zDist

            apple_coordinates[0].x = 0.0
            apple_coordinates[0].y = yChange
            apple_coordinates[0].z = zChange
    
            # Publish Arm Movement
            self.publish_arm_movement(apple_coordinates[0], 0, 0)
            
            feedback_msg.feedback = f'moved to {apple_coordinates[0].y}, {apple_coordinates[0].z}'  
            goal_handle.publish_feedback(feedback_msg)
            

        if self.distance_msg is not None:

            feedback_msg.feedback = f'found apple at absolute distance {self.distance_msg.data}'
            goal_handle.publish_feedback(feedback_msg)

            arm_msg = ArmControl()
            arm_msg.position.x = self.distance_msg.data
            arm_msg.position.y = 99.0
            arm_msg.position.z = 99.0
            arm_msg.gripper_state = 1
            arm_msg.wrist_angle = 0.0

            self.absolute_move_publisher.publish(arm_msg)
            time.sleep(2.0)
        else:
            self.get_logger().info('No depth found')

        # arm_msg.position.x = 0.0
        # arm_msg.position.y = 0.0
        # arm_msg.position.z = 0.7
        # arm_msg.gripper_state = 1
        time.sleep(5)
        self.publish_arm_movement([0.0, 0.0, 0.05], 0.0, 2)
        time.sleep(1)
        self.publish_arm_movement([0.0, 0.0, 0.0], 180.0, 0)
        time.sleep(15.0)
        self.publish_arm_movement([-0.2, 0.0, 0.0], 0.0, 0)
        time.sleep(5.0)
        self.publish_arm_movement([0.0, 0.0, 0.0], 0.0, 1)

        
        # try:
        #     dist = GetDepth(CvBridge().imgmsg_to_cv2(self.depth_msg))
        # except:
        #     dist = None
        #     self.get_logger().info('depth could not be converted')


        # if not dist == None:
        #     feedback_msg.feedback = f'found apple at distance {dist}'  
        #     goal_handle.publish_feedback(feedback_msg)

        #     depthMove = Point()

        #     depthMove.x = dist
        #     depthMove.y = 0.0
        #     depthMove.z = 0.0

        #    # self.publish_arm_movement(depthMove)

        # else:
        #     result = PickApple.Result()
        #     result.result = 'No Depth Found'
        #     goal_handle.abort()
        #     self.get_logger().info('No depth found')
 
        #     return result



        goal_handle.succeed()
        self.get_logger().info('Success')

        result = PickApple.Result()
        result.result = 'Success'
        return result


    def publish_arm_movement(self, position: Union[Point, List[float]], wrist_angle: float, gripper_state):
        msg = ArmControl()
        if isinstance(position, Point):
            msg.position = position
        elif isinstance(position, list):
            msg.position.x = position[0]
            msg.position.y = position[1]
            msg.position.z = position[2]
        else:
            self.get_logger().error('Invalid position type')
            return
        msg.wrist_angle = float(wrist_angle)
        msg.gripper_state = gripper_state
        self.move_publisher.publish(msg)
        self.get_logger().info('Published Arm Movement')


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
        

        




def main(args=None):
    rclpy.init(args=args)

    pick_apple_action_server = PickAppleServer()

    rclpy.spin(pick_apple_action_server)

    pick_apple_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()