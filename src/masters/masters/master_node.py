import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from custom_interfaces.msg import ArmControl

from masters.ImageProcess import processImage

import cv2
from cv_bridge import CvBridge
import time



class MasterNode(Node):
    def __init__(self):
        super().__init__('pick_apple_action_server')
        self.image_sub = self.create_subscription(Image, 'image_topic', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, 'depth_topic', self.depth_callback, 10)
        self.distance_sub = self.create_subscription(Float32, 'zed_distance_topic', self.distance_callback, 10)
        self.image_msg = None
        self.depth_msg = None
        self.distance_msg = None

        self.move_publisher = self.create_publisher(ArmControl, 'arm_move', 10)
        self.absolute_move_publisher = self.create_publisher(ArmControl, 'absolute_arm_move', 10)
        self.Masked_publisher = self.create_publisher(Image, 'masked_image_topic', 10)


    def image_callback(self, msg):
        self.image_msg = msg

    def depth_callback(self, msg):
        self.depth_msg = msg

    def distance_callback(self, msg):
        self.distance_msg = msg

    def publish_arm_movement(self, position, wrist_angle, gripper_state):
        msg = ArmControl()
        if isinstance(position, list):
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


    def Search(self):
        end_search = False

        move_msg = ArmControl()
        move_msg.position.x = 0.57
        move_msg.position.y = 0.0
        move_msg.position.z = 0.45
        move_msg.gripper_state = 0
        move_msg.wrist_angle = 0.0


        while not end_search:
            rclpy.spin_once(self)
            image = CvBridge().imgmsg_to_cv2(self.image_msg)

            num_apples, apple_coordinates, maskedImage = processImage(image)

            mask_msg = CvBridge().cv2_to_imgmsg(cv2.multiply(maskedImage,255))
            self.Masked_publisher.publish(mask_msg)

            if num_apples <= 0:
                if move_msg.position.y < 0.2:
                    self.absolute_move_publisher.publish(move_msg)
                    self.get_logger().info(f'Published Search Arm Movement: {move_msg.position.y}, {move_msg.position.z}')

                    move_msg.position.y += 0.1
                
                elif move_msg.position.z < 0.7:
                    self.absolute_move_publisher.publish(move_msg)
                    self.get_logger().info(f'Published Search Arm Movement: {move_msg.position.y}, {move_msg.position.z}')

                    move_msg.position.z += 0.1
                    move_msg.position.y = 0.0
                    

                else:
                    self.get_logger().info('No apples found')
                    end_search = True

            else:
                end_search = True
                self.get_logger().info('Apple found')

            time.sleep(1.0)


    def pixel_scale(self, x, y):
        xDesire = 640; 
        xDist = xDesire - x
        xChange = 0.00040*xDist

        yDesire = 420; #was 420
        yDist = yDesire - y
        yChange = 0.00025*yDist

        return xChange, yChange
       


    def center_apple(self):
        rclpy.spin_once(self)

        move_msg = ArmControl()

        image = CvBridge().imgmsg_to_cv2(self.image_msg)

        num_apples, apple_coordinates, maskedImage = processImage(image)
        self.get_logger().info('found %.d apples' % num_apples)

        for apple in apple_coordinates:
            self.get_logger().info('Location: %.2f, %.2f' % (apple.x, apple.y))

        mask_msg = CvBridge().cv2_to_imgmsg(cv2.multiply(maskedImage,255))
        self.Masked_publisher.publish(mask_msg)

        if num_apples == 0:
            self.get_logger().info('No apples found')

            return False


        # scale the coordinates and create message
        move_msg.position.x = 0.0
        move_msg.position.y, move_msg.position.z = self.pixel_scale(apple_coordinates[0].x, apple_coordinates[0].y)
        move_msg.gripper_state = 0
        move_msg.wrist_angle = 0.0

        # Publish Arm Movement
        self.move_publisher.publish(move_msg)
        self.get_logger().info(f'moved {move_msg.position.y} in y, {move_msg.position.z} in z')
        
        return True


    def reach_apple(self):
        self.get_logger().info('found apple at absolute distance %f' % self.distance_msg.data)
        arm_msg = ArmControl()
        arm_msg.position.x = self.distance_msg.data
        arm_msg.position.y = 99.0
        arm_msg.position.z = 99.0
        arm_msg.gripper_state = 1
        arm_msg.wrist_angle = 0.0

        self.absolute_move_publisher.publish(arm_msg)
        time.sleep(2.0)
        return
        

    def grab_apple(self):
        time.sleep(5)
        self.publish_arm_movement([0.0, 0.0, 0.05], 0.0, 2)
        time.sleep(1)
        self.publish_arm_movement([0.0, 0.0, 0.0], 180.0, 0)
        time.sleep(15.0)
        self.publish_arm_movement([-0.2, 0.0, 0.0], 0.0, 0)
        time.sleep(15.0)
        self.publish_arm_movement([0.0, 0.0, 0.0], 0.0, 1)

        return



    def run(self):
        self.get_logger().info('Master Node Routine Started')
        
        while not (self.image_msg is None):
            self.get_logger().info('Waiting for images')
            rclpy.spin_once(self)

        self.Search()

        # Center the apple
        self.get_logger().info('Centering Apple once')
        if not self.center_apple():
            return
        
        time.sleep(5)

        # Do it again for better accuracy
        self.get_logger().info('Centering Apple twice')
        if not self.center_apple():
            return
        
        # Reach for the Apple
        if self.depth_msg is not None:
            self.reach_apple()

        else:
            self.publish_arm_movement([0.2, 0.0, 0.0], 1, 0)
        
        # Grab the Apple
        self.grab_apple()

        return
        



def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    node.run()
    node.get_logger().info('Destroying Master Node')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
