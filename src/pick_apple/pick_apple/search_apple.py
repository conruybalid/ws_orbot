import rclpy
from rclpy.node import Node
from custom_interfaces.action import PickApple

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from custom_interfaces.msg import ArmControl

from pick_apple.ImageProcess import processImage

import cv2
from cv_bridge import CvBridge



class LookAround(Node):
    def __init__(self):
        super().__init__('video_subscriber')

        self.node = rclpy.create_node('look_around_node')

        self.image_sub = self.create_subscription(Image, 'image_topic', self.image_callback, 10)
        self.image_msg = Image()
        self.end_node = False

        self.move_publisher = self.create_publisher(ArmControl, 'absolute_arm_move', 10)
        self.Masked_publisher = self.create_publisher(Image, 'masked_image_topic', 10)

        self.move_z = 0
        self.move_y = 0

        self.move_msg = ArmControl()
        self.move_msg.position.x = 0.57
        self.move_msg.position.y = 0.0
        self.move_msg.position.z = 0.45
        self.move_msg.gripper_state = 0
        self.move_msg.wrist_angle = 0.0

        

        self.count = 0
        self.get_logger().info('Look Around Node Started')

    def image_callback(self, msg):
        if self.count > 20:
            self.count = 0

            self.image_msg = msg
            image = CvBridge().imgmsg_to_cv2(self.image_msg)

            num_apples, apple_coordinates, maskedImage = processImage(image, 0)

            mask_msg = CvBridge().cv2_to_imgmsg(cv2.multiply(maskedImage,255))
            self.Masked_publisher.publish(mask_msg)


            if num_apples <= 0:
                if self.move_msg.position.y < 0.2:
                    self.move_publisher.publish(self.move_msg)
                    self.get_logger().info('Published Search Arm Movement')

                    self.move_msg.position.y += 0.1
                
                elif self.move_msg.position.z < 0.7:
                    self.move_publisher.publish(self.move_msg)
                    self.get_logger().info('Published Search Arm Movement')

                    self.move_msg.position.z += 0.1
                    self.move_msg.position.y = 0.0
                    

                else:
                    self.get_logger().info('No apples found')
                    self.end_node = True

            else:
                self.end_node = True
                self.get_logger().info('Apple found')

        else:
            self.count += 1



def main(args=None):
    rclpy.init(args=args)
    look_around = LookAround()
    while rclpy.ok():
        rclpy.spin_once(look_around, timeout_sec=5)
        if look_around.end_node:
            look_around.get_logger().info('Destroying Node')
            break
    look_around.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()