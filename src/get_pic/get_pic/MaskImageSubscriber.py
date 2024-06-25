import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class MaskedSubscriber(Node):
    def __init__(self):
        super().__init__('Maked_Image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'masked_image_topic',
            self.image_callback,
            10
        )
        self.zed_subscription = self.create_subscription(
            Image,
            'zed_mask_topic',
            self.zed_callback,
            10
        )
        self.subscription

        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info('Received a zed mask')
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow('Masked_Image', cv_image)
        cv2.waitKey(0)

    def zed_callback(self, msg):
        self.get_logger().info('Received a zed mask')
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow('Masked_Zed_Image', cv_image)
        cv2.waitKey(0)

def main(args=None):
    rclpy.init(args=args)
    Masked_subscriber = MaskedSubscriber()
    rclpy.spin(Masked_subscriber)
    Masked_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()