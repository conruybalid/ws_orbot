import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_topic',
            self.image_callback,
            10
        )
        self.subscription

        self.depth_subscription = self.create_subscription(
            Image,
            'depth_topic',
            self.depth_callback,
            10
        )
        self.subscription

        self.bridge = CvBridge()

    def image_callback(self, msg):
        #self.get_logger().info('Received an image')
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow('Image', cv_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        #self.get_logger().info('Received a depth image')
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow('Depth', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    video_subscriber = VideoSubscriber()
    rclpy.spin(video_subscriber)
    video_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()