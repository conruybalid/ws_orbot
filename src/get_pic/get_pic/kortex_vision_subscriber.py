import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from screeninfo import get_monitors
import numpy as np

class VideoSubscriber(Node):
    """
    This Node is used to view usefull image outputs for debug from kortex-vision package in a 4x4 grid

    """
    def __init__(self):
        super().__init__('video_subscriber')
        self.arm_rgb_sub = self.create_subscription(
            Image,
            #'camera/color/image_raw',
            'camera/color/image_raw',
            self.arm_image_callback,
            10
        )
        self.rgb_arm_image = None

        self.arm_depth_sub = self.create_subscription(
            Image,
            'camera/depth/image_raw',
            self.depth_callback,
            10
        )
        self.arm_depth_image = None

    
        self.timer = self.create_timer(0.1, self.display_images)

        self.bridge = CvBridge()

        # Create a black image
        self.screen = get_monitors()[0]
        width = int(self.screen.width / 2.5)
        height = int(self.screen.height / 2.5)
        black_image = np.zeros((height, width, 3), dtype=np.uint8)

        # Define the border color and thickness
        border_color = (0, 0, 255)  # Blue in BGR
        border_thickness = 10

        # Add a blue border
        cv2.rectangle(black_image, (0, 0), (width-1, height-1), border_color, thickness=border_thickness)


        self.images = [black_image, black_image, black_image, black_image]

        self.get_logger().info('Video subscriber node has been initialized')

    """
    CALLBACK FUNCTIONS
    """

    def arm_image_callback(self, msg):
        self.get_logger().debug('Received an arm rgb image')
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        cv_image = self.Resize_to_screen(cv_image)
        self.images[0] = cv_image
        # cv2.imshow('Image', cv_image)
        # cv2.waitKey(1)

    def depth_callback(self, msg):
        self.get_logger().debug('Received an arm depth image')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_image = cv2.applyColorMap(cv2.convertScaleAbs(cv_image, alpha=0.02), cv2.COLORMAP_JET)
        cv_image = self.Resize_to_screen(cv_image)
        self.images[1] = cv_image
        # cv2.imshow('Depth', cv_image)
        # cv2.waitKey(1)


    def Resize_to_screen(self, image):
        """
        Resized the image to fit the screen
        """
        width = int(self.screen.width / 2.5)
        height = int(self.screen.height / 2.5)
        return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    
    def display_images(self):
        """
        Called by the timer to display images
        Combines images in self.images into a 4x4 grid and displays them
        """
        try:
            get_monitors()
        except:
            self.get_logger().warn('No monitor detected')
            return
        top_image = np.hstack((self.images[0], self.images[1]))
        bottom_image = np.hstack((self.images[2], self.images[3]))
        combined_image = np.vstack((top_image, bottom_image))
        cv2.imshow('Combined Image', combined_image)
        cv2.waitKey(1)
        
        return
        

def main(args=None):
    rclpy.init(args=args)
    video_subscriber = VideoSubscriber()
    rclpy.spin(video_subscriber)
    video_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()