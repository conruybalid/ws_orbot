import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from screeninfo import get_monitors
import numpy as np

class VideoSubscriber(Node):
    """
    This Node is used to view usefull image outputs for debug in a 4x4 grid
    If images are not available, a black image with a red border will be displayed

    Attributes:

        subscriptions:
            arm_rgb_sub (subscription): subscribes to image_topic
            arm_depth_sub (subscription): subscribes to depth_topic
            arm_mask_sub (subscription): subscribes to masked_image_topic
            zed_image_subscription (subscription): subscribes to zed_image_topic
            zed_mask_subscription (subscription): subscribes to zed_mask_topic

        other attributes:
            timer (timer): timer to display images at a fixed rate
            bridge (CvBridge): bridge to convert ROS Image messages to OpenCV images
            rgb_arm_image (np.array): latest image from arm camera
            arm_depth_image (np.array): latest depth image from arm camera
            arm_mask_image (np.array): latest masked image from arm camera
            zed_image (np.array): latest image from zed camera
            zed_mask_image (np.array): latest masked image from zed camera
            images (list): list of images to display

    """
    def __init__(self):
        super().__init__('video_subscriber')
        self.arm_rgb_sub = self.create_subscription(
            Image,
            'image_topic',
            self.arm_image_callback,
            10
        )
        self.rgb_arm_image = None

        self.arm_depth_sub = self.create_subscription(
            Image,
            'depth_topic',
            self.depth_callback,
            10
        )
        self.arm_depth_image = None

        self.arm_mask_sub = self.create_subscription(
            Image,
            'masked_image_topic',
            self.arm_mask_callback,
            10
        )
        self.arm_mask_image = None

        self.zed_image_subscription = self.create_subscription(
            Image,
            'zed_image_topic',
            self.zed_image_callback,
            10
        )
        self.zed_image = None

        self.zed_mask_subscription = self.create_subscription(
            Image,
            'zed_mask_topic',
            self.zed_mask_callback,
            10
        )
        self.zed_mask_image = None

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

    """
    CALLBACK FUNCTIONS
    """

    def arm_image_callback(self, msg):
        #self.get_logger().info('Received an image')
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv_image = self.Resize_to_screen(cv_image)
        self.images[0] = cv_image
        # cv2.imshow('Image', cv_image)
        # cv2.waitKey(1)

    def depth_callback(self, msg):
        #self.get_logger().info('Received a depth image')
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv_image = self.Resize_to_screen(cv_image)
        # cv2.imshow('Depth', cv_image)
        # cv2.waitKey(1)

    def arm_mask_callback(self, msg):
        #self.get_logger().info('Received an arm camera mask')
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv_image = self.Resize_to_screen(cv_image)
        #self.get_logger().info('Recieved Image Mask Shape: {}'.format(cv_image.shape))
        if len(cv_image.shape) < 3:
            self.images[1] = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
        else:
            self.images[1] = cv_image 
        # cv2.imshow('Masked_Image', cv_image)
        # cv2.waitKey(1)

    def zed_image_callback(self, msg):
        #self.get_logger().info('Received a zed image')
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv_image = self.Resize_to_screen(cv_image)
        if cv_image.shape[2] > 3:
            self.images[2] = cv_image[:, :, :3]
        else:
            self.images[2] = cv_image
        # cv2.imshow('Zed_Image', cv_image)
        # cv2.waitKey(1)

    def zed_mask_callback(self, msg):
        #self.get_logger().info('Received a zed mask')
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv_image = self.Resize_to_screen(cv_image)
        if len(cv_image.shape) < 3:
            self.images[3] = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
        else:
            self.images[3] = cv_image 
        # cv2.imshow('Masked_Zed_Image', cv_image)
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
            self.get_logger().info('No monitor detected')
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