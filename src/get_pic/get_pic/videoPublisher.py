import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from get_pic.VideoQueue import VideoStreamHandler

import time

class VideoPublisher(Node):
    """
    This class is a ROS2 node that collects and publishes images from the Arm Camera 

    Attributes:

        publishers:
            rgbpublisher_ (publisher): publishes rgb image to image_topic
            depthpublisher_ (publisher): publishes depth image to depth_topic (not working)

        Other Attributes:
            self.timer_ (timer): timer to publish images at a fixed rate
            rtsp_url (str): rtsp stream url
            depth_url (str): rtsp stream url for depth
            video_stream_handler (VideoStreamHandler): object to handle video stream
            depth_stream_handler (VideoStreamHandler): object to handle video stream for depth

            image_msg (Image): Image message to publish
            depth_msg (Image): Image message to publish for depth

    """

    def __init__(self):
        super().__init__('video_publisher')
        self.rgbpublisher_ = self.create_publisher(Image, 'image_topic', 10)

        self.timer_ = self.create_timer(0.1, self.publish_images)
        
        # RTSP stream URL
        rtsp_url = "rtsp://192.168.1.10/color"
        depth_url = 'rtsp://192.168.1.10/depth'
        # Create a VideoStream object
        self.video_stream_handler = VideoStreamHandler(rtsp_url)
        i = 0
        while not self.video_stream_handler.cap.isOpened():
            self.get_logger().error(f'Failed to initialize video stream handler: attempt {i}')
            self.video_stream_handler = None
            i += 1
            time.sleep(2)
            self.video_stream_handler = VideoStreamHandler(rtsp_url)
            

        self.get_logger().info('rtsp has been initialized')

        self.image_msg = Image()
        self.depth_msg = Image()

        self.get_logger().info('Video publisher node has been initialized')


    def publish_images(self):
        """
        Called every time the timer is triggered to publish images
        Retrieves the latest frame from the video stream handlers and publishes it
        Depth image is not working
        """
        # Create an Image message and publish it
        frame = self.video_stream_handler.get_latest_frame()
        if frame is not None:
            self.image_msg = CvBridge().cv2_to_imgmsg(frame)
            self.rgbpublisher_.publish(self.image_msg)
            #self.get_logger().info('Image published')
        else:
            self.get_logger().warn('Failed to read frame from RTSP rgb stream')

         

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    # Check if the stream is opened successfully
    if video_publisher.video_stream_handler.cap.isOpened():
        try:
            rclpy.spin(video_publisher)
        except KeyboardInterrupt:
            video_publisher.video_stream_handler.stop()
    else:    
        video_publisher.get_logger().fatal("Failed to open RTSP stream, destroying node...")
    
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()