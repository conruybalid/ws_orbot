import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from get_pic.VideoQueue import VideoStreamHandler

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        self.timer_ = self.create_timer(0.1, self.publish_image)
        self.get_logger().info('Video publisher node has been initialized')

        
        # RTSP stream URL
        rtsp_url = "rtsp://192.168.1.10/color"
        # Create a VideoStream object
        self.video_stream_handler = VideoStreamHandler(rtsp_url)

        self.get_logger().info('rtsp has been initialized')

        self.image_msg = Image()


    def publish_image(self):
        # Create an Image message and publish it
        frame = self.video_stream_handler.get_latest_frame()
        if frame is not None:
            self.image_msg = CvBridge().cv2_to_imgmsg(frame)
            self.publisher_.publish(self.image_msg)
            self.get_logger().info('Image published')
        else:
            self.get_logger().info('Failed to read frame from RTSP stream')
        

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    # Check if the stream is opened successfully
    if video_publisher.video_stream_handler.cap.isOpened():
        rclpy.spin(video_publisher)
    else:    
        video_publisher.get_logger().info("Failed to open RTSP stream, destroying node...")
    
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()