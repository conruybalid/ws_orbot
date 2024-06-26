import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from get_pic.VideoQueue import VideoStreamHandler

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.rgbpublisher_ = self.create_publisher(Image, 'image_topic', 10)
        self.depthpublisher_ = self.create_publisher(Image, 'depth_topic', 10)
        self.timer_ = self.create_timer(0.1, self.publish_images)
        self.get_logger().info('Video publisher node has been initialized')

        
        # RTSP stream URL
        rtsp_url = "rtsp://192.168.1.10/color"
        depth_url = 'rtsp://192.168.1.10/depth'
        # Create a VideoStream object
        self.video_stream_handler = VideoStreamHandler(rtsp_url)
        self.depth_stream_handler = VideoStreamHandler(depth_url, gstreamer=True)

        self.get_logger().info('rtsp has been initialized')

        self.image_msg = Image()
        self.depth_msg = Image()


    def publish_images(self):
        # Create an Image message and publish it
        frame = self.video_stream_handler.get_latest_frame()
        if frame is not None:
            self.image_msg = CvBridge().cv2_to_imgmsg(frame)
            self.rgbpublisher_.publish(self.image_msg)
            #self.get_logger().info('Image published')
        else:
            self.get_logger().info('Failed to read frame from RTSP rgb stream')

        # Create an Image message and publish it
        # frame = self.depth_stream_handler.get_latest_frame()
        # if frame is not None:
        #     self.depth_msg = CvBridge().cv2_to_imgmsg(frame)
        #     self.depthpublisher_.publish(self.depth_msg)
        #     self.get_logger().info('Depth published')
        # else:
        #     self.get_logger().info('Failed to read frame from RTSP depth stream')
        

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    # Check if the stream is opened successfully
    if video_publisher.video_stream_handler.cap.isOpened():
        try:
            rclpy.spin(video_publisher)
        except KeyboardInterrupt:
            video_publisher.video_stream_handler.stop()
            video_publisher.depth_stream_handler.stop()
    else:    
        video_publisher.get_logger().info("Failed to open RTSP stream, destroying node...")
    
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()