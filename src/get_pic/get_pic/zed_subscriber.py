import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import open3d as o3d


import cv2
from cv_bridge import CvBridge

#import open3d as o3d
import numpy as np


class VideoSubscriber(Node):
    """
    This Node is used to test the outputs of the zed_publisher node

    Attributes:
    
            subscriptions:
                subscription (subscription): subscribes to zed_image_topic
                depth_subscription (subscription): subscribes to zed_depth_topic
                pointcloud_subscription (subscription): subscribes to zed_pointcloud_topic
            
    
            Other Attributes:
                bridge (CvBridge): bridge to convert ROS Image messages to OpenCV images
    """

    def __init__(self):
        super().__init__('zed_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'zed_image_topic',
            self.image_callback,
            10
        )
        self.subscription

        self.depth_subscription = self.create_subscription(
            Image,
            'zed_depth_topic',
            self.depth_callback,
            10
        )
        
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            'zed_pointcloud_topic',
            self.pointcloud_callback,
            10
        )
        
        self.bridge = CvBridge()

    """
    CALLBACK FUNCTIONS
    """

    def image_callback(self, msg):
        self.get_logger().info('Received a color image')
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        # cv2.imshow('Color_Image', cv_image)
        # cv2.waitKey(1)

    def depth_callback(self, msg):
        self.get_logger().info('Received a depth image')
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv_image = cv2.multiply(cv_image,255)
        # cv2.imshow('Depth_Image', cv_image)
        # cv2.waitKey(1)

    def pointcloud_callback(self, msg: PointCloud2):
        self.get_logger().info('Received a pointcloud')

        # Convert PointCloud2 to array of points
        np_points = self.ros_point_cloud2_to_zed_point_cloud(msg)

        print(np_points[100][100])


    def ros_point_cloud2_to_zed_point_cloud(self, ros_point_cloud):
        """
        Convert a ROS2 PointCloud2 message to a NumPy array of points (xyz data only)
        """
        # Assuming ros_point_cloud is a PointCloud2 message
        # Extract fields and data from the PointCloud2 message
        height, width = ros_point_cloud.height, ros_point_cloud.width
        point_step = ros_point_cloud.point_step
        row_step = ros_point_cloud.row_step
        data = ros_point_cloud.data

        # Convert data to a NumPy array of bytes, then to float32
        data_np = np.frombuffer(bytearray(data), dtype=np.float32)

        # Reshape the array to have the correct dimensions: (height, width, 3)
        points_array = np.reshape(data_np, (height, width, 3))



        return points_array

def main(args=None):
    rclpy.init(args=args)
    video_subscriber = VideoSubscriber()
    rclpy.spin(video_subscriber)
    video_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()