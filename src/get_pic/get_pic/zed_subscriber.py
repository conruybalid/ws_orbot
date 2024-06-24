import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
#import point_cloud2_methods as pc2

from cv_bridge import CvBridge
import cv2

import open3d as o3d
import numpy as np


class VideoSubscriber(Node):
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
            'zed_depth_topic',
            self.pointcloud_callback,
            10
        )
        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info('Received an image')
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow('Image', cv_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        self.get_logger().info('Received a depth image')
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow('Image', cv_image)
        cv2.waitKey(1)

    def pointcloud_callback(self, msg: PointCloud2):
        self.get_logger().info('Received a pointcloud')

        # # Convert PointCloud2 to array of points
        # points_list = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))

        # # Convert to numpy array for Open3D
        # np_points = np.array(points_list)

        # # Create Open3D point cloud
        # pc_o3d = o3d.geometry.PointCloud()
        # pc_o3d.points = o3d.utility.Vector3dVector(np_points)

        # # Visualize
        # o3d.visualization.draw_geometries([pc_o3d])



def main(args=None):
    rclpy.init(args=args)
    video_subscriber = VideoSubscriber()
    rclpy.spin(video_subscriber)
    video_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()