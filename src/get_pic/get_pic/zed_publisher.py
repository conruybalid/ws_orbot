import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg
import cv2
from cv_bridge import CvBridge
from get_pic.VideoQueue import VideoStreamHandler

import pyzed.sl as sl


class ZedPublisher(Node):
    def __init__(self):
        super().__init__('zed_publisher')
        self.rgbpublisher_ = self.create_publisher(Image, 'zed_rgb_topic', 10)
        self.depthpublisher_ = self.create_publisher(Image, 'zed_depth_topic', 10)
        self.pointcloutpublisher_ = self.create_publisher(PointCloud2, 'zed_pointcloud_topic', 10)
        self.timer_ = self.create_timer(0.1, self.publish_images)
        self.get_logger().info('Video publisher node has been initialized')

        
        init = sl.InitParameters(depth_mode=sl.DEPTH_MODE.ULTRA,
                                    coordinate_units=sl.UNIT.MILLIMETER,
                                    coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)

        self.zed = sl.Camera()
        status = self.zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            self.get_logger().info("Failed to open ZED camera, destroying node...")
            self.destroy_node()

        init_params = sl.InitParameters()
        init_params.sdk_verbose = True # Enable verbose logging
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE # Set the depth mode to performance (fastest)
        init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print("Error {}, exit program".format(err)) # Display the error
            self.get_logger().info("Failed to open ZED camera, destroying node...")
            self.destroy_node()


        self.get_logger().info('ZED camera has been initialized')

        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()


        



    def publish_images(self):
        # Retrieve the image, depth, and point cloud
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT) # Get the left image
        self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH) # Retrieve depth matrix. Depth is aligned on the left RGB image
        self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA) # Retrieve colored point cloud


        if self.image is not None:
            image_msg = self.s1_mat_to_ros_image(self.image)
            self.rgbpublisher_.publish(image_msg)
            self.get_logger().info('Image published')
        else:
            self.get_logger().info('self.image is None')

        # depth
        if self.depth is not None:
            depth_msg = self.s1_mat_to_ros_image(self.depth)
            self.depthpublisher_.publish(depth_msg)
            self.get_logger().info('Depth published')
        else:
            self.get_logger().info('self.depth is None')

        # point cloud    
        if self.point_cloud is not None:
            depth_msg = self.zed_point_cloud_to_ros_point_cloud2(self.point_cloud)
            self.depthpublisher_.publish(depth_msg)
            self.get_logger().info('point cloud published')
        else:
            self.get_logger().info('self.point_cloud is None')
    
    
    def sl_mat_to_ros_image(zed_image):
        # Convert sl.Mat to numpy array
        image_np = zed_image.get_data()

        # Convert the numpy array to an OpenCV image
        # Note: Depending on the retrieval mode, you might need to convert the color space from BGR to RGB
        # opencv_image = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

        # Use cv_bridge to convert the OpenCV image to a ROS 2 image message
        bridge = CvBridge()
        ros_image_msg = bridge.cv2_to_imgmsg(image_np, "bgr8")  # Use "rgb8" if you converted the color space

        return ros_image_msg
    
    def zed_point_cloud_to_ros_point_cloud2(zed_point_cloud):
        # Initialize the PointCloud2 message
        ros_point_cloud = PointCloud2()
      #  ros_point_cloud.header.stamp = self.get_clock().now().to_msg()
        ros_point_cloud.header.frame_id = "zed_camera_center"  # Adjust according to your TF frames

        # Define the point fields
        fields = [
            sensor_msgs.msg.PointField(name="x", offset=0, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1),
            sensor_msgs.msg.PointField(name="y", offset=4, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1),
            sensor_msgs.msg.PointField(name="z", offset=8, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1),
            # Add color or other fields if needed
        ]
        ros_point_cloud.fields = fields
        ros_point_cloud.is_bigendian = False
        ros_point_cloud.point_step = 12  # Size of a point in bytes (3 floats: x, y, z)
        ros_point_cloud.is_dense = True  # Assuming no invalid points

        # Convert ZED point cloud to numpy array
        pc_data = zed_point_cloud.get_data()
        # Flatten the array and keep only x, y, z values
        flat_pc = pc_data.reshape(-1, pc_data.shape[-1])[:, :3]
        # Convert to float32
        flat_pc = np.asarray(flat_pc, dtype=np.float32)
        # Flatten the array to a list of bytes
        ros_point_cloud.data = flat_pc.tobytes()

        # Set the row step size
        ros_point_cloud.row_step = ros_point_cloud.point_step * flat_pc.shape[0]

        # Set the height and width of the point cloud
        ros_point_cloud.height = 1  # Unordered point cloud
        ros_point_cloud.width = len(flat_pc)

        return ros_point_cloud
                

def main(args=None):
    rclpy.init(args=args)
    zed_publisher = ZedPublisher()
    # Check if the stream is opened successfully  
    rclpy.spin(zed_publisher)
    zed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()