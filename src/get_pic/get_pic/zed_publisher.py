import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg
import cv2
from cv_bridge import CvBridge
import numpy as np

import pyzed.sl as sl


class ZedPublisher(Node):
    """
    This class is a ROS2 node that collects and publishes images from the Zed Camera 

    Attributes:

        publishers:
            rgbpublisher_ (publisher): publishes rgb image to zed_image_topic
            depthpublisher_ (publisher): publishes depth image to zed_depth_topic
            pointcloutpublisher_ (publisher): publishes point cloud to zed_pointcloud_topic (just xyz for now)

        Other Attributes:
            self.timer_ (timer): timer to publish images at a fixed rate
            zed (sl.Camera): ZED camera object
            runtime_parameters (sl.RuntimeParameters): ZED runtime parameters
            image (sl.Mat): ZED image object
            depth (sl.Mat): ZED depth object
            point_cloud (sl.Mat): ZED point cloud object (xyz only)


    """

    def __init__(self):
        super().__init__('zed_publisher')
        self.rgbpublisher_ = self.create_publisher(Image, 'zed_image_topic', 10)
        self.depthpublisher_ = self.create_publisher(Image, 'zed_depth_topic', 10)
        self.pointcloutpublisher_ = self.create_publisher(PointCloud2, 'zed_pointcloud_topic', 10)
        self.timer_ = self.create_timer(0.1, self.publish_images)

        self.get_logger().info('zed publisher node has been initialized')

        
        

        init = sl.InitParameters(depth_mode=sl.DEPTH_MODE.ULTRA,
                                 coordinate_units=sl.UNIT.MILLIMETER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)

        self.zed = sl.Camera()
        status = self.zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            


        # Create a ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.sdk_verbose = 1 # Enable verbose logging
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE # Set the depth mode to performance (fastest)
        init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("Error {}".format(err)) # Display the error
            

        self.get_logger().info('ZED camera has been initialized')

        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()
        self.runtime_parameters = sl.RuntimeParameters()



    def publish_images(self):

        """
        Called every time the timer is triggered to publish images
        Retrieves the latest rgb, depth, and point cloud frame from the Zed Camera
        and publishes them
        """
        # Grab an image
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # A new image is available if grab() returns sl.ERROR_CODE.SUCCESS

            # Retrieve the image, depth, and point cloud
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT) # Get the left image
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH) # Retrieve depth matrix. Depth is aligned on the left RGB image
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA) # Retrieve colored point cloud


            if self.image is not None:
                image_msg = self.sl_mat_to_ros_image(self.image)
                self.rgbpublisher_.publish(image_msg)
                self.get_logger().debug('Image published')
            else:
                self.get_logger().warn('self.image is None')

            # depth
            if self.depth is not None:
                depth_msg = self.sl_mat_to_ros_image(self.depth)
                self.depthpublisher_.publish(depth_msg)
                self.get_logger().debug('Depth published')
            else:
                self.get_logger().warn('self.depth is None')

            # point cloud    
            if self.point_cloud is not None:
                # print point cloud data at 100, 100
                #print(self.point_cloud.get_value(100, 100))

                point_cloud_msg = self.zed_point_cloud_to_ros_point_cloud2(self.point_cloud)
                self.pointcloutpublisher_.publish(point_cloud_msg)
                self.get_logger().debug('point cloud published')
            else:
                self.get_logger().warn('self.point_cloud is None')
    
    
    def sl_mat_to_ros_image(self, zed_image: sl.Mat) -> Image:
        """
        Convert a ZED Mat image to a ROS2 Image message
        """

        # Convert sl.Mat to numpy array
        image_np = zed_image.get_data()

        # Use cv_bridge to convert the OpenCV image to a ROS 2 image message
        bridge = CvBridge()
        ros_image_msg = None
        if image_np.dtype == np.uint8:
            # cv2.imshow('color source',image_np)
            # cv2.waitKey(1)
            ros_image_msg = bridge.cv2_to_imgmsg(image_np, "8UC4")
        elif image_np.dtype == np.float32:
            image_np = np.nan_to_num(image_np, nan=0.0, posinf=2450.0)
            # cv2.imshow('depth source',image_np)
            # cv2.waitKey(1)
            ros_image_msg = bridge.cv2_to_imgmsg(image_np, "32FC1")
        else:
            self.get_logger().error('Invalid image data type when converting to ROS Image message')
        return ros_image_msg
         
    
    def zed_point_cloud_to_ros_point_cloud2(self, zed_point_cloud) -> PointCloud2:
        """
        Convert a ZED point cloud to a ROS2 PointCloud2 message
        Only sends xyz data for now
        """

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
        ros_point_cloud.height = zed_point_cloud.get_height()  # Unordered point cloud
        ros_point_cloud.width = zed_point_cloud.get_width()

        return ros_point_cloud
                

def main(args=None):
    
    rclpy.init(args=args)
    zed_publisher = ZedPublisher()
    try:
        # Check if the stream is opened successfully  
        rclpy.spin(zed_publisher)
        zed_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        zed_publisher.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()