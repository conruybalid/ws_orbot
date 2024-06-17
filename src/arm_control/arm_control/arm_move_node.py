import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

import arm_control.utilities as utilities

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

from arm_control.mytry import example_move_to_home_position, example_trajectory


class ArmMoveSubscriber(Node):
    def __init__(self):
        super().__init__('arm_move_subscriber')
        self.subscription = self.create_subscription(
            # Specify the message type and topic name
            msg_type=Point,
            topic='arm_move',
            callback=self.callback,
            qos_profile=10
        )
        self.absoluteSubscription = self.create_subscription(
            # Specify the message type and topic name
            msg_type=Point,
            topic='absolute_arm_move',
            callback=self.absolute_callback,
            qos_profile=10
        )
        
        #self.subscription  # prevent unused variable warning

        self.args = utilities.parseConnectionArguments()

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(self.args) as router:

            # Create required services
            base = BaseClient(router)
            
            # Example core
            success = True

            success &= example_move_to_home_position(base)


    def callback(self, msg):
        # Process the received message here
        pass
        
        with utilities.DeviceConnection.createTcpConnection(self.args) as router:
            
            # Create required services
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)     
            
            feedback = base_cyclic.RefreshFeedback()

            # Accessing the X, Y, Z position of the tool
            arm_x = feedback.base.tool_pose_x
            arm_y = feedback.base.tool_pose_y
            arm_z = feedback.base.tool_pose_z

            # Get the coordinates from the user
            coordinates = [msg.x + arm_x, msg.y + arm_y, msg.z + arm_z]

            success = True   

            # Update the waypointsDefinition with the new coordinates
            waypointsDefinition = (coordinates[0], coordinates[1], coordinates[2], 0.0, 90.0, 0.0, 90.0)

            success &= example_trajectory(base, base_cyclic, waypointsDefinition)

            self.get_logger().info('Moved to position: %f, %f, %f' % (feedback.base.tool_pose_x, feedback.base.tool_pose_y, feedback.base.tool_pose_z))

    def absolute_callback(self, msg):
        with utilities.DeviceConnection.createTcpConnection(self.args) as router:
            
            # Create required services
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)     
            
            feedback = base_cyclic.RefreshFeedback()

            # Get the coordinates from the user
            coordinates = [msg.x, msg.y, msg.z]

            success = True   

            # Update the waypointsDefinition with the new coordinates
            waypointsDefinition = (coordinates[0], coordinates[1], coordinates[2], 0.0, 90.0, 0.0, 90.0)

            success &= example_trajectory(base, base_cyclic, waypointsDefinition)
            
            self.get_logger().info('Moved to position: %f, %f, %f' % (feedback.base.tool_pose_x, feedback.base.tool_pose_y, feedback.base.tool_pose_z))


def main(args=None):
    rclpy.init(args=args)

    arm_move_subscriber = ArmMoveSubscriber()
    rclpy.spin(arm_move_subscriber)
    arm_move_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()