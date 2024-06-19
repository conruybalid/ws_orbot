import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from custom_interfaces.msg import ArmControl

import arm_control.utilities as utilities

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

from arm_control.mytry import example_move_to_home_position, example_trajectory

import time


class ArmMoveSubscriber(Node):
    def __init__(self):
        super().__init__('arm_move_subscriber')
        self.subscription = self.create_subscription(
            # Specify the message type and topic name
            msg_type=ArmControl,
            topic='arm_move',
            callback=self.callback,
            qos_profile=10
        )
        self.absoluteSubscription = self.create_subscription(
            # Specify the message type and topic name
            msg_type=ArmControl,
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
        point = msg.position
        
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
            coordinates = [point.x + arm_x, point.y + arm_y, point.z + arm_z]

            success = True   

            # Update the waypointsDefinition with the new coordinates
            waypointsDefinition = (coordinates[0], coordinates[1], coordinates[2], 0.0, 90.0, 0.0, 90.0)

            success &= example_trajectory(base, base_cyclic, waypointsDefinition)
            
            self.get_logger().info('Moved to position: %f, %f, %f' % (feedback.base.tool_pose_x, feedback.base.tool_pose_y, feedback.base.tool_pose_z))

            self.gripper_control(base, msg.gripper_state)


    def absolute_callback(self, msg):
        point = msg.position

        with utilities.DeviceConnection.createTcpConnection(self.args) as router:   

            # Create required services
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)     
            
            feedback = base_cyclic.RefreshFeedback()

            # Get the coordinates from the user
            coordinates = [point.x, point.y, point.z]

            success = True   

            # Update the waypointsDefinition with the new coordinates
            waypointsDefinition = (coordinates[0], coordinates[1], coordinates[2], 0.0, 90.0, 0.0, 90.0)

            success &= example_trajectory(base, base_cyclic, waypointsDefinition)

            self.get_logger().info('Moved to position: %f, %f, %f' % (feedback.base.tool_pose_x, feedback.base.tool_pose_y, feedback.base.tool_pose_z))

            self.gripper_control(base, msg.gripper_state)



    def gripper_control(self, base, gripper_state):
        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        gripper_request = Base_pb2.GripperRequest()


        if gripper_state == 0:
            # Set speed to open gripper
            print ("Opening gripper using speed command...")
            gripper_command.mode = Base_pb2.GRIPPER_SPEED
            finger.value = 0.1
            base.SendGripperCommand(gripper_command)

            # Wait for reported position to be opened
            gripper_request.mode = Base_pb2.GRIPPER_POSITION
            while True:
                gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
                if len (gripper_measure.finger):
                    print("Current position is : {0}".format(gripper_measure.finger[0].value))
                    if gripper_measure.finger[0].value < 0.01:
                        break
                else: # Else, no finger present in answer, end loop
                    break

            self.get_logger().info('Gripper opened')

        elif gripper_state == 1:
            # Set speed to close gripper
            print ("Closing gripper using speed command...")
            gripper_command.mode = Base_pb2.GRIPPER_SPEED
            finger.value = -0.05
            base.SendGripperCommand(gripper_command)

            time.sleep(0.02)

            # Wait for reported speed to be 0
            gripper_request.mode = Base_pb2.GRIPPER_SPEED
            while True:
                gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
                if len (gripper_measure.finger):
                    print("Current speed is : {0}".format(gripper_measure.finger[0].value))
                    if gripper_measure.finger[0].value == 0.0:
                        break
                else: # Else, no finger present in answer, end loop
                    break

            self.get_logger().info('Gripper closed')

        else:
            self.get_logger().info('Invalid gripper state')






def main(args=None):
    rclpy.init(args=args)

    arm_move_subscriber = ArmMoveSubscriber()
    rclpy.spin(arm_move_subscriber)
    arm_move_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()