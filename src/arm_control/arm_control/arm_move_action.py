import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_interfaces.action import MoveArm

import arm_control.utilities as utilities

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

from arm_control.mytry import example_move_to_home_position, example_trajectory

import time


class MoveArmServer(Node):
    def __init__(self):
        super().__init__('move_arm_server')
        self.action_server = ActionServer(
            self,
            MoveArm,
            'move_arm_action',
            self.execute_callback
        )

        self.args = utilities.parseConnectionArguments()

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(self.args) as router:

            # Create required services
            base = BaseClient(router)
            
            # Example core
            success = True

            success &= example_move_to_home_position(base)


    def FormatWaypoint(self, waypointInformation: MoveArm.Goal, feedback: BaseCyclic_pb2.Feedback):
        waypoint = Base_pb2.CartesianWaypoint()
        
        if waypointInformation.reference_frame == Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL:
            waypoint.pose.x = waypointInformation.position.x
            waypoint.pose.y = waypointInformation.position.y
            waypoint.pose.z = waypointInformation.position.z
            waypoint.blending_radius = 0.0
            waypoint.pose.theta_x = waypointInformation.angle.x
            waypoint.pose.theta_y = waypointInformation.angle.y
            waypoint.pose.theta_z = waypointInformation.angle.z 
            waypoint.reference_frame = waypointInformation.reference_frame

        elif waypointInformation.reference_frame == Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE:
            
            if waypointInformation.position.z == 99.0:
                waypoint.pose.x = feedback.base.tool_pose_x
            else:
                waypoint.pose.x = waypointInformation.position.z
            
            if waypointInformation.position.x == 99.0:
                waypoint.pose.y = feedback.base.tool_pose_y
            else:
                waypoint.pose.y = waypointInformation.position.x

            if waypointInformation.position.y == 99.0:
                waypoint.pose.z = feedback.base.tool_pose_z
            else:
                waypoint.pose.z = waypointInformation.position.y

            waypoint.blending_radius = 0.0
            waypoint.pose.theta_x = waypointInformation.angle.z
            waypoint.pose.theta_y = waypointInformation.angle.x
            waypoint.pose.theta_z = waypointInformation.angle.y 
            waypoint.reference_frame = waypointInformation.reference_frame
        
        return waypoint



    def execute_callback(self, goal_handle):
        # Implement your action server logic here
        # You can access the goal, feedback, and result using goal_handle
        # For example:
        goal = goal_handle.request.goal
        feedback = MoveArm.Feedback()
        result = MoveArm.Result()

        # Publish feedback periodically
        # # Update feedback values
        # feedback.feedback = "Beginning movement..."
        # # Publish feedback
        # goal_handle.publish_feedback(feedback)

        with utilities.DeviceConnection.createTcpConnection(self.args) as router:
            point = goal.position
            gripper_state = goal.gripper_state
        
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
            waypointsDefinition = self.FormatWaypoint(goal, feedback)

            try:
                success &= example_trajectory(base, base_cyclic, waypointsDefinition)
            except:
                self.get_logger().info(f'Error in trajectory: {coordinates[0]}, {coordinates[1]}, {coordinates[2]}')
                goal_handle.abort()
                result.result = False
            
            self.get_logger().info('Moved to position: %f, %f, %f' % (feedback.base.tool_pose_x, feedback.base.tool_pose_y, feedback.base.tool_pose_z))

            self.gripper_control(base, gripper_state)

            


        # Check if the action was canceled
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.result = False
        else:
            # Set the result
            result.result = True  
            goal_handle.succeed()
        
        return result
    

    def gripper_control(self, base, gripper_state):
        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        gripper_request = Base_pb2.GripperRequest()

        # Do nothing with the gripper
        if gripper_state == 0:
            pass

        # Open Gripper
        elif gripper_state == 1:
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

        # Close Gripper
        elif gripper_state == 2:
            # Set speed to close gripper
            print ("Closing gripper using speed command...")
            gripper_command.mode = Base_pb2.GRIPPER_SPEED
            finger.value = -0.05
            base.SendGripperCommand(gripper_command)

            time.sleep(0.05)

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
    action_server = MoveArmServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()