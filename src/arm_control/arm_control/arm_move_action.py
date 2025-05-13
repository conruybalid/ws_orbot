import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from threading import Lock

from custom_interfaces.action import MoveArm
from custom_interfaces.action import ArmPreset

import arm_control.utilities as utilities

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2

from arm_control.moveArm import move_to_home_position, move_to_preset_position, move_trajectory, move_trajectory_Record
from arm_control.gripperControl import GripperCommand

from arm_control.FileRecorder import FileRecorder

class MoveArmServer(Node):
    """
    Action server for the MoveArm action.

    This class handles the execution of the MoveArm action. It receives a goal from a client,
    moves the arm to the specified position, and controls the gripper based on the gripper state
    specified in the goal.

    It also handles the execution of the ArmPreset action, which moves the arm to a named preset position.

    Args:
        router: The router object used for communication with the robot.

    Attributes:
        action_server (ActionServer): The action server instance for the move_arm_waypoint.
        preset_action_server (ActionServer): The action server instance for the preset action.
        router: The router object used for communication with the robot.
        gripper_control (GripperCommand): The gripper control instance.
        self.move_tolerance (float): The tolerance for the arm to be in the desired position.
        self.mutex (Lock): A mutex to avoid doing waypoint and preset action callbacks at the same time.
    """

    def __init__(self, router):
        super().__init__('move_arm_server')
        
        self.mutex = Lock() # To avoid doing waypoint and preset action callbacks at the same time

        self.action_server = ActionServer(
            self,
            MoveArm,
            'move_arm_waypoint',
            self.execute_callback
        )

        self.preset_action_server = ActionServer(
            self,
            ArmPreset,
            'move_arm_preset_position',
            self.execute_preset_callback
        )

        self.move_tolerance = 0.01

        self.router = router        

        # Create required services
        base = BaseClient(self.router)
        
        # Example core
        success = True

        success &= move_to_home_position(base)
        self.get_logger().info('Moved to home position')

        self.gripper_control = GripperCommand(self.router)

        # self.csvFile = FileRecorder()


    def FormatWaypoint(self, waypointInformation: MoveArm.Goal, feedback: BaseCyclic_pb2.Feedback) -> Base_pb2.CartesianWaypoint: # type: ignore
        """
        Format the waypoint information based on the reference frame.

        Args:
            waypointInformation (MoveArm.Goal): The goal containing the waypoint information.
            feedback (BaseCyclic_pb2.Feedback): The feedback received from the robot.

        Returns:
            Base_pb2.CartesianWaypoint: The formatted waypoint.
        """
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

    def execute_callback(self, goal_handle) -> MoveArm.Result:
        """
        Execute the MoveArm action.

        This method is called when a goal is received from a client. It moves the arm to the specified position
        and controls the gripper based on the gripper state specified in the goal.

        Args:
            goal_handle (ActionServer.GoalHandle): The goal handle containing the goal, feedback, and result.

        Returns:
            MoveArm.Result: The result of the action execution. (True if successful, False otherwise)
        """
        with self.mutex: # To avoid doing waypoint and preset action callbacks at the same time
            goal = goal_handle.request.goal
            feedback = MoveArm.Feedback()
            result = MoveArm.Result()

            # Seperate out the different parts of the goal
            point = goal.position
            gripper_state = goal.gripper_state
        
            # Create required services
            base = BaseClient(self.router)
            base_cyclic = BaseCyclicClient(self.router)     
            
            feedback = base_cyclic.RefreshFeedback()

            # Accessing the X, Y, Z position of the tool
            old_arm_z = feedback.base.tool_pose_x
            old_arm_x = feedback.base.tool_pose_y
            old_arm_y = feedback.base.tool_pose_z


            success = True   

            # Update the waypointsDefinition with the new coordinates
            waypointsDefinition = self.FormatWaypoint(goal, feedback)

            try:
                success &= move_trajectory(base, base_cyclic, waypointsDefinition)
                # success &= move_trajectory_Record(base, base_cyclic, waypointsDefinition, self.csvFile)
            except:
                self.get_logger().error(f'Error in trajectory: {point.x}, {point.y}, {point.z}')
                goal_handle.abort()
                result.result = False
                return result

            # Check if the new position of the arm is what was expected

            # Get the new position of the arm
            feedback = base_cyclic.RefreshFeedback()

            arm_z = feedback.base.tool_pose_x
            arm_x = feedback.base.tool_pose_y
            arm_y = feedback.base.tool_pose_z

            self.get_logger().info('Moved to position: %f, %f, %f' % (arm_x, arm_y, arm_z))

            # If the reference frame is tool, calculate desired position with respect to the base
            if goal.reference_frame == Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL:
                point.x += old_arm_x
                point.y += old_arm_y
                point.z += old_arm_z


            # Check if the arm actually reached the desired position
            if ((arm_x < point.x - self.move_tolerance or arm_x > point.x + self.move_tolerance)
                or (arm_y < point.y - self.move_tolerance or arm_y > point.y + self.move_tolerance)
                or (arm_z < point.z - self.move_tolerance or arm_z > point.z + self.move_tolerance)):
                    self.get_logger().error('Failed to reach the desired position')
                    goal_handle.abort()
                    result.result = False
                    return result

            # Gripper control

            # Do nothing with the gripper
            if gripper_state == 0:
                pass

            # Open Gripper
            elif gripper_state == 1:
                self.gripper_control.OpenGripper()
                self.get_logger().info('Gripper opened')

            # Close Gripper
            elif gripper_state == 2:
                # self.gripper_control.GripApple() # Closes the gripper incrementally to avoid crushing apple
                self.gripper_control.SetGripper(0.55) # Closes the gripper to a set position
                self.get_logger().info('Gripper closed')

            else:
                self.get_logger().warn('Invalid gripper state')


            # Check if the action was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.result = False
            else:
                # Set the result
                result.result = True  
                goal_handle.succeed()
            
            return result


    def execute_preset_callback(self, goal_handle) -> ArmPreset.Result:
        """
        Moves to a named preset position defined in the web UI (under action)
        WARNING: Angle presets do not check for collisions, use with caution

        Args:
            goal_handle (ActionServer.GoalHandle): The goal handle containing the goal, feedback, and result.

        Returns:
            MoveArm.Result: The result of the action execution. (True if successful, False otherwise)
        """

        with self.mutex: # To avoid doing waypoint and preset action callbacks at the same time
            preset = goal_handle.request.preset_name
            feedback = ArmPreset.Feedback()
            result = ArmPreset.Result()

            # Create required services
            base = BaseClient(self.router)
            base_cyclic = BaseCyclicClient(self.router) 

            self.get_logger().info(f'Moving to preset position: {preset}')
            success = True
            try:
                success &= move_to_preset_position(base, preset)
            except:
                self.get_logger().error(f'Error in moving to preset: {preset}')
                goal_handle.abort()
                result.result = False
                return result
            
            if not success:
                self.get_logger().error(f'Error in moving to preset: {preset}')
                goal_handle.abort()
                result.result = False
                return result

            # Check if the action was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.result = False
            else:
                # Set the result
                result.result = True  
                goal_handle.succeed()
            
            return result
    







def main(args=None):
    print(f'arguments: {args}')
    rclpy.init(args=args)
    arm_args = utilities.parseConnectionArguments()
    with utilities.DeviceConnection.createTcpConnection(arm_args) as router: # create connection to robot
        action_server = MoveArmServer(router)
        rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()