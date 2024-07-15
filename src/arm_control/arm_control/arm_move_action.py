import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_interfaces.action import MoveArm

import arm_control.utilities as utilities

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2

from arm_control.moveArm import move_to_home_position, move_trajectory
from arm_control.gripperControl import GripperCommand


class MoveArmServer(Node):
    """
    Action server for the MoveArm action.

    This class handles the execution of the MoveArm action. It receives a goal from a client,
    moves the arm to the specified position, and controls the gripper based on the gripper state
    specified in the goal.

    Args:
        router: The router object used for communication with the robot.

    Attributes:
        action_server (ActionServer): The action server instance for the move_arm_action.
        router: The router object used for communication with the robot.
        gripper_control (GripperCommand): The gripper control instance.
    """

    def __init__(self, router):
        super().__init__('move_arm_server')
        self.action_server = ActionServer(
            self,
            MoveArm,
            'move_arm_action',
            self.execute_callback
        )

        self.router = router        

        # Create required services
        base = BaseClient(self.router)
        
        # Example core
        success = True

        success &= move_to_home_position(base)

        self.gripper_control = GripperCommand(self.router)

    def FormatWaypoint(self, waypointInformation: MoveArm.Goal, feedback: BaseCyclic_pb2.Feedback):
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

    def execute_callback(self, goal_handle):
        """
        Execute the MoveArm action.

        This method is called when a goal is received from a client. It moves the arm to the specified position
        and controls the gripper based on the gripper state specified in the goal.

        Args:
            goal_handle (ActionServer.GoalHandle): The goal handle containing the goal, feedback, and result.

        Returns:
            MoveArm.Result: The result of the action execution. (True if successful, False otherwise)
        """

        goal = goal_handle.request.goal
        feedback = MoveArm.Feedback()
        result = MoveArm.Result()
        
        point = goal.position
        gripper_state = goal.gripper_state
    
        # Create required services
        base = BaseClient(self.router)
        base_cyclic = BaseCyclicClient(self.router)     
        
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
            success &= move_trajectory(base, base_cyclic, waypointsDefinition)
        except:
            self.get_logger().info(f'Error in trajectory: {coordinates[0]}, {coordinates[1]}, {coordinates[2]}')
            goal_handle.abort()
            result.result = False
        
        self.get_logger().info('Moved to position: %f, %f, %f' % (feedback.base.tool_pose_x, feedback.base.tool_pose_y, feedback.base.tool_pose_z))


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
            self.gripper_control.GripApple()
            self.get_logger().info('Gripper closed')

        else:
            self.get_logger().info('Invalid gripper state')


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
    rclpy.init(args=args)
    args = utilities.parseConnectionArguments()
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        action_server = MoveArmServer(router)
        rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()