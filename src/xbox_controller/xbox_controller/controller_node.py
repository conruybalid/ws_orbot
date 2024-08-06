import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from custom_interfaces.msg import Tank

from custom_interfaces.action import ArmPreset
from custom_interfaces.action import MoveArm

import time
from typing import List

import inputs
import importlib


def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

class XboxPublisher(Node):
    """
    Publisher node that takes in xbox controller inputs

    This class gets inputs from the xbox controller (wired or bluetooth)
    Then it converts the joystick inputs into valid tank commands
    If the A button is pressed, manual_control will be toggled 
    and the node will publish the tank commands

    Attributes:
        publisher_ (Publisher): The publisher instance for the Tank message.
        manual_control (bool): The state of manual control.
        left_speed (int): The speed of the left motor.
        right_speed (int): The speed of the right motor
    
    """
    def __init__(self):
        super().__init__('xbox_publisher')
        self.publisher_ = self.create_publisher(Tank, 'move_tank_commands', 10)
        
        self.manual_control: bool = False
        self.left_speed = 0
        self.right_speed = 0

        self.arm_action_client = ActionClient(self, ArmPreset, 'move_arm_preset_position')
        self.arm_waypoint_client = ActionClient(self, MoveArm, 'move_arm_waypoint')


        self.get_logger().info("Xbox Publisher Node Initialized")

    def check_controller(self):
        try:
            events = inputs.get_gamepad()
        except Exception as e:
            self.get_logger().warn(str(e))
            self.right_speed = 0
            self.left_speed = 0
            if self.manual_control:
                self.publish_tank(0,0)
            time.sleep(3)
            importlib.reload(inputs) # Re-import the devices module to reinitialize
            return
        for event in events:
            if event.ev_type == 'Key' or event.ev_type == 'Absolute':
                if event.code == 'BTN_SOUTH' and event.state == 1:
                    self.manual_control = not self.manual_control
                    self.get_logger().info(f"Manual Control: {self.manual_control}")
                
                if not self.manual_control:
                    if event.code == 'BTN_NORTH' and event.state == 1:
                        self.get_logger().info("Moving Arm Left")
                        self.send_preset_goal("Left Travel Angle")
                        return #So that only command is sent
                    if event.code == 'BTN_EAST' and event.state == 1:
                        self.get_logger().info("Moving Arm Right")
                        self.send_preset_goal("Right Travel Angle")
                        return
                    if event.code == 'BTN_WEST' and event.state == 1:
                        self.get_logger().info("Moving Arm Home")
                        self.send_preset_goal("Home")
                        return
                    if event.code == 'ABS_HAT0X':
                        if event.state == 1:
                            self.get_logger().info("Opening Gripper")
                            self.send_grip_goal(1)
                            return

                        elif event.state == -1:
                            self.get_logger().info("Closing Gripper")
                            self.send_grip_goal(2)
                            return

            if event.ev_type == 'Absolute':
                if event.code == 'ABS_Y':
                    self.left_speed = map_value(event.state, -32768, 32767, 1000, -1000)
                        
                if event.code == 'ABS_RY':
                    self.right_speed = map_value(event.state, -32768, 32767, 1000, -1000)

        return
    
    def send_preset_goal(self, preset: str):
        """
        Sends an preset goal to the arm action server.
        Waits for the server to respond and returns the result.
        """
        goal_msg = ArmPreset.Goal()
        goal_msg.preset_name = preset
        self.arm_action_client.wait_for_server()
        future = self.arm_action_client.send_goal_async(goal_msg)
        #future.add_done_callback(self.goal_response_callback)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Move Goal rejected :(')
            return False

        self.get_logger().debug(f'Move Goal sent: \n{goal_msg}')

        result_future = goal_handle.get_result_async()
        #result_future.add_done_callback(self.get_result_callback)
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().debug(f'Move Result: {result.result}')

        
        return result.result
    
    def send_grip_goal(self, gripper_state: int):
        """
        Sends a gripper goal to the arm waypoint action server.
        Waits for the server to respond and returns the result.
        """
        goal_msg = MoveArm.Goal()
        goal_msg.goal.reference_frame = 2 # The tool reference frame
        goal_msg.goal.gripper_state = gripper_state

        self.arm_waypoint_client.wait_for_server()
        future = self.arm_waypoint_client.send_goal_async(goal_msg)
        #future.add_done_callback(self.goal_response_callback)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Move Goal rejected :(')
            return False

        self.get_logger().debug(f'Move Goal sent: \n{goal_msg}')

        result_future = goal_handle.get_result_async()
        #result_future.add_done_callback(self.get_result_callback)
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().debug(f'Move Result: {result.result}')

    
    def publish_tank(self, L, R):
        if L < 100 and L > -100:
            L = 0
        if R < 100 and R > -100:
            R = 0
        msg = Tank()
        msg.left_speed = L
        msg.right_speed = R
        self.publisher_.publish(msg)
        self.get_logger().debug(f"Publishing: {msg.left_speed}, {msg.right_speed}")
        

def main(args=None):
    rclpy.init(args=args)
    node = XboxPublisher()
    try:
        while rclpy.ok():
            node.check_controller()
            if node.manual_control:
                node.publish_tank(node.left_speed, node.right_speed)

            #rclpy.spin_once(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()