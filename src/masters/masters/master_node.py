import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from custom_interfaces.msg import ArmControl
from custom_interfaces.action import MoveArm
from custom_interfaces.action import ArmPreset
from custom_interfaces.srv import GetLocation
from custom_interfaces.msg import Tank

from geometry_msgs.msg import Point


from kortex_api.autogen.messages import Base_pb2

import time

from typing import Tuple

from masters.Timing import OrbotTimer


SCOOTMOVETIME = 10

class MasterNode(Node):
    """
    This class represents the master node of the system.
    This node is the main node that ties the other nodes together and performs the main logic of the system.
    Currently, it completes its task and then destroys itself.

    Attributes:

        Service Clients:
            zed_client (ServiceClient): A service client for the zed location service. Returns the location of apples from the zed camera (not active in legacy mode)
            arm_client (ServiceClient): A service client for the arm location service. Returns the location of apples from the arm camera

        Action Clients:
            arm_action_client (ActionClient): An action client for the move arm waypoint action.
            arm_presets_client (ActionClient): An action client for the arm preset action.

        Other Attributes:
            home (str): The name of the home position preset in use ('Home' or 'Home Right')

    args:
        tank (bool): If true, the tank wheels will be used to move the robot.
        legacy (bool): If true, the legacy routine will be used (One Camera with Red Filter)

    """
    def __init__(self, tank: bool = False, legacy: bool = False):
        """
        Init fuction for the MasterNode class.
        calls the init function of the parent class (Node) and initializes the subscriptions, publishers and action clients.
        """

        super().__init__('Master_Node')
        self.tank: bool = tank

        self.get_logger().info('Initializing Master Node')

        if not legacy:
            self.zed_client = self.create_client(GetLocation, 'zed_location_service')
            while not self.zed_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('zed_location_service not available, waiting again...')
        else:
            self.get_logger().info('Legacy Camera Activated')

        self.arm_client = self.create_client(GetLocation, 'Arm_Locate_Apple_Service')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Arm_Locate_Apple_Service not available, waiting again...')
        
        self.arm_action_client = ActionClient(self, MoveArm, 'move_arm_waypoint')
        self.arm_presets_client = ActionClient(self, ArmPreset, 'move_arm_preset_position')

        if self.tank:
            self.get_logger().info('Tank Mode Activated')
            self.tank_comand_publisher = self.create_publisher(Tank, 'move_tank_commands', 10)

        self.home: str = 'Home'

        # self.TimeData = OrbotTimer()

        self.get_logger().info('Master Node Initialized')

    

    """
    SERVICE CALL FUNCTIONS
    """
    def call_zed_service(self) -> Tuple[int, Point]:
        """
        Calls the zed location service to get the location of the apple in the image.
        """
        request = GetLocation.Request() # Create a request object

        future = self.zed_client.call_async(request) # Call the service
        rclpy.spin_until_future_complete(self, future) # Wait for the service to respond
        response = future.result() # Get the response

        return response.error_status, response.apple_coordinates
    
    def call_arm_service(self) -> Tuple[int, Point]:
        """
        Calls the arm location service to get the location of the apple in the image.
        """
        request = GetLocation.Request() # Create a request object

        future = self.arm_client.call_async(request) # Call the service
        rclpy.spin_until_future_complete(self, future) # Wait for the service to respond
        response = future.result()  # Get the response

        return response.error_status, response.apple_coordinates
    

    def process_service_error(self, error_code: int) -> None:
        """
        Processes the error code received from either camera service.
        Outputs the appropriate message based on the error code.
        """
        if error_code == 1:
            self.get_logger().info('No apples found')
        elif error_code == 2:
            self.get_logger().error('Failed to process image')
        elif error_code == 3:
            self.get_logger().error('Service Node had no image to process')
        else:
            self.get_logger().error('Unknown Error Code')
        return
    

    """
    ARM MOVEMENT FUNCTIONS
    These Functions are used to send arm movement goals to the arm action server.
    """

    def format_move_goal(self, position: Tuple[float,float,float]=[0.0,0.0,0.0], angle: Tuple[float,float,float]=[0.0,0.0,0.0], gripper_state: int=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL) -> MoveArm.Goal:
        """
        Quickly formates a goal that can be sent to the arm action server.
        """
        goal_msg = MoveArm.Goal()
        goal_msg.goal.position.x = position[0]
        goal_msg.goal.position.y = position[1]
        goal_msg.goal.position.z = position[2]
        goal_msg.goal.angle.x = angle[0]
        goal_msg.goal.angle.y = angle[1]
        goal_msg.goal.angle.z = angle[2]
        goal_msg.goal.reference_frame = reference_frame
        goal_msg.goal.gripper_state = gripper_state

        return goal_msg
    
    def send_move_goal(self, goal_msg: MoveArm.Goal) -> bool:
        """
        Sends an waypoint goal to the arm action server.
        Waits for the server to respond and returns the result.
        """

        self.arm_action_client.wait_for_server() # Wait for the server to be ready
        future = self.arm_action_client.send_goal_async(goal_msg) # Send the goal
        rclpy.spin_until_future_complete(self, future) # Wait for the server to respond

        goal_handle = future.result() # See if the goal was accepted

        if not goal_handle.accepted:
            self.get_logger().error('Move Goal rejected :(')
            return False

        self.get_logger().debug(f'Move Goal sent: \n{goal_msg}')

        result_future = goal_handle.get_result_async() # Prepare to get the result
        rclpy.spin_until_future_complete(self, result_future) # Wait for the result
        result = result_future.result().result # Get the result
        self.get_logger().debug(f'Move Result: {result.result}')
        
        return result.result
    
    def send_preset_goal(self, preset: str) -> bool:
        """
        Sends an preset goal to the arm preset action server.
        Waits for the server to respond and returns the result.
        """
        goal_msg = ArmPreset.Goal()
        goal_msg.preset_name = preset
        self.arm_presets_client.wait_for_server()
        future = self.arm_presets_client.send_goal_async(goal_msg)
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


    """
    GO TO APPLE FUNCTIONS
    """
   
    def Go_to_Apple(self) -> bool:
        """
        Will ask the zed_location_service for the location of the apple relative to the base of the arm.
        Then it will move the arm to that location.
        """
        self.get_logger().debug('Asking Zed Camera for Apple Location')

        # self.TimeData.RecordTime(0)
             
        error_status, point = self.call_zed_service()

        if error_status == 3:
            for _ in range(3):
                time.sleep(1) 
                error_status, point = self.call_zed_service()
                if error_status != 3:
                    break

        if error_status != 0:
            self.process_service_error(error_status)
            return False

        if point.x < 0.0:
            self.get_logger().info('Apple to the right: using right home')
            self.home = 'Home Right'
        else:
            self.get_logger().info('Apple to the left, using forward home')
            self.home = 'Home Left'

        if (self.send_preset_goal(self.home)):#WARNING: This preset does not check for collisions
            
            
            x = point.x
            y = point.y
            z = point.z 
            
            self.get_logger().info(f'Apple at {x}, {y}, {z}')

            #Adjust coordinates so that the camera can see the apple
            if (self.send_move_goal(self.format_move_goal(position=[x, y - 0.025, z - 0.2], angle=[0.0, 90.0, 90.0], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE))):
                self.get_logger().info('Moved to apple location')
            else:
                self.get_logger().error('Failed to move to apple location')
                return False

            return True
        
        else:
            self.get_logger().error('Failed to move to preset')
            return False
    


    """
    CENTER APPLE FUNCTIONS
    These functions are used to center the apple in the frame.
    """

    def center_apple(self) -> bool:
        """
        Will ask the arm_location_service for the location of the apple relative to the arm.
        Then it will move the arm to attempt to center the apple in the frame.
        It will continue until the apple is within a certain range of the center of the frame and return True.
        If sight of the apple is lost, the function will return False.
        Once the apple is centered, the arm will move towards the apple.
        """
        tolerance = 0.02
        apple_coordinates = None
        while (apple_coordinates == None or not ((apple_coordinates.x <= tolerance and apple_coordinates.x >= -tolerance) and (apple_coordinates.y <= tolerance and apple_coordinates.y >= -tolerance))):

            move_msg = ArmControl()

            self.get_logger().info('Asking Arm Camera for Apple Location')
            error_status, apple_coordinates = self.call_arm_service()

            if error_status != 0:
                self.process_service_error(error_status)
                return False


            # Turn apple coordinate into action message
            move_msg.position = apple_coordinates
            # If the Arm is not currently centered on the apple, overwrite z coordinate with 0
            if not ((apple_coordinates.x <= tolerance and apple_coordinates.x >= -tolerance) and (apple_coordinates.y <= tolerance and apple_coordinates.y >= -tolerance)):
                move_msg.position.z = 0.0
            else:
                self.get_logger().info(f'Apple Centered. Moving to {apple_coordinates.x}, {apple_coordinates.y}, {apple_coordinates.z}')
                if not (move_msg.position.z > 0.0):
                    self.get_logger().warn("Rejecting Move, Distance was not greater than zero")
                    return False
            
            move_msg.angle.x = 0.0
            move_msg.angle.y = 0.0
            move_msg.angle.z = 0.0        
            move_msg.gripper_state = 0
            move_msg.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL


            move_goal = MoveArm.Goal()
            move_goal.goal = move_msg

            # Publish Arm Movement
            if not self.send_move_goal(move_goal):
                return False
            # self.send_move_goal(move_goal)
            self.get_logger().debug(f'moved {move_msg.position.x} in x, {move_msg.position.y} in y')
        
        
        return True
    

    
    """
    RETRIEVE APPLE FUNCTIONS
    These functions are used to grab the apple and drop it in a basket.
    """     

    def grab_apple(self):
        """
        Once the apple is within the gripper's reach, 
        this function can be used to grab the apple and deposit it in the basket.
        movement goals are in an if statement to ensure they have been completed before moving on.
        """

        try:
            # Pick Apple
            # Perform the movements to pick up the apple
            if not (
            self.send_move_goal(self.format_move_goal(position=[0.0, 0.025, 0.0], gripper_state=2, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL))
            and self.send_move_goal(self.format_move_goal(angle=[0.0, 0.0, 100.0], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL))
            and self.send_move_goal(self.format_move_goal(position=[0.0, 0.0, -0.2], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL))
            ):
                # Once of the above movements failed
                self.get_logger().error('failed to perform pick apple movements')
            else:
                # All movements were successful
                self.get_logger().info('finished pick apple movements')
                # self.TimeData.RecordTime(5)

            
            # Drop off apple
            self.send_preset_goal(self.home) #WARNING: This preset does not check for collisions

            if not (
            self.send_preset_goal("Drop Apple Angle")#WARNING: This preset does not check for collisions
            ):
                self.get_logger().error('Failed to drop off apple')
                self.send_move_goal(self.format_move_goal(gripper_state=1))
                time.sleep(1)
            else:
                self.send_move_goal(self.format_move_goal(gripper_state=1))
                self.get_logger().info('Dropped off apple')



            
        # If the user aborts the program, the gripper will open
        except KeyboardInterrupt:
            self.get_logger().warn('Abort, Opening Gripper')
            self.send_move_goal(self.format_move_goal(gripper_state=1))


        return
    
    def publish_tank_commands(self, L: int, R: int):
        """
        Publishes the tank movement commands to the tank controller node.
        """
        msg = Tank() # Create a message object
        msg.left_speed = L
        msg.right_speed = R
        self.tank_comand_publisher.publish(msg) # Publish the message
        self.get_logger().debug(f"Publishing: {msg.left_speed}, {msg.right_speed}")
        return

    def move_tank(self):
        """
        Moves the tank forward until apple is found
        """
        self.get_logger().info('Moving Tank')
        try:                    
            error_status, point = self.call_zed_service()
            
            fail_count = 0
            while error_status != 0 and fail_count < 3: # If there is an error in image processing 3 times in a row, then stop. Or stop if apple is found (error_status == 0)
                self.publish_tank_commands(-200, -200)
                error_status, point = self.call_zed_service()
                if error_status == 0 or error_status == 1:
                    fail_count = 0
                    continue
                else:
                    fail_count += 1
                    time.sleep(2)
                
        finally:
            self.get_logger().info('Stopping Tank')
            self.publish_tank_commands(0, 0)
            self.publish_tank_commands(0, 0) # Twice for extra measure I guess (in case the first one doesn't go through)

        return


    """
    MAIN ROUTINES
    This is the main routine of the master node.
    """

    def run(self):
        """
        This function:
        Moves the arm to the apple with info form the zed camera
        Then it centers the apple using the arm camera
        Then it reaches for the apple
        Then it grabs the apple and deposits it in the basket
        if tank is active, it moves the tank forward until reachable apples are found
        """

        self.get_logger().info('Master Node Routine Started')
        

        try:
            # self.TimeData.RestartTimer()
            while True: # only broken on keyboard interrupt or error

                if self.tank:
                    self.get_logger().info('Moving on...')
                    self.move_tank()
                
                while self.Go_to_Apple(): # Broken if Go to Apple fails (no apples found)
                    # self.TimeData.RecordTime(1)
                    # Center the apple
                    self.get_logger().info('Centering Apple')
                    # self.TimeData.RecordTime(2)
                    if not self.center_apple():
                        self.send_preset_goal(self.home)
                        # self.TimeData.RecordRow()
                        continue
                    # self.TimeData.RecordTime(3)
                    # self.TimeData.RecordTime(4)
                    self.grab_apple()

                    # self.TimeData.RecordRow()


        except KeyboardInterrupt:
            self.get_logger().warn('Routine Aborted')

        finally:
            if self.tank:
                # Stop tank
                self.publish_tank_commands(0, 0)

            # Return to home position
            self.send_preset_goal(self.home)

    
        return
            

    def PickNScoot(self, picks):
        """
        This function:
        is the same as run except it picks one apple then moves on "picks number of times 
        """
        self.get_logger().info('Pick N Scoot Routine Started')
        

        try:
            for i in range(picks):
                if i != 0:
                    self.get_logger().info('Moving on...')
                    self.publish_tank_commands(-200, -200)
                    time.sleep(SCOOTMOVETIME)
                    self.publish_tank_commands(0, 0)
                    self.publish_tank_commands(0, 0) # Twice for good measure

                time.sleep(1)

                self.get_logger().info(f'picking iteration {i+1}')
                if not self.Go_to_Apple():
                    continue
                
                # Center the apple
                self.get_logger().info('Centering Apple')
                if not self.center_apple():
                    self.send_preset_goal(self.home)
                    continue
                
                self.grab_apple()


        except KeyboardInterrupt:
            self.get_logger().warn('Routine Aborted')

        finally:
            # Stop tank
            self.publish_tank_commands(0, 0)

            # Return to home position
            self.send_preset_goal('Home')

    
        return




    def legacy(self):
        """
        This is a legacy routine that uses the red filter camera to locate the apple using one camera
        This function:
        centers the apple using the arm camera
        Then it reaches for the apple
        Then it grabs the apple and deposits it in the basket
        if tank is active, it moves the tank forward for 3 seconds when no apples are found
        """

        self.get_logger().info('Master Node Routine Started (one camera)')
        

        try:
            while True:

                if self.tank:
                    self.get_logger().info('Moving on...')
                    self.publish_tank_commands(-200, -200)
                    time.sleep(3)
                    self.publish_tank_commands(0, 0)

                                
                while True: # Broken if Go to Apple fails (no apples found)
                    
                    self.get_logger().info('Centering Apple')
                    if not self.center_apple():
                        self.send_preset_goal(self.home)
                        break

                    self.get_logger().info('Centered Apple, proceeding to grab')
                    
                    self.grab_apple()

                    self.send_preset_goal(self.home)
                
                time.sleep(2)


        except KeyboardInterrupt:
            self.get_logger().warn('Routine Aborted')

        finally:
            if self.tank:
                # Stop tank
                self.publish_tank_commands(0, 0)

            # Return to home position
            self.send_preset_goal('Home')

    
        return
   
    

    """
    TESTING ROUTINE
    This function can be used to test the functionality of the master node.
    """
    
    def test(self):
        """
        Feel free to modify this function to test the functionality of the master node.
        """
        self.get_logger().info('Testing')
    

        # Center the apple
        self.get_logger().info('Centering Apple')
        if not self.center_apple():
            self.get_logger().error('Failed to center apple')
            self.send_preset_goal(self.home)
        
        else:
            self.grab_apple()
            self.send_preset_goal(self.home)


        self.get_logger().info('Test Complete')


        return


"""
Parse Arguments Function
"""

def parse_args():
    """
    This function parses the command line arguments.
    """
    import argparse
    parser = argparse.ArgumentParser(description='Master Node')
    parser.add_argument('--tank', action='store_true', help='Run the Tank wheels')
    parser.add_argument('--legacy', action='store_true', help='Run the legacy routine (One Camera with Red Filter)')
    parser.add_argument('--test', action='store_true', help='Run the Test Routine')
    parser.add_argument('--PickNScoot', type=int, help='Run the PickNScoot routine with the specified value')

    parsed_args, unknown = parser.parse_known_args()
    return parsed_args, unknown

"""
Main Function
"""

def main(args=None):
    """
    This function initializes the master node and runs the main routine.
    Once it is complete, it cleans up by destroying the node and shutting down the rclpy system.
    """
    parsed_args, unknown = parse_args()
    rclpy.init(args=unknown)
    if parsed_args.PickNScoot is not None:
        node = MasterNode(tank=True, legacy=False)
    elif parsed_args.test:
        node = MasterNode(tank=False, legacy=True)
    else:
        node = MasterNode(parsed_args.tank, parsed_args.legacy)
    rclpy.spin_once(node, timeout_sec=1.0)
    # Check what routine to run
    if parsed_args.test:
        node.test()
    elif parsed_args.legacy:
        node.legacy()
    elif parsed_args.PickNScoot is not None:
        node.PickNScoot(parsed_args.PickNScoot)
    else:
        node.run() # Run the main routine  

    node.get_logger().info('Destroying Master Node')
    # Destroy action clients
    node.arm_action_client.destroy()
    node.arm_presets_client.destroy()

    # Destroy node and shutdown rclpy
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
