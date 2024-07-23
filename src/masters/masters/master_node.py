import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from custom_interfaces.msg import ArmControl
from custom_interfaces.action import MoveArm
from custom_interfaces.srv import GetLocation

from kortex_api.autogen.messages import Base_pb2

import time


class MasterNode(Node):
    """
    This class represents the master node of the system.
    This node is the main node that ties the other nodes together and performs the main logic of the system.
    Currently, it completes its task and then destroys itself.

    Attributes:

        Service Clients:
            zed_client (ServiceClient): A service client for the zed location service. Returns the location of apples from the zed camera
            arm_client (ServiceClient): A service client for the arm location service. Returns the location of apples from the arm camera

        Action Clients:
            arm_action_client (ActionClient): An action client for the move arm action.

        Other Attributes:
            distance (float): The distance to the apple from the zed camera. Used to determine how far the arm should reach out.

    """
    def __init__(self):
        """
        Init fuction for the MasterNode class.
        calls the init function of the parent class (Node) and initializes the subscriptions, publishers and action clients.
        """

        super().__init__('Master_Node')

        self.get_logger().info('Initializing Master Node')

        self.zed_client = self.create_client(GetLocation, 'zed_location_service')
        while not self.zed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('zed_location_service not available, waiting again...')

        self.arm_client = self.create_client(GetLocation, 'Arm_Locate_Apple_Service')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Arm_Locate_Apple_Service not available, waiting again...')
        
        self.distance = None

        self.arm_action_client = ActionClient(self, MoveArm, 'move_arm_action')

        self.get_logger().info('Master Node Initialized')

    

    """
    SERVICE CALL FUNCTIONS
    """
    def call_zed_service(self):
        """
        Calls the zed location service to get the location of the apple in the image.
        """
        request = GetLocation.Request()

        future = self.zed_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.error_status, response.apple_coordinates
    
    def call_arm_service(self):
        """
        Calls the arm location service to get the location of the apple in the image.
        """
        request = GetLocation.Request()

        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.error_status, response.apple_coordinates
    

    def process_service_error(self, error_code):
        """
        Processes the error code received from either camera service.
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

    def format_move_goal(self, position=[0.0,0.0,0.0], angle=[0.0,0.0,0.0], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL):
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
    
    def send_move_goal(self, goal_msg):
        """
        Sends an waypoint goal to the arm action server.
        Waits for the server to respond and returns the result.
        """

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


    """
    GO TO APPLE FUNCTIONS
    """
   
    def Go_to_Apple(self):
        """
        Will ask the zed_location_service for the location of the apple relative to the base of the arm.
        Then it will move the arm to that location.
        """
        self.get_logger().info('Asking Zed Camera for Apple Location')
        for _ in range(3):     
            error_status, point = self.call_zed_service()
           
            if error_status != 0:
                self.process_service_error(error_status)
                time.sleep(5)
                continue
        if error_status != 0:
            return False
        
        x = point.x
        y = point.y
        self.distance = point.z 
        
        self.get_logger().info(f'Apple at {x}, {y}, {self.distance}')

        #Adjust coordinates so that the camera can see the apple
        if (self.send_move_goal(self.format_move_goal(position=[x, y - 0.025, self.distance - 0.2], angle=[0.0, 90.0, 90.0], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE))):
            self.get_logger().info('Moved to apple location')
        else:
            self.get_logger().error('Failed to move to apple location')
            return False

        return True
    


    """
    CENTER APPLE FUNCTIONS
    These functions are used to center the apple in the frame.
    """

    def center_apple(self):
        """
        Will ask the arm_location_service for the location of the apple relative to the arm.
        Then it will move the arm to attempt to center the apple in the frame.
        It will continue until the apple is within a certain range of the center of the frame and return True.
        If sight of the apple is lost, the function will return False.
        """
        tolerance = 0.02
        apple_coordinates = None
        while (apple_coordinates == None or not ((apple_coordinates.x <= tolerance and apple_coordinates.x >= -tolerance) and (apple_coordinates.y <= tolerance and apple_coordinates.y >= -tolerance))):
            rclpy.spin_once(self)

            move_msg = ArmControl()

            error_status, apple_coordinates = self.call_arm_service()

            if error_status != 0:
                self.process_service_error(error_status)
                return False


            # scale the coordinates and create message
            move_msg.position = apple_coordinates
            move_msg.angle.x = 0.0
            move_msg.angle.y = 0.0
            move_msg.angle.z = 0.0        
            move_msg.gripper_state = 0
            move_msg.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL


            move_goal = MoveArm.Goal()
            move_goal.goal = move_msg

            # Publish Arm Movement
            self.send_move_goal(move_goal)
            self.get_logger().debug(f'moved {move_msg.position.x} in x, {move_msg.position.y} in y')
        
        return True
    

    
    """
    RETRIEVE APPLE FUNCTIONS
    These functions are used to reach out, grab the apple and drop it in a basket.
    """

    def reach_apple(self):
        """
        If a distance has been received from the zed camera,
        this function can be used to reach out to the apple.
        """

        self.get_logger().debug('Reaching for Apple at absolute distance %f' % self.distance)
        arm_msg = ArmControl()
        arm_msg.position.x = 99.0
        arm_msg.position.y = 99.0
        arm_msg.position.z = self.distance
        arm_msg.angle.x = 0.0
        arm_msg.angle.y = 90.0
        arm_msg.angle.z = 90.0        
        arm_msg.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE 
        arm_msg.gripper_state = 1
        

        move_goal = MoveArm.Goal()
        move_goal.goal = arm_msg

        if self.send_move_goal(move_goal):
            return True
        else:
            self.get_logger().error('Could not reach for apple')
            return False
        

    def grab_apple(self):
        """
        Once the apple is within the gripper's reach, 
        this function can be used to grab the apple and deposit it in the basket.
        movement goals are in an if statement to ensure they have been completed before moving on.
        """

        try:
            # Pick Apple
            if not (
            self.send_move_goal(self.format_move_goal(position=[0.0, 0.025, 0.0], gripper_state=2, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL))
            and self.send_move_goal(self.format_move_goal(angle=[0.0, 0.0, 100.0], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL))
            and self.send_move_goal(self.format_move_goal(position=[0.0, 0.0, -0.2], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL))
            ):
                self.get_logger().error('failed to perform pick apple movements')
            else:
                self.get_logger().info('finished pick apple movements')

            
            # Drop off apple
            self.send_move_goal(self.format_move_goal(position=[0.0, 0.5, 0.5], angle=[0.0, 90.0, 90.0], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE))

            if not (
            self.send_move_goal(self.format_move_goal(position=[-0.3, 0.4, 0.0], angle=[0.0, 90.0, 180.0], gripper_state=1, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE))
            ):
                self.get_logger().error('Failed to drop off apple')
                self.send_move_goal(self.format_move_goal(gripper_state=1))
            else:
                self.get_logger().info('Dropped off apple')
                self.send_move_goal(self.format_move_goal(position=[0.0, 0.5, 0.5], angle=[0.0, 90.0, 90.0], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE))


            
        # If the user aborts the program, the gripper will open
        except KeyboardInterrupt:
            self.get_logger().warn('Abort, Opening Gripper')
            self.send_move_goal(self.format_move_goal(gripper_state=1))


        return


    """
    MAIN ROUTINE
    This is the main routine of the master node.
    """

    def run(self):
        """
        This function:
        Moves the arm to the apple with info form the zed camera
        Then it centers the apple using the arm camera
        Then it reaches for the apple
        Then it grabs the apple and deposits it in the basket
        """

        self.get_logger().info('Master Node Routine Started')
        try:
            while True:

                if not self.Go_to_Apple():
                    break

                # Center the apple
                self.get_logger().info('Centering Apple')
                if not self.center_apple():
                    break
                
                # Reach for the Apple
                if self.distance is not None:
                    if self.reach_apple():
                        # Grab the Apple
                        self.grab_apple()

                else:
                    self.get_logger().warn('No depth found \nPerfroming placeholder "reach for apple"')
                    self.send_move_goal(self.format_move_goal(position=[0.0, 0.0, 0.2]))
                    # Grab the Apple
                    self.grab_apple()

        except KeyboardInterrupt:
            self.get_logger().warn('Routine Aborted')


        # Return to home position
        self.send_move_goal(self.format_move_goal(position=[0.0, 0.5, 0.5], angle=[0.0, 90.0, 90.0], gripper_state=1, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE))

    
        return
    

    """
    TESTING ROUTINE
    This function can be used to test the functionality of the master node.
    """
    
    def test(self):
        self.get_logger().info('Testing')
        
        # self.send_move_goal(self.format_move_goal(position=[0.0, 0.0, 0.2]))
        # self.get_logger().info('Forward Goal Sent')
        # self.send_move_goal(self.format_move_goal(gripper_state=2))
        # self.send_move_goal(self.format_move_goal(angle=[0.0, 0.0, 100.0]))
        # self.send_move_goal(self.format_move_goal(position=[0.0, 0.0, -0.2], angle=[0.0, 0.0, -100.0], gripper_state=1))
        # self.get_logger().info('Backward Goal Sent')

        self.Go_to_Apple()

        self.send_move_goal(self.format_move_goal(position=[0.0, 0.5, 0.5], angle=[0.0, 90.0, 90.0], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE))

        self.get_logger().info('Test Complete')
        return



"""
Main Function
"""

def main(args=None):
    """
    This function initializes the master node and runs the main routine.
    Once it is complete, it cleans up by destroying the node and shutting down the rclpy system.
    """
    rclpy.init(args=args)
    node = MasterNode()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.run() # Run the main routine       
    node.get_logger().info('Destroying Master Node')
    node.arm_action_client.destroy()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
