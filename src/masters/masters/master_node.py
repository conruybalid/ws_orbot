import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from custom_interfaces.msg import ArmControl
from custom_interfaces.action import MoveArm
from custom_interfaces.srv import GetLocation

from masters.ImageProcess import processImage

from kortex_api.autogen.messages import Base_pb2

import cv2
from cv_bridge import CvBridge
import time



class MasterNode(Node):
    """
    This class represents the master node of the system.
    This node is the main node that ties the other nodes together and performs the main logic of the system.
    Currently, it completes its task and then destroys itself.

    Attributes:
        Subscriptions:
            image_sub (Subscription): A subscription to the color image topic. (arm camera)
            depth_sub (Subscription): A subscription to the depth image topic. (arm camera)
            distance_sub (Subscription): A subscription to the zed distance topic. (zed camera)

        Publishers:
            move_publisher (Publisher): A publisher for arm movement commands. (LEGACY: used arm_action_client instead)
            absolute_move_publisher (Publisher): A publisher for absolute arm movement commands. (LEGACY: use arm_absolute_action_client instead)
            Masked_publisher (Publisher): A publisher for the masked image topic.

        Action Clients:
            arm_action_client (ActionClient): An action client for the move arm action. (Relative movement)
            arm_absolute_action_client (ActionClient): An action client for the absolute move arm action.  (absolute position)

        Other Attributes:
            image_msg (Image): The latest image message received.
            depth_msg (Image): The latest depth message received.
            distance_msg (Float32): The latest distance message received.
    """
    def __init__(self):
        """
        Init fuction for the MasterNode class.
        calls the init function of the parent class (Node) and initializes the subscriptions, publishers and action clients.
        """

        super().__init__('Master_Node')
        self.image_sub = self.create_subscription(Image, 'image_topic', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, 'depth_topic', self.depth_callback, 10)
        self.zed_client = self.create_client(GetLocation, 'zed_location_service')
        while not self.zed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('zed_location_service not available, waiting again...')
        self.distance_sub = self.create_subscription(Float32, 'zed_distance_topic', self.distance_callback, 10)
        self.image_msg = None
        self.depth_msg = None
        self.distance_msg = None

        self.arm_action_client = ActionClient(self, MoveArm, 'move_arm_action')

        self.Masked_publisher = self.create_publisher(Image, 'masked_image_topic', 10)

        self.get_logger().info('Master Node Initialized')

    """
    Callback functions for the subscriptions.
    Called whenever a message is received from the respective topic.
    Assigns the received message to the respective attribute (image_msg, depth_msg, distance_msg)
    """

    def image_callback(self, msg):
        self.image_msg = msg

    def depth_callback(self, msg):
        self.depth_msg = msg

    def distance_callback(self, msg):
        self.distance_msg = msg


    def request_process_image(self, image_msg):
        """
        Process Image Service Client
        Not used in the current implementation
        Calls a service to create red mask and identify apples in the image.
        """
        self.process_request.image = image_msg
        
        self.future = self.process_client.call_async(self.process_request)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        self.future = None
        return response
    
    
    """
    ZED LOCATION SERVICE CALL
    """
    def call_zed_service(self):
        """
        Calls the zed location service to get the location of the apple in the image.
        """
        request = GetLocation.Request()

        future = self.zed_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response
    
    

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
            self.get_logger().info('Move Goal rejected :(')
            return False

        self.get_logger().info('Move Goal accepted :)')

        result_future = goal_handle.get_result_async()
        #result_future.add_done_callback(self.get_result_callback)
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'Move Result: {result.result}')

        
        return result.result
    


    """
    SEARCH FUNCTION
    This function is used to search for apples that may be out of frame
    Hopefully this will be unnecessary in the future
    """

    def Search(self):
        end_search = False

        move_msg = ArmControl()
        move_msg.position.x = 0.0
        move_msg.position.y = 0.45
        move_msg.position.z = 0.5
        move_msg.angle.x = 0.0
        move_msg.angle.y = 90.0
        move_msg.angle.z = 90.0
        move_msg.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
        move_msg.gripper_state = 0

        move_goal = MoveArm.Goal()
        move_goal.goal = move_msg


        while not end_search:
            rclpy.spin_once(self)
            image = CvBridge().imgmsg_to_cv2(self.image_msg)

            num_apples, apple_coordinates, maskedImage = processImage(image)

            mask_msg = CvBridge().cv2_to_imgmsg(cv2.multiply(maskedImage,255))
            self.Masked_publisher.publish(mask_msg)

            if num_apples <= 0:
                if move_goal.goal.position.x < 0.3:
                    self.send_move_goal(move_goal)
                    self.get_logger().info(f'Published Search Arm Movement: {move_goal.goal.position.x}, {move_goal.goal.position.y}')

                    move_goal.goal.position.x += 0.1
                
                elif move_goal.goal.position.y < 0.6:
                    self.send_move_goal(move_goal)
                    self.get_logger().info(f'Published Search Arm Movement: {move_goal.goal.position.x}, {move_goal.goal.position.y}')

                    move_goal.goal.position.y += 0.1
                    move_goal.goal.position.x = 0.0
                    

                else:
                    self.get_logger().info('No apples found')
                    end_search = True
                    self.send_move_goal(self.format_move_goal([0.0, 0.45, 0.5]))

            else:
                end_search = True
                self.get_logger().info('Apple found')

        return


    """
    CENTER APPLE FUNCTIONS
    These functions are used to center the apple in the frame.
    """

    def Go_to_Apple(self):
        self.get_logger().info('Asking Camera for Apple Location')
        point = self.call_zed_service().apple_coordinates
        x = point.x
        y = point.y
        self.distance = point.z 

        if (x == 0.0 and y == 0.0 and self.distance == 0.0):
            self.get_logger().info('No apple found')
            return False
        else:
            self.get_logger().info(f'Apple at {x}, {y}, {self.distance}')

        #Adjust coordinates so that the camera can see the apple
        if (self.send_move_goal(self.format_move_goal(position=[x, y - 0.025, self.distance - 0.2], angle=[0.0, 90.0, 90.0], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE))):
            self.get_logger().info('Moved to apple location')
        else:
            self.get_logger().info('Failed to move to apple location')
            return False

        return True

    def center_apple(self):
        """
        Will apply a mask to the image and find the apple location.
        Then it will move the arm to attempt to center the apple in the frame.
        It will continue until the apple is within a certain range of the center of the frame and return True.
        If sight of the apple is lost, the function will return False.
        """
        tolerance = 15
        apple_coordinates = None
        while (apple_coordinates == None or not ((apple_coordinates[0].x <= 640+tolerance and apple_coordinates[0].x >= 640-tolerance) and (apple_coordinates[0].y <= 420+tolerance and apple_coordinates[0].y >= 420-tolerance))):
            rclpy.spin_once(self)

            move_msg = ArmControl()

            image = CvBridge().imgmsg_to_cv2(self.image_msg)

            num_apples, apple_coordinates, maskedImage = processImage(image)
            self.get_logger().info('found %.d apples' % num_apples)

            for apple in apple_coordinates:
                self.get_logger().info('Location: %.2f, %.2f' % (apple.x, apple.y))

            mask_msg = CvBridge().cv2_to_imgmsg(cv2.multiply(maskedImage,255))
            self.Masked_publisher.publish(mask_msg)

            if num_apples == 0:
                self.get_logger().info('No apples found')

                return False


            # scale the coordinates and create message
            move_msg.position.z = 0.0
            move_msg.position.x, move_msg.position.y = self.pixel_scale(apple_coordinates[0].x, apple_coordinates[0].y)
            move_msg.angle.x = 0.0
            move_msg.angle.y = 0.0
            move_msg.angle.z = 0.0        
            move_msg.gripper_state = 0
            move_msg.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL


            move_goal = MoveArm.Goal()
            move_goal.goal = move_msg

            # Publish Arm Movement
            self.send_move_goal(move_goal)
            self.get_logger().info(f'moved {move_msg.position.x} in x, {move_msg.position.y} in y')
        
        return True
    
    
    def pixel_scale(self, x, y):
        """
        Transforms pixel coordinates to relative arm movement coordinates.
        """
        xDesire = 640; 
        xDist = xDesire - x
        xChange = 0.00025*xDist

        yDesire = 420; #was 420
        yDist = yDesire - y
        yChange = 0.00025*yDist

        return xChange, yChange
    

    
    """
    RETRIEVE APPLE FUNCTIONS
    These functions are used to reach out, grab the apple and drop it in a basket.
    """

    def reach_apple(self):
        """
        If a distance has been received from the zed camera,
        this function can be used to reach out to the apple.
        """

        self.get_logger().info('Reaching for Apple at absolute distance %f' % self.distance)
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
            self.get_logger().info('Could not reach for apple')
            return False
        

    def grab_apple(self):
        """
        Once the apple is within the gripper's reach, 
        this function can be used to grab the apple and deposit it in the basket.
        """

        try:
            # Pick Apple
            if not (
            self.send_move_goal(self.format_move_goal(position=[0.0, 0.025, 0.0], gripper_state=2, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL))
            and self.send_move_goal(self.format_move_goal(angle=[0.0, 0.0, 100.0], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL))
            and self.send_move_goal(self.format_move_goal(position=[0.0, 0.0, -0.2], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL))
            ):
                self.get_logger().info('failed to perform pick apple movements')
            else:
                self.get_logger().info('finished pick apple movements')

            
            # Drop off apple
            self.send_move_goal(self.format_move_goal(position=[0.0, 0.5, 0.5], angle=[0.0, 90.0, 90.0], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE))

            if not (
            self.send_move_goal(self.format_move_goal(position=[-0.3, 0.4, 0.0], angle=[0.0, 90.0, 180.0], gripper_state=1, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE))
            ):
                self.get_logger().info('Failed to drop off apple')
                self.send_move_goal(self.format_move_goal(gripper_state=1))
            else:
                self.get_logger().info('Dropped off apple')
                self.send_move_goal(self.format_move_goal(position=[0.0, 0.5, 0.5], angle=[0.0, 90.0, 90.0], gripper_state=0, reference_frame=Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE))


            

        except KeyboardInterrupt:
            self.get_logger().info('Abort, Opening Gripper')
            self.send_move_goal(self.format_move_goal(gripper_state=1))




        return


    """
    MAIN ROUTINE
    This is the main routine of the master node.
    """

    def run(self):
        self.get_logger().info('Master Node Routine Started')
        try:
            while True:
                while (self.image_msg is None):
                    self.get_logger().info('Waiting for images')
                    rclpy.spin_once(self)

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
                    self.get_logger().info('No depth found \nPerfroming placeholder "reach for apple"')
                    self.send_move_goal(self.format_move_goal(position=[0.0, 0.0, 0.2]))
                    # Grab the Apple
                    self.grab_apple()

        except KeyboardInterrupt:
            self.get_logger().info('Routine Aborted')
        
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
    rclpy.spin_once(node)
    node.run() # Run the main routine
    node.get_logger().info('Destroying Master Node')
    node.arm_action_client.destroy()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
