import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from custom_interfaces.msg import ArmControl
from custom_interfaces.action import MoveArm

from masters.ImageProcess import processImage

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
        self.distance_sub = self.create_subscription(Float32, 'zed_distance_topic', self.distance_callback, 10)
        self.image_msg = None
        self.depth_msg = None
        self.distance_msg = None

        self.move_publisher = self.create_publisher(ArmControl, 'arm_move', 10)
        self.absolute_move_publisher = self.create_publisher(ArmControl, 'absolute_arm_move', 10)

        self.arm_action_client = ActionClient(self, MoveArm, 'move_arm_action')
        self.arm_absolute_action_client = ActionClient(self, MoveArm, 'absolute_move_arm_action')
        

        self.Masked_publisher = self.create_publisher(Image, 'masked_image_topic', 10)

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
    

    def publish_arm_movement(self, position, wrist_angle, gripper_state):
        """
        LEGACY
        Used to publish arm movement commands to the arm_move topic.
        """
        msg = ArmControl()
        if isinstance(position, list):
            msg.position.x = position[0]
            msg.position.y = position[1]
            msg.position.z = position[2]
        else:
            self.get_logger().error('Invalid position type')
            return
        msg.wrist_angle = float(wrist_angle)
        msg.gripper_state = gripper_state
        self.move_publisher.publish(msg)
        self.get_logger().info('Published Arm Movement')



    """
    ARM MOVEMENT FUNCTIONS
    These Functions are used to send arm movement goals to the arm action server.
    """

    def format_move_goal(self, position, wrist_angle, gripper_state):
        """
        Quickly formates a goal that can be sent to the arm action server.
        """
        goal_msg = MoveArm.Goal()
        goal_msg.goal.position.x = position[0]
        goal_msg.goal.position.y = position[1]
        goal_msg.goal.position.z = position[2]
        goal_msg.goal.wrist_angle = float(wrist_angle)
        goal_msg.goal.gripper_state = gripper_state

        return goal_msg
    
    def send_move_goal(self, goal_msg):
        """
        Sends a relative arm movement goal to the arm action server.
        Waits for the server to respond and returns the result.
        """

        self.arm_action_client.wait_for_server()
        future = self.arm_action_client.send_goal_async(goal_msg)
        #future.add_done_callback(self.goal_response_callback)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return False

        self.get_logger().info('Goal accepted :)')

        result_future = goal_handle.get_result_async()
        #result_future.add_done_callback(self.get_result_callback)
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'Move Result: {result.result}')

        
        return result.result

    def send_absolute_move_goal(self, goal_msg):
        """
        Sends an absolute arm movement goal to the arm action server.
        Waits for the server to respond and returns the result.
        """

        self.arm_absolute_action_client.wait_for_server()
        future = self.arm_absolute_action_client.send_goal_async(goal_msg)
        #future.add_done_callback(self.goal_response_callback)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return False

        self.get_logger().info(f'Goal accepted :) {goal_msg.goal.position.x}, {goal_msg.goal.position.y}, {goal_msg.goal.position.z}')

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
        move_msg.position.x = 0.5
        move_msg.position.y = 0.0
        move_msg.position.z = 0.35
        move_msg.gripper_state = 0
        move_msg.wrist_angle = 0.0

        move_goal = MoveArm.Goal()
        move_goal.goal = move_msg


        while not end_search:
            rclpy.spin_once(self)
            image = CvBridge().imgmsg_to_cv2(self.image_msg)

            num_apples, apple_coordinates, maskedImage = processImage(image)

            mask_msg = CvBridge().cv2_to_imgmsg(cv2.multiply(maskedImage,255))
            self.Masked_publisher.publish(mask_msg)

            if num_apples <= 0:
                if move_goal.goal.position.y < 0.3:
                    self.send_absolute_move_goal(move_goal)
                    self.get_logger().info(f'Published Search Arm Movement: {move_goal.goal.position.y}, {move_goal.goal.position.z}')

                    move_goal.goal.position.y += 0.1
                
                elif move_goal.goal.position.z < 0.6:
                    self.send_absolute_move_goal(move_goal)
                    self.get_logger().info(f'Published Search Arm Movement: {move_goal.goal.position.y}, {move_goal.goal.position.z}')

                    move_goal.goal.position.z += 0.1
                    move_goal.goal.position.y = 0.0
                    

                else:
                    self.get_logger().info('No apples found')
                    end_search = True
                    self.send_absolute_move_goal(self.format_move_goal([0.5, 0.0, 0.45], 0.0, 0))

            else:
                end_search = True
                self.get_logger().info('Apple found')

        return


    """
    CENTER APPLE FUNCTIONS
    These functions are used to center the apple in the frame.
    """

    def center_apple(self):
        """
        Will apply a mask to the image and find the apple location.
        Then it will move the arm to attempt to center the apple in the frame.
        It will continue until the apple is within a certain range of the center of the frame and return True.
        If sight of the apple is lost, the function will return False.
        """
        apple_coordinates = None
        while (apple_coordinates == None or not ((apple_coordinates[0].x <= 650 and apple_coordinates[0].x >= 630) and (apple_coordinates[0].y <= 430 and apple_coordinates[0].y >= 410))):
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
            move_msg.position.x = 0.0
            move_msg.position.y, move_msg.position.z = self.pixel_scale(apple_coordinates[0].x, apple_coordinates[0].y)
            move_msg.gripper_state = 0
            move_msg.wrist_angle = 0.0

            move_goal = MoveArm.Goal()
            move_goal.goal = move_msg

            # Publish Arm Movement
            self.send_move_goal(move_goal)
            self.get_logger().info(f'moved {move_msg.position.y} in y, {move_msg.position.z} in z')
        
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

        self.get_logger().info('found apple at absolute distance %f' % self.distance_msg.data)
        arm_msg = ArmControl()
        arm_msg.position.x = self.distance_msg.data
        arm_msg.position.y = 99.0
        arm_msg.position.z = 99.0
        arm_msg.gripper_state = 1
        arm_msg.wrist_angle = 0.0

        move_goal = MoveArm.Goal()
        move_goal.goal = arm_msg

        if self.send_absolute_move_goal(move_goal):
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
            self.send_move_goal(self.format_move_goal([0.0, 0.0, 0.025], 0.0, 2))
            and self.send_move_goal(self.format_move_goal([0.0, 0.0, 0.0], 180.0, 0))
            and self.send_move_goal(self.format_move_goal([-0.2, 0.0, 0.0], 0.0, 0))
            and self.send_move_goal(self.format_move_goal([0.0, 0.0, 0.0], 0.0, 0))
            ):
                self.get_logger().info('failed to perform pick apple movements')
            else:
                self.get_logger().info('finished pick apple movements')

            
            # Drop off apple
            self.send_absolute_move_goal(self.format_move_goal([0.5, 0.0, 0.4], 0.0, 0))
            if not (
            self.send_absolute_move_goal(self.format_move_goal([0.5, -0.28, 0.4], 0.0, 0))
            and self.send_absolute_move_goal(self.format_move_goal([0.0, -0.28, 0.4], 0.0, 0))
            ):
                self.get_logger().info('Failed to drop off apple')
                self.send_move_goal(self.format_move_goal([0.0, 0.0, 0.0], 0.0, 1))
            else:
                self.send_move_goal(self.format_move_goal([0.0, 0.0, 0.0], 0.0, 1))
                self.get_logger().info('Dropped off apple')
                self.send_absolute_move_goal(self.format_move_goal([0.5, -0.28, 0.4], 0.0, 0))
                self.send_absolute_move_goal(self.format_move_goal([0.5, 0.0, 0.5], 0.0, 0))


            

        except KeyboardInterrupt:
            self.get_logger().info('Abort, Opening Gripper')
            self.send_move_goal(self.format_move_goal([0.0, 0.0, 0.0], 0.0, 1))




        return


    """
    MAIN ROUTINE
    This is the main routine of the master node.
    """

    def run(self):
        self.get_logger().info('Master Node Routine Started')
        
        while (self.image_msg is None):
            self.get_logger().info('Waiting for images')
            rclpy.spin_once(self)

        self.Search()

        # Center the apple
        self.get_logger().info('Centering Apple')
        if not self.center_apple():
            return
        
        # Reach for the Apple
        if self.distance_msg is not None:
            if self.reach_apple():
                # Grab the Apple
                self.grab_apple()


        else:
            self.get_logger().info('No depth found \nPerfroming placeholder "reach for apple"')
            self.send_move_goal(self.format_move_goal([0.2, 0.0, 0.0], 1, 0))
            # Grab the Apple
            self.grab_apple()

        
    
        return
    

    """
    TESTING ROUTINE
    This function can be used to test the functionality of the master node.
    """
    
    def test(self):
        self.get_logger().info('Testing')
        
        self.send_move_goal(self.format_move_goal([0.2, 0.0, 0.0], 0, 0))
        self.get_logger().info('Forward Goal Sent')
        self.send_move_goal(self.format_move_goal([-0.2, 0.0, 0.0], 0, 0))
        self.get_logger().info('Backward Goal Sent')

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
    node.run() # Run the main routine
    node.get_logger().info('Destroying Master Node')
    node.action_client.destroy()
    node.absolute_action_client.destroy()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
