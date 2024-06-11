import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_interfaces.action import pick_apple


class PickAppleServer(Node):

    def __init__(self):
        super().__init__('pick_apple_action_server')
        self._action_server = ActionServer(
            self,
            pick_apple,
            'pick_apple',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        



        result = pick_apple.Result()
        result.result = 'Success'
        return result


def main(args=None):
    rclpy.init(args=args)

    pick_apple_action_server = PickAppleServer()

    rclpy.spin(pick_apple_action_server)

    pick_apple_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()