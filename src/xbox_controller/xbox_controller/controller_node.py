import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Tank
import time

from inputs import get_gamepad

def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

class XboxPublisher(Node):

    def __init__(self):
        super().__init__('xbox_publisher')
        self.publisher_ = self.create_publisher(Tank, 'move_tank_commands', 10)
        
        self.manual_control: bool = False
        self.left_speed = 0
        self.right_speed = 0

        self.get_logger().info("Xbox Publisher Node Initialized")

    def check_controller(self):
        try:
            events = get_gamepad()
        except:
            self.get_logger().warn("No controller found")
            time.sleep(15)
            return
        for event in events:
            if event.ev_type == 'Key':
                if event.code == 'BTN_SOUTH' and event.state == 1:
                    self.manual_control = not self.manual_control
            if event.ev_type == 'Absolute':
                if event.code == 'ABS_Y':
                    self.left_speed = map_value(event.state, -32768, 32767, 1000, -1000)
                        
                if event.code == 'ABS_RY':
                    self.right_speed = map_value(event.state, -32768, 32767, 1000, -1000)

        return
    
    def publish(self, L, R):
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
                node.publish(node.left_speed, node.right_speed)
            #rclpy.spin_once(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()