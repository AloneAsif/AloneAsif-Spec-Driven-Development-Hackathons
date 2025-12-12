# Brief description of what this code does
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        # Initialization code here
        self.get_logger().info('Simple node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()