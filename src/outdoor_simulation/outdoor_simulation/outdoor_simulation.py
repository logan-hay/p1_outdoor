import rclpy
from rclpy.node import Node

class SimulationNode(Node):
    def __init__(self):
        super().__init__('outdoor_simulation')
        self.get_logger().info('Webots simulation node is running.')

def main(args=None):
    rclpy.init(args=args)
    node = SimulationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
