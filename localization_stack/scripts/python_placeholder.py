import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PlaceholderNode(Node):

    def __init__(self):
        super().__init__('placeholder_node')
        self.subscription = self.create_subscription(
            String,
            '/input',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    placeholder_node = PlaceholderNode()

    rclpy.spin(placeholder_node)

    placeholder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()