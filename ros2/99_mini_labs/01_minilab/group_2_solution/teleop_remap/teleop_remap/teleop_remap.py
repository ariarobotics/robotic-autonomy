import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class RemapperNode(Node):

    def __init__(self):
        super().__init__('remapper_node')
        self.subscription = self.create_subscription(Twist, '/turtle1/teleop_raw', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('invert_angular', False)

    def listener_callback(self, msg):
        cmd_msg = msg
        cmd_msg.linear.x *= self.get_parameter('linear_scale').value
        if self.get_parameter('invert_angular').value:
            cmd_msg.angular.z *= -1.0
        self.publisher.publish(cmd_msg)
        self.get_logger().info('Publishing: "%s"' % cmd_msg)

def main(args=None):
    rclpy.init(args=args)

    remapper_node = RemapperNode()

    rclpy.spin(remapper_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    remapper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
