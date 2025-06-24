import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Circle(Node):
    radius = 2.0

    def __init__(self):
        super().__init__("publisher_")
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.radius
        msg.angular.z = 1.0
        self.publisher_.publish(msg)

        if self.i % 50 == 0:
            print('publishing...')

        self.i += 1


def main():
    rclpy.init()

    my_circle = Circle()
    rclpy.spin(my_circle)

    my_circle.destroy_node()
    rclpy.shutdown()