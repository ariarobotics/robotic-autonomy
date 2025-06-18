import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from math import cos, sin, atan2, sqrt, pi

print('This is Turtlesim, Arnav Mehta is the GOAT')

class Spirograph(Node):
    def __init__(self):
        super().__init__('spirograph')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.declare_parameter('radius', 5.0)
        self.declare_parameter('speed', 1.3)
        self.declare_parameter('petals', 8)
        self.declare_parameter('pen_color', [255, 0, 255])

        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.set_pen_client.wait_for_service(1.0):
            self.get_logger().info("Waiting for /turtle1/set_pen...")

        self.set_pen()
        self.pose = None
        self.theta = 0.0
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)

    def pose_callback(self, msg):
        self.pose = msg

    def set_pen(self):
        color = self.get_parameter('pen_color').get_parameter_value().integer_array_value
        req = SetPen.Request()
        req.r, req.g, req.b = color
        req.width = 2
        req.off = False
        future = self.set_pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def timer_callback(self):
        if self.pose is None:
            return

        a = self.get_parameter('radius').get_parameter_value().double_value
        k = self.get_parameter('petals').get_parameter_value().integer_value
        speed = self.get_parameter('speed').get_parameter_value().double_value

        self.theta += speed * 0.02

        
        r = a * cos(k * self.theta)

        # Center 
        target_x = 5.5 + r * cos(self.theta)
        target_y = 5.5 + r * sin(self.theta)

        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = sqrt(dx**2 + dy**2)
        angle_to_target = atan2(dy, dx)
        angle_diff = angle_to_target - self.pose.theta

        # Normalize angle
        while angle_diff > pi:
            angle_diff -= 2 * pi
        while angle_diff < -pi:
            angle_diff += 2 * pi

        twist = Twist()
        twist.linear.x = 1.5 * distance
        twist.angular.z = 6.0 * angle_diff

        self.publisher.publish(twist)


def main():
    rclpy.init()
    node = Spirograph()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
