import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('basic_cmdvel_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f'{self.get_name()} -> callback')
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 1.0
        self.get_logger().info(f'{self.get_name()} -> {t}')
        self.publisher.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
