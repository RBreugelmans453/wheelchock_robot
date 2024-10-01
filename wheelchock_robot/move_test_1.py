import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class SquarePublisher(Node):
    def __init__(self):
        super().__init__('square_publisher')
        self.od = 0.1 # seconds
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback0)
        self.i = 0

    # wait a bit
    def timer_callback0(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)
        self.i += 1
        global rotating
        rotating = False
        if self.i == 20:
            self.i = 0
            self.timer.cancel()
            self.timer = self.create_timer(0.1, self.timer_callback)

    # drive forward
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)
        self.i += 1
        global rotating
        rotating = False
        if self.i == 2:
            self.i = 0
            self.timer.cancel()
            self.timer = self.create_timer(0.1, self.timer_callback2)
    
    # rotate to the right
    def timer_callback2(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = -1.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.angular.z)
        self.i += 1
        global rotating 
        rotating = True
        if self.i == 20:
            self.i = 0
            self.timer.cancel()
            self.timer = self.create_timer(0.1, self.timer_callback3)

    # stop
    def timer_callback3(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)

def main(args=None):
    rclpy.init(args=args)
    square_publisher = SquarePublisher()
    rclpy.spin(square_publisher)
    square_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()