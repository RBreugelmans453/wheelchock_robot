import rclpy
from rclpy.node import Node
import odrive
from odrive.enums import *
import time
import math
from geometry_msgs.msg import Twist as twist
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry as odom

speed_to_rps = 3.108 #should be 3.108, but testing lower speeds now
rotation_to_rps = 0.9 # this is a guess, should be tested
wheel_separation = 0.6
wheel_radius = 0.178

class ODriveControl(Node):

    
    def __init__(self):
        super().__init__('odrive_control')
        self.vel_subscription = self.create_subscription(
            twist,
            '/cmd_vel',
            self.odrive_control_callback,
            10)
        self.odom_publisher = self.create_publisher(odom, '/odrive/odom', 10)
        #self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('test1')
        self.odrive_left = odrive.find_any(serial_number="347536553330")
        self.odrive_right = odrive.find_any(serial_number="395335783431")
        self.get_logger().info('ODrives found')
        self.odrive_left.axis0.requested_state = AXIS_STATE_IDLE
        self.odrive_right.axis0.requested_state = AXIS_STATE_IDLE

        self.odrive_left.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.odrive_right.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        self.odrive_left.axis0.controller.config.vel_gain = 0.84836
        self.odrive_right.axis0.controller.config.vel_gain = 0.84836
        self.odrive_left.axis0.controller.config.vel_integrator_gain = 4.5
        self.odrive_right.axis0.controller.config.vel_integrator_gain = 4.5

        self.odrive_left.axis0.requested_state = AXIS_STATE_STARTUP_SEQUENCE
        self.odrive_right.axis0.requested_state = AXIS_STATE_STARTUP_SEQUENCE
        self.get_logger().info('ODrives initialized')

        self.odrive_left.axis0.config.load_encoder = ENCODER_ID_ONBOARD_ENCODER0
        self.odrive_right.axis0.config.load_encoder = ENCODER_ID_ONBOARD_ENCODER0

        self.odrive_left.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrive_right.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # initial pose
        self.x = 0
        self.y = 0
        self.th = 0

        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def odrive_control_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg)

        # to the right
        if msg.linear.x != 0 and msg.angular.z <= 0:
            self.odrive_left.axis0.controller.input_vel = (msg.linear.x - msg.angular.z * rotation_to_rps * wheel_separation / 2) * -speed_to_rps
            self.odrive_right.axis0.controller.input_vel = (msg.linear.x + msg.angular.z * rotation_to_rps * wheel_separation / 2) * speed_to_rps

        # to the left
        if msg.linear.x != 0 and msg.angular.z >= 0:
            self.odrive_left.axis0.controller.input_vel = (msg.linear.x - msg.angular.z * rotation_to_rps * wheel_separation / 2) * -speed_to_rps
            self.odrive_right.axis0.controller.input_vel = (msg.linear.x + msg.angular.z * rotation_to_rps * wheel_separation / 2) * speed_to_rps

        if msg.linear.x == 0 and msg.angular.z != 0:
            self.odrive_left.axis0.controller.input_vel = (msg.linear.x - msg.angular.z * wheel_separation / 2) * -speed_to_rps
            self.odrive_right.axis0.controller.input_vel = (msg.linear.x + msg.angular.z * wheel_separation / 2) * speed_to_rps

        #possibly add the torque boost here

        else:
            self.odrive_left.axis0.controller.input_vel = (msg.linear.x - msg.angular.z * wheel_separation / 2) * -speed_to_rps
            self.odrive_right.axis0.controller.input_vel = (msg.linear.x + msg.angular.z * wheel_separation / 2) * speed_to_rps

    def publish_odom(self, x, y, z, qx, qy, qz, qw, vel_x, ang_z):
        msg = odom()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = vel_x
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = ang_z
        self.odom_publisher.publish(msg)
    
    def timer_callback(self):
        # get velocity (rev/s) from encoders
        vel_rps_left = self.odrive_left.axis0.vel_estimate / -speed_to_rps
        vel_rps_right = self.odrive_right.axis0.vel_estimate / speed_to_rps
        #torque_left = self.odrive_left.motor.torque_estimate
        #torque_right = self.odrive_right.motor.torque_estimate

        #self.get_logger().info('Left: %f, Right: %f' % (torque_left, torque_right))

        # convert to m/s
        vel_left = vel_rps_left * wheel_radius * 2 * math.pi
        vel_right = vel_rps_right * wheel_radius * 2 * math.pi

        # calculate linear and angular velocity
        linear_vel = (vel_left + vel_right) / 2
        angular_vel = (vel_right - vel_left) / wheel_separation

        # calculate the time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # calculate the new pose
        self.x += linear_vel * math.cos(self.th) * dt
        self.y += linear_vel * math.sin(self.th) * dt
        self.th += angular_vel * dt

        self.last_time = current_time

        # publish the new pose to /odom
        self.publish_odom(self.x, self.y, 0.0, 0.0, 0.0, math.sin(self.th / 2), math.cos(self.th / 2), linear_vel, angular_vel)

        #transform = TransformStamped()
        #transform.header.stamp = current_time.to_msg()
        #transform.header.frame_id = 'odom'
        #transform.child_frame_id = 'base_link'
        #transform.transform.translation.x = self.x
        #transform.transform.translation.y = self.y
        #transform.transform.translation.z = 0.0
        #transform.transform.rotation.x = 0.0
        #transform.transform.rotation.y = 0.0
        #transform.transform.rotation.z = math.sin(self.th / 2)
        #transform.transform.rotation.w = math.cos(self.th / 2)

        #self.tf_broadcaster.sendTransform(transform)
        


    def set_idle(self):
        try:
            self.odrive_left.axis0.requested_state = AXIS_STATE_IDLE
            self.odrive_right.axis0.requested_state = AXIS_STATE_IDLE
        except:
            self.get_logger().info('ODrives not set to idle')

    
def main(args=None):
    rclpy.init(args=args)
    odrive_controller = ODriveControl()
    
    try:
        rclpy.spin(odrive_controller)
    except KeyboardInterrupt:
        pass

    # Clean up and shut down
    odrive_controller.set_idle()
    odrive_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


