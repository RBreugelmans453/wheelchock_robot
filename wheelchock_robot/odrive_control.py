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

# set the vehicle parameter variables
speed_to_rps = 3.108        # conversion from linear velocity (m/s) to rotations per second
rotation_to_rps = 0.9       # multiplier to increase/decrease the rotational speed during a differential turn (increase to decrease rotational speed)
wheel_separation = 0.6      # wheel separation in meters
wheel_radius = 0.178        # wheel radius in meters

class ODriveControl(Node):

    
    def __init__(self):
        super().__init__('odrive_control')

        # Set up subscriber for the /cmd_vel topic, to listen for velocity commands
        self.vel_subscription = self.create_subscription(
            twist,
            '/cmd_vel',
            self.odrive_control_callback,
            10)
        
        # Set up the odometry publisher
        self.odom_publisher = self.create_publisher(odom, '/odrive/odom', 10)

        #self.tf_broadcaster = TransformBroadcaster(self)                   # unconmment if you want to broadcast tf from here, instead of using the ekf filter 
        
        # Find the right Odrive motor controller en link them to the right side
        self.odrive_left = odrive.find_any(serial_number="347536553330")
        self.odrive_right = odrive.find_any(serial_number="395335783431")
        self.get_logger().info('ODrives found')

        # Set the state of the motors to IDLE
        self.odrive_left.axis0.requested_state = AXIS_STATE_IDLE
        self.odrive_right.axis0.requested_state = AXIS_STATE_IDLE

        # Set the mode of the motors to velocity control, to be able to send it velocity commands in rps
        self.odrive_left.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.odrive_right.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        # Set the velocity gain and velocity integrator gain of the motor controllers, this could be tuned again to work better on tarmac or concrete
        self.odrive_left.axis0.controller.config.vel_gain = 0.84836
        self.odrive_right.axis0.controller.config.vel_gain = 0.84836
        self.odrive_left.axis0.controller.config.vel_integrator_gain = 4.5
        self.odrive_right.axis0.controller.config.vel_integrator_gain = 4.5

        # Set the state of the motors to startup
        self.odrive_left.axis0.requested_state = AXIS_STATE_STARTUP_SEQUENCE
        self.odrive_right.axis0.requested_state = AXIS_STATE_STARTUP_SEQUENCE
        self.get_logger().info('ODrives initialized')

        # Set the motor controllers to use the onboard encoder
        self.odrive_left.axis0.config.load_encoder = ENCODER_ID_ONBOARD_ENCODER0
        self.odrive_right.axis0.config.load_encoder = ENCODER_ID_ONBOARD_ENCODER0

        # Set the state of the motors to closed loop control (they can now accept commands)
        self.odrive_left.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrive_right.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # initial pose, used by the tf broadcaster
        self.x = 0
        self.y = 0
        self.th = 0

        self.last_time = self.get_clock().now()

        # Set a timer callback for reading the encoders every 0.1 second
        self.timer = self.create_timer(0.1, self.timer_callback)

    # This is called every time a velocity command is received on the /cmd_vel topic
    def odrive_control_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg)

        # The left odrive command is reversed because this motor is facing the opposite direction
        # Calculate the right motor velocity commands for doing a differential turn to the right
        if msg.linear.x != 0 and msg.angular.z <= 0:
            self.odrive_left.axis0.controller.input_vel = (msg.linear.x - msg.angular.z * rotation_to_rps * wheel_separation / 2) * -speed_to_rps
            self.odrive_right.axis0.controller.input_vel = (msg.linear.x + msg.angular.z * rotation_to_rps * wheel_separation / 2) * speed_to_rps

        # Calculate the right motor velocity commands for doing a differential turn to the left
        if msg.linear.x != 0 and msg.angular.z >= 0:
            self.odrive_left.axis0.controller.input_vel = (msg.linear.x - msg.angular.z * rotation_to_rps * wheel_separation / 2) * -speed_to_rps
            self.odrive_right.axis0.controller.input_vel = (msg.linear.x + msg.angular.z * rotation_to_rps * wheel_separation / 2) * speed_to_rps

        # Calculate the right motor velocity commands for doing on the spot rotation
        if msg.linear.x == 0 and msg.angular.z != 0:
            self.odrive_left.axis0.controller.input_vel = (msg.linear.x - msg.angular.z * wheel_separation / 2) * -speed_to_rps
            self.odrive_right.axis0.controller.input_vel = (msg.linear.x + msg.angular.z * wheel_separation / 2) * speed_to_rps

        # Calculate the right motor velocity commands for going straight
        else:
            self.odrive_left.axis0.controller.input_vel = (msg.linear.x - msg.angular.z * wheel_separation / 2) * -speed_to_rps
            self.odrive_right.axis0.controller.input_vel = (msg.linear.x + msg.angular.z * wheel_separation / 2) * speed_to_rps

    # Fill an odometry message with the right positional values (x, y, z) and rotational values (quaternion x, y, z, w)
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
    
    # This is called every 0.1 seconds to read the encoders and convert that speed to the linear speed of the robot
    def timer_callback(self):
        # Get velocity (rev/s) from encoders
        vel_rps_left = self.odrive_left.axis0.vel_estimate / -speed_to_rps
        vel_rps_right = self.odrive_right.axis0.vel_estimate / speed_to_rps

        # Convert to m/s
        vel_left = vel_rps_left * wheel_radius * 2 * math.pi
        vel_right = vel_rps_right * wheel_radius * 2 * math.pi

        # Calculate linear and angular velocity
        linear_vel = (vel_left + vel_right) / 2
        angular_vel = (vel_right - vel_left) / wheel_separation

        # Calculate the time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Calculate the new pose
        self.x += linear_vel * math.cos(self.th) * dt
        self.y += linear_vel * math.sin(self.th) * dt
        self.th += angular_vel * dt

        self.last_time = current_time

        # Publish the new pose to /odrive/odom
        self.publish_odom(self.x, self.y, 0.0, 0.0, 0.0, math.sin(self.th / 2), math.cos(self.th / 2), linear_vel, angular_vel)

        # Uncomment if you want to publish to /tf here, instead of using the ekf
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
        

    # Set the motor state to idle if the node is closed off
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


