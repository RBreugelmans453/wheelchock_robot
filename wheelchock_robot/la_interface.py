import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO
import time

from std_msgs.msg import String

#initialize the GPIO pins for the grabber la
grabber_GPIO = [1, 2] #change
lifter_GPIO = [3, 4] #change

class LAInterface(Node):

    def __init__(self):
        super().__init__('la_interface')
        self.subscription = self.create_subscription(
            String,
            'la_grabber',
            self.la_grabber_callback,
            10)
        self.subscription = self.create_subscription(
            String,
            'la_lifter',
            self.la_lifter_callback,
            10)
        
    def la_grabber_callback(self, msg):
        self.get_logger().info('Grabber LA: "%s"' % msg.data)
        if msg == 'extent':
            GPIO.output(grabber_GPIO, GPIO.HIGH)
            #should this also be set to LOW afterwards? since it will draw some current from the jetson
            time.sleep(5) #time depends on how long it takes to completely extent/retract the LAs

        if msg == 'retract':
             GPIO.output(grabber_GPIO, GPIO.LOW)

    def la_lifter_callback(self, msg):
        self.get_logger().info('Lifter LA: "%s"' % msg.data)
        if msg == 'extent':
            GPIO.output(grabber_GPIO, GPIO.HIGH)
            
            #should this also be set to LOW afterwards? since it will draw some current from the jetson
        
        if msg == 'retract':
             GPIO.output(grabber_GPIO, GPIO.LOW)
