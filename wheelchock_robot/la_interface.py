import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO
import time

from std_msgs.msg import String

#initialize the GPIO pins for the grabber la
grabber_GPIO = [1, 2] #change
lifter_GPIO = [3, 4] #change
EM_GPIO = 33

class LAInterface(Node):

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(EM_GPIO, GPIO.OUT, initial=GPIO.LOW)
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
        self.subscription = self.create_subscription(
            String,
            'em',
            self.em_callback,
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

    def em_callback(self, msg):
        self.get_logger().info('EM: "%s"' % msg.data)
        if msg.data == 'on':
            self.get_logger().info('relay on')
            GPIO.output(EM_GPIO, GPIO.HIGH)
        if msg.data == 'off':
            GPIO.output(EM_GPIO, GPIO.LOW)

def main(args=None):
    rclpy.init(args=args)

    la_interface = LAInterface()

    rclpy.spin(la_interface)

    la_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()