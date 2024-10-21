import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO
import time

from std_msgs.msg import String

# Set the board pin numbers for the grabber LA
#grabber_GPIO_IN3 = xxx
#grabber_GPIO_IN4 = xxx

# Set the board pin numbers for the lifter motor controller
lifter_GPIO_REN = 33
lifter_GPIO_LEN = 32
lifter_GPIO_RPWM = 29
lifter_GPIO_LPWM = 31

# Set the board pin numbers for the EM relay
EM_GPIO = 15
lifter_count = 0

class LAInterface(Node):

    GPIO.setmode(GPIO.BOARD)
    # GPIO.setup(grabber_GPIO_IN3, GPIO.OUT, initial=GPIO.LOW)

    # Set the GPIO pin to output for the EM relay
    GPIO.setup(EM_GPIO, GPIO.OUT, initial=GPIO.LOW)

    # Set the GPIO pins to output for the lifter motor controller
    GPIO.setup(lifter_GPIO_REN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(lifter_GPIO_LEN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(lifter_GPIO_RPWM, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(lifter_GPIO_LPWM, GPIO.OUT, initial=GPIO.LOW)



    def __init__(self):
        super().__init__('la_interface')

        # Create a subscription for the /la_lifter topic, to listen to commands published for the lifting and lowering of the grabber
        self.subscription = self.create_subscription(
            String,
            'la_lifter',
            self.la_lifter_callback,
            10)

        # Create a subscription for the /em topic, to listen for commands published for activating or deactivating the EM
        self.subscription = self.create_subscription(
            String,
            'em',
            self.em_callback,
            10)
    
    # Handle the commands for the lifter LAs
    def la_lifter_callback(self, msg):
        global lifter_count                             # This is to make sure that the lifter LAs do not extent completely
        if msg.data == 'down' and lifter_count >= 0:
            self.get_logger().info('Lifter LA: "down"')
            GPIO.output(lifter_GPIO_REN, GPIO.HIGH)
            GPIO.output(lifter_GPIO_LEN, GPIO.HIGH)
            GPIO.output(lifter_GPIO_RPWM, GPIO.HIGH)
            GPIO.output(lifter_GPIO_LPWM, GPIO.LOW)
            lifter_count -= 1

        if msg.data == 'up' and lifter_count <= 3:      # Increase/decrease these values to make the LAs extent longer or shorter
            self.get_logger().info('Lifter LA: "up"')
            GPIO.output(lifter_GPIO_REN, GPIO.HIGH)
            GPIO.output(lifter_GPIO_LEN, GPIO.HIGH)
            GPIO.output(lifter_GPIO_RPWM, GPIO.LOW)
            GPIO.output(lifter_GPIO_LPWM, GPIO.HIGH)
            lifter_count += 1
        
        if msg.data == 'up' and lifter_count >=3:
            self.get_logger().info('Lifter LA: "stop"')
            GPIO.output(lifter_GPIO_REN, GPIO.HIGH)
            GPIO.output(lifter_GPIO_LEN, GPIO.HIGH)
            GPIO.output(lifter_GPIO_RPWM, GPIO.LOW)
            GPIO.output(lifter_GPIO_LPWM, GPIO.LOW)

    # Handle the commands for the EMs        
    def em_callback(self, msg):
        self.get_logger().info('EM: "%s"' % msg.data)
        if msg.data == 'on':
            self.get_logger().info('relay on')
            GPIO.output(EM_GPIO, GPIO.HIGH)
        if msg.data == 'off':
            GPIO.output(EM_GPIO, GPIO.LOW)

"""    
    def la_grabber_callback(self, msg):
        if msg.data == 'retract':
            self.get_logger().info('Grabber LA: "retracting"')
            GPIO.output(grabber_GPIO_IN3, GPIO.HIGH)
            GPIO.output(grabber_GPIO_IN4, GPIO.LOW)

            #should this also be set to LOW afterwards? since it will draw some current from the jetson
            #time.sleep(5) #time depends on how long it takes to completely extent/retract the LAs

        if msg.data == 'extent':
            self.get_logger().info('Grabber LA: "extending"')
            GPIO.output(grabber_GPIO_IN3, GPIO.LOW)
            GPIO.output(grabber_GPIO_IN4, GPIO.HIGH)
"""




def main(args=None):
    rclpy.init(args=args)

    la_interface = LAInterface()

    rclpy.spin(la_interface)

    la_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()