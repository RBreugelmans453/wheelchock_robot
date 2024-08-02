import py_trees
import py_trees_ros
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import rclpy
import std_msgs.msg as std_msgs
import time

# add behaviors for the system here



# open gripper
# close gripper
class ControlLA(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        la_grabber_topic: str='la_grabber',
        la_lifter_topic: str='la_lifter',
        direction: str='none'
    ):
        super(ControlLA, self).__init__(name=name)
        self.la_lifter_topic = la_lifter_topic
        self.la_grabber_topic = la_grabber_topic
        self.direction = direction   

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("Missing required keyword argument 'node'") from e

        self.grabber_publisher = self.node.create_publisher(std_msgs.String, self.la_grabber_topic, 10)
        self.lifter_publisher = self.node.create_publisher(std_msgs.String, self.la_lifter_topic, 10)

    def update(self):
        self.grabber_publisher.publish(std_msgs.String(data=self.direction))
        # determine how long this should be done for
        return py_trees.common.Status.SUCCESS # only if it is extended/retracted completely

    #def terminate(self, new_status: py_trees.common.Status):
        # what should happen when it terminates?

class ControlEM(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        em_topic: str='em',
        state: str='none'
    ):
        super(ControlEM, self).__init__(name=name)
        self.em_topic = em_topic
        self.state = state
    
    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("Missing required keyword argument 'node'") from e

        self.em_publisher = self.node.create_publisher(std_msgs.String, self.em_topic, 10)
    
    def update(self):
        self.em_publisher.publish(std_msgs.String(data=self.state))
        return py_trees.common.Status.SUCCESS # but is has to be on for the entire time

    #def terminate(self, new_status: py_trees.common.Status):
        # what should happen when it terminates?

class FakeDrive(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(FakeDrive, self).__init__(name)
    
    def update(self):
        #time.sleep(5)
        return py_trees.common.Status.SUCCESS



 # class DriveTowardsPoint(py_trees.behaviour.Behaviour):
    
        
# lift arm
# lower arm

# energize EM
# disable EM