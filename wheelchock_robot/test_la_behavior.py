import operator
import sys

import launch
import launch_ros
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import py_trees_ros_interfaces as py_trees_actions
import std_msgs.msg as std_msgs
import rclpy

from . import behaviors

# generate launch description below

# create the root

def la_behavior_create_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Parallel(
        name="Drive and drop",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    topics_2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    driven_check_2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Driven2BB",
        topic_name="/checks/driven",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="event_driven_button"
    )
    placed_check_2bb = py_trees_ros.subscribers.ToBlackboard(
        name="Placed2BB",
        topic_name="/checks/placed",
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={'event_placed_button': 'data'},
        clearing_policy=3
    )
    tasks = py_trees.composites.Sequence(name="Tasks", memory=False)
    drive_towards_point = py_trees.composites.Selector(name="Drive towards point", memory=False)
    drive_checker = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Driven?",
        check=py_trees.common.ComparisonExpression(
            variable="event_driven_button",
            value=True,
            operator=operator.eq
        )
    )
    driving_to_point = behaviors.FakeDrive(name="Drive")
    place_chocks = py_trees.composites.Selector(name="Place chocks", memory=True)
    #place_checker = py_trees.behaviours.WaitForBlackboardVariable(
     #   name="Placed?",
      #  variable_name="event_placed_button",        
    #)
    place_checker = behaviors.CheckForPlacement(name="Placed?")
    place_chocks_em = behaviors.ControlEM(name="EM", state="on") # this should be off, but for testing it is on now
    root.add_children([topics_2bb, tasks])
    topics_2bb.add_children([driven_check_2bb, placed_check_2bb])
    tasks.add_children([drive_towards_point, place_chocks])
    drive_towards_point.add_children([drive_checker, driving_to_point])
    place_chocks.add_children([place_checker, place_chocks_em])
    return root

def main():
    rclpy.init()
    root = la_behavior_create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )

    try:
        tree.setup(timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=100.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()