import operator
import sys

import launch
import launch_ros
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import py_trees_ros_interfaces as py_trees_actions
import rclpy

from . import behaviors

# generate launch description below

# create the root

def la_behavior_create_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Parallel(
        name="Drive and drop",
        memory=True
    )
    topics_2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    driven_check_2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Driven2BB",
        topic_name="/checks/driven",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="event_scan_button"
    )
    placed_check_2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Placed2BB",
        topic_name="/checks/placed",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="event_placed_button"
    )
    retract_la = behaviors
    tasks = py_trees.composites.Sequence(name="Tasks", memory=False)
    drive_towards_point = py_trees.composites.Selector(name="Drive towards point", memory=False)
    drive_checker = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Driven?",
        check=py_trees.common.ComparisonExpression(
            variable="event_scan_button",
            value=True,
            operator=operator.eq
        )
    driving_to_point = py
    )