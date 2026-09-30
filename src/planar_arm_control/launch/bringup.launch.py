"""
bringup.launch.py  —  STARTER STUB.

Launch the controller node and the GUI node together. Fill in the nodes once
you have implemented them, and expose any parameters you add (publish rate,
control mode, trajectory duration, etc.) here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("control_mode", default_value="position"),
        Node(
            package="planar_arm_control",
            executable="controller_node",
            name="controller_node",
            output="screen",
            parameters=[{"control_mode": LaunchConfiguration("control_mode")}],
        ),
        Node(
            package="planar_arm_control",
            executable="gui_node",
            name="gui_node",
            output="screen",
        ),
    ])
