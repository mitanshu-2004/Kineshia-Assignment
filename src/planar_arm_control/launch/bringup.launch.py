"""Start the controller (with its simulated arm) and the GUI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("control_mode", default_value="position"),
        LifecycleNode(
            package="planar_arm_control",
            executable="controller_node",
            name="controller_node",
            namespace="",
            autostart=True,
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
