"""Spawn the planar arm in Gazebo Sim and drive it with the same controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    share = get_package_share_directory("planar_arm_gazebo")
    model = os.path.join(share, "models", "planar_arm.sdf")
    bridge = os.path.join(share, "config", "gazebo_bridge.yaml")
    gz_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={"gz_args": "-r empty.sdf"}.items(),
        ),
        TimerAction(period=3.0, actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_planar_arm",
                output="screen",
                arguments=["-name", "planar_arm", "-file", model],
            ),
        ]),
        TimerAction(period=4.0, actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gazebo_bridge",
                parameters=[{"config_file": bridge}],
                output="screen",
            ),
        ]),
        TimerAction(period=5.0, actions=[
            LifecycleNode(
                package="planar_arm_gazebo",
                executable="gazebo_controller_node",
                name="controller_node",
                namespace="",
                autostart=True,
                output="screen",
                parameters=[{"control_mode": "position", "joint_tolerance": 0.02}],
            ),
            Node(
                package="planar_arm_control",
                executable="gui_node",
                name="gui_node",
                output="screen",
            ),
        ]),
    ])
