#!/usr/bin/env python3
"""
controller_node.py  —  STARTER STUB. This is YOUR work to implement.

Goal: a ROS 2 node that owns the arm state, accepts a target, plans a
time-parameterized joint trajectory, and streams joint states as it executes.

Suggested interface (you may adapt, but document any changes in your README):
    - Publishes:   /joint_states   (sensor_msgs/JointState)   at a fixed rate
    - Service:     /move_to_target (your choice of srv; e.g. a Point target)
      OR Topic:    /target_pose    (geometry_msgs/PointStamped)
    - Parameters:  publish_rate_hz, control_mode, trajectory_duration, ...

Core requirements (see the task brief):
    1. Run IK on the incoming target (use PlanarArm.inverse_kinematics).
    2. Generate a smooth joint-space trajectory from the current q to the goal
       q (trapezoidal or quintic — your choice; explain it).
    3. Step along the trajectory in a timer callback and publish JointState.
    4. Respect joint limits and the ground constraint.
    5. Design the command path so a hardware backend (e.g. Dynamixel) could be
       swapped in later without rewriting the planner.

Stretch (optional, rewarded): velocity / current control modes, a PID
trajectory-tracking loop with an error signal, a ROS 2 action for the full
pick-and-place with feedback/cancel.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

# The provided kinematics library — do not modify it.
from planar_arm_control.planar_arm import PlanarArm

LINK_LENGTHS = [3.0, 2.0, 1.5]
HOME = [math.pi / 2, 0.0, 0.0]  # pointing straight up
MAX_SPEED = math.radians(60.0)  # per joint
MAX_ACCEL = math.radians(120.0)
MIN_MOVE_TIME = 0.5  # also avoids dividing by zero on a zero-length move


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.arm = PlanarArm(LINK_LENGTHS)
        rate = self.declare_parameter("publish_rate_hz", 50.0).value
        self.q = list(HOME)
        self.move = None
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(1.0 / rate, self.tick)
        self.create_subscription(PointStamped, "target_pose", self.on_target, 10)

    def on_target(self, msg):
        if self.move is not None:
            _, _, t0, duration = self.move
            if time.monotonic() - t0 < duration:
                self.get_logger().info("busy: finishing the current move, target ignored")
                return
        # Start the search from the current pose, so IK picks the closest solution.
        goal = self.arm.inverse_kinematics([msg.point.x, msg.point.y], initial_guess=self.q)
        travel = max(abs(b - a) for a, b in zip(self.q, goal))
        # The quintic peaks at 1.875 * travel / T in speed and 5.774 * travel / T^2 in acceleration.
        duration = max(1.875 * travel / MAX_SPEED, math.sqrt(5.774 * travel / MAX_ACCEL), MIN_MOVE_TIME)
        self.move = (list(self.q), goal, time.monotonic(), duration)

    def tick(self):
        if self.move is not None:
            start, goal, t0, duration = self.move
            # Fraction of the move done, from the time passed since it started.
            s = min((time.monotonic() - t0) / duration, 1.0)
            # Quintic: zero speed and acceleration at both ends, so the arm eases in and out.
            s = 10 * s**3 - 15 * s**4 + 6 * s**5
            self.q = [a + (b - a) * s for a, b in zip(start, goal)]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1", "joint2", "joint3"]
        msg.position = self.q
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
