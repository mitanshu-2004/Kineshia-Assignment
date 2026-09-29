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
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState

# The provided kinematics library — do not modify it.
from planar_arm_control.planar_arm import PlanarArm
from planar_arm_msgs.msg import ArmStatus
from planar_arm_msgs.srv import MoveToTarget, PickPlace

LINK_LENGTHS = [3.0, 2.0, 1.5]
REACH = sum(LINK_LENGTHS)
HOME = [math.pi / 2, 0.0, 0.0]  # pointing straight up
MAX_SPEED = math.radians(60.0)  # per joint
MAX_ACCEL = math.radians(120.0)
MIN_MOVE_TIME = 0.5  # also avoids dividing by zero on a zero-length move
GRASP_TIME = 0.5  # pause to close or open the gripper
GOAL_TOLERANCE = 1e-3  # the IK answer must put the tool within 1 mm of the target


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.arm = PlanarArm(LINK_LENGTHS)
        rate = self.declare_parameter("publish_rate_hz", 50.0).value
        self.q = list(HOME)
        self.holding = False
        # Steps to run, as (phase, start pose, goal pose, duration); the first one is running.
        self.steps = []
        self.step_start = 0.0
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        # Transient local: a GUI started later still gets the current status.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(ArmStatus, "arm_status", latched)
        self.create_timer(1.0 / rate, self.tick)
        self.create_service(MoveToTarget, "move_to_target", self.on_move)
        self.create_service(PickPlace, "pick_place", self.on_pick_place)
        self.publish_status()

    def plan(self, x, y, start, allow_projection):
        """Check a target reached from pose `start`; return (goal pose, point used, reason refused)."""
        if y < 0:
            return None, None, f"({x:.2f}, {y:.2f}) is below the ground"
        if math.hypot(x, y) > REACH and not allow_projection:
            return None, None, f"({x:.2f}, {y:.2f}) is out of reach"
        target = self.arm.reachable_target([x, y])  # the library's projection of points beyond reach
        # Start the search from the start pose, so IK picks the closest solution.
        goal = self.arm.inverse_kinematics(target, initial_guess=start)
        # The library's fallback IK checks neither the limits nor the ground, and may miss the target.
        reached = math.dist(self.arm.end_effector(goal), target) < GOAL_TOLERANCE
        if not (reached and self.arm.within_joint_limits(goal) and self.arm.arm_above_base(goal)):
            return None, None, f"the library's IK gave no valid pose for ({x:.2f}, {y:.2f})"
        # A straight joint-space line between two valid poses can still dip below the ground.
        samples = int(math.degrees(max(abs(b - a) for a, b in zip(start, goal)))) + 2
        for i in range(samples):
            s = i / (samples - 1)
            if not self.arm.arm_above_base([a + (b - a) * s for a, b in zip(start, goal)]):
                return None, None, "the path would pass below the ground"
        return goal, target, None

    def move_step(self, start, goal):
        travel = max(abs(b - a) for a, b in zip(start, goal))
        # The quintic peaks at 1.875 * travel / T in speed and 5.774 * travel / T^2 in acceleration.
        duration = max(1.875 * travel / MAX_SPEED, math.sqrt(5.774 * travel / MAX_ACCEL), MIN_MOVE_TIME)
        return ("moving", start, goal, duration)

    def run(self, steps):
        self.steps = steps
        self.step_start = time.monotonic()
        self.publish_status()

    def on_move(self, request, response):
        if self.steps:
            response.message = "busy: finishing the current move"
            return response
        x, y = request.x, request.y
        goal, target, reason = self.plan(x, y, self.q, allow_projection=True)
        if goal is None:
            response.message = "refused: " + reason
            return response
        self.run([self.move_step(self.q, goal)])
        response.accepted = True
        response.goal_x, response.goal_y = float(target[0]), float(target[1])
        response.message = f"moving to ({target[0]:.2f}, {target[1]:.2f})"
        if math.hypot(x, y) > REACH:
            response.message = f"({x:.2f}, {y:.2f}) is out of reach, " + response.message
        return response

    def on_pick_place(self, request, response):
        if self.steps:
            response.message = "busy: finishing the current move"
            return response
        # Neither point is projected: picking at the wrong spot is worse than not picking.
        pick, _, reason = self.plan(request.pick_x, request.pick_y, self.q, allow_projection=False)
        if pick is None:
            response.message = "refused: pick " + reason
            return response
        place, _, reason = self.plan(request.place_x, request.place_y, pick, allow_projection=False)
        if place is None:
            response.message = "refused: place " + reason
            return response
        # A pick or place is a pause at one pose while the gripper closes or opens.
        self.run([self.move_step(self.q, pick), ("picking", pick, pick, GRASP_TIME),
                  self.move_step(pick, place), ("placing", place, place, GRASP_TIME)])
        response.accepted = True
        response.message = (f"pick at ({request.pick_x:.2f}, {request.pick_y:.2f}), "
                            f"place at ({request.place_x:.2f}, {request.place_y:.2f})")
        return response

    def tick(self):
        if self.steps:
            phase, start, goal, duration = self.steps[0]
            # Fraction of the step done, from the time passed since it started.
            s = min((time.monotonic() - self.step_start) / duration, 1.0)
            # Quintic: zero speed and acceleration at both ends, so the arm eases in and out.
            eased = 10 * s**3 - 15 * s**4 + 6 * s**5
            self.q = [a + (b - a) * eased for a, b in zip(start, goal)]
            if s == 1.0:
                if phase != "moving":
                    self.holding = phase == "picking"
                self.steps.pop(0)
                self.step_start += duration
                self.publish_status()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1", "joint2", "joint3"]
        msg.position = self.q
        self.joint_pub.publish(msg)

    def publish_status(self):
        self.status_pub.publish(ArmStatus(phase=self.steps[0][0] if self.steps else "idle", holding=self.holding))


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Under ros2 launch a second Ctrl-C follows the first; it must not interrupt cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
