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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from sensor_msgs.msg import JointState

# The provided kinematics library — do not modify it.
from planar_arm_control.planar_arm import PlanarArm
from planar_arm_msgs.action import PickPlace as PickPlaceAction
from planar_arm_msgs.msg import ArmStatus
from planar_arm_msgs.srv import MoveToTarget

LINK_LENGTHS = [3.0, 2.0, 1.5]
REACH = sum(LINK_LENGTHS)
HOME = [math.pi / 2, 0.0, 0.0]  # pointing straight up
MAX_SPEED = math.radians(60.0)  # per joint
MAX_ACCEL = math.radians(120.0)
MIN_MOVE_TIME = 0.5  # also avoids dividing by zero on a zero-length move
GRASP_TIME = 0.5  # pause to close or open the gripper
GOAL_TOLERANCE = 1e-3  # the IK answer must put the tool within 1 mm of the target
JOINT_TOLERANCE = 1e-3  # measured pose must reach the commanded goal before the next move
VELOCITY_GAIN = 8.0  # position correction in velocity mode, 1/s
JITTER_SAMPLE_S = 10.0


class SimArm:
    def __init__(self, q, mode="position", period=0.02):
        self.q = list(q)
        self.mode = mode
        self.period = period
        self.velocity = [0.0] * len(q)
        self.last_update = time.monotonic()

    def advance(self):
        now = time.monotonic()
        if self.mode == "velocity":
            dt = now - self.last_update
            # A missed timer must not keep an old velocity running beyond the checked period.
            self.q = [q + v * min(dt, self.period) for q, v in zip(self.q, self.velocity)]
            if dt > self.period:
                self.velocity = [0.0] * len(self.q)
        self.last_update = now

    def command(self, values):
        self.advance()
        if self.mode == "velocity":
            self.velocity = list(values)
        else:
            self.q = list(values)

    def read(self):
        self.advance()
        return list(self.q)


class ControllerNode(LifecycleNode):
    def __init__(self, backend=None, backend_factory=None):
        super().__init__("controller_node")
        self.arm = PlanarArm(LINK_LENGTHS)
        rate = self.declare_parameter("publish_rate_hz", 50.0).value
        self.mode = self.declare_parameter("control_mode", "position").value
        if self.mode not in ("position", "velocity"):
            raise ValueError("control_mode must be 'position' or 'velocity'")
        if rate <= 0:
            raise ValueError("publish_rate_hz must be positive")
        self.period = 1.0 / rate
        self.joint_tol = self.declare_parameter("joint_tolerance", JOINT_TOLERANCE).value
        if self.joint_tol <= 0:
            raise ValueError("joint_tolerance must be positive")
        self.backend = backend if backend is not None else (
            backend_factory(self) if backend_factory is not None else SimArm(HOME, self.mode, self.period))
        self.q = self.backend.read()
        self.holding = False
        # Steps to run, as (phase, start pose, goal pose, duration); the first one is running.
        self.steps = []
        self.step_start = 0.0
        self.action_goal = None
        self.action_done = None
        self.joint_pub = None
        self.status_pub = None
        self.timer = None
        self.create_service(MoveToTarget, "move_to_target", self.on_move)
        self.action_server = ActionServer(self, PickPlaceAction, "pick_place", self.execute_pick_place,
                                          goal_callback=self.on_goal, cancel_callback=self.on_cancel,
                                          callback_group=ReentrantCallbackGroup())

    def on_configure(self, state):
        self.joint_pub = self.create_lifecycle_publisher(JointState, "joint_states", 10)
        # Transient local: a GUI started later still gets the current status.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_lifecycle_publisher(ArmStatus, "arm_status", latched)
        return super().on_configure(state)

    def on_activate(self, state):
        result = super().on_activate(state)
        if result != TransitionCallbackReturn.SUCCESS:
            return result
        self.q = self.backend.read()
        self.last_tick = self.jitter_start = time.monotonic()
        self.jitter_count = 0
        self.jitter_total = self.jitter_max = 0.0
        self.timer = self.create_timer(self.period, self.tick)
        self.publish_status()
        return TransitionCallbackReturn.SUCCESS

    def stop(self):
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        if self.mode == "velocity":
            self.backend.command([0.0] * len(self.q))
        self.q = self.backend.read()
        self.steps.clear()
        if self.action_done:
            self.finish_action("inactive")

    def on_deactivate(self, state):
        self.stop()
        self.status_pub.publish(ArmStatus(phase="inactive", holding=self.holding, mode=self.mode))
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.destroy_lifecycle_publisher(self.joint_pub)
        self.destroy_lifecycle_publisher(self.status_pub)
        self.joint_pub = self.status_pub = None
        return super().on_cleanup(state)

    def on_shutdown(self, state):
        self.stop()
        return super().on_shutdown(state)

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

    def backend_ready(self):
        return getattr(self.backend, "ready", True)

    def on_move(self, request, response):
        if self.timer is None:
            response.message = "controller inactive"
            return response
        if not self.backend_ready():
            response.message = "waiting for joint feedback"
            return response
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
        response.message = f"moving to ({target[0]:.2f}, {target[1]:.2f})"
        if math.hypot(x, y) > REACH:
            response.message = f"({x:.2f}, {y:.2f}) is out of reach, " + response.message
        return response

    def start_pick_place(self, request):
        if self.timer is None:
            return "controller inactive"
        if not self.backend_ready():
            return "waiting for joint feedback"
        if self.steps:
            return "busy: finishing the current move"
        # Neither point is projected: picking at the wrong spot is worse than not picking.
        pick, _, reason = self.plan(request.pick_x, request.pick_y, self.q, allow_projection=False)
        if pick is None:
            return "refused: pick " + reason
        place, _, reason = self.plan(request.place_x, request.place_y, pick, allow_projection=False)
        if place is None:
            return "refused: place " + reason
        # A pick or place is a pause at one pose while the gripper closes or opens.
        self.run([self.move_step(self.q, pick), ("picking", pick, pick, GRASP_TIME),
                  self.move_step(pick, place), ("placing", place, place, GRASP_TIME)])
        return None

    def on_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def on_goal(self, _request):
        return GoalResponse.ACCEPT if self.timer is not None and self.backend_ready() else GoalResponse.REJECT

    async def execute_pick_place(self, goal_handle):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return PickPlaceAction.Result(success=False, message="canceled")
        error = self.start_pick_place(goal_handle.request)
        if error:
            goal_handle.abort()
            return PickPlaceAction.Result(success=False, message=error)
        self.action_goal = goal_handle
        done = Future()
        self.action_done = done
        goal_handle.publish_feedback(PickPlaceAction.Feedback(phase=self.steps[0][0]))
        outcome = await done
        if outcome == "canceled":
            goal_handle.canceled()
        elif outcome in ("failed", "inactive"):
            goal_handle.abort()
        else:
            goal_handle.succeed()
        message = {"succeeded": "pick/place complete", "canceled": "pick/place canceled",
                   "failed": "joint goal not reached", "inactive": "controller deactivated"}[outcome]
        return PickPlaceAction.Result(success=outcome == "succeeded", message=message)

    def finish_action(self, outcome):
        done = self.action_done
        self.action_goal = None
        self.action_done = None
        done.set_result(outcome)

    def tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now
        if self.jitter_start is not None:
            jitter = abs(dt - self.period)
            self.jitter_count += 1
            self.jitter_total += jitter
            self.jitter_max = max(self.jitter_max, jitter)
            if now - self.jitter_start >= JITTER_SAMPLE_S:
                self.get_logger().info(
                    f"timer jitter ({1000 * self.period:.1f} ms target, {self.jitter_count} intervals): "
                    f"mean abs {1000 * self.jitter_total / self.jitter_count:.2f} ms, "
                    f"max {1000 * self.jitter_max:.2f} ms")
                self.jitter_start = None
        if not self.backend_ready():
            return
        previous_q = self.q
        if self.mode == "velocity":
            self.q = self.backend.read()
        if self.steps and self.action_goal and self.action_goal.is_cancel_requested:
            if self.mode == "position":
                self.q = self.backend.read()
            self.backend.command([0.0] * len(self.q) if self.mode == "velocity" else self.q)
            self.steps.clear()
            self.publish_status()
            self.finish_action("canceled")
        if self.steps:
            phase, start, goal, duration = self.steps[0]
            # Fraction of the step done, from the time passed since it started.
            s = min((now - self.step_start) / duration, 1.0)
            # Quintic: zero speed and acceleration at both ends, so the arm eases in and out.
            eased = 10 * s**3 - 15 * s**4 + 6 * s**5
            command = [a + (b - a) * eased for a, b in zip(start, goal)]
            if self.mode == "velocity":
                slope = (30 * s**2 - 60 * s**3 + 30 * s**4) / duration
                velocity = [(b - a) * slope + VELOCITY_GAIN * (ref - actual)
                            for a, b, ref, actual in zip(start, goal, command, self.q)]
                velocity = [max(-MAX_SPEED, min(MAX_SPEED, v)) for v in velocity]
                predicted = [q + v * self.period for q, v in zip(self.q, velocity)]
                if not (self.arm.within_joint_limits(predicted) and self.arm.arm_above_base(predicted)):
                    velocity = [0.0] * len(self.q)
                self.backend.command(velocity)
            else:
                self.backend.command(command)
                self.q = self.backend.read()
            arrived = max(abs(a - b) for a, b in zip(self.q, goal)) <= self.joint_tol
            settle = 1.0 if self.mode == "velocity" else 4.0
            if (phase == "moving" and s == 1.0 and not arrived
                    and now - self.step_start > duration + settle):
                if self.mode == "velocity":
                    self.backend.command([0.0] * len(self.q))
                self.steps.clear()
                self.status_pub.publish(ArmStatus(phase="failed", holding=self.holding, mode=self.mode))
                self.get_logger().error("move failed: joint goal not reached")
                if self.action_done:
                    self.finish_action("failed")
            elif s == 1.0 and (phase != "moving" or arrived):
                if self.mode == "velocity":
                    self.backend.command([0.0] * len(self.q))
                if phase != "moving":
                    self.holding = phase == "picking"
                self.steps.pop(0)
                self.step_start = time.monotonic()
                self.publish_status()
                if not self.steps and self.action_done:
                    self.finish_action("succeeded")
        else:
            if self.mode == "velocity":
                self.backend.command([0.0] * len(self.q))
            else:
                self.q = self.backend.read()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1", "joint2", "joint3"]
        msg.position = self.q
        msg.velocity = [(q - old) / dt for q, old in zip(self.q, previous_q)] if dt > 0 else [0.0] * len(self.q)
        self.joint_pub.publish(msg)

    def publish_status(self):
        self.status_pub.publish(ArmStatus(phase=self.steps[0][0] if self.steps else "idle",
                                          holding=self.holding, mode=self.mode))
        if self.steps and self.action_goal:
            self.action_goal.publish_feedback(PickPlaceAction.Feedback(phase=self.steps[0][0]))


def main(args=None, backend_factory=None):
    rclpy.init(args=args)
    node = ControllerNode(backend_factory=backend_factory)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Under ros2 launch a second Ctrl-C follows the first; it must not interrupt cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
