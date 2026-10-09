#!/usr/bin/env python3
import math
import signal
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from sensor_msgs.msg import JointState

# Supplied kinematics library, used unchanged.
from planar_arm_control.planar_arm import PlanarArm
from planar_arm_msgs.action import PickPlace as PickPlaceAction
from planar_arm_msgs.msg import ArmStatus
from planar_arm_msgs.srv import MoveToTarget

LINK_LENGTHS = [3.0, 2.0, 1.5]
REACH = sum(LINK_LENGTHS)
JOINTS = ["joint1", "joint2", "joint3"]
HOME = [math.pi / 2, 0.0, 0.0]  # pointing straight up
MAX_SPEED = math.radians(60.0)  # per joint
MAX_ACCEL = math.radians(120.0)
MIN_MOVE_TIME = 0.5  # also avoids dividing by zero on a zero-length move
GRASP_TIME = 0.5  # pause to close or open the gripper
GOAL_TOLERANCE = 1e-3  # the IK answer must put the tool within 1 mm of the target
JOINT_TOLERANCE = 1e-3  # measured pose must reach the commanded goal before the next move
VELOCITY_GAIN = 8.0  # position correction in velocity mode, 1/s
JITTER_SAMPLE_S = 10.0
SETTLE_SPEED = math.radians(0.1)  # measured joint speed before a move completes
SETTLE_TIME = 0.3  # how long the arm must stay settled
SETTLE_TIMEOUT = 1.0  # after the planned duration, a move that has not settled fails
RESULT_MESSAGES = {"succeeded": "pick/place complete", "canceled": "pick/place canceled",
                   "failed": "joint goal not settled", "inactive": "controller deactivated"}


class SimArm:
    def __init__(self, q, clock, mode="position", period=0.02):
        self.q = list(q)
        self.clock = clock
        self.mode = mode
        self.period = period
        self.velocity = [0.0] * len(q)
        self.last_update = self.last_state_time = time.monotonic()
        self.last_state_q = list(q)

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

    def read_state(self):
        q = self.read()
        now = time.monotonic()
        dt = now - self.last_state_time
        velocity = [(a - b) / dt for a, b in zip(q, self.last_state_q)] if dt > 0 else [0.0] * len(q)
        self.last_state_q, self.last_state_time = q, now
        return q, velocity, self.clock.now().to_msg()


class ControllerNode(LifecycleNode):
    # Every callback runs on main()'s single-threaded executor, so callbacks never overlap.
    def __init__(self, backend_factory=None):
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
        self.backend = (backend_factory(self) if backend_factory is not None
                        else SimArm(HOME, self.get_clock(), self.mode, self.period))
        self.q = self.backend.read()
        self.holding = False
        # Steps to run, as (phase, start pose, goal pose, duration); the first one is running.
        self.steps = []
        self.step_start = 0.0
        self.settled_since = None
        self.action_goal = None
        self.action_done = None
        self.joint_pub = None
        self.status_pub = None
        self.timer = None
        self.create_service(MoveToTarget, "move_to_target", self.on_move)
        # rclpy rejects cancel requests unless a cancel callback accepts them.
        self.action_server = ActionServer(self, PickPlaceAction, "pick_place", self.execute_pick_place,
                                          cancel_callback=lambda _: CancelResponse.ACCEPT)

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
        self.last_feedback_stamp = None
        self.waiting_for_backend = not self.backend_ready()
        self.last_tick = self.jitter_start = None  # set on the first tick
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
        self.settled_since = None
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
        # The start pose is skipped: the arm is already there, and in Gazebo it can rest a hair below y = 0.
        samples = int(math.degrees(max(abs(b - a) for a, b in zip(start, goal)))) + 2
        for i in range(1, samples):
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
        self.step_start = self.clock_now()
        self.settled_since = None
        self.publish_status()

    def clock_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def backend_ready(self):
        return getattr(self.backend, "ready", True)

    def unavailable_reason(self):
        if self.timer is None:
            return "controller inactive"
        if not self.backend_ready():
            return "waiting for joint feedback"
        if self.steps or self.action_goal is not None:
            return "busy: finishing the current move"
        return None

    def on_move(self, request, response):
        reason = self.unavailable_reason()
        if reason:
            response.message = reason
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
        reason = self.unavailable_reason()
        if reason:
            return reason
        # Neither point is projected: picking at the wrong spot is worse than not picking.
        # A pick or place is a pause at one pose while the gripper closes or opens.
        steps, place_from = [], self.q
        if not self.holding:  # after a cancel mid-carry the block is still held: go straight to placing
            pick, _, reason = self.plan(request.pick_x, request.pick_y, self.q, allow_projection=False)
            if pick is None:
                return "refused: pick " + reason
            steps, place_from = [self.move_step(self.q, pick), ("picking", pick, pick, GRASP_TIME)], pick
        place, _, reason = self.plan(request.place_x, request.place_y, place_from, allow_projection=False)
        if place is None:
            return "refused: place " + reason
        self.run(steps + [self.move_step(place_from, place), ("placing", place, place, GRASP_TIME)])
        return None

    async def execute_pick_place(self, goal_handle):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return PickPlaceAction.Result(success=False, message=RESULT_MESSAGES["canceled"])
        error = self.start_pick_place(goal_handle.request)
        if error:
            goal_handle.abort()
            return PickPlaceAction.Result(success=False, message=error)
        self.action_goal = goal_handle
        done = self.action_done = Future()
        goal_handle.publish_feedback(PickPlaceAction.Feedback(phase=self.steps[0][0]))
        outcome = await done  # set by finish_action() from the timer, the cancel or deactivation
        if outcome == "succeeded":
            goal_handle.succeed()
        elif outcome == "canceled":
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return PickPlaceAction.Result(success=outcome == "succeeded", message=RESULT_MESSAGES[outcome])

    def finish_action(self, outcome):
        done = self.action_done
        self.action_goal = self.action_done = None
        if done is not None and not done.done():
            done.set_result(outcome)

    def record_jitter(self, now):
        """Log how far the timer strays from its period, once, over the first JITTER_SAMPLE_S."""
        if self.last_tick is None:
            # In Gazebo the simulation clock can start after activation, so measure from the first tick.
            self.last_tick = self.jitter_start = now
            return
        jitter = abs(now - self.last_tick - self.period)
        self.last_tick = now
        if self.jitter_start is None:
            return
        self.jitter_count += 1
        self.jitter_total += jitter
        self.jitter_max = max(self.jitter_max, jitter)
        if now - self.jitter_start >= JITTER_SAMPLE_S:
            self.get_logger().info(
                f"timer jitter ({1000 * self.period:.1f} ms target, {self.jitter_count} intervals): "
                f"mean abs {1000 * self.jitter_total / self.jitter_count:.2f} ms, "
                f"max {1000 * self.jitter_max:.2f} ms")
            self.jitter_start = None

    def velocity_command(self, start, goal, reference, s, duration):
        """Planned joint speed plus a correction toward the planned pose, kept inside the limits."""
        slope = (30 * s**2 - 60 * s**3 + 30 * s**4) / duration  # derivative of the quintic
        command = [(b - a) * slope + VELOCITY_GAIN * (ref - q)
                   for a, b, ref, q in zip(start, goal, reference, self.q)]
        # Scale all joints together, so the arm keeps the planned direction.
        ratio = max(abs(v) / MAX_SPEED for v in command)
        if ratio > 1.0:
            command = [v / ratio for v in command]
        # Stop rather than pass a joint limit or the ground within the next period.
        predicted = [q + v * self.period for q, v in zip(self.q, command)]
        if not (self.arm.within_joint_limits(predicted) and self.arm.arm_above_base(predicted)):
            return [0.0] * len(command)
        return command

    def tick(self):
        now = self.clock_now()
        self.record_jitter(now)
        if not self.backend_ready():
            return
        if self.waiting_for_backend:
            self.waiting_for_backend = False
            self.publish_status()
        if self.mode == "velocity":
            self.q = self.backend.read()  # the correction needs the latest measured pose
        hold = [0.0] * len(self.q) if self.mode == "velocity" else self.q
        if self.steps and self.action_goal and self.action_goal.is_cancel_requested:
            self.backend.command(hold)
            self.steps.clear()
            self.publish_status()
            self.finish_action("canceled")

        if self.steps:
            phase, start, goal, duration = self.steps[0]
            # Fraction of the step done, from the time passed since it started.
            s = min((now - self.step_start) / duration, 1.0)
            # Quintic: zero speed and acceleration at both ends, so the arm eases in and out.
            eased = 10 * s**3 - 15 * s**4 + 6 * s**5
            reference = [a + (b - a) * eased for a, b in zip(start, goal)]
            if self.mode == "velocity":
                self.backend.command(self.velocity_command(start, goal, reference, s, duration))
            else:
                self.backend.command(reference)
        elif self.mode == "velocity":
            self.backend.command(hold)

        # Read after commanding, so /joint_states shows this tick's result, not the last one's.
        self.q, velocity, stamp = self.backend.read_state()
        fresh = (stamp.sec, stamp.nanosec) != self.last_feedback_stamp
        self.last_feedback_stamp = (stamp.sec, stamp.nanosec)
        if self.steps:
            self.finish_step_if_done(now, fresh, velocity)
        msg = JointState(name=JOINTS, position=self.q, velocity=velocity)
        msg.header.stamp = stamp
        self.joint_pub.publish(msg)

    def finish_step_if_done(self, now, fresh, velocity):
        phase, _, goal, duration = self.steps[0]
        elapsed = now - self.step_start
        if elapsed < duration:
            return
        if phase == "moving":
            # Done only when the measured arm has stayed at the goal, nearly still, for SETTLE_TIME.
            arrived = max(abs(a - b) for a, b in zip(self.q, goal)) <= self.joint_tol
            still = fresh and max(abs(v) for v in velocity) <= SETTLE_SPEED
            if not (arrived and still):
                self.settled_since = None
            elif self.settled_since is None:
                self.settled_since = now
            if self.settled_since is None or now - self.settled_since < SETTLE_TIME:
                if elapsed > duration + SETTLE_TIMEOUT:
                    self.fail_move()
                return
        if self.mode == "velocity":
            self.backend.command([0.0] * len(self.q))
        if phase != "moving":
            self.holding = phase == "picking"
        self.steps.pop(0)
        self.step_start = now
        self.settled_since = None
        self.publish_status()
        if not self.steps and self.action_done:
            self.finish_action("succeeded")

    def fail_move(self):
        if self.mode == "velocity":
            self.backend.command([0.0] * len(self.q))
        self.steps.clear()
        self.settled_since = None
        self.status_pub.publish(ArmStatus(phase="failed", holding=self.holding, mode=self.mode))
        self.get_logger().error("move failed: " + RESULT_MESSAGES["failed"])
        if self.action_done:
            self.finish_action("failed")

    def publish_status(self):
        if self.status_pub is not None:
            phase = self.steps[0][0] if self.steps else ("idle" if self.backend_ready() else "homing")
            self.status_pub.publish(ArmStatus(phase=phase, holding=self.holding, mode=self.mode))
        if self.steps and self.action_goal:
            self.action_goal.publish_feedback(PickPlaceAction.Feedback(phase=self.steps[0][0]))


def main(args=None, backend_factory=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ControllerNode(backend_factory=backend_factory)
    # Ctrl-C only sets a flag: a KeyboardInterrupt can land inside rclpy while it takes a message.
    stopping = []
    signal.signal(signal.SIGINT, lambda *_: stopping.append(True))
    executor = rclpy.get_global_executor()
    executor.add_node(node)
    while not stopping:
        executor.spin_once(timeout_sec=0.1)
    # Under ros2 launch a second Ctrl-C follows the first; it must not kill the process while it exits.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    node.stop()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
