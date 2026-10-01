import math
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JOINTS = ("joint1", "joint2", "joint3")
JOINT_LIMITS = [
    (0.0, math.pi),
    (-2.0 * math.pi / 3.0, 2.0 * math.pi / 3.0),
    (-2.0 * math.pi / 3.0, 2.0 * math.pi / 3.0),
]


class GazeboArm:
    """Drives the Gazebo arm through the ros_gz bridge.

    Same command()/read() interface as SimArm, so the planner is unchanged.
    Supports both position setpoints and velocity integration.
    """

    def __init__(self, node, initial_q, mode="position", period=0.02):
        self.mode = mode
        self.period = period
        self.q = list(initial_q)
        self.q_target = list(initial_q)
        self.last_cmd_time = time.monotonic()
        self.ready = False
        self.publishers = [
            node.create_publisher(Float64, f"/gazebo/{name}/cmd_pos", 10)
            for name in JOINTS
        ]
        node.create_subscription(JointState, "/gazebo/joint_states", self.on_state, 10)

    def on_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in JOINTS):
            self.q = [positions[name] for name in JOINTS]
            if not self.ready:
                self.q_target = list(self.q)
                self.ready = True

    def command(self, values):
        now = time.monotonic()
        dt = max(1e-4, min(now - self.last_cmd_time, self.period * 2.0))
        self.last_cmd_time = now

        if self.mode == "velocity":
            # Integrate velocity commands into safe, limit-clamped position targets for Gazebo PID
            self.q_target = [
                max(q_min, min(q_max, target + float(v) * dt))
                for target, v, (q_min, q_max) in zip(self.q_target, values, JOINT_LIMITS)
            ]
            cmd_positions = self.q_target
        else:
            self.q_target = list(values)
            cmd_positions = self.q_target

        for publisher, value in zip(self.publishers, cmd_positions):
            publisher.publish(Float64(data=float(value)))

    def read(self):
        return list(self.q)

