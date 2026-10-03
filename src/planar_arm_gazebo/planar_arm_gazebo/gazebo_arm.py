import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from planar_arm_control.controller_node import JOINTS


class GazeboArm:
    def __init__(self, node, initial_q, mode="velocity"):
        if mode != "velocity":
            raise ValueError("GazeboArm requires control_mode=velocity")
        self.q = list(initial_q)
        self.home = list(initial_q)
        self.last_home_command = 0.0
        self.velocity = None
        self.feedback_stamp = None
        self.ready = False
        self.publishers = [
            node.create_publisher(Float64, f"/gazebo/{name}/cmd_vel", 10)
            for name in JOINTS
        ]
        node.create_subscription(JointState, "/gazebo/joint_states", self.on_state, 10)

    def on_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        if all(name in positions and name in velocities for name in JOINTS):
            self.q = [positions[name] for name in JOINTS]
            self.velocity = [velocities[name] for name in JOINTS]
            self.feedback_stamp = msg.header.stamp
            if not self.ready and time.monotonic() - self.last_home_command >= 0.02:
                self.last_home_command = time.monotonic()
                error = [goal - actual for goal, actual in zip(self.home, self.q)]
                if max(abs(value) for value in error) <= 0.005:
                    self.command([0.0] * len(JOINTS))
                    self.ready = True
                else:
                    self.command([max(-0.5, min(0.5, 2.0 * value)) for value in error])

    def command(self, values):
        for publisher, value in zip(self.publishers, values):
            publisher.publish(Float64(data=float(value)))

    def read(self):
        return list(self.q)

    def read_state(self):
        if not self.ready:
            raise RuntimeError("no complete Gazebo joint feedback yet")
        return list(self.q), list(self.velocity), self.feedback_stamp

