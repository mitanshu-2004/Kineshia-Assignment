from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JOINTS = ("joint1", "joint2", "joint3")


class GazeboArm:
    """Drives the Gazebo arm through the ros_gz bridge.

    Same command()/read() interface as SimArm, so the planner is unchanged.
    """

    def __init__(self, node, initial_q):
        self.q = list(initial_q)
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
            self.ready = True

    def command(self, values):
        for publisher, value in zip(self.publishers, values):
            publisher.publish(Float64(data=float(value)))

    def read(self):
        return list(self.q)
