"""Run the existing planner with Gazebo commands and joint feedback."""

from planar_arm_control.controller_node import HOME, main as run_controller

from .gazebo_arm import GazeboArm


def main():
    def backend(node):
        return GazeboArm(node, HOME)

    run_controller(backend_factory=backend)


if __name__ == "__main__":
    main()
