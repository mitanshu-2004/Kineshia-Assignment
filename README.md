# ROS 2 planar arm

Controller and PyQt5 GUI for the supplied three-joint arm. `planar_arm.py` is unchanged.

## Run

Tested on Ubuntu 24.04 with ROS 2 Jazzy.

```bash
sudo apt install python3-colcon-common-extensions python3-numpy python3-pyqt5 python3-pyqtgraph
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch planar_arm_control bringup.launch.py
```

## Check

The GUI starts with the assignment targets filled in. **Move** sends `(7, 3)`, which is out of reach. The controller moves to about `(5.97, 2.56)` and reports that point in the reply.

After the move finishes, **Pick & Place** picks at `(4, 2)` and places at `(-3, 3)`. The orange block follows the tool. Picking and placing are half-second pauses that change the `holding` state; there is no gripper hardware.

## Notes

The controller uses quintic interpolation for smooth starts and stops. It checks joint limits and the ground constraint along each move, then publishes `/joint_states` at 50 Hz. Pick/place rejects unreachable targets; moving to a nearby point would pick or place at the wrong location.

ROS runs on a background thread and passes updates to Qt through signals. The GUI sends service requests without waiting in the Qt thread.

Joint commands go through `SimArm.command()` and the controller reads joint angles through `SimArm.read()`. A motor driver can use the same two methods without changing the trajectory code. No motor driver is included.
