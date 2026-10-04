# Design note

```
GUI ── target ──► controller ── command ──► SimArm or GazeboArm
    ◄── joint angles, status ──         ◄── angles ──
```

## Architecture

**Controller.** One node with a 50 Hz timer that runs the moves. It checks each target first: the IK answer must reach it, stay within the joint limits, and keep the arm above the ground. If a check fails, it refuses and says why. One exception: an out-of-reach **Move**, like `(7, 3)`, goes to the closest point; pick and place still refuses it.

**Moves.** Each move is a quintic in joint space, so it starts and stops smoothly. It only ends when the arm has stopped at the goal. Position mode sends angles; velocity mode sends the planned speed plus a P correction toward the planned angle.

**Swappable arm.** The controller only talks to the arm through a small backend class. `SimArm` and `GazeboArm` are two backends, and real motors would be a third.

**GUI.** ROS runs on a background thread and only sends data to Qt through signals, so ROS never touches the widgets and neither loop waits for the other.

## Bug in `planar_arm.py`

In the IK, `beta` is calculated but not used for `q2`, so the exact solution only finds poses with joint 3 at 0°. Other targets go to a fallback that skips the limit and ground checks, so the controller checks every answer again.

## Trade-offs

- **One thread** in the controller: simple, no locks, but a slow request can delay a tick.
- **Quintic** instead of trapezoidal: smoother, but slower moves.
- **Gazebo's own joint controllers** instead of ros2_control: less setup, but not how real hardware would connect.

## Next

- Automated tests for the brief's cases.
- An I term and a tracking-error plot for velocity mode.
- A ros2_control interface for real motors.
