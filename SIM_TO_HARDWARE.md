# Moving this task from simulation to hardware

My controller talks to the arm through a backend (`SimArm` or `GazeboArm`). For a real arm I would add a motor backend, for example for Dynamixel servos, and keep the planner and the checks. The hard part: my `SimArm` is exact and never fails. A real arm is neither.

## Software

**Timing.** Talking to motors over a bus takes time, and Python is not real-time. If a command or reading is late, the arm drifts from the plan. I would measure the real loop time, read and write all motors together once per tick, and move the control loop into ros2_control, which runs it at a fixed rate apart from the rest of the node.

**Failures.** If the controller stops, a motor in velocity mode keeps moving, so the motors need their own timeout. A motor can also fault and switch off, and an arm without brakes falls. I would check motor errors every tick, set joint limits in the motors too, and keep a physical emergency stop within reach.

## Control

**Dynamics.** A real arm has gravity, friction and gear play, so the joints lag and sag. My tolerances were tuned in simulation. I would tune the gains and tolerances on the real arm at low speed, and log how far the arm is from the plan.

**Safety margin.** My velocity mode checks limits only one step ahead, but a real arm needs room to slow down, so I would add a margin.

## GUI

I would keep the GUI's "no data" warning and add the commanded and real arm side by side, plus motor temperature and errors.
