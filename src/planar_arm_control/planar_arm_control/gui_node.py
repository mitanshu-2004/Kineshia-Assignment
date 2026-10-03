#!/usr/bin/env python3
import math
import signal
import threading
import time
from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

from planar_arm_control.planar_arm import PlanarArm
from planar_arm_msgs.action import PickPlace as PickPlaceAction
from planar_arm_msgs.msg import ArmStatus
from planar_arm_msgs.srv import MoveToTarget

LINK_LENGTHS = [3.0, 2.0, 1.5]
REACH = sum(LINK_LENGTHS)
HISTORY_S = 10.0  # seconds of joint angles in the plot
STALE_S = 0.5  # no joint state for this long means the controller is gone


def number_box(value):
    box = QtWidgets.QDoubleSpinBox()
    box.setRange(-10.0, 10.0)
    box.setValue(value)
    return box


class Window(QtWidgets.QWidget):
    # Emitted on the ROS thread; Qt delivers them on the GUI thread.
    joints = QtCore.pyqtSignal(float, list, list)
    status = QtCore.pyqtSignal(str, bool, str)
    reply = QtCore.pyqtSignal(object)
    action_update = QtCore.pyqtSignal(str, bool)
    goal_ready = QtCore.pyqtSignal(object)

    def __init__(self, node):
        super().__init__()
        self.setWindowTitle("Planar arm")
        self.arm = PlanarArm(LINK_LENGTHS)
        self.history = deque(maxlen=2000)
        self.last_data = -math.inf
        self.draw_count = 0
        self.action_start_time = 0.0
        self.idle = False
        self.holding = False
        self.action_goal = None
        self.action_pending = False
        self.cancel_requested = False
        self.move_completion = None
        self.joints.connect(self.on_joints)
        self.status.connect(self.on_status)
        self.reply.connect(self.on_reply)
        self.action_update.connect(self.on_action_update)
        self.goal_ready.connect(self.on_goal_ready)
        node.create_subscription(JointState, "joint_states", lambda msg: self.joints.emit(
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, list(msg.position), list(msg.velocity)), 10)
        # Transient local, like the publisher, so the current status arrives even if the GUI starts later.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(ArmStatus, "arm_status", lambda msg: self.status.emit(msg.phase, msg.holding, msg.mode),
                                 latched)
        self.move_client = node.create_client(MoveToTarget, "move_to_target")
        self.action_client = ActionClient(node, PickPlaceAction, "pick_place")

        view = pg.PlotWidget(background="w")
        view.setTitle("Arm position")
        view.setLabel("bottom", "x")
        view.setLabel("left", "y")
        view.setAspectLocked(True)
        view.setRange(xRange=(-7, 7), yRange=(-1, 7))
        dark = (60, 60, 60)
        # The ground: a line at y = 0, shaded below.
        view.plot([-100, 100], [0, 0], pen=pg.mkPen(dark, width=2), fillLevel=-100, brush=(220, 220, 220))
        # The arm's reach: anything outside this dashed arc is out of reach.
        arc = [i * math.pi / 100 for i in range(101)]
        view.plot([REACH * math.cos(a) for a in arc], [REACH * math.sin(a) for a in arc],
                  pen=pg.mkPen((150, 150, 150), style=QtCore.Qt.DashLine))
        self.links = view.plot(pen=pg.mkPen(dark, width=8), symbol="o", symbolSize=12,
                               symbolBrush="w", symbolPen=pg.mkPen(dark, width=2))
        # Added after the arm so it is drawn on top, visible on the tool while carried.
        self.block = view.plot(pen=None, symbol="s", symbolSize=18, symbolBrush=(230, 160, 40))
        self.place_mark = view.plot(pen=None, symbol="o", symbolSize=26,
                                    symbolPen=pg.mkPen((40, 100, 200), width=2), symbolBrush=None)
        # Follows the move boxes, so the target is visible before it is sent.
        self.move_mark = view.plot(pen=None, symbolSize=16)
        self.projected_mark = view.plot(pen=None, symbol="o", symbolSize=14,
                                        symbolPen=pg.mkPen((40, 160, 60), width=2), symbolBrush=None)
        angles = pg.PlotWidget(background="w")
        speeds = pg.PlotWidget(background="w")
        speeds.setTitle("Joint velocity")
        for plot, label, unit, limits in ((angles, "angle", "deg", (-120, 180)),
                                          (speeds, "velocity", "deg/s", (-70, 70))):
            plot.setLabel("left", label, units=unit)
            plot.setLabel("bottom", "time (s, 0 = now)")
            plot.setXRange(-HISTORY_S, 0, padding=0)
            plot.setYRange(*limits, padding=0)
            plot.addLegend(offset=(5, 5), colCount=3, brush=(255, 255, 255, 220))
        colours = [(31, 119, 180), (255, 127, 14), (44, 160, 44)]
        self.angle_curves = [angles.plot(pen=pg.mkPen(c, width=2), name=f"joint {i + 1}")
                             for i, c in enumerate(colours)]
        self.speed_curves = [speeds.plot(pen=pg.mkPen(c, width=2), name=f"joint {i + 1}")
                             for i, c in enumerate(colours)]
        # A stray drag or wheel would stop the plots following the data.
        for plot in (view, angles, speeds):
            plot.setMouseEnabled(x=False, y=False)
            plot.hideButtons()
        self.plots = QtWidgets.QTabWidget()
        self.plots.addTab(angles, "Angles")
        self.plots.addTab(speeds, "Velocities")
        self.plots.currentChanged.connect(self.update_plot)
        self.tool_label = QtWidgets.QLabel()
        self.joint_label = QtWidgets.QLabel()
        self.velocity_label = QtWidgets.QLabel()
        self.connection_label = QtWidgets.QLabel()
        self.status_label = QtWidgets.QLabel()
        self.mode_label = QtWidgets.QLabel("—")
        self.reply_label = QtWidgets.QLabel()
        self.reply_label.setWordWrap(True)
        self.tool_label.setText("waiting for joint states")
        self.joint_label.setText("—")
        self.velocity_label.setText("—")
        self.status_label.setText("waiting for controller")

        # Filled in with the test scenario: the out-of-reach move, then the pick and place.
        self.move_x, self.move_y = number_box(7.0), number_box(3.0)
        self.pick_x, self.pick_y = number_box(4.0), number_box(2.0)
        self.place_x, self.place_y = number_box(-3.0), number_box(3.0)
        for box in (self.move_x, self.move_y):
            box.valueChanged.connect(self.show_move)
        for box in (self.pick_x, self.pick_y, self.place_x, self.place_y):
            box.valueChanged.connect(self.show_pick)
        self.show_pick()
        self.show_move()
        self.move_button = QtWidgets.QPushButton("Move")
        self.move_button.clicked.connect(self.send_move)
        self.pick_place_button = QtWidgets.QPushButton("Pick && Place")
        self.pick_place_button.clicked.connect(self.send_pick_place)
        self.cancel_button = QtWidgets.QPushButton("Cancel")
        self.cancel_button.clicked.connect(self.cancel_pick_place)

        controls = QtWidgets.QGridLayout()
        controls.setVerticalSpacing(1)
        controls.addWidget(QtWidgets.QLabel("x"), 0, 1)
        controls.addWidget(QtWidgets.QLabel("y"), 0, 2)
        for row, widgets in enumerate([
                (QtWidgets.QLabel("Move to"), self.move_x, self.move_y, self.move_button),
                (QtWidgets.QLabel("Pick at"), self.pick_x, self.pick_y),
                (QtWidgets.QLabel("Place at"), self.place_x, self.place_y,
                 self.pick_place_button, self.cancel_button)], start=1):
            for column, widget in enumerate(widgets):
                controls.addWidget(widget, row, column)
        controls.setColumnStretch(5, 1)
        targets = QtWidgets.QGroupBox("Targets (x, y)")
        targets.setLayout(controls)
        telemetry = QtWidgets.QGroupBox("Telemetry")
        readings = QtWidgets.QFormLayout(telemetry)
        readings.setVerticalSpacing(1)
        readings.addRow("Tool", self.tool_label)
        readings.addRow("Joints", self.joint_label)
        readings.addRow("Velocity", self.velocity_label)
        readings.addRow("State", self.status_label)
        readings.addRow("Mode", self.mode_label)
        readings.addRow("Connection", self.connection_label)
        readings.addRow("Last command", self.reply_label)
        side = QtWidgets.QVBoxLayout()
        side.addWidget(self.plots, 1)
        side.addWidget(telemetry)
        side.addWidget(targets)
        layout = QtWidgets.QHBoxLayout(self)
        layout.addWidget(view, 3)
        layout.addLayout(side, 2)

        # Also lets Python run its Ctrl-C handler while Qt is in its C++ loop.
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.check_health)
        timer.start(200)

    def on_joints(self, stamp, q, velocity):
        if self.history and stamp == self.history[-1][0]:
            return  # The controller repeated the latest Gazebo sample; no new measurement.
        self.last_data = time.monotonic()
        # A gap or backward reset in the controller's timestamps means it stopped or restarted:
        # start the plot afresh instead of joining across the gap.
        if self.history and (stamp - self.history[-1][0] > STALE_S or stamp < self.history[-1][0]):
            self.history.clear()
        points = self.arm.forward_kinematics(q)
        self.links.setData([p[0] for p in points], [p[1] for p in points])
        x, y = points[-1]
        angles = [math.degrees(a) for a in q]
        speeds = [math.degrees(v) for v in velocity]
        self.tool_label.setText(f"x {x:.3f}   y {y:.3f}")
        self.joint_label.setText("   ".join(f"J{i + 1} {a:.1f}°" for i, a in enumerate(angles)))
        self.velocity_label.setText("   ".join(f"J{i + 1} {v:.1f}°/s" for i, v in enumerate(speeds)))
        if self.holding:
            self.block.setData([x], [y])
        self.history.append((stamp, angles, speeds))
        while self.history[0][0] < stamp - HISTORY_S:
            self.history.popleft()
        self.draw_count += 1
        if self.draw_count % 2 == 0:
            self.update_plot()

    def update_plot(self):
        if not self.history:
            return
        index = self.plots.currentIndex()
        curves = self.angle_curves if index == 0 else self.speed_curves
        stamp = self.history[-1][0]
        times = [row[0] - stamp for row in self.history]
        for i, curve in enumerate(curves):
            curve.setData(times, [row[index + 1][i] for row in self.history])

    def on_status(self, phase, holding, mode):
        self.idle = phase in ("idle", "failed")
        self.holding = holding
        self.status_label.setText(f"{phase}   holding: {'yes' if holding else 'no'}")
        self.mode_label.setText(mode)
        if self.move_completion and phase in ("idle", "failed", "inactive"):
            self.reply_label.setText(self.move_completion if phase == "idle" else f"move {phase}")
            self.reply_label.setStyleSheet("" if phase == "idle" else "color: red")
            self.move_completion = None
        self.check_health()

    def check_health(self):
        fresh = time.monotonic() - self.last_data < STALE_S
        self.connection_label.setText("connected" if fresh else "no data")
        if self.action_pending and (time.monotonic() - self.action_start_time > 4.0 or not fresh):
            self.action_pending = False
            self.action_goal = None
            self.reply_label.setText("action timed out or controller unavailable")
            self.reply_label.setStyleSheet("color: red")
        busy = (fresh and not self.idle) or self.action_pending
        for box in (self.move_x, self.move_y, self.pick_x, self.pick_y, self.place_x, self.place_y):
            box.setEnabled(not busy)
        ready = fresh and self.idle and not self.action_pending
        self.move_button.setEnabled(ready and self.move_client.service_is_ready())
        self.pick_place_button.setEnabled(ready and self.action_client.server_is_ready())
        self.cancel_button.setEnabled(fresh and self.action_goal is not None and not self.cancel_requested)

    def show_move(self):
        x, y = self.move_x.value(), self.move_y.value()
        # ○ above the ground and within reach; ✕ where the arm cannot go as given.
        distance = math.hypot(x, y)
        valid = y >= 0 and distance <= REACH
        colour = (40, 160, 60) if valid else (220, 40, 40)
        self.move_mark.setData([x], [y], symbol="o" if valid else "x", symbolPen=pg.mkPen(colour, width=2),
                               symbolBrush=(0, 0, 0, 0) if valid else colour)
        self.projected_mark.setData([], [])
        self.place_mark.setData([], [])

    def show_pick(self):
        # The block waits at the pick point; the move target is hidden while pick and place is in use.
        if not self.holding:
            self.block.setData([self.pick_x.value()], [self.pick_y.value()])
        self.move_mark.setData([], [])
        self.projected_mark.setData([], [])
        x, y = self.place_x.value(), self.place_y.value()
        self.place_mark.setData([x], [y])

    def send_move(self):
        self.show_move()
        x, y = self.move_x.value(), self.move_y.value()
        self.move_client.call_async(MoveToTarget.Request(x=x, y=y)).add_done_callback(
            lambda future: self.reply.emit((future.result(), x, y)))

    def send_pick_place(self):
        self.show_pick()
        if not self.action_client.server_is_ready():
            self.action_update.emit("pick/place action unavailable", False)
            return
        self.action_pending = True
        self.action_start_time = time.monotonic()
        self.check_health()
        goal = PickPlaceAction.Goal(
            pick_x=self.pick_x.value(), pick_y=self.pick_y.value(),
            place_x=self.place_x.value(), place_y=self.place_y.value())
        self.action_client.send_goal_async(goal, feedback_callback=self.on_action_feedback).add_done_callback(
            self.on_action_accepted)

    def on_action_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.goal_ready.emit(None)
            self.action_update.emit("pick/place goal rejected", False)
            return
        self.goal_ready.emit(handle)
        handle.get_result_async().add_done_callback(self.on_action_result)

    def on_action_feedback(self, message):
        self.action_update.emit(f"pick/place: {message.feedback.phase}", True)

    def on_action_result(self, future):
        result = future.result().result
        self.action_update.emit(result.message, result.success)
        self.goal_ready.emit(None)

    def on_goal_ready(self, handle):
        self.action_goal = handle
        self.action_pending = False
        self.cancel_requested = False
        self.check_health()

    def cancel_pick_place(self):
        goal = self.action_goal
        if goal is None:
            return
        self.cancel_requested = True
        self.check_health()
        goal.cancel_goal_async()

    def on_reply(self, reply):
        response, x, y = reply
        self.reply_label.setText(response.message)
        self.reply_label.setStyleSheet("" if response.accepted else "color: red")
        self.move_completion = response.message.replace("moving to", "moved to") if response.accepted else None
        distance = math.hypot(x, y)
        if response.accepted and y >= 0 and distance > REACH:
            self.projected_mark.setData([x * REACH / distance], [y * REACH / distance])

    def on_action_update(self, message, success):
        self.reply_label.setText(message)
        self.reply_label.setStyleSheet("" if success else "color: red")


def main(args=None):
    # Ctrl-C is handled below, so it closes the window instead of only stopping ROS.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    app = QtWidgets.QApplication([])
    node = rclpy.create_node("gui_node")
    window = Window(node)
    window.show()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,))
    ros_thread.start()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    try:
        app.exec_()
    finally:
        # Under ros2 launch a second Ctrl-C follows the first; it must not interrupt the shutdown.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        rclpy.try_shutdown()
        ros_thread.join()
        node.destroy_node()


if __name__ == "__main__":
    main()
