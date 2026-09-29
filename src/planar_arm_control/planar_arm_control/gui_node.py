#!/usr/bin/env python3
"""
gui_node.py  —  STARTER STUB. This is YOUR work to implement.

Goal: a PyQt5 + PyQtGraph node that is a live client of the controller.

Suggested behaviour (adapt as you like, document changes):
    - Subscribes to /joint_states and renders the arm live (links + joints).
    - Plots joint angles (and, if you do PID tracking, tracking error) over
      time with PyQtGraph.
    - Lets the operator enter pick and place targets and send them to the
      controller (service call or topic publish).
    - Shows telemetry: end-effector position, current mode, status.

THE KEY CHALLENGE: the ROS 2 executor and the Qt event loop must run together
without either one blocking the other. Spinning ROS in a background thread or
driving rclpy.spin_once from a QTimer are both acceptable — your handling of
this is a graded signal (multithreading / timer synchronisation).

You may reuse the look and feel of a standard PyQtGraph arm plot; the point of
this task is the ROS 2 integration, not pixel-perfect styling.
"""

import math
import signal
import threading
import time
from collections import deque

import rclpy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

from planar_arm_control.planar_arm import PlanarArm
from planar_arm_msgs.srv import MoveToTarget

LINK_LENGTHS = [3.0, 2.0, 1.5]
HISTORY_S = 10.0  # seconds of joint angles in the plot
STALE_S = 0.5  # no joint state for this long means the controller is gone


class Window(QtWidgets.QWidget):
    # Emitted on the ROS thread; Qt delivers them on the GUI thread.
    joints = QtCore.pyqtSignal(float, list)
    reply = QtCore.pyqtSignal(object)

    def __init__(self, node):
        super().__init__()
        self.setWindowTitle("Planar arm")
        self.arm = PlanarArm(LINK_LENGTHS)
        self.history = deque()
        self.last_data = -math.inf
        self.joints.connect(self.on_joints)
        self.reply.connect(self.on_reply)
        node.create_subscription(JointState, "joint_states", lambda msg: self.joints.emit(
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, list(msg.position)), 10)
        self.client = node.create_client(MoveToTarget, "move_to_target")

        view = pg.PlotWidget(background="w")
        view.setAspectLocked(True)
        view.setRange(xRange=(-7, 7), yRange=(-1, 7))
        dark = (60, 60, 60)
        # The ground: a line at y = 0, shaded below.
        view.plot([-100, 100], [0, 0], pen=pg.mkPen(dark, width=2), fillLevel=-100, brush=(220, 220, 220))
        # The arm's reach: anything outside this dashed arc is out of reach.
        arc = [i * math.pi / 100 for i in range(101)]
        view.plot([sum(LINK_LENGTHS) * math.cos(a) for a in arc], [sum(LINK_LENGTHS) * math.sin(a) for a in arc],
                  pen=pg.mkPen((150, 150, 150), style=QtCore.Qt.DashLine))
        self.links = view.plot(pen=pg.mkPen(dark, width=8), symbol="o", symbolSize=12,
                               symbolBrush="w", symbolPen=pg.mkPen(dark, width=2))
        # Follows the target boxes, so the target is visible before it is sent.
        self.target = view.plot(pen=None, symbolSize=16)
        angles = pg.PlotWidget(background="w", title="Joint angles (deg)")
        angles.setLabel("bottom", "seconds ago")
        angles.addLegend(brush=(255, 255, 255, 220))  # readable where it covers the curves
        self.curves = [angles.plot(pen=pg.mkPen(c, width=2), name=f"joint {i + 1}")
                       for i, c in enumerate([(31, 119, 180), (255, 127, 14), (44, 160, 44)])]
        # A stray drag or wheel would stop the plots following the data.
        for plot in (view, angles):
            plot.setMouseEnabled(x=False, y=False)
            plot.hideButtons()
        self.tool_label = QtWidgets.QLabel()
        self.connection_label = QtWidgets.QLabel()
        self.reply_label = QtWidgets.QLabel()
        self.reply_label.setWordWrap(True)
        # Filled in with the out-of-reach test case.
        self.x_box, self.y_box = QtWidgets.QDoubleSpinBox(), QtWidgets.QDoubleSpinBox()
        for box, value in ((self.x_box, 7.0), (self.y_box, 3.0)):
            box.setRange(-10.0, 10.0)
            box.setValue(value)
            box.valueChanged.connect(self.show_target)
        self.show_target()
        self.move_button = QtWidgets.QPushButton("Move")
        self.move_button.clicked.connect(self.send_move)

        controls = QtWidgets.QHBoxLayout()
        for widget in (QtWidgets.QLabel("x"), self.x_box, QtWidgets.QLabel("y"), self.y_box, self.move_button):
            controls.addWidget(widget)
        controls.addStretch()
        side = QtWidgets.QVBoxLayout()
        for widget in (angles, self.tool_label, self.connection_label, self.reply_label):
            side.addWidget(widget)
        side.addLayout(controls)
        layout = QtWidgets.QHBoxLayout(self)
        layout.addWidget(view)
        layout.addLayout(side)

        # Also lets Python run its Ctrl-C handler while Qt is in its C++ loop.
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.check_health)
        timer.start(200)

    def on_joints(self, stamp, q):
        self.last_data = time.monotonic()
        points = self.arm.forward_kinematics(q)
        self.links.setData([p[0] for p in points], [p[1] for p in points])
        self.tool_label.setText(f"tool: x {points[-1][0]:.3f}   y {points[-1][1]:.3f}")
        self.history.append((stamp, [math.degrees(a) for a in q]))
        while self.history[0][0] < stamp - HISTORY_S:
            self.history.popleft()
        for i, curve in enumerate(self.curves):
            curve.setData([t - stamp for t, _ in self.history], [a[i] for _, a in self.history])

    def check_health(self):
        fresh = time.monotonic() - self.last_data < STALE_S
        self.connection_label.setText("controller: connected" if fresh else "controller: no data")
        self.move_button.setEnabled(fresh)

    def show_target(self):
        x, y = self.x_box.value(), self.y_box.value()
        # ○ above the ground and within reach; ✕ where the arm cannot go as given.
        valid = y >= 0 and math.hypot(x, y) <= sum(LINK_LENGTHS)
        colour = (40, 160, 60) if valid else (220, 40, 40)
        self.target.setData([x], [y], symbol="o" if valid else "x", symbolPen=pg.mkPen(colour, width=2),
                            symbolBrush=(0, 0, 0, 0) if valid else colour)

    def send_move(self):
        request = MoveToTarget.Request(x=self.x_box.value(), y=self.y_box.value())
        # call_async returns at once; the reply arrives on the ROS thread and goes out as a signal.
        self.client.call_async(request).add_done_callback(lambda future: self.reply.emit(future.result()))

    def on_reply(self, response):
        self.reply_label.setText(response.message)
        self.reply_label.setStyleSheet("" if response.accepted else "color: red")


def main(args=None):
    # Ctrl-C is handled below, so it closes the window instead of only stopping ROS.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    app = QtWidgets.QApplication([])
    node = rclpy.create_node("gui_node")
    window = Window(node)
    window.show()
    threading.Thread(target=rclpy.spin, args=(node,)).start()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    app.exec_()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
