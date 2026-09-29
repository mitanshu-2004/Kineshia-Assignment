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

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

from planar_arm_control.planar_arm import PlanarArm

LINK_LENGTHS = [3.0, 2.0, 1.5]


class GuiNode(Node):
    def __init__(self, view):
        super().__init__("gui_node")
        self.create_subscription(JointState, "joint_states",
                                 lambda msg: view.joints.emit(list(msg.position)), 10)


class ArmView(pg.PlotWidget):
    # Emitted on the ROS thread; Qt delivers it on the GUI thread.
    joints = QtCore.pyqtSignal(list)

    def __init__(self):
        super().__init__(background="w")
        self.arm = PlanarArm(LINK_LENGTHS)
        self.joints.connect(self.show_joints)
        self.setAspectLocked(True)
        self.setRange(xRange=(-7, 7), yRange=(-1, 7))
        self.addItem(pg.LinearRegionItem(values=(-2, 0), orientation="horizontal", movable=False,
                                         brush=(200, 200, 200, 150)))
        dark = (60, 60, 60)
        self.links = self.plot(pen=pg.mkPen(dark, width=8), symbol="o", symbolSize=12,
                               symbolBrush="w", symbolPen=pg.mkPen(dark, width=2))

    def show_joints(self, q):
        points = self.arm.forward_kinematics(q)
        self.links.setData([p[0] for p in points], [p[1] for p in points])


def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication([])
    view = ArmView()
    view.show()
    threading.Thread(target=rclpy.spin, args=(GuiNode(view),)).start()
    app.exec_()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
