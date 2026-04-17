#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget


class AngleMonitorNode(Node):
    def __init__(self):
        super().__init__('gui_moteus_angles')

        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3'])
        self.joint_names = list(self.get_parameter('joint_names').value)

        self.angles_deg = [0.0] * len(self.joint_names)

        self.create_subscription(
            Float64MultiArray,
            '/moteus_angles_deg',
            self.cb_angles,
            10
        )

    def cb_angles(self, msg: Float64MultiArray):
        n = min(len(msg.data), len(self.angles_deg))
        for i in range(n):
            self.angles_deg[i] = float(msg.data[i])


class AngleWindow(QWidget):
    def __init__(self, ros_node: AngleMonitorNode):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle('Moteus Angles Monitor')
        self.resize(320, 180)

        layout = QVBoxLayout()
        self.labels = []

        for name in self.node.joint_names:
            label = QLabel(f'{name}: 0.00°')
            label.setStyleSheet('font-size: 18px;')
            layout.addWidget(label)
            self.labels.append(label)

        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(20)

    def update_gui(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        for i, name in enumerate(self.node.joint_names):
            self.labels[i].setText(f'{name}: {self.node.angles_deg[i]:.2f}°')

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    ros_node = AngleMonitorNode()

    app = QApplication(sys.argv)
    window = AngleWindow(ros_node)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
