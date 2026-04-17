#!/usr/bin/env python3
import math
from functools import partial

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from moteus_msgs.msg import ControllerState


class MoteusJointBridge(Node):
    def __init__(self):
        super().__init__('moteus_joint_bridge')

        self.declare_parameter('motor_ids', [1, 2, 3])
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3'])
        self.declare_parameter('gear_ratios', [1.0, 1.0, 1.0])
        self.declare_parameter('signs', [1.0, 1.0, 1.0])
        self.declare_parameter('zero_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('moteus_ns', 'moteus')
        self.declare_parameter('publish_rate', 50.0)

        self.motor_ids = list(self.get_parameter('motor_ids').value)
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.gear_ratios = list(self.get_parameter('gear_ratios').value)
        self.signs = list(self.get_parameter('signs').value)
        self.zero_deg = list(self.get_parameter('zero_deg').value)
        self.moteus_ns = str(self.get_parameter('moteus_ns').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        n = len(self.motor_ids)
        if not (
            len(self.joint_names) == n and
            len(self.gear_ratios) == n and
            len(self.signs) == n and
            len(self.zero_deg) == n
        ):
            raise ValueError('Todos los parámetros deben tener la misma longitud')

        self.state = []
        for _ in range(n):
            self.state.append({
                'deg': 0.0,
                'rad': 0.0,
                'vel_deg_s': 0.0,
                'vel_rad_s': 0.0,
                'torque': 0.0,
            })

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.deg_pub = self.create_publisher(Float64MultiArray, '/moteus_angles_deg', 10)

        for i, motor_id in enumerate(self.motor_ids):
            topic = f'/{self.moteus_ns}/id_{motor_id}/state'
            self.create_subscription(
                ControllerState,
                topic,
                partial(self.state_cb, i),
                10
            )
            self.get_logger().info(f'Suscrito a {topic}')

        period = 1.0 / self.publish_rate
        self.create_timer(period, self.publish_data)

    def state_cb(self, idx, msg: ControllerState):
        motor_rev = float(msg.position)
        motor_vel_rev_s = float(msg.velocity)

        joint_rev = self.signs[idx] * (motor_rev / self.gear_ratios[idx])
        joint_deg = joint_rev * 360.0 + self.zero_deg[idx]
        joint_rad = math.radians(joint_deg)

        joint_vel_deg_s = self.signs[idx] * (motor_vel_rev_s / self.gear_ratios[idx]) * 360.0
        joint_vel_rad_s = math.radians(joint_vel_deg_s)

        self.state[idx]['deg'] = joint_deg
        self.state[idx]['rad'] = joint_rad
        self.state[idx]['vel_deg_s'] = joint_vel_deg_s
        self.state[idx]['vel_rad_s'] = joint_vel_rad_s
        self.state[idx]['torque'] = float(msg.torque)

    def publish_data(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [s['rad'] for s in self.state]
        js.velocity = [s['vel_rad_s'] for s in self.state]
        js.effort = [s['torque'] for s in self.state]
        self.joint_pub.publish(js)

        deg_msg = Float64MultiArray()
        deg_msg.data = [s['deg'] for s in self.state]
        self.deg_pub.publish(deg_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoteusJointBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
