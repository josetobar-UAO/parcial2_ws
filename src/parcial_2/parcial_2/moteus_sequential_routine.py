#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from moteus_msgs.msg import PositionCommand, ControllerState


class MoteusSequentialRoutine(Node):
    def __init__(self):
        super().__init__('moteus_sequential_routine')

        self.motor_ids = [1, 2, 3]
        self.targets = [1.0, 0.0]
        self.rate_hz = 50.0

        self.pos_tol = 0.2
        self.vel_tol = 0.02
        self.required_ok_cycles = 10
        self.pause_cycles = 25

        self.state = {
            mid: {'position': None, 'velocity': None}
            for mid in self.motor_ids
        }

        self.cmd_pubs = {}
        self.stop_pubs = {}

        for mid in self.motor_ids:
            self.cmd_pubs[mid] = self.create_publisher(
                PositionCommand, f'/moteus/id_{mid}/cmd_position', 10
            )
            self.stop_pubs[mid] = self.create_publisher(
                Empty, f'/moteus/id_{mid}/cmd_stop', 10
            )

            self.create_subscription(
                ControllerState,
                f'/moteus/id_{mid}/state',
                lambda msg, motor_id=mid: self.state_cb(msg, motor_id),
                10
            )

        self.motor_index = 0
        self.target_index = 0
        self.ok_count = 0
        self.pause_count = 0
        self.finished = False

        self.started = False
        self.start_delay_cycles = 50   # 1 segundo a 50 Hz

        self.timer = self.create_timer(1.0 / self.rate_hz, self.loop_cb)

        self.get_logger().info('Rutina secuencial iniciada')
        self.get_logger().info('Limpiando fallas y esperando estado...')

    def state_cb(self, msg, motor_id):
        self.state[motor_id]['position'] = float(msg.position)
        self.state[motor_id]['velocity'] = float(msg.velocity)

    def publish_position(self, motor_id, target):
        msg = PositionCommand()
        msg.position = [float(target)]
        msg.velocity_limit = [1.0]
        msg.accel_limit = [0.5]
        msg.maximum_torque = [0.5]
        msg.watchdog_timeout = [float('nan')]
        self.cmd_pubs[motor_id].publish(msg)

    def publish_stop(self, motor_id):
        self.stop_pubs[motor_id].publish(Empty())

    def all_states_received(self):
        return all(
            self.state[mid]['position'] is not None and
            self.state[mid]['velocity'] is not None
            for mid in self.motor_ids
        )

    def loop_cb(self):
        if self.finished:
            return

        # limpiar fallas al inicio
        if not self.started:
            for mid in self.motor_ids:
                self.publish_stop(mid)

            if self.start_delay_cycles > 0:
                self.start_delay_cycles -= 1
                return

            if not self.all_states_received():
                self.get_logger().info('Aún esperando estados de los motores...')
                return

            self.started = True
            self.get_logger().info('Estados recibidos, comenzando rutina')
            return

        current_motor = self.motor_ids[self.motor_index]

        if self.pause_count > 0:
            self.pause_count -= 1
            return

        current_target = self.targets[self.target_index]
        self.publish_position(current_motor, current_target)

        pos = self.state[current_motor]['position']
        vel = self.state[current_motor]['velocity']

        if abs(pos - current_target) < self.pos_tol and abs(vel) < self.vel_tol:
            self.ok_count += 1
        else:
            self.ok_count = 0

        if self.ok_count >= self.required_ok_cycles:
            self.get_logger().info(
                f'Motor {current_motor} llegó a {current_target:.3f} rev '
                f'(pos={pos:.4f}, vel={vel:.4f})'
            )
            self.ok_count = 0
            self.pause_count = self.pause_cycles
            self.target_index += 1

            if self.target_index >= len(self.targets):
                self.publish_stop(current_motor)
                self.get_logger().info(f'Motor {current_motor} detenido')
                self.target_index = 0
                self.motor_index += 1

                if self.motor_index >= len(self.motor_ids):
                    self.finished = True
                    self.get_logger().info('Rutina completada en todos los motores')


def main(args=None):
    rclpy.init(args=args)
    node = MoteusSequentialRoutine()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()