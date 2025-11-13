#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# ROS2 node wrapper for Inspire RH56DFQ-2L hands (dual-hand setup)
# Author: Jose Victorio Salazar (2025)
#
# This node exposes both left and right Inspire hands to ROS2,
# following the g1pilot/manipulation/dx3_hand.py structure,
# using the .open() trick to initialize each hand.

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String

from inspire_hand import InspireHand


class InspireHandNode(Node):
    """
    Dual-hand ROS2 wrapper for Inspire RH56DFQ-2L hands.
    Uses hand.open() in constructor to initialize connection.
    """

    def __init__(self):
        super().__init__('inspire_hand_node')

        # === Parameters ===
        self.declare_parameter('serial_port_left', '/dev/ttyUSB1')
        self.declare_parameter('serial_port_right', '/dev/ttyUSB2')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('slave_id_left', 1)
        self.declare_parameter('slave_id_right', 2)
        self.declare_parameter('default_force', 500)
        self.declare_parameter('default_speed', 800)
        self.declare_parameter('status_rate', 10.0)  # Hz

        # Load params
        serial_left = self.get_parameter('serial_port_left').value
        serial_right = self.get_parameter('serial_port_right').value
        baudrate = self.get_parameter('baudrate').value
        slave_left = self.get_parameter('slave_id_left').value
        slave_right = self.get_parameter('slave_id_right').value
        self.default_force = self.get_parameter('default_force').value
        self.default_speed = self.get_parameter('default_speed').value
        rate = self.get_parameter('status_rate').value

        # === Initialize Hands ===
        try:
            self.hand_left = InspireHand(port=serial_left, baudrate=baudrate, slave_id=slave_left)
            self.hand_left.open()  # initialize connection
            self.get_logger().info(f'Connected and initialized LEFT hand on {serial_left}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect LEFT hand: {e}')
            self.hand_left = None

        try:
            self.hand_right = InspireHand(port=serial_right, baudrate=baudrate, slave_id=slave_right)
            self.hand_right.open()  # initialize connection
            self.get_logger().info(f'Connected and initialized RIGHT hand on {serial_right}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect RIGHT hand: {e}')
            self.hand_right = None

        # === Services ===
        # Left hand
        self.create_service(Trigger, 'left_hand/open_all', self._wrap(self._open_all, 'left'))
        self.create_service(Trigger, 'left_hand/close_all', self._wrap(self._close_all, 'left'))
        self.create_service(SetBool, 'left_hand/pinch', self._wrap(self._pinch, 'left'))
        self.create_service(SetBool, 'left_hand/grip', self._wrap(self._grip, 'left'))
        self.create_service(Trigger, 'left_hand/point', self._wrap(self._point, 'left'))
        self.create_service(Trigger, 'left_hand/thumbs_up', self._wrap(self._thumbs_up, 'left'))

        # Right hand
        self.create_service(Trigger, 'right_hand/open_all', self._wrap(self._open_all, 'right'))
        self.create_service(Trigger, 'right_hand/close_all', self._wrap(self._close_all, 'right'))
        self.create_service(SetBool, 'right_hand/pinch', self._wrap(self._pinch, 'right'))
        self.create_service(SetBool, 'right_hand/grip', self._wrap(self._grip, 'right'))
        self.create_service(Trigger, 'right_hand/point', self._wrap(self._point, 'right'))
        self.create_service(Trigger, 'right_hand/thumbs_up', self._wrap(self._thumbs_up, 'right'))

        # === Publishers ===
        self.pub_status = self.create_publisher(String, 'hands/status', 10)
        self.pub_angles_left = self.create_publisher(String, 'left_hand/angles', 10)
        self.pub_angles_right = self.create_publisher(String, 'right_hand/angles', 10)

        # === Timer ===
        self.timer = self.create_timer(1.0 / rate, self.publish_status)

        self.get_logger().info('InspireHandNode ready (dual-hand mode).')

    # === Service wrapper ===
    def _wrap(self, func, side):
        def inner(request, response):
            try:
                func(side, request, response)
            except Exception as e:
                response.success = False
                response.message = str(e)
            return response
        return inner

    # === Core operations ===
    def _get_hand(self, side):
        if side == 'left':
            if not self.hand_left:
                raise RuntimeError('Left hand not connected.')
            return self.hand_left
        else:
            if not self.hand_right:
                raise RuntimeError('Right hand not connected.')
            return self.hand_right

    def _open_all(self, side, request, response):
        hand = self._get_hand(side)
        hand.open_all_fingers()
        response.success = True
        response.message = f'{side.capitalize()} hand opened.'

    def _close_all(self, side, request, response):
        hand = self._get_hand(side)
        hand.close_all_fingers()
        response.success = True
        response.message = f'{side.capitalize()} hand closed.'

    def _pinch(self, side, request, response):
        hand = self._get_hand(side)
        if request.data:
            hand.pinch(force=self.default_force)
            msg = f'{side.capitalize()} hand pinch (force={self.default_force}).'
        else:
            hand.open_all_fingers()
            msg = f'{side.capitalize()} hand released pinch.'
        response.success = True
        response.message = msg

    def _grip(self, side, request, response):
        hand = self._get_hand(side)
        if request.data:
            hand.grip(force=self.default_force)
            msg = f'{side.capitalize()} hand grip (force={self.default_force}).'
        else:
            hand.open_all_fingers()
            msg = f'{side.capitalize()} hand released grip.'
        response.success = True
        response.message = msg

    def _point(self, side, request, response):
        hand = self._get_hand(side)
        hand.point()
        response.success = True
        response.message = f'{side.capitalize()} hand point gesture.'

    def _thumbs_up(self, side, request, response):
        hand = self._get_hand(side)
        hand.thumbs_up()
        response.success = True
        response.message = f'{side.capitalize()} hand thumbs-up gesture.'

    # === Status ===
    def publish_status(self):
        try:
            left_angles = self.hand_left.get_all_angles() if self.hand_left else None
            right_angles = self.hand_right.get_all_angles() if self.hand_right else None

            if left_angles:
                self.pub_angles_left.publish(String(data=str(left_angles)))
            if right_angles:
                self.pub_angles_right.publish(String(data=str(right_angles)))

            self.pub_status.publish(
                String(data=f'L: {left_angles} | R: {right_angles}')
            )
        except Exception as e:
            self.get_logger().warn(f'Status read error: {e}')

    def destroy_node(self):
        for hand in [self.hand_left, self.hand_right]:
            try:
                if hand:
                    hand.close()
            except Exception:
                pass
        super().destroy_node()
        self.get_logger().info('InspireHandNode terminated.')


def main(args=None):
    rclpy.init(args=args)
    node = InspireHandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
