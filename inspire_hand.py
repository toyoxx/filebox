#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# ROS2 node wrapper for Inspire RH56DFQ-2L hand
# Author: Jose Victorio Salazar (2025)
#
# This node exposes the Inspire hand control API as ROS2 services,
# following the same structure as g1pilot/manipulation/dx3_hand.py.

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from inspire_hand import Hand
import threading
import time


class InspireHandNode(Node):
    def __init__(self):
        super().__init__('inspire_hand')

        # Serial port parameters
        self.declare_parameter('serial_port_left', '/dev/ttyUSB0')
        self.declare_parameter('serial_port_right', '/dev/ttyUSB1')

        # Movement parameters
        self.declare_parameter('force', 100)
        self.declare_parameter('speed', 50)

        left_port = self.get_parameter('serial_port_left').get_parameter_value().string_value
        right_port = self.get_parameter('serial_port_right').get_parameter_value().string_value
        self.force = self.get_parameter('force').get_parameter_value().integer_value
        self.speed = self.get_parameter('speed').get_parameter_value().integer_value

        self.hands = {}
        self.lock = threading.Lock()

        # Connect to both hands
        self._connect_hand('left', left_port)
        self._connect_hand('right', right_port)

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'inspire_hand/joint_states', 10)

        # Register behavior services for each hand
        for side in ['left', 'right']:
            self._register_services(side)

        # Background thread for joint states
        self.running = True
        self.state_thread = threading.Thread(target=self._state_publisher, daemon=True)
        self.state_thread.start()

        self.get_logger().info("InspireHandNode initialized with dual-hand and parameterized control.")

    # ---- Initialization ----
    def _connect_hand(self, side, port):
        try:
            hand = Hand(port)
            hand.open()  # ensures communication starts
            self.hands[side] = hand
            self.get_logger().info(f"Connected to {side} hand on {port}")
        except Exception as e:
            self.hands[side] = None
            self.get_logger().warn(f"Could not connect to {side} hand on {port}: {e}")

    def _register_services(self, side):
        self.create_service(Trigger, f'inspire_hand/{side}/open', lambda req, s=side: self._srv_open(req, s))
        self.create_service(Trigger, f'inspire_hand/{side}/close', lambda req, s=side: self._srv_close(req, s))
        self.create_service(Trigger, f'inspire_hand/{side}/reset', lambda req, s=side: self._srv_reset(req, s))
        self.create_service(Trigger, f'inspire_hand/{side}/pinch', lambda req, s=side: self._srv_pinch(req, s))
        self.create_service(Trigger, f'inspire_hand/{side}/point', lambda req, s=side: self._srv_point(req, s))
        self.create_service(Trigger, f'inspire_hand/{side}/thumbs_up', lambda req, s=side: self._srv_thumbs_up(req, s))
        self.create_service(Trigger, f'inspire_hand/{side}/grip', lambda req, s=side: self._srv_grip(req, s))

    # ---- Helpers ----
    def _error_response(self, msg):
        self.get_logger().error(msg)
        return Trigger.Response(success=False, message=msg)

    def _success_response(self, msg):
        self.get_logger().info(msg)
        return Trigger.Response(success=True, message=msg)

    def _get_hand(self, side):
        hand = self.hands.get(side)
        if not hand:
            self.get_logger().warn(f"{side} hand not connected.")
        return hand

    def _apply_force_speed(self, hand):
        try:
            hand.set_force(self.force)
            hand.set_speed(self.speed)
        except AttributeError:
            # Some Inspire drivers use different names, e.g. set_grip_force() / set_grip_speed()
            try:
                hand.set_grip_force(self.force)
                hand.set_grip_speed(self.speed)
            except Exception as e:
                self.get_logger().warn(f"Force/speed control not available: {e}")

    # ---- Basic Services ----
    def _srv_open(self, request, side):
        hand = self._get_hand(side)
        if not hand:
            return self._error_response(f"{side} hand not connected.")
        self._apply_force_speed(hand)
        hand.open()
        return self._success_response(f"{side} hand opened (force={self.force}, speed={self.speed}).")

    def _srv_close(self, request, side):
        hand = self._get_hand(side)
        if not hand:
            return self._error_response(f"{side} hand not connected.")
        self._apply_force_speed(hand)
        hand.close()
        return self._success_response(f"{side} hand closed (force={self.force}, speed={self.speed}).")

    def _srv_reset(self, request, side):
        hand = self._get_hand(side)
        if not hand:
            return self._error_response(f"{side} hand not connected.")
        self._apply_force_speed(hand)
        hand.open()
        return self._success_response(f"{side} hand reset.")

    # ---- Behavior Services ----
    def _srv_pinch(self, request, side):
        hand = self._get_hand(side)
        if not hand:
            return self._error_response(f"{side} hand not connected.")
        self._apply_force_speed(hand)
        try:
            # Close thumb + index
            hand.move_finger(0, 100, self.force, self.speed)
            hand.move_finger(1, 100, self.force, self.speed)
            for i in range(2, 5):
                hand.move_finger(i, 0, self.force, self.speed)
            return self._success_response(f"{side} hand pinch gesture executed.")
        except Exception as e:
            return self._error_response(str(e))

    def _srv_point(self, request, side):
        hand = self._get_hand(side)
        if not hand:
            return self._error_response(f"{side} hand not connected.")
        self._apply_force_speed(hand)
        try:
            # Extend index finger only
            hand.move_finger(1, 0, self.force, self.speed)
            for i in [0, 2, 3, 4]:
                hand.move_finger(i, 100, self.force, self.speed)
            return self._success_response(f"{side} hand point gesture executed.")
        except Exception as e:
            return self._error_response(str(e))

    def _srv_thumbs_up(self, request, side):
        hand = self._get_hand(side)
        if not hand:
            return self._error_response(f"{side} hand not connected.")
        self._apply_force_speed(hand)
        try:
            # Extend thumb, close others
            hand.move_finger(0, 0, self.force, self.speed)
            for i in range(1, 5):
                hand.move_finger(i, 100, self.force, self.speed)
            return self._success_response(f"{side} hand thumbs-up gesture executed.")
        except Exception as e:
            return self._error_response(str(e))

    def _srv_grip(self, request, side):
        hand = self._get_hand(side)
        if not hand:
            return self._error_response(f"{side} hand not connected.")
        self._apply_force_speed(hand)
        try:
            # Full grip
            for i in range(5):
                hand.move_finger(i, 100, self.force, self.speed)
            return self._success_response(f"{side} hand grip gesture executed.")
        except Exception as e:
            return self._error_response(str(e))

    # ---- State Publisher ----
    def _state_publisher(self):
        rate = self.create_rate(10)
        while rclpy.ok() and self.running:
            with self.lock:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name, msg.position = [], []

                for side, hand in self.hands.items():
                    if not hand:
                        continue
                    try:
                        # Adapt depending on SDK: get_pos(), get_positions(), etc.
                        positions = hand.get_positions()
                        for i, pos in enumerate(positions):
                            msg.name.append(f"{side}_finger_{i+1}")
                            msg.position.append(pos)
                    except Exception:
                        self.get_logger().warn(f"Lost connection to {side} hand.")
                        self.hands[side] = None

                if msg.name:
                    self.joint_pub.publish(msg)

            time.sleep(0.1)

    # ---- Shutdown ----
    def destroy_node(self):
        self.running = False
        super().destroy_node()


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

