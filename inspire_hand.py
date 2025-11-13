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

        # Parameters
        self.declare_parameter('serial_port_left', '/dev/ttyUSB0')
        self.declare_parameter('serial_port_right', '/dev/ttyUSB1')
        self.declare_parameter('force', 100)
        self.declare_parameter('speed', 50)
        self.declare_parameter('connection_delay', 1.0)

        self.serial_port_left = self.get_parameter('serial_port_left').value
        self.serial_port_right = self.get_parameter('serial_port_right').value
        self.force = self.get_parameter('force').value
        self.speed = self.get_parameter('speed').value
        self.connection_delay = self.get_parameter('connection_delay').value

        # Hands dictionary
        self.hands = {}
        self.lock = threading.Lock()

        # Connect both hands
        self._connect_hand('left', self.serial_port_left)
        self._connect_hand('right', self.serial_port_right)

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'inspire_hand/joint_states', 10)

        # Register services for both hands
        for side in ['left', 'right']:
            self._register_services(side)

        # Start background thread for publishing joint states
        self.running = True
        self.state_thread = threading.Thread(target=self._state_publisher, daemon=True)
        self.state_thread.start()

        self.get_logger().info("Inspire hand node initialized with dual-hand support.")

    # ---- Connect Hand ----
    def _connect_hand(self, side, port):
        try:
            hand = Hand(port)
            hand.open()  # first version logic
            time.sleep(self.connection_delay)
            hand.set_speed(self.speed)
            hand.set_force(self.force)
            self.hands[side] = hand
            self.get_logger().info(f"Connected {side} hand on {port}")
        except Exception as e:
            self.hands[side] = None
            self.get_logger().error(f"Failed to connect {side} hand on {port}: {e}")

    # ---- Register Services ----
    def _register_services(self, side):
        behaviors = ['open', 'close', 'reset', 'pinch', 'point', 'thumbs_up', 'grip']
        for behavior in behaviors:
            self.create_service(
                Trigger,
                f'inspire_hand/{side}/{behavior}',
                lambda req, s=side, b=behavior: self._service_callback(req, s, b)
            )

    # ---- Generic Service Callback ----
    def _service_callback(self, request, side, behavior):
        hand = self.hands.get(side)
        if not hand:
            return Trigger.Response(success=False, message=f"{side} hand not connected.")

        try:
            # Apply speed/force
            hand.set_speed(self.speed)
            hand.set_force(self.force)

            # Use the SDK primitive behaviors
            if behavior == 'reset':
                hand.open()
            else:
                getattr(hand, behavior)()  # open(), close(), pinch(), point(), thumbs_up(), grip()

            return Trigger.Response(success=True, message=f"{side} hand {behavior} executed.")
        except Exception as e:
            return Trigger.Response(success=False, message=f"{side} hand {behavior} failed: {e}")

    # ---- State Publisher ----
    def _state_publisher(self):
        while self.running and rclpy.ok():
            with self.lock:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = []
                msg.position = []

                for side, hand in self.hands.items():
                    if hand is None:
                        continue
                    try:
                        positions = hand.get_positions()  # first version logic
                        for i, pos in enumerate(positions):
                            msg.name.append(f"{side}_finger_{i+1}")
                            msg.position.append(pos)
                    except Exception as e:
                        self.get_logger().warn(f"Lost connection to {side} hand: {e}")
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


