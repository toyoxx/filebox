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

# ROS2 message and service types
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String

# Inspire hand library
from inspire_hand import InspireHand


class InspireHandNode(Node):
    """
    ROS2 wrapper node for Inspire RH56DFQ-2L hand.
    Provides simple service interfaces for basic hand behaviors.
    """

    def __init__(self):
        super().__init__('inspire_hand_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('default_force', 500)
        self.declare_parameter('default_speed', 800)
        self.declare_parameter('status_rate', 10.0)  # Hz

        # Load parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        slave_id = self.get_parameter('slave_id').value
        self.default_force = self.get_parameter('default_force').value
        self.default_speed = self.get_parameter('default_speed').value
        status_rate = self.get_parameter('status_rate').value

        # Initialize hand
        try:
            self.hand = InspireHand(port=port, baudrate=baudrate, slave_id=slave_id)
            self.get_logger().info(f'Connected to Inspire hand on {port} (baudrate={baudrate})')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Inspire hand: {e}')
            raise e

        # Define ROS services
        self.srv_open_all = self.create_service(Trigger, 'open_all', self.handle_open_all)
        self.srv_close_all = self.create_service(Trigger, 'close_all', self.handle_close_all)
        self.srv_pinch = self.create_service(SetBool, 'pinch', self.handle_pinch)
        self.srv_point = self.create_service(Trigger, 'point', self.handle_point)
        self.srv_thumbs_up = self.create_service(Trigger, 'thumbs_up', self.handle_thumbs_up)
        self.srv_grip = self.create_service(SetBool, 'grip', self.handle_grip)

        # Publishers for feedback
        self.pub_status = self.create_publisher(String, 'hand_status', 10)
        self.pub_angles = self.create_publisher(String, 'finger_angles', 10)

        # Periodic publisher
        self.timer = self.create_timer(1.0 / status_rate, self.publish_status)

        self.get_logger().info('InspireHandNode initialized successfully.')

    # === Service callbacks ===

    def handle_open_all(self, request, response):
        try:
            self.hand.open_all_fingers()
            response.success = True
            response.message = 'Opened all fingers.'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def handle_close_all(self, request, response):
        try:
            self.hand.close_all_fingers()
            response.success = True
            response.message = 'Closed all fingers.'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def handle_pinch(self, request, response):
        try:
            if request.data:
                self.hand.pinch(force=self.default_force)
                response.message = f'Pinch with force {self.default_force}.'
            else:
                self.hand.open_all_fingers()
                response.message = 'Pinch released (opened fingers).'
            response.success = True
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def handle_point(self, request, response):
        try:
            self.hand.point()
            response.success = True
            response.message = 'Point gesture executed.'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def handle_thumbs_up(self, request, response):
        try:
            self.hand.thumbs_up()
            response.success = True
            response.message = 'Thumbs up gesture executed.'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def handle_grip(self, request, response):
        try:
            if request.data:
                self.hand.grip(force=self.default_force)
                response.message = f'Grip with force {self.default_force}.'
            else:
                self.hand.open_all_fingers()
                response.message = 'Grip released (opened fingers).'
            response.success = True
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # === Status publisher ===

    def publish_status(self):
        try:
            angles = self.hand.get_finger_angles()
            status_msg = f'Angles: {angles}'
            self.pub_angles.publish(String(data=str(angles)))
            self.pub_status.publish(String(data=status_msg))
        except Exception as e:
            self.get_logger().warn(f'Failed to read status: {e}')

    def destroy_node(self):
        try:
            self.hand.close()
        except Exception:
            pass
        super().destroy_node()
        self.get_logger().info('InspireHandNode shut down cleanly.')


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
