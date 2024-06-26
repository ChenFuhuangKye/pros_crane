"""
PROS Arm Commercial License
Copyright 2023 帕亞科技 (PAIA-Tech) . All rights reserved.
Primary Author: 陳麒麟(Kylin Chen)
Contributor(s): 蘇文鈺(Alvin Su) ,
This software and associated documentation files are proprietary to 帕亞科技 (PAIA-Tech),
and are not to be copied, reproduced, or disclosed to any third party, in whole or in part, without
express prior written permission. Use and modification of this software is permitted for licensed
users only, and must always remain proprietary to 帕亞科技 (PAIA-Tech).


This code is a node to read data from specific arm.
It will publish arm state.

"""

import orjson

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from rclpy.duration import Duration

from serial import Serial
from .env_virtual import ARM_SERIAL_PORT_DEFAULT


class ArmSerialReader(Node):
    def __init__(self):
        super().__init__('arm_serial_reader')

        # Set up the serial connection
        serial_port = self.declare_parameter('serial_port', ARM_SERIAL_PORT_DEFAULT).value
        self._serial = Serial(serial_port, 115200, timeout=0)

        # Create a publisher for the serial data
        # TODO dynamic to adjust arm
        self._publisher = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self._timer = self.create_timer(0.1, self.reader_callback)
        self._joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        self._position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.log_interval = Duration(seconds=0.5)  # Log every 1 seconds
        current_time = self.get_clock().now()
        self.last_log_time = current_time

        # Set up a timer to read data from the serial device
        # 每 n 秒收一次資料

    def reader_callback(self):

        # Read data from the serial device
        data = self._serial.readline()
        try:
            # esp32_json_str = data.decode('utf-8')
            degree_data = orjson.loads(data)
            degree_positions = degree_data["servo_current_angles"]  # Assuming this is the key in your JSON

        except orjson.JSONDecodeError as error:
            self.get_logger().error(f"Json decode error when recv {data}")
            return
        except KeyError as error:
            self.get_logger().error(f"KeyError when recv {degree_data}")
            return
        except UnicodeDecodeError as error:
            self.get_logger().error(f"UnicodeDecodeError when recv {data}")
            return

        # esp32_json_obj -> position

        # Convert degree positions to radians
        radian_positions = [math.radians(deg) for deg in degree_positions]

        # Publish the data to the serial_data topic
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = radian_positions
        msg.velocity = []
        msg.effort = []

        self._publisher.publish(msg)
        current_time = self.get_clock().now()
        if current_time - self.last_log_time >= self.log_interval:
            self.get_logger().info(f'Receive from arm esp32: {degree_data}')
            self.last_log_time = current_time


def main(args=None):
    rclpy.init(args=args)

    serial_reader = ArmSerialReader()
    rclpy.spin(serial_reader)

    serial_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
