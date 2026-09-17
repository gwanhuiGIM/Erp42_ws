#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from struct import pack
from erp_interfaces.msg import ErpStatusMsg, ErpCmdMsg

import serial
import numpy as np
from ByteHandler import ErpMsg2Packet, Packet2ErpMsg

START_BITS = b'\x53\x54\x58'  # "STX" in hex
PACKET_SIZE = 18  # Expected packet size from ERP-42


class ERPHandler(Node):
    def __init__(self):
        super().__init__("erp_base")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        _port = self.get_parameter("port").value
        _baudrate = self.get_parameter("baudrate").value

        self.get_logger().info(f"erp_base::Uart Port : {_port}")
        self.get_logger().info(f"erp_base::Baudrate  : {_baudrate}")

        try:
            self.serial = serial.Serial(port=_port, baudrate=_baudrate, timeout=0.1)
            self.get_logger().info(f"Serial {_port} Connected")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            rclpy.shutdown()
            return

        self.alive = 0
        self.packet = ErpCmdMsg()
        self.packet.gear = 0
        self.packet.e_stop = False
        self.packet.brake = 1

        self.erpMotionMsg_pub = self.create_publisher(
            ErpStatusMsg, "/erp42_status", 3
        )
        self.erpCmdMsg_sub = self.create_subscription(
            ErpCmdMsg, "/erp42_ctrl_cmd", self.send_packet, 10
        )

        self.timer = self.create_timer(1.0 / 40, self.run_loop)  # 40 Hz

    def recv_packet(self):
        try:
            packet = self.serial.read(PACKET_SIZE)

            if len(packet) != PACKET_SIZE:
                self.get_logger().warn(f"Incomplete packet received: {packet.hex()}")
                return

            if not packet.startswith(START_BITS):
                self.get_logger().warn("Packet does not start with STX, discarding.")
                return

            msg = Packet2ErpMsg(packet)
            self.erpMotionMsg_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Failed to unpack serial data: {e}")

    def send_packet(self, _data: ErpCmdMsg):
        self.packet = _data

    def serial_send(self):
        try:
            packet = ErpMsg2Packet(self.packet, self.alive)
            self.serial.write(packet)
            self.alive = (self.alive + 1) % 256
        except Exception as e:
            self.get_logger().error(f"Failed to send serial data: {e}")

    def run_loop(self):
        self.recv_packet()
        self.serial_send()


def main(args=None):
    rclpy.init(args=args)
    node = ERPHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
