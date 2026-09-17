#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from erp_interfaces.msg import ErpCmdMsg, ErpStatusMsg
import serial
import struct
import numpy as np

START_BITS = "535458"

class ERPHandler(Node):
    def __init__(self):
        super().__init__('erp_base')
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        self.get_logger().info(f'Connecting to {port} at {baudrate} baud')
        self.serial = serial.Serial(port=port, baudrate=baudrate)
        self.alive = 0
        
        self.erp_status_pub = self.create_publisher(ErpStatusMsg, '/erp42_status', 3)
        self.erp_cmd_sub = self.create_subscription(ErpCmdMsg, '/erp42_ctrl_cmd', self.send_packet, 3)
        
        self.timer = self.create_timer(0.025, self.serial_send)  # 40Hz
        self.packet = ErpCmdMsg()

    def recv_packet(self):
        packet = self.serial.read(18)
        if not packet.hex().find(START_BITS) == 0:
            end, data = packet.hex().split(START_BITS)
            packet = bytes.fromhex(START_BITS + data + end)
        
        msg = self.packet_to_msg(packet)
        self.erp_status_pub.publish(msg)

    def send_packet(self, msg):
        self.packet = msg

    def serial_send(self):
        packet = self.msg_to_packet(self.packet, self.alive)
        self.serial.write(packet)
        self.alive = (self.alive + 1) % 256
        self.recv_packet()
    
    def packet_to_msg(self, _byte):
        formated_packet = struct.unpack('<BBBBBBhhBiBBB', _byte)
        msg = ErpStatusMsg()
        msg.control_mode = formated_packet[3]
        msg.e_stop = bool(formated_packet[4])
        msg.gear = formated_packet[5]
        msg.speed = formated_packet[6]
        msg.steer = -formated_packet[7]
        msg.brake = formated_packet[8]
        msg.encoder = int(formated_packet[9])
        msg.alive = formated_packet[10]
        return msg
    
    def msg_to_packet(self, msg, alive):
        header = "STX".encode()
        tail = "\r\n".encode()
        
        data = struct.pack(
            ">BBBHhBB", 1,
            msg.e_stop,
            msg.gear,
            msg.speed,
            msg.steer,
            msg.brake,
            alive
        )
        return header + data + tail


def main(args=None):
    rclpy.init(args=args)
    erp_handler = ERPHandler()
    rclpy.spin(erp_handler)
    erp_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
