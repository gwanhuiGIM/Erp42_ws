#! /usr/bin/env python3

import struct
import numpy as np
from erp_interfaces.msg import ErpStatusMsg, ErpCmdMsg  # ROS2 message definitions

# Constants
EXPECTED_PACKET_SIZE = 18  # Ensure this matches the ERP-42 protocol
HEADER = b"STX"
TAIL = b"\r\n"


def Packet2ErpMsg(byte_data: bytes) -> ErpStatusMsg:
    """
    Converts raw bytes into an ErpStatusMsg.
    """
    if len(byte_data) != EXPECTED_PACKET_SIZE:
        print(f"Warning: Incomplete packet received ({len(byte_data)} bytes): {byte_data.hex()}")
        return None  # Return None to indicate a failed conversion
    
    try:
        # Adjust struct format based on ERP-42 message definition
        formatted_packet = struct.unpack('<BBBBBBhhBiBBBB', byte_data)
        
        msg = ErpStatusMsg()
        msg.control_mode = formatted_packet[3]
        msg.e_stop = bool(formatted_packet[4])
        msg.gear = formatted_packet[5]
        msg.speed = formatted_packet[6]
        msg.steer = -formatted_packet[7]  # Invert steering if needed
        msg.brake = formatted_packet[8]
        msg.encoder = int(np.int32(formatted_packet[9]))  # Ensure proper type casting
        msg.alive = formatted_packet[10]
        
        return msg
    except struct.error as e:
        print(f"Error: Failed to unpack serial data: {e}")
        return None


def ErpMsg2Packet(msg: ErpCmdMsg, alive: int) -> bytes:
    """
    Converts an ErpCmdMsg into a raw byte packet.
    """
    try:
        # Ensure values are within valid ranges (add checks if necessary)
        data = struct.pack(
            '>BBBHhBB',
            1,  # Packet type
            msg.e_stop,
            msg.gear,
            msg.speed,
            msg.steer,
            msg.brake,
            alive % 256  # Keep alive within 0-255
        )
        packet = HEADER + data + TAIL
        return packet
    except struct.error as e:
        print(f"Error: Failed to pack serial data: {e}")
        return None
