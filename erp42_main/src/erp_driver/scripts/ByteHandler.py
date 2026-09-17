#! /usr/bin/env python3

import struct
import numpy as np
from erp_interfaces.msg import ErpStatusMsg, ErpCmdMsg

HEADER = b"STX"  # 3-byte header
TAIL = b"\r\n"  # 2-byte tail
PACKET_FORMAT = "<BBBBBBhhBiBBB"  # 18-byte format


def Packet2ErpMsg(byte_data: bytes) -> ErpStatusMsg:
    try:
        if len(byte_data) != 18:
            raise ValueError(f"Invalid packet size: {len(byte_data)} bytes")

        formated_packet = struct.unpack(PACKET_FORMAT, byte_data)

        msg = ErpStatusMsg()
        msg.control_mode = formated_packet[3]
        msg.e_stop = bool(formated_packet[4])
        msg.gear = formated_packet[5]
        msg.speed = formated_packet[6]
        msg.steer = -formated_packet[7]
        msg.brake = formated_packet[8]
        msg.encoder = int(np.int32(formated_packet[9]))  # Ensure correct integer type
        msg.alive = formated_packet[10]

        return msg

    except struct.error as e:
        print(f"Packet unpacking error: {e}")
        return ErpStatusMsg()  # Return empty message to avoid crashes


def ErpMsg2Packet(msg: ErpCmdMsg, alive: int) -> bytes:
    try:
        data = struct.pack(
            "<BBBHhBB",
            1,
            msg.e_stop,
            msg.gear,
            msg.speed,
            msg.steer,
            msg.brake,
            alive & 0xFF,  # Ensure alive is 8-bit
        )
        return HEADER + data + TAIL

    except struct.error as e:
        print(f"Packet packing error: {e}")
        return b""  # Return empty packet to avoid errors
