#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from tf_transformations import quaternion_from_euler, quaternion_multiply

class ImuYawOffsetNode(Node):
    def __init__(self):
        super().__init__('imu_yaw_offset_node')
        # Subscribe to the original IMU topic.
        self.subscription = self.create_subscription(
            Imu,
            '/vectornav/imu',
            self.imu_callback,
            50)
        # Publish the modified IMU message to a new topic.
        self.publisher = self.create_publisher(Imu, '/vectornav/imu_offset', 50)
        
        # Compute the offset quaternion corresponding to -90° yaw.
        # Negative 90 degrees in radians:
        yaw_offset = math.pi / 2   
        self.offset_q = quaternion_from_euler(0, 0, yaw_offset)
        self.get_logger().info("IMU yaw offset node initialized with -90° offset.")

    def imu_callback(self, msg: Imu):
        # Extract the original orientation as a quaternion.
        orig_q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        
        # Multiply the offset quaternion with the original orientation.
        # Order: new_q = offset_q * orig_q.
        new_q = quaternion_multiply(self.offset_q, orig_q)
        
        # Create a new IMU message and assign header and modified orientation.
        new_msg = Imu()
        new_msg.header = msg.header
        new_msg.orientation.x = new_q[0]
        new_msg.orientation.y = new_q[1]
        new_msg.orientation.z = new_q[2]
        new_msg.orientation.w = new_q[3]
        
        # Copy over the angular velocity and linear acceleration fields.
        new_msg.angular_velocity = msg.angular_velocity
        new_msg.linear_acceleration = msg.linear_acceleration

        # Also copy covariance matrices if needed.
        new_msg.orientation_covariance = msg.orientation_covariance
        new_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        new_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # Publish the modified message.
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuYawOffsetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
