#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
from tf_transformations import quaternion_from_euler, quaternion_multiply

class OdometryRotator(Node):
    def __init__(self):
        super().__init__('odometry_rotator')
        # Subscribe to /odometry/local topic.
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/local',
            self.odom_callback,
            10)
        # Publisher for the new odometry message with rotated orientation.
        self.publisher = self.create_publisher(Odometry, '/odometry/local1', 10)
        self.get_logger().info('Odometry Rotator Node has started.')

    def odom_callback(self, msg):
        # Create a new Odometry message to republish.
        new_msg = Odometry()
        new_msg.header = msg.header
        new_msg.child_frame_id = msg.child_frame_id
        
        # Copy the position from the original message.
        new_msg.pose.pose.position.x = msg.pose.pose.position.x
        new_msg.pose.pose.position.y = msg.pose.pose.position.y
        new_msg.pose.pose.position.z = msg.pose.pose.position.z
        
        # Get the original orientation as a quaternion.
        orig_q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        
        # Create an offset quaternion corresponding to a 90° (π/2) rotation about the Z-axis.
        # The order here applies the offset rotation first: new_q = offset_q * orig_q.
        yaw_offset = -math.pi / 2.0  # 90° in radians.
        offset_q = quaternion_from_euler(0, 0, yaw_offset)
        
        # Multiply the offset quaternion with the original to add the 90° yaw offset.
        new_q = quaternion_multiply(offset_q, orig_q)
        
        # Update the orientation with the new quaternion.
        new_msg.pose.pose.orientation.x = new_q[0]
        new_msg.pose.pose.orientation.y = new_q[1]
        new_msg.pose.pose.orientation.z = new_q[2]
        new_msg.pose.pose.orientation.w = new_q[3]
        
        # Copy the twist (velocity) information as well.
        new_msg.twist = msg.twist
        
        # Publish the modified odometry message.
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryRotator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
