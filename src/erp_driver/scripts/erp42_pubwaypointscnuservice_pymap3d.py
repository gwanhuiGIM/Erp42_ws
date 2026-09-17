#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pandas as pd
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import pymap3d as pm  # <--- Use pymap3d for ENU conversion
from erp_interfaces.srv import SetOrigin

class WaypointPublisher(Node):
    def __init__(self, excel_path: str):
        super().__init__('waypoints_path_publisher')
        self.excel_path = excel_path
        self.origin_lon = None
        self.origin_lat = None
        self.origin_alt = 0.0  # Assume ground robots, altitude = 0 by default

        self.path_pub = self.create_publisher(Path, '/waypoints_path1', 10)
        self.srv = self.create_service(SetOrigin, '/set_origin', self.set_origin_callback)
        self.timer = self.create_timer(1.0, self.publish_path)

    def set_origin_callback(self, request, response):
        self.origin_lon = request.longitude
        self.origin_lat = request.latitude
        self.origin_alt = 0.0  # (set or let user define if needed)
        self.get_logger().info(
            f"Origin set to: Longitude={self.origin_lon}, Latitude={self.origin_lat}, Alt={self.origin_alt}"
        )
        response.success = True
        return response

    def load_and_convert_waypoints(self):
        if self.origin_lon is None or self.origin_lat is None:
            self.get_logger().warning(
                'Origin not set yet. Waiting for /set_origin service call...'
            )
            return []

        try:
            df = pd.read_excel(self.excel_path)
            if 'Longitude' not in df.columns or 'Latitude' not in df.columns:
                self.get_logger().error(
                    "Excel file must contain 'Longitude' and 'Latitude' columns."
                )
                return []

            waypoints = []
            for _, row in df.iterrows():
                # If you have altitude info, replace 0.0 below with row['Altitude'] or similar
                east, north, up = pm.geodetic2enu(
                    row['Latitude'], row['Longitude'], 0.0,      # waypoint lat, lon, alt
                    self.origin_lat, self.origin_lon, self.origin_alt  # origin lat, lon, alt
                )
                wp = PoseStamped()
                wp.header.frame_id = 'map'
                wp.header.stamp = self.get_clock().now().to_msg()
                wp.pose.position.x = float(east)
                wp.pose.position.y = float(north)
                wp.pose.position.z = 0.0
                wp.pose.orientation.w = 1.0
                waypoints.append(wp)
            return waypoints
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")
            return []

    def publish_path(self):
        if self.origin_lon is None or self.origin_lat is None:
            return

        waypoints = self.load_and_convert_waypoints()
        if not waypoints:
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = waypoints

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(waypoints)} waypoints.")

def main(args=None):
    rclpy.init(args=args)
    excel_path = '/home/mrlam/colcon_ws/bagfiles_ros2/hils/June_07/waypoint/waypoints_real_w2toe4_gps.xls'
    node = WaypointPublisher(excel_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
