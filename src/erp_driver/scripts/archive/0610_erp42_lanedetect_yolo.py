#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from erp_interfaces.msg import ErpCmdMsg
from std_msgs.msg import Header

from yolo_msgs.msg import DetectionArray  # Adjust if your message is different
from yolo_msgs.msg import Detection

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FIXED_SPEED = 15 * 10  # ERP42 speed is KPH * 10
STEER_GAIN = 7.0       # P-controller gain for steering (tune as needed)
MAX_STEER = 2000       # ERP42 max steer value (+/-)

class LaneCenteringNode(Node):
    def __init__(self):
        super().__init__('lane_centering_node')
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.yolo_callback,
            10
        )
        self.cmd_pub = self.create_publisher(ErpCmdMsg, '/erp42_ctrl_cmd/lane', 10)
        self.image_center_x = IMAGE_WIDTH / 2

    def yolo_callback(self, msg):
        # Find lane detections
        left_lane = None
        right_lane = None
        min_left_dist = float('inf')
        min_right_dist = float('inf')

        for det in msg.detections:
            # Support both custom and vision_msgs formats
            # Adjust below if your custom message is different!
            try:
                class_name = det.results[0].hypothesis.class_id if hasattr(det, "results") else det.class_name
            except Exception:
                class_name = getattr(det, "class_name", "lane")
            if class_name != "lane":
                continue
            # Get bbox center x
            if hasattr(det, 'bbox'):
                bbox = det.bbox
                if hasattr(bbox, "center"):
                    cx = bbox.center.position.x
                else:  # fallback for custom format
                    cx = bbox.center.x
            else:
                continue

            dist = cx - self.image_center_x
            if cx < self.image_center_x and abs(dist) < min_left_dist:
                left_lane = cx
                min_left_dist = abs(dist)
            elif cx > self.image_center_x and abs(dist) < min_right_dist:
                right_lane = cx
                min_right_dist = abs(dist)

        # Calculate lane center
        if left_lane is not None and right_lane is not None:
            lane_center_x = (left_lane + right_lane) / 2
        elif left_lane is not None:
            lane_center_x = left_lane  # fallback
        elif right_lane is not None:
            lane_center_x = right_lane  # fallback
        else:
            self.stop_vehicle()
            self.get_logger().info("No lane detected, stopping.")
            return

        error = self.image_center_x - lane_center_x

        # Simple P-control for steering
        steer_cmd = int(STEER_GAIN * error)
        steer_cmd = max(min(steer_cmd, MAX_STEER), -MAX_STEER)

        cmd = ErpCmdMsg()
        cmd.e_stop = False
        cmd.gear = 0
        cmd.speed = FIXED_SPEED
        cmd.steer = steer_cmd
        cmd.brake = 1  # No braking

        self.cmd_pub.publish(cmd)

    def stop_vehicle(self):
        cmd = ErpCmdMsg()
        cmd.e_stop = False
        cmd.gear = 0
        cmd.speed = 0
        cmd.steer = 0
        cmd.brake = 155
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LaneCenteringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

