#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from erp_interfaces.msg import ErpCmdMsg
from collections import deque
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis

class GradientEnhancedLaneDetector:
    def __init__(self, frame_width=1280, frame_height=720, edge_margin_frac=0.18):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.edge_margin_frac = edge_margin_frac
        self.training_data_frames = 10
        self.road_samples = deque(maxlen=500)
        self.white_lane_samples = deque(maxlen=500)
        self.yellow_lane_samples = deque(maxlen=500)
        self.white_lda = None
        self.yellow_lda = None
        self.frame_count = 0
        self.initialized = False
        self.roi_vertices = self._get_roi_vertices()
        self.left_coeffs_hist = deque(maxlen=5)
        self.right_coeffs_hist = deque(maxlen=5)
        self.prev_left_coeffs = None
        self.prev_right_coeffs = None
        self._current_frame = None
        # For temporal smoothing of lost lanes
        self.left_coeffs_buffer = deque(maxlen=5)
        self.right_coeffs_buffer = deque(maxlen=5)
        self.missing_left_count = 0
        self.missing_right_count = 0

    def _get_roi_vertices(self):
        width, height = self.frame_width, self.frame_height
        vertices = np.array([
            [int(0.10 * width), height],
            [int(540), 430],
            [int(740), 430],
            [int(0.90 * width), height],
        ], dtype=np.int32)
        return vertices

    def _create_roi_mask(self, img_shape):
        mask = np.zeros(img_shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [self.roi_vertices], 255)
        return mask

    def _collect_initial_training_data(self, frame):
        height, width = frame.shape[:2]
        road_region = frame[int(0.8*height):height, int(0.3*width):int(0.7*width)]
        road_pixels = road_region.reshape(-1, 3)
        self.road_samples.extend(road_pixels[::10])
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))
        white_pixels = frame[white_mask > 0]
        if len(white_pixels) > 0:
            self.white_lane_samples.extend(white_pixels[::5])
        yellow_mask = cv2.inRange(hsv, (20, 100, 100), (38, 255, 255))
        yellow_pixels = frame[yellow_mask > 0]
        if len(yellow_pixels) > 0:
            self.yellow_lane_samples.extend(yellow_pixels[::5])

    def _train_lda_models(self):
        if len(self.road_samples) < 50 or len(self.white_lane_samples) < 20:
            return False
        road_data = np.array(list(self.road_samples)[-500:])
        white_data = np.array(list(self.white_lane_samples)[-200:])
        if len(road_data) > 0 and len(white_data) > 0:
            X_white = np.vstack([road_data, white_data])
            y_white = np.hstack([np.zeros(len(road_data)), np.ones(len(white_data))])
            self.white_lda = LinearDiscriminantAnalysis()
            self.white_lda.fit(X_white, y_white)
        if len(self.yellow_lane_samples) > 20:
            yellow_data = np.array(list(self.yellow_lane_samples)[-200:])
            X_yellow = np.vstack([road_data, yellow_data])
            y_yellow = np.hstack([np.zeros(len(road_data)), np.ones(len(yellow_data))])
            self.yellow_lda = LinearDiscriminantAnalysis()
            self.yellow_lda.fit(X_yellow, y_yellow)
        return True

    # --- Histogram peak helpers ---
    def _lane_histogram_peaks(self, mask, min_offset=50, margin=200):
        h, w = mask.shape
        scan_bottom = int(h * 0.75)
        roi = mask[scan_bottom:, :]
        histogram = np.sum(roi, axis=0)

        mid = w // 2
        left_hist = histogram[min_offset:mid]
        right_hist = histogram[mid:w - min_offset]

        leftx = np.argmax(left_hist) + min_offset
        rightx = np.argmax(right_hist) + mid

        return leftx, rightx, margin

    def _lines_near_histogram_peak(self, lines, peak_x, margin=100):
        filtered = []
        if lines is None: return []
        for line in lines:
            x1, y1, x2, y2 = line
            # Use base point (greater y)
            if y1 > y2: xbase = x1
            else: xbase = x2
            if abs(xbase - peak_x) < margin:
                filtered.append(line)
        return filtered

    # ---- New Preprocessing Pipeline ----
    def _enhance_preprocessing(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, (15, 50, 80), (38, 255, 255))
        white_mask  = cv2.inRange(hsv, (0, 0, 160), (180, 70, 255))
        combined = cv2.bitwise_or(yellow_mask, white_mask)
        kernel = np.ones((5,5), np.uint8)
        closed = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced = clahe.apply(closed)
        return enhanced

    def _gradient_enhancing_conversion(self, frame):
        return self._enhance_preprocessing(frame)

    def _adaptive_canny_edge_detection(self, img):
        v = np.median(img)
        sigma = 0.33
        lower_thresh = int(max(0, (1.0 - sigma) * v))
        upper_thresh = int(min(255, (1.0 + sigma) * v))
        blurred = cv2.GaussianBlur(img, (5, 5), 0)
        edges = cv2.Canny(blurred, lower_thresh, upper_thresh)
        return edges

    def _hough_line_detection(self, edges):
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=50,
            minLineLength=20,
            maxLineGap=160
        )
        return lines

    def _classify_lines(self, lines, img_width):
        if lines is None:
            return [], []
        left_lines = []
        right_lines = []
        margin_px = int(self.edge_margin_frac * img_width)
        yellow_lower = np.array([15, 40, 60])
        yellow_upper = np.array([38, 255, 255])
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0:
                continue
            if min(x1, x2) < margin_px or max(x1, x2) > (img_width - margin_px):
                continue
            slope = (y2 - y1) / (x2 - x1)
            if -0.9 < slope < -0.4:
                xs = np.linspace(x1, x2, num=10, dtype=np.int32)
                ys = np.linspace(y1, y2, num=10, dtype=np.int32)
                colors = []
                for x, y in zip(xs, ys):
                    if 0 <= x < img_width and 0 <= y < self.frame_height:
                        colors.append(self._current_frame[y, x])
                if len(colors) > 0:
                    colors = np.array(colors)
                    hsv_colors = cv2.cvtColor(colors.reshape(-1, 1, 3).astype(np.uint8), cv2.COLOR_BGR2HSV).reshape(-1,3)
                    yellow_pixels = np.logical_and.reduce([
                        hsv_colors[:,0] >= yellow_lower[0],
                        hsv_colors[:,0] <= yellow_upper[0],
                        hsv_colors[:,1] >= yellow_lower[1],
                        hsv_colors[:,1] <= yellow_upper[1],
                        hsv_colors[:,2] >= yellow_lower[2],
                        hsv_colors[:,2] <= yellow_upper[2]
                    ])
                    if np.sum(yellow_pixels) > 2:
                        left_lines.append(line[0])
            elif 0.4 < slope < 0.9:
                right_lines.append(line[0])
        return left_lines, right_lines

    def _fit_lane_polynomial(self, lines, img_height):
        if not lines:
            return None
        x_coords = []
        y_coords = []
        for line in lines:
            x1, y1, x2, y2 = line
            x_coords.extend([x1, x2])
            y_coords.extend([y1, y2])
        if len(x_coords) < 2:
            return None
        try:
            coeffs = np.polyfit(y_coords, x_coords, 1)
            return coeffs
        except Exception:
            return None

    def _smooth_coeffs(self, coeffs_hist, new_coeffs):
        if new_coeffs is not None:
            coeffs_hist.append(new_coeffs)
        if coeffs_hist:
            return np.mean(coeffs_hist, axis=0)
        else:
            return None

    def _is_similar(self, old_coeffs, new_coeffs, thresh=80):
        if old_coeffs is None or new_coeffs is None:
            return True
        return np.linalg.norm(np.array(old_coeffs) - np.array(new_coeffs)) < thresh

    def _draw_lane(self, img, coeffs, color=(0, 255, 0), thickness=8):
        if coeffs is None:
            return img
        height = img.shape[0]
        y_vals = np.linspace(height//2, height-1, height//2)
        try:
            x_vals = np.polyval(coeffs, y_vals)
            pts = np.array([[int(x), int(y)] for x, y in zip(x_vals, y_vals)
                            if 0 <= x < img.shape[1]], dtype=np.int32)
            if len(pts) > 1:
                cv2.polylines(img, [pts], False, color, thickness)
        except Exception:
            pass
        return img

    def _update_training_data(self, frame, left_coeffs, right_coeffs):
        if left_coeffs is None and right_coeffs is None:
            return
        height, width = frame.shape[:2]
        if left_coeffs is not None and right_coeffs is not None:
            y_sample = int(0.8 * height)
            x_left = int(np.polyval(left_coeffs, y_sample))
            x_right = int(np.polyval(right_coeffs, y_sample))
            if 0 < x_left < x_right < width:
                road_region = frame[y_sample-10:y_sample+10, x_left+20:x_right-20]
                if road_region.size > 0:
                    road_pixels = road_region.reshape(-1, 3)
                    self.road_samples.extend(road_pixels[::20])

    def detect_lanes(self, frame):
        self._current_frame = frame
        original_frame = frame.copy()
        if self.frame_count < 10:
            self._collect_initial_training_data(frame)
            self.frame_count += 1
            if self.frame_count == 10:
                self.initialized = self._train_lda_models()
        enhanced_img = self._enhance_preprocessing(frame)
        edges = self._adaptive_canny_edge_detection(enhanced_img)
        roi_mask = self._create_roi_mask(frame.shape)
        edges_roi = cv2.bitwise_and(edges, edges, mask=roi_mask)
        lines = self._hough_line_detection(edges_roi)

        # -------- Histogram-based filtering --------
        left_peak, right_peak, margin = self._lane_histogram_peaks(edges_roi)
        left_lines, right_lines = self._classify_lines(lines, frame.shape[1])
        left_lines = self._lines_near_histogram_peak(left_lines, left_peak, margin)
        right_lines = self._lines_near_histogram_peak(right_lines, right_peak, margin)

        left_coeffs = self._fit_lane_polynomial(left_lines, frame.shape[0])
        right_coeffs = self._fit_lane_polynomial(right_lines, frame.shape[0])
        
        # Temporal smoothing for lost lanes (hold for up to 5 frames)
        if left_coeffs is not None:
            self.left_coeffs_buffer.append(left_coeffs)
            self.missing_left_count = 0
        else:
            self.missing_left_count += 1
            if self.missing_left_count < 5 and len(self.left_coeffs_buffer) > 0:
                left_coeffs = self.left_coeffs_buffer[-1]
        if right_coeffs is not None:
            self.right_coeffs_buffer.append(right_coeffs)
            self.missing_right_count = 0
        else:
            self.missing_right_count += 1
            if self.missing_right_count < 5 and len(self.right_coeffs_buffer) > 0:
                right_coeffs = self.right_coeffs_buffer[-1]

        if not self._is_similar(self.prev_left_coeffs, left_coeffs):
            left_coeffs = self.prev_left_coeffs
        if not self._is_similar(self.prev_right_coeffs, right_coeffs):
            right_coeffs = self.prev_right_coeffs
        left_coeffs = self._smooth_coeffs(self.left_coeffs_hist, left_coeffs)
        right_coeffs = self._smooth_coeffs(self.right_coeffs_hist, right_coeffs)
        self.prev_left_coeffs = left_coeffs
        self.prev_right_coeffs = right_coeffs
        result_frame = original_frame.copy()
        result_frame = self._draw_lane(result_frame, left_coeffs, (0, 255, 0), 8)
        result_frame = self._draw_lane(result_frame, right_coeffs, (0, 255, 0), 8)
        cv2.polylines(result_frame, [self.roi_vertices], True, (255, 0, 0), 2)
        result_frame = self._draw_center_lines(result_frame, left_coeffs, right_coeffs)
        self._update_training_data(frame, left_coeffs, right_coeffs)
        self.frame_count += 1
        return result_frame, enhanced_img, edges_roi

    def _draw_center_lines(self, img, left_coeffs, right_coeffs):
        height = img.shape[0]
        roi_top = min(self.roi_vertices[:, 1])
        roi_bottom = max(self.roi_vertices[:, 1])
        x_middle = self.frame_width // 2
        cv2.line(img, (x_middle, roi_bottom), (x_middle, roi_top), (255, 255, 0), 3)
        if left_coeffs is not None and right_coeffs is not None:
            y_vals = np.linspace(roi_top, roi_bottom, 40)
            pts = []
            for y in y_vals:
                x_left = np.polyval(left_coeffs, y)
                x_right = np.polyval(right_coeffs, y)
                x_center = int((x_left + x_right) / 2)
                if 0 <= x_center < self.frame_width:
                    pts.append([x_center, int(y)])
            pts = np.array(pts, dtype=np.int32)
            if len(pts) > 1:
                pts = pts.reshape((-1, 1, 2))
                cv2.polylines(img, [pts], False, (0, 0, 255), 4)
        return img

class LaneFollowerNode(Node):
    def __init__(self):
        super().__init__('lane_follower')
        self.detector = GradientEnhancedLaneDetector(1280, 720)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/usb_cam_0/image_raw', self.image_callback, 10)
        self.steer_pub = self.create_publisher(ErpCmdMsg, '/erp42_ctrl_cmd/lane', 10)
        self.last_steer = 0.0
        self.visualize = True

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame.shape[:2] != (720, 1280):
            frame = cv2.resize(frame, (1280, 720))
        result_frame, enhanced_img, edges = self.detector.detect_lanes(frame)
        steer = self.compute_steering_from_lanes(self.detector.prev_left_coeffs,
                                                 self.detector.prev_right_coeffs)
        self.publish_steering(steer)
        if self.visualize:
            status = "Initialized" if self.detector.initialized else "Initializing..."
            cv2.putText(result_frame, f'Status: {status}', (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.imshow('Lane Detection', result_frame)
            cv2.imshow('Enhanced Image', enhanced_img)
            cv2.imshow('Edges', edges)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                self.get_logger().info("Shutting down visualization")
                rclpy.shutdown()

    def compute_steering_from_lanes(self, left_coeffs, right_coeffs):
        if left_coeffs is not None and right_coeffs is not None:
            lookahead_y = int(0.8 * self.detector.frame_height)
            x_left = np.polyval(left_coeffs, lookahead_y)
            x_right = np.polyval(right_coeffs, lookahead_y)
            lane_center = (x_left + x_right) / 2
            frame_center = self.detector.frame_width / 2
            max_steer_angle = 1.0
            offset = lane_center - frame_center
            Kp = 0.0025
            steer = -Kp * offset
            steer = np.clip(steer, -max_steer_angle, max_steer_angle)
            self.get_logger().info(f"Lane center offset: {offset:.2f}, steer: {steer:.2f}")
            return float(steer)
        else:
            self.get_logger().warn("Lane not detected!")
            return float(self.last_steer)

    def publish_steering(self, steer):
        msg = ErpCmdMsg()
        msg.steer = int(steer)
        self.steer_pub.publish(msg)
        self.last_steer = steer

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
