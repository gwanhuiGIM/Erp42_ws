import cv2
import numpy as np
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from collections import deque
import time

class GradientEnhancedLaneDetector:
    def __init__(self, frame_width=1280, frame_height=720):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.training_data_frames = 5
        self.road_samples = deque(maxlen=1000)
        self.white_lane_samples = deque(maxlen=1000)
        self.yellow_lane_samples = deque(maxlen=1000)
        self.white_lda = None
        self.yellow_lda = None
        self.frame_count = 0
        self.initialized = False
        # ROI vertices: narrowed at the bottom to exclude boundaries
        self.roi_vertices = self._get_roi_vertices()
        self.left_coeffs_hist = deque(maxlen=5)
        self.right_coeffs_hist = deque(maxlen=5)
        self.prev_left_coeffs = None
        self.prev_right_coeffs = None

    def _get_roi_vertices(self):
        width, height = self.frame_width, self.frame_height
        # Narrow at bottom: focus inside lanes
        vertices = np.array([
            [int(0.22 * width), height],    # Bottom left (in from left)
            [int(570), 430],       # Top left
            [int(700), 430],       # Top right
            [int(0.78 * width), height],    # Bottom right (in from right)
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
        yellow_mask = cv2.inRange(hsv, (20, 100, 100), (35, 255, 255))
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

    def _color_filter(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # White mask
        white_mask = cv2.inRange(hsv, (0, 0, 180), (180, 50, 255))
        # Yellow mask
        yellow_mask = cv2.inRange(hsv, (15, 80, 80), (35, 255, 255))
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
        return cv2.bitwise_and(frame, frame, mask=combined_mask)

    def _gradient_enhancing_conversion(self, frame):
        if not self.initialized:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height, width = frame.shape[:2]
        frame_flat = frame.reshape(-1, 3).astype(np.float32)
        enhanced_images = []
        if self.white_lda is not None:
            try:
                white_enhanced = self.white_lda.transform(frame_flat)
                white_enhanced = white_enhanced.reshape(height, width)
                white_enhanced = cv2.normalize(white_enhanced, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                enhanced_images.append(white_enhanced)
            except Exception:
                pass
        if self.yellow_lda is not None:
            try:
                yellow_enhanced = self.yellow_lda.transform(frame_flat)
                yellow_enhanced = yellow_enhanced.reshape(height, width)
                yellow_enhanced = cv2.normalize(yellow_enhanced, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                enhanced_images.append(yellow_enhanced)
            except Exception:
                pass
        if enhanced_images:
            combined = np.maximum.reduce(enhanced_images)
            return combined
        else:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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
            threshold=45,       # Try 40–60
            minLineLength=45,   # Try 40–80
            maxLineGap=150      # Try 40–120
        )
        return lines

    def _classify_lines(self, lines, img_width):
        if lines is None:
            return [], []
        left_lines = []
        right_lines = []
        # Only accept lines in "lane" zones, not near boundaries
        left_zone = (int(0.22*img_width), int(0.45*img_width))
        right_zone = (int(0.55*img_width), int(0.78*img_width))
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0:
                continue
            slope = (y2 - y1) / (x2 - x1)
            # Stricter slope and position filtering
            if -0.9 < slope < -0.4 and left_zone[0] < x1 < left_zone[1] and left_zone[0] < x2 < left_zone[1]:
                left_lines.append(line[0])
            elif 0.4 < slope < 0.9 and right_zone[0] < x1 < right_zone[1] and right_zone[0] < x2 < right_zone[1]:
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
            coeffs = np.polyfit(y_coords, x_coords, 2)
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

    def _is_similar(self, old_coeffs, new_coeffs, thresh=100):
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
        original_frame = frame.copy()
        if self.frame_count < 10:
            self._collect_initial_training_data(frame)
            self.frame_count += 1
            if self.frame_count == 10:
                self.initialized = self._train_lda_models()

        # Step 0: Color filter before anything else
        filtered = self._color_filter(frame)
        # Step 1: Gradient enhance
        enhanced_img = self._gradient_enhancing_conversion(filtered)
        # Step 2: Edge detection
        edges = self._adaptive_canny_edge_detection(enhanced_img)
        # Step 3: Apply ROI to edges (masking after edge detection)
        roi_mask = self._create_roi_mask(frame.shape)
        edges_roi = cv2.bitwise_and(edges, edges, mask=roi_mask)
        # Step 4: Hough line detection
        lines = self._hough_line_detection(edges_roi)
        left_lines, right_lines = self._classify_lines(lines, frame.shape[1])
        left_coeffs = self._fit_lane_polynomial(left_lines, frame.shape[0])
        right_coeffs = self._fit_lane_polynomial(right_lines, frame.shape[0])
        # Outlier rejection
        if not self._is_similar(self.prev_left_coeffs, left_coeffs):
            left_coeffs = self.prev_left_coeffs
        if not self._is_similar(self.prev_right_coeffs, right_coeffs):
            right_coeffs = self.prev_right_coeffs
        # Smoothing
        left_coeffs = self._smooth_coeffs(self.left_coeffs_hist, left_coeffs)
        right_coeffs = self._smooth_coeffs(self.right_coeffs_hist, right_coeffs)
        self.prev_left_coeffs = left_coeffs
        self.prev_right_coeffs = right_coeffs
        # Draw lanes
        result_frame = original_frame.copy()
        result_frame = self._draw_lane(result_frame, left_coeffs, (0, 255, 0), 8)
        result_frame = self._draw_lane(result_frame, right_coeffs, (0, 255, 0), 8)
        cv2.polylines(result_frame, [self.roi_vertices], True, (255, 0, 0), 2)
        result_frame = self._draw_center_lines(result_frame, left_coeffs, right_coeffs)
        # Draw raw Hough lines for debugging (optional)
        #if lines is not None:
            #for line in lines:
                #x1, y1, x2, y2 = line[0]
                #cv2.line(result_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
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
                pts = pts.reshape((-1, 1, 2))  # Correct OpenCV shape
                cv2.polylines(img, [pts], False, (0, 0, 255), 4)
        return img

def main():
    detector = GradientEnhancedLaneDetector(1280, 720)
    cap = cv2.VideoCapture('/home/mrlam/Downloads/Highway_driving.mp4')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        print("Error: Could not open video source")
        return
    print("Lane Detection Started. Press 'q' to quit.")
    frame_count = 0
    start_time = time.time()
    while True:
        ret, frame = cap.read()
        if not ret:
            print("End of video or failed to capture frame")
            break
        if frame.shape[:2] != (720, 1280):
            frame = cv2.resize(frame, (1280, 720))
        result_frame, enhanced_img, edges = detector.detect_lanes(frame)
        frame_count += 1
        elapsed_time = time.time() - start_time
        fps = frame_count / elapsed_time if elapsed_time > 0 else 0
        cv2.putText(result_frame, f'FPS: {fps:.1f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        status = "Initialized" if detector.initialized else "Initializing..."
        cv2.putText(result_frame, f'Status: {status}', (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow('Lane Detection', result_frame)
        cv2.imshow('Enhanced Image', enhanced_img)
        cv2.imshow('Edges', edges)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    print(f"Processing complete. Average FPS: {fps:.1f}")

if __name__ == "__main__":
    main()
