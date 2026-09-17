import cv2
import numpy as np
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from collections import deque
import time

class GradientEnhancedLaneDetector:
    def __init__(self, frame_width=1280, frame_height=720):
        self.frame_width = frame_width
        self.frame_height = frame_height
       
        # Initialize training data storage for LDA
        self.road_samples = deque(maxlen=500)
        self.white_lane_samples = deque(maxlen=100)
        self.yellow_lane_samples = deque(maxlen=100)
       
        # LDA models for gradient enhancement
        self.white_lda = None
        self.yellow_lda = None
       
        # Frame counter for initialization
        self.frame_count = 0
        self.initialized = False
       
        # ROI parameters
        self.roi_vertices = self._get_roi_vertices()
       
        # Lane detection parameters
        self.left_lane_coeffs = None
        self.right_lane_coeffs = None
       
    def _get_roi_vertices(self):
        """Define Region of Interest (ROI) vertices for lane detection"""
        h, w = self.frame_height, self.frame_width
        return np.array([
            [int(0.15*w), h],
            [int(0.45*w), int(0.65*h)],
            [int(0.55*w), int(0.65*h)],
            [int(0.85*w), h]
        ], dtype=np.int32)
   
    def _create_roi_mask(self, img_shape):
        mask = np.zeros(img_shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [self.roi_vertices], 255)
        return mask
   
    def _collect_initial_training_data(self, frame):
        roi_mask = self._create_roi_mask(frame.shape)
        roi_frame = cv2.bitwise_and(frame, frame, mask=roi_mask)
       
        hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2LAB)
       
        white_mask_hsv = cv2.inRange(hsv, (0,0,180), (180,25,255))
        white_mask_lab = cv2.inRange(lab, (200,0,0), (255,255,255))
        white_mask = cv2.bitwise_or(white_mask_hsv, white_mask_lab)
       
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
       
        yellow_mask1 = cv2.inRange(hsv, (15,80,80), (35,255,255))
        yellow_mask2 = cv2.inRange(hsv, (20,100,100), (30,255,255))
        yellow_mask = cv2.bitwise_or(yellow_mask1, yellow_mask2)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
       
        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
        road_mask = cv2.bitwise_and(roi_mask, cv2.bitwise_not(lane_mask))
       
        white_pixels = roi_frame[white_mask>0]
        yellow_pixels = roi_frame[yellow_mask>0]
        road_pixels = roi_frame[road_mask>0]
       
        if len(white_pixels)>0:
            self.white_lane_samples.extend(white_pixels[::3])
        if len(yellow_pixels)>0:
            self.yellow_lane_samples.extend(yellow_pixels[::3])
        if len(road_pixels)>0:
            self.road_samples.extend(road_pixels[::5])
   
    def _train_lda_models(self):
        if len(self.road_samples)<100:
            return False
       
        road_data = np.array(list(self.road_samples)[-500:])
        success = False
       
        if len(self.white_lane_samples)>50:
            white_data = np.array(list(self.white_lane_samples)[-200:])
            Xw = np.vstack([road_data, white_data])
            yw = np.hstack([np.zeros(len(road_data)), np.ones(len(white_data))])
            self.white_lda = LinearDiscriminantAnalysis().fit(Xw, yw)
            success = True
       
        if len(self.yellow_lane_samples)>50:
            yellow_data = np.array(list(self.yellow_lane_samples)[-200:])
            Xy = np.vstack([road_data, yellow_data])
            yy = np.hstack([np.zeros(len(road_data)), np.ones(len(yellow_data))])
            self.yellow_lda = LinearDiscriminantAnalysis().fit(Xy, yy)
            success = True
       
        return success
   
    def _gradient_enhancing_conversion(self, frame):
        if not self.initialized or (self.white_lda is None and self.yellow_lda is None):
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            return clahe.apply(gray)
       
        h, w = frame.shape[:2]
        flat = frame.reshape(-1,3).astype(np.float32)
        enhanced = []
       
        if self.white_lda is not None:
            try:
                we = self.white_lda.transform(flat).reshape(h,w)
                enhanced.append(cv2.normalize(we,None,0,255,cv2.NORM_MINMAX).astype(np.uint8))
            except: pass
       
        if self.yellow_lda is not None:
            try:
                ye = self.yellow_lda.transform(flat).reshape(h,w)
                enhanced.append(cv2.normalize(ye,None,0,255,cv2.NORM_MINMAX).astype(np.uint8))
            except: pass
       
        if enhanced:
            return np.maximum.reduce(enhanced)
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            return clahe.apply(gray)
   
    def _advanced_edge_detection(self, img):
        blur = cv2.GaussianBlur(img,(5,5),0)
        v = np.median(blur)
        sigma = 0.33
        low = max(50, int((1.0-sigma)*v))
        high = max(150, int((1.0+sigma)*v))
        edges = cv2.Canny(blur, low, high)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        return cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
   
    def _hough_line_detection(self, edges):
        return cv2.HoughLinesP(edges, 2, np.pi/180, 30, minLineLength=30, maxLineGap=100)
   
    def _filter_and_classify_lines(self, lines, img_w, img_h):
        if lines is None:
            return [], []
       
        left, right = [], []
        for l in lines:
            x1,y1,x2,y2 = l[0]
            if abs(y2-y1)<10 or x2==x1: continue
            slope = (y2-y1)/(x2-x1)
            length = np.hypot(x2-x1, y2-y1)
            if abs(slope)<0.3 or abs(slope)>3.0 or length<20: continue
            cx = (x1+x2)/2
            if slope<0 and cx<img_w*0.6:
                left.append([x1,y1,x2,y2,slope,length])
            elif slope>0 and cx>img_w*0.4:
                right.append([x1,y1,x2,y2,slope,length])
       
        left.sort(key=lambda x: x[5], reverse=True)
        right.sort(key=lambda x: x[5], reverse=True)
        return left[:5], right[:5]
   
    def _fit_lane_line(self, lines, img_h):
        if not lines:
            return None
       
        xs, ys, ws = [], [], []
        for x1,y1,x2,y2,slope,length in lines:
            xs += [x1,x2]; ys += [y1,y2]; ws += [length,length]
       
        xs, ys, ws = np.array(xs), np.array(ys), np.array(ws)
        try:
            return np.polyfit(ys, xs, 2, w=ws)
        except:
            try:
                lin = np.polyfit(ys, xs, 1, w=ws)
                return np.array([0, lin[0], lin[1]])
            except:
                return None
   
    def _draw_lane_lines(self, img, left_c, right_c):
        h, w = img.shape[:2]
        overlay = np.zeros_like(img)
        yv = np.linspace(h//2, h-1, h//2, dtype=int)
        pts = []
       
        if left_c is not None:
            xl = np.polyval(left_c, yv)
            lp = [[int(x),int(y)] for x,y in zip(xl, yv) if 0<=x<w]
            if len(lp)>1:
                pts.append(np.array(lp, dtype=np.int32))
                cv2.polylines(overlay, [np.array(lp)], False, (0,255,0), 8)
       
        if right_c is not None:
            xr = np.polyval(right_c, yv)
            rp = [[int(x),int(y)] for x,y in zip(xr, yv) if 0<=x<w]
            if len(rp)>1:
                pts.append(np.array(rp, dtype=np.int32))
                cv2.polylines(overlay, [np.array(rp)], False, (0,255,0), 8)
       
        if len(pts)==2:
            poly = np.vstack([pts[0], pts[1][::-1]])
            cv2.fillPoly(overlay, [poly], (0,255,0))
            img = cv2.addWeighted(img,0.8,overlay,0.2,0)
        else:
            img = cv2.addWeighted(img,1.0,overlay,1.0,0)
       
        return img

    def _draw_center_and_midline(self, img, left_c, right_c):
        h, w = img.shape[:2]
        # draw vertical center of image
        cv2.line(img, (w//2, 0), (w//2, h), (255, 0, 0), 2)
        # draw midline between lanes
        if left_c is not None and right_c is not None:
            yv = np.linspace(h//2, h-1, h//2, dtype=int)
            xl = np.polyval(left_c, yv)
            xr = np.polyval(right_c, yv)
            mp = [[int((l+r)/2), int(y)] for l,r,y in zip(xl, xr, yv)]
            if len(mp)>1:
                cv2.polylines(img, [np.array(mp, dtype=np.int32)], False, (0,0,255), 4)

    def detect_lanes(self, frame):
        orig = frame.copy()
       
        if self.frame_count < 15:
            self._collect_initial_training_data(frame)
            self.frame_count += 1
            if self.frame_count == 15:
                self.initialized = self._train_lda_models()
                print(f"Training completed. Initialized: {self.initialized}")
       
        mask = self._create_roi_mask(frame.shape)
        masked = cv2.bitwise_and(frame, frame, mask=mask)
        enh = self._gradient_enhancing_conversion(masked)
        enh_roi = cv2.bitwise_and(enh, enh, mask=mask)
        edges = self._advanced_edge_detection(enh_roi)
        lines = self._hough_line_detection(edges)
        left, right = self._filter_and_classify_lines(lines, frame.shape[1], frame.shape[0])
        lc = self._fit_lane_line(left, frame.shape[0])
        rc = self._fit_lane_line(right, frame.shape[0])
       
        alpha = 0.2
        if lc is not None:
            self.left_lane_coeffs = alpha*lc + (1-alpha)*self.left_lane_coeffs if self.left_lane_coeffs is not None else lc
        if rc is not None:
            self.right_lane_coeffs = alpha*rc + (1-alpha)*self.right_lane_coeffs if self.right_lane_coeffs is not None else rc
       
        rf = self._draw_lane_lines(orig, self.left_lane_coeffs, self.right_lane_coeffs)
        self._draw_center_and_midline(rf, self.left_lane_coeffs, self.right_lane_coeffs)
        cv2.polylines(rf, [self.roi_vertices], True, (255,0,0), 2)
        
        # draw debug line segments
        if lines is not None:
            for l in lines:
                x1,y1,x2,y2 = l[0]
                cv2.line(edges, (x1,y1), (x2,y2), 255, 2)

        return rf, enh_roi, edges


def main():
    detector = GradientEnhancedLaneDetector(1280,720)
    cap = cv2.VideoCapture('/home/mrlam/Downloads/Highway_driving.mp4')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    cap.set(cv2.CAP_PROP_FPS,30)
   
    if not cap.isOpened():
        print("Error: Could not open video source")
        return
   
    print("Lane Detection Started. Press 'q' to quit.")
    print("Initializing... (collecting training data)")
   
    cnt = 0
    start = time.time()
   
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame.shape[:2] != (720,1280):
            frame = cv2.resize(frame,(1280,720))
       
        rf, enh, edges = detector.detect_lanes(frame)
        cnt += 1
        fps = cnt/(time.time()-start) if time.time()>start else 0
       
        cv2.putText(rf, f'FPS: {fps:.1f}', (10,30), cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
        status = "Initialized" if detector.initialized else f"Initializing... ({detector.frame_count}/15)"
        cv2.putText(rf, f'Status: {status}', (10,70), cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,0),2)
        cv2.putText(rf, f'Left Lane: {"Yes" if detector.left_lane_coeffs is not None else "No"}', (10,110), cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,0),2)
        cv2.putText(rf, f'Right Lane: {"Yes" if detector.right_lane_coeffs is not None else "No"}', (10,150), cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,0),2)
       
        cv2.imshow('Lane Detection', rf)
        cv2.imshow('Enhanced Image', enh)
        cv2.imshow('Edges', edges)
       
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
   
    cap.release()
    cv2.destroyAllWindows()
    print(f"Processing complete. Average FPS: {fps:.1f}")

if __name__ == "__main__":
    main()
