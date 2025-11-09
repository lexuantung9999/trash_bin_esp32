#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Color -> waste class from /dev/video2 with stability gate, cooldown, and ESP32 HTTP routing (ASYNC).
- TOP-LEFT: show last committed waste class + % UNTIL cooldown ends
- TOP-RIGHT: status (Calibrating/Stabilizing/Cooldown/No object/Stable)
- BOTTOM-RIGHT: queued / HTTP status
"""

import cv2
import numpy as np
import time
from collections import deque
import requests
from queue import Queue, Empty
import threading

# ============================ Config ============================

CAMERA_DEVICE = "/dev/video2"
FRAME_WIDTH   = 1280
FRAME_HEIGHT  = 720
FPS_TARGET    = 30

# Background calibration
CALIBRATION_SECONDS = 5.0
BASE_LEARNING_RATE  = 0.001

# Foreground / presence (hysteresis)
AREA_ENTER_THRESH   = 2500
AREA_EXIT_THRESH    = 1200
MORPH_KERNEL        = (5, 5)
PRESENCE_GRACE_SEC  = 1.2

# Stability
STABLE_WINDOW            = 40
CENTER_JITTER_PX         = 5
AREA_JITTER_RATIO        = 0.05
STABLE_SPAN_SEC          = 1.8
MIN_CONSEC_STABLE_FRAMES = 30

# EMA smoothing
EMA_ALPHA = 0.3

# ROI cleanup
EDGE_ERODE_ITERS = 1
EDGE_DILATE_ITERS = 1
S_MIN_OBJ = 60

# HUD
HUD_MARGIN            = 12
HUD_FONT_SCALE_MAIN   = 0.9   # TR
HUD_FONT_SCALE_TL     = 0.9   # TL (committed class)
HUD_FONT_SCALE_SMALL  = 0.7   # BR

# HSV thresholds
HSV_RANGES = {
    "red1":   ((0,   80,  50), (10,  255, 255)),
    "red2":   ((170, 80,  50), (179, 255, 255)),
    "yellow": ((20,  80,  70), (35,  255, 255)),
    "green":  ((35,  60,  60), (85,  255, 255)),
    "blue":   ((90,  60,  60), (130, 255, 255)),
    "purple": ((125, 60,  60), (145, 255, 255)),
    "black":  ((0,    0,   0), (179, 255,  50)),
    "orange": ((10, 100, 150), (25, 255, 255)),
}

IGNORE_NEAR_WHITE = True
WHITE_S_MAX       = 25
WHITE_V_MIN       = 210

MIN_DOMINANCE     = 0.25

# Color -> waste class
COLOR_TO_CLASS = {
    "red":     "RECYCLABLE CAN",
    "purple":  "RECYCLABLE CAN",
    "yellow":  "RECYCLABLE PAPER",
    "black":   "PLASTIC",
    "blue":    "PLASTIC",
    "green":   "PLASTIC",
    "orange":  "ORGANIC",
}

# Class -> bin
CLASS_TO_BIN = {
    "ORGANIC":           "bin1",
    "PLASTIC":           "bin2",
    "RECYCLABLE CAN":    "bin3",
    "RECYCLABLE PAPER":  "bin4",
}

# ESP32 endpoints
HOSTS = ["http://172.28.63.100"]
ENDPOINT_PATHS = {
    "bin1": "/bin1", "bin2": "/bin2", "bin3": "/bin3", "bin4": "/bin4",
    "reset": "/reset",
    "servo_open": "/servo/open", "servo_close": "/servo/close",
    "motor_left": "/motor/left", "motor_right": "/motor/right", "motor_stop": "/motor/stop",
    "encoder": "/encoder", "state": "/state", "index": "/",
}

# HTTP tuning
CONNECT_TIMEOUT = 0.7
READ_TIMEOUT    = 10.0
HTTP_TIMEOUT    = (CONNECT_TIMEOUT, READ_TIMEOUT)
REQUEST_RETRIES = 2
REQUEST_BACKOFF = 0.3

# Cycle
CYCLE_COOLDOWN_SEC = 15.0

# ========================== Utilities ===========================

def open_camera(dev, w, h, fps):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS,          fps)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera at {dev}")
    return cap

def make_subtractor():
    return cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=16, detectShadows=True)

def largest_contour(binary):
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0
    c = max(contours, key=cv2.contourArea)
    return c, cv2.contourArea(c)

def _draw_boxed_text(img, text, corner="tr", margin=12, font_scale=0.9):
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2)
    pad = 8
    box_w, box_h = tw + 2*pad, th + base + 2*pad
    H, W = img.shape[:2]
    if corner == "tl":
        x0, y0 = margin, margin
    elif corner == "tr":
        x0, y0 = W - box_w - margin, margin
    elif corner == "bl":
        x0, y0 = margin, H - box_h - margin
    else:
        x0, y0 = W - box_w - margin, H - box_h - margin
    overlay = img.copy()
    cv2.rectangle(overlay, (x0, y0), (x0+box_w, y0+box_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.45, img, 0.55, 0, img)
    cv2.rectangle(img, (x0, y0), (x0+box_w, y0+box_h), (0, 255, 255), 2)
    cv2.putText(img, text, (x0+pad, y0+pad+th), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255,255,255), 2, cv2.LINE_AA)

def hud_status_tr(img, text):
    _draw_boxed_text(img, text, corner="tr", margin=HUD_MARGIN, font_scale=HUD_FONT_SCALE_MAIN)

def hud_detected_tl(img, text):
    _draw_boxed_text(img, text, corner="tl", margin=HUD_MARGIN, font_scale=HUD_FONT_SCALE_TL)

def hud_br(img, text):
    _draw_boxed_text(img, text, corner="br", margin=HUD_MARGIN, font_scale=HUD_FONT_SCALE_SMALL)

def truncate(s, maxlen=54):
    return (s if len(s) <= maxlen else (s[:maxlen-1] + "…"))

def dominant_basic_color(hsv_roi, fg_mask_roi):
    obj_mask = (fg_mask_roi > 0).astype(np.uint8) * 255
    if IGNORE_NEAR_WHITE:
        white_mask = cv2.inRange(
            hsv_roi,
            np.array([0, 0, WHITE_V_MIN], np.uint8),
            np.array([179, WHITE_S_MAX, 255], np.uint8),
        )
        obj_mask = cv2.bitwise_and(obj_mask, cv2.bitwise_not(white_mask))
    s_channel = hsv_roi[:, :, 1]
    _, sat_mask = cv2.threshold(s_channel, int(S_MIN_OBJ) - 1, 255, cv2.THRESH_BINARY)
    obj_mask = cv2.bitwise_and(obj_mask, sat_mask)
    total = int(np.count_nonzero(obj_mask))
    if total == 0:
        return "Unknown", 0.0

    masks = {k: cv2.inRange(hsv_roi, np.array(lo, np.uint8), np.array(hi, np.uint8))
             for k, (lo, hi) in HSV_RANGES.items()}
    red_mask = cv2.bitwise_or(masks["red1"], masks["red2"])

    S = s_channel.astype(np.int64)
    obj = (obj_mask > 0)

    def sw_sum(m):
        m2 = (m > 0) & obj
        return int(S[m2].sum())

    scores = {
        "black":  sw_sum(masks["black"]),
        "red":    sw_sum(red_mask),
        "orange": sw_sum(masks["orange"]),
        "yellow": sw_sum(masks["yellow"]),
        "green":  sw_sum(masks["green"]),
        "blue":   sw_sum(masks["blue"]),
        "purple": sw_sum(masks["purple"]),
    }
    best_color = max(scores, key=scores.get)
    denom = int(S[obj].sum())
    dominance = (scores[best_color] / denom) if denom > 0 else 0.0

    if dominance < 0.35:
        H = hsv_roi[:, :, 0]
        H_sel = H[obj]
        if H_sel.size > 0:
            hist = np.bincount(H_sel, minlength=180)
            peak = int(hist.argmax())
            lo = (peak - 10) % 180
            hi = (peak + 10) % 180
            if lo <= hi: band = ((H >= lo) & (H <= hi)) & obj
            else:        band = ((H >= lo) | (H <= hi)) & obj
            sw = int(S[band].sum())
            dominance = (sw / denom) if denom > 0 else 0.0

            def hue_to_basic(h):
                if (h <= 10) or (h >= 170): return "red"
                if 10 <= h <= 25: return "orange"
                if 26 <= h <= 34: return "yellow"
                if 35 <= h <= 85: return "green"
                if 90 <= h <= 130: return "blue"
                if 125 <= h <= 145: return "purple"
                return "Unknown"
            best_color = hue_to_basic(peak)

    if dominance < MIN_DOMINANCE:
        return "Unknown", dominance
    return best_color, dominance

# =================== Async ESP32 HTTP routing ===================

class HttpWorker:
    def __init__(self):
        self.session = requests.Session()
        self.q = Queue(maxsize=16)
        self.results = Queue(maxsize=16)  # (timestamp, url, ok, msg)
        self._stop = threading.Event()
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def stop(self):
        self._stop.set()
        try: self.q.put_nowait(None)
        except Exception: pass
        self.t.join(timeout=1.0)

    def enqueue_bin(self, bin_name):
        path = ENDPOINT_PATHS.get(bin_name)
        if not path:
            self.results.put((time.time(), "", False, "Unknown bin path")); return False
        host = HOSTS[0] if HOSTS else ""
        if not host:
            self.results.put((time.time(), "", False, "No host")); return False
        url = f"{host}{path}"
        try:
            self.q.put_nowait(url)
            self.results.put((time.time(), url, True, "Queued"))
            return True
        except Exception:
            self.results.put((time.time(), url, False, "Queue full"))
            return False

    def _run(self):
        while not self._stop.is_set():
            try: item = self.q.get(timeout=0.2)
            except Empty: continue
            if item is None: break
            url = item
            last_msg, ok = "ERR", False
            for attempt in range(REQUEST_RETRIES + 1):
                try:
                    r = self.session.get(url, timeout=HTTP_TIMEOUT)
                    if 200 <= r.status_code < 300:
                        ok, last_msg = True, f"API OK {r.status_code}"; break
                    else:
                        last_msg = f"HTTP {r.status_code}"
                except requests.Timeout:
                    last_msg = "Timeout"
                except Exception as e:
                    last_msg = f"ERR {e.__class__.__name__}"
                if attempt < REQUEST_RETRIES: time.sleep(REQUEST_BACKOFF * (attempt + 1))
            self.results.put((time.time(), url, ok, last_msg))
            self.q.task_done()

# ======================= Stability tracker ======================

class StabilityTracker:
    def __init__(self, window=STABLE_WINDOW, px=CENTER_JITTER_PX, area_ratio=AREA_JITTER_RATIO, min_span=STABLE_SPAN_SEC):
        self.window = window; self.center_tol = px; self.area_tol_ratio = area_ratio; self.min_span = min_span
        self.q = deque()
    def reset(self): self.q.clear()
    def push(self, t, center, area):
        self.q.append((t, center, area))
        while len(self.q) > self.window: self.q.popleft()
    def is_stable(self):
        if len(self.q) < max(6, int(self.window * 0.6)): return False
        times = [t for (t,_,_) in self.q]; centers = [c for (_,c,_) in self.q]; areas = [a for (_,_,a) in self.q]
        if times[-1] - times[0] < self.min_span: return False
        cx = [c[0] for c in centers]; cy = [c[1] for c in centers]
        if (max(cx)-min(cx) > self.center_tol) or (max(cy)-min(cy) > self.center_tol): return False
        amax, amin = max(areas), min(areas); amean = (amax + amin) / 2.0 if (amax + amin) > 0 else 1.0
        if (amax - amin) / amean > self.area_tol_ratio: return False
        return True

# ============================== Main ============================

def main():
    cap = open_camera(CAMERA_DEVICE, FRAME_WIDTH, FRAME_HEIGHT, FPS_TARGET)
    subtractor = make_subtractor()
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, MORPH_KERNEL)

    print(f"[INFO] Calibrating background for {CALIBRATION_SECONDS:.1f}s...")
    t0 = time.time(); calibrating = True

    tracker = StabilityTracker()
    object_present = False; last_seen_ts = 0.0; last_bbox = None
    stable_ok_frames = 0; ema_cx = ema_cy = ema_area = None

    # Last committed result to display at TL until cooldown ends
    committed_label = "Unknown"
    committed_conf  = 0.0
    show_tl_until_ts = 0.0

    # Status + HTTP
    next_allowed_time = 0.0
    br_status = ""
    worker = HttpWorker()

    try:
        while True:
            ok, frame = cap.read()
            if not ok: print("[WARN] Empty frame."); break
            now = time.time()

            lr = -1 if calibrating else (0.0 if object_present else BASE_LEARNING_RATE)
            fg = subtractor.apply(frame, learningRate=lr)
            _, fg_bin = cv2.threshold(fg, 200, 255, cv2.THRESH_BINARY)
            fg_bin = cv2.morphologyEx(fg_bin, cv2.MORPH_OPEN, kernel, iterations=1)
            fg_bin = cv2.morphologyEx(fg_bin, cv2.MORPH_CLOSE, kernel, iterations=2)

            if calibrating and (now - t0) >= CALIBRATION_SECONDS:
                calibrating = False; print("[INFO] Background calibration complete.")

            display = frame.copy()

            # Drain HTTP results -> BR status
            try:
                while True:
                    ts, url, ok_flag, msg = worker.results.get_nowait()
                    try: path_only = url.split('/', 3)[-1]
                    except Exception: path_only = url
                    br_status = truncate(f"{path_only}: {msg}", 46)
            except Empty:
                pass

            cooldown_active = now < next_allowed_time
            cooldown_left   = max(0.0, next_allowed_time - now)

            # Always draw committed result at TL while cycle is active
            if committed_label != "Unknown" and now < show_tl_until_ts:
                _draw_boxed_text(display, f"{committed_label}  {committed_conf*100:.0f}%", corner="tl",
                                 margin=HUD_MARGIN, font_scale=HUD_FONT_SCALE_TL)

            if calibrating:
                rem = max(0, int(CALIBRATION_SECONDS - (now - t0)))
                hud_status_tr(display, f"Calibrating background... {rem}s")

            else:
                contour, area_raw = largest_contour(fg_bin)
                candidate_present = False
                if contour is not None:
                    if (not object_present and area_raw >= AREA_ENTER_THRESH) or \
                       (object_present and area_raw >= AREA_EXIT_THRESH):
                        candidate_present = True

                if candidate_present and not cooldown_active:
                    object_present = True; last_seen_ts = now
                    x, y, w, h = cv2.boundingRect(contour); last_bbox = (x, y, w, h)
                    cx, cy = x + w/2.0, y + h/2.0; area = float(w*h)
                    if ema_cx is None: ema_cx, ema_cy, ema_area = cx, cy, area
                    else:
                        ema_cx = EMA_ALPHA*cx + (1-EMA_ALPHA)*ema_cx
                        ema_cy = EMA_ALPHA*cy + (1-EMA_ALPHA)*ema_cy
                        ema_area = EMA_ALPHA*area + (1-EMA_ALPHA)*ema_area
                    tracker.push(now, (ema_cx, ema_cy), ema_area)
                    cv2.rectangle(display, (x,y), (x+w,y+h), (0,255,255), 2)

                    if tracker.is_stable(): stable_ok_frames += 1
                    else:                   stable_ok_frames  = 0

                    if stable_ok_frames >= MIN_CONSEC_STABLE_FRAMES:
                        roi_bgr = frame[y:y+h, x:x+w]; roi_fg = fg_bin[y:y+h, x:x+w]
                        if EDGE_ERODE_ITERS or EDGE_DILATE_ITERS:
                            k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
                            if EDGE_ERODE_ITERS:  roi_fg = cv2.erode(roi_fg, k3, iterations=EDGE_ERODE_ITERS)
                            if EDGE_DILATE_ITERS: roi_fg = cv2.dilate(roi_fg, k3, iterations=EDGE_DILATE_ITERS)
                        roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
                        color_name, conf = dominant_basic_color(roi_hsv, roi_fg)
                        mapped = COLOR_TO_CLASS.get(color_name, "Unknown")

                        # Commit result + show TL persistently through cooldown
                        if mapped != "Unknown":
                            committed_label = mapped
                            committed_conf  = conf
                            # Start cycle: enqueue + cooldown
                            bin_name = CLASS_TO_BIN.get(mapped)
                            if bin_name:
                                worker.enqueue_bin(bin_name)
                                next_allowed_time = now + CYCLE_COOLDOWN_SEC
                                show_tl_until_ts  = next_allowed_time  # keep TL until cycle end
                                br_status = f"{bin_name}: queued"
                        hud_status_tr(display, "Stable")
                    else:
                        hud_status_tr(display, f"Stabilizing... {stable_ok_frames}/{MIN_CONSEC_STABLE_FRAMES}")

                else:
                    if cooldown_active:
                        hud_status_tr(display, f"Cooldown... {cooldown_left:.1f}s")
                    else:
                        if object_present and (now - last_seen_ts) <= PRESENCE_GRACE_SEC:
                            if last_bbox is not None:
                                x, y, w, h = last_bbox
                                cv2.rectangle(display, (x, y), (x+w, y+h), (0,255,255), 1)
                            hud_status_tr(display, "Holding presence...")
                        else:
                            object_present = False
                            tracker.reset(); last_bbox = None
                            stable_ok_frames = 0; ema_cx = ema_cy = ema_area = None
                            hud_status_tr(display, "No object")
                            # If cooldown is over, clear committed TL
                            if not cooldown_active and now >= show_tl_until_ts:
                                committed_label = "Unknown"
                                committed_conf  = 0.0

            # BR status
            if br_status:
                hud_br(display, br_status)

            # Footer
            cv2.putText(display, "ESC to quit",
                        (10, FRAME_HEIGHT - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

            cv2.imshow("Waste Class (async + cooldown + ESP32)", display)
            cv2.imshow("Foreground Mask", fg_bin)

            if (cv2.waitKey(1) & 0xFF) == 27: break

    finally:
        cap.release(); cv2.destroyAllWindows()
        try: worker.stop()
        except Exception: pass

if __name__ == "__main__":
    main()
