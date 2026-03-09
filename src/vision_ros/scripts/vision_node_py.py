#!/usr/bin/env python3

import sys
import threading
import time

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import rospy
from sensor_msgs.msg import Image

from autodrive_msgs.msg import HUAT_VisionDetections

try:
    import onnxruntime as ort
except ImportError:
    ort = None


# Cone color types
# Note: Vision outputs unified YELLOW (1), LiDAR classifies by size into YELLOW_SMALL/YELLOW_BIG
BLUE = 0
YELLOW = 1  # Unified yellow (vision output)
YELLOW_SMALL = 1  # Alias for compatibility
YELLOW_BIG = 2
RED = 3
NONE = 4
ORANGE_SMALL = 1  # Legacy orange type, mapped to YELLOW per latest spec
ORANGE_BIG = 2  # Legacy orange type, mapped to YELLOW_BIG per latest spec

QUALITY_GOOD = 0
QUALITY_DEGRADED = 1
QUALITY_POOR = 2
QUALITY_UNUSABLE = 3

STATE_NORMAL = 0
STATE_DEGRADED = 1
STATE_FALLBACK = 2
STATE_VISION_LOST = 3


class Detection:
    __slots__ = ("x", "y", "w", "h", "confidence", "class_id", "color_type")

    def __init__(self, x, y, w, h, confidence, class_id, color_type):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.confidence = confidence
        self.class_id = class_id
        self.color_type = color_type


class PendingImageFrame:
    __slots__ = ("msg", "receive_mono")

    def __init__(self, msg, receive_mono):
        self.msg = msg
        self.receive_mono = float(receive_mono)


class LatestFrameBuffer:
    __slots__ = (
        "stale_after_sec",
        "_pending",
        "replaced_total",
        "stale_total",
        "last_stale_age_ms",
    )

    def __init__(self, stale_after_sec):
        self.stale_after_sec = float(stale_after_sec)
        self._pending = None
        self.replaced_total = 0
        self.stale_total = 0
        self.last_stale_age_ms = 0.0

    def store(self, frame):
        if self._pending is not None:
            self.replaced_total += 1
        self._pending = frame

    def has_pending(self):
        return self._pending is not None

    def pending_depth(self):
        return 1 if self._pending is not None else 0

    def take_latest(self, now_mono):
        frame = self._pending
        self._pending = None
        if frame is None:
            return None

        age_sec = float(now_mono) - frame.receive_mono
        if self.stale_after_sec > 0.0 and age_sec > self.stale_after_sec:
            self.stale_total += 1
            self.last_stale_age_ms = max(age_sec * 1000.0, 0.0)
            return None

        return frame


class FrameStageTiming:
    __slots__ = (
        "header_stamp_sec",
        "receive_mono",
        "pick_mono",
        "preprocess_done_mono",
        "inference_done_mono",
        "postprocess_done_mono",
        "publish_done_mono",
        "publish_ros_sec",
    )

    def __init__(self, header_stamp_sec, receive_mono):
        self.header_stamp_sec = float(header_stamp_sec)
        self.receive_mono = float(receive_mono)
        self.pick_mono = float(receive_mono)
        self.preprocess_done_mono = float(receive_mono)
        self.inference_done_mono = float(receive_mono)
        self.postprocess_done_mono = float(receive_mono)
        self.publish_done_mono = float(receive_mono)
        self.publish_ros_sec = 0.0

    def mark_picked(self, pick_mono):
        self.pick_mono = float(pick_mono)
        self.preprocess_done_mono = self.pick_mono
        self.inference_done_mono = self.pick_mono
        self.postprocess_done_mono = self.pick_mono
        self.publish_done_mono = self.pick_mono

    def mark_preprocess_done(self, done_mono):
        self.preprocess_done_mono = float(done_mono)

    def mark_inference_done(self, done_mono):
        self.inference_done_mono = float(done_mono)

    def mark_postprocess_done(self, done_mono):
        self.postprocess_done_mono = float(done_mono)

    def mark_publish_done(self, done_mono, publish_ros_sec):
        self.publish_done_mono = float(done_mono)
        self.publish_ros_sec = float(publish_ros_sec)

    @staticmethod
    def _delta_ms(start_mono, end_mono):
        return max((float(end_mono) - float(start_mono)) * 1000.0, 0.0)

    def snapshot(self):
        end_to_end_lag_ms = 0.0
        if self.publish_ros_sec > 0.0 and self.header_stamp_sec > 0.0:
            end_to_end_lag_ms = max((self.publish_ros_sec - self.header_stamp_sec) * 1000.0, 0.0)

        return {
            "receive_to_pick_ms": self._delta_ms(self.receive_mono, self.pick_mono),
            "preprocess_ms": self._delta_ms(self.pick_mono, self.preprocess_done_mono),
            "inference_stage_ms": self._delta_ms(
                self.preprocess_done_mono, self.inference_done_mono
            ),
            "postprocess_ms": self._delta_ms(self.inference_done_mono, self.postprocess_done_mono),
            "publish_ms": self._delta_ms(self.postprocess_done_mono, self.publish_done_mono),
            "total_processing_ms": self._delta_ms(self.pick_mono, self.publish_done_mono),
            "end_to_end_publish_lag_ms": end_to_end_lag_ms,
        }


class TemporalTracker:
    def __init__(self, iou_threshold, max_miss, min_hits):
        self.iou_threshold = float(iou_threshold)
        self.max_miss = int(max_miss)
        self.min_hits = int(min_hits)
        self.tracks = []

    @staticmethod
    def _iou(a, b):
        ax1, ay1 = a.x - a.w * 0.5, a.y - a.h * 0.5
        ax2, ay2 = a.x + a.w * 0.5, a.y + a.h * 0.5
        bx1, by1 = b.x - b.w * 0.5, b.y - b.h * 0.5
        bx2, by2 = b.x + b.w * 0.5, b.y + b.h * 0.5
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
        union = a.w * a.h + b.w * b.h - inter
        return inter / union if union > 0.0 else 0.0

    def update(self, detections):
        for track in self.tracks:
            track["misses"] += 1

        matched = [False] * len(detections)
        for track in self.tracks:
            best_iou = 0.0
            best_idx = -1
            for idx, det in enumerate(detections):
                if matched[idx]:
                    continue
                iou = self._iou(track["det"], det)
                if iou > best_iou:
                    best_iou = iou
                    best_idx = idx
            if best_idx >= 0 and best_iou >= self.iou_threshold:
                track["det"] = detections[best_idx]
                track["hits"] += 1
                track["misses"] = 0
                matched[best_idx] = True

        for idx, det in enumerate(detections):
            if not matched[idx]:
                self.tracks.append({"det": det, "hits": 1, "misses": 0})

        self.tracks = [t for t in self.tracks if t["misses"] <= self.max_miss]
        return [t["det"] for t in self.tracks if t["hits"] >= self.min_hits]


class VisionNodePy:
    def __init__(self):
        self.bridge = CvBridge()
        self.frame_count = 0

        self.state = STATE_NORMAL
        self.consecutive_degraded = 0
        self.consecutive_poor = 0
        self.consecutive_unusable = 0
        self.consecutive_good = 0

        self.last_debug_pub = rospy.Time(0)
        self.last_diag_pub = rospy.Time(0)
        self.last_stage_timing = None
        self.skipped_inference_newer_pending = 0
        self.skipped_postprocess_newer_pending = 0
        self.last_fallback_ms = 0.0
        self.last_tracker_ms = 0.0

        self._load_params()
        self._init_backend()
        if not self.backend_ready and self.fallback_enabled:
            self.backend_name = "fallback_hsv"
        self.tracker = TemporalTracker(
            self.tracker_iou_threshold, self.tracker_max_miss, self.tracker_min_hits
        )

        self.detections_pub = rospy.Publisher(
            "/perception/vision/detections", HUAT_VisionDetections, queue_size=1
        )
        self.debug_image_pub = rospy.Publisher(
            "/perception/vision/debug_image", Image, queue_size=1
        )
        self.diag_pub_local = rospy.Publisher(
            "/perception/vision/diagnostics", DiagnosticArray, queue_size=1, latch=True
        )
        self.diag_pub_global = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

        self.latest_frame_buffer = LatestFrameBuffer(self.stale_frame_age_sec)
        self.latest_frame_cv = threading.Condition()
        self.worker_shutdown = False
        self.worker = threading.Thread(
            target=self._worker_loop, name="vision_node_py_worker", daemon=True
        )
        self.worker.start()
        rospy.on_shutdown(self._shutdown_worker)

        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self._image_callback, queue_size=1
        )

        rospy.loginfo(
            "[vision_node_py] Initialized - backend=%s, mode=%s, topic=%s, tracker=%s, "
            "fallback=%s, stale_frame_age_sec=%.3f, skip_heavy_if_newer_pending=%s, "
            "publish_debug_image=%s",
            self.backend_name,
            "onnx_inference" if self.backend_ready else "fallback_only",
            self.image_topic,
            "on" if self.tracker_enabled else "off",
            "on" if self.fallback_enabled else "off",
            self.stale_frame_age_sec,
            "on" if self.skip_heavy_if_newer_pending else "off",
            "on" if self.publish_debug_image else "off",
        )

    def _load_params(self):
        self.image_topic = rospy.get_param("~node/image_topic", "/camera/image_raw")
        self.max_detections = int(rospy.get_param("~detection/max_detections", 20))
        self.require_model = bool(rospy.get_param("~node/require_model", True))

        self.backend_type = rospy.get_param("~inference/backend_type", "onnx")
        self.model_path = rospy.get_param("~inference/model_path", "")
        self.input_width = int(rospy.get_param("~inference/input_width", 640))
        self.input_height = int(rospy.get_param("~inference/input_height", 640))
        self.conf_threshold = float(rospy.get_param("~detection/conf_threshold", 0.5))
        self.nms_threshold = float(rospy.get_param("~detection/nms_threshold", 0.45))
        self.num_threads = int(rospy.get_param("~inference/num_threads", 2))

        self.publish_debug_image = bool(rospy.get_param("~output/publish_debug_image", False))
        self.debug_image_rate = float(rospy.get_param("~output/debug_image_rate", 5.0))
        self.diag_rate_hz = 2.0
        self.stale_frame_age_sec = float(rospy.get_param("~node/stale_frame_age_sec", 0.5))
        self.skip_heavy_if_newer_pending = bool(
            rospy.get_param("~node/skip_heavy_if_newer_pending", True)
        )

        self.confidence_scale = 1000.0
        self.fallback_enabled = bool(rospy.get_param("~fallback/enabled", True))
        self.model_confidence_floor = float(
            rospy.get_param("~fallback/model_confidence_floor", 0.3)
        )
        self.fallback_min_area = float(rospy.get_param("~fallback/min_contour_area", 200.0))
        self.fallback_max_area = float(rospy.get_param("~fallback/max_contour_area", 50000.0))

        self.tracker_enabled = bool(rospy.get_param("~tracker/enabled", True))
        self.tracker_iou_threshold = float(rospy.get_param("~tracker/iou_threshold", 0.3))
        self.tracker_max_miss = int(rospy.get_param("~tracker/max_miss", 1))
        self.tracker_min_hits = int(rospy.get_param("~tracker/min_hits", 2))

        self.degraded_frame_count = int(rospy.get_param("~node/degraded_frame_count", 3))
        self.poor_frame_count = int(rospy.get_param("~node/poor_frame_count", 5))
        self.unusable_frame_count = int(rospy.get_param("~node/unusable_frame_count", 10))
        self.recovery_frame_count = int(rospy.get_param("~node/recovery_frame_count", 5))

        self.quality_thresholds = {
            "blur_good": float(rospy.get_param("~quality/blur_threshold", 200.0)),
            "blur_degraded": float(rospy.get_param("~quality/blur_degraded", 100.0)),
            "blur_poor": float(rospy.get_param("~quality/blur_poor", 50.0)),
            "brightness_low": float(rospy.get_param("~quality/brightness_low", 40.0)),
            "brightness_high": float(rospy.get_param("~quality/brightness_high", 220.0)),
            "brightness_very_low": float(rospy.get_param("~quality/brightness_very_low", 15.0)),
            "brightness_very_high": float(rospy.get_param("~quality/brightness_very_high", 250.0)),
            "overexposure_limit": float(rospy.get_param("~quality/overexposure_limit", 0.3)),
            "underexposure_limit": float(rospy.get_param("~quality/underexposure_limit", 0.3)),
            "overexposure_unusable": float(rospy.get_param("~quality/overexposure_unusable", 0.5)),
            "underexposure_unusable": float(
                rospy.get_param("~quality/underexposure_unusable", 0.5)
            ),
        }

        self.enhancer_cfg = {
            "auto_clahe": bool(rospy.get_param("~enhancement/auto_clahe", True)),
            "clahe_clip_limit": float(rospy.get_param("~enhancement/clahe_clip_limit", 2.0)),
            "auto_gamma": bool(rospy.get_param("~enhancement/auto_gamma", True)),
            "denoise_on_poor": bool(rospy.get_param("~enhancement/denoise_on_poor", True)),
            "sharpen_on_blur": bool(rospy.get_param("~enhancement/sharpen_on_blur", True)),
        }

        self.class_to_color_map = []
        class_map_param = rospy.get_param("~inference/class_to_color", [])
        if isinstance(class_map_param, list):
            for idx, value in enumerate(class_map_param):
                if not isinstance(value, int):
                    rospy.logwarn(
                        "[vision_node_py] inference/class_to_color[%d] is not int, ignored", idx
                    )
                    continue
                if value < 0 or value > 5:
                    rospy.logwarn(
                        "[vision_node_py] inference/class_to_color[%d]=%d out of range [0,5], "
                        "ignored",
                        idx,
                        value,
                    )
                    continue
                self.class_to_color_map.append(value)
            if self.class_to_color_map:
                rospy.loginfo(
                    "[vision_node_py] Loaded class remap entries: %d", len(self.class_to_color_map)
                )
        else:
            rospy.logwarn("[vision_node_py] inference/class_to_color must be an int array")

    def _init_backend(self):
        self.session = None
        self.backend_ready = False
        self.backend_name = "none"
        self.input_name = ""
        self.output_name = ""

        if self.backend_type == "fallback_only":
            self.backend_name = "fallback_hsv"
            return

        if self.backend_type != "onnx":
            if self.require_model:
                rospy.logfatal(
                    "[vision_node_py] Unsupported backend_type=%s with node/require_model=true",
                    self.backend_type,
                )
                raise RuntimeError("unsupported backend for required-model mode")
            rospy.logwarn(
                "[vision_node_py] backend_type=%s unsupported, fallback_only", self.backend_type
            )
            return

        if ort is None:
            if self.require_model:
                rospy.logfatal(
                    "[vision_node_py] onnxruntime not installed with node/require_model=true"
                )
                raise RuntimeError("onnxruntime missing")
            rospy.logwarn("[vision_node_py] onnxruntime not installed, running fallback only")
            return

        if not self.model_path:
            if self.require_model:
                rospy.logfatal(
                    "[vision_node_py] inference/model_path is empty while node/require_model=true"
                )
                raise RuntimeError("model_path required but empty")
            rospy.logwarn("[vision_node_py] model_path is empty, running fallback only")
            return

        providers = ["CPUExecutionProvider"]
        available = ort.get_available_providers()
        if "CUDAExecutionProvider" in available:
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]

        try:
            options = ort.SessionOptions()
            options.intra_op_num_threads = self.num_threads
            options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.session = ort.InferenceSession(
                self.model_path, sess_options=options, providers=providers
            )
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            self.backend_ready = True
            self.backend_name = "onnx"
        except Exception as exc:
            if self.require_model:
                rospy.logfatal("[vision_node_py] Failed to initialize ONNX backend: %s", str(exc))
                raise
            rospy.logwarn("[vision_node_py] Failed to initialize ONNX backend: %s", str(exc))

    def spin(self):
        rospy.spin()

    def _shutdown_worker(self):
        with self.latest_frame_cv:
            self.worker_shutdown = True
            self.latest_frame_cv.notify_all()
        if hasattr(self, "worker") and self.worker.is_alive():
            self.worker.join(timeout=1.0)

    def _image_callback(self, msg):
        with self.latest_frame_cv:
            self.latest_frame_buffer.store(
                PendingImageFrame(msg=msg, receive_mono=time.perf_counter())
            )
            self.latest_frame_cv.notify()

    @staticmethod
    def _current_ros_time_sec():
        now_ros = rospy.Time.now().to_sec()
        if now_ros > 0.0:
            return now_ros
        return time.time()

    def _worker_loop(self):
        while not rospy.is_shutdown():
            with self.latest_frame_cv:
                while not self.worker_shutdown and not self.latest_frame_buffer.has_pending():
                    self.latest_frame_cv.wait(timeout=0.1)
                if self.worker_shutdown:
                    return
                pick_mono = time.perf_counter()
                pending_frame = self.latest_frame_buffer.take_latest(now_mono=pick_mono)

            if pending_frame is None:
                if self.latest_frame_buffer.last_stale_age_ms > 0.0:
                    rospy.logwarn_throttle(
                        2.0,
                        "[vision_node_py] Dropped stale frame age_ms=%.1f total=%d",
                        self.latest_frame_buffer.last_stale_age_ms,
                        self.latest_frame_buffer.stale_total,
                    )
                continue

            self._process_pending_frame(pending_frame, pick_mono)

    def _process_pending_frame(self, pending_frame, pick_mono):
        try:
            bgr = self.bridge.imgmsg_to_cv2(pending_frame.msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logerr_throttle(5.0, "[vision_node_py] cv_bridge exception: %s", str(exc))
            return

        self._process_frame(bgr, pending_frame.msg.header, pending_frame.receive_mono, pick_mono)

    def _process_frame(self, bgr, header, receive_mono, pick_mono):
        self.frame_count += 1
        timing = FrameStageTiming(header_stamp_sec=header.stamp.to_sec(), receive_mono=receive_mono)
        timing.mark_picked(pick_mono)
        quality_metrics = self._assess_quality(bgr)
        self._update_state(quality_metrics["overall"])

        enhanced = self._enhance_image(bgr, quality_metrics["overall"])
        timing.mark_preprocess_done(time.perf_counter())

        model_detections = []
        inference_us = 0
        newer_pending_before_inference = (
            self.skip_heavy_if_newer_pending and self.latest_frame_buffer.has_pending()
        )
        skip_model_for_freshness = (
            newer_pending_before_inference and self.fallback_enabled and self.backend_ready
        )
        if skip_model_for_freshness:
            self.skipped_inference_newer_pending += 1
        skip_model = (
            skip_model_for_freshness
            or quality_metrics["overall"] == QUALITY_UNUSABLE
            or self.state == STATE_VISION_LOST
            or self.state == STATE_FALLBACK
        )

        if not skip_model and self.backend_ready:
            t0 = time.perf_counter()
            model_detections = self._detect_onnx(enhanced, bgr.shape[1], bgr.shape[0])
            inference_done_mono = time.perf_counter()
            inference_us = int((inference_done_mono - t0) * 1e6)
            timing.mark_inference_done(inference_done_mono)
        else:
            timing.mark_inference_done(time.perf_counter())

        # Check if a newer frame is already waiting before doing heavy postprocess work.
        newer_pending = newer_pending_before_inference or (
            self.skip_heavy_if_newer_pending and self.latest_frame_buffer.has_pending()
        )
        if newer_pending:
            self.skipped_postprocess_newer_pending += 1

        _t0_fallback = time.perf_counter()
        fallback_detections = []
        _need_fallback = self.fallback_enabled and (
            not self.backend_ready
            or quality_metrics["overall"] == QUALITY_UNUSABLE
            or self.state in (STATE_VISION_LOST, STATE_FALLBACK)
        )
        if _need_fallback:
            fallback_detections = self._detect_fallback_hsv(enhanced)
        self.last_fallback_ms = (time.perf_counter() - _t0_fallback) * 1000.0

        detections = self._fuse_detections(
            model_detections, fallback_detections, quality_metrics["overall"], self.state
        )

        _t0_tracker = time.perf_counter()
        if self.tracker_enabled and not newer_pending:
            detections = self.tracker.update(detections)
        self.last_tracker_ms = (time.perf_counter() - _t0_tracker) * 1000.0

        detections = self._filter_topk(detections, self.max_detections)
        timing.mark_postprocess_done(time.perf_counter())

        self._publish_detections(detections, header, quality_metrics, inference_us)
        if self.publish_debug_image and not newer_pending:
            self._publish_debug_image(enhanced, detections, header)
        timing.mark_publish_done(time.perf_counter(), self._current_ros_time_sec())
        self.last_stage_timing = timing.snapshot()
        self._publish_diagnostics(quality_metrics, len(detections), inference_us)

    def _assess_quality(self, bgr):
        metrics = {
            "blur_score": 0.0,
            "brightness": 0.0,
            "contrast": 0.0,
            "overexposure_ratio": 0.0,
            "underexposure_ratio": 0.0,
            "overall": QUALITY_UNUSABLE,
        }

        if bgr is None or bgr.size == 0:
            return metrics

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        lap = cv2.Laplacian(gray, cv2.CV_64F)
        _, sigma_lap = cv2.meanStdDev(lap)
        metrics["blur_score"] = float(sigma_lap[0][0] * sigma_lap[0][0])

        mu_gray, sigma_gray = cv2.meanStdDev(gray)
        metrics["brightness"] = float(mu_gray[0][0])
        metrics["contrast"] = float(sigma_gray[0][0])

        total = float(gray.size)
        if total > 0:
            metrics["overexposure_ratio"] = float(np.count_nonzero(gray > 240)) / total
            metrics["underexposure_ratio"] = float(np.count_nonzero(gray < 15)) / total

        t = self.quality_thresholds
        if (
            metrics["blur_score"] < t["blur_poor"]
            or metrics["overexposure_ratio"] > t["overexposure_unusable"]
            or metrics["underexposure_ratio"] > t["underexposure_unusable"]
            or metrics["brightness"] < t["brightness_very_low"]
            or metrics["brightness"] > t["brightness_very_high"]
        ):
            metrics["overall"] = QUALITY_UNUSABLE
        elif (
            metrics["blur_score"] < t["blur_degraded"]
            or metrics["brightness"] < t["brightness_low"]
            or metrics["brightness"] > t["brightness_high"]
            or metrics["overexposure_ratio"] > t["overexposure_limit"]
        ):
            metrics["overall"] = QUALITY_POOR
        elif metrics["blur_score"] < t["blur_good"] or metrics["contrast"] < 30.0:
            metrics["overall"] = QUALITY_DEGRADED
        else:
            metrics["overall"] = QUALITY_GOOD

        return metrics

    def _enhance_image(self, bgr, quality):
        if quality in (QUALITY_GOOD, QUALITY_UNUSABLE):
            return bgr  # no-copy: callers never modify the array in place

        out = bgr.copy()
        cfg = self.enhancer_cfg

        if cfg["auto_clahe"]:
            lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
            channels = list(cv2.split(lab))
            clahe = cv2.createCLAHE(clipLimit=cfg["clahe_clip_limit"], tileGridSize=(8, 8))
            channels[0] = clahe.apply(channels[0])
            lab = cv2.merge(channels)
            out = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        gray = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
        brightness = float(np.mean(gray))
        if cfg["auto_gamma"]:
            gamma = 1.0
            if brightness < 80.0:
                gamma = 0.6
            elif brightness > 180.0:
                gamma = 1.5
            if gamma != 1.0:
                lut = np.array(
                    [np.clip(((i / 255.0) ** gamma) * 255.0, 0.0, 255.0) for i in range(256)],
                    dtype=np.uint8,
                )
                out = cv2.LUT(out, lut)

        if quality == QUALITY_POOR:
            if cfg["denoise_on_poor"]:
                small = cv2.resize(out, (0, 0), fx=0.5, fy=0.5)
                denoised = cv2.bilateralFilter(small, 5, 50, 50)
                out = cv2.resize(denoised, (out.shape[1], out.shape[0]))
            if cfg["sharpen_on_blur"]:
                blurred = cv2.GaussianBlur(out, (0, 0), 2.0)
                out = cv2.addWeighted(out, 1.5, blurred, -0.5, 0)

        return out

    def _detect_fallback_hsv(self, bgr):
        if bgr is None or bgr.size == 0:
            return []

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        results = []
        kernels = (
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
        )
        # HSV bounds are intentionally broad for degraded-light fallback.
        ranges = [
            (
                BLUE,
                np.array([90, 60, 60], dtype=np.uint8),
                np.array([140, 255, 255], dtype=np.uint8),
            ),
            (
                YELLOW,
                np.array([15, 60, 60], dtype=np.uint8),
                np.array([40, 255, 255], dtype=np.uint8),
            ),
            (
                ORANGE_SMALL,
                np.array([5, 80, 80], dtype=np.uint8),
                np.array([20, 255, 255], dtype=np.uint8),
            ),
        ]
        for color_type, lower, upper in ranges:
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernels[0])
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernels[1])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < self.fallback_min_area or area > self.fallback_max_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                if w <= 0 or h <= 0:
                    continue
                aspect = float(w) / float(h)
                if aspect < 0.25 or aspect > 2.5:
                    continue
                fill_ratio = area / float(w * h)
                if fill_ratio < 0.2:
                    continue
                confidence = min(1.0, max(0.2, 0.3 * fill_ratio + 0.2))
                results.append(
                    Detection(
                        x=float(x) + float(w) * 0.5,
                        y=float(y) + float(h) * 0.5,
                        w=float(w),
                        h=float(h),
                        confidence=confidence,
                        class_id=int(color_type),
                        color_type=int(color_type),
                    )
                )
        return results

    @staticmethod
    def _fuse_detections(model_dets, fallback_dets, quality, state):
        if state in (STATE_VISION_LOST, STATE_FALLBACK):
            return fallback_dets

        if quality == QUALITY_GOOD and model_dets and state == STATE_NORMAL:
            return model_dets

        if quality == QUALITY_UNUSABLE:
            return fallback_dets

        merged = list(model_dets)
        for fallback_det in fallback_dets:
            overlap = False
            for model_det in model_dets:
                dx = fallback_det.x - model_det.x
                dy = fallback_det.y - model_det.y
                dist = float(np.hypot(dx, dy))
                avg_size = (model_det.w + model_det.h + fallback_det.w + fallback_det.h) * 0.25
                if dist < avg_size * 0.5:
                    overlap = True
                    break
            if not overlap:
                merged.append(fallback_det)
        return merged

    def _detect_onnx(self, image_bgr, src_w, src_h):
        if not self.backend_ready or self.session is None:
            return []

        resized = cv2.resize(image_bgr, (self.input_width, self.input_height))
        blob = cv2.dnn.blobFromImage(
            resized,
            scalefactor=1.0 / 255.0,
            size=(self.input_width, self.input_height),
            mean=(0.0, 0.0, 0.0),
            swapRB=True,
            crop=False,
        )

        try:
            outputs = self.session.run([self.output_name], {self.input_name: blob})
        except Exception as exc:
            rospy.logerr_throttle(2.0, "[vision_node_py] ONNX inference failed: %s", str(exc))
            return []

        if not outputs:
            return []

        output = outputs[0]
        if output.ndim == 3 and output.shape[0] == 1:
            output = output[0]

        if output.ndim != 2:
            rospy.logerr_throttle(
                2.0, "[vision_node_py] Unexpected ONNX output shape: %s", str(list(output.shape))
            )
            return []

        rows, cols = int(output.shape[0]), int(output.shape[1])
        if rows <= 4 or cols <= 0:
            return []

        num_classes = rows - 4
        if num_classes <= 0:
            return []

        boxes = []
        scores = []
        class_ids = []

        for i in range(cols):
            cx = float(output[0, i])
            cy = float(output[1, i])
            w = float(output[2, i])
            h = float(output[3, i])

            cls_scores = output[4:, i]
            max_cls = int(np.argmax(cls_scores))
            max_score = float(cls_scores[max_cls])
            if max_score < self.conf_threshold:
                continue

            boxes.append([int(cx - w / 2.0), int(cy - h / 2.0), int(w), int(h)])
            scores.append(max_score)
            class_ids.append(max_cls)

        if not boxes:
            return []

        indices = cv2.dnn.NMSBoxes(boxes, scores, self.conf_threshold, self.nms_threshold)
        if indices is None or len(indices) == 0:
            return []

        scale_x = float(src_w) / float(self.input_width)
        scale_y = float(src_h) / float(self.input_height)

        detections = []
        for idx in np.array(indices).reshape(-1):
            box = boxes[int(idx)]
            det = Detection(
                x=(box[0] + box[2] * 0.5) * scale_x,
                y=(box[1] + box[3] * 0.5) * scale_y,
                w=float(box[2]) * scale_x,
                h=float(box[3]) * scale_y,
                confidence=float(scores[int(idx)]),
                class_id=int(class_ids[int(idx)]),
                color_type=self._model_class_to_color_type(int(class_ids[int(idx)])),
            )
            detections.append(det)

        return detections

    def _model_class_to_color_type(self, cls):
        """Map model class to color type.

        Vision outputs unified YELLOW (1), size classification done by LiDAR.
        """
        if self.class_to_color_map:
            if 0 <= cls < len(self.class_to_color_map):
                mapped = int(self.class_to_color_map[cls])
                if 0 <= mapped <= RED:
                    return mapped
            return NONE

        # Default map for vision model: 0=red, 1=blue, 2=yellow
        # Note: Vision outputs unified YELLOW, LiDAR classifies by size
        default_map = [RED, BLUE, YELLOW]
        if 0 <= cls < len(default_map):
            return default_map[cls]
        return NONE

    @staticmethod
    def _filter_topk(detections, max_det):
        if len(detections) <= max_det:
            return detections
        return sorted(detections, key=lambda d: d.confidence, reverse=True)[:max_det]

    def _publish_detections(self, detections, header, quality_metrics, inference_us):
        msg = HUAT_VisionDetections()
        msg.header = header
        msg.image_quality = int(quality_metrics["overall"])

        blur_score = float(quality_metrics["blur_score"])
        quality_score = 1.0 - (200.0 / blur_score if blur_score > 0.0 else 0.0) * 0.5
        msg.quality_score = max(0.0, min(1.0, quality_score))

        msg.x = [float(det.x) for det in detections]
        msg.y = [float(det.y) for det in detections]
        msg.color_types = [int(det.color_type) for det in detections]
        msg.confidences = [int(det.confidence * self.confidence_scale) for det in detections]
        msg.bbox_widths = [float(det.w) for det in detections]
        msg.bbox_heights = [float(det.h) for det in detections]

        using_fallback = (not self.backend_ready) or (
            quality_metrics["overall"] == QUALITY_UNUSABLE
        )
        msg.backend_name = self.backend_name
        msg.fallback_active = bool(using_fallback)
        msg.inference_time_us = int(max(inference_us, 0))

        self.detections_pub.publish(msg)

    def _publish_debug_image(self, bgr, detections, header):
        if self.debug_image_pub.get_num_connections() == 0:
            return
        now = rospy.Time.now()
        if self.last_debug_pub.to_sec() > 0.0 and self.debug_image_rate > 0.0:
            min_interval = 1.0 / self.debug_image_rate
            if (now - self.last_debug_pub).to_sec() < min_interval:
                return
        self.last_debug_pub = now

        canvas = bgr.copy()
        for det in detections:
            if det.color_type == BLUE:
                color = (255, 0, 0)
            elif det.color_type == YELLOW:
                color = (0, 255, 255)
            elif det.color_type == ORANGE_SMALL:
                color = (0, 165, 255)
            elif det.color_type == ORANGE_BIG:
                color = (0, 100, 255)
            elif det.color_type == RED:
                color = (0, 0, 255)
            else:
                color = (200, 200, 200)

            x1 = int(det.x - det.w * 0.5)
            y1 = int(det.y - det.h * 0.5)
            x2 = int(det.x + det.w * 0.5)
            y2 = int(det.y + det.h * 0.5)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)

        cv2.putText(
            canvas,
            self._state_to_string(self.state),
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )

        try:
            out_msg = self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
            out_msg.header = header
            self.debug_image_pub.publish(out_msg)
        except CvBridgeError as exc:
            rospy.logerr_throttle(
                5.0, "[vision_node_py] cv_bridge debug publish failed: %s", str(exc)
            )

    def _publish_diagnostics(self, quality_metrics, n_detections, inference_us):
        now = rospy.Time.now()
        if self.last_diag_pub.to_sec() > 0.0 and self.diag_rate_hz > 0.0:
            min_interval = 1.0 / self.diag_rate_hz
            if (now - self.last_diag_pub).to_sec() < min_interval:
                return
        self.last_diag_pub = now

        if self.state == STATE_NORMAL:
            level = DiagnosticStatus.OK
            message = "Vision operating normally"
        elif self.state == STATE_DEGRADED:
            level = DiagnosticStatus.WARN
            message = "Vision degraded - image quality reduced"
        elif self.state == STATE_FALLBACK:
            level = DiagnosticStatus.WARN
            message = "Vision fallback - using HSV color detection"
        else:
            level = DiagnosticStatus.ERROR
            message = "Vision lost - image unusable"

        kvs = [
            KeyValue(key="state", value=self._state_to_string(self.state)),
            KeyValue(key="n_detections", value=str(n_detections)),
            KeyValue(key="inference_time_us", value=str(inference_us)),
            KeyValue(key="image_quality", value=str(int(quality_metrics["overall"]))),
            KeyValue(key="blur_score", value=str(float(quality_metrics["blur_score"]))),
            KeyValue(key="brightness", value=str(float(quality_metrics["brightness"]))),
            KeyValue(key="frame_count", value=str(self.frame_count)),
            KeyValue(key="backend", value=self.backend_name),
            KeyValue(key="require_model", value="1" if self.require_model else "0"),
            KeyValue(key="tracker_enabled", value="1" if self.tracker_enabled else "0"),
            KeyValue(key="fallback_enabled", value="1" if self.fallback_enabled else "0"),
            KeyValue(key="model_path", value=self.model_path if self.model_path else "<empty>"),
            KeyValue(key="stale_frame_age_sec", value=str(self.stale_frame_age_sec)),
            KeyValue(
                key="frame_drop_replaced_total", value=str(self.latest_frame_buffer.replaced_total)
            ),
            KeyValue(key="frame_drop_stale_total", value=str(self.latest_frame_buffer.stale_total)),
            KeyValue(
                key="pending_frame_depth", value=str(self.latest_frame_buffer.pending_depth())
            ),
            KeyValue(
                key="last_stale_age_ms", value=str(self.latest_frame_buffer.last_stale_age_ms)
            ),
            KeyValue(
                key="skipped_inference_newer_pending",
                value=str(self.skipped_inference_newer_pending),
            ),
            KeyValue(
                key="skipped_postprocess_newer_pending",
                value=str(self.skipped_postprocess_newer_pending),
            ),
            KeyValue(
                key="skip_heavy_if_newer_pending",
                value="1" if self.skip_heavy_if_newer_pending else "0",
            ),
            KeyValue(key="fallback_ms", value=str(self.last_fallback_ms)),
            KeyValue(key="tracker_ms", value=str(self.last_tracker_ms)),
        ]

        stage_timing = self.last_stage_timing or {}
        kvs.extend(
            [
                KeyValue(
                    key="receive_to_pick_ms", value=str(stage_timing.get("receive_to_pick_ms", 0.0))
                ),
                KeyValue(key="preprocess_ms", value=str(stage_timing.get("preprocess_ms", 0.0))),
                KeyValue(
                    key="inference_stage_ms", value=str(stage_timing.get("inference_stage_ms", 0.0))
                ),
                KeyValue(key="postprocess_ms", value=str(stage_timing.get("postprocess_ms", 0.0))),
                KeyValue(key="publish_ms", value=str(stage_timing.get("publish_ms", 0.0))),
                KeyValue(
                    key="total_processing_ms",
                    value=str(stage_timing.get("total_processing_ms", 0.0)),
                ),
                KeyValue(
                    key="end_to_end_publish_lag_ms",
                    value=str(stage_timing.get("end_to_end_publish_lag_ms", 0.0)),
                ),
            ]
        )

        status = DiagnosticStatus()
        status.name = "vision_node"
        status.hardware_id = "camera"
        status.level = level
        status.message = message
        status.values = kvs

        arr = DiagnosticArray()
        arr.header.stamp = now
        arr.status = [status]

        self.diag_pub_local.publish(arr)
        self.diag_pub_global.publish(arr)

    def _update_state(self, quality):
        if quality == QUALITY_GOOD:
            self.consecutive_good += 1
            self.consecutive_degraded = 0
            self.consecutive_poor = 0
            self.consecutive_unusable = 0
        elif quality == QUALITY_DEGRADED:
            self.consecutive_degraded += 1
            self.consecutive_good = 0
            self.consecutive_poor = 0
            self.consecutive_unusable = 0
        elif quality == QUALITY_POOR:
            self.consecutive_poor += 1
            self.consecutive_good = 0
            self.consecutive_degraded = 0
            self.consecutive_unusable = 0
        else:
            self.consecutive_unusable += 1
            self.consecutive_good = 0
            self.consecutive_degraded = 0
            self.consecutive_poor = 0

        previous = self.state

        if self.state == STATE_NORMAL:
            if self.consecutive_unusable >= self.unusable_frame_count:
                self.state = STATE_VISION_LOST
            elif self.consecutive_poor >= self.poor_frame_count:
                self.state = STATE_FALLBACK
            elif self.consecutive_degraded >= self.degraded_frame_count:
                self.state = STATE_DEGRADED
        elif self.state == STATE_DEGRADED:
            if self.consecutive_unusable >= self.unusable_frame_count:
                self.state = STATE_VISION_LOST
            elif self.consecutive_poor >= self.poor_frame_count:
                self.state = STATE_FALLBACK
            elif self.consecutive_good >= self.recovery_frame_count:
                self.state = STATE_NORMAL
        elif self.state == STATE_FALLBACK:
            if self.consecutive_unusable >= self.unusable_frame_count:
                self.state = STATE_VISION_LOST
            elif self.consecutive_good >= self.recovery_frame_count:
                self.state = STATE_NORMAL
            elif self.consecutive_degraded >= self.recovery_frame_count:
                self.state = STATE_DEGRADED
        elif self.state == STATE_VISION_LOST:
            if self.consecutive_good >= self.recovery_frame_count:
                self.state = STATE_NORMAL
            elif self.consecutive_degraded >= self.recovery_frame_count:
                self.state = STATE_DEGRADED
            elif self.consecutive_poor >= self.recovery_frame_count:
                self.state = STATE_FALLBACK

        if previous != self.state:
            rospy.loginfo(
                "[vision_node_py] State transition: %s -> %s",
                self._state_to_string(previous),
                self._state_to_string(self.state),
            )

    @staticmethod
    def _state_to_string(state):
        if state == STATE_NORMAL:
            return "NORMAL"
        if state == STATE_DEGRADED:
            return "DEGRADED"
        if state == STATE_FALLBACK:
            return "FALLBACK"
        if state == STATE_VISION_LOST:
            return "VISION_LOST"
        return "UNKNOWN"


def main():
    rospy.init_node("vision_node")
    try:
        node = VisionNodePy()
    except Exception as exc:
        rospy.logfatal("[vision_node_py] startup failed: %s", str(exc))
        sys.exit(2)
    node.spin()


if __name__ == "__main__":
    main()
