#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""鲁棒 ArUco + QR 二维码识别 ROS 节点（带多帧投票与双保险）

兼容原接口：
  Service  : /aruco/enable  (std_srvs/SetBool)
  Publisher: /aruco/detected_id (std_msgs/Int32)
  Publisher: /aruco/debug_image (sensor_msgs/Image)
  Publisher: /qr_result (std_msgs/String)   -- 新增：QR 原始内容
  Subscriber: ~image_topic (default /rflysim/sensor2/img_rgb)

改进点：
  1. 多帧投票：同一 ID 连续/累计出现 N 次才发布，抑制单帧误检。
  2. QRCodeDetector fallback：ArUco 失败时尝试 OpenCV QR 检测。
  3. 预处理增强：CLAHE + 中值滤波，提升仿真图像识别率。
  4. 自动去抖：发布成功后延迟一小段时间再 unsubscribe，避免漏帧。
"""

import rospy
import cv2
import numpy as np
from collections import deque, Counter
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool, SetBoolResponse

# --- cv_bridge: pure-numpy fallback (Boost.Python conflict on Python 3.9+) ---
try:
    from cv_bridge import CvBridge as _CvBridge
    _CV_BRIDGE_OK = True
except Exception:
    _CV_BRIDGE_OK = False


def _imgmsg_to_numpy(msg: Image) -> np.ndarray:
    """Zero-copy ROS Image -> numpy array without cv_bridge C++ extension."""
    enc = msg.encoding.lower()
    dtype_map = {
        "rgb8":   (np.uint8,   3), "bgr8":   (np.uint8,   3),
        "rgba8":  (np.uint8,   4), "bgra8":  (np.uint8,   4),
        "mono8":  (np.uint8,   1), "8uc1":   (np.uint8,   1),
        "mono16": (np.uint16,  1), "16uc1":  (np.uint16,  1),
        "32fc1":  (np.float32, 1),
    }
    if enc in dtype_map:
        dtype, channels = dtype_map[enc]
        arr = np.frombuffer(msg.data, dtype=dtype)
        arr = arr.reshape(msg.height, msg.width) if channels == 1 \
              else arr.reshape(msg.height, msg.width, channels)
        if enc == "rgb8":
            arr = arr[:, :, ::-1].copy()
        elif enc == "rgba8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        elif enc == "bgra8":
            arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        return arr
    elif _CV_BRIDGE_OK:
        return _CvBridge().imgmsg_to_cv2(msg, "bgr8")
    raise ValueError(f"Unsupported encoding: {msg.encoding}")


def _numpy_to_imgmsg(arr: np.ndarray, header, encoding="bgr8") -> Image:
    """numpy array -> ROS Image (no cv_bridge needed)."""
    if _CV_BRIDGE_OK:
        return _CvBridge().cv2_to_imgmsg(arr, encoding)
    msg = Image()
    msg.header = header
    msg.height, msg.width = arr.shape[:2]
    msg.encoding = encoding
    msg.step = arr.strides[0]
    msg.data = arr.tobytes()
    return msg


# ---------------------------------------------------------------------------

class RobustQrArucoNode:
    def __init__(self):
        rospy.init_node("aruco_detect_node", anonymous=False)

        # --- params ---
        self.image_topic    = rospy.get_param("~image_topic", "/rflysim/sensor2/img_rgb")
        self.dict_id        = rospy.get_param("~dictionary_id", 16)  # DICT_ARUCO_ORIGINAL
        self.show_debug     = rospy.get_param("~show_debug", False)
        self.save_dir       = rospy.get_param("~save_dir", "")
        enabled_default     = rospy.get_param("~enabled_default", False)

        # 多帧投票参数
        self.consensus_frames = max(1, rospy.get_param("~consensus_frames", 3))
        self.buffer_size      = max(self.consensus_frames,
                                    rospy.get_param("~buffer_size", 8))
        self.qr_fallback      = rospy.get_param("~qr_fallback", True)
        self.use_clahe        = rospy.get_param("~use_clahe", True)

        # --- ArUco detector ---
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.dict_id)
        params = cv2.aruco.DetectorParameters_create()
        # 使用较稳定的默认参数，不过度放宽，减少误检
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 53
        params.adaptiveThreshWinSizeStep = 4
        params.adaptiveThreshConstant = 7.0
        params.minMarkerPerimeterRate = 0.02
        params.maxMarkerPerimeterRate = 4.0
        params.polygonalApproxAccuracyRate = 0.05
        params.minCornerDistanceRate = 0.02
        params.perspectiveRemovePixelPerCell = 8
        params.perspectiveRemoveIgnoredMarginPerCell = 0.2
        params.errorCorrectionRate = 0.8
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 30
        params.cornerRefinementMinAccuracy = 0.1

        self.aruco_dict = aruco_dict
        self.params = params

        # --- QR fallback ---
        self.qr_detector = cv2.QRCodeDetector() if self.qr_fallback else None

        # --- state ---
        self.enabled       = enabled_default
        self.has_detection = False
        self.img_sub       = None   # dynamically managed
        self.id_buffer     = deque(maxlen=self.buffer_size)

        # --- publishers / services ---
        self.id_pub        = rospy.Publisher("/aruco/detected_id", Int32, queue_size=10)
        self.qr_pub        = rospy.Publisher("/qr_result", String, queue_size=10)
        self.debug_img_pub = rospy.Publisher("/aruco/debug_image", Image, queue_size=1)
        self.enable_srv    = rospy.Service("/aruco/enable", SetBool, self._enable_cb)

        if self.enabled:
            self._subscribe()

        rospy.loginfo("=" * 42)
        rospy.loginfo("[RobustQrArucoNode] Python node started")
        rospy.loginfo("[RobustQrArucoNode] Image topic   : %s", self.image_topic)
        rospy.loginfo("[RobustQrArucoNode] Dictionary    : %d (DICT_ARUCO_ORIGINAL=16, DICT_6X6_250=10)", self.dict_id)
        rospy.loginfo("[RobustQrArucoNode] Consensus     : %d frames", self.consensus_frames)
        rospy.loginfo("[RobustQrArucoNode] QR fallback   : %s", "ON" if self.qr_fallback else "OFF")
        rospy.loginfo("[RobustQrArucoNode] Detection     : %s",
                      "ENABLED" if self.enabled else "DISABLED (call /aruco/enable)")
        rospy.loginfo("=" * 42)

    # ------------------------------------------------------------------ subscribe
    def _subscribe(self):
        if self.img_sub is None:
            self.img_sub = rospy.Subscriber(
                self.image_topic, Image, self._image_cb, queue_size=1)
            rospy.loginfo("[RobustQrArucoNode] Subscribed to %s", self.image_topic)

    def _unsubscribe(self):
        if self.img_sub is not None:
            self.img_sub.unregister()
            self.img_sub = None
            rospy.loginfo("[RobustQrArucoNode] Unsubscribed (saving CPU)")

    def _deferred_unsubscribe(self, _event=None):
        """Unsubscribe via Timer so it runs OUTSIDE the image callback."""
        self._unsubscribe()
        rospy.loginfo("[RobustQrArucoNode] Detection auto-disabled after success (saving CPU)")

    # ------------------------------------------------------------------ service
    def _enable_cb(self, req):
        self.enabled       = req.data
        self.has_detection = False
        self.id_buffer.clear()
        if self.enabled:
            self._subscribe()
            rospy.loginfo("[RobustQrArucoNode] >>> Detection ENABLED <<<")
            return SetBoolResponse(success=True, message="ArUco/Qr detection enabled")
        else:
            self._unsubscribe()
            rospy.loginfo("[RobustQrArucoNode] >>> Detection DISABLED <<<")
            return SetBoolResponse(success=True, message="ArUco/Qr detection disabled")

    # ------------------------------------------------------------------ helpers
    def _preprocess(self, gray: np.ndarray) -> np.ndarray:
        """轻微增强对比度，提升仿真图像中二维码/ArUco 识别率。"""
        if self.use_clahe:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            gray = clahe.apply(gray)
        # 轻中值滤波去噪，保留边缘
        gray = cv2.medianBlur(gray, 3)
        return gray

    def _detect_aruco(self, gray: np.ndarray):
        """Return (id, corners) or (None, None)."""
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.params)
        if ids is not None and len(ids) > 0:
            return int(ids[0][0]), corners
        return None, None

    def _detect_qr(self, gray: np.ndarray):
        """Return numeric id if QR data is integer, else publish raw string and return None."""
        if self.qr_detector is None:
            return None
        data, _, _ = self.qr_detector.detectAndDecode(gray)
        if data:
            rospy.loginfo_throttle(0.5, "[RobustQrArucoNode] QR raw data: %s", data)
            self.qr_pub.publish(String(data=data))
            try:
                return int(data)
            except ValueError:
                rospy.logwarn("[RobustQrArucoNode] QR data is not numeric: '%s'", data)
                return None
        return None

    def _find_consensus(self):
        """返回 buffer 中满足 consensus_frames 的最频繁非负 ID，否则 None。"""
        valid_ids = [x for x in self.id_buffer if x is not None and x >= 0]
        if len(valid_ids) < self.consensus_frames:
            return None
        counter = Counter(valid_ids)
        best_id, best_count = counter.most_common(1)[0]
        if best_count >= self.consensus_frames:
            return best_id
        return None

    # ------------------------------------------------------------------ image cb
    def _image_cb(self, msg: Image):
        if not self.enabled:
            return

        try:
            bgr = _imgmsg_to_numpy(msg)
        except Exception as e:
            rospy.logerr_throttle(1.0, "[RobustQrArucoNode] Image decode error: %s", e)
            return

        if bgr is None or bgr.size == 0:
            return

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = self._preprocess(gray)

        # --- detection ---
        detected_id, corners = self._detect_aruco(gray)
        source = "aruco"

        if detected_id is None and self.qr_fallback:
            detected_id = self._detect_qr(gray)
            source = "qr"

        # --- voting buffer ---
        self.id_buffer.append(detected_id if detected_id is not None else -1)
        consensus_id = self._find_consensus()

        need_debug = self.show_debug or self.debug_img_pub.get_num_connections() > 0

        # --- draw debug image ---
        if need_debug or consensus_id is not None:
            dbg = bgr.copy()
            if detected_id is not None and corners is not None:
                cv2.aruco.drawDetectedMarkers(dbg, corners,
                                              np.array([[detected_id]], dtype=np.int32))
            if need_debug:
                if detected_id is not None:
                    status = f"{source.upper()} ID: {detected_id} (buffer: {list(self.id_buffer)})"
                    color = (0, 255, 0)
                else:
                    status = f"NO MATCH (buffer: {list(self.id_buffer)})"
                    color = (0, 0, 255)
                cv2.putText(dbg, status, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                if self.debug_img_pub.get_num_connections() > 0:
                    self.debug_img_pub.publish(_numpy_to_imgmsg(dbg, msg.header))
                if self.show_debug:
                    cv2.imshow("ArUco / QR Detection", dbg)
                    cv2.waitKey(1)

        # --- publish result ---
        if consensus_id is not None:
            self.id_pub.publish(Int32(data=consensus_id))

            if self.save_dir and need_debug:
                path = f"{self.save_dir}/qr_aruco_id_{consensus_id}.jpg"
                cv2.imwrite(path, dbg)
                rospy.loginfo("[RobustQrArucoNode] Image saved: %s", path)

            self.has_detection = True

            rospy.loginfo("\033[1;33m+==========================================+\033[0m")
            rospy.loginfo("\033[1;33m|  \033[1;32m*** QR/ArUco Marker Detected! ***\033[1;33m    |\033[0m")
            rospy.loginfo("\033[1;33m|                                          |\033[0m")
            rospy.loginfo("\033[1;33m|    \033[1;37mSource: \033[1;36m%-8s\033[1;33m                  |\033[0m", source)
            rospy.loginfo("\033[1;33m|    \033[1;37mDetected ID: \033[1;36m%-4d\033[1;33m                  |\033[0m",
                          consensus_id)
            rospy.loginfo("\033[1;33m+==========================================+\033[0m")

            # auto-disable: schedule unsubscription OUTSIDE this callback
            self.enabled = False
            rospy.Timer(rospy.Duration(0.05), self._deferred_unsubscribe, oneshot=True)
        else:
            rospy.loginfo_throttle(2.0,
                "[RobustQrArucoNode] Searching... buffer: %s",
                list(self.id_buffer))

    # ------------------------------------------------------------------ run
    def run(self):
        rospy.spin()
        if self.show_debug:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        node = RobustQrArucoNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("[RobustQrArucoNode] Fatal: %s", e)
        import traceback
        traceback.print_exc()
