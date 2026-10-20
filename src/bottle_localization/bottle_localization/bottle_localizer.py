#!/usr/bin/env python3
#coding=utf-8
"""藥罐 3D 定位節點。

把 bottle_detect_v2 的 2D 偵測框，結合 ZED 的深度圖，轉成每個藥罐瓶身中心
在 base_link 下的 3D 位置，發布給手臂抓取端。

本節點只負責：訂閱、時間配對、TF 查詢、呼叫 geometry.py、發布。
幾何計算全部在 geometry.py，不依賴 ROS，可以單獨測試。

輸入:
    detection_topic     std_msgs/String（JSON），stamp 放在 JSON 裡
    depth_topic         sensor_msgs/Image。32FC1（公尺，ZED wrapper）或 mono16（公釐，/depth_mm）
    camera_info_topic   ZED 左眼原生 camera_info
    TF                  target_frame <-> 左眼 optical frame
輸出:
    /bottle_detections_3d   vision_msgs/Detection3DArray
    /bottle_markers         visualization_msgs/MarkerArray（RViz 檢查用）

所有數值都從 params.yaml / bottle_dims.yaml 讀，程式裡不寫預設值。
參數說明見 README.md。
"""
import json
from collections import deque
from types import SimpleNamespace

import numpy as np
import rclpy
import rclpy.executors
import yaml
from rclpy.logging import get_logger
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from bottle_localization import geometry


#節點參數與型別。全部由 params.yaml 提供（bottle_dims_file 由 launch 帶進來），
#少任何一個就在啟動時報錯，不在程式裡放預設值。
PARAM_SPEC = (
    ('detection_topic',         str),
    ('depth_topic',             str),
    ('camera_info_topic',       str),
    ('target_frame',            str),
    ('bottle_dims_file',        str),
    ('class_names',             'str_list'),
    ('image_scale',             float),
    ('letterbox_top',           int),
    ('letterbox_left',          int),
    ('zoom',                    float),
    ('depth_is_zoomed',         bool),
    ('z_table',                 float),
    ('score_threshold',         float),
    ('min_valid_points',        int),
    ('edge_margin',             int),
    ('sync_tolerance',          float),
    ('depth_buffer_size',       int),
    ('tf_timeout',              float),
    ('smoothing_frames',        int),
    ('smoothing_timeout',       float),
    ('enable_size_check',       bool),
    ('size_check_ratio',        float),
    ('enable_workspace_bounds', bool),
    ('workspace_x_min',         float),
    ('workspace_x_max',         float),
    ('workspace_y_min',         float),
    ('workspace_y_max',         float),
    ('enable_top_surface_filter', bool),
    ('top_surface_margin',      float),
    ('enable_symmetric_arc',    bool),
    ('symmetric_arc_iterations', int),
    ('symmetric_arc_trim_pct',  float),
    ('marker_rgba',             'float_list'),
    ('marker_text_size',        float),
    ('warn_throttle_sec',       float),
)


def _cast(name, value, kind):
    """把參數值轉成指定型別；YAML 把 0.0 寫成 0 這種情況也收。"""
    try:
        if kind is bool:
            if not isinstance(value, bool):
                raise TypeError
            return value
        if kind is int:
            if isinstance(value, bool) or float(value) != int(value):
                raise TypeError
            return int(value)
        if kind is float:
            if isinstance(value, bool):
                raise TypeError
            return float(value)
        if kind is str:
            return str(value)
        if kind == 'str_list':
            return [str(v) for v in value]
        if kind == 'float_list':
            return [float(v) for v in value]
    except (TypeError, ValueError):
        pass
    raise ValueError(f'參數 {name} 的型別不對: {value!r}')


def stamp_to_ns(sec, nanosec):
    """ROS 時間 -> 整數奈秒，比對時間差用。"""
    return int(sec) * 1_000_000_000 + int(nanosec)


class BottleLocalizer(Node):
#國科會計畫 - 藥罐 3D 定位
    def __init__(self):
        super().__init__('bottle_localizer')

        self.p = self.load_params()
        self.validate_params()
        with open(self.p.bottle_dims_file, 'r', encoding='utf-8') as f:
            self.dims = geometry.parse_bottle_dims(yaml.safe_load(f), self.p.class_names)

        self._bridge = CvBridge()
        self._camera_info = None          #(fx, fy, cx, cy, frame_id)
        self._intr_cache = {}             #(影像寬, 高) -> Intrinsics
        self._depth_buffer = deque(maxlen=self.p.depth_buffer_size)
        self._depth_format = None         #(寬, 高, encoding)，第一次收到或改變時印 log
        self._smoother = geometry.CenterSmoother(
            self.p.smoothing_frames, self.p.smoothing_timeout)
        #選用修正（不在原始規格內，見 README「選用修正」）
        self._top_margin = self.p.top_surface_margin if self.p.enable_top_surface_filter else None
        self._arc_iterations = self.p.symmetric_arc_iterations if self.p.enable_symmetric_arc else 0
        self._workspace = None
        if self.p.enable_workspace_bounds:
            self._workspace = (self.p.workspace_x_min, self.p.workspace_x_max,
                               self.p.workspace_y_min, self.p.workspace_y_max)

        self.tf_buffer = Buffer()
        #TF 的訂閱是 ReentrantCallbackGroup，配合 main() 的 MultiThreadedExecutor，
        #lookup 在 timeout 內等待時 TF 仍收得到新資料。本節點自己的三個訂閱都在預設的
        #互斥群組裡，彼此不會同時執行，深度緩衝不需要加鎖。
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #YOLO 結果是小訊息，用 RELIABLE 一筆都不漏。camera_info 用 BEST_EFFORT 收，
        #不管 wrapper 那邊設成哪一種都接得上。
        qos_reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                  reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE)
        qos_best_effort = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                     reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE)
        #深度圖是 30 Hz、每張 2.3 MB 的大訊息。用 RELIABLE + 長佇列時，傳輸或處理稍慢就會
        #一路積壓（上機實測延遲從 66 ms 堆到 267 ms，偵測結果到時緩衝裡的深度都是舊的）。
        #改成和 bottle_detect_v2 收影像一樣的 BEST_EFFORT、只留最新幾張：跟不上就丟，不會積壓。
        qos_depth = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(CameraInfo, self.p.camera_info_topic,
                                 self.camera_info_callback, qos_best_effort)
        self.create_subscription(Image, self.p.depth_topic,
                                 self.depth_callback, qos_depth)
        self.create_subscription(String, self.p.detection_topic,
                                 self.detection_callback, qos_reliable)

        self.det_pub = self.create_publisher(Detection3DArray, '/bottle_detections_3d', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/bottle_markers', 10)

        self.get_logger().info(f"偵測: {self.p.detection_topic}")
        self.get_logger().info(
            f"深度: {self.p.depth_topic}（{'已' if self.p.depth_is_zoomed else '未'}套用數位變焦）")
        self.get_logger().info(f"內參: {self.p.camera_info_topic}")
        self.get_logger().info(
            f"座標換算: scale={self.p.image_scale} letterbox(top={self.p.letterbox_top}, "
            f"left={self.p.letterbox_left}) zoom={self.p.zoom}")
        self.get_logger().info(
            "藥罐尺寸: " + ", ".join(f"{k}=(D {d:.4f}, H {h:.4f})"
                                     for k, (d, h) in self.dims.items()))
        self.get_logger().info(f"輸出座標系: {self.p.target_frame}，z_table={self.p.z_table}")
        self.get_logger().info(
            f"選用修正: 去瓶蓋頂面={'開' if self._top_margin is not None else '關'}，"
            f"對稱弧={'開' if self._arc_iterations else '關'}")
        self.get_logger().info("Bottle Localizer Node Initialized")

    # -------------------- 參數 --------------------
    def load_params(self):
        """宣告並讀取全部參數。不給預設值，少了就報錯並列出缺哪些。"""
        values = {}
        missing = []
        for name, kind in PARAM_SPEC:
            self.declare_parameter(name, None, ParameterDescriptor(dynamic_typing=True))
            value = self.get_parameter(name).value
            if value is None:
                missing.append(name)
                continue
            values[name] = _cast(name, value, kind)
        if missing:
            raise ValueError('params.yaml 缺少參數: ' + ', '.join(missing))
        return SimpleNamespace(**values)

    def validate_params(self):
        """明顯不合理的參數在啟動時就擋下來。"""
        p = self.p
        problems = []
        if p.image_scale <= 0:
            problems.append(f'image_scale 必須 > 0（{p.image_scale}）')
        if p.letterbox_top < 0 or p.letterbox_left < 0:
            problems.append('letterbox_top / letterbox_left 不能是負的')
        if p.zoom < 1.0:
            problems.append(f'zoom 必須 >= 1（{p.zoom}）')
        if p.zoom != 1.0 and not p.depth_is_zoomed:
            #偵測框已經換算到變焦後的座標，深度圖卻沒有裁切，兩邊會對到不同的地方
            problems.append(f'zoom = {p.zoom} 時，深度圖也必須套用同樣的變焦'
                            f'（改用 /depth_mm 並設 depth_is_zoomed: true），'
                            f'ZED wrapper 的深度圖只能在 zoom = 1.0 時使用')
        if p.zoom != 1.0 and (p.letterbox_top or p.letterbox_left):
            #變焦後黑邊的位置會跟著變，touches_edge() 沒有處理這種組合
            problems.append('zoom != 1 時不支援黑邊（ZED 模式沒有黑邊，請確認設定）')
        for name in ('min_valid_points', 'depth_buffer_size', 'smoothing_frames'):
            if getattr(p, name) < 1:
                problems.append(f'{name} 必須 >= 1')
        if p.edge_margin < 0:
            problems.append('edge_margin 不能是負的')
        for name in ('sync_tolerance', 'tf_timeout', 'smoothing_timeout',
                     'size_check_ratio', 'marker_text_size', 'warn_throttle_sec'):
            if getattr(p, name) <= 0:
                problems.append(f'{name} 必須 > 0')
        if not p.class_names:
            problems.append('class_names 不能是空的')
        if len(p.marker_rgba) != 4:
            problems.append('marker_rgba 要有 4 個值 [r, g, b, a]')
        if p.top_surface_margin < 0:
            problems.append('top_surface_margin 不能是負的')
        if p.symmetric_arc_iterations < 1:
            problems.append('symmetric_arc_iterations 必須 >= 1')
        if not 0.0 <= p.symmetric_arc_trim_pct < 50.0:
            problems.append('symmetric_arc_trim_pct 必須在 [0, 50) 之間')
        if p.enable_workspace_bounds and (p.workspace_x_min >= p.workspace_x_max or
                                          p.workspace_y_min >= p.workspace_y_max):
            problems.append('啟用 enable_workspace_bounds 時，min 必須小於 max')
        if problems:
            raise ValueError('params.yaml 有問題:\n  ' + '\n  '.join(problems))

    # -------------------- 訂閱回呼 --------------------
    def camera_info_callback(self, msg: CameraInfo):
        """ZED 左眼原生內參。K = [fx 0 cx; 0 fy cy; 0 0 1]。"""
        info = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]),
                msg.header.frame_id)
        if info != self._camera_info:
            self._camera_info = info
            self._intr_cache.clear()
            self.get_logger().info(
                f"camera_info: {msg.width}x{msg.height} fx={info[0]:.2f} fy={info[1]:.2f} "
                f"cx={info[2]:.2f} cy={info[3]:.2f} frame={info[4]}")

    def depth_callback(self, msg: Image):
        """深度圖連同 header.stamp 存進緩衝，等偵測結果來配對。

        原樣存（passthrough），配對到偵測結果時才換成公尺，沒被用到的幀不花轉換的成本。
        """
        if msg.encoding not in geometry.DEPTH_SCALE_TO_M:
            self.get_logger().warn(
                f"[depth] 不支援的編碼 {msg.encoding}，"
                f"只支援 {', '.join(geometry.DEPTH_SCALE_TO_M)}",
                throttle_duration_sec=self.p.warn_throttle_sec)
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f"[depth] convert failed: {e}",
                throttle_duration_sec=self.p.warn_throttle_sec)
            return
        fmt = (msg.width, msg.height, msg.encoding)
        if fmt != self._depth_format:
            self._depth_format = fmt
            self.get_logger().info(f"深度圖: {msg.width}x{msg.height} {msg.encoding}")
        self._depth_buffer.append(
            (stamp_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec),
             msg.header.frame_id, msg.encoding, np.array(depth, copy=True)))

    def detection_callback(self, msg: String):
        """收到一筆 YOLO 結果：配對深度 -> 查 TF -> 逐框定位 -> 平滑 -> 發布。"""
        try:
            data = json.loads(msg.data)
            stamp = data['stamp']
            stamp_msg = TimeMsg(sec=int(stamp['sec']), nanosec=int(stamp['nanosec']))
            bottles = data.get('bottles', [])
        except Exception as e:
            self.get_logger().warn(f"[detection] JSON 格式不對: {e}",
                throttle_duration_sec=self.p.warn_throttle_sec)
            return

        if self._camera_info is None:
            self.get_logger().warn(f"還沒收到 camera_info（{self.p.camera_info_topic}）",
                throttle_duration_sec=self.p.warn_throttle_sec)
            return

        stamp_ns = stamp_to_ns(stamp_msg.sec, stamp_msg.nanosec)
        if stamp_ns == 0:
            self.get_logger().warn("偵測結果的 stamp 是 0，無法和深度圖配對",
                throttle_duration_sec=self.p.warn_throttle_sec)
            return

        matched = self.match_depth(stamp_ns)
        if matched is None:
            return
        depth_frame_id, depth_encoding, depth_raw = matched
        height, width = depth_raw.shape[:2]

        size = data.get('image_size')
        if size and [width, height] != [int(v) for v in size]:
            self.get_logger().warn(f"座標系不一致: 偵測端 {size} vs 深度圖 [{width}, {height}]，跳過",
                throttle_duration_sec=self.p.warn_throttle_sec)
            return

        candidates = self.select_candidates(bottles, width, height)
        stamp_sec = stamp_ns * 1e-9
        self._smoother.expire(stamp_sec)

        results = []
        if candidates:
            optical_frame = self._camera_info[4] or depth_frame_id
            transform = self.lookup(optical_frame, stamp_msg)
            if transform is None:
                return
            rot, trans = transform
            intr = self.intrinsics(width, height)
            depth_m = geometry.depth_to_meters(depth_raw, depth_encoding)
            for label, conf, bbox in candidates:
                diameter, bottle_h = self.dims[label]
                res, reason = geometry.localize(
                    depth_m, bbox, intr, diameter, bottle_h, rot, trans,
                    self.p.z_table, self.p.min_valid_points,
                    self.p.size_check_ratio if self.p.enable_size_check else None,
                    self._workspace, self._top_margin,
                    self._arc_iterations, self.p.symmetric_arc_trim_pct)
                if res is None:
                    self.get_logger().debug(f"{label} {bbox} 跳過: {reason}")
                    continue
                xy = self._smoother.update(label, res.center[:2], stamp_sec)
                center = np.array([xy[0], xy[1], res.center[2]])
                results.append((label, conf, center, diameter, bottle_h))

        self.publish(stamp_msg, results)

    # -------------------- 時間配對 / TF / 內參 --------------------
    def match_depth(self, stamp_ns):
        """在深度緩衝裡找時間最接近的那一張；超過 sync_tolerance 回 None。

        YOLO 輸出是 String，不能用 message_filters，所以自己配對。
        """
        if not self._depth_buffer:
            self.get_logger().warn(f"還沒收到深度圖（{self.p.depth_topic}）",
                throttle_duration_sec=self.p.warn_throttle_sec)
            return None
        best = min(self._depth_buffer, key=lambda item: abs(item[0] - stamp_ns))
        dt = abs(best[0] - stamp_ns) * 1e-9
        if dt > self.p.sync_tolerance:
            newest = self._depth_buffer[-1][0]
            self.get_logger().warn(f"偵測與深度時間差 {dt * 1000:.1f} ms > {self.p.sync_tolerance * 1000:.0f} ms，"
                f"跳過（偵測比最新深度{'新' if stamp_ns > newest else '舊'}）",
                throttle_duration_sec=self.p.warn_throttle_sec)
            return None
        return best[1], best[2], best[3]

    def lookup(self, optical_frame, stamp_msg):
        """查 optical -> target_frame 的轉換，回 (R, t)；t 就是相機在 target_frame 的位置。

        用影像的 stamp 查。這一版外參是 static transform，時間不影響結果；
        日後改成頭部馬達的動態 TF 時，這個寫法可以直接沿用。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.p.target_frame, optical_frame,
                Time(seconds=stamp_msg.sec, nanoseconds=stamp_msg.nanosec),
                timeout=Duration(seconds=self.p.tf_timeout))
        except TransformException as e:
            self.get_logger().warn(f"TF {self.p.target_frame} <- {optical_frame} 查詢失敗，跳過這一幀: {e}",
                throttle_duration_sec=self.p.warn_throttle_sec)
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        return (geometry.quaternion_to_matrix(q.x, q.y, q.z, q.w),
                np.array([t.x, t.y, t.z]))

    def intrinsics(self, width, height):
        """原生 camera_info 換算到偵測座標系的內參，依影像尺寸快取。"""
        key = (width, height)
        intr = self._intr_cache.get(key)
        if intr is None:
            fx, fy, cx, cy, _ = self._camera_info
            intr = geometry.detection_intrinsics(
                fx, fy, cx, cy, self.p.image_scale,
                self.p.letterbox_left, self.p.letterbox_top,
                self.p.zoom, width, height)
            self._intr_cache[key] = intr
            self.get_logger().info(
                f"偵測座標系 {width}x{height} 內參: fx'={intr.fx:.2f} fy'={intr.fy:.2f} "
                f"cx'={intr.cx:.2f} cy'={intr.cy:.2f}")
        return intr

    # -------------------- 框的前處理 --------------------
    def select_candidates(self, bottles, width, height):
        """信心值、類別、邊緣過濾；同一類別多個框時取信心值最高的。

        Returns:
            list: [(label, confidence, bbox), ...]，依 class_names 的順序。
        """
        best = {}
        for b in bottles:
            try:
                label = str(b['label'])
                conf = float(b['confidence'])
                bbox = tuple(int(round(float(v))) for v in b['bbox'])
                if len(bbox) != 4:
                    raise ValueError
            except Exception:
                self.get_logger().warn(f"[detection] 框的格式不對: {b}",
                    throttle_duration_sec=self.p.warn_throttle_sec)
                continue
            if conf < self.p.score_threshold:
                continue
            if label not in self.dims:
                self.get_logger().warn(f"類別 {label} 不在 bottle_dims.yaml 裡，跳過",
                    throttle_duration_sec=self.p.warn_throttle_sec)
                continue
            if geometry.touches_edge(bbox, width, height, self.p.edge_margin,
                                     self.p.letterbox_left, self.p.letterbox_top):
                self.get_logger().debug(f"{label} {bbox} 碰到畫面邊緣，跳過")
                continue
            prev = best.get(label)
            if prev is not None:
                self.get_logger().warn(f"{label} 同時出現不只一個框，只留信心值最高的",
                    throttle_duration_sec=self.p.warn_throttle_sec)
                if prev[1] >= conf:
                    continue
            best[label] = (label, conf, bbox)
        return [best[name] for name in self.p.class_names if name in best]

    # -------------------- 發布 --------------------
    def publish(self, stamp_msg, results):
        """發 Detection3DArray 與 MarkerArray。沒有有效偵測時照樣發空陣列。"""
        out = Detection3DArray()
        out.header.stamp = stamp_msg
        out.header.frame_id = self.p.target_frame

        markers = MarkerArray()
        clear = Marker()
        clear.header = out.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        r, g, b, a = self.p.marker_rgba
        for i, (label, conf, center, diameter, bottle_h) in enumerate(results):
            det = Detection3D()
            det.header = out.header
            det.id = label
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = float(conf)
            hyp.pose.pose.position.x = float(center[0])
            hyp.pose.pose.position.y = float(center[1])
            hyp.pose.pose.position.z = float(center[2])
            hyp.pose.pose.orientation.w = 1.0
            det.results.append(hyp)
            det.bbox.center.position = hyp.pose.pose.position
            det.bbox.center.orientation.w = 1.0
            det.bbox.size.x = float(diameter)
            det.bbox.size.y = float(diameter)
            det.bbox.size.z = float(bottle_h)
            out.detections.append(det)

            cyl = Marker()
            cyl.header = out.header
            cyl.ns = 'bottle'
            cyl.id = i
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position = hyp.pose.pose.position
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = float(diameter)
            cyl.scale.y = float(diameter)
            cyl.scale.z = float(bottle_h)
            cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = r, g, b, a
            markers.markers.append(cyl)

            txt = Marker()
            txt.header = out.header
            txt.ns = 'bottle_label'
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            #分兩行：RViz 2 的文字把空白畫得特別寬，寫成一行時類別和信心值會被拉得很開
            lines = [label, f"{conf:.2f}"]
            txt.pose.position.x = float(center[0])
            txt.pose.position.y = float(center[1])
            #文字以 pose 為中心上下置中，往上抬「行數 x 字高」，最下面那行剛好在瓶頂上方
            txt.pose.position.z = float(center[2] + bottle_h / 2.0 +
                                        len(lines) * self.p.marker_text_size)
            txt.pose.orientation.w = 1.0
            txt.scale.z = self.p.marker_text_size
            txt.color.r, txt.color.g, txt.color.b, txt.color.a = r, g, b, a
            txt.text = '\n'.join(lines)
            markers.markers.append(txt)

        self.det_pub.publish(out)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = BottleLocalizer()
    except (ValueError, OSError, yaml.YAMLError) as e:
        get_logger('bottle_localizer').fatal(f"啟動失敗: {e}")
        rclpy.shutdown()
        raise SystemExit(1)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
