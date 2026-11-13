"""深度處理節點：裁切對齊 + distance class 判斷 + 視覺化。

職責
  - 訂閱 ZED 深度圖，套用與 image_node 相同的中心裁切倍率，維持與色模逐像素對齊
  - 依 depth.ini 的 D1~D8 距離區間產生距離標籤圖，供 overlap_node 取交集
  - 產生深度視覺化畫面（JET 上色）與 All_distance / Single_distance 建模畫面

單位
  ZED 深度原生為公尺 (float32)，本節點對外（ini、網頁、msg）一律用公分 int，
  只在比對前換算成公尺。

對齊
  裁切邏輯必須與 image.py 的 image_callback 完全相同（同樣的 zoom、同樣的置中裁切），
  差別只在深度圖用 INTER_NEAREST 縮放 —— 線性插值會在物體邊緣造出不存在的中間距離。
"""
import configparser
import json
import os
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16, String

from nstc_msgs.msg import DepthValue, Location, Zoom
from nstc_msgs.srv import BuildParam, DepthInfo, SaveParam

# ZED 深度有效範圍，必須與 zedxm.yaml 的 min_depth / max_depth 一致，
# 否則 JET 上色的映射區間會對不上實際資料。改 yaml 時這裡要一起改，
# 或改用 ros2 param set 覆蓋（depth_min_m / depth_max_m）。
DEFAULT_DEPTH_MIN_M = 0.1
DEFAULT_DEPTH_MAX_M = 10.0
# 網頁滑桿範圍（公分），由上面兩個值換算
DIST_MIN_CM = int(round(DEFAULT_DEPTH_MIN_M * 100))
DIST_MAX_CM = int(round(DEFAULT_DEPTH_MAX_M * 100))


def blob_distance_cm(labels_cc, depth_img, idx, x, y, w, h, scale):
    """取單一 blob 的距離統計（公分）。

    只在該 blob 的 bbox 範圍內取樣，避免每個 blob 都掃過整張影像。
    用平均而非中心點：質心可能落在凹形物體之外而取到背景，且單點取樣受
    深度雜訊影響大；blob 本來就被距離區間框住，平均值不會偏離該區間。

    scale：depth_img 單位換算成公分的乘數（公尺圖用 100，公釐圖用 0.1）。
    """
    roi_lbl = labels_cc[y:y + h, x:x + w]
    roi_d = depth_img[y:y + h, x:x + w]
    m = cv2.compare(roi_lbl, idx, cv2.CMP_EQ)
    mean = cv2.mean(roi_d, mask=m)[0]
    d_min, d_max, _, _ = cv2.minMaxLoc(roi_d, mask=m)
    return {
        "distance_cm": int(round(mean * scale)),
        "distance_min_cm": int(round(d_min * scale)),
        "distance_max_cm": int(round(d_max * scale)),
    }


@dataclass
class DistanceRange:
    DistMin: int = 0    # 公分
    DistMax: int = 0    # 公分
    LabelName: str = ""


class DepthProcessNode(Node):
    def __init__(self):
        super().__init__('depth_process_node')
        self.get_logger().info("=======Depth Process On=======")

        # 距離 class：與色模的 8 色各自獨立，只是數量相同
        self.labels = [f"D{i}" for i in range(1, 9)]
        self.DistanceRange = {name: DistanceRange(LabelName=name) for name in self.labels}
        self.select_distance = self.labels[0]
        self.build_status = 0   # 0=All_distance, 1=Single_distance

        # 顯示色沿用 image.py 的 color_labels，label 1~8 對應 D1~D8
        self.color_labels = {
            "BlackLabel":   {"label": 1, "color": [255,   0, 255]},
            "BlueLabel":    {"label": 2, "color": [128,   0, 128]},
            "GreenLabel":   {"label": 3, "color": [  0,   0, 128]},
            "OrangeLabel":  {"label": 4, "color": [128,   0,   0]},
            "RedLabel":     {"label": 5, "color": [255, 255,   0]},
            "YellowLabel":  {"label": 6, "color": [128, 128,   0]},
            "WhiteLabel":   {"label": 7, "color": [  0, 255, 255]},
            "OthersLabel":  {"label": 8, "color": [255,   0, 128]},
        }
        # label 編號 -> BGR，一次查表產生偽彩色（避免逐 class 的 numpy 布林索引）
        self.color_lut = np.zeros((1, 256, 3), dtype=np.uint8)
        self.dist_color = {}
        for meta in self.color_labels.values():
            idx = int(meta["label"])
            bgr = np.array(meta["color"], dtype=np.uint8)
            self.color_lut[0, idx] = bgr
            self.dist_color[self.labels[idx - 1]] = bgr
        self._label_fill_cache = {}
        self._solid_cache = {}

        # --- 路徑解析：與 image.py 相同的規則 ---
        self.location = ""
        self.strategy_root = self._resolve_strategy_root()
        fallback_dir = Path.home() / "nstc" / "config" / "image"
        fallback_dir.mkdir(parents=True, exist_ok=True)
        self.path_dir = fallback_dir

        raw = self._read_strategy_ini_raw()
        if raw:
            self.location = self._canon_location(raw)
            self.get_logger().info(f"[BOOT] strategy.ini -> {raw} => {self.location}")

        # 裁切倍率：本節點只讀不寫，寫入端是 usb_cam / camera_param_bridge_node
        self.declare_parameter('zoom_in', 1.0)
        self.zoom = float(self.get_parameter('zoom_in').value)

        # JET 上色的映射區間，需與 zedxm.yaml 的 min_depth / max_depth 相符
        self.declare_parameter('depth_min_m', DEFAULT_DEPTH_MIN_M)
        self.declare_parameter('depth_max_m', DEFAULT_DEPTH_MAX_M)
        self.depth_min_m = float(self.get_parameter('depth_min_m').value)
        self.depth_max_m = float(self.get_parameter('depth_max_m').value)
        self.get_logger().info(
            f"[depth] colorize range {self.depth_min_m}m ~ {self.depth_max_m}m")

        # 連通元件最小面積門檻，與 image.py 的色模採同一預設值
        self.declare_parameter('min_blob_area', 375)

        # JSON 類 topic 的節流 + 去抖，數值與色模一致（最快 20Hz，內容沒變就不發）
        self._pub_period = 0.05
        self._last_pub_t = {"info": 0.0, **{f"det_{l}": 0.0 for l in self.labels}}
        self._last_payload = {l: "" for l in self.labels}
        self._last_info_payload = ""

        # depth=1：影像鏈路只留最新一幀，避免消費端變慢時佇列堆出數百 ms 延遲
        qos_img = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._qos_img = qos_img

        self.bridge = CvBridge()

        # Single_distance 用原始畫面當底 —— JET 上色的深度圖本身就用滿了所有色相，
        # 疊上純色遮罩根本分不出來，也看不出框到的實體是什麼。
        # 只在該模式訂閱：訂閱著就算 callback 直接 return，rclpy 仍會先把
        # 960x600 bgr8 反序列化成 Python 物件，要真的不訂閱才省得到。
        self._base_img = None
        self._base_sub = None

        self.depth_sub = self.create_subscription(
            Image, '/zed/zed_node/depth/depth_registered', self.depth_callback, qos_img)
        self.zoom_sub = self.create_subscription(
            Zoom, '/Zoom_In_Topic', self.zoom_callback, qos_img)
        self.location_sub = self.create_subscription(
            Location, '/location', self.location_callback, 1000)
        self.build_status_sub = self.create_subscription(
            Int16, '/DepthBuildStatus', self.build_status_callback, 10)
        self.depth_value_sub = self.create_subscription(
            DepthValue, '/DepthValue_Topic', self.depth_value_callback, 1000)

        qos_json = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE)
        # 與色模對等的輸出：每個距離 class 一條 JSON、一條彙總、一張二值總遮罩
        self.det_pubs = {
            name: self.create_publisher(String, f'depth_detections/{name}', qos_json)
            for name in self.labels
        }
        self.info_pub = self.create_publisher(String, 'depth_object_info', qos_json)
        self.mask_pub = self.create_publisher(Image, 'depth_mask', qos_img)
        # 裁切對齊後的深度值（公釐 uint16），給 overlap_node 算每個交集區塊的距離。
        # 用 mono16 而非原生 32FC1：資料量減半，公釐精度對這個用途綽綽有餘。
        self.depth_mm_pub = self.create_publisher(Image, 'depth_mm', qos_img)

        self.depth_view_pub = self.create_publisher(Image, 'depth_view', qos_img)
        self.depth_processed_pub = self.create_publisher(Image, 'depth_processed_image', qos_img)
        self.depth_label_pub = self.create_publisher(Image, 'depth_label_map', qos_img)

        self.load_srv = self.create_service(DepthInfo, '/LoadDepthInfo', self.load_depth_info_callback)
        self.save_srv = self.create_service(SaveParam, '/SaveDepth', self.save_depth_callback)
        self.build_srv = self.create_service(BuildParam, '/BuildDepthModel', self.build_model_callback)

        self.init_depth_from_ini()
        self.load_zoomin_from_ini()

    # ------------------------------------------------------------------
    # 路徑解析（與 image.py 一致，但本節點不寫 strategy.ini）
    # ------------------------------------------------------------------
    def _resolve_strategy_root(self) -> Path:
        for up in Path(__file__).resolve().parents:
            if up.name == "src" and (up / "strategy" / "strategy").is_dir():
                return up / "strategy" / "strategy"
        return Path.home() / "nstc" / "config"

    def _strategy_ini_path(self) -> Path:
        env = os.environ.get("tku_STRATEGY_INI")
        if env:
            return Path(env).expanduser().resolve()
        return self.strategy_root / "strategy.ini"

    def _read_strategy_ini_raw(self) -> str:
        try:
            for line in self._strategy_ini_path().read_text(encoding="utf-8").splitlines():
                if line.strip():
                    return line.strip()
        except OSError:
            pass
        return ""

    def _canon_location(self, raw: str) -> str:
        if not raw:
            return ""
        return str(self.strategy_root / Path(raw.strip().lstrip("/")))

    def _resolve_ini_path(self, filename: str) -> Path:
        base = Path(self.location) if self.location else Path(self.path_dir)
        ini = base if base.suffix else (base / filename)
        ini.parent.mkdir(parents=True, exist_ok=True)
        return ini

    # ------------------------------------------------------------------
    # 參數載入 / 儲存
    # ------------------------------------------------------------------
    def load_zoomin_from_ini(self):
        """depth_process_node 只在 zed 模式啟動，故以 ZedCameraSet.ini 為主、CameraSet.ini 為後援。"""
        for filename, section in (("ZedCameraSet.ini", "ZED Camera Set Parameter"),
                                  ("CameraSet.ini", "Camera Set Parameter")):
            ini_path = self._resolve_ini_path(filename)
            cfg = configparser.ConfigParser()
            if not cfg.read(str(ini_path)):
                continue
            if section not in cfg or "zoomin" not in cfg[section]:
                continue
            try:
                self.zoom = float(cfg[section]["zoomin"])
                self.get_logger().info(f"[{filename}] zoom={self.zoom} from {ini_path}")
                return
            except ValueError as e:
                self.get_logger().error(f"[{filename}] invalid zoomin: {e}")
        self.get_logger().warn(f"[zoomin] no usable ini found, keep zoom={self.zoom}")

    def init_depth_from_ini(self):
        ini_path = self._resolve_ini_path("depth.ini")
        cfg = configparser.ConfigParser()

        if not cfg.read(str(ini_path)):
            # 沒檔就建模板：把 30~1000cm 均分成 8 段，首次啟動即可看到效果
            step = (DIST_MAX_CM - DIST_MIN_CM) / len(self.labels)
            for i, name in enumerate(self.labels):
                lo = int(round(DIST_MIN_CM + step * i))
                hi = int(round(DIST_MIN_CM + step * (i + 1)))
                cfg[name] = {"distance_min": str(lo), "distance_max": str(hi)}
            try:
                with open(ini_path, "w") as f:
                    cfg.write(f)
                self.get_logger().info(f"[Depth INI] created template: {ini_path}")
            except OSError as e:
                self.get_logger().error(f"[Depth INI] failed to create template: {e}")

        for name in self.labels:
            if name in cfg:
                sec = cfg[name]
                self.DistanceRange[name].DistMin = self._clamp(sec.get("distance_min", 0))
                self.DistanceRange[name].DistMax = self._clamp(sec.get("distance_max", 0))
        self.get_logger().info(
            f"[Depth INI] loaded from {ini_path}: "
            + ", ".join(f"{n}[{self.DistanceRange[n].DistMin},{self.DistanceRange[n].DistMax}]"
                        for n in self.labels)
        )

    def save_depth_callback(self, request, response):
        del request
        try:
            ini_path = self._resolve_ini_path("depth.ini")
            cfg = configparser.ConfigParser()
            for name, data in self.DistanceRange.items():
                cfg[name] = {
                    "distance_min": str(int(data.DistMin)),
                    "distance_max": str(int(data.DistMax)),
                }
            with open(ini_path, "w") as f:
                cfg.write(f)
            self.get_logger().info(f"[Depth INI] saved: {ini_path}")
            response.already = True
        except OSError as e:
            self.get_logger().error(f"[Depth INI] save failed: {e}")
            response.already = False
        return response

    def build_model_callback(self, request, response):
        """Build：重新從 depth.ini 載回全部 8 個距離區間。"""
        del request
        self.init_depth_from_ini()
        response.already = True
        return response

    def load_depth_info_callback(self, request, response):
        name = request.distancelabel
        if name not in self.DistanceRange:
            self.get_logger().warn(f"[Depth INI] unknown distance label '{name}'")
            response.dmin, response.dmax = 0, 0
            return response

        self.select_distance = name
        ini_path = self._resolve_ini_path("depth.ini")
        cfg = configparser.ConfigParser()
        if cfg.read(str(ini_path)) and name in cfg:
            self.DistanceRange[name].DistMin = self._clamp(cfg[name].get("distance_min", 0))
            self.DistanceRange[name].DistMax = self._clamp(cfg[name].get("distance_max", 0))
        else:
            self.get_logger().warn(f"[Depth INI] section '{name}' not found, using memory")

        response.dmin = int(self.DistanceRange[name].DistMin)
        response.dmax = int(self.DistanceRange[name].DistMax)
        self.get_logger().info(f"[Depth INI] '{name}' -> [{response.dmin},{response.dmax}] cm")
        return response

    def _clamp(self, v) -> int:
        try:
            return max(0, min(DIST_MAX_CM, int(float(v))))
        except (TypeError, ValueError):
            return 0

    # ------------------------------------------------------------------
    # Topic callbacks
    # ------------------------------------------------------------------
    def zoom_callback(self, msg):
        self.zoom = float(msg.zoomin)

    def build_status_callback(self, msg):
        status = int(msg.data)
        if status == self.build_status:
            return
        self.build_status = status
        self.get_logger().info(f"DepthBuildStatus: {self.build_status}")
        self._sync_base_subscription()

    def _sync_base_subscription(self):
        """只在 Single_distance 模式訂閱原始畫面，避免其他模式白白反序列化每一幀。"""
        if self.build_status == 1:
            if self._base_sub is None:
                self._base_sub = self.create_subscription(
                    Image, '/zoom_in', self.base_callback, self._qos_img)
        elif self._base_sub is not None:
            self.destroy_subscription(self._base_sub)
            self._base_sub = None
            self._base_img = None

    def base_callback(self, msg):
        self._base_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_value_callback(self, msg):
        rng = self.DistanceRange[self.select_distance]
        rng.DistMin = self._clamp(msg.dmin)
        rng.DistMax = self._clamp(msg.dmax)

    def location_callback(self, msg):
        """切換 strategy：只重載自己的參數，strategy.ini 由 image_node 負責寫。"""
        raw = (msg.data or "").strip()
        if not raw:
            return
        rel = Path(raw)
        if rel.is_absolute():
            parts = rel.parts
            rel = Path(parts[-2]) / parts[-1] if len(parts) >= 2 else Path(rel.name)
        self.location = self._canon_location(str(rel))
        self.get_logger().info(f"[location] -> {self.location}")
        self.init_depth_from_ini()
        self.load_zoomin_from_ini()

    # ------------------------------------------------------------------
    # 影像緩衝快取
    # ------------------------------------------------------------------
    def _label_fill(self, label_id: int, h: int, w: int) -> np.ndarray:
        key = (label_id, h, w)
        buf = self._label_fill_cache.get(key)
        if buf is None:
            buf = np.full((h, w), label_id, dtype=np.uint8)
            self._label_fill_cache[key] = buf
        return buf

    def _solid(self, bgr, h: int, w: int) -> np.ndarray:
        key = (int(bgr[0]), int(bgr[1]), int(bgr[2]), h, w)
        buf = self._solid_cache.get(key)
        if buf is None:
            buf = np.empty((h, w, 3), dtype=np.uint8)
            buf[:] = bgr
            self._solid_cache[key] = buf
        return buf

    # ------------------------------------------------------------------
    # 主流程
    # ------------------------------------------------------------------
    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            height, width = depth.shape[:2]

            # --- 與 image.py 完全相同的置中裁切；深度用 INTER_NEAREST ---
            zoom = self.zoom if self.zoom and self.zoom > 0 else 1.0
            new_w = max(1, int(width / zoom))
            new_h = max(1, int(height / zoom))
            x1 = (width - new_w) // 2
            y1 = (height - new_h) // 2
            cropped = depth[y1:y1 + new_h, x1:x1 + new_w]
            depth_m = cv2.resize(cropped, (width, height), interpolation=cv2.INTER_NEAREST)

            # NaN / inf 一律視為無效，換成 0 後就落在所有距離區間之外
            depth_m = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
            valid = cv2.compare(depth_m, 0.0, cv2.CMP_GT)   # uint8 0/255

            stamp = {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec}
            depth_view = self._colorize(depth_m, valid)
            label_map, total_mask = self._build_label_map(depth_m, valid, stamp)

            if self.build_status == 1:
                vis = self._build_single(depth_m, valid, depth_view)
            else:
                vis = cv2.LUT(cv2.cvtColor(label_map, cv2.COLOR_GRAY2BGR), self.color_lut)

            self._publish(self.depth_view_pub, depth_view, 'bgr8', msg.header)
            self._publish(self.depth_processed_pub, vis, 'bgr8', msg.header)
            self._publish(self.depth_label_pub, label_map, 'mono8', msg.header)
            self._publish(self.mask_pub, total_mask, 'mono8', msg.header)
            depth_mm = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
            self._publish(self.depth_mm_pub, depth_mm, 'mono16', msg.header)
        except Exception as e:
            self.get_logger().error(f"Failed to process depth: {e}")

    def _colorize(self, depth_m, valid):
        """有效距離區間映射到 JET：近紅遠藍，無效區為黑。"""
        lo, hi = self.depth_min_m, self.depth_max_m
        clipped = np.clip(depth_m, lo, hi)
        norm = cv2.convertScaleAbs(clipped - lo, alpha=255.0 / max(hi - lo, 1e-6))
        view = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
        return cv2.bitwise_and(view, view, mask=valid)

    def _build_label_map(self, depth_m, valid, stamp):
        """產生距離標籤圖與二值總遮罩，同時發布各距離 class 的偵測結果。

        區間若重疊，後處理的 class 會覆蓋前者（與色模 label map 行為一致）。
        偵測輸出的欄位與節流策略刻意與色模 build_all_hsv_table 完全相同。
        """
        h, w = depth_m.shape[:2]
        label_map = np.zeros((h, w), dtype=np.uint8)
        total_mask = np.zeros((h, w), dtype=np.uint8)
        detections_all = {name: [] for name in self.labels}
        min_area = int(self.get_parameter('min_blob_area').value)

        for i, name in enumerate(self.labels):
            rng = self.DistanceRange[name]
            if rng.DistMin == 0 and rng.DistMax == 0:   # 未設定就跳過
                continue
            lo, hi = rng.DistMin / 100.0, rng.DistMax / 100.0
            if hi < lo:
                lo, hi = hi, lo
            mask = cv2.inRange(depth_m, lo, hi)
            mask = cv2.bitwise_and(mask, valid)

            total_mask = cv2.bitwise_or(total_mask, mask)
            cv2.copyTo(self._label_fill(i + 1, h, w), mask, label_map)

            objects = self._extract_objects(mask, name, min_area, depth_m)
            detections_all[name] = objects
            self._publish_detections(name, objects, stamp, w, h)

        self._publish_object_info(detections_all, stamp)
        return label_map, total_mask

    def _extract_objects(self, mask, name, min_area, depth_m):
        """連通元件 -> 物體清單。欄位與色模一致，另加距離資訊。"""
        num, labels_cc, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        objects = []
        for i in range(1, num):     # 0 是背景
            x, y, w_box, h_box, area = stats[i]
            if area < min_area:
                continue
            cx, cy = centroids[i]
            item = {
                "bbox": (int(x), int(y), int(w_box), int(h_box)),
                "centroid": (int(cx), int(cy)),
                "area": float(area),
                "aspect_ratio": float(w_box) / float(h_box) if h_box > 0 else 0.0,
                "label": name,
            }
            item.update(blob_distance_cm(labels_cc, depth_m, i, x, y, w_box, h_box, 100.0))
            objects.append(item)
        return objects

    def _publish_detections(self, name, objects, stamp, w, h):
        payload = json.dumps(
            {"stamp": stamp, "width": w, "height": h, "label": name, "objects": objects},
            separators=(',', ':'))
        now = time.time()
        key = f"det_{name}"
        if payload != self._last_payload[name] and (now - self._last_pub_t[key] >= self._pub_period):
            try:
                self.det_pubs[name].publish(String(data=payload))
                self._last_payload[name] = payload
                self._last_pub_t[key] = now
            except Exception as e:
                self.get_logger().warning(f"publish depth_detections/{name} failed: {e}")

    def _publish_object_info(self, detections_all, stamp):
        detections_all["_stamp"] = stamp
        payload = json.dumps(detections_all, separators=(',', ':'))
        now = time.time()
        if payload != self._last_info_payload and (now - self._last_pub_t["info"] >= self._pub_period):
            try:
                self.info_pub.publish(String(data=payload))
                self._last_info_payload = payload
                self._last_pub_t["info"] = now
            except Exception as e:
                self.get_logger().warning(f"publish depth_object_info failed: {e}")

    def _build_single(self, depth_m, valid, depth_view):
        """Single_distance：以原始畫面為底，只疊當前選取的距離區間。

        底圖用 /zoom_in 而非深度圖 —— JET 上色的深度圖已經用滿所有色相，
        疊上純色遮罩會分不出來，也看不出框到的是什麼實體。
        深度圖仍可在左側面板的「深度畫面」對照。

        /zoom_in 還沒收到或尺寸不符時退回用深度圖，避免畫面空白。
        """
        h, w = depth_m.shape[:2]
        base = self._base_img
        if base is None or base.shape[:2] != (h, w):
            base = depth_view

        rng = self.DistanceRange[self.select_distance]
        if rng.DistMin == 0 and rng.DistMax == 0:
            return base

        lo, hi = rng.DistMin / 100.0, rng.DistMax / 100.0
        if hi < lo:
            lo, hi = hi, lo
        mask = cv2.bitwise_and(cv2.inRange(depth_m, lo, hi), valid)

        bgr = self.dist_color[self.select_distance]
        solid = self._solid(bgr, h, w)
        fg = cv2.bitwise_and(solid, solid, mask=mask)
        bg = cv2.bitwise_and(base, base, mask=cv2.bitwise_not(mask))
        return cv2.add(bg, fg)

    def _publish(self, pub, img, encoding, header):
        out = self.bridge.cv2_to_imgmsg(img, encoding=encoding)
        out.header.stamp = header.stamp
        out.header.frame_id = header.frame_id
        pub.publish(out)


def main():
    rclpy.init()
    node = DepthProcessNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
