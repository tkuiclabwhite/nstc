"""疊合節點：色彩 mask 與距離 mask 取交集。

Set1~Set8 各自綁定一組（顏色 class, 距離 class），交集區域以該 Set 的固定顯示色呈現。

輸入是兩張標籤圖而非 16 條二值遮罩：
  /color_label_map  0=背景，1~8=顏色 class
  /depth_label_map  0=無效，1~8=D1~D8
交集即 (color_map == Ci) AND (depth_map == Dj)，頻寬與運算都最省。

注意 /color_label_map 只在色模為 All_color 模式時更新；色模切到 Single_color
調單色期間，疊合畫面會停在最後一次的結果（不會壞，切回去就恢復）。
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

from nstc_msgs.msg import Location, OverlapValue
from nstc_msgs.srv import BuildParam, OverlapInfo, SaveParam

N_SETS = 8


def blob_distance_cm(labels_cc, depth_img, idx, x, y, w, h, scale):
    """取單一 blob 的距離統計（公分）。

    只在該 blob 的 bbox 範圍內取樣，避免每個 blob 都掃過整張影像。
    用平均而非中心點：質心可能落在凹形物體之外而取到背景，且單點取樣受
    深度雜訊影響大。

    scale：depth_img 單位換算成公分的乘數（公釐圖用 0.1）。
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
class OverlapSet:
    color: str = ""
    distance: str = ""
    enable: bool = False


class OverlapNode(Node):
    def __init__(self):
        super().__init__('overlap_node')
        self.get_logger().info("=======Overlap On=======")

        # 顏色 class 與距離 class 是各自獨立的分類系統，只是數量都是 8
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
        self.color_names = ["orange", "yellow", "blue", "green",
                            "black", "red", "white", "others"]
        self.distance_names = [f"D{i}" for i in range(1, N_SETS + 1)]
        self.set_names = [f"Set{i}" for i in range(1, N_SETS + 1)]

        # 顏色名稱 -> label_map 內的編號
        self.color_id = {
            name: int(self.color_labels[f"{name.capitalize()}Label"]["label"])
            for name in self.color_names
        }
        # D1~D8 -> 1~8
        self.distance_id = {name: i + 1 for i, name in enumerate(self.distance_names)}

        # Set 顯示色沿用同一張表，Set1~Set8 對應 label 1~8
        self.set_color = {}
        self.color_lut = np.zeros((1, 256, 3), dtype=np.uint8)
        for meta in self.color_labels.values():
            idx = int(meta["label"])
            bgr = np.array(meta["color"], dtype=np.uint8)
            self.color_lut[0, idx] = bgr
            self.set_color[self.set_names[idx - 1]] = bgr

        self.sets = {
            name: OverlapSet(color=self.color_names[0], distance=self.distance_names[0])
            for name in self.set_names
        }
        self.select_set = self.set_names[0]
        self.build_status = 0   # 0=All_overlap, 1=Single_overlap

        self._label_fill_cache = {}
        self._solid_cache = {}

        # --- 路徑解析：與 image.py 相同規則，但本節點不寫 strategy.ini ---
        self.location = ""
        self.strategy_root = self._resolve_strategy_root()
        fallback_dir = Path.home() / "nstc" / "config" / "image"
        fallback_dir.mkdir(parents=True, exist_ok=True)
        self.path_dir = fallback_dir
        raw = self._read_strategy_ini_raw()
        if raw:
            self.location = self._canon_location(raw)
            self.get_logger().info(f"[BOOT] strategy.ini -> {raw} => {self.location}")

        self.bridge = CvBridge()
        self._color_map = None
        self._base_img = None
        self._depth_mm = None
        self._warned_shape = False

        # depth=1：影像鏈路只保留最新一幀。用預設的 10，publisher 與 subscriber
        # 各堆 10 幀，消費端一慢就會累積出數百 ms 的延遲。
        qos_img = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE)

        # 疊合畫面只給人看，網頁顯示 320x200；用全解析度發布會讓
        # cv2_to_imgmsg、DDS 傳輸與 web_video_server 的 JPEG 編碼都多花 7.5 倍成本。
        # 交集運算仍在 label map 的原解析度上進行，精度不受影響。
        self.declare_parameter('display_width', 480)
        self.display_width = max(80, int(self.get_parameter('display_width').value))

        # 連通元件最小面積門檻，與色模採同一預設值
        self.declare_parameter('min_blob_area', 375)

        # JSON 類 topic 的節流 + 去抖，數值與色模一致
        self._pub_period = 0.05
        self._last_pub_t = {"info": 0.0, **{f"det_{n}": 0.0 for n in self.set_names}}
        self._last_payload = {n: "" for n in self.set_names}
        self._last_info_payload = ""

        self.color_sub = self.create_subscription(
            Image, '/color_label_map', self.color_map_callback, qos_img)
        # 交集區塊的實際距離：本節點只拿得到 label map（class 編號），
        # 真正的距離值要由 depth_process_node 供給
        self.depth_mm_sub = self.create_subscription(
            Image, '/depth_mm', self.depth_mm_callback, qos_img)
        # 深度標籤圖到達時才觸發運算，色彩標籤圖與底圖取最新的一張
        self.depth_sub = self.create_subscription(
            Image, '/depth_label_map', self.depth_map_callback, qos_img)
        # 底圖只有 Single_overlap 用得到。訂閱著就算 callback 直接 return，
        # rclpy 仍會先把 960x600 bgr8 反序列化成 Python 物件，所以要真的不訂閱才省得到
        self._qos_img = qos_img
        self.base_sub = None
        self.build_status_sub = self.create_subscription(
            Int16, '/OverlapBuildStatus', self.build_status_callback, 10)
        self.value_sub = self.create_subscription(
            OverlapValue, '/OverlapValue_Topic', self.value_callback, 1000)
        self.location_sub = self.create_subscription(
            Location, '/location', self.location_callback, 1000)

        self.overlap_pub = self.create_publisher(Image, 'overlap_image', qos_img)
        # 與色模對等的輸出
        self.det_pubs = {
            name: self.create_publisher(String, f'overlap_detections/{name}', qos_img)
            for name in self.set_names
        }
        self.info_pub = self.create_publisher(String, 'overlap_object_info', qos_img)
        self.mask_pub = self.create_publisher(Image, 'overlap_mask', qos_img)
        self.label_pub = self.create_publisher(Image, 'overlap_label_map', qos_img)

        self.load_srv = self.create_service(
            OverlapInfo, '/LoadOverlapInfo', self.load_overlap_info_callback)
        self.save_srv = self.create_service(
            SaveParam, '/SaveOverlap', self.save_overlap_callback)
        self.build_srv = self.create_service(
            BuildParam, '/BuildOverlapModel', self.build_model_callback)

        self.init_overlap_from_ini()

    # ------------------------------------------------------------------
    # 路徑解析
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
    def init_overlap_from_ini(self):
        ini_path = self._resolve_ini_path("overlap.ini")
        cfg = configparser.ConfigParser()

        if not cfg.read(str(ini_path)):
            # 沒檔就建模板：全部停用，等使用者從網頁設定
            for name in self.set_names:
                cfg[name] = {
                    "color": self.color_names[0],
                    "distance": self.distance_names[0],
                    "enable": "0",
                }
            try:
                with open(ini_path, "w") as f:
                    cfg.write(f)
                self.get_logger().info(f"[Overlap INI] created template: {ini_path}")
            except OSError as e:
                self.get_logger().error(f"[Overlap INI] failed to create template: {e}")

        for name in self.set_names:
            if name not in cfg:
                continue
            sec = cfg[name]
            color = sec.get("color", self.color_names[0]).strip().lower()
            distance = sec.get("distance", self.distance_names[0]).strip().upper()
            if color not in self.color_id:
                self.get_logger().warn(f"[Overlap INI] {name}: unknown color '{color}'")
                color = self.color_names[0]
            if distance not in self.distance_id:
                self.get_logger().warn(f"[Overlap INI] {name}: unknown distance '{distance}'")
                distance = self.distance_names[0]
            self.sets[name] = OverlapSet(
                color=color,
                distance=distance,
                enable=sec.get("enable", "0").strip() not in ("0", "", "false", "False"),
            )

        enabled = [n for n in self.set_names if self.sets[n].enable]
        self.get_logger().info(
            f"[Overlap INI] loaded from {ini_path}, enabled: {enabled or '(none)'}")

    def save_overlap_callback(self, request, response):
        del request
        try:
            ini_path = self._resolve_ini_path("overlap.ini")
            cfg = configparser.ConfigParser()
            for name in self.set_names:
                s = self.sets[name]
                cfg[name] = {
                    "color": s.color,
                    "distance": s.distance,
                    "enable": "1" if s.enable else "0",
                }
            with open(ini_path, "w") as f:
                cfg.write(f)
            self.get_logger().info(f"[Overlap INI] saved: {ini_path}")
            response.already = True
        except OSError as e:
            self.get_logger().error(f"[Overlap INI] save failed: {e}")
            response.already = False
        return response

    def build_model_callback(self, request, response):
        del request
        self.init_overlap_from_ini()
        response.already = True
        return response

    def load_overlap_info_callback(self, request, response):
        idx = int(request.setid)
        if not 1 <= idx <= N_SETS:
            self.get_logger().warn(f"[Overlap INI] setid out of range: {idx}")
            response.color, response.distance, response.enable = "", "", False
            return response

        name = self.set_names[idx - 1]
        self.select_set = name

        ini_path = self._resolve_ini_path("overlap.ini")
        cfg = configparser.ConfigParser()
        if cfg.read(str(ini_path)) and name in cfg:
            sec = cfg[name]
            color = sec.get("color", self.color_names[0]).strip().lower()
            distance = sec.get("distance", self.distance_names[0]).strip().upper()
            if color in self.color_id and distance in self.distance_id:
                self.sets[name] = OverlapSet(
                    color=color,
                    distance=distance,
                    enable=sec.get("enable", "0").strip() not in ("0", "", "false", "False"),
                )

        s = self.sets[name]
        response.color = s.color
        response.distance = s.distance
        response.enable = s.enable
        self.get_logger().info(
            f"[Overlap INI] '{name}' -> {s.color} x {s.distance} enable={s.enable}")
        return response

    # ------------------------------------------------------------------
    # Topic callbacks
    # ------------------------------------------------------------------
    def build_status_callback(self, msg):
        status = int(msg.data)
        if status == self.build_status:
            return
        self.build_status = status
        self.get_logger().info(f"OverlapBuildStatus: {self.build_status}")
        self._sync_base_subscription()

    def _sync_base_subscription(self):
        """只在 Single_overlap 模式訂閱底圖，避免 All 模式白白反序列化每一幀。"""
        if self.build_status == 1:
            if self.base_sub is None:
                self.base_sub = self.create_subscription(
                    Image, '/zoom_in', self.base_callback, self._qos_img)
        elif self.base_sub is not None:
            self.destroy_subscription(self.base_sub)
            self.base_sub = None
            self._base_img = None

    def value_callback(self, msg):
        idx = int(msg.setid)
        if not 1 <= idx <= N_SETS:
            return
        name = self.set_names[idx - 1]
        color = (msg.color or "").strip().lower()
        distance = (msg.distance or "").strip().upper()
        if color not in self.color_id or distance not in self.distance_id:
            self.get_logger().warn(f"[Overlap] {name}: bad combo {color}/{distance}")
            return
        self.sets[name] = OverlapSet(color=color, distance=distance, enable=bool(msg.enable))
        self.select_set = name

    def location_callback(self, msg):
        raw = (msg.data or "").strip()
        if not raw:
            return
        rel = Path(raw)
        if rel.is_absolute():
            parts = rel.parts
            rel = Path(parts[-2]) / parts[-1] if len(parts) >= 2 else Path(rel.name)
        self.location = self._canon_location(str(rel))
        self.get_logger().info(f"[location] -> {self.location}")
        self.init_overlap_from_ini()

    def color_map_callback(self, msg):
        self._color_map = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

    def depth_mm_callback(self, msg):
        self._depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')

    def base_callback(self, msg):
        self._base_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # ------------------------------------------------------------------
    # 主流程
    # ------------------------------------------------------------------
    def _display_size(self, h, w):
        """等比縮到 display_width；已經比目標小就不動。"""
        if w <= self.display_width:
            return w, h
        return self.display_width, max(1, int(round(h * self.display_width / w)))

    def depth_map_callback(self, msg):
        try:
            depth_map = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            color_map = self._color_map
            if color_map is None:
                return
            if color_map.shape != depth_map.shape:
                if not self._warned_shape:
                    self.get_logger().error(
                        f"label map 尺寸不符，無法取交集："
                        f"color{color_map.shape} vs depth{depth_map.shape}")
                    self._warned_shape = True
                return
            self._warned_shape = False

            h, w = depth_map.shape[:2]
            stamp = {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec}
            if self.build_status == 1:
                vis = self._build_single(color_map, depth_map, h, w)
            else:
                vis = self._build_all(color_map, depth_map, h, w, stamp, msg.header)

            out = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            out.header.stamp = msg.header.stamp
            out.header.frame_id = msg.header.frame_id
            self.overlap_pub.publish(out)
        except Exception as e:
            self.get_logger().error(f"Failed to process overlap: {e}")

    def _intersect(self, color_map, depth_map, s: OverlapSet):
        """色彩 mask AND 距離 mask。"""
        cmask = cv2.compare(color_map, self.color_id[s.color], cv2.CMP_EQ)
        dmask = cv2.compare(depth_map, self.distance_id[s.distance], cv2.CMP_EQ)
        return cv2.bitwise_and(cmask, dmask)

    def _build_all(self, color_map, depth_map, h, w, stamp, header):
        """黑底 + 所有已啟用的 Set 各自的交集區域，以 Set 對應色疊加。

        偵測輸出只在本模式產生，與色模只在 All_color 時跑 build_all_hsv_table 一致。
        """
        set_map = np.zeros((h, w), dtype=np.uint8)
        total_mask = np.zeros((h, w), dtype=np.uint8)
        detections_all = {name: [] for name in self.set_names}
        min_area = int(self.get_parameter('min_blob_area').value)

        for i, name in enumerate(self.set_names):
            s = self.sets[name]
            if not s.enable:
                continue
            inter = self._intersect(color_map, depth_map, s)
            total_mask = cv2.bitwise_or(total_mask, inter)
            cv2.copyTo(self._label_fill(i + 1, h, w), inter, set_map)

            objects = self._extract_objects(inter, name, min_area, self._depth_mm)
            detections_all[name] = objects
            self._publish_detections(name, objects, stamp, w, h)

        self._publish_object_info(detections_all, stamp)
        self._publish_img(self.mask_pub, total_mask, 'mono8', header)
        self._publish_img(self.label_pub, set_map, 'mono8', header)

        # 先縮再上色：LUT 只需處理縮小後的像素量
        dw, dh = self._display_size(h, w)
        if (dw, dh) != (w, h):
            set_map = cv2.resize(set_map, (dw, dh), interpolation=cv2.INTER_NEAREST)
        return cv2.LUT(cv2.cvtColor(set_map, cv2.COLOR_GRAY2BGR), self.color_lut)

    def _extract_objects(self, mask, name, min_area, depth_mm):
        """連通元件 -> 物體清單。欄位與色模一致，另加距離資訊。

        depth_mm 尚未收到或尺寸不符時就略過距離欄位，其餘欄位照常輸出。
        """
        num, labels_cc, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        has_depth = depth_mm is not None and depth_mm.shape[:2] == mask.shape[:2]
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
            if has_depth:
                item.update(blob_distance_cm(labels_cc, depth_mm, i, x, y, w_box, h_box, 0.1))
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
                self.get_logger().warning(f"publish overlap_detections/{name} failed: {e}")

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
                self.get_logger().warning(f"publish overlap_object_info failed: {e}")

    def _publish_img(self, pub, img, encoding, header):
        out = self.bridge.cv2_to_imgmsg(img, encoding=encoding)
        out.header.stamp = header.stamp
        out.header.frame_id = header.frame_id
        pub.publish(out)

    def _build_single(self, color_map, depth_map, h, w):
        """原始畫面為底，只疊當前選取 Set 的交集區域。"""
        base = self._base_img
        if base is None or base.shape[:2] != (h, w):
            base = np.zeros((h, w, 3), dtype=np.uint8)

        s = self.sets[self.select_set]
        inter = self._intersect(color_map, depth_map, s)

        # 同樣先縮小再合成
        dw, dh = self._display_size(h, w)
        if (dw, dh) != (w, h):
            inter = cv2.resize(inter, (dw, dh), interpolation=cv2.INTER_NEAREST)
            base = cv2.resize(base, (dw, dh), interpolation=cv2.INTER_AREA)

        bgr = self.set_color[self.select_set]
        solid = self._solid(bgr, dh, dw)
        fg = cv2.bitwise_and(solid, solid, mask=inter)
        bg = cv2.bitwise_and(base, base, mask=cv2.bitwise_not(inter))
        return cv2.add(bg, fg)

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


def main():
    rclpy.init()
    node = OverlapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
