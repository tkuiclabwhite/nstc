#!/usr/bin/env python3
#coding=utf-8
"""藥罐偵測節點 v2 —— 四種藥罐（b1 / b2 / b3 / b4）。

跟 bottle_detect.py 是同一套流程，只差在模型會分出四種藥罐:
    1. 收 ZED 的彩色影像
    2. 丟進 YOLO 模型（預設 Parameter/best(1).onnx）做偵測
    3. 把偵測到的框從相機原生解析度換算到策略層的 960x600 座標系
    4. 依種類排序 b1 → b4，每一種只留信心值最高的一個框
    5. 每個框攤成四個角點（左上、右上、左下、右下），座標依種類固定編號
    6. print 出種類、信心值、座標，並發到 RESULT_TOPIC

座標編號規則（依**種類**，不依畫面位置；某一種沒看到時那四個編號就空著，
後面的不會往前遞補，下游拿 x9 就一定是 b3）::

    b1: x1,y1 左上   x2,y2 右上   x3,y3 左下   x4,y4 右下
    b2: x5,y5 左上   x6,y6 右上   x7,y7 左下   x8,y8 右下
    b3: x9,y9  ...                             x12,y12
    b4: x13,y13 ...                            x16,y16

RESULT_TOPIC 的 JSON 欄位與 v1 相同，name 是 bottle1..bottle4（對應 b1..b4），
label 是種類 "b1".."b4"，med_detect.py 不用改就能接。

畫面：發到 IMAGE_OUT_TOPIC（跟 v1 同一個 /med_detect/image），
瀏覽器開 http://<機器人IP>:8080/stream?topic=/med_detect/image 就看得到。
⚠ topic 跟 v1 一樣，所以 v1 與 v2 不要同時跑，兩邊的結果會混在一起。

模型的 metadata 讀取、座標換算等共用工具直接沿用 bottle_detect.py 的，
要改換算規則改那邊，兩個節點會一起生效。

http://172.17.121.10:8080/stream?topic=/med_detect/image

source ~/ros2_adult/install/setup.bash
source ~/nstc/install/local_setup.bash

"""
import json
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge

from tku_msgs.msg import Zoom

from med_detect.bottle_detect import (
    map_to_strategy, _literal, normalize_imgsz, load_model_metadata, local_ip,
    IMAGE_TOPIC, ZOOM_TOPIC, RESULT_TOPIC, IMAGE_OUT_TOPIC,
    IMAGE_W, IMAGE_H, CORNER_ORDER)


BASE_DIR = os.path.dirname(os.path.abspath(__file__))

#--- 模型 ---#
#四類藥罐的模型。也可以不改程式，用
#    ros2 run med_detect bottle_detect_v2 --ros-args -p engine_path:=/abs/path/xxx.onnx
#覆蓋。要換成 TensorRT 的話，記得 engine 要從**這顆** onnx 重新編
#（目前的 best(1).engine 比這顆 onnx 還早產生，不是同一顆模型）。
ENGINE_PATH                = os.path.join(BASE_DIR, 'Parameter', 'best(1).engine')
#模型匯出時綁定的輸入邊長。None = 開機時自動從模型檔讀。現在這顆是 [1, 3, 960, 960]，
#不要手動填別的值，原因見 bottle_detect.py 的 IMGSZ 說明。
IMGSZ                      = None
DEFAULT_IMGSZ              = 640
#信心值門檻，低於這個值的框直接丟掉
CONF_THRES                 = 0.70
#NMS 的 IoU 門檻
IOU_THRES                  = 0.45
#跨類別 NMS。多類別模型預設是「每一類各自做 NMS」，同一個藥罐若同時被判成
#b1 跟 b2 就會留下兩個疊在一起的框，編號整個往後擠。打開後重疊的框只留信心值最高的那個。
AGNOSTIC_NMS               = True
#畫面中最多幾瓶藥罐 = 模型的類別數。類別編號 0..3 依序是 b1..b4，
#類別編號 >= 這個值的框直接丟掉（座標編號沒有位置放）。
MAX_BOTTLES                = 4

#--- 偵測節流 ---#
#兩次推論的最小間隔(秒)。0 代表有新影像就推論。
MIN_INTERVAL               = 0.0

#--- 輸出 ---#
#沒偵測到任何藥罐時也印一行
LOG_EMPTY                  = False
#框碰到畫面邊緣時直接丟掉(True)或只是照常輸出(False)
REJECT_EDGE_BBOX           = False
EDGE_MARGIN_PX             = 2

#--- 畫面輸出 ---#
PUBLISH_IMAGE              = True
#發布用的寬度(像素)，高度等比。要「畫面上的像素座標 = 發出去的座標」就改成 960。
DISPLAY_WIDTH              = 960
DISPLAY_MAX_FPS            = 30.0
#每一**種**藥罐一個顏色(BGR)，同一種藥罐不管排第幾瓶都是同一個顏色
CLASS_COLORS               = {
    'b1': (0, 255, 0),          #綠
    'b2': (0, 200, 255),        #橘
    'b3': (255, 160, 0),        #藍
    'b4': (255, 0, 255),        #紫
}
#類別名稱不在上表時用的顏色
OTHER_COLOR                = (200, 200, 200)
STATUS_ROW_PX              = 34
#框四個角上的圓點 + 座標編號(x1/x2/...)。要核對編號規則時打開。
DRAW_CORNERS               = False
#框上方那行 b2 0.93（種類 + 信心值）
DRAW_LABEL                 = True


class BottleDetectV2(Node):
#國科會計畫 - 藥罐偵測（四種藥罐）
    def __init__(self):
        super().__init__('bottle_detect_v2_node')

        self.image_cbg = ReentrantCallbackGroup()
        self.qos_fast = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.declare_parameter('engine_path', ENGINE_PATH)
        self.declare_parameter('image_topic', IMAGE_TOPIC)
        engine_path = str(self.get_parameter('engine_path').value).strip() or ENGINE_PATH
        image_topic = str(self.get_parameter('image_topic').value).strip() or IMAGE_TOPIC

        #先問檔案、再載模型 —— 暖機就要用到輸入邊長
        meta, meta_src = load_model_metadata(engine_path)
        self.imgsz = IMGSZ or normalize_imgsz(meta.get('imgsz'))
        self.imgsz_src = 'IMGSZ 指定' if IMGSZ else meta_src

        self.model = self.load_engine(engine_path)
        if not self.imgsz:
            self.imgsz = self.backend_imgsz(self.model)

        self.names = _literal(meta.get('names')) or getattr(self.model, 'names', {}) or {}

        self._bridge = CvBridge()
        self.zoom = 1.0
        self._last_infer = 0.0

        self.result_pub = self.create_publisher(String, RESULT_TOPIC, 10)

        #畫面用 RELIABLE，web_video_server 是用 RELIABLE 訂閱的，
        #用 BEST_EFFORT 發的話網頁一片空白而且兩邊都不會報錯
        self.view_pub = None
        if PUBLISH_IMAGE:
            self.view_pub = self.create_publisher(
                RosImage, IMAGE_OUT_TOPIC,
                QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE))
        self._last_bottles = []
        self._last_view = 0.0
        self._infer_fps = 0.0
        self._infer_t = 0.0

        #推論在自己的執行緒，畫面才不會被推論拖慢（同 v1）
        self._pending = None
        self._pending_lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._worker = threading.Thread(
            target=self.infer_loop, name='bottle_v2_infer', daemon=True)
        self._worker.start()

        self.create_subscription(
            RosImage, image_topic, self.image_callback,
            self.qos_fast, callback_group=self.image_cbg)
        self.create_subscription(
            Zoom, ZOOM_TOPIC, self.zoom_callback,
            self.qos_fast, callback_group=self.image_cbg)

        self.get_logger().info(f"模型: {engine_path}")
        self.get_logger().info(f"輸入邊長: {self.imgsz}（來源: {self.imgsz_src}）")
        self.get_logger().info(f"類別: {self.names or '（模型沒帶，畫面上會顯示編號）'}")
        self.get_logger().info(f"影像來源: {image_topic}")
        self.get_logger().info(f"輸出: {RESULT_TOPIC}（座標系 {IMAGE_W}x{IMAGE_H}）")
        if self.view_pub is not None:
            self.get_logger().info(
                f"畫面: {IMAGE_OUT_TOPIC}  ->  "
                f"http://{local_ip()}:8080/stream?topic={IMAGE_OUT_TOPIC}")
        else:
            self.get_logger().info("畫面輸出關閉（PUBLISH_IMAGE = False）")
        self.get_logger().info("Bottle Detect V2 Node Initialized")

    # -------------------- 模型 --------------------
    def infer_kwargs(self):
        """每次推論共用的參數。imgsz 一定要明確傳，理由見 bottle_detect.py 的 backend_imgsz()。"""
        kwargs = {'conf': CONF_THRES, 'iou': IOU_THRES,
                  'agnostic_nms': AGNOSTIC_NMS, 'verbose': False}
        if self.imgsz:
            kwargs['imgsz'] = self.imgsz
        return kwargs

    def load_engine(self, engine_path):
        """載入模型並餵一張黑圖暖機，第一張真的影像才不會卡住。"""
        if not os.path.exists(engine_path):
            self.get_logger().error(
                f"模型檔不存在: {engine_path}\n"
                f"  把模型放到這個路徑，或用 "
                f"--ros-args -p engine_path:=/abs/path/model.onnx 指定")
            raise FileNotFoundError(engine_path)

        try:
            from ultralytics import YOLO
        except ImportError as e:
            self.get_logger().error(f"ultralytics 沒裝: {e}")
            raise

        self.get_logger().info("載入模型中…")
        model = YOLO(engine_path, task='detect')
        dummy = np.zeros((IMAGE_H, IMAGE_W, 3), dtype=np.uint8)
        t0 = time.time()
        model(dummy, **self.infer_kwargs())
        self.get_logger().info(f"模型暖機完成，耗時 {time.time() - t0:.2f} s")
        return model

    def backend_imgsz(self, model):
        """檔案問不出輸入邊長時，回頭問載好的 backend。"""
        predictor = getattr(model, 'predictor', None)
        for holder in (getattr(predictor, 'model', None), getattr(predictor, 'args', None)):
            size = normalize_imgsz(getattr(holder, 'imgsz', None))
            if size:
                self.imgsz_src = 'backend metadata'
                return size

        self.imgsz_src = f'問不出來，退回預設 {DEFAULT_IMGSZ}'
        self.get_logger().warn(
            f"問不出模型的輸入邊長，退回 {DEFAULT_IMGSZ}。"
            f"對不上的話把 IMGSZ 填成錯誤訊息裡的 Expected / max model size\033[K")
        return DEFAULT_IMGSZ

    # -------------------- 訂閱回呼 --------------------
    def zoom_callback(self, msg):
        """數位變焦倍率。/depth_mm 有跟著變焦，座標要對得上就得跟著換算。"""
        try:
            self.zoom = float(msg.zoomin)
        except Exception as e:
            self.get_logger().warn(f"[zoom] bad value: {e}")

    def image_callback(self, msg: RosImage):
        """收到影像：交棒給推論執行緒，然後馬上把畫面發出去。"""
        now = time.time()
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"[image] convert failed: {e}")
            return

        #只留最新的一幀給推論
        with self._pending_lock:
            self._pending = (frame, msg.header.stamp)
        self._wake.set()

        if self.view_pub is not None:
            self.publish_view(frame, self._last_bottles, msg.header, now)

    def infer_loop(self):
        """推論執行緒：永遠只推論手上最新的那一幀。"""
        while not self._stop.is_set():
            if not self._wake.wait(0.2):
                continue
            self._wake.clear()

            with self._pending_lock:
                item, self._pending = self._pending, None
            if item is None:
                continue

            now = time.time()
            if MIN_INTERVAL > 0.0 and now - self._last_infer < MIN_INTERVAL:
                continue

            frame, stamp = item
            self._last_infer = now
            self.tick_fps(now)

            bottles = self.detect(frame)
            self._last_bottles = bottles
            self.report(bottles)
            self.publish_result(bottles, stamp)

    def destroy_node(self):
        """收掉推論執行緒再拆節點。"""
        self._stop.set()
        self._wake.set()
        worker = getattr(self, '_worker', None)
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
        return super().destroy_node()

    # -------------------- 主流程 --------------------
    def detect(self, frame):
        """推論一張影像，回傳排序好、換算好座標的藥罐清單。

        Returns:
            list: 每個元素是 dict，含 name / label / confidence / corners / bbox / index。
                  label 是種類（b1~b4）。已依種類排序，每一種最多一個。
        """
        src_h, src_w = frame.shape[:2]

        try:
            results = self.model(frame, **self.infer_kwargs())
        except Exception as e:
            self.get_logger().error(f"[infer] failed: {e}")
            return []

        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return []

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        clses = boxes.cls.cpu().numpy().astype(int)

        best = {}          #類別編號 -> 該類信心值最高的框
        for (x1, y1, x2, y2), conf, cls in zip(xyxy, confs, clses):
            u1, v1 = map_to_strategy(x1, y1, src_w, src_h, self.zoom)
            u2, v2 = map_to_strategy(x2, y2, src_w, src_h, self.zoom)

            #變焦裁掉的區域在 /depth_mm 上不存在，中心落在畫面外就不要了
            cu, cv = (u1 + u2) / 2.0, (v1 + v2) / 2.0
            if not (0 <= cu < IMAGE_W and 0 <= cv < IMAGE_H):
                continue

            xmin = int(round(max(0, min(u1, u2))))
            ymin = int(round(max(0, min(v1, v2))))
            xmax = int(round(min(IMAGE_W - 1, max(u1, u2))))
            ymax = int(round(min(IMAGE_H - 1, max(v1, v2))))
            if xmax <= xmin or ymax <= ymin:
                continue

            if REJECT_EDGE_BBOX and self.touches_edge(xmin, ymin, xmax, ymax):
                continue

            cls = int(cls)
            if not (0 <= cls < MAX_BOTTLES):
                continue

            #同一種出現兩個框時只留信心值高的（四種藥罐各只有一罐）
            prev = best.get(cls)
            if prev is not None:
                self.get_logger().warn(
                    f"{prev['label']} 偵測到不只一個，只留信心值最高的\033[K",
                    throttle_duration_sec=2.0)
                if prev['confidence'] >= conf:
                    continue
            best[cls] = {
                'label': str(self.names.get(cls, cls)),
                'confidence': float(conf),
                'bbox': (xmin, ymin, xmax, ymax),
            }

        bottles = []
        for cls in sorted(best):
            b = best[cls]
            xmin, ymin, xmax, ymax = b['bbox']
            b['name'] = f'bottle{cls + 1}'
            #左上、右上、左下、右下
            b['corners'] = [(xmin, ymin), (xmax, ymin), (xmin, ymax), (xmax, ymax)]
            #這瓶的第一個座標編號，依種類固定:b1 從 1 起，b2 從 5 起，b3 從 9 起。
            b['index'] = cls * 4 + 1
            bottles.append(b)
        return bottles

    def touches_edge(self, xmin, ymin, xmax, ymax):
        """框碰到畫面邊緣代表藥罐被切掉了，框不完整。"""
        m = EDGE_MARGIN_PX
        return (xmin <= m or ymin <= m or
                xmax >= IMAGE_W - 1 - m or ymax >= IMAGE_H - 1 - m)

    # -------------------- 畫面 --------------------
    def tick_fps(self, now):
        """推論張數的指數平滑，畫在畫面上當健康指標。"""
        if self._infer_t > 0.0:
            dt = now - self._infer_t
            if dt > 1e-6:
                inst = 1.0 / dt
                self._infer_fps = inst if self._infer_fps <= 0.0 else \
                    0.8 * self._infer_fps + 0.2 * inst
        self._infer_t = now

    def to_strategy_frame(self, frame):
        """把相機原生影像換到 960x600 的策略座標系（map_to_strategy() 的影像版）。

        ⚠ 恆等變換時回的是傳進來的陣列本身，呼叫端不可以直接畫上去。
        """
        h, w = frame.shape[:2]
        if w != IMAGE_W:
            h = max(1, int(round(h * IMAGE_W / float(w))))
            frame = cv2.resize(frame, (IMAGE_W, h), interpolation=cv2.INTER_LINEAR)
        if h < IMAGE_H:
            top = (IMAGE_H - h) // 2
            frame = cv2.copyMakeBorder(frame, top, IMAGE_H - h - top, 0, 0,
                                       cv2.BORDER_CONSTANT, value=(0, 0, 0))
        elif h > IMAGE_H:
            top = (h - IMAGE_H) // 2
            frame = frame[top:top + IMAGE_H]

        zoom = self.zoom
        if zoom and zoom > 1.0:
            nw, nh = int(IMAGE_W / zoom), int(IMAGE_H / zoom)
            x0, y0 = (IMAGE_W - nw) // 2, (IMAGE_H - nh) // 2
            frame = cv2.resize(frame[y0:y0 + nh, x0:x0 + nw], (IMAGE_W, IMAGE_H),
                               interpolation=cv2.INTER_LINEAR)
        return frame

    @staticmethod
    def put_text(img, text, org, color, scale=0.5):
        """描黑邊寫字，整串字推回畫面內。"""
        h, w = img.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(text, font, scale, 1)
        x = int(min(max(org[0], 2), max(2, w - tw - 2)))
        y = int(min(max(org[1], th + 2), h - 2))
        cv2.putText(img, text, (x, y), font, scale, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, text, (x, y), font, scale, color, 1, cv2.LINE_AA)

    def annotate(self, frame, bottles, scale=1.0):
        """把偵測結果畫上去。顏色依種類（b1~b4），標籤是「種類 信心值」。

        Args:
            frame (np.ndarray): 底圖，可能已經縮小過。
            bottles (list): 偵測結果，座標一律是 960x600 座標系的。
            scale (float): 底圖相對 960x600 的縮放比例。

        Returns:
            np.ndarray: 畫好的新影像，不會動到傳進來的底圖。
        """
        def s(v):
            return int(round(v * scale))

        view = frame.copy()
        for b in bottles:
            color = CLASS_COLORS.get(b['label'], OTHER_COLOR)
            xmin, ymin, xmax, ymax = b['bbox']
            cv2.rectangle(view, (s(xmin), s(ymin)), (s(xmax), s(ymax)), color, 2)

            if DRAW_LABEL:
                #框上緣外側；太靠近頂端時改寫到下緣外側
                label_y = s(ymin) - 8
                if label_y < STATUS_ROW_PX:
                    label_y = s(ymax) + 16
                self.put_text(view, f"{b['label']} {b['confidence']:.2f}",
                              (s(xmin), label_y), color)

            if DRAW_CORNERS:
                n = b['index']
                for k, (x, y) in enumerate(b['corners']):
                    cv2.circle(view, (s(x), s(y)), 4, color, -1)
                    dx = 7 if k in (0, 2) else -32
                    dy = 16 if k in (0, 1) else -7
                    self.put_text(view, f"x{n + k}", (s(x) + dx, s(y) + dy),
                                  color, scale=0.4)

        head = f"bottles {len(bottles)}  infer {self._infer_fps:.1f} fps"
        if self.zoom and abs(self.zoom - 1.0) > 1e-3:
            head += f"  zoom {self.zoom:.2f}"
        self.put_text(view, head, (8, 22), (255, 255, 255), scale=0.6)
        return view

    def publish_view(self, frame, bottles, header, now):
        """把畫好框的影像發出去給 web_video_server。沒偵測到東西時照樣發。"""
        if DISPLAY_MAX_FPS > 0.0 and (now - self._last_view) < 1.0 / DISPLAY_MAX_FPS:
            return
        self._last_view = now

        try:
            view = self.to_strategy_frame(frame)
            #先縮再畫，字才不會跟著糊掉
            scale = 1.0
            if DISPLAY_WIDTH and DISPLAY_WIDTH != IMAGE_W:
                scale = DISPLAY_WIDTH / float(IMAGE_W)
                out_h = max(1, int(round(IMAGE_H * scale)))
                view = cv2.resize(view, (DISPLAY_WIDTH, out_h),
                                  interpolation=cv2.INTER_AREA)
            view = self.annotate(view, bottles, scale)
            msg = self._bridge.cv2_to_imgmsg(view, encoding='bgr8')
            msg.header = header
            self.view_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"[view] publish failed: {e}",
                                   throttle_duration_sec=5.0)

    # -------------------- 輸出 --------------------
    def report(self, bottles):
        """把結果印到終端機。"""
        if not bottles:
            if LOG_EMPTY:
                self.get_logger().info("沒有偵測到藥罐\033[K")
            return

        self.get_logger().info('________________________________________\033[K')
        for b in bottles:
            self.get_logger().info(
                f"{b['label']}  conf {b['confidence']:.2f}\033[K")
            n = b['index']
            for k, (x, y) in enumerate(b['corners']):
                self.get_logger().info(
                    f"    {CORNER_ORDER[k]}  x{n + k}={x}  y{n + k}={y}\033[K")

    def publish_result(self, bottles, stamp):
        """發布偵測結果，格式與 bottle_detect.py 相同::

            {"stamp": {"sec": 0, "nanosec": 0},
             "image_size": [960, 600],
             "count": 2,
             "bottles": [{"name": "bottle1",
                          "label": "b1",
                          "confidence": 0.93,
                          "bbox": [xmin, ymin, xmax, ymax],
                          "points": {"x1": .., "y1": .., ... "x4": .., "y4": ..}}],
             "coords": {"x1": .., "y1": .., ... "x8": .., "y8": ..}}
        """
        coords = {}
        out = []
        for b in bottles:
            n = b['index']
            points = {}
            for k, (x, y) in enumerate(b['corners']):
                points[f'x{n + k}'] = x
                points[f'y{n + k}'] = y
            coords.update(points)
            out.append({
                'name': b['name'],
                'label': b['label'],
                'confidence': round(b['confidence'], 4),
                'bbox': list(b['bbox']),
                'points': points,
            })

        payload = {
            'stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
            'image_size': [IMAGE_W, IMAGE_H],
            'count': len(out),
            'bottles': out,
            'coords': coords,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.result_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BottleDetectV2()
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
