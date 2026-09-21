#!/usr/bin/env python3
#coding=utf-8
"""藥罐偵測節點。

流程:
    1. 收 ZED 的彩色影像
    2. 丟進 TensorRT engine（best.engine，用 ultralytics 載）做偵測
    3. 把偵測到的框從相機原生解析度換算到策略層的 960x600 座標系
    4. 由左到右排序，最左邊的是 bottle1，其次 bottle2，依此類推
    5. 每個框攤成四個角點（左上、右上、左下、右下），座標連號往下編
    6. print 出名稱、信心值、座標，並發到 RESULT_TOPIC

座標編號規則（畫面中最多 MAX_BOTTLES 瓶）::

    bottle1: x1,y1 左上   x2,y2 右上   x3,y3 左下   x4,y4 右下
    bottle2: x5,y5 左上   x6,y6 右上   x7,y7 左下   x8,y8 右下
    bottle3: x9,y9  ...                             x12,y12
    bottle4: x13,y13 ...                            x16,y16

畫面：本節點把畫好框的影像發到 IMAGE_OUT_TOPIC（預設 /med_detect/image），
瀏覽器開 http://<機器人IP>:8080/stream?topic=/med_detect/image 就看得到
（web_video_server 由 camera.launch.py 一起帶起來；只跑這個節點時改用本包的
launch/bottle_detect.launch.py，它會自己開一個在 8081）。
不開 cv2 視窗 —— 機器人上通常沒有桌面，開了只會 crash。
"""
import ast
import json
import os
import socket
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


BASE_DIR = os.path.dirname(os.path.abspath(__file__))

#--- Topic ---#
#ZED 彩色影像。ZED 原生就是 960x600，與 /depth_mm 同一個座標系。
#換相機的話改這裡（usb 是 /camera1/image_raw，960x540，下面會自動補黑對齊）。
IMAGE_TOPIC                = '/zed/zed_node/left/image_rect_color'
#數位變焦倍率，image.py 與 depth_process_node 都吃同一個 topic
ZOOM_TOPIC                 = '/Zoom_In_Topic'
#本節點輸出
RESULT_TOPIC               = '/med_detect/bottles'

#--- 模型 ---#
#TensorRT engine。檔案還沒放進來，放好之後路徑要對得上；
#也可以不改程式，用 ros2 run ... --ros-args -p engine_path:=/abs/path/best.engine 覆蓋。
ENGINE_PATH                = os.path.join(BASE_DIR, 'Parameter', 'best.engine')
#模型匯出時綁定的輸入邊長。None = 開機時自動從模型檔讀（建議，見 load_model_metadata()）。
#⚠ 不要憑印象填這個數字。現在這顆模型是 dynamic=False 匯出的，輸入被焊死成
#  [1, 3, 960, 960]，填別的值會被擋下來，而且兩種格式的死法還不一樣:
#      .onnx   每一幀丟 INVALID_ARGUMENT: Invalid dimensions for input: images
#      .engine 開機暖機就 AssertionError: input size ... not equal to max model size
#  要換邊長請重新匯出模型，不是改這裡。
IMGSZ                      = None
#連模型都問不出來時的退路，也是 ultralytics 的預設值
DEFAULT_IMGSZ              = 640
#信心值門檻，低於這個值的框直接丟掉
CONF_THRES                 = 0.50
#NMS 的 IoU 門檻
IOU_THRES                  = 0.45
#畫面中最多幾瓶藥罐。多於這個數量時只留信心值最高的前幾瓶。
MAX_BOTTLES                = 4

#--- 影像尺寸 ---#
#策略層統一的座標系，/depth_mm 與 med_detect.py 的 bbox 都在這個座標系上
IMAGE_W                    = 960
IMAGE_H                    = 600

#--- 偵測節流 ---#
#兩次推論的最小間隔(秒)。0 代表有新影像就推論。
#推論跑在自己的執行緒上而且一次只跑一張（見 infer_loop()），所以設 0 也不會讓
#callback 互相排隊、也不會拖慢畫面。要把 CPU 讓給別的節點時才調大。
MIN_INTERVAL               = 0.0

#--- 輸出 ---#
#沒偵測到任何藥罐時也印一行。除錯時打開，平常關著免得洗版。
LOG_EMPTY                  = False
#框碰到畫面邊緣代表藥罐被切掉了，框不完整。這裡只警告不丟掉，
#要跟 med_detect.py 一樣直接跳過的話把這個改成 True。
REJECT_EDGE_BBOX           = False
#距離邊緣幾個像素內算「碰到邊緣」
EDGE_MARGIN_PX             = 2

#--- 畫面輸出 ---#
#把畫好框的影像發成 sensor_msgs/Image 給人看。關掉可省下複製、編碼與傳輸的成本，
#偵測與 RESULT_TOPIC 不受影響。
PUBLISH_IMAGE              = True
#畫面的 topic。用瀏覽器看:
#    http://<機器人IP>:8080/stream?topic=/med_detect/image
#8080 是 camera.launch.py 帶起來的 web_video_server。只跑這個節點時自己開一個:
#    ros2 run web_video_server web_video_server
IMAGE_OUT_TOPIC            = '/med_detect/image'
#發布用的寬度(像素)，高度等比。畫面只給人看，縮小可省下 cv2_to_imgmsg、DDS 傳輸
#與 web_video_server 的 JPEG 編碼（overlap_node 實測全解析度貴 7.5 倍）——
#這顆機器的推論在 CPU 上跑，省下來的都是推論搶得到的。
#框是**縮完才畫**的，所以縮小不會讓字跟著糊掉，只有框的位置會有半像素級的誤差。
#要「畫面上的像素座標 = 發出去的座標」（對座標時方便）就改成 960。
DISPLAY_WIDTH              = 480
#畫面最高張數(每秒)。相機是 30，設 30 就是相機有多快跟多快。
#推論慢不會讓畫面跟著慢 —— 慢的只是框更新的頻率。調低這個值可以把 CPU 還給推論。
DISPLAY_MAX_FPS            = 30.0
#每瓶一個顏色(BGR)，順序同 bottle1..bottleN，超過就循環
BOTTLE_COLORS              = ((0, 255, 0), (0, 200, 255), (255, 160, 0), (255, 0, 255))
#左上角狀態列佔掉的高度(像素)。標籤會避開這一條，不跟它疊在一起。
STATUS_ROW_PX              = 34
#角點標記：框四個角上的圓點 + 座標編號(x1/x2/...)，畫在框的**內側**。
#要核對編號規則時打開，平常關著畫面比較乾淨。
DRAW_CORNERS               = False
#框上方那行 bottle1 0.93（名稱 + 信心值）。關掉就只剩下框本身。
#不印類別名稱:模型只有一個類別，每個框都寫一次 pill-bottle 只是佔位置。
#類別名稱仍然照常印在終端機、也照常發在 RESULT_TOPIC 的 label 欄位。
DRAW_LABEL                 = True

#角點的順序，跟輸出的 x1..x4 一一對應
CORNER_ORDER               = ('左上', '右上', '左下', '右下')


def map_to_strategy(x, y, src_w, src_h, zoom):
    """把相機原生座標換算到 960x600 的策略座標系。

    兩段換算，順序與 image.py 一致：

    1. 補黑對齊：等比縮到寬度 960，高度置中放進 600。ZED 原生就是 960x600，
       這一段是恆等變換；usb 的 960x540 會在上下各補 30 列。
    2. 數位變焦：image.py 是「中心裁成 1/zoom 再放大回原尺寸」，
       所以座標要先減掉裁切原點再乘上倍率。zoom=1 時同樣是恆等變換。

    Args:
        x, y (float): 相機原生解析度下的像素座標。
        src_w, src_h (int): 來源影像的寬高。
        zoom (float): 數位變焦倍率。

    Returns:
        tuple: (u, v)，960x600 座標系下的像素座標，尚未夾回畫面內。
    """
    s = IMAGE_W / float(src_w)
    pad_top = (IMAGE_H - src_h * s) / 2.0
    u = x * s
    v = y * s + pad_top

    if zoom and zoom > 1.0:
        x0 = (IMAGE_W - IMAGE_W / zoom) / 2.0
        y0 = (IMAGE_H - IMAGE_H / zoom) / 2.0
        u = (u - x0) * zoom
        v = (v - y0) * zoom

    return u, v


def _literal(value):
    """metadata 的值在 .onnx 裡是字串、在 .engine 裡是真的型別，統一轉回型別。"""
    if not isinstance(value, str):
        return value
    try:
        return ast.literal_eval(value)
    except Exception:
        return value


def normalize_imgsz(raw):
    """把各種寫法的 imgsz 收斂成 int（正方形）或 [高, 寬]。讀不出來回 None。"""
    raw = _literal(raw)
    if isinstance(raw, (list, tuple)) and len(raw) == 2:
        try:
            h, w = int(raw[0]), int(raw[1])
        except (TypeError, ValueError):
            return None
        #動態維度在 onnx 裡是 0、在 engine 裡是 -1，兩種都不是真的尺寸
        if h > 0 and w > 0:
            return h if h == w else [h, w]
        return None
    if isinstance(raw, int) and not isinstance(raw, bool) and raw > 0:
        return raw
    return None


def load_model_metadata(path):
    """在**載入模型之前**先讀出模型檔自己帶的 metadata。

    為什麼要提早讀：ultralytics 的暖機會拿 args.imgsz 去配輸入緩衝區，對不上時
    是直接掛在暖機那一步，連第一張影像都收不到:
        AssertionError: input size torch.Size([1, 3, 640, 640])
                        not equal to max model size (1, 3, 960, 960)
    模型自己知道答案，只是 ultralytics 不一定讀得到（見下面第 3 點）。

    三個來源，由可靠到勉強:
      1. .engine 開頭的 ultralytics metadata。格式是「4 bytes 長度(小端) + JSON +
         engine 本體」，只有 `yolo export ... format=engine` 產的才有。
      2. .onnx 的 graph 輸入維度。這是 onnxruntime 真正會拿去檢查的東西，
         比 metadata 裡的 imgsz 字串可靠（後者只是匯出時抄進去的）。
      3. .engine 旁邊同名的 .onnx。trtexec / TensorRT API 直接從 onnx 編出來的
         engine **沒有**第 1 點那段 metadata（開頭是 TensorRT 自己的 'ftrt'），
         但它就是拿旁邊那個 onnx 編的，輸入尺寸與類別名稱一定一樣。
         沒有這一步，trtexec 轉出來的 engine 就只能靠手動填 IMGSZ。

    Returns:
        tuple: (meta, source)。meta 是 dict（可能含 'imgsz' / 'names'），
            source 是一句話，寫進 log 讓人知道數字是哪來的。讀不到回 ({}, '')。
    """
    if path.lower().endswith('.engine'):
        meta = _meta_from_engine_header(path)
        if meta:
            return meta, 'engine 內建 metadata'
        sibling = path[:-len('.engine')] + '.onnx'
        if os.path.exists(sibling):
            meta = _meta_from_onnx(sibling)
            if meta:
                return meta, f'{os.path.basename(sibling)}（engine 是拿它編的）'
        return {}, ''

    if path.lower().endswith('.onnx'):
        meta = _meta_from_onnx(path)
        if meta:
            return meta, 'onnx 檔'
    return {}, ''


def _meta_from_engine_header(path):
    """讀 .engine 開頭的 ultralytics metadata；trtexec 產的沒有，回 None。"""
    try:
        with open(path, 'rb') as f:
            #長度不合理就代表這不是 ultralytics 的檔頭，別拿 8 MB 去 decode
            n = int.from_bytes(f.read(4), byteorder='little')
            if not (0 < n < 65536):
                return None
            return json.loads(f.read(n).decode('utf-8'))
    except Exception:
        return None


def _meta_from_onnx(path):
    """讀 .onnx 的 metadata_props，並用 graph 的輸入維度覆蓋 imgsz。"""
    try:
        import onnx
    except ImportError:
        return None
    try:
        model = onnx.load(path, load_external_data=False)
    except Exception:
        return None

    meta = {p.key: p.value for p in model.metadata_props}
    try:
        dims = [d.dim_value for d in model.graph.input[0].type.tensor_type.shape.dim]
        #dim_value 是 0 表示那一維是動態的，動態就不必也不該鎖尺寸
        if len(dims) == 4 and dims[2] > 0 and dims[3] > 0:
            meta['imgsz'] = [dims[2], dims[3]]
    except Exception:
        pass
    return meta


def local_ip():
    """找一張對外網卡的 IP，純粹為了在 log 裡印出可以直接點的網址。

    connect 到一個外部位址只是讓作業系統挑網卡，UDP 不會真的送出封包，
    所以沒網路也不會卡住。查不到就回一個佔位字串。
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('8.8.8.8', 1))
            return s.getsockname()[0]
        finally:
            s.close()
    except Exception:
        return '<機器人IP>'


class BottleDetect(Node):
#國科會計畫 - 藥罐偵測
    def __init__(self):
        super().__init__('bottle_detect_node')

        self.image_cbg = ReentrantCallbackGroup()
        self.qos_fast = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        #空字串當作「沒有指定」，落回上面的常數 —— 沒有這一段，
        #帶了空的 -p engine_path:= 會變成去開一個叫 '' 的檔案。
        self.declare_parameter('engine_path', ENGINE_PATH)
        self.declare_parameter('image_topic', IMAGE_TOPIC)
        engine_path = str(self.get_parameter('engine_path').value).strip() or ENGINE_PATH
        image_topic = str(self.get_parameter('image_topic').value).strip() or IMAGE_TOPIC

        #先問檔案、再載模型 —— 暖機就要用到輸入邊長，來不及等模型載完才問
        meta, meta_src = load_model_metadata(engine_path)
        self.imgsz = IMGSZ or normalize_imgsz(meta.get('imgsz'))
        self.imgsz_src = 'IMGSZ 指定' if IMGSZ else meta_src

        self.model = self.load_engine(engine_path)
        if not self.imgsz:
            #檔案問不出來，退回問載好的 backend（.pt 與 yolo export 的檔走這條）
            self.imgsz = self.backend_imgsz(self.model)

        #類別名稱優先用模型檔裡的:trtexec 產的 engine 沒有這一段，ultralytics 會
        #自己填 class0/class1…，畫面上就看不出框到的是什麼東西了。
        self.names = _literal(meta.get('names')) or getattr(self.model, 'names', {}) or {}

        self._bridge = CvBridge()
        self.zoom = 1.0
        self._last_infer = 0.0

        self.result_pub = self.create_publisher(String, RESULT_TOPIC, 10)

        #畫面用 RELIABLE，不能沿用上面那組 BEST_EFFORT 的 qos_fast:
        #web_video_server 是用預設 QoS（RELIABLE）訂閱的，發布端若是 BEST_EFFORT
        #兩邊就對不上，網頁一片空白而且**兩邊都不會報錯**。image.py 的
        #zoom_in / processed_image 也是這樣發的，跟著它走準沒錯。
        self.view_pub = None
        if PUBLISH_IMAGE:
            self.view_pub = self.create_publisher(
                RosImage, IMAGE_OUT_TOPIC,
                QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE))
        #畫面疊的永遠是「最近一次推論的結果」，不等這一幀自己的
        self._last_bottles = []
        self._last_view = 0.0
        self._infer_fps = 0.0
        self._infer_t = 0.0

        #推論搬到自己的執行緒。理由見 infer_loop()。
        self._pending = None
        self._pending_lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._worker = threading.Thread(
            target=self.infer_loop, name='bottle_infer', daemon=True)
        self._worker.start()

        self.create_subscription(
            RosImage, image_topic, self.image_callback,
            self.qos_fast, callback_group=self.image_cbg)
        self.create_subscription(
            Zoom, ZOOM_TOPIC, self.zoom_callback,
            self.qos_fast, callback_group=self.image_cbg)

        self.get_logger().info(f"engine: {engine_path}")
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
        self.get_logger().info("Bottle Detect Node Initialized")

    # -------------------- 模型 --------------------
    def load_engine(self, engine_path):
        """載入 TensorRT engine 並暖機。

        engine 的第一次推論要配置一堆 GPU 記憶體，會比之後慢一個數量級。
        開機時先餵一張黑圖把這個成本吃掉，真正的第一張影像才不會卡住。
        """
        if not os.path.exists(engine_path):
            self.get_logger().error(
                f"engine 檔不存在: {engine_path}\n"
                f"  把 best.engine 放到這個路徑，或用 "
                f"--ros-args -p engine_path:=/abs/path/best.engine 指定")
            raise FileNotFoundError(engine_path)

        try:
            from ultralytics import YOLO
        except ImportError as e:
            self.get_logger().error(f"ultralytics 沒裝: {e}")
            raise

        self.get_logger().info("載入 engine 中…")
        model = YOLO(engine_path, task='detect')
        dummy = np.zeros((IMAGE_H, IMAGE_W, 3), dtype=np.uint8)
        t0 = time.time()
        #已經從檔案問出邊長就直接指定；問不出來才不傳，讓 ultralytics 用它自己
        #從 metadata 讀到的值（.pt 與 yolo export 產的檔都讀得到）。
        kwargs = {'conf': CONF_THRES, 'iou': IOU_THRES, 'verbose': False}
        if self.imgsz:
            kwargs['imgsz'] = self.imgsz
        model(dummy, **kwargs)
        self.get_logger().info(f"engine 暖機完成，耗時 {time.time() - t0:.2f} s")
        return model

    def backend_imgsz(self, model):
        """模型載進來之後，回頭問 backend 它要的輸入邊長。

        這是 load_model_metadata() 問不出來時的退路。模型載好之後 ultralytics 會把
        自己讀到的 metadata 掛在 backend 上，這裡就是去拿那一份。

        為什麼拿到之後每次推論都要明確傳進去、而不是靠 ultralytics 自己記得:
            它只有在**建 backend 的那一次**（第一次推論）才會把 metadata 的 imgsz
            蓋回 args.imgsz。之後每次呼叫若自己傳了別的 imgsz=，就等於把它改回來，
            於是變成「暖機過得去、每一幀都失敗」——
                [infer] failed: ... Got: 640 Expected: 960
            錯誤訊息指著模型，實際上壞的是呼叫端的參數，是最難查的那一種。

        Returns:
            int | list: 邊長。問不出來時回 DEFAULT_IMGSZ。
        """
        predictor = getattr(model, 'predictor', None)
        #backend 的 imgsz 直接來自 metadata，比 args 可靠（args 會被呼叫端蓋掉）
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
        """收到影像：交棒給推論執行緒，然後**馬上**把畫面發出去。

        這個 callback 裡不做推論。推論要 300 ms 的話，在這裡做就等於畫面也只有
        3 fps —— 卡的不是相機也不是模型，是「畫面排在推論後面」這件事本身。
        """
        now = time.time()
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"[image] convert failed: {e}")
            return

        #只留最新的一幀給推論。積壓的舊幀推論完也沒用，框只會越貼越後面。
        with self._pending_lock:
            self._pending = (frame, msg.header.stamp)
        self._wake.set()

        if self.view_pub is not None:
            self.publish_view(frame, self._last_bottles, msg.header, now)

    def infer_loop(self):
        """推論執行緒：永遠只推論「手上最新的那一幀」。

        為什麼不留在 callback 裡:
            1. 畫面會被推論擋住，相機 30 fps 也只看得到推論的那幾 fps。
            2. callback group 是 ReentrantCallbackGroup + 多執行緒 executor，
               推論比相機慢的時候會有好幾幀同時在推論，一起搶同一顆 CPU，
               每一張都變更慢 —— 越忙越慢的那種惡性循環。
            這裡一次只跑一張，慢下來的表現是「框更新得慢」，不是「畫面卡住」。

        onnxruntime 與 torch 在算的時候會放掉 GIL，所以這條執行緒不會擋住
        executor 發畫面。
        """
        while not self._stop.is_set():
            #逾時是為了讓 _stop 有機會被看到，沒有新影像時就在這裡等著
            if not self._wake.wait(0.2):
                continue
            self._wake.clear()

            with self._pending_lock:
                item, self._pending = self._pending, None
            if item is None:
                continue

            now = time.time()
            if MIN_INTERVAL > 0.0 and now - self._last_infer < MIN_INTERVAL:
                continue          #這一幀丟掉，相機 30 fps，下一幀馬上就來

            frame, stamp = item
            self._last_infer = now
            self.tick_fps(now)

            bottles = self.detect(frame)      #內部已經把推論的例外接起來了
            self._last_bottles = bottles
            self.report(bottles)
            self.publish_result(bottles, stamp)

    def destroy_node(self):
        """收掉推論執行緒再拆節點，不然關的時候會卡在那條執行緒上。"""
        self._stop.set()
        self._wake.set()
        worker = getattr(self, '_worker', None)
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
        return super().destroy_node()

    # -------------------- 主流程 --------------------
    def detect(self, frame):
        """推論一張影像，回傳排序好、換算好座標的藥罐清單。

        推論直接餵相機原生影像，讓 ultralytics 自己做 letterbox，
        座標是拿到結果之後才換算到 960x600 —— 先縮圖再推論會多一次重取樣，
        小物件的細節白白損失掉。

        Returns:
            list: 每個元素是 dict，含 name / label / confidence / corners / bbox。
                  已由左到右排序，最多 MAX_BOTTLES 個。
        """
        src_h, src_w = frame.shape[:2]

        try:
            results = self.model(frame, imgsz=self.imgsz, conf=CONF_THRES,
                                 iou=IOU_THRES, verbose=False)
        except Exception as e:
            self.get_logger().error(f"[infer] failed: {e}")
            return []

        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return []

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        clses = boxes.cls.cpu().numpy().astype(int)

        cand = []
        for (x1, y1, x2, y2), conf, cls in zip(xyxy, confs, clses):
            #兩個角點各自換算，中間的變換都是等比縮放，換完仍是軸對齊的矩形
            u1, v1 = map_to_strategy(x1, y1, src_w, src_h, self.zoom)
            u2, v2 = map_to_strategy(x2, y2, src_w, src_h, self.zoom)

            #變焦裁掉的區域在 /depth_mm 上根本不存在，中心落在畫面外就不要了
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

            cand.append({
                'label': str(self.names.get(int(cls), int(cls))),
                'confidence': float(conf),
                'bbox': (xmin, ymin, xmax, ymax),
            })

        #超過上限時先砍信心值低的，再排回左到右，編號才不會因為信心值跳動而亂跑
        if len(cand) > MAX_BOTTLES:
            self.get_logger().warn(
                f"偵測到 {len(cand)} 瓶，超過上限 {MAX_BOTTLES}，"
                f"只留信心值最高的 {MAX_BOTTLES} 瓶\033[K")
            cand.sort(key=lambda b: b['confidence'], reverse=True)
            cand = cand[:MAX_BOTTLES]

        #「最左邊」以框的左緣為準。藥罐互相重疊而左緣順序不直覺時，
        #把 b['bbox'][0] 換成 (b['bbox'][0] + b['bbox'][2]) / 2 改用中心排序。
        cand.sort(key=lambda b: b['bbox'][0])

        bottles = []
        for i, b in enumerate(cand):
            xmin, ymin, xmax, ymax = b['bbox']
            b['name'] = f'bottle{i + 1}'
            #左上、右上、左下、右下
            b['corners'] = [(xmin, ymin), (xmax, ymin), (xmin, ymax), (xmax, ymax)]
            #這瓶的第一個座標編號。bottle1 從 1 起，bottle2 從 5 起。
            b['index'] = i * 4 + 1
            bottles.append(b)
        return bottles

    def touches_edge(self, xmin, ymin, xmax, ymax):
        """框碰到畫面邊緣代表藥罐被切掉了，框不完整。"""
        m = EDGE_MARGIN_PX
        return (xmin <= m or ymin <= m or
                xmax >= IMAGE_W - 1 - m or ymax >= IMAGE_H - 1 - m)

    # -------------------- 畫面 --------------------
    def tick_fps(self, now):
        """推論張數的指數平滑，畫在畫面上當健康指標。

        看的是**推論**的張數而不是相機的張數：畫面在節流時照樣更新，
        只看畫面是連續的看不出推論其實已經掉到 2 fps。
        """
        if self._infer_t > 0.0:
            dt = now - self._infer_t
            if dt > 1e-6:
                inst = 1.0 / dt
                self._infer_fps = inst if self._infer_fps <= 0.0 else \
                    0.8 * self._infer_fps + 0.2 * inst
        self._infer_t = now

    def to_strategy_frame(self, frame):
        """把相機原生影像換到 960x600 的策略座標系。

        這是 map_to_strategy() 的影像版，兩邊是同一組變換，要改一起改:
        先等比縮到寬度 960、高度置中補黑，再套數位變焦的「中心裁切 + 放大」。
        底圖與座標走同一組變換，畫面上的框才會是下游真正收到的那個框 ——
        對不上的時候一眼就看得出是偵測錯了還是座標換算錯了。

        ZED 原生就是 960x600 且 zoom=1 時整段是恆等變換，只花一次 shape 比對。
        ⚠ 這種情況下回的是傳進來的那個陣列本身，呼叫端不可以直接畫上去。
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
        """先描一圈黑邊再寫字。

        畫面亮的地方純色的字會糊進背景裡，描邊之後亮底暗底都讀得到。
        整串字會被推回畫面內（量過字寬，不是只夾起點），貼著右緣的框
        才不會被切掉半截 —— 縮圖之後字相對變寬，這件事更容易發生。
        """
        h, w = img.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(text, font, scale, 1)
        x = int(min(max(org[0], 2), max(2, w - tw - 2)))
        y = int(min(max(org[1], th + 2), h - 2))
        cv2.putText(img, text, (x, y), font, scale, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, text, (x, y), font, scale, color, 1, cv2.LINE_AA)

    def annotate(self, frame, bottles, scale=1.0):
        """把偵測結果畫上去。

        畫的東西:
            外框            每瓶一個顏色，一定會畫
            標籤            框上方一行，名稱 + 信心值（DRAW_LABEL）
            角點標記        圓點 + 座標編號 x1/x2/...，畫在框內側（DRAW_CORNERS）
                            —— 編號規則對不對看畫面就知道，不用去比對 JSON
            左上角一行字    瓶數、推論張數、變焦倍率，用來確認節點還活著

        Args:
            frame (np.ndarray): 底圖，可能已經縮小過。
            bottles (list): 偵測結果，座標一律是 960x600 座標系的。
            scale (float): 底圖相對 960x600 的縮放比例。座標要跟著乘，
                字與線寬不乘 —— 那是給人看的，不該跟著圖一起縮。

        Returns:
            np.ndarray: 畫好的**新**影像，不會動到傳進來的底圖。
        """
        def s(v):
            return int(round(v * scale))

        view = frame.copy()          #to_strategy_frame() 可能回的是原圖本身
        for i, b in enumerate(bottles):
            color = BOTTLE_COLORS[i % len(BOTTLE_COLORS)]
            xmin, ymin, xmax, ymax = b['bbox']
            cv2.rectangle(view, (s(xmin), s(ymin)), (s(xmax), s(ymax)), color, 2)

            if DRAW_LABEL:
                #標籤寫在框的上緣外側。太靠近畫面頂端時上面沒位置（會疊到左上角
                #的狀態列），改寫到**下緣外側** —— 框內一律保持乾淨，擋住的畫面
                #比字本身還難補回來。
                label_y = s(ymin) - 8
                if label_y < STATUS_ROW_PX:
                    label_y = s(ymax) + 16
                self.put_text(view, f"{b['name']} {b['confidence']:.2f}",
                              (s(xmin), label_y), color)

            if DRAW_CORNERS:
                n = b['index']
                for k, (x, y) in enumerate(b['corners']):
                    cv2.circle(view, (s(x), s(y)), 4, color, -1)
                    #編號往框的內側標，貼著畫面邊緣時才不會被夾字夾到疊在框上
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
        """把畫好框的影像發出去給 web_video_server。

        沒偵測到東西時照樣發 —— 停住的畫面跟節點掛掉看起來一模一樣，
        有連續的影像才分得出是「沒看到瓶子」還是「節點沒在跑」。
        """
        if DISPLAY_MAX_FPS > 0.0 and (now - self._last_view) < 1.0 / DISPLAY_MAX_FPS:
            return
        self._last_view = now

        try:
            view = self.to_strategy_frame(frame)
            #先縮再畫，不是畫完再縮:
            #  畫在小圖上便宜，而且字是用固定的像素大小畫的，縮小不會跟著糊掉。
            scale = 1.0
            if DISPLAY_WIDTH and DISPLAY_WIDTH != IMAGE_W:
                scale = DISPLAY_WIDTH / float(IMAGE_W)
                out_h = max(1, int(round(IMAGE_H * scale)))
                view = cv2.resize(view, (DISPLAY_WIDTH, out_h),
                                  interpolation=cv2.INTER_AREA)
            view = self.annotate(view, bottles, scale)
            msg = self._bridge.cv2_to_imgmsg(view, encoding='bgr8')
            msg.header = header          #時戳沿用來源影像，對得回原始那一幀
            self.view_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"[view] publish failed: {e}",
                                   throttle_duration_sec=5.0)

    # -------------------- 輸出 --------------------
    def report(self, bottles):
        """把結果印到終端機。畫面在 IMAGE_OUT_TOPIC，不開 cv2 視窗。"""
        if not bottles:
            if LOG_EMPTY:
                self.get_logger().info("沒有偵測到藥罐\033[K")
            return

        self.get_logger().info('________________________________________\033[K')
        for b in bottles:
            self.get_logger().info(
                f"{b['name']}  {b['label']}  conf {b['confidence']:.2f}\033[K")
            n = b['index']
            for k, (x, y) in enumerate(b['corners']):
                self.get_logger().info(
                    f"    {CORNER_ORDER[k]}  x{n + k}={x}  y{n + k}={y}\033[K")

    def publish_result(self, bottles, stamp):
        """發布偵測結果。

        格式::

            {"stamp": {"sec": 0, "nanosec": 0},
             "image_size": [960, 600],
             "count": 2,
             "bottles": [{"name": "bottle1",
                          "label": "...",
                          "confidence": 0.93,
                          "bbox": [xmin, ymin, xmax, ymax],
                          "points": {"x1": .., "y1": .., ... "x4": .., "y4": ..}}],
             "coords": {"x1": .., "y1": .., ... "x8": .., "y8": ..}}

        ``points`` 是單瓶自己的四個角點，``coords`` 是全部藥罐攤平連號的版本
        —— 同一份數字兩種讀法，看哪種好接就用哪種。
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
    node = BottleDetect()
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
