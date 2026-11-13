import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, time
import math
import numpy as np
import json
import configparser
from nstc_msgs.msg import HSVValue, Location, Zoom,DrawImage
# from nstc_msgs.msg import YUVValue, OpenCvOrder
from nstc_msgs.srv import HSVInfo,SaveHSV,BuildModel
# from nstc_msgs.srv import YUVInfo,SaveYUV,OpenCvInfo,SaveOpenCv
from std_msgs.msg import String,Int16,Bool
from dataclasses import dataclass
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from pathlib import Path
import os
import configparser

@dataclass
class ColorRange:
    HueMax: float = 1.0
    HueMin: float = 0.0
    SaturationMax: float = 1.0
    SaturationMin: float = 0.0
    BrightnessMax: float = 1.0
    BrightnessMin: float = 0.0
    LabelName: str = ""

# @dataclass
# class YUVColorRange:
#     YMax: float = 1.0
#     YMin: float = 0.0
#     CRMax: float = 128.0
#     CRMin: float = 127.0
#     CBMax: float = 128.0
#     CBMin: float = 127.0
#     YUVLabelName: str = ""


class ImageNode(Node):
    def __init__(self):

        super().__init__('image_node')
        self.get_logger().info("=======Image On=======")

        # 影像/HSV 初始狀態（保留原名）
        # -----------------------------
        self.lower = None
        self.upper = None
        #YUV
        # self.yuv_lower = None
        # self.yuv_upper = None



        # --- 開頭路徑邏輯（取代原本 strategy_root 與 ws_root 那段） ---
        self.location = ""
        self.strategy_root = self._resolve_strategy_root()

        # 仍保留舊路徑（當 strategy.ini 空或不存在時的後援）
        ws_root = Path.home() / "nstc" / "config" / "image"
        ws_root.mkdir(parents=True, exist_ok=True)
        self.path_dir = ws_root

        # 先嘗試讀 strategy.ini 的一行路徑（例如 "/ar/Parameter"）
        raw = self._read_strategy_ini_raw()
        if raw:
            # 轉成絕對路徑：<workspace>/tku/src/strategy/strategy/ar/Parameter
            self.location = self._canon_location(raw)
            # 解析實際 hsv.ini 路徑（若 location 指到資料夾，就自動補上 "hsv.ini"）
            ini = self._resolve_hsv_path()
            self.path = str(ini)
            self.get_logger().info(f"[BOOT] strategy.ini -> {raw} => {self.path}")
        else:
            # 後援：沿用舊的 image/config/hsv.ini
            self.path = str(self.path_dir / "hsv.ini")

        # if raw:
        #     # 轉成絕對路徑：<workspace>/tku/src/strategy/strategy/ar/Parameter
        #     self.yuv_location = self._canon_location(raw)
        #     # 解析實際 hsv.ini 路徑（若 location 指到資料夾，就自動補上 "hsv.ini"）
        #     ini = self._resolve_yuv_path()
        #     self.yuv_path = str(ini)
        #     self.get_logger().info(f"[BOOT] strategy.ini -> {raw} => {self.yuv_path}")
        # else:
        #     # 後援：沿用舊的 image/config/hsv.ini
        #     self.yuv_path = str(self.path_dir / "yuv.ini")
        
        # # 設定顏色範圍的標籤（保留原名）
        self.labels = ["orange", "yellow", "blue", "green", "black", "red", "white", "others"]
        self.color_labels = {
            "BlackLabel":   {"label": 1, "color": [255,   0, 255]},   # 粉
            "BlueLabel":    {"label": 2, "color": [128,   0, 128]},   # 紫
            "GreenLabel":   {"label": 3, "color": [  0,   0, 128]},   # 深藍
            "OrangeLabel":  {"label": 4, "color": [128,   0,   0]},   # 深紅
            "RedLabel":     {"label": 5, "color": [255, 255,   0]},   # 黃
            "YellowLabel":  {"label": 6, "color": [128, 128,   0]},   # 黃綠
            "WhiteLabel":   {"label": 7, "color": [  0, 255, 255]},   # 青綠
            "OthersLabel":  {"label": 8, "color": [255,   0, 128]},   # 紫粉
        }

        # label 編號 -> BGR 的查表，用於一次把 label_map 轉成偽彩色影像
        # （取代原本每色一次的 numpy fancy indexing，實測是最大的熱點）
        self.color_lut = np.zeros((1, 256, 3), dtype=np.uint8)   # 未定義的 label 一律黑
        for meta in self.color_labels.values():
            self.color_lut[0, int(meta["label"])] = np.array(meta["color"], dtype=np.uint8)

        # label 編號 -> 該編號的常數填充圖，供 cv2.copyTo 使用；依影像尺寸延遲配置
        self._label_fill_cache = {}

        # # 顏色參數表
        self.HSVColorRange = {label: ColorRange(LabelName=label) for label in self.labels}
        # self.YUVColorRange = {label: YUVColorRange(YUVLabelName=label) for label in self.labels}
        qos_latest = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # depth=1：影像鏈路只留最新一幀。預設的 10 會讓 publisher 與 subscriber
        # 各堆到 10 幀，下游一慢就累積出數百 ms 延遲。
        # 注意 label_matrix 維持 depth=10 不動，那是策略層在用的。
        qos_img = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        self.info_pub = self.create_publisher(String, 'object_info', qos_latest)
        # self.yuv_info_pub = self.create_publisher(String, 'yuv_object_info', qos_latest)
        self.label_pub = self.create_publisher(Image, 'label_matrix', 10)
        # 顏色 class 標籤圖：0=背景，1~8=color_labels 的 label 編號，供 overlap_node 取交集
        self.color_label_pub = self.create_publisher(Image, 'color_label_map', qos_latest)
        # self.yuv_label_pub = self.create_publisher(Image, 'yuv_label_matrix', 10)
        # 每個顏色各一個 Publisher：detections/<label>
        self.det_pubs = {
            label: self.create_publisher(String, f'detections/{label}', qos_latest)
            for label in self.labels
        }
        # self.yuv_det_pubs = {
        #     label: self.create_publisher(String, f'yuv_detections/{label}', qos_latest)
        #     for label in self.labels
        # }

        self.kernel3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        # ===== JSON 類 Topic 的「節流 + 去抖」狀態 =====
        # 最快 20Hz；想更省可以把 0.05 改大（例如 0.1 = 10Hz）
        self._pub_period = 0.05  # 秒
        # 每個顏色一個時間戳 + 總表一個
        self._last_pub_t = {"info": 0.0, **{f"det_{l}": 0.0 for l in self.labels}}
        # self._yuv_last_pub_t = {"info": 0.0, **{f"det_{l}": 0.0 for l in self.labels}}
        # 每個顏色上一份 payload（字串）—用來判斷「內容是否有變化」
        self._last_payload = {l: "" for l in self.labels}
        # self._yuv_last_payload = {l: "" for l in self.labels}
        # 總表上一份 payload（字串）
        self._last_info_payload = ""
        # self._yuv_last_info_payload = ""

        self.declare_parameter('zoom_in', 1.0)
        self.zoom = float(self.get_parameter('zoom_in').value)

        # 相機來源：決定 zoomin 存放於哪個 ini
        #   usb -> CameraSet.ini（usb_cam 擁有）  zed -> ZedCameraSet.ini（bridge node 擁有）
        self.declare_parameter('camera_source', 'zed')
        self.camera_source = str(self.get_parameter('camera_source').value).strip().lower()

        # usb 相機是 960x540（16:9），ZED 是 960x600（16:10）。usb 模式下先把影像
        # 上下補黑成 960x600，讓裁切放大、對外座標、網頁顯示框都只有一套規格。
        self.declare_parameter('pad_to_height', 600)
        self._pad_top = 0            # 這一幀上緣補了幾列（輸入座標）
        self._pad_bottom = 0         # 下緣補了幾列
        self._pad_out_top = 0        # 裁切放大「之後」上緣還有幾列是補黑區
        self._pad_out_bot_start = None   # 之後從第幾列起是補黑區；None 表示沒有

        # 連通元件的最小面積門檻（960x600 下等價於原本 320x240 的 50）
        # 可用 ros2 param set /image_node min_blob_area <n> 即時調整
        self.declare_parameter('min_blob_area', 375)

        # 色模去雜訊的形態學 kernel 大小。960x600 的線性尺度是舊 320x240 的 3 倍，
        # 所以等價於舊版 OPEN:3 的是 9。設 <3 則完全不做形態學。
        self.declare_parameter('morph_kernel', 9)
        self._morph_kernel = None

        self.bridge = CvBridge()
        # 影像來源 topic 依相機分流。原本寫死 ZED 的 topic，usb 模式下沒有任何節點
        # 在發，image_callback 一次都不會被呼叫，整個色模就是黑的。
        CAMERA_TOPICS = {
            'usb': '/camera1/image_raw',
            'zed': '/zed/zed_node/left/image_rect_color',
        }
        self.declare_parameter('image_topic', '')     # 非空字串時蓋過 camera_source
        override = str(self.get_parameter('image_topic').value).strip()
        self.image_topic = override or CAMERA_TOPICS.get(self.camera_source, CAMERA_TOPICS['zed'])
        self.get_logger().info(f"[image] camera_source={self.camera_source} topic={self.image_topic}")
        self.subscription = self.create_subscription(Image, self.image_topic, self.image_callback, qos_img)
        self.zoom_in = self.create_publisher(Image, 'zoom_in', qos_img)

        self.ZoomSub = self.create_subscription(Zoom, '/Zoom_In_Topic', self.zoomValue, qos_img)


        self.OpenCvOrders = []
        # self.OpenCv_sub = self.create_subscription(OpenCvOrder, '/openCvOrder', self.OpenCvOrder_callback, 10)
        # self.OpenCv_pub = self.create_publisher(
        #     OpenCvOrder,
        #     '/openCvOrder_back',
        #     qos_latched
        # )
        self.op_table = {
            "ERODE": self.op_erode,
            "DILATE": self.op_dilate,
            "OPEN": self.op_open,
            "CLOSE": self.op_close,
            "GRADIENT": self.op_gradient,
            # "TOPHAT":self.op_tophat,
            # "BLACKHAT":self.op_blackhat,
            "BLUR":self.op_blur,
            "MEDIANBLUR":self.op_medianblur,
            "GAUSSIANBLUR":self.op_gaussianblur,
            "BILATERALFILTER":self.op_bilateralfilter,
            "SHARPEN":self.op_sharpen,
            'WATERSHED': self.op_watershed,
        }

        self.build_status_sub = self.create_subscription(
            Int16, '/BuildStatus', self.build_status_callback, 10
        )
        # self.yuv_build_status_sub = self.create_subscription(
        #     Int16, '/yuv_BuildStatus', self.yuv_build_status_callback, 10
        # )
        self.build_status = 0
        self.select_color = "red"

        # self.yuv_build_status = 0
        # self.yuv_select_color = "red"


        self.color_model_HSV = self.create_subscription(
            HSVValue, '/HSVValue_Topic', self.color_model_HSV_callback, 1000
        )
        # self.color_model_YUV = self.create_subscription(
        #     YUVValue, '/YUVValue_Topic', self.color_model_YUV_callback, 1000
        # )
        self.location_sub = self.create_subscription(
            Location, '/location', self.location_callback, 1000
        )
  
        self.build_model_srv = self.create_service(
            BuildModel, '/BuildModel', self.build_model_callback
        )


        self.location_pub = self.create_publisher(Location, '/locationBack', 10)

        self.save_hsv = self.create_service(SaveHSV, '/SaveHSV', self.save_hsv_callback)
        self.hsv_load = self.create_service(HSVInfo, '/LoadHSVInfo', self.load_hsv_info_callback)

        # self.save_yuv = self.create_service(SaveYUV, '/SaveYUV', self.save_yuv_callback)
        # self.yuv_load = self.create_service(YUVInfo, '/LoadYUVInfo', self.load_yuv_info_callback)

        # self.save_opencv = self.create_service(SaveOpenCv, '/SaveOpenCv', self.save_opencv_callback)
        # self.opencv_load = self.create_service(OpenCvInfo, '/LoadOpenCvInfo', self.load_opencv_info_callback)

        self.processed_image = self.create_publisher(Image, 'processed_image', qos_img)
        # self.yuv_processed_image = self.create_publisher(Image, 'yuvprocessed_image', qos_img)
        self._img_period = 0.001 
        # self._last_img_pub = {"build_image": 0.0, "mask_image": 0.0, "zoom_in": 0.0}
        # self._last_yuv_img_pub = {"build_image": 0.0, "mask_image": 0.0}
        self._last_hsv_img_pub = {"build_image": 0.0, "mask_image": 0.0}
        # self._last_yuv_img_pub = {"build_image": 0.0, "mask_image": 0.0}
        self._last_img_pub = {"zoom_in": 0.0}

        self.init_hsv_from_ini(active_label=self.select_color)
        self.load_zoomin_from_camera_ini()
        # self.init_yuv_from_ini(active_label=self.yuv_select_color)
        self.init_opencv_from_ini()

        self.draw_image_array = [] # 儲存繪圖資訊
        self.draw_sub = self.create_subscription(
            DrawImage,
            '/drawimage', 
            self.draw_image_callback, 
            qos_img
        )
        # 繪圖是以 cnt 為 key 累積的，策略停掉之後圖形會一直留在畫面上。
        # 提供一個明確的清除入口給網頁按鈕與策略（API.clearDrawImage）使用。
        self.draw_clear_sub = self.create_subscription(
            Bool, '/drawimage/clear', self.draw_clear_callback, 10
        )


    def draw_clear_callback(self, msg):
        """清除所有已累積的繪圖。收到 data=False 時不動作，方便當開關用。"""
        if not msg.data:
            return
        n = len(self.draw_image_array)
        self.draw_image_array.clear()
        self.get_logger().info(f"[DrawImage] 已清除 {n} 個圖形")

    def draw_image_callback(self, msg):
        # 尋找是否有相同編號的圖形
        found = False
        for item in self.draw_image_array:
            if item.cnt == msg.cnt:
                # 更新現有圖形資料
                item.xmin, item.ymin = msg.xmin, msg.ymin
                item.xmax, item.ymax = msg.xmax, msg.ymax
                item.rvalue, item.gvalue, item.bvalue = msg.rvalue, msg.gvalue, msg.bvalue
                item.thickness = msg.thickness
                found = True
                break
        
        if not found:
            # 新增圖形
            self.draw_image_array.append(msg)

    # def publish_opencv_order(self):
    #     msg = OpenCvOrder()
    #     msg.order = self.OpenCvOrders
    #     self.OpenCv_pub.publish(msg)
    #     self.get_logger().info(f"[OpenCV INI] pushed to web: {self.OpenCvOrders}")

    # def OpenCvOrder_callback(self,msg):
    #     self.OpenCvOrders = list(msg.order)
    #     self.get_logger().info(f"OpenCvOrders: {self.OpenCvOrders}")
    
    def make_kernel(self, n=None, *, allow_even: bool = False) -> np.ndarray:
        # 1) 先處理沒有給或給奇怪值的情況
        if n is None or n == "":
            k = 3
        else:
            # 有些情況前端會給 '3' 或 '3.0'
            k = int(float(n))
        
        # 2) 不要讓它變成 0 或負的
        if k < 1:
            k = 1

        # 3) 預設強制成奇數，除非呼叫的人說 allow_even=True
        if (not allow_even) and (k % 2 == 0):
            k += 1

        return np.ones((k, k), np.uint8)
    
    def op_erode(self, img, n):
        kernel = self.make_kernel(n)
        return cv2.erode(img, kernel, iterations=1)

    def op_dilate(self, img, n):
        kernel = self.make_kernel(n)
        return cv2.dilate(img, kernel, iterations=1)
    
    def op_open(self, img, n):
        kernel = self.make_kernel(n)
        return cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)

    def op_close(self, img, n):
        kernel = self.make_kernel(n)
        return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

    def op_gradient(self, img, n):
        kernel = self.make_kernel(n)
        return cv2.morphologyEx(img, cv2.MORPH_GRADIENT, kernel)

    # def op_tophat(self, img, n):
    #     kernel = self.make_kernel(n)
    #     return cv2.morphologyEx(img, cv2.MORPH_TOPHAT, kernel)

    # def op_blackhat(self, img, n):
    #     kernel = self.make_kernel(n)
    #     return cv2.morphologyEx(img, cv2.MORPH_BLACKHAT, kernel)

    def op_blur(self, img, n):
        """
        均值濾波：每個像素用鄰域的平均值取代，去小雜訊但邊緣會糊。
        前端傳進來的 n 視為 kernel 大小。
        """
        # 均值濾波其實可以用偶數，不過我們可以偷用原本的 make_kernel
        k = self.make_kernel(n, allow_even=True).shape[0]
        if k <= 1:
            return img
        return cv2.blur(img, (k, k))

    def op_medianblur(self, img, n):
        """
        中值濾波：對椒鹽雜訊特別有用，必須是奇數且 >= 3。
        """
        # 這裡就用你原本的 make_kernel，因為它會幫你變奇數
        k = self.make_kernel(n).shape[0]
        if k < 3:
            k = 3
        return cv2.medianBlur(img, k)

    def op_gaussianblur(self, img, n):
        """
        高斯濾波：比均值濾波自然，常用在後面要做邊緣或二值化前的平滑。
        需要奇數的 kernel。
        """
        k = self.make_kernel(n).shape[0]
        if k < 3:
            k = 3
        # sigmaX=0 交給 OpenCV 自己算
        return cv2.GaussianBlur(img, (k, k), 0)

    def op_bilateralfilter(self, img, n):
        """
        雙邊濾波：可以「去雜訊但保留邊緣」，比前面三個都高級一點。
        這個的參數跟前面不一樣，我們用前端的 n 當作「大致的強度」，轉成 d/sigma。
        """
        # 轉成整數，給不給都用 3
        d = int(float(n)) if n else 3
        if d < 1:
            d = 1
        # 為了不要一按就爆很慢，限制一下範圍
        if d > 15:
            d = 15

        # 這兩個是經驗值：d 越大我們就讓 sigma 大一點
        sigma_color = 30 + d * 10
        sigma_space = 30 + d * 10

        return cv2.bilateralFilter(img, d, sigma_color, sigma_space)
    

    def op_sharpen(self, img, n):
        """
        銳化：用一個 3x3 kernel 做銳化，n 變大就加強一點中心權重。
        這種寫法不會改 input channel 數，所以彩色也能用。
        """
        # n 當成「強度等級」，1~5 之間就好
        level = int(float(n)) if n else 1
        if level < 1:
            level = 1
        if level > 5:
            level = 5

        # 基本銳化 kernel：中心大、四周負的
        # 基本版是 [[0,-1,0],[-1,5,-1],[0,-1,0]]
        base_center = 4 + level   # level=1 → 5, level=5 → 9
        kernel = np.array([
            [0, -1,  0],
            [-1, base_center, -1],
            [0, -1,  0]
        ], dtype=np.float32)

        sharp = cv2.filter2D(img, -1, kernel)
        return sharp
    
    def op_watershed(self, img, n):    
        if img is None:
            return img

        # 1) 參數處理
        k = int(float(n)) if n else 3
        if k < 3:
            k = 3
        if k % 2 == 0:
            k += 1

        # 2) 轉灰階 + 模糊
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (k, k), 0)

        # 3) Otsu 二值化（先不反相）
        _, th = cv2.threshold(blur, 0, 255,
                            cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # 3-1) 判斷前景是不是太大，如果是白底黑幣就反轉
        white_ratio = th.mean() / 255.0  # 約略算一下白色比例
        # 如果白色超過 0.6，表示背景可能是白色、硬幣是暗的，要反轉來讓硬幣是白色
        if white_ratio > 0.6:
            th = cv2.bitwise_not(th)

        # 4) 形態學開運算，去雜訊
        kernel = self.make_kernel(k, allow_even=True)
        opening = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel, iterations=2)

        # 5) 確定背景
        sure_bg = cv2.dilate(opening, kernel, iterations=3)

        # 6) 距離轉換 -> 確定前景（硬幣中心）
        dist = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
        # 0.4~0.6 都可以，越大分越開，但太大會斷掉
        _, sure_fg = cv2.threshold(dist, 0.5 * dist.max(), 255, 0)
        sure_fg = np.uint8(sure_fg)

        # 7) 不確定區域
        unknown = cv2.subtract(sure_bg, sure_fg)

        # 8) 標記連通域當作 marker
        num_labels, markers = cv2.connectedComponents(sure_fg)
        markers = markers + 1            # 符合 cv2.watershed 要求
        markers[unknown == 255] = 0      # 不確定的地方設 0

        # 9) 跑分水嶺
        markers = cv2.watershed(img, markers)

        # 10) 畫結果
        result = img.copy()
        h, w = result.shape[:2]

        # 分水嶺的邊界是 -1
        result[markers == -1] = [0, 0, 255]

        # 11) 把每一個硬幣標上數字
        coin_count = 0
        for label in np.unique(markers):
            # label 說明：
            # -1: 邊界
            #  1: 背景
            # >=2: 真的被分出來的區域（我們當成一個硬幣）
            if label <= 1:
                continue

            ys, xs = np.where(markers == label)
            if len(xs) == 0:
                continue

            coin_count += 1

            cx = int(xs.mean())
            cy = int(ys.mean())

            cv2.putText(
                result,
                str(coin_count),
                (cx, cy),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

        # 12) 左上角寫總數（依字大小畫框）
        text = f"{coin_count}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2

        # 先量字的大小
        (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, thickness)

        # 設一點點邊距，讓字不要貼邊
        pad = 5
        x1, y1 = 5, 5
        x2 = x1 + text_w + pad * 2
        y2 = y1 + text_h + baseline + pad * 2

        # 畫框 -> 跟實際字差不多大
        cv2.rectangle(result, (x1, y1), (x2, y2), (0, 0, 0), -1)

        # 再把字寫上去（要加回 padding）
        cv2.putText(
            result,
            text,
            (x1 + pad, y2 - pad - baseline),
            font,
            font_scale,
            (0, 255, 255),
            thickness,
            cv2.LINE_AA
        )

        # 你如果想讓別的地方也讀得到，可以順便存一下
        self.last_coin_count = coin_count

        return result

    def process_by_order(self,img, order_list,ba):
        out = img
        for item in order_list:
            # 1. 拆字串
            s = str(item)
            name, num = (s.split(":", 1) + [""])[:2]  # 一行寫完拆 name 跟數字
            name = name.strip().upper()
            num = num.strip()
            n = int(num) if num else None

            # 2. 找對應的處理函式
            func = self.op_table.get(name)
            if func is None:
                # 不認得的指令就跳過
                self.get_logger().info(f"[OPEN CV] unknown op:{name}")
                # print(f"[WARN] unknown op: {name}")
                continue            
            # 3. 執行並更新 out
            if ba == "b" :
                if (name == "WATERSHED") or (name == "SHARPEN"):
                    out = func(out, n)
                else:
                    continue
            elif (name == "WATERSHED") or (name == "SHARPEN"):
                continue
            else:
                out = func(out, n)
            # self.get_logger().info(f"[OPEN CV]Processing:{name}")

        return out

    def _resolve_hsv_path(self) -> Path:
        if getattr(self, "location", None):
            p = Path(self.location)
            ini = p if p.suffix else (p / "hsv.ini")
        else:
            ini = Path(self.path_dir) / "hsv.ini"

        # 不論 default 或 location，都確保父資料夾存在
        ini.parent.mkdir(parents=True, exist_ok=True)
        return ini
    # def _resolve_yuv_path(self) -> Path:
    #     if getattr(self, "location", None):
    #         p = Path(self.location)
    #         ini = p if p.suffix else (p / "yuv.ini")
    #     else:
    #         ini = Path(self.path_dir) / "yuv.ini"

    #     # 不論 default 或 location，都確保父資料夾存在
    #     ini.parent.mkdir(parents=True, exist_ok=True)
    #     return ini
    
    # def _resolve_opencv_path(self) -> Path:
    #     if getattr(self, "location", None):
    #         p = Path(self.location)
    #         ini = p if p.suffix else (p / "opencv.ini")
    #     else:
    #         # 沒指定路徑就跟以前一樣丟到 image/config
    #         ini = Path(self.path_dir) / "opencv.ini"

    #     ini.parent.mkdir(parents=True, exist_ok=True)
    #     return ini
    def _canon_location(self, raw: str) -> str:
        if not raw:
            return ""

        # [修正] 強制去掉開頭所有的斜線，避免系統誤判為根目錄絕對路徑
        # 例如 "//Parameter" 會變成 "Parameter"
        clean_raw = raw.strip().lstrip("/")
        
        p = Path(clean_raw)

        # [修正] 不管它是什麼，一律接在 strategy_root 底下
        # 這樣路徑就會變成 /home/aa/tku/src/strategy/strategy/Parameter (安全路徑)
        return str(self.strategy_root / p)
    
    def _resolve_strategy_root(self) -> Path:
        # 盡量用目前檔案向上回推 <workspace>/tku/src/strategy/strategy
        for up in Path(__file__).resolve().parents:
            if up.name == "src" and (up / "strategy" / "strategy").is_dir():
                return up / "strategy" / "strategy"
        # 找不到就退回家目錄預設
        return Path.home() / "nstc" / "config"

    def _read_strategy_ini_raw(self) -> str:
        try:
            p = self._strategy_ini_path()   # 你原本就有這個函式
            txt = p.read_text(encoding="utf-8")
            for line in txt.splitlines():
                s = line.strip()
                if s:
                    return s
        except Exception:
            pass
        return ""

    def zoomValue(self,msg):
        self.zoom = float(msg.zoomin)

    def build_status_callback(self, msg):
        print("BuildStatus:", msg.data)
        self.build_status = msg.data
    
    # def yuv_build_status_callback(self, msg):
    #     print("yuvBuildStatus:", msg.data)
    #     self.yuv_build_status = msg.data

    def color_model_HSV_callback(self, msg):
        self.HSVColorRange[self.select_color].HueMax = msg.hmax
        self.HSVColorRange[self.select_color].HueMin = msg.hmin
        self.HSVColorRange[self.select_color].SaturationMax = msg.smax
        self.HSVColorRange[self.select_color].SaturationMin = msg.smin
        self.HSVColorRange[self.select_color].BrightnessMax = msg.vmax
        self.HSVColorRange[self.select_color].BrightnessMin = msg.vmin
        self.lower = np.array([msg.hmin, msg.smin, msg.vmin], dtype=np.uint8)
        self.upper = np.array([msg.hmax, msg.smax, msg.vmax], dtype=np.uint8)

    # def color_model_YUV_callback(self, msg):
    #     self.YUVColorRange[self.yuv_select_color].YMax = msg.ymax
    #     self.YUVColorRange[self.yuv_select_color].YMin = msg.ymin
    #     self.YUVColorRange[self.yuv_select_color].CRMax = msg.crmax
    #     self.YUVColorRange[self.yuv_select_color].CRMin = msg.crmin
    #     self.YUVColorRange[self.yuv_select_color].CBMax = msg.cbmax
    #     self.YUVColorRange[self.yuv_select_color].CBMin = msg.cbmin
    #     self.yuv_lower = np.array([msg.ymin, msg.crmin, msg.cbmin], dtype=np.uint8)
    #     self.yuv_upper = np.array([msg.ymax, msg.crmax, msg.cbmax], dtype=np.uint8)

    def _strategy_ini_path(self) -> Path:
        env = os.environ.get("tku_STRATEGY_INI")
        if env:
            p = Path(env).expanduser().resolve()
        else:
            # 由目前檔案一路往上找 src，確保路徑相對於專案
            p = None
            for up in Path(__file__).resolve().parents:
                if up.name == "src" and (up / "strategy").is_dir():
                    p = up / "strategy" / "strategy" / "strategy.ini"
                    break
            if p is None:
                # 萬一找不到就退回家目錄一個預設位置
                p = Path.home() / "nstc/config/strategy.ini"
        p.parent.mkdir(parents=True, exist_ok=True)
        return p

    def _write_strategy_ini_raw(self, raw: str) -> None:
        ini_path = self._strategy_ini_path()
        tmp = ini_path.with_suffix(".tmp")
        content = (raw or "").strip() + "\n"
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(content)
        tmp.replace(ini_path)  # 原子替換，避免半寫入

        # === 以下為新增：同步儲存 .js 檔的段落 ===
        js_path = ini_path.with_suffix(".js") # 將附檔名改為 .js，儲存在同一個資料夾
        js_tmp = js_path.with_suffix(".js.tmp")
        
        # 這裡設定你要寫入 JS 檔的內容，你可以依照需求修改變數名稱或格式
        js_content = f'window.currentStrategy = "{raw.strip()}";\n' 
        
        with open(js_tmp, "w", encoding="utf-8") as f:
            f.write(js_content)
        js_tmp.replace(js_path)
        # =======================================

    # def _write_autoload_js_from_raw(self, raw: str) -> str:
    #     s = (raw or "").strip()
    #     loc = s.strip("/").split("/", 1)[0] if s else ""

    #     env = os.environ.get("tku_STRATEGY_AUTOLOAD")
    #     if env:
    #         js_path = Path(env).expanduser().resolve()
    #     else:
    #         # [修正] 使用 Path.home() 自動抓取當前使用者目錄 (例如 /home/aa)
    #         # 避免寫死 /home/iclab 導致 Permission denied
    #         js_path = Path.home() / "ros2_adult" / "hurocup_interface" / "strategy_autoload.js"

    #     js_path.parent.mkdir(parents=True, exist_ok=True)

    #     tmp = js_path.with_suffix(".tmp")
    #     # 注意：這裡原本是用 write_text，但我保留你的寫法以防萬一，改用 open 比較保險
    #     with open(tmp, "w", encoding="utf-8") as f:
    #          f.write(f'window._strategy_location = {json.dumps(loc)};\n')

    #     tmp.replace(js_path)
    #     self.loc = loc
    #     return loc

    def build_model_callback(self, request, response):
        self.get_logger().info(f"[BuildModel] request, build={request.build}")

        # 1) 重新從 hsv.ini 把所有顏色載回 HSVColorRange
        try:
            hsv_ini = self._resolve_hsv_path()
            if hsv_ini.exists():
                config = configparser.ConfigParser()
                config.read(hsv_ini)
                for label, data in self.HSVColorRange.items():
                    if label in config:
                        sec = config[label]
                        data.HueMax        = float(sec.get("hue_max",        data.HueMax))
                        data.HueMin        = float(sec.get("hue_min",        data.HueMin))
                        data.SaturationMax = float(sec.get("s_max",          data.SaturationMax))
                        data.SaturationMin = float(sec.get("s_min",          data.SaturationMin))
                        data.BrightnessMax = float(sec.get("v_max",          data.BrightnessMax))
                        data.BrightnessMin = float(sec.get("v_min",          data.BrightnessMin))
            else:
                self.get_logger().warn(f"[BuildModel] HSV ini not found: {hsv_ini}")
        except Exception as e:
            self.get_logger().error(f"[BuildModel] reload HSV failed: {e}")

        # 2) 同樣方式讀 yuv.ini 回 YUVColorRange（可選）
        # try:
        #     yuv_ini = self._resolve_yuv_path()
        #     if yuv_ini.exists():
        #         config = configparser.ConfigParser()
        #         config.read(yuv_ini)
        #         for label, data in self.YUVColorRange.items():
        #             if label in config:
        #                 sec = config[label]
        #                 data.YMax  = float(sec.get("y_max",  data.YMax))
        #                 data.YMin  = float(sec.get("y_min",  data.YMin))
        #                 data.CRMax = float(sec.get("cr_max", data.CRMax))
        #                 data.CRMin = float(sec.get("cr_min", data.CRMin))
        #                 data.CBMax = float(sec.get("cb_max", data.CBMax))
        #                 data.CBMin = float(sec.get("cb_min", data.CBMin))
        #     else:
        #         self.get_logger().warn(f"[BuildModel] YUV ini not found: {yuv_ini}")
        # except Exception as e:
        #     self.get_logger().error(f"[BuildModel] reload YUV failed: {e}")

        # 3) 讓前端知道「建好了」，解鎖按鈕
        response.already = True
        return response

    def location_callback(self, msg):
        # 1. 讀進來的 raw（前端會送 "ar/Parameter" 或 "bb/Parameter"）
        raw = (msg.data or "").strip()
        if not raw:
            raw = "ar/Parameter"   # 預設用 ar

        # 2. 一律視為相對路徑，如 "bb/Parameter"
        rel = Path(raw)
        if rel.is_absolute():
            # 如果前端送了絕對路徑，強制只保留最後兩層
            parts = rel.parts
            if len(parts) >= 2:
                rel = Path(parts[-2]) / parts[-1]
            else:
                rel = Path(rel.name)

        loc_str = str(rel)  # e.g. "bb/Parameter"

        # 3. 寫回 strategy.ini
        self._write_strategy_ini_raw(loc_str)

        # 4. 寫 autoload.js（只寫 ar / bb 那一段給前端）
        # try:
        #     loc = self._write_autoload_js_from_raw(loc_str)
        #     self.get_logger().info(f"[AUTOLOAD.JS] write ok: {loc}")
        # except Exception as e:
        #     self.get_logger().error(f"[AUTOLOAD.JS] write fail: {e}")

        # 5. 更新內部 location / path，後續 _resolve_hsv_path() 就會指到正確目錄
        self.location = self._canon_location(loc_str)
        ini = self._resolve_hsv_path()
        self.path = str(ini)

        # 立刻用新路徑 reload HSV/YUV/OpenCV（如果檔案不存在會自動建）
        self.init_hsv_from_ini(active_label=self.select_color)
        self.load_zoomin_from_camera_ini()
        # self.init_yuv_from_ini(active_label=self.yuv_select_color)
        self.init_opencv_from_ini()

        # 6. 對外轉發「乾淨的」 loc（ar/Parameter / bb/Parameter）
        msg.data = loc_str
        self.location_pub.publish(msg)


    def load_hsv_info_callback(self, request, response):
        color = request.colorlabel
        self.select_color = color
        ini_path = str(self._resolve_hsv_path())
        self.get_logger().info(f"[HSV INI] load request for '{color}' from: {ini_path}")

        # 預設回傳值（先用目前記憶體，沒有就 0）
        def _defaults_from_memory():
            cd = self.HSVColorRange.get(color)
            if cd is None:
                return 0, 0, 0, 0, 0, 0
            return (
                int(cd.HueMin), int(cd.HueMax),
                int(cd.SaturationMin), int(cd.SaturationMax),
                int(cd.BrightnessMin), int(cd.BrightnessMax)
            )

        hmin, hmax, smin, smax, vmin, vmax = _defaults_from_memory()

        try:
            config = configparser.ConfigParser()
            read_ok = config.read(ini_path)  # 讀檔
            if not read_ok:
                self.get_logger().warn(f"[HSV INI] ini not found, using memory/defaults: {self.path}")
            elif color in config:
                sec = config[color]

                # 取值並做邊界限制
                hmin = max(0,   min(179, int(sec.get("hue_min",        hmin))))
                hmax = max(0,   min(179, int(sec.get("hue_max",        hmax))))
                smin = max(0,   min(255, int(sec.get("saturation_min", smin))))
                smax = max(0,   min(255, int(sec.get("saturation_max", smax))))
                vmin = max(0,   min(255, int(sec.get("brightness_min", vmin))))
                vmax = max(0,   min(255, int(sec.get("brightness_max", vmax))))
            else:
                self.get_logger().warn(f"[HSV INI] section '{color}' not found, using memory/defaults.")

            # 寫回 response
            response.hmin = hmin
            response.hmax = hmax
            response.smin = smin
            response.smax = smax
            response.vmin = vmin
            response.vmax = vmax

            # 更新記憶體表與 inRange 門檻
            if color not in self.HSVColorRange:
                self.HSVColorRange[color] = ColorRange(LabelName=color)

            self.HSVColorRange[color].HueMin        = hmin
            self.HSVColorRange[color].HueMax        = hmax
            self.HSVColorRange[color].SaturationMin = smin
            self.HSVColorRange[color].SaturationMax = smax
            self.HSVColorRange[color].BrightnessMin = vmin
            self.HSVColorRange[color].BrightnessMax = vmax

            self.lower = np.array([hmin, smin, vmin], dtype=np.uint8)
            self.upper = np.array([hmax, smax, vmax], dtype=np.uint8)

            self.get_logger().info(
                f"[HSV INI] '{color}' -> H[{hmin},{hmax}] S[{smin},{smax}] V[{vmin},{vmax}]"
            )
        except Exception as e:
            self.get_logger().error(f"[HSV INI] load failed: {e}")
            # 讀檔失敗時仍回目前記憶體/預設值
            response.hmin = hmin
            response.hmax = hmax
            response.smin = smin
            response.smax = smax
            response.vmin = vmin
            response.vmax = vmax

        return response
    
    # def load_yuv_info_callback(self, request, response):
    #     color = request.colorlabel
    #     self.yuv_select_color = color
    #     ini_path = str(self._resolve_yuv_path())
    #     self.get_logger().info(f"[YUV INI] load request for '{color}' from: {ini_path}")

    #             # 預設回傳值（先用目前記憶體，沒有就 0）
    #     def yuv_defaults_from_memory():
    #         cd = self.YUVColorRange.get(color)
    #         if cd is None:
    #             return 0, 0, 0, 0, 0, 0
    #         return (
    #             int(cd.YMin ), int(cd.YMax ),
    #             int(cd.CRMin ), int(cd.CRMax ),
    #             int(cd.CBMin), int(cd.CBMax )
    #         )

    #     ymin, ymax, crmin, crmax, cbmin, cbmax = yuv_defaults_from_memory()

    #     try:
    #         config = configparser.ConfigParser()
    #         read_ok = config.read(ini_path)  # 讀檔
    #         if not read_ok:
    #             self.get_logger().warn(f"[YUV INI] ini not found, using memory/defaults: {self.path}")
    #         elif color in config:
    #             sec = config[color]

    #             # 取值並做邊界限制
    #             ymin = max(0,   min(255, int(sec.get("y_min",  ymin ))))
    #             ymax = max(0,   min(255, int(sec.get("y_max",  ymax ))))
    #             crmin = max(0,   min(255, int(sec.get("cr_min", crmin ))))
    #             crmax = max(0,   min(255, int(sec.get("cr_max", crmax ))))
    #             cbmin = max(0,   min(255, int(sec.get("cb_min", cbmin))))
    #             cbmax = max(0,   min(255, int(sec.get("cb_max",cbmax ))))
    #         else:
    #             self.get_logger().warn(f"[YUV INI] section '{color}' not found, using memory/defaults.")

    #         # 寫回 response
    #         response.ymin = ymin 
    #         response.ymax = ymax 
    #         response.crmin = crmin 
    #         response.crmax = crmax 
    #         response.cbmin = cbmin
    #         response.cbmax = cbmax 

    #         # 更新記憶體表與 inRange 門檻
    #         if color not in self.YUVColorRange:
    #             self.YUVColorRange[color] = YUVColorRange(YUVLabelName=color)

    #         self.YUVColorRange[color].YMin   = ymin 
    #         self.YUVColorRange[color].YMax   = ymax 
    #         self.YUVColorRange[color].CRMin  = crmin 
    #         self.YUVColorRange[color].CRMax  = crmax 
    #         self.YUVColorRange[color].CBMin  = cbmin
    #         self.YUVColorRange[color].CBMax  = cbmax 

    #         self.yuv_lower = np.array([ymin, crmin, cbmin], dtype=np.uint8)
    #         self.yuv_upper = np.array([ymax, crmax, cbmax], dtype=np.uint8)

    #         self.get_logger().info(
    #             f"[YUV INI] '{color}' -> Y[{ymin},{ymax}] CR[{crmin},{crmax}] CB[{cbmin},{cbmax}]"
    #         )
    #     except Exception as e:
    #         self.get_logger().error(f"[YUV INI] load failed: {e}")
    #         # 讀檔失敗時仍回目前記憶體/預設值
    #         response.ymin = ymin
    #         response.ymax = ymax
    #         response.crmin = crmin
    #         response.crmax = crmax
    #         response.cbmin = cbmin
    #         response.cbmax = cbmax

    #     return response
    
    # def load_opencv_info_callback(self, request, response):
    #     # 1) 確保記憶體是最新的（因為你可能剛剛 changeLocation）
    #     self.init_opencv_from_ini()

    #     # 2) 把目前這個節點裡的順序丟回去
    #     response.order = list(self.OpenCvOrders)
    #     response.already = True
    #     self.get_logger().info(f"[OpenCV INI] send to client: {response.order}")
    #     return response

    
    def _clamp(self, v, lo, hi):
        try:
            return max(lo, min(hi, int(v)))
        except Exception:
            return lo
    def init_hsv_from_ini(self, active_label: str = None):
        ini_path = str(self._resolve_hsv_path())
        cfg = configparser.ConfigParser()

        # 1) 檢查並讀入檔案
        read_ok = cfg.read(ini_path)
        if not read_ok:
            # 沒檔 -> 建立模板 (不改變記憶體中的數值，只是寫檔)
            
            Path(ini_path).parent.mkdir(parents=True, exist_ok=True)
            for label in self.labels:
                # 這裡給「全通」作為模板，方便第一次看到效果；你也可以改成 0~1
                cfg[label] = {
                    "hue_min": "0",   "hue_max": "179",
                    "saturation_min": "0",   "saturation_max": "255",
                    "brightness_min": "0",   "brightness_max": "255",
                }
            try:
                with open(ini_path, "w") as f:
                    cfg.write(f)
                self.get_logger().info(f"[HSV INI] created template: {ini_path}")
            except Exception as e:
                self.get_logger().error(f"[HSV INI] failed to create template: {e}")
        else:
            # 2) 有檔 -> 逐 label 載入到記憶體
            for label in self.labels:
                if label in cfg:
                    sec = cfg[label]
                    hmin = self._clamp(sec.get("hue_min",         0),   0, 179)
                    hmax = self._clamp(sec.get("hue_max",       179),   0, 179)
                    smin = self._clamp(sec.get("saturation_min",  0),   0, 255)
                    smax = self._clamp(sec.get("saturation_max",255),   0, 255)
                    vmin = self._clamp(sec.get("brightness_min",  0),   0, 255)
                    vmax = self._clamp(sec.get("brightness_max",255),   0, 255)

                    self.HSVColorRange[label].HueMin        = hmin
                    self.HSVColorRange[label].HueMax        = hmax
                    self.HSVColorRange[label].SaturationMin = smin
                    self.HSVColorRange[label].SaturationMax = smax
                    self.HSVColorRange[label].BrightnessMin = vmin
                    self.HSVColorRange[label].BrightnessMax = vmax

            self.get_logger().info(f"[HSV INI] loaded from: {ini_path}")

        # 3) 決定哪個 label 要作為當前色並回填 lower/upper
        active = active_label if active_label else self.select_color
        if active not in self.labels:
            # 若傳入的 active_label 不在名單，用原本 select_color；再不行就用第一個
            active = self.select_color if self.select_color in self.labels else self.labels[0]

        self.select_color = active
        cd = self.HSVColorRange[active]
        self.lower = np.array([int(cd.HueMin), int(cd.SaturationMin), int(cd.BrightnessMin)], dtype=np.uint8)
        self.upper = np.array([int(cd.HueMax), int(cd.SaturationMax), int(cd.BrightnessMax)], dtype=np.uint8)

        self.get_logger().info(
            f"[HSV INI] active '{active}' -> "
            f"H[{cd.HueMin},{cd.HueMax}] S[{cd.SaturationMin},{cd.SaturationMax}] "
            f"V[{cd.BrightnessMin},{cd.BrightnessMax}]"
        )

    def load_zoomin_from_camera_ini(self):
        # zoomin 依相機來源分家：usb 由 usb_cam 寫 CameraSet.ini，
        # zed 由 camera_param_bridge_node 寫 ZedCameraSet.ini，兩者互斥啟動不會打架。
        # zed 模式下 ZedCameraSet.ini 若尚未建立，退回讀 CameraSet.ini，
        # 讓既有的 zoom 設定能自動沿用（第一次 Camera Save 後就會有自己的檔）
        if self.camera_source == "usb":
            candidates = [("CameraSet.ini", "Camera Set Parameter")]
        else:
            candidates = [
                ("ZedCameraSet.ini", "ZED Camera Set Parameter"),
                ("CameraSet.ini", "Camera Set Parameter"),
            ]

        base = Path(self.location) if getattr(self, "location", None) else Path(self.path_dir)

        for filename, section in candidates:
            ini_path = base if base.suffix else (base / filename)

            cfg = configparser.ConfigParser()
            if not cfg.read(str(ini_path)):
                self.get_logger().warn(f"[{filename}] not found: {ini_path}")
                continue
            if section not in cfg:
                self.get_logger().warn(f"[{filename}] section '{section}' missing")
                continue
            if "zoomin" not in cfg[section]:
                self.get_logger().warn(f"[{filename}] key 'zoomin' missing")
                continue

            try:
                self.zoom = float(cfg[section]["zoomin"])
                self.get_logger().info(f"[{filename}] zoom={self.zoom} from {ini_path}")
                return
            except ValueError as e:
                self.get_logger().error(f"[{filename}] invalid zoomin value: {e}")

        self.get_logger().warn(f"[zoomin] no usable ini found, keep zoom={self.zoom}")

    # def init_yuv_from_ini(self, active_label: str = None):
    #     ini_path = str(self._resolve_yuv_path())
    #     cfg = configparser.ConfigParser()

    #     # 1) 檢查並讀入檔案
    #     read_ok = cfg.read(ini_path)
    #     if not read_ok:
    #         # 沒檔 -> 建立模板 (不改變記憶體中的數值，只是寫檔)
            
    #         Path(ini_path).parent.mkdir(parents=True, exist_ok=True)
    #         for label in self.labels:
    #             # 這裡給「全通」作為模板，方便第一次看到效果；你也可以改成 0~1
    #             cfg[label] = {
    #                 "y_min": "0",   "y_max": "255",
    #                 "cr_min": "0",   "cr_max": "255",
    #                 "cb_min": "0",   "cb_max": "255",
    #             }
    #         try:
    #             with open(ini_path, "w") as f:
    #                 cfg.write(f)
    #             self.get_logger().info(f"[YUV INI] created template: {ini_path}")
    #         except Exception as e:
    #             self.get_logger().error(f"[YUV INI] failed to create template: {e}")
    #     else:
    #         # 2) 有檔 -> 逐 label 載入到記憶體
    #         for label in self.labels:
    #             if label in cfg:
    #                 sec = cfg[label]
    #                 ymin = self._clamp(sec.get("y_min",         0),   0, 255)
    #                 ymax  = self._clamp(sec.get("y_max",       255),   0, 255)
    #                 crmin  = self._clamp(sec.get("cr_min",  0),   0, 255)
    #                 crmax  = self._clamp(sec.get("cr_max",255),   0, 255)
    #                 cbmin = self._clamp(sec.get("cb_min",  0),   0, 255)
    #                 cbmax  = self._clamp(sec.get("cb_max",255),   0, 255)

    #                 self.YUVColorRange[label].YMin   = ymin 
    #                 self.YUVColorRange[label].YMax   = ymax 
    #                 self.YUVColorRange[label].CRMin  = crmin
    #                 self.YUVColorRange[label].CRMax  = crmax 
    #                 self.YUVColorRange[label].CBMin  = cbmin
    #                 self.YUVColorRange[label].CBMax  = cbmax 

    #         self.get_logger().info(f"[YUV INI] loaded from: {ini_path}")

    #     # 3) 決定哪個 label 要作為當前色並回填 lower/upper
    #     active = active_label if active_label else self.yuv_select_color
    #     if active not in self.labels:
    #         # 若傳入的 active_label 不在名單，用原本 select_color；再不行就用第一個
    #         active = self.yuv_select_color if self.yuv_select_color in self.labels else self.labels[0]

    #     self.yuv_select_color = active
    #     cd = self.YUVColorRange[active]
    #     self.yuv_lower = np.array([int(cd.YMin ), int(cd.CRMin ), int(cd.CBMin)], dtype=np.uint8)
    #     self.yuv_upper = np.array([int(cd.YMax ), int(cd.CRMax ), int(cd.CBMax )], dtype=np.uint8)
    #     self.get_logger().info(
    #         f"[YUV INI] active '{active}' -> "
    #         f"Y[{cd.YMin },{cd.YMax }] CR[{cd.CRMin },{cd.CRMax }] "
    #         f"CB[{cd.CBMin},{cd.CBMax }]"
    #     )

        
    def init_opencv_from_ini(self):
        # self.OpenCvOrders = ["ERODE:3", "DILATE:5"]
        self._morph_kernel = None      # 強制下次 refresh 重建
        self.refresh_opencv_orders()

    def refresh_opencv_orders(self):
        """依 morph_kernel 參數重建處理順序，kernel 沒變就不做事（每幀呼叫）。"""
        k = int(self.get_parameter('morph_kernel').value)
        if k == self._morph_kernel:
            return
        self._morph_kernel = k
        self.OpenCvOrders = [f"OPEN:{k}"] if k >= 3 else []
        self.get_logger().info(f"[OpenCV] orders -> {self.OpenCvOrders}")

        # ini_path = str(self._resolve_opencv_path())
        # cfg = configparser.ConfigParser()

        # read_ok = cfg.read(ini_path)
        # if not read_ok:
        #     # 沒檔就建一個空的，跟 HSV/YUV 一樣的寫法
        #     cfg["OpenCvOrder"] = {"order": ""}
        #     try:
        #         with open(ini_path, "w") as f:
        #             cfg.write(f)
        #         self.get_logger().info(f"[OpenCV INI] created empty: {ini_path}")
        #     except Exception as e:
        #         self.get_logger().error(f"[OpenCV INI] create failed: {e}")
        #     # 記憶體維持現在的 self.OpenCvOrders，不動或清空都可以
        #     self.OpenCvOrders = []
        #     return

        # # 有檔案就載進記憶體
        # sec = cfg["OpenCvOrder"] if "OpenCvOrder" in cfg else None
        # if sec:
        #     raw = sec.get("order", "")
        #     if raw.strip():
        #         self.OpenCvOrders = [s.strip() for s in raw.split(",") if s.strip()]
        #     else:
        #         self.OpenCvOrders = []
        # else:
        #     self.OpenCvOrders = []

        # self.get_logger().info(f"[OpenCV INI] loaded from {ini_path}: {self.OpenCvOrders}")
        # self.publish_opencv_order()

    
    def save_hsv_callback(self, request, response):
        try:
            # 確保資料夾存在
            ini_path = self._resolve_hsv_path()
            ini_path.parent.mkdir(parents=True, exist_ok=True)

            config = configparser.ConfigParser()

            for label, data in self.HSVColorRange.items():
                config[label] = {
                    "hue_max":        str(int(round(float(data.HueMax)))),
                    "hue_min":        str(int(round(float(data.HueMin)))),
                    "saturation_max": str(int(round(float(data.SaturationMax)))),
                    "saturation_min": str(int(round(float(data.SaturationMin)))),
                    "brightness_max": str(int(round(float(data.BrightnessMax)))),
                    "brightness_min": str(int(round(float(data.BrightnessMin))))
                }

            with open(ini_path, "w") as configfile:
                config.write(configfile)
            self.path = str(ini_path)
            self.get_logger().info(f"[HSV INI] saved: {self.path}")
            response.already = True
        except Exception as e:
            self.get_logger().error(f"[HSV INI] save failed: {e}")
            response.already = False

        return response
    
    # def save_yuv_callback(self, request, response):
    #     try:
    #         # 確保資料夾存在
    #         ini_path = self._resolve_yuv_path()
    #         ini_path.parent.mkdir(parents=True, exist_ok=True)

    #         config = configparser.ConfigParser()

    #         for label, data in self.YUVColorRange.items():
    #             config[label] = {
    #                 "y_max":        str(int(round(float(data.YMax )))),
    #                 "y_min":        str(int(round(float(data.YMin )))),
    #                 "cr_max": str(int(round(float(data.CRMax )))),
    #                 "cr_min": str(int(round(float(data.CRMin)))),
    #                 "cb_max": str(int(round(float(data.CBMax )))),
    #                 "cb_min": str(int(round(float(data.CBMin))))
    #             }

    #         with open(ini_path, "w") as configfile:
    #             config.write(configfile)
    #         self.yuv_path = str(ini_path)
    #         self.get_logger().info(f"[YUV INI] saved: {self.yuv_path}")
    #         response.already = True
    #     except Exception as e:
    #         self.get_logger().error(f"[YUV INI] save failed: {e}")
    #         response.already = False

    #     return response
    
    # def save_opencv_callback(self, request, response):
    #     try:
    #         ini_path = self._resolve_opencv_path()
    #         cfg = configparser.ConfigParser()
    #         cfg["OpenCvOrder"] = {
    #             "order": ",".join(self.OpenCvOrders)
    #         }
    #         with open(ini_path, "w") as f:
    #             cfg.write(f)
    #         self.get_logger().info(f"[OpenCV INI] saved to {ini_path}: {self.OpenCvOrders}")
    #         self.publish_opencv_order()
    #         response.already = True
    #     except Exception as e:
    #         self.get_logger().error(f"[OpenCV INI] save failed: {e}")
    #         response.already = False
    #     return response

    
    def image_callback(self, msg):
        try:
            self.refresh_opencv_orders()
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # usb 模式先補黑成 960x600，之後所有邏輯都當作只有一種尺寸
            cv_img = self._pad_frame(cv_img)
            height, width = cv_img.shape[:2]

            new_w = int(width / self.zoom)
            new_h = int(height / self.zoom)
            x1 = (width - new_w) // 2
            y1 = (height - new_h) // 2
            x2 = x1 + new_w
            y2 = y1 + new_h

            # 補黑區在放大後落在哪幾列，色模要據此排除
            self._update_pad_rows(y1, new_h, height)

            cropped = cv_img[y1:y2, x1:x2]
            zoomed_frame = cv2.resize(cropped, (width, height), interpolation=cv2.INTER_LINEAR)
            # zoomed_frame = cv2.morphologyEx(zoomed_frame, cv2.MORPH_BLACKHAT, self.kernel3)        
            zoomed_frame = self.process_by_order(zoomed_frame, self.OpenCvOrders,"b")
            now = time.time()
            
            draw_fram = zoomed_frame.copy()
            for draw_item in self.draw_image_array:
                color = (int(draw_item.bvalue), int(draw_item.gvalue), int(draw_item.rvalue))
                pt1 = (int(draw_item.xmin), int(draw_item.ymin))
                pt2 = (int(draw_item.xmax), int(draw_item.ymax))
                try:
                    draw_item.xmin = int(draw_item.xmin)
                    draw_item.ymin = int(draw_item.ymin)
                    draw_item.xmax = int(draw_item.xmax)
                    draw_item.ymax = int(draw_item.ymax)
                    draw_item.thickness = int(draw_item.thickness)
                    if draw_item.mode == 1: # 假設 1 是 etDrawLine
                        cv2.line(draw_fram, pt1, pt2, color, draw_item.thickness)
                    elif draw_item.mode == 2: # 假設 2 是 etDrawObject (Rectangle)
                        cv2.rectangle(draw_fram, pt1, pt2, color, draw_item.thickness)
                    elif draw_item.mode == 3:
                        pt1 = (int(draw_item.xmin),int(draw_item.ymin))
                        pt2 = (int(draw_item.xmax))
                        cv2.circle(draw_fram, pt1, pt2, color, draw_item.thickness)
                except Exception as e:
                    self.get_logger().error(f"Drawing error: {e}")
            
            if now - self._last_img_pub["zoom_in"] >= self._img_period:
                zoom_msg = self.bridge.cv2_to_imgmsg(draw_fram, encoding='bgr8')
                try:
                    zoom_msg.header.stamp.sec = int(msg.header.stamp.sec)
                    zoom_msg.header.stamp.nanosec = int(msg.header.stamp.nanosec)
                except Exception:
                    pass
                self.zoom_in.publish(zoom_msg)
                self._last_img_pub["zoom_in"] = now

            # 色模直接在放大後的原尺寸（960x600）上運算，與深度圖同一座標系
            proc_frame = zoomed_frame
            hsv_proc = cv2.cvtColor(proc_frame, cv2.COLOR_BGR2HSV)
            # yuv_proc = cv2.cvtColor(proc_frame, cv2.COLOR_BGR2YCrCb)

            if self.build_status == 1:                
                # 4) 單色 build（僅在 lower/upper 已設定時跑；保持你原本邏輯）
                if self.lower is not None and self.upper is not None:
                    self.build_hsv_table(hsv_proc, proc_frame)

            elif self.build_status == 0:
                # 5) 取 header 時間戳，丟給多色偵測（節流＋去抖在函式內處理）
                
                stamp = {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec}
                self.build_all_hsv_table(hsv_proc, proc_frame, stamp)

            # if self.yuv_build_status == 1:  
            #     # 4) 單色 build（僅在 lower/upper 已設定時跑；保持你原本邏輯）
            #     if self.yuv_lower is not None and self.yuv_upper is not None:
            #         self.build_yuv_table(yuv_proc, proc_frame)

            # elif self.yuv_build_status == 0:
            #     # 5) 取 header 時間戳，丟給多色偵測（節流＋去抖在函式內處理）
            #     stamp = {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec}
            #     self.build_all_yuv_table(yuv_proc, proc_frame, stamp)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def build_hsv_table(self, hsv, resized):
        H_MAX, S_MAX, V_MAX = 179, 255, 255

        h_low, s_low, v_low = map(int, self.lower)
        h_high, s_high, v_high = map(int, self.upper)

        # --- 特殊情況：全0 → 空選取；全滿 → 全選取 ---
        if (h_low, s_low, v_low) == (0, 0, 0) and (h_high, s_high, v_high) == (0, 0, 0):
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        elif (h_low, s_low, v_low) == (0, 0, 0) and (h_high, s_high, v_high) == (H_MAX, S_MAX, V_MAX):
            mask = np.full(hsv.shape[:2], 255, dtype=np.uint8)
        elif (h_low, s_low, v_low) == (H_MAX, S_MAX, V_MAX) and (h_high, s_high, v_high) == (H_MAX, S_MAX, V_MAX):
            # 兼容：若上下限都被拉到最大，也視為全選
            mask = np.full(hsv.shape[:2], 255, dtype=np.uint8)
        else:
            # --- 一般情況：沿用你原本的區間與跨零點邏輯 ---
            if h_low <= h_high:
                mask = cv2.inRange(
                    hsv,
                    (h_low, s_low, v_low),
                    (h_high, s_high, v_high)
                )
            else:
                mask1 = cv2.inRange(hsv, (0, s_low, v_low), (h_high, s_high, v_high))
                mask2 = cv2.inRange(hsv, (h_low, s_low, v_low), (H_MAX, s_high, v_high))
                mask = cv2.bitwise_or(mask1, mask2)

            # 形態學清雜訊（只在一般情況下做，避免把「全選」變稀疏）
            mask = self.process_by_order(mask, self.OpenCvOrders,"a")
            # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel3, iterations=2)

        # usb 補黑區不算物體（放在特殊情況之後，「全選」也要一併排除）
        mask = self._strip_pad(mask)

        # 後處理與發佈：保持你原本的流程
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')

        key = f"{self.select_color.capitalize()}Label"
        if key in self.color_labels:
            b, g, r = self.color_labels[key]['color']
            color_img = np.zeros_like(resized)
            color_img[:] = (b, g, r)

            fg = cv2.bitwise_and(color_img, color_img, mask=mask)
            inv = cv2.bitwise_not(mask)
            bg = cv2.bitwise_and(resized, resized, mask=inv)
            composed = cv2.add(bg, fg)

            color_msg = self.bridge.cv2_to_imgmsg(composed, encoding='bgr8')
            self.processed_image.publish(color_msg)
        else:
            self.processed_image.publish(self.bridge.cv2_to_imgmsg(resized, encoding='bgr8'))

        vis_msg = self.bridge.cv2_to_imgmsg(resized, encoding='bgr8')
        self.label_pub.publish(mask_msg)
        return vis_msg, mask_msg
    # def build_yuv_table(self, yuv, resized):
    #     Y_MAX, CR_MAX, CB_MAX = 255, 255, 255

    #     y_low, cr_low, cb_low = map(int, self.yuv_lower)
    #     y_high, cr_high, cb_high = map(int, self.yuv_upper)

    #     # --- 特殊情況：全0 → 空選取；全滿 → 全選取 ---
    #     if (y_low, cr_low, cb_low) == (0, 0, 0) and (y_high, cr_high, cb_high) == (0, 0, 0):
    #         mask = np.zeros(yuv.shape[:2], dtype=np.uint8)
    #     elif (y_low, cr_low, cb_low) == (0, 0, 0) and (y_high, cr_high, cb_high) == (Y_MAX, CR_MAX, CB_MAX):
    #         mask = np.full(yuv.shape[:2], 255, dtype=np.uint8)
    #     elif (y_low, cr_low, cb_low) == (Y_MAX, CR_MAX, CB_MAX) and (y_high, cr_high, cb_high) == (Y_MAX, CR_MAX, CB_MAX):
    #         # 兼容：若上下限都被拉到最大，也視為全選
    #         mask = np.full(yuv.shape[:2], 255, dtype=np.uint8)
    #     else:
    #         # --- 一般情況：沿用你原本的區間與跨零點邏輯 ---

    #         mask = cv2.inRange(
    #             yuv,
    #             (y_low, cr_low, cb_low),
    #             (y_high, cr_high, cb_high)
    #         )


    #         # 形態學清雜訊（只在一般情況下做，避免把「全選」變稀疏）
    #         mask = self.process_by_order(mask, self.OpenCvOrders,"a")
    #         # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel3, iterations=2)

    #     # 後處理與發佈：保持你原本的流程
    #     mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')

    #     key = f"{self.yuv_select_color.capitalize()}Label"
    #     if key in self.color_labels:
    #         b, g, r = self.color_labels[key]['color']
    #         color_img = np.zeros_like(resized)
    #         color_img[:] = (b, g, r)

    #         fg = cv2.bitwise_and(color_img, color_img, mask=mask)
    #         inv = cv2.bitwise_not(mask)
    #         bg = cv2.bitwise_and(resized, resized, mask=inv)
    #         composed = cv2.add(bg, fg)

    #         color_msg = self.bridge.cv2_to_imgmsg(composed, encoding='bgr8')
    #         self.yuv_processed_image.publish(color_msg)
    #     else:
    #         self.yuv_processed_image.publish(self.bridge.cv2_to_imgmsg(resized, encoding='bgr8'))

    #     vis_msg = self.bridge.cv2_to_imgmsg(resized, encoding='bgr8')
    #     self.yuv_label_pub.publish(mask_msg)
    #     return vis_msg, mask_msg


    # ---------------- usb 相機的補黑（pad）處理 ----------------
    def _pad_frame(self, img):
        """usb 相機的 960x540 上下補黑，補成與 ZED 相同的 960x600。

        統一尺寸之後，裁切放大邏輯、對外座標、網頁顯示框都不用分兩套。
        zed 模式或高度本來就達標時原樣返回，不產生任何複製成本。
        """
        self._pad_top = self._pad_bottom = 0
        if self.camera_source != 'usb':
            return img

        target_h = int(self.get_parameter('pad_to_height').value)
        h = img.shape[0]
        if h >= target_h:
            return img

        top = (target_h - h) // 2
        bottom = target_h - h - top
        self._pad_top, self._pad_bottom = top, bottom
        return cv2.copyMakeBorder(img, top, bottom, 0, 0,
                                  cv2.BORDER_CONSTANT, value=(0, 0, 0))

    def _update_pad_rows(self, y1, new_h, out_h):
        """算出補黑區在「裁切放大後」的影像裡佔哪幾列，供色模排除。

        裁切是置中的，所以只要 zoom 超過 600/540≈1.11，補黑區就整個被裁掉了，
        這時算出來會是 0 —— 也就是說多數放大情況下根本不需要排除。
        """
        self._pad_out_top = 0
        self._pad_out_bot_start = None
        if self._pad_top <= 0 and self._pad_bottom <= 0:
            return

        scale = out_h / float(max(new_h, 1))      # 輸入 1 列 → 輸出幾列

        if self._pad_top > 0:
            top = math.ceil((self._pad_top - y1) * scale)
            self._pad_out_top = max(0, min(out_h, int(top)))

        if self._pad_bottom > 0:
            # 輸入座標中，補黑區從這一列開始（out_h 同時也是補黑後的輸入高度）
            bot_in = out_h - self._pad_bottom
            start = math.floor((bot_in - y1) * scale)
            start = max(0, min(out_h, int(start)))
            if start < out_h:
                self._pad_out_bot_start = start

    def _strip_pad(self, mask):
        """把補黑區那幾列從 mask 清成 0。

        補黑區是人工加上去的，不是真的畫面。960x30 = 28800 px 遠大於
        min_blob_area(375)，不清掉的話上下會各多一個橫跨整個畫面寬度的假物體，
        而且永遠都在 —— 策略挑「面積最大的物體」時一定會中招。
        """
        if self._pad_out_top > 0:
            mask[:self._pad_out_top, :] = 0
        if self._pad_out_bot_start is not None:
            mask[self._pad_out_bot_start:, :] = 0
        return mask

    def _label_fill(self, label_id: int, h: int, w: int) -> np.ndarray:
        """取得填滿 label_id 的 mono8 常數圖，供 cv2.copyTo 當來源用（快取，尺寸變才重配）。"""
        key = (label_id, h, w)
        buf = self._label_fill_cache.get(key)
        if buf is None:
            buf = np.full((h, w), label_id, dtype=np.uint8)
            self._label_fill_cache[key] = buf
        return buf

    def build_all_hsv_table(self, hsv, resized, stamp):
        h, w = hsv.shape[:2]
        total_mask = np.zeros((h, w), dtype=np.uint8)     # 純黑白二值化
        label_map = np.zeros((h, w), dtype=np.uint8)      # 0=背景，1~8=顏色 class 編號
        # 偽彩色不在迴圈內逐色填，改在最後用 label_map 查表產生（省掉 8 次三通道 fancy indexing）

        # 全色偵測結果（精簡）
        detections_all = {label: [] for label in self.HSVColorRange.keys()}
        H_MAX, S_MAX, V_MAX = 179, 255, 255

        # 每幀讀一次，讓 ros2 param set 能即時生效
        min_area = int(self.get_parameter('min_blob_area').value)

        for label, color_obj in self.HSVColorRange.items():
            h_low, h_high = int(color_obj.HueMin), int(color_obj.HueMax)
            s_low, s_high = int(color_obj.SaturationMin), int(color_obj.SaturationMax)
            v_low, v_high = int(color_obj.BrightnessMin), int(color_obj.BrightnessMax)

        # 沒設定就跳過
            if (h_low, h_high, s_low, s_high, v_low, v_high) == (0, 0, 0, 0, 0, 0):
                continue
            # Hue 跨 0 度處理
            # mask_i = cv2.inRange(hsv, (y_low, u_low, v_low), (y_high, u_high, v_high))
            if h_low <= h_high:
                mask_i = cv2.inRange(
                    hsv,
                    (h_low, s_low, v_low),
                    (h_high, s_high, v_high)
                )
            else:
                # 兩段接起來
                mask1 = cv2.inRange(hsv, (0,    s_low, v_low), (h_high, s_high, v_high))
                mask2 = cv2.inRange(hsv, (h_low, s_low, v_low), (H_MAX,  s_high, v_high))
                mask_i = cv2.bitwise_or(mask1, mask2)          
            mask_i = self.process_by_order(mask_i, self.OpenCvOrders,"a")
            # mask_i = cv2.morphologyEx(mask_i, cv2.MORPH_OPEN, self.kernel3, iterations=2)
            mask_i = self._strip_pad(mask_i)   # usb 補黑區不算物體

            total_mask = cv2.bitwise_or(total_mask, mask_i)

            label_key = label.capitalize() + "Label"
            entry = self.color_labels.get(label_key, {"label": 0, "color": [255, 255, 255]})
            # 用 OpenCV 的遮罩複製取代 numpy fancy indexing，兩者語意相同但快一個數量級
            cv2.copyTo(self._label_fill(int(entry.get("label", 0)), h, w), mask_i, label_map)

            # vis_msg_all = self.bridge.cv2_to_imgmsg(color_mask, encoding='bgr8')
            # self.processed_image.publish(vis_msg_all)
            # ====== 連通元件（比 findContours 更省） ======
            num, labels_cc, stats, centroids = cv2.connectedComponentsWithStats(mask_i, connectivity=8)
            color_list_compact = []
            for i in range(1, num):  # 0 是背景
                x, y, w_box, h_box, area = stats[i]
                if area < min_area:
                    continue
                cx, cy = centroids[i]
                item = {
                    "bbox": (int(x), int(y), int(w_box), int(h_box)),
                    "centroid": (int(cx), int(cy)),
                    "area": float(area),
                    "aspect_ratio": float(w_box) / float(h_box) if h_box > 0 else 0.0,
                    "label": label
                }
                color_list_compact.append(item)
                detections_all[label].append(item)

            # ====== 每色各發一則 JSON（節流 + 去抖） ======
            msg_color = {
                "stamp": stamp,
                "width": w, "height": h,
                "label": label,
                "objects": color_list_compact
            }
            payload = json.dumps(msg_color, separators=(',', ':'))
            now = time.time()
            key = f"det_{label}"
            if payload != self._last_payload[label] and (now - self._last_pub_t[key] >= self._pub_period):
                try:
                    self.det_pubs[label].publish(String(data=payload))
                    self._last_payload[label] = payload
                    self._last_pub_t[key] = now
                except Exception as e:
                    self.get_logger().warning(f"publish detections/{label} failed: {e}")

        # === 發佈總表 object_info（節流 + 去抖） ===
        detections_all["_stamp"] = stamp
        payload_all = json.dumps(detections_all, separators=(',', ':'))
        now = time.time()
        if payload_all != self._last_info_payload and (now - self._last_pub_t["info"] >= self._pub_period):
            try:
                self.info_pub.publish(String(data=payload_all))
                self._last_info_payload = payload_all
                self._last_pub_t["info"] = now
            except Exception as e:
                self.get_logger().warning(f"publish object_info failed: {e}")

        # === 發佈 total_mask 為 mono8 Image（影像節流） ===
        img_msg = self.bridge.cv2_to_imgmsg(total_mask, encoding='mono8')
        try:
            img_msg.header.stamp.sec = int(stamp.get('sec', 0))
            img_msg.header.stamp.nanosec = int(stamp.get('nanosec', 0))
        except Exception:
            pass
        now = time.time()
        if now - self._last_hsv_img_pub["mask_image"] >= self._img_period:
            self.label_pub.publish(img_msg)
            self._last_hsv_img_pub["mask_image"] = now

            # === 顏色 class 標籤圖，供 overlap_node 與距離標籤圖取交集 ===
            lbl_msg = self.bridge.cv2_to_imgmsg(label_map, encoding='mono8')
            lbl_msg.header.stamp = img_msg.header.stamp
            self.color_label_pub.publish(lbl_msg)

        # === 彩色可視化影像：由 label_map 查表產生（背景 label 0 對到黑，等同原本套 total_mask） ===
        vis_all = cv2.LUT(cv2.cvtColor(label_map, cv2.COLOR_GRAY2BGR), self.color_lut)
        vis_msg_all = self.bridge.cv2_to_imgmsg(vis_all, encoding='bgr8')
        try:
            vis_msg_all.header.stamp.sec = int(stamp.get('sec', 0))
            vis_msg_all.header.stamp.nanosec = int(stamp.get('nanosec', 0))
        except Exception:
            pass
        now = time.time()
        if now - self._last_hsv_img_pub["build_image"] >= self._img_period:
            self.processed_image.publish(vis_msg_all)
            self._last_hsv_img_pub["build_image"] = now

    # def build_all_yuv_table(self, yuv, resized, stamp):
    #     h, w = yuv.shape[:2]
    #     total_mask = np.zeros((h, w), dtype=np.uint8)     # 純黑白二值化
    #     color_mask = np.zeros((h, w, 3), dtype=np.uint8)  # BGR 偽彩色輸出

    #     # 全色偵測結果（精簡）
    #     detections_all = {label: [] for label in self.YUVColorRange.keys()}

    #     # === 逐色門檻 + 形態學 + 蒐集結果 ===
    #     for label, color_obj in self.YUVColorRange.items():
    #         y_low, y_high = int(color_obj.YMin ), int(color_obj.YMax  )
    #         cr_low, cr_high = int(color_obj.CRMin ), int(color_obj.CRMax)
    #         cb_low, cb_high = int(color_obj.CBMin), int(color_obj.CBMax )

    #         # 空設定直接跳過
    #         if (y_low, y_high, cr_low, cr_high, cb_low, cb_high) == (0, 0, 128, 128, 128, 128):
    #             continue

    #         mask_i = cv2.inRange(
    #             yuv,
    #             (y_low, cr_low, cb_low),
    #             (y_high, cr_high, cb_high)
    #         )

    #         # 形態學開運算去雜訊（只建一次的 kernel：self.kernel3）
    #         mask_i = self.process_by_order(mask_i, self.OpenCvOrders,"a")
    #         # mask_i = cv2.morphologyEx(mask_i, cv2.MORPH_OPEN, self.kernel3, iterations=2)

    #         # 累積總 mask（用清理後的 mask_i）
    #         total_mask = cv2.bitwise_or(total_mask, mask_i)

    #         # 偽彩色（用清理後的 mask_i）
    #         label_key = label.capitalize() + "Label"
    #         bgr_color = np.array(self.color_labels.get(label_key, {"color": [255, 255, 255]})["color"],
    #                             dtype=np.uint8)
    #         color_mask[mask_i > 0] = bgr_color
    #         # vis_msg_all = self.bridge.cv2_to_imgmsg(color_mask, encoding='bgr8')
    #         # self.yuv_processed_image.publish(vis_msg_all)
    #         # ====== 連通元件（比 findContours 更省） ======
    #         num, labels_cc, stats, centroids = cv2.connectedComponentsWithStats(mask_i, connectivity=8)
    #         color_list_compact = []
    #         for i in range(1, num):  # 0 是背景
    #             x, y, w_box, h_box, area = stats[i]
    #             if area < 50:
    #                 continue
    #             cx, cy = centroids[i]
    #             item = {
    #                 "bbox": (int(x), int(y), int(w_box), int(h_box)),
    #                 "centroid": (int(cx), int(cy)),
    #                 "area": float(area),
    #                 "aspect_ratio": float(w_box) / float(h_box) if h_box > 0 else 0.0,
    #                 "label": label
    #             }
    #             color_list_compact.append(item)
    #             detections_all[label].append(item)

    #         # ====== 每色各發一則 JSON（節流 + 去抖） ======
    #         msg_color = {
    #             "stamp": stamp,
    #             "width": w, "height": h,
    #             "label": label,
    #             "objects": color_list_compact
    #         }
    #         payload = json.dumps(msg_color, separators=(',', ':'))
    #         now = time.time()
    #         key = f"det_{label}"
    #         if payload != self._yuv_last_payload[label] and (now - self._yuv_last_pub_t[key] >= self._pub_period):
    #             try:
    #                 self.yuv_det_pubs[label].publish(String(data=payload))
    #                 self._yuv_last_payload[label] = payload
    #                 self._yuv_last_pub_t[key] = now
    #             except Exception as e:
    #                 self.get_logger().warning(f"publish detections/{label} failed: {e}")

    #     # === 發佈總表 object_info（節流 + 去抖） ===
    #     detections_all["_stamp"] = stamp
    #     payload_all = json.dumps(detections_all, separators=(',', ':'))
    #     now = time.time()
    #     if payload_all != self._yuv_last_info_payload and (now - self._yuv_last_pub_t["info"] >= self._pub_period):
    #         try:
    #             self.yuv_info_pub.publish(String(data=payload_all))
    #             self._yuv_last_info_payload = payload_all
    #             self._yuv_last_pub_t["info"] = now
    #         except Exception as e:
    #             self.get_logger().warning(f"publish object_info failed: {e}")

    #     # === 發佈 total_mask 為 mono8 Image（影像節流） ===
    #     img_msg = self.bridge.cv2_to_imgmsg(total_mask, encoding='mono8')
    #     try:
    #         img_msg.header.stamp.sec = int(stamp.get('sec', 0))
    #         img_msg.header.stamp.nanosec = int(stamp.get('nanosec', 0))
    #     except Exception:
    #         pass
    #     now = time.time()
    #     if now - self._last_yuv_img_pub["mask_image"] >= self._img_period:
    #         self.yuv_label_pub.publish(img_msg)   # ← 用 YUV 專用 topic
    #         self._last_yuv_img_pub["mask_image"] = now

    #     # === 彩色可視化影像（把 color_mask 套 total_mask，影像節流） ===
    #     vis_all = cv2.bitwise_and(color_mask, color_mask, mask=total_mask)
    #     vis_msg_all = self.bridge.cv2_to_imgmsg(vis_all, encoding='bgr8')
    #     try:
    #         vis_msg_all.header.stamp.sec = int(stamp.get('sec', 0))
    #         vis_msg_all.header.stamp.nanosec = int(stamp.get('nanosec', 0))
    #     except Exception:
    #         pass
    #     now = time.time()
    #     if now - self._last_yuv_img_pub["build_image"] >= self._img_period:
    #         self.yuv_processed_image.publish(vis_msg_all)
    #         self._last_yuv_img_pub["build_image"] = now


def main():
    rclpy.init()
    node = ImageNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()