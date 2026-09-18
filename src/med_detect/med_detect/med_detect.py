#!/usr/bin/env python3
#coding=utf-8
"""藥物尺寸辨識節點。

流程:
    1. 收 YOLO 偵測到的物體 bbox（別的節點發，格式見 YOLO_TOPIC 說明）
    2. 把 bbox 平分成四個區塊
    3. 每個區塊各自從 ZED 深度圖取像素，剔除不合理值後取平均
    4. 四個深度 + bbox 丟進真實大小計算器  <- 別人負責，目前是空殼
    5. 拿算出來的實際長寬去對照表找最相近的，輸出結果

第 4 步還沒實作，所以目前只會輸出 bbox 與四區塊深度。
要怎麼算、需要哪些補償，見檔案最下方的說明。
"""
import configparser
import json
import os
import time

import numpy as np
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage

from strategy.API import API


BASE_DIR = os.path.dirname(os.path.abspath(__file__))

#--- Topic ---#
#YOLO 偵測結果（暫定格式，等對方確定再改）
YOLO_TOPIC                 = '/yolo_detections'
#深度圖，公釐 uint16，由 imageprocess/depth_process_node 發布
DEPTH_TOPIC                = '/depth_mm'
#本節點輸出
RESULT_TOPIC               = '/med_detect/result'

#--- 深度取樣 ---#
#IQR 離群剔除的倍數，1.5 是標準值，調大保留較多
IQR_K                      = 1.5
#一個區塊至少要有這麼多有效像素才算數，否則該區塊回 None
MIN_VALID_PIXELS           = 20

#--- bbox 過濾 ---#
#物體碰到畫面邊緣代表被切掉了，bbox 不完整，直接跳過
REJECT_EDGE_BBOX           = True
#距離邊緣幾個像素內算「碰到邊緣」
EDGE_MARGIN_PX             = 2

#--- 顯示 ---#
#在 Image 網頁上畫出抓到的 bbox 與四區塊分隔線
DRAW_FUNCTION_FLAG         = True

#--- 對照表 ---#
SIZE_TABLE_PATH            = os.path.join(BASE_DIR, 'Parameter', 'size_table.ini')
#長寬誤差超過這個值(公分)就視為對不到
MATCH_TOLERANCE_CM         = 1.5

#--- bbox 來源 ---#
#'color' 色模（YOLO 還沒好時用這個測，抓真實物體）
#'yolo'  等 YOLO 節點接上後改成這個
#'sim'   餵固定的假 bbox
BBOX_SOURCE                = 'color'
#來源是 color / sim 時的取樣週期(秒)
TICK_PERIOD                = 0.5

#--- 色模來源 ---#
#'Orange' 'Yellow' 'Blue' 'Green' 'Black' 'Red' 'White'
TARGET_COLOR               = 'Red'
#色模物體面積小於這個值(像素)視為雜訊。960x600 下 300 約等於 17x17
MIN_COLOR_AREA             = 300

#--- 模擬來源 ---#
SIMULATE_BBOX              = [400, 250, 160, 100]      #[x, y, w, h]
SIMULATE_LABEL             = 'sim_object'


def estimate_real_size(bbox, depths_cm, node=None):
    """真實大小計算器。

    ⚠ 尚未實作，由別人負責。實作時把這個函式的內容換掉就好，
      呼叫端 (MedDetect.process_object) 不需要改。
      演算法與必要的補償見檔案最下方的說明。

    Args:
        bbox (tuple): (xmin, ymin, xmax, ymax)，960x600 座標系，
            與 ``/depth_mm`` 對齊。
        depths_cm (list): 四個區塊的平均深度(公分)，順序為
            左上 / 右上 / 左下 / 右下，有效像素不足的區塊是 None。
        node (MedDetect, optional): 節點本身。需要 IMU 姿態、數位變焦倍率、
            相機參數之類的額外資訊時從這裡取，不必改動函式簽名。

    Returns:
        tuple: (width_cm, height_cm)，物體的實際寬與高(公分)。
        算不出來時回 (None, None)；目前未實作，固定回 (None, None)。
    """
    return None, None


def reject_outliers_iqr(values, k=IQR_K):
    """用四分位距剔除離群值。

    比 ±k 倍標準差穩：平均值本身會被離群值拉走，四分位數不會。
    區塊裡同時有物體前景和背景牆時這點差很多。

    Args:
        values (np.ndarray): 已經濾掉無效值的一維陣列。
        k (float): IQR 倍數。

    Returns:
        np.ndarray: 落在 [Q1 - k*IQR, Q3 + k*IQR] 內的值。
    """
    if values.size == 0:
        return values
    q1, q3 = np.percentile(values, [25, 75])
    iqr = q3 - q1
    if iqr == 0:
        return values
    return values[(values >= q1 - k * iqr) & (values <= q3 + k * iqr)]


def split_bbox(xmin, ymin, xmax, ymax):
    """把 bbox 從中線切成四個區塊。

    Returns:
        list: 四個 (x0, y0, x1, y1)，順序為左上、右上、左下、右下。
              座標是半開區間，可直接拿去切 numpy 陣列。
    """
    xmid = (xmin + xmax) // 2
    ymid = (ymin + ymax) // 2
    return [
        (xmin, ymin, xmid, ymid),      #Q1 左上
        (xmid, ymin, xmax, ymid),      #Q2 右上
        (xmin, ymid, xmid, ymax),      #Q3 左下
        (xmid, ymid, xmax, ymax),      #Q4 右下
    ]


def load_size_table(path=SIZE_TABLE_PATH):
    """讀對照表。

    每個 section 是一個品項，長寬單位公分:

        [item_a]
        length_cm = 12.0
        width_cm  = 8.0

    Returns:
        list: [{'name': str, 'length_cm': float, 'width_cm': float}, ...]
    """
    table = []
    if not os.path.exists(path):
        return table

    config = configparser.ConfigParser()
    config.read(path, encoding='utf-8')
    for name in config.sections():
        try:
            table.append({
                'name': name,
                'length_cm': config.getfloat(name, 'length_cm'),
                'width_cm': config.getfloat(name, 'width_cm'),
            })
        except Exception:
            continue
    return table


def match_size(width_cm, height_cm, table, tolerance=MATCH_TOLERANCE_CM):
    """在對照表裡找最相近的品項。

    兩邊各自排序成 (長邊, 短邊) 再比，這樣物體躺著或立著都對得到同一筆。
    距離用長短邊誤差的歐氏距離。

    Returns:
        tuple: (best, error_cm, within_tolerance)，表是空的時回 (None, None, False)。
    """
    if not table:
        return None, None, False

    long_in, short_in = max(width_cm, height_cm), min(width_cm, height_cm)

    best = None
    best_err = None
    for item in table:
        long_t = max(item['length_cm'], item['width_cm'])
        short_t = min(item['length_cm'], item['width_cm'])
        err = float(np.hypot(long_in - long_t, short_in - short_t))
        if best_err is None or err < best_err:
            best, best_err = item, err

    return best, best_err, best_err <= tolerance


class MedDetect(API):
#國科會計畫 - 藥物尺寸辨識
    def __init__(self):
        super().__init__('med_detect_node')

        #自己留一份深度圖。API 內部也有訂 /depth_mm，但那是私有的，
        #而且 depth_at() 只回鄰域中位數，拿不到整個區塊的像素分佈。
        self.depth_mm = None
        self.create_subscription(
            RosImage, DEPTH_TOPIC, self.depth_mm_callback,
            self.qos_fast, callback_group=self.image_cbg)

        self.create_subscription(
            String, YOLO_TOPIC, self.yolo_callback,
            self.qos_fast, callback_group=self.image_cbg)

        self.result_pub = self.create_publisher(String, RESULT_TOPIC, 10)

        self.size_table = load_size_table()
        if self.size_table:
            self.get_logger().info(f"對照表載入 {len(self.size_table)} 筆: {SIZE_TABLE_PATH}")
        else:
            self.get_logger().warn(f"對照表是空的或不存在: {SIZE_TABLE_PATH}")

        if BBOX_SOURCE == 'color':
            self.target_color = self.COLORS.index(TARGET_COLOR.lower())
            self.tick_timer = self.create_timer(TICK_PERIOD, self.color_tick)
            self.get_logger().warn(f"bbox 來源: 色模 ({TARGET_COLOR})")
        elif BBOX_SOURCE == 'sim':
            self.tick_timer = self.create_timer(TICK_PERIOD, self.simulate_tick)
            self.get_logger().warn("bbox 來源: 模擬，正在餵假的 bbox")
        else:
            self.get_logger().info(f"bbox 來源: YOLO ({YOLO_TOPIC})")

        self.get_logger().info("Med Detect Node Initialized")

    # -------------------- 訂閱回呼 --------------------
    def depth_mm_callback(self, msg: RosImage):
        """快取深度圖。mono16，單位公釐，0 代表無效（上游已把 NaN/inf 歸零）。"""
        try:
            self.depth_mm = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
        except Exception as e:
            self.get_logger().error(f"[depth_mm] convert failed: {e}")

    def yolo_callback(self, msg: String):
        """YOLO 偵測結果。

        暫定格式（沿用本專案 detections 的慣例，等對方確定再改）::

            {"stamp": {"sec": 0, "nanosec": 0},
             "objects": [{"bbox": [x, y, w, h],
                          "label": "...",
                          "confidence": 0.93}]}

        bbox 是 [左上x, 左上y, 寬, 高]，座標系 960x600，與 /depth_mm 對齊。
        """
        try:
            data = json.loads(msg.data)
            objects = data.get('objects', [])
        except Exception as e:
            self.get_logger().error(f"[yolo] parse error: {e}")
            return

        for obj in objects:
            try:
                x, y, w, h = obj['bbox']
            except Exception as e:
                self.get_logger().warn(f"[yolo] malformed object: {e}")
                continue
            self.process_object(
                (int(x), int(y), int(x + w), int(y + h)),
                obj.get('label', ''))

    # -------------------- 主流程 --------------------
    def process_object(self, bbox, label=''):
        """一個物體走完整條流程並發布結果。"""
        if self.depth_mm is None:
            self.get_logger().warn("還沒收到深度圖\033[K")
            return

        if REJECT_EDGE_BBOX and self.touches_edge(bbox):
            self.get_logger().info(f"物體碰到畫面邊緣，bbox 不完整，跳過: {bbox}\033[K")
            return

        if DRAW_FUNCTION_FLAG:
            self.draw_bbox(bbox)

        depths = self.quadrant_depths(bbox)

        self.get_logger().info('________________________________________\033[K')
        self.get_logger().info(f"label: {label}, bbox: {bbox}\033[K")
        self.get_logger().info(
            "四區塊深度(cm): " +
            ", ".join('None' if d is None else f'{d:.1f}' for d in depths) + "\033[K")

        width_cm, height_cm = estimate_real_size(bbox, depths, self)

        if width_cm is None or height_cm is None:
            self.get_logger().info(
                "真實大小計算器尚未實作，只輸出 bbox 與四區塊深度\033[K")
            self.publish_result(label, bbox, depths)
            return

        self.get_logger().info(f"實際尺寸: {width_cm:.2f} x {height_cm:.2f} cm\033[K")

        best, err, ok = match_size(width_cm, height_cm, self.size_table)
        if best is None:
            self.get_logger().warn("對照表是空的，無法比對\033[K")
            self.publish_result(label, bbox, depths, width_cm, height_cm)
            return

        if ok:
            self.get_logger().info(f"✅ 判定: {best['name']} (誤差 {err:.2f} cm)\033[K")
        else:
            self.get_logger().info(
                f"⚠ 最接近 {best['name']} 但誤差 {err:.2f} cm 超過門檻 "
                f"{MATCH_TOLERANCE_CM} cm\033[K")

        self.publish_result(label, bbox, depths, width_cm, height_cm, best, err, ok)

    def touches_edge(self, bbox):
        """bbox 是否碰到畫面邊緣。

        物體被畫面切掉時 bbox 只框到露出來的部分，寬高都偏小，
        算出來的實際尺寸必錯，與其輸出錯的不如不輸出。
        """
        h, w = self.depth_mm.shape[:2]
        xmin, ymin, xmax, ymax = bbox
        m = EDGE_MARGIN_PX
        return (xmin <= m or ymin <= m or xmax >= w - m or ymax >= h - m)

    def quadrant_depths(self, bbox):
        """把 bbox 切四塊，每塊各自剔除不合理值後取平均。

        Returns:
            list: 四個平均深度(公分)，有效像素不足的區塊為 None。
        """
        img = self.depth_mm
        h, w = img.shape[:2]

        xmin, ymin, xmax, ymax = bbox
        #夾回畫面內，YOLO 的框可能超出邊界
        xmin, xmax = max(0, xmin), min(w, xmax)
        ymin, ymax = max(0, ymin), min(h, ymax)
        if xmax - xmin < 2 or ymax - ymin < 2:
            return [None] * 4

        depths = []
        for x0, y0, x1, y1 in split_bbox(xmin, ymin, xmax, ymax):
            roi = img[y0:y1, x0:x1]
            valid = roi[roi > 0].astype(np.float32)      #0 = 無效
            kept = reject_outliers_iqr(valid)
            if kept.size < MIN_VALID_PIXELS:
                depths.append(None)
            else:
                depths.append(float(np.mean(kept)) / 10.0)   #公釐 -> 公分
        return depths

    def draw_bbox(self, bbox):
        """在 Image 網頁上畫出 bbox 與四區塊的十字分隔線。

        ⚠ 發布順序有講究：image.py 的 /drawimage 訂閱是 KEEP_LAST depth=1，
        連續發的多個圖形只要來不及被取走就會被後面的擠掉，愈早發的愈容易掉。
        所以把最重要的外框放到**最後**發，確保它每次都更新得到。
        根治要改 image.py 那邊的佇列深度。
        """
        #drawImageFunction(cnt, mode, xmin, xmax, ymin, ymax, r, g, b, thickness)
        #mode: 1直線 2矩形 3圓形
        xmin, ymin, xmax, ymax = bbox
        xmid = (xmin + xmax) // 2
        ymid = (ymin + ymax) // 2
        self.drawImageFunction(2, 1, xmid, xmid, ymin, ymax, 255, 255, 0, 1)   #垂直中線 黃
        time.sleep(0.01)
        self.drawImageFunction(3, 1, xmin, xmax, ymid, ymid, 255, 255, 0, 1)   #水平中線 黃
        time.sleep(0.01)
        self.drawImageFunction(1, 2, xmin, xmax, ymin, ymax, 0, 255, 0, 2)     #外框 綠
        time.sleep(0.01)

    def publish_result(self, label, bbox, depths,
                       width_cm=None, height_cm=None, best=None, err=None, ok=False):
        """發布結果。

        尺寸算不出來時照樣發 bbox 與四區塊深度 —— 那是本節點負責的部分，
        下游（含之後的計算器）可以直接吃這些。
        """
        out = {
            'label': label,
            'bbox': list(bbox),
            'quadrant_depth_cm': depths,
        }
        if width_cm is not None and height_cm is not None:
            out['width_cm'] = round(width_cm, 2)
            out['height_cm'] = round(height_cm, 2)
        if best is not None:
            out['match'] = best['name']
            out['match_error_cm'] = round(err, 2)
            out['within_tolerance'] = ok

        msg = String()
        msg.data = json.dumps(out, ensure_ascii=False)
        self.result_pub.publish(msg)

    # -------------------- 色模來源 --------------------
    def color_tick(self):
        """拿色模偵測到的目標顏色物體當 bbox，走同一條流程。

        YOLO 還沒接上時的測試路徑。色模的座標系跟 /depth_mm 一樣是 960x600
        （見 imageprocess/image.py 的 build_all_hsv_table），可以直接餵。
        """
        #沒有新的視覺資料就不重算，避免同一幀被處理很多次
        if not self.new_object_info:
            return
        self.new_object_info = False

        color = self.target_color
        count = self.color_counts[color]
        sizes = self.object_sizes[color]

        #挑面積最大的那個
        max_idx = None
        max_area = 0
        for i in range(min(count, len(sizes))):
            if sizes[i] > max_area:
                max_area = sizes[i]
                max_idx = i

        if max_idx is None or max_area < MIN_COLOR_AREA:
            self.get_logger().info(
                f"沒有面積足夠的 {TARGET_COLOR} 物體 (最大 {max_area:.0f} px)\033[K")
            return

        bbox = (self.object_x_min[color][max_idx],
                self.object_y_min[color][max_idx],
                self.object_x_max[color][max_idx],
                self.object_y_max[color][max_idx])
        self.process_object(bbox, f'color:{TARGET_COLOR}')

    # -------------------- 模擬來源 --------------------
    def simulate_tick(self):
        """自己發一筆假的偵測給自己，純粹驗證鏈路。"""
        x, y, w, h = SIMULATE_BBOX
        fake = {
            'stamp': {'sec': 0, 'nanosec': 0},
            'objects': [{'bbox': [x, y, w, h],
                         'label': SIMULATE_LABEL,
                         'confidence': 1.0}],
        }
        msg = String()
        msg.data = json.dumps(fake)
        self.yolo_callback(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MedDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# =============================================================================
# 真實大小計算器 —— 給實作者的說明
# =============================================================================
#
# 目標：由 bbox（像素）+ 四區塊深度（公分）反推物體的實際寬與高（公分）。
# 實作位置：本檔最上方的 estimate_real_size()。
#
# -----------------------------------------------------------------------------
# 1. 基本公式（小孔成像）
# -----------------------------------------------------------------------------
#     實際尺寸 = 像素尺寸 x 距離 / 焦距
#
#         width_cm  = (xmax - xmin) * distance_cm / fx
#         height_cm = (ymax - ymin) * distance_cm / fy
#
#     fx / fy 要分開。影像若經過非等比縮放，兩軸的等效焦距不一樣。
#     實測（ZED、960x600、zoom=1.0）：fx 約 400~450 px。這個範圍是用
#     單一物體反推的，不是正式校正值，請自己重新量。
#
#     校正方法：拿已知尺寸的**平面**物體正對鏡頭，
#         fx = 像素寬 x 距離 / 實際寬
#     不要用圓柱或球校正，理由見第 4 點。
#
#     更好的來源：ZED SDK 本身有相機內參（fx, fy, cx, cy）。但注意那是
#     **原生解析度**的值，要換算到 960x600，而且要再乘上 zoom（見第 3 點）。
#
# -----------------------------------------------------------------------------
# 2. 輸入從哪裡來
# -----------------------------------------------------------------------------
#     bbox              estimate_real_size() 的參數，960x600 座標系
#     四區塊深度         estimate_real_size() 的參數，公分，無效區塊是 None
#     相機姿態           node.zed_imu_abs_rpy = [roll, pitch, yaw]，單位「度」
#                       abs_roll / abs_pitch 以重力為基準可信；
#                       abs_yaw 無絕對基準會漂移，不要用
#                       （來源：walking/zed_imu_node.py，API.py 已接好）
#     數位變焦           訂閱 /Zoom_In_Topic (tku_msgs/Zoom)，欄位 zoomin
#     頭部馬達角度       ⚠ 拿不到。API 只有 head_motor_pub 發送端，沒有位置回授，
#                       只能從自己下過的命令推測，且不含身體傾斜。用 IMU 取代。
#
# -----------------------------------------------------------------------------
# 3. 必要補償 A：數位變焦
# -----------------------------------------------------------------------------
#     imageprocess/image.py 的 image_callback 是「裁成 1/zoom 再放大回原尺寸」，
#     所以等效焦距會等比例放大：
#
#         有效焦距 = fx * zoom
#
#     網頁上調過 zoom 而這裡沒補，尺寸就整個差一個倍率。
#
# -----------------------------------------------------------------------------
# 4. 必要補償 B：深度基準（曲面物體）
# -----------------------------------------------------------------------------
#     深度圖量到的是物體**正面**的距離，但決定投影寬度的是**中心軸**的距離。
#     圓柱/球差一個半徑：
#
#         正確距離 = 量到的深度 + 半徑
#
#     實測影響：46.7 cm 的 10 cm 圓柱，這一項就差 11%。
#     平面物體沒有這個問題，所以校正焦距要用平面。
#
# -----------------------------------------------------------------------------
# 5. 必要補償 C：俯角（垂直方向）
# -----------------------------------------------------------------------------
#     相機俯視時，bbox 會把物體的頂面一起框進去：
#
#         輪廓高 = h * cos(φ) + d * sin(φ)
#
#         h = 真實高度、d = 物體前後厚度、φ = 相機俯角 (node.zed_imu_abs_rpy[1])
#
#     反解 h 需要知道 d，但單視角看不到背面，d 是未知的。
#     實測：假設 d = 量到的寬度，對圓柱/方柱正確；對扁盒（寬 10 厚 2）
#     在 45 度時會把高度低估 80%。所以**不要無條件套用這個反解**。
#
# -----------------------------------------------------------------------------
# 6. 必要補償 D：水平偏轉（這是方形物體誤差的主因）
# -----------------------------------------------------------------------------
#         輪廓寬 = w * cos(θ) + d * sin(θ)        θ = 物體繞自身垂直軸的偏轉角
#
#     10x10 方柱的實測誤差：10 度 +16%、30 度 +37%、45 度 +41%（最糟）、
#     90 度回到 0%。誤差一定偏大，不會偏小。
#
#     圓柱免疫這一項（旋轉對稱，輪廓寬永遠等於直徑），這就是為什麼圓柱看起來準。
#
#     θ 無法從頭部角度得知（物體怎麼擺跟頭轉到哪無關），但可以從四區塊深度估：
#         左半 (Q1+Q3)/2 與右半 (Q2+Q4)/2 的差值 -> 表面水平傾斜
#         上半 (Q1+Q2)/2 與下半 (Q3+Q4)/2 的差值 -> 表面垂直傾斜
#     差值接近 0 表示正對鏡頭，該次量測才可信。這就是切四個區塊真正的用途。
#
# -----------------------------------------------------------------------------
# 7. 其他要注意
# -----------------------------------------------------------------------------
#     roll：|roll| 大時畫面是斜的，軸對齊 bbox 的長寬已經不對應物體的長寬，
#           整個量測都不該信。node.zed_imu_abs_rpy[0]。
#     離軸斜視：物體不在畫面中心時是被斜看的。這個角度用**像素位置**算即可
#           （atan((cx - 影像中心x) / fx)），與頭部角度無關。
#     邊緣：物體被畫面切掉時 bbox 不完整。本節點已用 REJECT_EDGE_BBOX 擋掉。
#
# -----------------------------------------------------------------------------
# 8. 為什麼單純反解會失敗，建議的方向
# -----------------------------------------------------------------------------
#     第 5、6 點的兩條式子裡，w / h 與 d 是耦合的 —— 一個方程式兩個未知數，
#     單視角無解。已經試過「假設 d = 量到的寬度」，對圓柱準、對方形反而比
#     不補償更差。
#
#     比較可行的是**正向模型 + 候選比對**，不要反解：
#         對照表每筆加上厚度 depth_cm，對每個候選品項用已知的 φ、θ
#         算出「如果是這個品項，畫面上應該看到多大的輪廓」，
#         再跟實際量到的輪廓比，挑最接近的。
#     這樣未知數都在候選那邊，不必從單一觀測反解，也天然處理了姿態問題。
#
#     另一個方向是多視角：轉頭或移動機器人從不同角度量同一個物體，
#     把 w 與 d 解開。
#
# -----------------------------------------------------------------------------
# 9. 實測參考數據（2026-09-18，ZED、960x600、zoom=1.0）
# -----------------------------------------------------------------------------
#     10 x 10 cm 圓柱：
#         bbox (480, 192, 566, 281)  ->  86 x 89 px
#         四區塊深度 46.4 / 49.3 / 45.7 / 45.4 cm，平均 46.7 cm
#         由此反推 fx = 86 * 46.7 / 10 = 402 px、fy = 89 * 46.7 / 10 = 416 px
#         （但含第 4 點的圓柱偏差，平面校正應該會得到約 445 px）
# =============================================================================
