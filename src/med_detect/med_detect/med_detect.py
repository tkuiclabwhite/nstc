#!/usr/bin/env python3
#coding=utf-8
"""藥物尺寸辨識節點。

流程:
    1. 收 YOLO 偵測到的物體 bbox（別的節點發，格式見 YOLO_TOPIC 說明）
    2. 把 bbox 內的深度像素反投影成 3D 點雲
    3. 在深度上分群，把背景切掉，只留最近的那一團（物體）
    4. 用 IMU 求重力方向，把點雲轉到「上」是真正的上的座標系
    5. 直接量點雲：重力軸的延伸就是高，水平投影的最小外接矩形就是長寬
    6. 拿量到的尺寸去對照表找最相近的，輸出結果

四區塊深度不再參與尺寸計算，改當可信度閘門 —— 左右/上下的深度差接近 0
才代表物體正對鏡頭、這次量測可信。

為什麼不用「像素尺寸 x 距離 / 焦距」加一堆姿態補償，見檔案最下方的說明。
"""
import configparser
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage
from tku_msgs.msg import Zoom

from strategy.API import API


BASE_DIR = os.path.dirname(os.path.abspath(__file__))

#--- Topic ---#
#YOLO 偵測結果（暫定格式，等對方確定再改）
YOLO_TOPIC                 = '/yolo_detections'
#深度圖，公釐 uint16，由 imageprocess/depth_process_node 發布
DEPTH_TOPIC                = '/depth_mm'
#數位變焦倍率，image.py 也吃同一個 topic
ZOOM_TOPIC                 = '/Zoom_In_Topic'
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

#--- 影像尺寸 ---#
#策略層統一的座標系，/depth_mm 與色模/YOLO 的 bbox 都在這個座標系上
IMAGE_W                    = 960
IMAGE_H                    = 600

#--- 相機內參 ---#
#960x600 座標系下的等效焦距(像素)。
#影像是 16:9 等比縮到 960x540 再上下補黑成 960x600（見 usb_cam/config/params_1.yaml），
#補黑不改變任何一軸的比例，所以 fx 與 fy 相同，只有一個值要量。
#440 是用註解第 9 點的圓柱實測反推的粗估值（含半徑修正），請用平面標定重量：
#    f = 像素寬 x 距離 / 實際寬      <- 平面正對鏡頭，不要用圓柱或球
#更好的來源是 ZED 的 camera_info，但那是原生解析度的值，要自己換算到 960x600。
FOCAL_PX                   = 440.0
#主點。補黑是上下對稱的，所以主點仍在幾何中心。
PRINCIPAL_X                = 480.0
PRINCIPAL_Y                = 300.0
#數位變焦倍率。image.py 是「中心裁成 1/zoom 再放大回原尺寸」，等效焦距等比放大。
#⚠ 若深度圖沒有跟著裁切，zoom != 1 時 bbox 會索引到深度圖的錯誤位置，
#  那不是差一個倍率而是整個對錯地方，必須先確認上游有同步。
ZOOM                       = 1.0

#--- 補黑邊界 ---#
#960x540 上下各補 30 列黑，所以有效影像是第 30~569 列。
#黑邊的深度值是 0（無效），但 touches_edge 必須知道真正的上下緣在哪，
#否則貼著畫面頂端的物體（ymin 約 30）會被誤判成沒碰到邊緣。
LETTERBOX_TOP              = 30
LETTERBOX_BOTTOM           = 30

#--- 點雲 ---#
#深度超出這個範圍(公分)的像素直接丟掉，擋掉 ZED 的飛點與量不到的區域
DEPTH_RANGE_CM             = (10.0, 300.0)
#前景分群：直方圖的格寬(公分)
CLUSTER_BIN_CM             = 1.0
#兩團之間空超過這麼多公分才算斷開，用來把物體跟後面的牆分開
CLUSTER_GAP_CM             = 3.0
#某一格的點數少於最高格的這個比例就當它是空的
CLUSTER_MIN_FRAC           = 0.05
#點雲少於這麼多點就不量，回 None
MIN_CLOUD_POINTS           = 150
#量延伸時的離群值處理。直接取 min/max 會被 ZED 的飛點拉爆（合成測試裡 3% 的
#飛點就能把 12 cm 量成 56 cm），所以先用 IQR 柵欄濾一次，再用 percentile 取延伸。
#⚠ 這兩個值是對著合成的飛點模型調的，上機後請拿真實資料重調。
OUTLIER_IQR_FENCE          = True
EXTENT_TRIM_PCT            = 1.0
#點太多時隨機抽樣到這個數量，minAreaRect 不需要全部的點
MAX_CLOUD_POINTS           = 20000

#--- IMU ---#
#加速度模長與 9.81 差在這個範圍(m/s^2)內才當作靜止，此時加速度計就是重力方向
STATIC_ACCEL_TOL           = 0.35

#--- 可信度閘門 ---#
#四區塊的左右/上下深度差超過這個值(公分)代表物體沒有正對鏡頭。
#3D 量測對偏轉的耐受度比原本的反解好很多，所以這裡只警告、不擋掉。
TILT_WARN_CM               = 3.0

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


#IMU 是 REP-103 body frame（X 前、Y 左、Z 上，ZedImu.msg 註明靜止水平時 az 約 9.81），
#相機是 optical frame（X 右、Y 下、Z 前）。兩者只差一個固定旋轉。
R_OPT_FROM_BODY = np.array([[0.0, -1.0,  0.0],
                            [0.0,  0.0, -1.0],
                            [1.0,  0.0,  0.0]])


def effective_intrinsics(zoom=ZOOM):
    """把內參換算到套用數位變焦之後的值。

    image.py 的變焦是「中心裁成 1/zoom 再放大回原尺寸」，等效焦距等比放大；
    主點要先減掉裁切原點再放大。主點剛好在畫面正中時這一項是恆等變換。

    Returns:
        tuple: (fx, fy, cx, cy)，單位像素，960x600 座標系。
    """
    if zoom is None or zoom <= 0:
        zoom = 1.0
    x0 = (IMAGE_W - IMAGE_W / zoom) / 2.0
    y0 = (IMAGE_H - IMAGE_H / zoom) / 2.0
    return (FOCAL_PX * zoom,
            FOCAL_PX * zoom,
            (PRINCIPAL_X - x0) * zoom,
            (PRINCIPAL_Y - y0) * zoom)


def backproject(depth_mm, bbox, fx, fy, cx, cy):
    """把 bbox 內的深度像素反投影成相機座標系的 3D 點。

    小孔成像的反向：
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        Z = Z

    ⚠ 前提是深度值為「沿光軸的 Z」而非「到相機的徑向距離」。ZED SDK 的
      MEASURE::DEPTH 是前者，但上游若自己轉過就不是了。驗證方法：對著平牆比較
      畫面中心與角落的深度，相同是 Z，往角落遞增是徑向（需先除以
      sqrt(1 + ((u-cx)/fx)^2 + ((v-cy)/fy)^2)）。

    Args:
        depth_mm (np.ndarray): mono16 深度圖，公釐，0 表示無效。
        bbox (tuple): (xmin, ymin, xmax, ymax)，已夾回畫面內。

    Returns:
        np.ndarray: (N, 3) 的點，單位公分。沒有有效像素時是 (0, 3)。
    """
    xmin, ymin, xmax, ymax = bbox
    roi = depth_mm[ymin:ymax, xmin:xmax]
    if roi.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    rows, cols = np.nonzero(roi)                 #0 = 無效，順便當遮罩
    if rows.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    z = roi[rows, cols].astype(np.float32) / 10.0        #公釐 -> 公分
    lo, hi = DEPTH_RANGE_CM
    keep = (z >= lo) & (z <= hi)
    if not np.any(keep):
        return np.empty((0, 3), dtype=np.float32)

    z = z[keep]
    u = (cols[keep] + xmin).astype(np.float32)
    v = (rows[keep] + ymin).astype(np.float32)
    return np.stack([(u - cx) * z / fx,
                     (v - cy) * z / fy,
                     z], axis=1)


def segment_nearest_cluster(points,
                            bin_cm=CLUSTER_BIN_CM,
                            gap_cm=CLUSTER_GAP_CM,
                            min_frac=CLUSTER_MIN_FRAC):
    """在深度直方圖上找最近的一團，把背景切掉。

    比 IQR 可靠：IQR 只砍尾巴，砍不掉雙峰。bbox 裡若一半是物體一半是後面的牆，
    Q1/Q3 會落在兩堆之間，幾乎不剔除任何東西，平均值就停在物體與牆的中點。

    作法是從近到遠掃直方圖，找第一段夠密的連續區間（中間允許 gap_cm 以內的空洞），
    只留落在這個區間裡的點。

    Returns:
        np.ndarray: 屬於最近那一團的點，(M, 3)。
    """
    if points.shape[0] == 0:
        return points

    z = points[:, 2]
    zmin, zmax = float(z.min()), float(z.max())
    span = zmax - zmin
    if span < bin_cm:
        return points

    nbins = int(np.ceil(span / bin_cm))
    hist, edges = np.histogram(z, bins=nbins, range=(zmin, zmin + nbins * bin_cm))
    if hist.max() <= 0:
        return points

    occupied = hist >= max(1.0, hist.max() * min_frac)
    if not np.any(occupied):
        return points

    gap_bins = max(1, int(round(gap_cm / bin_cm)))
    first = int(np.argmax(occupied))              #第一個有東西的格
    last = first
    empty = 0
    for i in range(first + 1, nbins):
        if occupied[i]:
            last = i
            empty = 0
        else:
            empty += 1
            if empty >= gap_bins:                 #空太久，這一團到此為止
                break

    lo = edges[first]
    hi = edges[last + 1]
    return points[(z >= lo) & (z <= hi)]


def gravity_up_optical(node):
    """求「上」在相機座標系裡的方向。

    兩個來源，優先用加速度計：
      1. 靜止時加速度計讀到的就是重力，不必經過歐拉角，沒有慣例可以搞錯。
         水平時應得到 (0, -1, 0)，也就是 optical frame 的 -Y，這本身就是驗證。
      2. 移動中改用四元數解出的絕對角。abs_yaw 沒有絕對基準會漂移，不能用，
         但求「上」本來就只需要 roll 與 pitch。

    Returns:
        tuple: (up, source)。up 是 (3,) 單位向量；取不到時回 (None, None)。
    """
    accel = getattr(node, 'zed_accel', None)
    if accel is not None and len(accel) == 3:
        a = np.asarray(accel, dtype=np.float64)
        n = float(np.linalg.norm(a))
        if n > 1e-6 and abs(n - 9.81) < STATIC_ACCEL_TOL:
            return R_OPT_FROM_BODY @ (a / n), 'accel'

    rpy = getattr(node, 'zed_imu_abs_rpy', None)
    if rpy is not None and len(rpy) >= 2:
        r, pch = math.radians(float(rpy[0])), math.radians(float(rpy[1]))
        up_body = np.array([-math.sin(pch),
                            math.cos(pch) * math.sin(r),
                            math.cos(pch) * math.cos(r)])
        return R_OPT_FROM_BODY @ up_body, 'rpy'

    return None, None


def gravity_frame(up):
    """由「上」建一組重力對齊的正交基底。

    第三軸取重力方向，前軸取光軸在水平面上的投影，右軸補齊。
    yaw 沒有絕對基準，所以世界座標的水平朝向以相機自己的光軸為準 ——
    這不影響量出來的尺寸，尺寸與水平朝向無關。

    Returns:
        np.ndarray: (3, 3)，每一列是一個世界軸在相機座標系下的表示。
                    光軸與重力平行（正對地面）時水平方向無從定義，回 None。
    """
    n = float(np.linalg.norm(up))
    if n < 1e-6:
        return None
    up = np.asarray(up, dtype=np.float64) / n

    z_cam = np.array([0.0, 0.0, 1.0])
    fwd = z_cam - float(np.dot(z_cam, up)) * up
    n = float(np.linalg.norm(fwd))
    if n < 1e-3:
        return None
    fwd /= n

    return np.stack([np.cross(fwd, up), fwd, up])


def robust_extent(values, trim=None):
    """一維上的穩健延伸量測。

    兩段：先用 IQR 柵欄砍掉明顯脫隊的飛點，再用 percentile 取延伸。
    直接取 min/max 會被一顆飛點拉爆；只用 percentile 則擋不住成群的飛點。

    Returns:
        tuple: (lo, hi)。
    """
    #在呼叫時才查模組常數，這樣改上面的設定值立刻生效（預設參數會在定義時就綁死）
    trim = EXTENT_TRIM_PCT if trim is None else trim

    v = reject_outliers_iqr(values) if OUTLIER_IQR_FENCE else values
    if v.size == 0:
        v = values
    lo, hi = np.percentile(v, [trim, 100.0 - trim])
    return float(lo), float(hi)


def measure_cloud(points, up):
    """在重力對齊的座標系裡直接量點雲的尺寸。

    高度取重力軸上的延伸 —— 量的是實際的 3D 點，不是輪廓，所以不需要
    「輪廓高 = h*cos(φ) + d*sin(φ)」那條式子，也就沒有一式兩未知的問題。

    水平長寬取點雲投影到水平面後的最小外接矩形。看得到物體兩個面時（偏轉角
    θ 不是 0 度也不是 90 度，正好是誤差最大的那些情況）投影是 L 形，
    minAreaRect 解得出兩個水平尺寸與 θ；只看得到一個面時 θ 約等於 0，
    此時短邊量不到（會接近 0），但長邊本來就是正確的。

    Returns:
        dict: 量測結果與診斷資訊；算不出來時回 None。
    """
    R = gravity_frame(up)
    if R is None:
        return None
    if points.shape[0] < MIN_CLOUD_POINTS:
        return None

    world = points @ R.T

    lo_z, hi_z = robust_extent(world[:, 2])
    height_cm = float(hi_z - lo_z)

    #水平投影。minAreaRect 取的是凸包的外接矩形，一顆飛點就能把它撐大，
    #所以先濾離群點 —— 但要用**離質心的距離**而不是兩軸各自的柵欄。
    #軸對齊的柵欄會偏袒方向：實測對圓柱會把直徑從 9.73 砍到 9.43，
    #因為圓弧最外緣的點同時落在兩軸的邊界附近，兩次柵欄各砍一刀。
    #徑向柵欄是旋轉不變的，方盒與圓柱都不受影響。
    xy = world[:, :2]
    centre = xy.mean(axis=0)
    radius = np.linalg.norm(xy - centre, axis=1)
    _, r_max = robust_extent(radius)
    xy = xy[radius <= r_max]
    if xy.shape[0] < MIN_CLOUD_POINTS:
        return None

    if xy.shape[0] > MAX_CLOUD_POINTS:
        idx = np.random.default_rng(0).choice(
            xy.shape[0], MAX_CLOUD_POINTS, replace=False)
        xy = xy[idx]

    (_, _), (side_a, side_b), angle_deg = cv2.minAreaRect(
        np.ascontiguousarray(xy, dtype=np.float32))
    long_cm, short_cm = (side_a, side_b) if side_a >= side_b else (side_b, side_a)

    return {
        'height_cm': height_cm,
        'horizontal_long_cm': float(long_cm),
        'horizontal_short_cm': float(short_cm),
        'yaw_deg': float(angle_deg),
        'n_points': int(points.shape[0]),
        'mean_depth_cm': float(np.mean(points[:, 2])),
    }


def estimate_real_size(bbox, depths_cm, node=None):
    """真實大小計算器。

    把 bbox 內的深度像素反投影成 3D 點雲，用 IMU 轉到重力對齊的座標系，
    再直接量點雲的延伸。不走「像素尺寸 x 距離 / 焦距」加姿態補償那條路 ——
    理由見檔案最下方的說明。

    Args:
        bbox (tuple): (xmin, ymin, xmax, ymax)，960x600 座標系，與 /depth_mm 對齊。
        depths_cm (list): 四個區塊的平均深度(公分)。本函式不用它算尺寸，
            只在 node 上留一份給可信度閘門。
        node (MedDetect): 節點本身，用來取深度圖與 IMU。

    Returns:
        tuple: (width_cm, height_cm)。width 取水平面上的長邊，height 取重力軸上的
            延伸。算不出來時回 (None, None)，失敗原因寫在 node.size_detail['error']。
    """
    detail = {}
    if node is not None:
        node.size_detail = detail

    if node is None or getattr(node, 'depth_mm', None) is None:
        detail['error'] = '沒有深度圖'
        return None, None

    up, up_source = gravity_up_optical(node)
    if up is None:
        detail['error'] = '取不到 IMU 姿態，無法決定重力方向'
        return None, None
    detail['up_source'] = up_source
    detail['up_optical'] = [round(float(v), 4) for v in up]

    fx, fy, cx, cy = effective_intrinsics(getattr(node, 'zoom', ZOOM))
    detail['focal_px'] = round(fx, 1)

    points = backproject(node.depth_mm, bbox, fx, fy, cx, cy)
    detail['n_raw_points'] = int(points.shape[0])
    if points.shape[0] < MIN_CLOUD_POINTS:
        detail['error'] = f'bbox 內有效深度只有 {points.shape[0]} 點'
        return None, None

    points = segment_nearest_cluster(points)
    detail['n_object_points'] = int(points.shape[0])
    if points.shape[0] < MIN_CLOUD_POINTS:
        detail['error'] = f'前景分群後只剩 {points.shape[0]} 點'
        return None, None

    result = measure_cloud(points, up)
    if result is None:
        detail['error'] = '點雲量測失敗（點太少或光軸與重力平行）'
        return None, None

    detail.update(result)
    return result['horizontal_long_cm'], result['height_cm']


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


def tilt_from_quadrants(depths_cm):
    """由四區塊深度看物體表面的傾斜，當作這次量測的可信度指標。

    這是切四個區塊真正的用途：
        左半 (Q1+Q3)/2 與右半 (Q2+Q4)/2 的差 -> 表面水平傾斜
        上半 (Q1+Q2)/2 與下半 (Q3+Q4)/2 的差 -> 表面垂直傾斜
    兩個差值都接近 0 才代表物體正對鏡頭。

    Returns:
        tuple: (左右差, 上下差)，單位公分。任一區塊無效時回 (None, None)。
    """
    if depths_cm is None or any(d is None for d in depths_cm):
        return None, None
    q1, q2, q3, q4 = depths_cm
    return ((q1 + q3) / 2.0 - (q2 + q4) / 2.0,
            (q1 + q2) / 2.0 - (q3 + q4) / 2.0)


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

        #數位變焦倍率。網頁上調過 zoom 而這裡沒跟上，等效焦距就會差一個倍率。
        self.zoom = ZOOM
        self.create_subscription(
            Zoom, ZOOM_TOPIC, self.zoom_callback,
            self.qos_fast, callback_group=self.image_cbg)

        #estimate_real_size() 把量測細節與失敗原因寫在這裡
        self.size_detail = {}

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

    def zoom_callback(self, msg):
        """數位變焦倍率。image.py 的 zoomValue 也是吃同一個 topic。"""
        try:
            self.zoom = float(msg.zoomin)
        except Exception as e:
            self.get_logger().warn(f"[zoom] bad value: {e}")

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

        lr, ud = tilt_from_quadrants(depths)
        if lr is not None and (abs(lr) > TILT_WARN_CM or abs(ud) > TILT_WARN_CM):
            self.get_logger().info(
                f"表面傾斜: 左右 {lr:+.1f} cm、上下 {ud:+.1f} cm，"
                f"物體沒有正對鏡頭\033[K")

        width_cm, height_cm = estimate_real_size(bbox, depths, self)
        detail = self.size_detail

        if width_cm is None or height_cm is None:
            self.get_logger().warn(
                f"量不出尺寸（{detail.get('error', '未知原因')}），"
                f"只輸出 bbox 與四區塊深度\033[K")
            self.publish_result(label, bbox, depths)
            return

        self.get_logger().info(
            f"實際尺寸: {width_cm:.2f} x {height_cm:.2f} cm"
            f"（厚 {detail['horizontal_short_cm']:.2f}、"
            f"偏轉 {detail['yaw_deg']:.0f} 度、"
            f"{detail['n_object_points']} 點、"
            f"重力來源 {detail['up_source']}）\033[K")

        best, err, ok = match_size(width_cm, height_cm, self.size_table)
        if best is None:
            self.get_logger().warn("對照表是空的，無法比對\033[K")
            self.publish_result(label, bbox, depths, width_cm, height_cm,
                                detail=detail)
            return

        if ok:
            self.get_logger().info(f"✅ 判定: {best['name']} (誤差 {err:.2f} cm)\033[K")
        else:
            self.get_logger().info(
                f"⚠ 最接近 {best['name']} 但誤差 {err:.2f} cm 超過門檻 "
                f"{MATCH_TOLERANCE_CM} cm\033[K")

        self.publish_result(label, bbox, depths, width_cm, height_cm,
                            best, err, ok, detail=detail)

    def touches_edge(self, bbox):
        """bbox 是否碰到畫面邊緣。

        物體被畫面切掉時 bbox 只框到露出來的部分，寬高都偏小，
        算出來的實際尺寸必錯，與其輸出錯的不如不輸出。

        ⚠ 真正的上下緣不在 0 與 h。影像是 16:9 等比縮到 960x540 再上下補黑成
        960x600，所以有效影像是第 LETTERBOX_TOP ~ (h - LETTERBOX_BOTTOM) 列。
        直接拿 0 與 h 比的話，貼著畫面頂端的物體（ymin 約 30）會被放行。
        """
        h, w = self.depth_mm.shape[:2]
        xmin, ymin, xmax, ymax = bbox
        m = EDGE_MARGIN_PX
        top = LETTERBOX_TOP
        bottom = h - LETTERBOX_BOTTOM
        return (xmin <= m or xmax >= w - m or
                ymin <= top + m or ymax >= bottom - m)

    def quadrant_depths(self, bbox):
        """把 bbox 切四塊，每塊各自剔除不合理值後取平均。

        這個值**不參與尺寸計算** —— 尺寸是由 estimate_real_size() 直接量點雲得到的。
        四區塊的用途是 tilt_from_quadrants()：看左右/上下的深度差判斷物體有沒有
        正對鏡頭，當這次量測的可信度指標。

        ⚠ IQR 只砍尾巴，砍不掉雙峰。區塊裡一半是物體一半是後面的牆時，
        平均值會停在兩者中間。所以這個數字只適合看趨勢（哪半邊比較遠），
        不適合當絕對距離用。真正的前景分割在 segment_nearest_cluster()。

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
                       width_cm=None, height_cm=None, best=None, err=None, ok=False,
                       detail=None):
        """發布結果。

        尺寸算不出來時照樣發 bbox 與四區塊深度，並附上失敗原因，
        下游看得出來是量不到還是量錯。
        """
        lr, ud = tilt_from_quadrants(depths)
        out = {
            'label': label,
            'bbox': list(bbox),
            'quadrant_depth_cm': depths,
            'surface_tilt_lr_cm': None if lr is None else round(lr, 2),
            'surface_tilt_ud_cm': None if ud is None else round(ud, 2),
        }
        if width_cm is not None and height_cm is not None:
            out['width_cm'] = round(width_cm, 2)
            out['height_cm'] = round(height_cm, 2)
        if detail:
            for key in ('horizontal_short_cm', 'yaw_deg', 'mean_depth_cm'):
                if key in detail:
                    out[key] = round(detail[key], 2)
            for key in ('n_object_points', 'up_source', 'focal_px', 'error'):
                if key in detail:
                    out[key] = detail[key]
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
# 真實大小計算器 —— 作法與待辦
# =============================================================================
#
# 目標：由 bbox（像素）+ 深度圖 + IMU 姿態，算出物體的實際尺寸（公分）。
# 實作位置：本檔上半部的 estimate_real_size() 及其幾個輔助函式。
#
# -----------------------------------------------------------------------------
# 1. 為什麼不用「像素尺寸 x 距離 / 焦距」
# -----------------------------------------------------------------------------
#     那條公式量的是**輪廓**，而輪廓與真實尺寸之間隔著兩個未知數：
#
#         輪廓高 = h * cos(φ) + d * sin(φ)      φ = 相機俯角
#         輪廓寬 = w * cos(θ) + d * sin(θ)      θ = 物體繞自身垂直軸的偏轉角
#
#     d（物體前後厚度）從單一視角看不到，兩條式子各是一式兩未知，解不開。
#     10x10 方柱的實測誤差：10 度 +16%、30 度 +37%、45 度 +41%。
#     試過「假設 d = 量到的寬度」，對圓柱準、對方形反而比不補償更差。
#
#     但這個困境**只在把深度圖壓成幾個純量之後才成立**。bbox 裡有幾千個
#     帶距離的像素，那是一團 3D 點雲，不是一個輪廓。當成點雲處理，
#     就從「一式兩未知」變成「上千個觀測解兩個未知數」。
#
# -----------------------------------------------------------------------------
# 2. 實際作法
# -----------------------------------------------------------------------------
#     backproject()             bbox 內每個有效深度像素 -> 相機座標系的 3D 點
#                                   X = (u - cx) * Z / fx
#                                   Y = (v - cy) * Z / fy
#     segment_nearest_cluster() 在深度直方圖上找最近的一團，把背景切掉
#     gravity_up_optical()      用 IMU 求「上」在相機座標系裡的方向
#     gravity_frame()           由「上」建一組重力對齊的正交基底
#     measure_cloud()           在那個座標系裡直接量：
#                                   高      = 重力軸上的延伸
#                                   水平長寬 = 水平投影的最小外接矩形
#     robust_extent()           量延伸時擋飛點：IQR 柵欄 + percentile
#
#     ⚠ 水平投影的離群值要用**離質心的距離**濾，不能用兩軸各自的柵欄。
#       軸對齊的柵欄會偏袒方向：圓弧最外緣的點同時落在兩軸邊界附近，
#       兩次柵欄各砍一刀，實測把圓柱直徑從 9.73 砍到 9.43。
#
# -----------------------------------------------------------------------------
# 3. 這樣做之後，原本需要的補償全部消失
# -----------------------------------------------------------------------------
#     曲面物體的「+半徑」   圓柱的正面在水平面投影是一段弧，minAreaRect 的
#                           寬邊直接就是直徑，不需要外加修正項
#     俯角 h*cosφ + d*sinφ  高度量的是實際 3D 點在重力軸上的延伸，不是輪廓
#     水平偏轉 w*cosθ+d*sinθ minAreaRect 直接解出 θ 與兩個水平尺寸
#     離軸斜視 atan(...)     (u - cx) 本來就含這一項
#     roll 大時整組作廢      roll 只是旋轉矩陣的一部分，不必作廢
#
#     只有數位變焦仍然需要補（effective_intrinsics()），因為它改的是相機模型
#     本身，不是物體姿態。
#
#     θ 確實無法從頭部角度得知（物體怎麼擺跟頭轉到哪無關），但也不需要：
#     看得到物體兩個面時（θ 不是 0 度也不是 90 度，正好是誤差最大的那些情況）
#     水平投影是 L 形，minAreaRect 解得出來。只看得到一個面時 θ 約等於 0，
#     短邊量不到（會接近 0），但長邊本來就是正確的。
#     **3D 擬合失效的情況，剛好就是不需要它的情況。**
#
# -----------------------------------------------------------------------------
# 4. 輸入從哪裡來
# -----------------------------------------------------------------------------
#     bbox          estimate_real_size() 的參數，960x600 座標系
#     深度圖         node.depth_mm，mono16 公釐，0 代表無效
#     相機姿態       node.zed_accel（靜止時就是重力，最可靠）
#                   node.zed_imu_abs_rpy = [roll, pitch, yaw]，單位「度」
#                   abs_roll / abs_pitch 以重力為基準可信；abs_yaw 會漂移不要用
#                   （來源：walking/zed_imu_node.py，API.py 已接好）
#     數位變焦       node.zoom，訂閱自 /Zoom_In_Topic
#     頭部馬達角度   ⚠ 拿不到，API 只有發送端沒有位置回授。用 IMU 取代。
#
# -----------------------------------------------------------------------------
# 5. 上機前必須確認的三件事
# -----------------------------------------------------------------------------
#     這三項都不影響演算法結構，只是參數，但沒確認之前量出來的數字不能信。
#
#     (a) 焦距 FOCAL_PX
#         目前的 440 是用第 7 點的圓柱實測反推的粗估值。請拿已知尺寸的
#         **平面**物體正對鏡頭重量：f = 像素寬 x 距離 / 實際寬。
#         不要用圓柱或球 —— 它們的輪廓是切線不是邊緣，會低估焦距約 10%。
#
#     (b) 深度值是「沿光軸的 Z」還是「到相機的徑向距離」
#         backproject() 假設是前者（ZED SDK 的 MEASURE::DEPTH 就是前者），
#         但上游若自己轉過就不是。驗證：對著平牆比較 depth_at(480,300) 與
#         depth_at(100,100)。相同 -> 是 Z；往角落遞增 -> 是徑向，
#         要先除以 sqrt(1 + ((u-cx)/fx)^2 + ((v-cy)/fy)^2)。
#
#     (c) 深度圖與 bbox 是否真的對齊、zoom 是否同步
#         彩色影像走的是「裁切 + 放大」。深度圖若沒跟著裁，zoom != 1 時
#         bbox 會索引到深度圖的**錯誤位置** —— 不是差一個倍率，是整個對錯地方。
#         驗證：拿色模物體的中心去查 depth_at()，看距離合不合理（strategy 的
#         apitest 節點就是為了這件事寫的）。
#
#         順便確認補黑邊界：np.count_nonzero(depth_mm[0:30, :]) 應該是 0。
#         若不是 0，代表深度圖沒有補黑，LETTERBOX_TOP/BOTTOM 要改成 0。
#
# -----------------------------------------------------------------------------
# 6. 合成驗證結果（test/test_geometry.py，不需要 ROS 也不需要相機）
# -----------------------------------------------------------------------------
#     作法是反過來做一遍：給定真實尺寸與相機姿態 -> 產生合成深度圖（z-buffer
#     含遮擋）-> 丟回本管線 -> 看能不能量回真值。
#
#     12 x 4 x 8 cm 方盒，距離 50 cm，俯角 0~45 度 x 滾轉 0~20 度 x 偏轉 0~45 度
#     共 32 組姿態：
#         本作法最大誤差   3.8%
#         舊的反解法最大誤差 77.8%
#
#     10 x 10 cm 圓柱，俯角 0 / 20 / 35 度：
#         最大誤差 2.7%，不需要外加「+半徑」修正
#
#     物體後方 25 cm 有牆：量出 12.07 x 8.04（前景分群有效切掉背景）
#
#     ⚠ 這是合成資料。真實的 ZED 深度圖有雜訊、破洞與飛點，實機誤差一定更大。
#       上機後請照第 5 點校正，並用實物重測一次。
#
# -----------------------------------------------------------------------------
# 7. 已知的限制
# -----------------------------------------------------------------------------
#     只看得到正面      水平短邊（厚度）在只看得到一個面時量不到，會接近 0。
#                       對照表比對只用長邊與高度，不受影響。
#     深度破洞          ZED 對反光、透明、很細的物體會量不到。點數不足時
#                       estimate_real_size() 回 (None, None) 並在
#                       node.size_detail['error'] 說明原因，不會硬給一個數字。
#     正對地面          光軸與重力平行時水平朝向無從定義，gravity_frame() 回 None。
#     移動中量測        加速度計在移動時讀到的不是純重力，此時會退回用四元數
#                       解出的角度。要準還是建議站定再量。
#     飛點              合成測試裡 1% 的飛點會讓誤差升到 7.6%、3% 升到 30%。
#                       OUTLIER_IQR_FENCE / EXTENT_TRIM_PCT 目前是對著猜出來的
#                       雜訊模型調的，上機後請拿真實資料重調。
#     IMU 安裝偏移      ZED 的 IMU 與左目光心之間還有一個固定的機械旋轉
#                       （SDK 的 camera_imu_transform），通常小於 1 度，
#                       本實作忽略。要更準可以把它乘進 R_OPT_FROM_BODY。
#
# -----------------------------------------------------------------------------
# 8. 實測參考數據（2026-09-18，ZED、960x600、zoom=1.0）
# -----------------------------------------------------------------------------
#     10 x 10 cm 圓柱：
#         bbox (480, 192, 566, 281)  ->  86 x 89 px
#         四區塊深度 46.4 / 49.3 / 45.7 / 45.4 cm，平均 46.7 cm
#
#     由寬度反推焦距（圓柱的輪廓是切線，要用軸心距離 46.7 + 5 = 51.7）：
#         半寬 = f * r / sqrt(D^2 - r^2)  ->  86 = f * 5 / 51.46  ->  f ≈ 442
#     由高度反推：
#         f = 89 * 46.7 / 10 ≈ 416
#
#     兩者差 6%。影像是等比縮放後補黑的，兩軸焦距應該相同，所以這個差距是
#     量測精度不足 —— 色模在 320x240 上算完再放大，bbox 邊界輕易有 ±3 px，
#     86 px 上差 3 px 就是 3.5%。FOCAL_PX = 440 取的是兩者中間偏寬度那一側，
#     請依第 5(a) 點重量。
# =============================================================================
