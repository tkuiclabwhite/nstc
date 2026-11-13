"""網頁 <-> ZED 相機參數橋接（僅 camera_source:=zed 時啟動）。

ZED 的相機設定不是 topic，而是 /zed/zed_node 的 ROS parameter，所以這裡透過
rcl_interfaces/srv/SetParameters 呼叫，而非發布訊息。

參數存放於 <location>/ZedCameraSet.ini，與 usb_cam 擁有的 CameraSet.ini 分家：
usb_cam 的 save_camera_set() 是整檔覆寫且只寫它認得的 7 個鍵，ZED 參數若混在
同一個檔，切回 usb 模式存檔時會被靜默洗掉。zoomin 也一併存在本檔，image_node
與 depth_process_node 在 zed 模式會來讀它。

ZED X Mini 不支援 video.brightness / contrast / hue（wrapper 對 ZED X 系列不宣告
這三個參數），故面板改用下列這組。
"""
import configparser
import os
import time
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node

from nstc_msgs.msg import Location, ZedCamera, ZedCameraSave
from nstc_msgs.srv import ZedCameraInfo

INI_FILENAME = "ZedCameraSet.ini"
INI_SECTION = "ZED Camera Set Parameter"

# 名稱 -> (ROS 參數型別, 下限, 上限)；預設值取自 zed_wrapper 的 common_stereo.yaml
PARAM_SPEC = {
    "auto_exposure_gain":       ("bool", 0, 1),
    "exposure":                 ("int", 0, 100),
    "gain":                     ("int", 0, 100),
    "auto_whitebalance":        ("bool", 0, 1),
    "whitebalance_temperature": ("int", 28, 65),   # ZED SDK 內部 x100 => 2800~6500K
    "saturation":               ("int", 0, 8),
    "denoising":                ("int", 0, 100),   # ZED X 專屬
    "gamma":                    ("int", 1, 9),
}
DEFAULTS = {
    "auto_exposure_gain": True,
    "exposure": 80,
    "gain": 80,
    "auto_whitebalance": True,
    "whitebalance_temperature": 42,
    "saturation": 4,
    "denoising": 50,
    "gamma": 8,
}
DEFAULT_ZOOM = 1.0

# 網頁拉滑桿會連續送訊息，限制實際打到 ZED 的頻率
APPLY_MIN_INTERVAL = 0.1


class CameraParamBridgeNode(Node):
    def __init__(self):
        super().__init__('camera_param_bridge_node')
        self.get_logger().info("=======ZED Camera Param Bridge On=======")

        self.declare_parameter('zed_node_name', '/zed/zed_node')
        zed_node = str(self.get_parameter('zed_node_name').value).rstrip('/')

        self.values = dict(DEFAULTS)
        self.zoomin = DEFAULT_ZOOM
        self._last_applied = None
        self._last_apply_t = 0.0

        # --- 路徑解析：與 image.py 相同規則，本節點不寫 strategy.ini ---
        self.location = ""
        self.strategy_root = self._resolve_strategy_root()
        fallback_dir = Path.home() / "nstc" / "config" / "image"
        fallback_dir.mkdir(parents=True, exist_ok=True)
        self.path_dir = fallback_dir
        raw = self._read_strategy_ini_raw()
        if raw:
            self.location = self._canon_location(raw)
            self.get_logger().info(f"[BOOT] strategy.ini -> {raw} => {self.location}")

        self.set_param_cli = self.create_client(
            SetParameters, f'{zed_node}/set_parameters')

        self.param_sub = self.create_subscription(
            ZedCamera, '/ZedCamera_Topic', self.param_callback, 10)
        self.save_sub = self.create_subscription(
            ZedCameraSave, '/ZedCamera_Save', self.save_callback, 10)
        self.location_sub = self.create_subscription(
            Location, '/location', self.location_callback, 1000)

        self.info_srv = self.create_service(
            ZedCameraInfo, '/ZedCameraInfo', self.info_callback)

        self.load_from_ini()

        # ZED node 起得比本節點慢，用計時器重試直到 set_parameters 服務出現
        self._boot_timer = self.create_timer(1.0, self._boot_apply)

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

    def _ini_path(self) -> Path:
        base = Path(self.location) if self.location else Path(self.path_dir)
        ini = base if base.suffix else (base / INI_FILENAME)
        ini.parent.mkdir(parents=True, exist_ok=True)
        return ini

    # ------------------------------------------------------------------
    # ini 讀寫
    # ------------------------------------------------------------------
    def load_from_ini(self):
        ini_path = self._ini_path()
        cfg = configparser.ConfigParser()

        if not cfg.read(str(ini_path)) or INI_SECTION not in cfg:
            # 沒檔就用預設值建一份；zoomin 若 CameraSet.ini 有就沿用，方便從 usb 設定遷移
            self.values = dict(DEFAULTS)
            self.zoomin = self._zoom_from_camera_set()
            self.save_to_ini()
            return

        sec = cfg[INI_SECTION]
        for name, (kind, lo, hi) in PARAM_SPEC.items():
            rawval = sec.get(name)
            if rawval is None:
                continue
            try:
                if kind == "bool":
                    self.values[name] = str(rawval).strip() not in ("0", "", "false", "False")
                else:
                    self.values[name] = max(lo, min(hi, int(float(rawval))))
            except ValueError as e:
                self.get_logger().warn(f"[{INI_FILENAME}] bad value for '{name}': {e}")

        try:
            self.zoomin = float(sec.get("zoomin", str(DEFAULT_ZOOM)))
        except ValueError:
            self.zoomin = DEFAULT_ZOOM

        self.get_logger().info(f"[{INI_FILENAME}] loaded from {ini_path}: "
                               f"{self.values}, zoomin={self.zoomin}")

    def _zoom_from_camera_set(self) -> float:
        """首次建檔時沿用 usb 版 CameraSet.ini 的 zoomin，避免既有設定失效。"""
        base = Path(self.location) if self.location else Path(self.path_dir)
        ini_path = base if base.suffix else (base / "CameraSet.ini")
        cfg = configparser.ConfigParser()
        if cfg.read(str(ini_path)) and "Camera Set Parameter" in cfg:
            try:
                return float(cfg["Camera Set Parameter"].get("zoomin", DEFAULT_ZOOM))
            except ValueError:
                pass
        return DEFAULT_ZOOM

    def save_to_ini(self) -> bool:
        ini_path = self._ini_path()
        cfg = configparser.ConfigParser()
        cfg.read(str(ini_path))          # 保留檔內其他區段（目前沒有，但不預設獨佔）
        cfg[INI_SECTION] = {
            **{name: ("1" if self.values[name] else "0") if kind == "bool"
               else str(int(self.values[name]))
               for name, (kind, _, _) in PARAM_SPEC.items()},
            "zoomin": f"{float(self.zoomin):.1f}",
        }
        try:
            with open(ini_path, "w") as f:
                cfg.write(f)
            self.get_logger().info(f"[{INI_FILENAME}] saved: {ini_path}")
            return True
        except OSError as e:
            self.get_logger().error(f"[{INI_FILENAME}] save failed: {e}")
            return False

    # ------------------------------------------------------------------
    # 套用到 ZED node
    # ------------------------------------------------------------------
    def _boot_apply(self):
        if not self.set_param_cli.service_is_ready():
            return
        self._boot_timer.cancel()
        self.get_logger().info("ZED set_parameters 服務已就緒，套用開機設定")
        self.apply_to_zed(force=True)

    def apply_to_zed(self, force: bool = False):
        now = time.time()
        if not force:
            if self.values == self._last_applied:
                return
            if now - self._last_apply_t < APPLY_MIN_INTERVAL:
                return

        if not self.set_param_cli.service_is_ready():
            self.get_logger().warn("ZED set_parameters 服務尚未就緒，略過本次套用")
            return

        params = []
        for name, (kind, _, _) in PARAM_SPEC.items():
            # exposure/gain 只在手動曝光時才送、色溫只在手動白平衡時才送，
            # 否則 ZED wrapper 會因為自動模式生效而回警告
            if name in ("exposure", "gain") and self.values["auto_exposure_gain"]:
                continue
            if name == "whitebalance_temperature" and self.values["auto_whitebalance"]:
                continue

            if kind == "bool":
                value = ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                       bool_value=bool(self.values[name]))
            else:
                value = ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                       integer_value=int(self.values[name]))
            params.append(ParameterMsg(name=f"video.{name}", value=value))

        req = SetParameters.Request(parameters=params)
        future = self.set_param_cli.call_async(req)
        future.add_done_callback(self._on_set_done)

        self._last_applied = dict(self.values)
        self._last_apply_t = now

    def _on_set_done(self, future):
        try:
            results = future.result().results
        except Exception as e:
            self.get_logger().error(f"set_parameters 呼叫失敗: {e}")
            return
        for r in results:
            if not r.successful:
                self.get_logger().warn(f"set_parameters 被拒: {r.reason}")

    # ------------------------------------------------------------------
    # Topic / Service callbacks
    # ------------------------------------------------------------------
    def _from_msg(self, msg):
        for name, (kind, lo, hi) in PARAM_SPEC.items():
            val = getattr(msg, name)
            self.values[name] = bool(val) if kind == "bool" else max(lo, min(hi, int(val)))

    def param_callback(self, msg):
        self._from_msg(msg)
        self.apply_to_zed()

    def save_callback(self, msg):
        self._from_msg(msg)
        self.zoomin = float(msg.zoomin)
        self.apply_to_zed(force=True)
        self.save_to_ini()

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
        self.load_from_ini()
        self.apply_to_zed(force=True)

    def info_callback(self, request, response):
        del request                # request.load 僅作觸發旗標，與既有 /CameraInfo 一致
        self.load_from_ini()
        for name in PARAM_SPEC:
            setattr(response, name, self.values[name])
        response.zoomin = float(self.zoomin)
        self.get_logger().info(f"[{INI_FILENAME}] send to client: {self.values}, "
                               f"zoomin={self.zoomin}")
        return response


def main():
    rclpy.init()
    node = CameraParamBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
