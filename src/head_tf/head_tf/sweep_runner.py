#!/usr/bin/env python3
"""頭部掃描的流程：轉頭 -> 等停穩 -> 記錄 -> 分析 -> 存檔。不依賴 ROS。

和 ROS 之間只透過一個 io 物件，要有這些方法（head_sweep.py 的節點、測試的模擬器都照這個介面）：

    now() -> float                       本機 monotonic 秒
    sleep(seconds)
    send_head(pan, tilt, speed)          下頭部刻度
    latest_joint() -> (t, pan, tilt)     最新一筆刻度，還沒收到時回 None
    joints_since(t) -> [(t, pan, tilt)]
    snapshot(t0, t1, max_frames) -> sweep_core.Snapshot
"""
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from head_tf import sweep_core as core


class SweepError(RuntimeError):
    """收集不能繼續的錯誤，訊息直接印給使用者看。"""


class HeadNotMoving(SweepError):
    pass


@dataclass
class SweepConfig:
    speed: int = 40                  # HeadPackage.speed（約 9 rpm）
    reach_tol: int = 12              # 離目標幾格以內算到達（約 1°）
    move_timeout: float = 5.0        # 等到達的上限（s）
    stuck_min_ticks: int = 40        # 目標離原位超過這麼多格（約 3.5°）卻完全沒動，才判定頭沒動
    resend_period: float = 0.5       # 還沒到達時隔多久重送一次指令（s）
    still_window: float = 0.5        # 這段時間內刻度變化不超過 still_tol 才算停穩（s）
    still_tol: int = 2
    still_timeout: float = 3.0
    settle: float = 1.5              # 停穩後再等多久才開始記錄（頭的晃動、ZED 深度的時間平滑）
    capture: float = 2.0             # 記錄多久（s）
    max_depth_frames: int = 10
    min_depth_frames: int = 3
    joint_stale: float = 1.0         # /joint_states 超過這麼久沒更新就停止（s）
    plane_rows: tuple = core.PLANE_ROWS
    plane_cols: tuple = core.PLANE_COLS
    plane_threshold: float = core.PLANE_THRESHOLD_M
    box_margin: int = core.BOX_MARGIN_PX

    def validate(self):
        core.check_speed(self.speed)
        if self.capture <= 0 or self.settle < 0:
            raise ValueError('記錄時間要大於 0，等待時間不能是負的')
        if self.max_depth_frames < self.min_depth_frames:
            raise ValueError(f'深度圖張數至少要 {self.min_depth_frames}')


@dataclass
class MoveResult:
    reached: bool
    still: bool
    move_s: float


def _fresh_joint(io, cfg):
    j = io.latest_joint()
    if j is None:
        raise SweepError('收不到 /joint_states 的 28、29 號馬達')
    if io.now() - j[0] > cfg.joint_stale:
        raise SweepError(f'/joint_states 超過 {cfg.joint_stale:g} 秒沒更新，馬達 driver 還在跑嗎？')
    return j


def wait_still(io, cfg):
    """等刻度停穩；超過 still_timeout 還在動就回 False（照樣記錄，CSV 會標出來）。"""
    t0 = io.now()
    while True:
        now = io.now()
        if core.is_still(io.joints_since(now - cfg.still_window - 0.2),
                         cfg.still_window, cfg.still_tol, now):
            return True
        if now - t0 > cfg.still_timeout:
            return False
        _fresh_joint(io, cfg)
        io.sleep(0.05)


def move_and_settle(io, pan, tilt, cfg):
    """下刻度，等到達、停穩、再等 settle 秒。

    到不了目標但有動（例如被機構擋住、重力讓 tilt 差幾格）就照樣記錄，因為分析用的是
    實際讀到的刻度，不是下的指令。完全沒動才停止：多半是扭力沒開或 driver 沒在跑。
    """
    before = _fresh_joint(io, cfg)
    t0 = io.now()
    io.send_head(pan, tilt, cfg.speed)
    last_send = t0
    reached = False
    while True:
        j = _fresh_joint(io, cfg)
        now = io.now()
        if abs(j[1] - pan) <= cfg.reach_tol and abs(j[2] - tilt) <= cfg.reach_tol:
            reached = True
            break
        if now - t0 > cfg.move_timeout:
            break
        if now - last_send >= cfg.resend_period:
            io.send_head(pan, tilt, cfg.speed)
            last_send = now
        io.sleep(0.02)
    move_s = io.now() - t0
    if not reached:
        stuck = []
        for name, col, target in (('pan（28 號）', 1, pan), ('tilt（29 號）', 2, tilt)):
            # 只在離目標很遠時才判定「沒動」：重力可能讓 tilt 停在差十幾格的地方，
            # 下一個目標如果剛好在附近，馬達本來就不會再動
            needed = abs(target - before[col]) > cfg.stuck_min_ticks
            moved = abs(j[col] - before[col]) > 3
            if needed and not moved:
                stuck.append(name)
        if stuck:
            raise HeadNotMoving(
                f'{"、".join(stuck)} 在 {cfg.move_timeout:g} 秒內完全沒動'
                f'（目標 pan {pan} tilt {tilt}，讀到 pan {j[1]:.0f} tilt {j[2]:.0f}）。'
                '頭部扭力可能沒開，或有別的程式在控制頭部')
    still = wait_still(io, cfg)
    io.sleep(cfg.settle)
    return MoveResult(reached, still, move_s)


def capture(io, cfg):
    t0 = io.now()
    io.sleep(cfg.capture)
    return io.snapshot(t0, io.now(), cfg.max_depth_frames)


class SweepWriter:
    """每量完一個姿勢就寫檔，中途中斷時已經量的都還在。

    檔案：
        meta.json          收集的設定、相機內參、開始時的刻度
        postures.csv       每個姿勢一列（sweep_core.POSTURE_FIELDS）
        bottles.csv        每個姿勢每個瓶子一列（sweep_core.BOTTLE_FIELDS）
        posture_XX.npz     深度中位數圖、內參、記錄期間的刻度 / IMU 原始資料、YOLO 原始結果
        posture_XX.png     彩色影像（機器人上沒有 OpenCV 時改存在 npz 的 color）
    """

    def __init__(self, out_dir):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=False)
        self.count = 0
        self._meta = {}

    def write_meta(self, meta):
        self._meta = dict(meta)
        self._dump_meta()

    def finish(self, status):
        self._meta.update(status=status, postures_recorded=self.count)
        self._dump_meta()

    def _dump_meta(self):
        with open(self.out_dir / 'meta.json', 'w', encoding='utf-8') as f:
            json.dump(self._meta, f, ensure_ascii=False, indent=2, default=_json_default)

    def _append_csv(self, name, fields, rows):
        path = self.out_dir / name
        new = not path.exists()
        with open(path, 'a', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
            if new:
                w.writeheader()
            for r in rows:
                w.writerow({k: _csv_value(r.get(k, '')) for k in fields})

    def add(self, row, bottle_rows, depth, snap):
        index = row['index']
        self._append_csv('postures.csv', core.POSTURE_FIELDS, [row])
        self._append_csv('bottles.csv', core.BOTTLE_FIELDS,
                         [dict(b, index=index) for b in bottle_rows])
        arrays = {
            'k': np.asarray(snap.k, dtype=np.float64),
            'size': np.asarray(snap.size, dtype=np.int32),
            'joints': np.asarray(snap.joints, dtype=np.float64).reshape(-1, 3),
            'imu': np.asarray(snap.imu, dtype=np.float64).reshape(-1, 9),
            'detections': np.asarray(json.dumps([d for _, d in snap.detections], ensure_ascii=False)),
        }
        if depth is not None:
            arrays['depth'] = depth
        if snap.color is not None and not _write_png(self.out_dir / f'posture_{index:02d}.png', snap.color):
            arrays['color'] = snap.color
        np.savez_compressed(self.out_dir / f'posture_{index:02d}.npz', **arrays)
        self.count += 1


def _write_png(path, bgr):
    try:
        import cv2
    except ImportError:
        return False
    return bool(cv2.imwrite(str(path), bgr))


def _csv_value(v):
    if isinstance(v, float):
        return f'{v:.6g}' if math.isfinite(v) else 'nan'
    if isinstance(v, bool):
        return int(v)
    return v


def _json_default(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, tuple):
        return list(o)
    raise TypeError(f'{type(o)} 不能轉成 JSON')


def record_posture(io, cfg, writer, index, kind, pan_cmd, tilt_cmd, move):
    snap = capture(io, cfg)
    row, bottle_rows, depth = core.analyze_capture(
        snap, cfg.plane_rows, cfg.plane_cols, cfg.plane_threshold, cfg.box_margin)
    row.update(index=index, kind=kind, pan_cmd=pan_cmd, tilt_cmd=tilt_cmd,
               reached=move.reached, still=move.still, move_s=move.move_s)
    writer.add(row, bottle_rows, depth, snap)
    return row, bottle_rows


def run_sweep(io, postures, cfg, writer, log=print):
    """依序量每個姿勢。回傳 (rows, bottle_rows)。"""
    rows, all_bottles = [], []
    for i, (kind, pan, tilt) in enumerate(postures):
        move = move_and_settle(io, pan, tilt, cfg)
        row, bottle_rows = record_posture(io, cfg, writer, i, kind, pan, tilt, move)
        rows.append(row)
        all_bottles += [dict(b, index=i) for b in bottle_rows]
        log(format_row(row, bottle_rows, len(postures), cfg))
    return rows, all_bottles


def run_manual(io, cfg, writer, log=print, ask=input):
    """手動模式：使用者自己把頭轉好，按 Enter 記錄一個姿勢，輸入 q 結束。

    第一個和最後一個姿勢請放在基準刻度，format_summary 才能檢查中間有沒有東西動過。
    手動模式的 kind：第一個是 'ref'，之後是 'manual'；結束時最後一個若刻度離第一個不到
    reach_tol 也標成 'ref'（CSV 已經寫出，標記只影響這次結束時的檢查）。
    """
    rows, all_bottles = [], []
    i = 0
    while True:
        try:
            ans = ask(f'[{i}] 把頭轉到要量的位置，停穩後按 Enter 記錄（輸入 q 結束）：')
        except EOFError:          # 沒有終端機可以輸入，當作結束
            break
        if ans.strip().lower() == 'q':
            break
        _fresh_joint(io, cfg)
        still = wait_still(io, cfg)
        io.sleep(min(cfg.settle, 0.5))
        move = MoveResult(reached=True, still=still, move_s=0.0)
        kind = 'ref' if i == 0 else 'manual'
        row, bottle_rows = record_posture(io, cfg, writer, i, kind, core.NAN, core.NAN, move)
        rows.append(row)
        all_bottles += [dict(b, index=i) for b in bottle_rows]
        log(format_row(row, bottle_rows, None, cfg))
        i += 1
    if len(rows) >= 2 and core._finite(rows[0]['pan_mean'], rows[-1]['pan_mean'],
                                       rows[0]['tilt_mean'], rows[-1]['tilt_mean']):
        if (abs(rows[-1]['pan_mean'] - rows[0]['pan_mean']) <= cfg.reach_tol
                and abs(rows[-1]['tilt_mean'] - rows[0]['tilt_mean']) <= cfg.reach_tol):
            rows[-1]['kind'] = 'ref'
    return rows, all_bottles


# ---------------------------------------------------------------------------
# 印出來給人看的
# ---------------------------------------------------------------------------
def _fmt(v, spec, missing='—'):
    return format(v, spec) if core._finite(v) else missing


def format_row(row, bottle_rows, total, cfg):
    i = row['index']
    head = f'[{i + 1:2d}/{total}]' if total else f'[{i}]'
    if core._finite(row.get('pan_cmd')):
        ticks = (f'pan {row["pan_cmd"]:.0f}→{_fmt(row["pan_mean"], ".0f")} '
                 f'tilt {row["tilt_cmd"]:.0f}→{_fmt(row["tilt_mean"], ".0f")}')
    else:
        ticks = f'pan {_fmt(row["pan_mean"], ".0f")} tilt {_fmt(row["tilt_mean"], ".0f")}'
    if row.get('kind') == 'ref':
        ticks += '（基準）'
    table = (f'桌面 roll {_fmt(row["table_roll_deg"], "+.2f")}° pitch {_fmt(row["table_pitch_deg"], "+.2f")}° '
             f'高 {_fmt(row["table_height_m"] * 100 if core._finite(row["table_height_m"]) else None, ".1f")} cm')
    imu = f'IMU roll {_fmt(row["imu_roll_deg"], "+.2f")}° pitch {_fmt(row["imu_pitch_deg"], "+.2f")}°'
    seen = [b['label'] for b in bottle_rows if b['n_points'] > 0]
    bottles = f'瓶子 {" ".join(seen)}' if seen else '瓶子 無'
    warn = []
    if not row.get('reached', True):
        warn.append('沒到目標刻度')
    if not row.get('still', True):
        warn.append('頭沒停穩')
    if row['depth_frames'] < cfg.min_depth_frames:
        warn.append(f'深度圖只有 {row["depth_frames"]} 張')
    if not core._finite(row['table_pitch_deg']):
        warn.append('桌面擬合失敗')
    elif row['plane_inliers'] < 0.9:
        warn.append(f'桌面取樣區只有 {row["plane_inliers"]:.0%} 在平面上')
    if core._finite(row.get('pan_std'), row.get('tilt_std')) and max(row['pan_std'], row['tilt_std']) > 1.5:
        warn.append('記錄時刻度在跳')
    line = f'{head} {ticks} | {table} | {imu} | {bottles}'
    return line + (f'  ⚠ {"、".join(warn)}' if warn else '')


def format_summary(rows, bottle_rows):
    """掃描結束時印的檢查：方向和比例、頭尾基準姿勢的差。"""
    lines = ['', '=== 快速檢查 ===']
    d = core.direction_check(rows, bottle_rows)
    if d['tilt_slope'] is None:
        lines.append('tilt：資料不夠（要有桌面擬合成功、且 tilt 至少差 2° 的姿勢）')
    else:
        s = d['tilt_slope']
        way = '往下看' if s > 0.5 else '往上看' if s < -0.5 else '方向不明'
        lines.append(f'tilt：刻度變大 → {way}（桌面 pitch 變化 / tilt 角度變化 = {s:+.2f}，'
                     f'{d["tilt_n"]} 個姿勢；理想值 ±1）')
        if 0.5 < abs(s) < 0.85 or abs(s) > 1.15:
            lines.append('  ⚠ 比例離 1 太遠：可能 tilt 軸和相機的左右軸不平行，或桌面擬合混到別的東西')
    if d['pan_slope'] is None:
        lines.append('pan：資料不夠（要有瓶子在基準姿勢和其他姿勢都被看到、且 pan 至少差 2°）')
    else:
        s = d['pan_slope']
        way = '往左轉' if s > 0.5 else '往右轉' if s < -0.5 else '方向不明'
        lines.append(f'pan：刻度變大 → {way}（瓶子方位角變化 / pan 角度變化 = {s:+.2f}，'
                     f'{d["pan_n"]} 筆；相機往下看，所以會比 1 小一點）')

    r = core.repeatability(rows, bottle_rows)
    if r is None:
        lines.append('頭尾基準姿勢：不到兩次，無法檢查中間有沒有東西動過')
        return lines
    parts = []
    if r['imu_pitch_deg'] is not None:
        parts.append(f'IMU pitch {r["imu_pitch_deg"]:+.2f}° roll {r["imu_roll_deg"]:+.2f}°')
    if r['height_mm'] is not None:
        parts.append(f'桌面高度 {r["height_mm"]:+.1f} mm（桌面角度 pitch {r["pitch_deg"]:+.2f}°，只供參考）')
    if r['bottle_mm']:
        parts.append('瓶子 ' + ' '.join(f'{k} {v:.1f} mm' for k, v in r['bottle_mm'].items()))
    if r['ticks'] is not None:
        parts.append(f'刻度差 {r["ticks"]:.0f} 格')
    lines.append('頭尾基準姿勢的差：' + ('，'.join(parts) if parts else '沒有可比較的資料'))
    # 用 IMU、瓶子、桌面高度判斷。桌面角度不用：深度的小誤差在桌面角度上會放大（sweep_01 誤報過）
    bad = ((r['imu_pitch_deg'] is not None and abs(r['imu_pitch_deg']) > 0.3)
           or (r['imu_roll_deg'] is not None and abs(r['imu_roll_deg']) > 0.3)
           or (r['height_mm'] is not None and abs(r['height_mm']) > 3)
           or any(v > 5 for v in r['bottle_mm'].values()))
    if bad:
        lines.append('  ⚠ 頭尾差太多：中間機器人、桌子或瓶子可能動過，這次資料最好重收')
    else:
        lines.append('  頭尾一致，中間沒有東西動過')
    return lines


def config_dict(cfg):
    return asdict(cfg)
