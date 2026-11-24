"""pose_demo 錄放（2026-11-24 使用者要的：錄影時不用等規劃、動作比較順）。純 numpy，有單元測試。

正常跑 pose_demo 時，每一步到了之後的關節角（MoveIt 模擬）、從上一步過來途中的模擬路徑都存進 poses.json；
`pose_demo --replay <那次的資料夾>` 照著做，不經 moveit_bridge、不用規劃：
  每一步從上一個姿勢**每顆馬達直接轉過去**（每 STEP_DEG 檢查一次會不會碰到），會碰到的那一段改走上次錄到的路線
  （moveit_bridge 規劃、真的走過的，例如先退開 10 cm 再靠近）。使用者選的（2026-11-24）。
  上次被擋下的測試步（手指朝上／朝下）不送，照上次的結果印出來（使用者選的）。
錄的時候實機補償（moveit_bridge 往上補 0.17 × 前）已經在關節角裡，所以重播時實機跟隨要跟錄的時候一樣（開／關）。
"""
import json
import os

import numpy as np

VERSION = 1
FILE = 'poses.json'
STEP_DEG = 1.0      # 直接轉：每隔這麼多度（最大那一軸）檢查一次碰撞
MIN_DEG = 0.3       # 錄到的路線：跟上一點差不到這麼多的去掉（MoveIt 每 20 ms 一筆，停住時一大堆一樣的）
SAME_DEG = 0.5      # 目前姿勢跟路線起點差這麼多以內，直接把起點換成目前姿勢；超過就先直接轉到起點


def deg(a, b):
    """兩組關節角差最多的那一軸（°）。"""
    return float(np.degrees(np.abs(np.asarray(a, float) - np.asarray(b, float)).max()))


def interp(q0, q1, step_deg=STEP_DEG):
    """每顆馬達從 q0 直接轉到 q1（關節空間直線），每 step_deg 一點，含兩端。"""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    n = max(1, int(np.ceil(deg(q0, q1) / step_deg)))
    return np.linspace(q0, q1, n + 1)


def thin(P, min_deg=MIN_DEG):
    """去掉跟上一點差不到 min_deg 的點；頭尾一定留。"""
    P = np.asarray(P, float)
    if len(P) <= 2:
        return P
    out = [P[0]]
    for q in P[1:-1]:
        if deg(q, out[-1]) >= min_deg:
            out.append(q)
    if deg(P[-1], out[-1]) < min_deg and len(out) > 1:
        out[-1] = P[-1]
    else:
        out.append(P[-1])
    return np.array(out)


def sim_path(samples, joints):
    """reach_test 記的 [(時間, 'sim' / 'real', {名: 值})] → 模擬（MoveIt）的關節角 N×7。
    每筆可能只有部分關節，累積起來；一個都沒有回傳 None。"""
    cur, out = {}, []
    for _, kind, d in samples:
        if kind != 'sim':
            continue
        cur.update({k: v for k, v in d.items() if k in joints})
        if all(j in cur for j in joints):
            out.append([cur[j] for j in joints])
    return thin(out) if out else None


def start_from(P, q_now, step_deg=STEP_DEG):
    """路線 P 改成從目前姿勢 q_now 出發：差很少就把第一點換掉，差多了就先直接轉到 P 的起點。
    回傳 (新路線, 前面多加的那段 或 None)。"""
    P = np.asarray(P, float)
    if deg(P[0], q_now) <= SAME_DEG:
        P = P.copy()
        P[0] = q_now
        return P, None
    head = interp(q_now, P[0], step_deg)
    return np.vstack([head, P[1:]]), head


def segments(steps, q_start, check, step_deg=STEP_DEG):
    """重播要走的每一段。steps = poses.json 的 steps，q_start = 起點（ready）的關節角，
    check(P) → None（不會碰到）或說明文字。
    回傳跟 steps 一樣長的清單：不用動的步（上次沒到，例如被擋下的測試步）是 None，其他是 dict：
      how = 'direct'（直接轉）/ 'recorded'（照上次的路線，why = 直接轉為什麼不行）/ None（兩種都不行，why）
      P = 要走的關節角 N×7（第一點 = 上一步的姿勢，最後一點 = 這一步的姿勢）"""
    out, prev = [], np.asarray(q_start, float)
    for s in steps:
        if s.get('q') is None:
            out.append(None)
            continue
        q = np.asarray(s['q'], float)
        P = interp(prev, q, step_deg)
        why = check(P)
        if why is None:
            out.append({'how': 'direct', 'P': P, 'why': ''})
        elif s.get('path') is not None and len(s['path']) >= 2:
            R, head = start_from(s['path'], prev, step_deg)
            R[-1] = q
            bad = check(head) if head is not None else None
            out.append({'how': 'recorded' if bad is None else None, 'P': R if bad is None else None,
                        'why': why if bad is None else f'{why}；接到上次路線的起點也會碰到：{bad}'})
        else:
            out.append({'how': None, 'P': None, 'why': f'{why}；上次沒錄到路線'})
        prev = q
    return out


def _plain(v):
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, (np.floating, np.integer)):
        return v.item()
    if isinstance(v, dict):
        return {k: _plain(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_plain(x) for x in v]
    return v


def save(d, data):
    """存成 <資料夾>/poses.json。"""
    path = os.path.join(d, FILE)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(_plain(dict(data, version=VERSION)), f, ensure_ascii=False)
    return path


def load(path):
    """讀 poses.json（給資料夾也可以）。格式不對丟 ValueError。"""
    if os.path.isdir(path):
        path = os.path.join(path, FILE)
    if not os.path.exists(path):
        raise ValueError(f'找不到 {path}（要先正常跑一次 pose_demo 才有；2026-11-24 加的，之前的結果沒有）')
    with open(path, encoding='utf-8') as f:
        data = json.load(f)
    if data.get('version') != VERSION:
        raise ValueError(f'{path} 的格式版本 {data.get("version")} 不認得')
    if not any(s.get('q') is not None for s in data.get('steps', [])):
        raise ValueError(f'{path} 裡沒有任何一步到了，沒東西可以重播')
    return data
