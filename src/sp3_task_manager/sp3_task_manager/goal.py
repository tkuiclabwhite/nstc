"""把主控送來的 parameter_keys / parameter_values 解析成取放任務。

欄位照 S3-05 IF-SP3-001 PickPlace 的 goal：
    task_id       T-YYYYMMDD-NNNN，沒給就自動產生
    bottle_class  藥瓶種類（必填，要在設定檔的 bottle_classes 裡）
    arm           auto / left / right（也接受 S3-05 的 0 / 1 / 2）
    place_target  放置點名稱（要在設定檔的 place_targets 裡）；空字串 = 抓起來就好，不放
    timeout_s     任務逾時（秒），預設看設定檔
不認得的 key、重複的 key 一律拒絕，免得打錯字被默默忽略。
"""
import re
import time
from dataclasses import dataclass

from sp3_task_manager.errors import OUT_OF_RANGE, TaskError

KEYS = ('task_id', 'bottle_class', 'arm', 'place_target', 'timeout_s')
ARM_ALIASES = {'auto': 'auto', '0': 'auto', 'left': 'left', '1': 'left',
               'right': 'right', '2': 'right'}
TASK_ID_RE = re.compile(r'^T-\d{8}-\d{4}$')


@dataclass
class PickPlaceGoal:
    task_id: str
    bottle_class: str
    arm: str
    place_target: str
    timeout_s: float


def parse(keys, values, cfg):
    """回傳 PickPlaceGoal；不合法時丟 TaskError(OUT_OF_RANGE)，message 寫哪一欄錯。"""
    keys = [k.strip() for k in keys]
    values = [v.strip() for v in values]
    if len(keys) != len(values):
        raise TaskError(OUT_OF_RANGE, f'keys {len(keys)} 筆、values {len(values)} 筆，數量不同')
    d = {}
    for k, v in zip(keys, values):
        if k not in KEYS:
            raise TaskError(OUT_OF_RANGE, f'不認得的欄位 {k!r}，可用：{", ".join(KEYS)}')
        if k in d:
            raise TaskError(OUT_OF_RANGE, f'欄位 {k!r} 重複')
        d[k] = v

    bottle = d.get('bottle_class', '')
    if not bottle:
        raise TaskError(OUT_OF_RANGE, '缺少 bottle_class')
    if bottle not in cfg['bottle_classes']:
        raise TaskError(OUT_OF_RANGE,
                        f'bottle_class {bottle!r} 不在 {cfg["bottle_classes"]}')

    arm = ARM_ALIASES.get(d.get('arm', 'auto').lower())
    if arm is None:
        raise TaskError(OUT_OF_RANGE, f'arm 只接受 auto / left / right，收到 {d["arm"]!r}')

    place = d.get('place_target', '')
    if place and place not in cfg['place_targets']:
        raise TaskError(OUT_OF_RANGE,
                        f'place_target {place!r} 不在 {sorted(cfg["place_targets"])}')

    try:
        timeout = float(d.get('timeout_s', cfg['timeouts']['task_s']))
    except ValueError:
        raise TaskError(OUT_OF_RANGE, f'timeout_s 不是數字：{d["timeout_s"]!r}') from None
    if not 0 < timeout <= cfg['timeouts']['task_max_s']:
        raise TaskError(OUT_OF_RANGE,
                        f'timeout_s 要在 0 ~ {cfg["timeouts"]["task_max_s"]} 秒，收到 {timeout}')

    task_id = d.get('task_id') or time.strftime('T-%Y%m%d-') + time.strftime('%H%M')
    if not TASK_ID_RE.match(task_id):
        raise TaskError(OUT_OF_RANGE, f'task_id 格式要是 T-YYYYMMDD-NNNN，收到 {task_id!r}')

    return PickPlaceGoal(task_id, bottle, arm, place, timeout)
