"""子三錯誤碼（S3-05 第 9 章草案，格式依 M-01 §8：模組-等級-序號）。

主控的 ExecuteTaskPlan.Result.error_code 是 uint8，放不下 'MAN-E-003' 這種字串，
所以 error_code 放序號，完整錯誤碼與說明放在 message 開頭，例如
'MAN-E-003 找不到藥瓶 b1'。0 = 成功。
"""

OK = 0
TIMEOUT = 1             # 任務逾時
OUT_OF_RANGE = 2        # 資料超出範圍（任務參數不合法、目標座標不合理）
BOTTLE_NOT_FOUND = 3    # 找不到藥瓶
GRASP_FAILED = 4        # 抓取失敗（含手臂到不了）
JOINT_STATE_LOST = 5    # 關節狀態中斷
HAND_STATE_LOST = 6     # 手部狀態中斷
DRIVER_ERROR = 7        # 驅動器錯誤
ESTOP = 8               # 急停

NAMES = {
    TIMEOUT: '任務逾時',
    OUT_OF_RANGE: '資料超出範圍',
    BOTTLE_NOT_FOUND: '找不到藥瓶',
    GRASP_FAILED: '抓取失敗',
    JOINT_STATE_LOST: '關節狀態中斷',
    HAND_STATE_LOST: '手部狀態中斷',
    DRIVER_ERROR: '驅動器錯誤',
    ESTOP: '急停',
}


def code_text(code):
    return f'MAN-E-{code:03d}'


class TaskError(Exception):
    """流程中任何一步失敗都丟這個，帶錯誤碼往上傳到 action 的 Result。"""

    def __init__(self, code, detail=''):
        self.code = code
        self.detail = detail
        super().__init__(f'{code_text(code)} {NAMES.get(code, "")} {detail}'.strip())
