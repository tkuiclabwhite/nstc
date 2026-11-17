"""dry_run 參數解析（不依賴 ROS，方便單元測試）。

可以只模擬其中幾段，其他接實機，例如 vision = 影像用設定檔的假藥瓶，手臂、手照樣真的動。
平常寫在設定檔 dry_run.modules；launch 有給 dry_run:= 就以 launch 為準。
"""
MODULES = ('vision', 'arm', 'hand')


def parse(value):
    """設定檔的 dry_run.modules 或 launch 的 dry_run → 要模擬的模組集合。

    true / all = 全部模擬；false / 空 = 全部接實機；其他 = vision / arm / hand 的清單或逗號分隔字串。
    """
    if value is None:
        return set()
    if isinstance(value, (list, tuple)):
        value = ','.join(str(v) for v in value)
    s = str(value).strip().lower()
    if s in ('true', 'all'):
        return set(MODULES)
    if s in ('false', 'none', ''):
        return set()
    mods = {m.strip() for m in s.split(',') if m.strip()}
    bad = mods - set(MODULES)
    if bad:
        raise ValueError(f'dry_run 只接受 true / false / all 或 {"、".join(MODULES)} 的組合，'
                         f'收到 {sorted(bad)}')
    return mods
