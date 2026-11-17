"""可及範圍與避障測試：不經任務管理，直接對 moveit_bridge 下 /arm_move，看結果（ok / reason）。

位置都用桌面座標 [右, 前, 上]（cm，見 task_manager.yaml 的 table），預設掌心在藥瓶中心高度（上 4 cm）。

    ros2 run sp3_task_manager reach_test scan                 # 實際極限（從上方、掌心離桌 10）：往前推到搆不到為止，跟模型比
    ros2 run sp3_task_manager reach_test scan --arm left --up 9   # 矮的藥瓶（b2、b4 約 7.8 高）
    ros2 run sp3_task_manager reach_test scan --dir front --fingers 90 --up 6
    ros2 run sp3_task_manager reach_test obstacles            # 避障：幾種擺法，看結果符不符合預期
    ros2 run sp3_task_manager reach_test point 18.9 12 10     # 移到一點停住（給實機拿捲尺量），按 Enter 回站姿
    ros2 run sp3_task_manager reach_test point 18.9 12 10 -10 5 10 --arm left   # 一次給好幾點（每三個數一點）

掌心離桌太近時手本身會插進桌子（moveit_bridge 回 collision）。URDF 算的掌心最低離桌：
front 手指朝上 14.9 cm、front／side 手指橫放拇指朝上約 5 cm、拇指朝下 11 cm、top 4.5 cm。

obstacles、point 每一項都會存一張圖（俯視、正視、側視：桌子、障礙物、目標、手臂外形），並印出手離每個物體最近幾 cm，
不用開 RViz 也看得到結果。圖在 ~/reach_test_out/<時間>/（--out 可改）。
移動途中也會一直記 /joint_states：模擬（MoveIt）的關節角、實機的馬達刻度（實機跟隨開著時），算「整段路徑上手離物體最近」
在哪個時候、掌心在哪，圖上畫出掌心走過的路線和最靠近那一刻的手臂（模擬 = 綠、實機 = 紫）。

要先開 moveit_bridge（nstc.launch.py 或 armpackage sim.launch.py）。任務管理不要同時跑（它也會發場景）。
實機跟隨開著時手臂會真的動，要加 --real 才會執行；建議先在只有模擬（實機跟隨關著）時跑一次。
"""
import argparse
import datetime
import json
import os
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory, PackageNotFoundError
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from sp3_task_manager import grasp_plan as gp
from sp3_task_manager.arm_shape import ARM_PREFIX, ArmShape, distance

SOURCE = 'sp3_test'        # 場景來源：跟任務管理（sp3）、網頁（web）分開，測完清掉
# 手臂關節名 = 字首 + 這些（armpackage arm_config.JOINT_SUFFIX）；moveit_bridge 結果的 q_deg 依這個順序、右手在前
JOINT_SUFFIX = ['shoulderJoint', 'armJoint', 'arm1Joint', 'forarmJoint', 'forarm1Joint', 'wristJoint', 'wrist1Joint']


class Tester(Node):
    def __init__(self, cfg, args):
        super().__init__('sp3_reach_test')
        self.cfg, self.args = cfg, args
        self.pub_move = self.create_publisher(String, '/arm_move', 10)
        self.pub_stand = self.create_publisher(Bool, '/arm_stand', 10)
        self.pub_obs = self.create_publisher(String, '/arm_ik_obstacles', 10)
        self.pub_speed = self.create_publisher(Float64, '/arm_speed_cmd', 10)
        self.result = None
        self.real = None
        self.create_subscription(String, '/arm_ik_status', self._status_cb, 10)
        self.create_subscription(String, '/arm_real_status', self._real_cb, 10)
        # 移動途中的路徑：[(時間, 'sim' / 'real', {關節名: rad} 或 {馬達 ID: 刻度})]
        self.recording, self.path = False, []
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 50)

    def _joint_cb(self, msg):
        if not self.recording or not msg.name:
            return
        if msg.name[0].isdigit():
            self.path.append((time.monotonic(), 'real', {int(n): p for n, p in zip(msg.name, msg.position)}))
        else:
            self.path.append((time.monotonic(), 'sim', dict(zip(msg.name, msg.position))))

    def _status_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if not d.get('stage_only') and d.get('cmd'):
            self.result = (time.monotonic(), d)

    def _real_cb(self, msg):
        try:
            self.real = json.loads(msg.data)
        except ValueError:
            pass

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, publish, cmd):
        """送出指令等結果；moveit_bridge 還在收尾上一個動作（busy）就等一下再送。"""
        for _ in range(20):
            t0 = time.monotonic()
            publish()
            res = self.wait_result(t0, cmd)
            if res.get('reason') != 'busy':
                self.spin_for(0.3)          # 讓 moveit_bridge 收完尾再送下一個
                return res
            self.spin_for(0.5)
        return res

    def wait_result(self, t0, cmd):
        end = time.monotonic() + self.args.timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            r = self.result
            if r and r[0] > t0 and r[1].get('cmd') == cmd:
                return r[1]
        return {'ok': False, 'reason': 'timeout', 'note': f'{self.args.timeout:g} 秒沒有回報（moveit_bridge 有開嗎？）'}

    def scene(self, objs):
        """發測試場景，問 MoveIt 確認真的套用了（沒有就重發，5 秒還不行就中止測試）。"""
        try:
            from moveit_msgs.msg import PlanningSceneComponents
            from moveit_msgs.srv import GetPlanningScene
        except ImportError:
            self.pub_obs.publish(String(data=json.dumps({'source': SOURCE, 'objects': objs})))
            self.spin_for(0.5)
            return
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        end, tries = time.monotonic() + 5.0, 0
        while True:
            tries += 1
            self.pub_obs.publish(String(data=json.dumps({'source': SOURCE, 'objects': objs})))
            self.spin_for(0.5)
            res = self._call(GetPlanningScene, '/get_planning_scene', req, timeout=2.0)
            if res is not None:
                missing, extra = gp.scene_missing(SOURCE, objs, {o.id for o in res.scene.world.collision_objects})
                if not missing and not extra:
                    if tries > 1:
                        print(f'      （場景發了 {tries} 次 MoveIt 才套用）')
                    return
            if time.monotonic() > end:
                why = '問不到 /get_planning_scene' if res is None else f'少了 {sorted(missing)}、多了 {sorted(extra)}'
                raise RuntimeError(f'測試場景沒有套用到 MoveIt（{why}），停止測試，手臂不動')

    def stand(self):
        return self.send(lambda: self.pub_stand.publish(Bool(data=True)), 'stand')

    def move(self, table_pt, approach, arm, fingers=None):
        """移動並記下途中的 /joint_states（self.path）。"""
        p = gp.table_to_meas(table_pt, self.cfg['table'], self.cfg['frames'])
        d = {'pos': [round(float(v), 1) for v in p], 'dir': approach, 'fingers': fingers, 'arm': arm}
        self.path, self.recording = [], True
        try:
            return self.send(lambda: self.pub_move.publish(String(data=json.dumps(d))), 'move')
        finally:
            self.spin_for(0.5)                 # 實機比模擬晚到，多記一下
            self.recording = False

    def _call(self, srv_type, name, req, timeout=5.0):
        cli = self.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    def moveit_view(self, arm, q):
        """問 MoveIt：場景裡有哪些物體、這隻手擺成 q 有沒有撞到。回傳說明文字。"""
        try:
            from moveit_msgs.msg import PlanningSceneComponents
            from moveit_msgs.srv import GetPlanningScene, GetStateValidity
        except ImportError:
            return '（沒有 moveit_msgs，不能問 MoveIt）'
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        res = self._call(GetPlanningScene, '/get_planning_scene', req)
        names = sorted(o.id for o in res.scene.world.collision_objects) if res else None
        req = GetStateValidity.Request()
        req.group_name = 'right_arm' if arm == 'right' else 'left_arm'
        req.robot_state.is_diff = True
        joints = [n for n in q if n.startswith('R' if arm == 'right' else 'L') and n[1:] in JOINT_SUFFIX]
        req.robot_state.joint_state.name = joints
        req.robot_state.joint_state.position = [float(q[n]) for n in joints]
        res = self._call(GetStateValidity, '/check_state_validity', req)
        if res is None:
            valid = '沒回應'
        elif res.valid:
            valid = '沒撞到'
        else:
            valid = '撞到 ' + '、'.join(sorted({f'{c.contact_body_1}↔{c.contact_body_2}' for c in res.contacts})[:6])
        return f'MoveIt 認為這個姿勢：{valid}｜MoveIt 場景裡的物體：{names if names is not None else "查不到"}'

    def check_controllers(self):
        """兩隻手的 MoveIt 控制器要是 active，不然規劃得出來也執行不了（結果會是 exec_failed / plan_failed）。"""
        try:
            from controller_manager_msgs.srv import ListControllers
        except ImportError:
            print('（沒有 controller_manager_msgs，略過控制器檢查）')
            return True
        cli = self.create_client(ListControllers, '/controller_manager/list_controllers')
        if not cli.wait_for_service(timeout_sec=5.0):
            print('連不到 /controller_manager（手臂 MoveIt 有開嗎？）')
            return False
        fut = cli.call_async(ListControllers.Request())
        end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        states = {c.name: c.state for c in (fut.result().controller if fut.done() else [])}
        bad = {n: states.get(n, '沒載入') for n in ('left_arm_controller', 'right_arm_controller')
               if states.get(n) != 'active'}
        if bad:
            print('手臂控制器沒有全部啟動：' + '、'.join(f'{n} = {s}' for n, s in bad.items()))
            print('  先執行：ros2 control set_controller_state <名稱> inactive，再 ... active（unconfigured 時）')
            print('  或重開 nstc.launch.py')
            return False
        return True

    def check_real(self):
        """實機跟隨開著（手臂會真的動）就要 --real 才繼續。"""
        self.spin_for(1.5)
        follow = (self.real or {}).get('state') == 'follow'
        print(f'實機跟隨：{"開著（手臂會真的動）" if follow else "關著（只動模擬）"}')
        if follow and not self.args.real:
            print('實機跟隨開著，要真的動手臂請加 --real；只想測模擬就先關掉實機跟隨。')
            return False
        return True


def short(res):
    note = (res.get('note') or '').split('\n')[0]
    return f'{"OK " if res.get("ok") else "NG "} {res.get("arm") or "-":5s} {res.get("reason") or "":12s} {note[:60]}'


def table_with_scene(cfg, extra):
    """場景：桌子 + 這次測試加的物體。"""
    return [dict(gp.table_obstacle(cfg['table'], cfg['frames']), id=f'{SOURCE}_table')] + extra


def model_front(cfg, arm, right, up):
    """設定檔的球模型：這個左右位置、高度，這隻手最遠能往前多少（cm）；搆不到回 None。"""
    best = None
    for f in np.arange(0.0, 80.0, 0.5):
        if arm in gp.reachable_arms(gp.table_to_meas((right, f, up), cfg['table'], cfg['frames']), cfg):
            best = f
    return best


def run_scan(t, cfg, args):
    """每個左右位置，從桌緣往前每 step cm 試一次，記下最遠成功的位置，跟模型比。"""
    t.scene(table_with_scene(cfg, []))
    side = 1.0 if args.arm == 'right' else -1.0        # 右手測右半邊，左手測左半邊（鏡像）
    rights = [side * r for r in args.rights]
    print(f'\n== 實際極限（{args.arm} 手、掌心方向 {args.dir}、手指 {args.fingers}、掌心離桌 {args.up:g} cm）')
    print(f'{"右":>7s} {"模型最遠前":>9s} {"實際最遠前":>9s}  最後一次失敗')
    rows = []
    for r in rights:
        t.stand()
        model = model_front(cfg, args.arm, r, args.up)
        last_ok, last_fail, reasons = None, None, {}
        stop = (model if model is not None else 0.0) + args.beyond
        for f in np.arange(0.0, stop + 1e-6, args.step):
            res = t.move((r, float(f), args.up), args.dir, args.arm, args.fingers)
            if res.get('ok'):
                last_ok = float(f)
            else:
                last_fail = (float(f), res)
                reasons.setdefault(res.get('reason'), []).append(f'{f:g}')
        fail_txt = f'前 {last_fail[0]:g}：{short(last_fail[1])}' if last_fail else '—'
        print(f'{r:7.1f} {model if model is not None else "搆不到":>9} '
              f'{last_ok if last_ok is not None else "都到不了":>9}  {fail_txt}')
        for why, fs in reasons.items():
            print(f'{"":19s}  {why}：前 {"、".join(fs)}')
        rows.append((r, model, last_ok))
    t.stand()
    t.scene([])
    diffs = [ok - m for _, m, ok in rows if m is not None and ok is not None]
    if diffs:
        print(f'\n實際 − 模型：平均 {np.mean(diffs):+.1f} cm（正 = 實際比模型遠，模型偏保守；負 = 模型太寬）')


# ----------------------------------------------------------------------
# 不用 RViz 看結果：用回報的關節角算手臂外形，畫圖、算離障礙物多近
# ----------------------------------------------------------------------
def joint_q(res, arm):
    """moveit_bridge 結果的 q_deg（度，14 個 = 右手 7 + 左手 7）→ {關節名: rad}。"""
    qd = res.get('q_deg') or []
    order = ['right', 'left'] if len(qd) == 14 else [res.get('arm') or arm]
    q = {}
    for i, a in enumerate(order):
        names = [('R' if a == 'right' else 'L') + s for s in JOINT_SUFFIX]
        q.update(zip(names, np.radians(qd[7 * i:7 * i + 7])))
    return q


def to_table(P, cfg):
    """base_link（m，N×3）→ 桌面座標 [右, 前, 上]（cm）。"""
    P = np.atleast_2d(np.asarray(P, float))
    o = gp.table_to_meas((0.0, 0.0, 0.0), cfg['table'], cfg['frames'])
    m = np.array([gp.to_meas(p, cfg['frames']) for p in P])
    return np.column_stack([m[:, 1] - o[1], m[:, 0] - o[0], m[:, 2] - o[2]])


def real_ticks_to_q():
    """回傳 f({馬達 ID: 刻度}) → {關節名: rad}，用 armpackage 的 arm_config（stand.ini 的偏移、轉向）。
    讀不到就回 None（只畫模擬的路徑）。"""
    try:
        import sys
        sys.path.insert(0, os.path.join(get_package_prefix('armpackage'), 'lib', 'armpackage'))
        import arm_config
    except Exception as e:                                   # noqa: BLE001
        print(f'（讀不到 armpackage 的 arm_config，只看模擬的路徑：{e}）')
        return None

    def conv(ticks):
        q = {}
        for arm, a in arm_config.ARMS.items():
            if all(m in ticks for m in a['motors']):
                q.update(zip(a['joints'], arm_config.ticks_to_rad(arm, [ticks[m] for m in a['motors']])))
        return q
    return conv


def path_frames(path, kind, conv, n=60):
    """錄到的路徑 → [(秒, {關節名: rad})]，最多 n 個（平均取樣）。sim 的訊息可能只有部分關節，累積起來。"""
    rows = [r for r in path if r[1] == kind]
    if not rows:
        return []
    t0, out, acc = path[0][0], [], {}
    for t, _, d in rows:
        if kind == 'sim':
            acc.update(d)
            out.append((t - t0, dict(acc)))
        elif conv is not None:
            out.append((t - t0, conv(d)))
    if len(out) > n:
        idx = np.linspace(0, len(out) - 1, n).round().astype(int)
        out = [out[i] for i in idx]
    return out


def path_clearance(shape, frames, arm, objs, cfg):
    """每一格算手臂離物體多近。回傳 (最近 (秒, id, cm), 那一刻的手臂頂點, 掌心走過的點 base_link)。"""
    best, ghost, trail, best_q = None, None, [], None
    palm = np.array([0.02, 0.0, 0.07, 1.0])                 # arm_config.TOOL_OFFSET（手掌 link 座標）
    hand_base = 'rh_hand_base_link' if arm == 'right' else 'lh_hand_base_link'
    for t, q in frames:
        if not q:
            continue
        trail.append((shape.link_T(hand_base, q) @ palm)[:3])
        pts = shape.points(q, arm)
        for oid, d, _ in clearances(pts, objs):
            if best is None or d < best[2]:
                best, ghost, best_q = (t, oid, d), pts, q
    return best, ghost, np.array(trail), best_q


def load_shape():
    try:
        urdf = os.path.join(get_package_share_directory('seven_axis_arm_urdf'), 'urdf', 'seven_axis_arm_urdf.urdf')
        return ArmShape(urdf, get_package_share_directory)
    except (PackageNotFoundError, OSError) as e:
        print(f'（讀不到手臂 URDF，不畫圖、不算距離：{e}）')
        return None


def clearances(pts, objs):
    """手臂頂點到每個物體表面的最短距離（cm）與最近的點：[(id, cm, 點)]。"""
    out = []
    for o in objs:
        d, p = distance(pts, o)
        if d is not None:
            out.append((o['id'], d * 100.0, p))
    return out


def draw_case(path, title, cfg, objs, target, arm_pts, hand_pts, line, near, paths=()):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    plt.rcParams['font.sans-serif'] = ['Noto Sans CJK TC', 'Noto Serif CJK TC', 'Noto Sans CJK JP', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    views = [('俯視（上 → 下看）', 0, 1, '右 (cm)', '前 (cm)'),
             ('正視（從機器人往前看）', 0, 2, '右 (cm)', '上 (cm)'),
             ('側視（從右邊看）', 1, 2, '前 (cm)', '上 (cm)')]
    fig, axes = plt.subplots(1, 3, figsize=(18, 6.5))
    A, H, L, T = to_table(arm_pts, cfg), to_table(hand_pts, cfg), to_table(line, cfg), np.asarray(target)
    for ax, (name, i, j, xl, yl) in zip(axes, views):
        for o in objs:
            if o['type'] == 'box':
                c = to_table(o['c'], cfg)[0]
                h = np.array([o['h'][1], o['h'][0], o['h'][2]]) * 100.0      # base_link x,y,z 半長 → 右、前、上
                is_table = o['id'].endswith('table')
                ax.add_patch(Rectangle((c[i] - h[i], c[j] - h[j]), 2 * h[i], 2 * h[j],
                                       color='0.85' if is_table else 'tab:orange', alpha=0.6 if is_table else 0.5,
                                       zorder=0 if is_table else 2))
            elif o['type'] == 'stick':
                a, b = to_table(o['a'], cfg)[0], to_table(o['b'], cfg)[0]
                r = o['r'] * 100.0
                if j == 1:                                                    # 俯視：圓
                    ax.add_patch(Circle((a[0], a[1]), r, color='tab:orange', alpha=0.5, zorder=2))
                else:
                    ax.add_patch(Rectangle((a[i] - r, a[2]), 2 * r, b[2] - a[2], color='tab:orange', alpha=0.5,
                                           zorder=2))
        ax.scatter(A[:, i], A[:, j], s=1, c='tab:blue', alpha=0.25, zorder=3, label='手臂')
        ax.scatter(H[:, i], H[:, j], s=1, c='navy', alpha=0.5, zorder=4, label='手')
        ax.plot(L[:, i], L[:, j], '-o', c='k', ms=3, lw=1, zorder=5)
        ax.plot(T[i], T[j], 'rx', ms=12, mew=3, zorder=6, label='目標掌心')
        for color, trail, ghost in paths:                      # 路徑：掌心走過的點、最靠近物體那一刻的手臂
            if ghost is not None:
                G = to_table(ghost, cfg)
                ax.scatter(G[:, i], G[:, j], s=1, c=color, alpha=0.15, zorder=3)
            if len(trail):
                R = to_table(trail, cfg)
                ax.plot(R[:, i], R[:, j], '-', c=color, lw=1.5, zorder=6)
        for oid, d, p in near:
            q = to_table(p, cfg)[0]
            ax.plot(q[i], q[j], 'o', mfc='none', mec='red' if d < 0 else 'green', ms=8, zorder=6)
        ax.axhline(0, c='0.5', lw=0.8) if j == 2 else ax.axhline(0, c='0.5', lw=0.8, ls='--')
        ax.set_title(name)
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.set_aspect('equal')
        ax.set_xlim((-55, 55) if i == 0 else (-30, 45))
        ax.set_ylim((-30, 45) if j == 1 else (-30, 60))
        ax.grid(alpha=0.3)
    from matplotlib.lines import Line2D
    axes[0].legend(handles=[Line2D([], [], ls='', marker='o', ms=5, c='tab:blue', alpha=0.5, label='手臂'),
                            Line2D([], [], ls='', marker='o', ms=5, c='navy', label='手'),
                            Line2D([], [], ls='', marker='x', ms=8, mew=2, c='r', label='目標掌心'),
                            Line2D([], [], ls='', marker='o', ms=8, mfc='none', mec='green', label='離物體最近的點'),
                            Rectangle((0, 0), 1, 1, color='tab:orange', alpha=0.5, label='障礙物'),
                            Rectangle((0, 0), 1, 1, color='0.85', label='桌子'),
                            Line2D([], [], c='green', label='路徑：模擬（淡色 = 最靠近那一刻）'),
                            Line2D([], [], c='purple', label='路徑：實機馬達讀數')],
                   loc='upper left', fontsize=8)
    txt = '　'.join(f'{oid.replace(SOURCE + "_", "")} {d:+.1f} cm' for oid, d, _ in near)
    fig.suptitle(f'{title}\n手離物體最近（含障礙物的餘裕；負的 = 插進去）：{txt}', fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def bottle(name, right, front, cfg_dims, cfg, margin):
    """桌上的一瓶藥（桌面座標的左右、前；直徑、高照 bottle_dims）→ 場景物體。"""
    dia, h = cfg_dims[name]
    c = gp.from_meas(gp.table_to_meas((right, front, h * 50.0), cfg['table'], cfg['frames']), cfg['frames'])
    return gp.bottle_obstacle(name, c, (dia, dia, h), margin, oid=f'{SOURCE}_{name}_{right:g}_{front:g}')


def describe(o, cfg):
    """場景物體 → 實機要怎麼擺（桌面座標，cm），給人照著放實體物品。"""
    if o['type'] == 'stick':
        a, b = to_table(o['a'], cfg)[0], to_table(o['b'], cfg)[0]
        m = cfg['obstacles']['margin_cm']
        return (f'藥瓶：中心 右 {a[0]:.1f}、前 {a[1]:.1f}；直徑 {o["r"] * 200 - 2 * m:.1f}、高 {b[2] - a[2] - 2 * m:.1f} cm'
                f'（場景裡各多 {m:g} cm 餘裕）')
    if o['type'] == 'box':
        c = to_table(o['c'], cfg)[0]
        h = np.array([o['h'][1], o['h'][0], o['h'][2]]) * 200.0
        return f'方塊：中心 右 {c[0]:.1f}、前 {c[1]:.1f}、上 {c[2]:.1f}；寬 {h[0]:.1f}、深 {h[1]:.1f}、厚 {h[2]:.1f} cm'
    return o['type']


def report_pose(t, cfg, shape, res, target, objs, arm, png, title):
    """印手離物體最近的距離（終點、整段路徑），存圖。"""
    if shape is None or not res.get('q_deg'):
        return
    paths = []
    conv = real_ticks_to_q() if any(k == 'real' for _, k, _ in t.path) else None
    for kind, color, label in (('sim', 'green', '模擬'), ('real', 'purple', '實機馬達讀數')):
        frames = path_frames(t.path, kind, conv)
        if not frames:
            continue
        best, ghost, trail, best_q = path_clearance(shape, frames, arm, objs, cfg)
        paths.append((color, trail, ghost))
        if best is not None:
            tt, oid, d = best
            k = int(np.argmin([abs(f[0] - tt) for f in frames]))
            at = to_table(trail[min(k, len(trail) - 1)], cfg)[0] if len(trail) else None
            where = f'，那時掌心在 右 {at[0]:.1f}、前 {at[1]:.1f}、上 {at[2]:.1f}' if at is not None else ''
            print(f'      路徑上手離物體最近（{label}，{len(frames)} 格）：{oid.replace(SOURCE + "_", "")} '
                  f'{d:+.1f} cm，第 {tt:.1f} 秒{where}')
            if kind == 'sim' and d < 0:          # 模擬的路徑插進物體：直接問 MoveIt 那一刻的姿勢
                print('      ⚠ 模擬路徑插進物體。' + t.moveit_view(arm, best_q))
    q = joint_q(res, arm)
    pts = shape.points(q, arm)
    hand_links = {k for k in shape.meshes if k.startswith(ARM_PREFIX[arm][1])}
    hand = np.vstack([shape.points_of(q, l) for l in hand_links]) if hand_links else pts[:0]
    near = clearances(pts, objs)
    where = '' if res.get('ok') else '（沒有動，這是停在原地的姿勢）'
    print(f'      手離物體最近{where}：' + '、'.join(f'{oid.replace(SOURCE + "_", "")} {d:+.1f} cm'
                                            for oid, d, _ in near) + '（含餘裕；負的 = 插進去）')
    draw_case(png, title, cfg, objs, target, pts, hand, shape.skeleton(q, arm), near, paths)
    print(f'      圖：{png}')


def out_dir(args, what):
    d = os.path.expanduser(os.path.join(args.out, datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                                        + f'_{what}_{args.arm}'))
    os.makedirs(d, exist_ok=True)
    return d


def run_point(t, cfg, args):
    """移到指定的幾個點，每點停住讓人量（實機），按 Enter 回站姿再做下一點。"""
    v = args.values
    if not v or len(v) % 3:
        print('point 要給 3 的倍數個數字：右 前 上（桌面座標 cm）…')
        return
    pts = [tuple(v[i:i + 3]) for i in range(0, len(v), 3)]
    shape, d = load_shape(), out_dir(args, 'point')
    objs = table_with_scene(cfg, [])
    t.scene(objs)
    print(f'\n== 移到指定點（{args.arm} 手、掌心方向 {args.dir}、手指 {args.fingers}）')
    print('   掌心點 = 手掌面上、左右置中、離手掌底部（接手腕那端）7 cm。量：右 = 離機器人中線、前 = 離桌緣、上 = 離桌面')
    for n, p in enumerate(pts, 1):
        t.stand()
        model = 'right' if args.arm == 'right' else 'left'
        reach = model in gp.reachable_arms(gp.table_to_meas(p, cfg['table'], cfg['frames']), cfg)
        res = t.move(p, args.dir, args.arm, args.fingers)
        print(f'{n}. 目標 右 {p[0]:g}、前 {p[1]:g}、上 {p[2]:g}（設定檔範圍內：{"是" if reach else "否"}）\n      {short(res)}')
        report_pose(t, cfg, shape, res, p, objs, args.arm, os.path.join(d, f'point{n}.png'),
                    f'點 {n}：右 {p[0]:g} 前 {p[1]:g} 上 {p[2]:g}｜{short(res)}')
        if res.get('ok'):
            input('      量好掌心的 右／前／上 後按 Enter（回站姿）…')
    t.stand()
    t.scene([])


def run_obstacles(t, cfg, args):
    """幾種擺法，每種都從站姿出發，看結果符不符合預期。"""
    dims = {'b1': (0.0829, 0.0923), 'b2': (0.04943, 0.07803), 'b3': (0.04485, 0.08988), 'b4': (0.03785, 0.07924)}
    m = cfg['obstacles']['margin_cm']
    arm = args.arm
    s = 1.0 if arm == 'right' else -1.0
    r0 = s * 18.9                      # 肩膀正前方
    tgt = (r0, args.front, args.up)    # 掌心目標
    big = lambda right, front: bottle('b1', right, front, dims, cfg, m)
    near = 7.0                         # 旁邊一瓶的中心距離：b1 半徑 4.15 + 餘裕，手會碰到
    cases = [        # (名稱, 加的物體, 掌心目標, 方向, 預期：True 到得了 / False 不行 / None 只看結果)
        ('1 沒有障礙物', [], tgt, 'top', True),
        ('2 旁邊 15 cm 有一瓶', [big(r0 + 15.0 * s, args.front)], tgt, 'top', True),
        ('3 旁邊 7 cm 緊貼一瓶（手會碰到）', [big(r0 + near * s, args.front)], tgt, 'top', False),
        ('4 前面 15 cm、另一側 15 cm 各一瓶', [big(r0, args.front + 15.0), big(r0 - 15.0 * s, args.front)], tgt, 'top', True),
        ('5 目標上方 15 cm 有板子', [{'id': f'{SOURCE}_ceiling', 'type': 'box',
                                   'c': list(gp.from_meas(gp.table_to_meas((r0, args.front, args.up + 15.0),
                                                                           cfg['table'], cfg['frames']), cfg['frames'])),
                                   'R': [[1, 0, 0], [0, 1, 0], [0, 0, 1]], 'h': [0.10, 0.10, 0.005]}],
         tgt, 'top', False),
        ('6 目標在桌面底下（上 −3）', [], (r0, args.front, -3.0), 'top', False),
    ]
    print(f'\n== 避障（{arm} 手，從上方，目標掌心 = 桌面座標 {list(tgt)}，手指 {args.fingers}）')
    shape, d = load_shape(), out_dir(args, 'obstacles')
    only = set(args.cases or [])
    passed = total = 0
    for n, (name, extra, target, approach, expect) in enumerate(cases, 1):
        if only and n not in only:
            continue
        total += 1
        t.scene(table_with_scene(cfg, []))
        t.stand()
        objs = table_with_scene(cfg, extra)
        t.scene(objs)
        print(f'{name}：目標掌心 右 {target[0]:g}、前 {target[1]:g}、上 {target[2]:g}')
        for o in extra:
            print(f'      擺：{describe(o, cfg)}')
        if args.pause and extra:
            input('      實機：照上面擺好實體物品（用輕的、軟的）後按 Enter，手臂開始動…')
        res = t.move(target, approach, arm, args.fingers)
        good = expect is None or bool(res.get('ok')) == expect
        passed += good
        want = '只看結果' if expect is None else ('到得了' if expect else '不行')
        print(f'{"PASS" if good else "FAIL"}  （預期 {want}）\n      {short(res)}')
        report_pose(t, cfg, shape, res, target, objs, arm, os.path.join(d, f'case{n}.png'),
                    f'{name}｜{"PASS" if good else "FAIL"}｜{short(res)}')
        if args.pause:
            input('      量好、看完按 Enter（回站姿、下一項）…')
    t.scene([])
    t.stand()
    print(f'\n{passed}/{total} 符合預期')


def main():
    ap = argparse.ArgumentParser(description='可及範圍與避障測試（直接對 moveit_bridge）')
    ap.add_argument('mode', choices=['scan', 'obstacles', 'point'])
    ap.add_argument('values', nargs='*', type=float, help='point 的點：右 前 上（桌面座標 cm），可以給好幾組')
    ap.add_argument('--cases', type=int, nargs='+', help='obstacles 只做這幾項（例如 --cases 1 2 3）')
    ap.add_argument('--arm', choices=['left', 'right'], default='right')
    ap.add_argument('--dir', default='top', help='掌心方向（scan 用）：top（預設，從上方抓）/ front / side / auto …')
    ap.add_argument('--up', type=float, default=10.0,
                    help='掌心離桌面（cm），預設 10 = 最高的 b1（9.2）頂上 1 cm，任務管理從上方抓時的高度')
    ap.add_argument('--fingers', type=float, default=None,
                    help='手指朝向（度），不給 = 讓手臂自己挑；front 時 0 = 手指朝上、±90 = 橫放（+ 往外）')
    ap.add_argument('--rights', type=float, nargs='+', default=[0, 10, 18.9, 30, 40, 50],
                    help='scan 的左右位置（右手往右、左手往左，cm）')
    ap.add_argument('--step', type=float, default=2.0, help='scan 往前每次多少 cm')
    ap.add_argument('--beyond', type=float, default=8.0, help='scan 試到模型極限再往前多少 cm')
    ap.add_argument('--front', type=float, default=12.0, help='obstacles 的目標離桌緣多遠（cm）')
    ap.add_argument('--speed', type=float, default=30.0, help='/arm_speed_cmd（Dynamixel 單位）')
    ap.add_argument('--timeout', type=float, default=30.0, help='每個指令最多等幾秒')
    ap.add_argument('--pause', action='store_true', help='obstacles 每一項做完停下來讓你看')
    ap.add_argument('--real', action='store_true', help='實機跟隨開著也照樣執行（手臂會真的動）')
    ap.add_argument('--out', default='~/reach_test_out', help='obstacles 的圖存在這個資料夾底下')
    ap.add_argument('--config', default=os.path.join(get_package_share_directory('sp3_task_manager'),
                                                     'config', 'task_manager.yaml'))
    args = ap.parse_args()
    with open(args.config, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    rclpy.init()
    t = Tester(cfg, args)
    try:
        if not t.check_controllers() or not t.check_real():
            return
        t.pub_speed.publish(Float64(data=args.speed))
        {'scan': run_scan, 'obstacles': run_obstacles, 'point': run_point}[args.mode](t, cfg, args)
    except KeyboardInterrupt:
        print('\n中斷：清掉測試場景')
        try:
            t.scene([])
        except RuntimeError as e:
            print(f'⚠ {e}')
    except RuntimeError as e:
        print(f'\n⚠ {e}')
    finally:
        t.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
