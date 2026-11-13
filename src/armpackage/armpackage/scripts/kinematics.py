"""
從 URDF 算 FK（只用 numpy），加上旋轉矩陣 ↔ 四元數、場景物體的幾何小工具

    fk = ChainFK(urdf_path, "base_link", "rh_hand_base_link", tool_offset=(0.02, 0, 0.07))
    T = fk(q)          # 4x4，掌心在 base_link 的位姿；q 依 fk.joints 的順序
"""
import xml.etree.ElementTree as ET
import numpy as np


def rot_rpy(r, p, y):
    """URDF 的 rpy：R = Rz(y) @ Ry(p) @ Rx(r)"""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rot_axis(axis, q):
    """Rodrigues 公式：繞單位向量 axis 轉 q 弧度"""
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


def make_T(R=None, p=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    """旋轉矩陣 → (x, y, z, w)（Shepperd 法，數值穩定）"""
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1)
        q = ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4)
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
        q[i], q[j], q[k] = s / 4, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return tuple(q / np.linalg.norm(q))


def _origin(j):
    o = j.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    return make_T(rot_rpy(*rpy), np.array(xyz))


class ChainFK:
    """URDF 裡 base → tip 這一串的 FK；fixed joint 直接乘進去，可動關節依序對應 q"""

    def __init__(self, urdf_path, base, tip, tool_offset=(0, 0, 0)):
        by_child = {j.find("child").get("link"): j for j in ET.parse(urdf_path).getroot().findall("joint")}
        chain, link = [], tip
        while link != base:
            if link not in by_child:
                raise ValueError(f"URDF 裡從 {tip} 往回找不到 {base}")
            chain.append(by_child[link])
            link = by_child[link].find("parent").get("link")
        self.steps, self.joints = [], []          # steps：(原點, 軸 或 None=fixed)
        for j in reversed(chain):
            if j.get("type") in ("revolute", "continuous"):
                axis = np.array([float(v) for v in j.find("axis").get("xyz").split()])
                self.steps.append((_origin(j), axis))
                self.joints.append(j.get("name"))
            else:
                self.steps.append((_origin(j), None))
        self.T_tool = make_T(p=np.array(tool_offset, float))

    def __call__(self, q):
        T, i = np.eye(4), 0
        for origin, axis in self.steps:
            T = T @ origin
            if axis is not None:
                T = T @ make_T(rot_axis(axis, q[i]))
                i += 1
        return T @ self.T_tool


# ---------- 場景物體（格式見網頁送的 /arm_ik_obstacles）----------
def parse_obstacle(o):
    o = dict(o)
    for k in ("c", "a", "b", "h", "p", "n", "R"):
        if k in o:
            o[k] = np.asarray(o[k], float)
    return o


def obstacle_center(ob):
    if ob["type"] == "stick":
        return (np.asarray(ob["a"]) + ob["b"]) / 2
    if ob["type"] == "plane":
        return np.asarray(ob["p"], float)
    return np.asarray(ob["c"], float)


def support(ob, d):
    """物體沿方向 d（單位向量）從中心到表面的距離：抓取時手掌要停在中心往 -d 這麼遠的地方"""
    k = ob["type"]
    if k == "ball":
        return ob["r"]
    if k == "box":
        return float(np.abs(np.asarray(ob["R"]).T @ d) @ ob["h"])
    if k == "stick":
        ax = np.asarray(ob["b"]) - ob["a"]
        return ob["r"] + 0.5 * abs(ax @ d)
    return 0.0


# ---------- IK（阻尼最小平方，從 ~/Arm_IK/arm_ik.py 移植）----------
def chain_limits(urdf_path, joints, overrides=None):
    """URDF 的關節範圍；overrides = {關節: (lo, hi)}（例如 MoveIt joint_limits.yaml 的線材保護）會再取交集"""
    lim = {j.get("name"): j.find("limit") for j in ET.parse(urdf_path).getroot().findall("joint")}
    lo = np.array([float(lim[n].get("lower", -np.pi)) for n in joints])
    hi = np.array([float(lim[n].get("upper", np.pi)) for n in joints])
    for i, n in enumerate(joints):
        if overrides and n in overrides:
            lo[i], hi[i] = max(lo[i], overrides[n][0]), min(hi[i], overrides[n][1])
    return lo, hi


def log_rot(R):
    """旋轉矩陣 → 旋轉向量（單位軸 × 角度）"""
    th = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if th < 1e-9:
        return np.zeros(3)
    if np.pi - th < 1e-6:
        A = (R + np.eye(3)) / 2
        k = A[:, np.argmax(np.diag(A))]
        return th * k / np.linalg.norm(k)
    return th / (2 * np.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


class ChainIK(ChainFK):
    """
    solve(p, q0, R=...)     位置 + 整個姿態
    solve(p, q0, axis=a)    位置 + 「掌心法線（工具座標 +X）對準 a」，繞法線轉多少都可以（讓 IK 自己挑轉得到的角度）
    solve(p, q0)            只對位置
    """

    def __init__(self, urdf_path, base, tip, tool_offset=(0, 0, 0), lower=None, upper=None):
        super().__init__(urdf_path, base, tip, tool_offset)
        self.lower = np.full(len(self.joints), -np.pi) if lower is None else np.asarray(lower, float)
        self.upper = np.full(len(self.joints), np.pi) if upper is None else np.asarray(upper, float)

    def jacobian(self, q):
        """幾何 Jacobian（base 座標）：上三列線速度、下三列角速度"""
        T, i, zs, ps = np.eye(4), 0, [], []
        for origin, axis in self.steps:
            T = T @ origin
            if axis is not None:
                zs.append(T[:3, :3] @ (axis / np.linalg.norm(axis)))
                ps.append(T[:3, 3].copy())
                T = T @ make_T(rot_axis(axis, q[i]))
                i += 1
        T = T @ self.T_tool
        Z = np.array(zs)
        return T, np.vstack([np.cross(Z, T[:3, 3] - np.array(ps)).T, Z.T])

    def solve(self, p, q0, R=None, axis=None, q_rest=None, iters=200, damping=1e-4, null_gain=0.05,
              limit_gain=0.2, max_step=0.3, tol_p=1e-4, tol_r=1e-3, stall=20):
        """回傳 (q, 是否收斂)。零空間：往 q_rest 靠，加上快碰到關節極限時往中間推（比較不會卡在極限上解不出來）
        stall：連續這麼多次誤差都沒有比最好的小 2% 就放棄（2026-11-12）。到不了的目標通常幾十次就卡在關節極限上，
        以前每個起點都跑滿 iters，到不了的時候很慢（實機的 AGX 上 6 個方向要 1 分鐘）。
        離線比過 11 種情況：找到的解數量跟不提早放棄一模一樣，快 4 倍。None = 不提早放棄"""
        q = np.clip(np.asarray(q0, float), self.lower, self.upper)
        q_rest = q.copy() if q_rest is None else np.asarray(q_rest, float)
        mid, half = (self.lower + self.upper) / 2, (self.upper - self.lower) / 2
        n = len(q)
        best, since = np.inf, 0
        for _ in range(iters):
            T, J = self.jacobian(q)
            e_p = np.asarray(p) - T[:3, 3]
            if R is not None:
                e_r = log_rot(np.asarray(R) @ T[:3, :3].T)
                Jf, e = J, np.concatenate([e_p, e_r])
            elif axis is not None:
                a = np.asarray(axis, float) / np.linalg.norm(axis)
                x = T[:3, 0]
                c, cos_ = np.cross(x, a), np.clip(x @ a, -1, 1)
                s_ = np.linalg.norm(c)
                if s_ > 1e-9:
                    e_r = np.arctan2(s_, cos_) * c / s_
                elif cos_ < 0:
                    v = np.cross(x, np.eye(3)[np.argmin(np.abs(x))])
                    e_r = np.pi * v / np.linalg.norm(v)
                else:
                    e_r = np.zeros(3)
                Jf, e = np.vstack([J[:3], (np.eye(3) - np.outer(x, x)) @ J[3:]]), np.concatenate([e_p, e_r])
            else:
                e_r, Jf, e = np.zeros(3), J[:3], e_p
            np_, nr = np.linalg.norm(e_p), np.linalg.norm(e_r)
            if np_ < tol_p and nr < tol_r:
                return q, True
            err = np_ / tol_p + nr / tol_r
            if err < best * 0.98:
                best, since = err, 0
            else:
                since += 1
                if stall and since >= stall:
                    return q, False
            Jp = Jf.T @ np.linalg.inv(Jf @ Jf.T + damping * np.eye(len(e)))
            N = np.eye(n) - np.linalg.pinv(Jf, rcond=1e-6) @ Jf
            u = (q - mid) / half                                       # -1 ~ 1，±1 = 在極限上
            push = -np.where(np.abs(u) > 0.8, (np.abs(u) - 0.8) / 0.2, 0.0) * np.sign(u) * half
            dq = Jp @ e + N @ (null_gain * (q_rest - q) + limit_gain * push)
            dq *= min(1.0, max_step / (np.abs(dq).max() + 1e-12))
            q = np.clip(q + dq, self.lower, self.upper)
        return q, False

    def solve_many(self, p, seeds, R=None, axis=None, same_tol=0.1, max_sols=None, **kw):
        """從很多起點各解一次，回傳收斂的不同解（每軸都差不到 same_tol 當成同一組）；夠 max_sols 組就停"""
        out = []
        for s in seeds:
            q, ok = self.solve(p, s, R=R, axis=axis, **kw)
            if ok and not any(np.abs(q - o).max() < same_tol for o in out):
                out.append(q)
                if max_sols and len(out) >= max_sols:
                    break
        return out
