#!/usr/bin/env python3
"""
產生 ~/nstc/web/RobotViewer.html：樣板 ~/nstc/web/RobotViewer.template.html + URDF 的連桿、關節 + 所有 STL（嵌進去，網頁不用另外讀檔）

URDF 或 STL 改了（例如換手、改支架）就重跑一次：
    ros2 run armpackage build_viewer.py
    或 python3 ~/nstc/src/armpackage/armpackage/scripts/build_viewer.py
網頁的程式碼要改請改樣板，再重跑這支；不要直接改 RobotViewer.html（下次產生會被蓋掉）

STL 壓成 16-bit 整數座標（每個網格依自己的包圍盒量化），檔案比原始 STL 的 base64 小很多，
精度 = 包圍盒 / 65535（手掌約 1.5 µm、支架約 15 µm），看不出差別
"""
import argparse
import base64
import json
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))           # realpath：從 install/（連結）執行也找得到原始碼
PKG = os.path.dirname(HERE)
WEB = os.path.normpath(os.path.join(PKG, "..", "..", "..", "web"))   # ~/nstc/web（src/armpackage/armpackage 往上三層）


def default_urdf_pkg():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.realpath(get_package_share_directory("seven_axis_arm_urdf"))
    except Exception:
        return os.path.join(os.path.dirname(PKG), "seven_axis_arm_urdf")


def _floats(s, default="0 0 0"):
    return [float(v) for v in (s or default).split()]


def _rgba_to_int(rgba):
    r, g, b = (int(round(float(v) * 255)) for v in rgba.split()[:3])
    return (r << 16) | (g << 8) | b


def parse_urdf(path):
    root = ET.parse(path).getroot()
    global_mat = {m.get("name"): m.find("color").get("rgba") for m in root.findall("material")
                  if m.find("color") is not None}
    links = {}
    for link in root.findall("link"):
        d = {"mesh": None, "vo": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}}
        v = link.find("visual")
        if v is not None:
            o = v.find("origin")
            if o is not None:
                d["vo"] = {"xyz": _floats(o.get("xyz")), "rpy": _floats(o.get("rpy"))}
            m = v.find("geometry/mesh")
            if m is not None:
                d["mesh"] = m.get("filename", "").split("meshes/")[-1]      # 例如 hand_r/index_distal.STL
            mat = v.find("material")
            if mat is not None:
                c = mat.find("color")
                rgba = c.get("rgba") if c is not None else global_mat.get(mat.get("name"))
                if rgba:
                    d["color"] = _rgba_to_int(rgba)
                if mat.get("name"):                  # 有名字的材質（silver、BlackAccent、stand）網頁用它的顏色，其他維持藍色
                    d["matName"] = mat.get("name")
        links[link.get("name")] = d
    joints = {}
    for j in root.findall("joint"):
        o, ax, lim, mim = j.find("origin"), j.find("axis"), j.find("limit"), j.find("mimic")
        d = {"type": j.get("type"), "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
             "origin": {"xyz": _floats(o.get("xyz") if o is not None else None),
                        "rpy": _floats(o.get("rpy") if o is not None else None)},
             "axis": _floats(ax.get("xyz") if ax is not None else None, "1 0 0")}
        if lim is not None and lim.get("lower") is not None:
            d["limit"] = [float(lim.get("lower")), float(lim.get("upper"))]
        if mim is not None:
            d["mimic"] = {"joint": mim.get("joint"), "multiplier": float(mim.get("multiplier", 1)),
                          "offset": float(mim.get("offset", 0))}
        joints[j.get("name")] = d
    return links, joints


def stl_positions(path):
    """二進位 STL → N×9 的三角形頂點（float32）"""
    b = open(path, "rb").read()
    n = struct.unpack_from("<I", b, 80)[0]
    rec = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("x", "<u2")])
    return np.frombuffer(b, dtype=rec, count=n, offset=84)["v"]


def quantize(pos):
    p = pos.reshape(-1, 3).astype(np.float64)
    lo, hi = p.min(axis=0), p.max(axis=0)
    span = np.where(hi - lo > 0, hi - lo, 1.0)
    q = np.round((p - lo) / span * 65535 - 32768).astype("<i2")
    return {"min": lo.tolist(), "max": hi.tolist(), "q": base64.b64encode(q.tobytes()).decode()}


def main():
    urdf_pkg = default_urdf_pkg()
    ap = argparse.ArgumentParser(description="產生雙手 RobotViewer.html")
    ap.add_argument("--urdf", default=os.path.join(urdf_pkg, "urdf", "seven_axis_arm_urdf.urdf"))
    ap.add_argument("--meshes", default=os.path.join(urdf_pkg, "meshes"))
    ap.add_argument("--template", default=os.path.join(WEB, "RobotViewer.template.html"))
    ap.add_argument("--out", default=os.path.join(WEB, "RobotViewer.html"))
    args = ap.parse_args()

    links, joints = parse_urdf(args.urdf)
    meshes, raw = {}, 0
    for d in links.values():
        f = d["mesh"]
        if f and f not in meshes:
            path = os.path.join(args.meshes, f)
            if not os.path.exists(path):
                print(f"⚠ 找不到 {path}，這個連桿不會顯示")
                continue
            raw += os.path.getsize(path)
            meshes[f] = quantize(stl_positions(path))
    tpl = open(args.template, encoding="utf-8").read()
    for key, val in (("URDF_LINKS", links), ("URDF_JOINTS", joints), ("MESH_DATA", meshes)):
        mark = f"__{key}__"
        if tpl.count(mark) != 1:
            raise SystemExit(f"樣板裡 {mark} 應該剛好一個")
        tpl = tpl.replace(mark, json.dumps(val, separators=(",", ":"), ensure_ascii=False))
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(tpl)
    print(f"URDF：{args.urdf}\n連桿 {len(links)}、關節 {len(joints)}、網格 {len(meshes)}（原始 STL {raw / 1e6:.1f} MB）")
    print(f"寫出 {args.out}（{os.path.getsize(args.out) / 1e6:.1f} MB）")


if __name__ == "__main__":
    main()
