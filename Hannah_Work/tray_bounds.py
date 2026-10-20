import sys
from pathlib import Path
 
import numpy as np
import xml.etree.ElementTree as ET
 
 
def tray_bounds_from_file(path):
    """Axis-aligned bounds (lo, hi) of a COLLADA mesh in metres, Z up."""
    root = ET.parse(path).getroot()
    ns = root.tag.split("}")[0] + "}" if root.tag.startswith("{") else ""
    T = lambda name: ns + name
 
    unit = 1.0
    u = root.find(f".//{T('unit')}")
    if u is not None and u.get("meter"):
        unit = float(u.get("meter"))
    up = (root.findtext(f".//{T('up_axis')}") or "Z_UP").strip()
 
    geoms = {}
    for g in root.iter(T("geometry")):
        mesh = g.find(T("mesh"))
        if mesh is None:
            continue
        sources = {s.get("id"): s for s in mesh.findall(T("source"))}
        pos_id = None
        v = mesh.find(T("vertices"))
        if v is not None:
            for inp in v.findall(T("input")):
                if inp.get("semantic") == "POSITION":
                    pos_id = inp.get("source").lstrip("#")
        if pos_id in sources:
            arr = sources[pos_id].find(T("float_array"))
            geoms[g.get("id")] = np.array(arr.text.split(), float).reshape(-1, 3)
    if not geoms:
        raise ValueError("no mesh geometry found")
 
    def node_matrix(node):
        M = np.eye(4)
        for ch in node:
            tag = ch.tag.replace(ns, "")
            vals = np.array(ch.text.split(), float) if ch.text else None
            if tag == "matrix":
                M = M @ vals.reshape(4, 4)
            elif tag == "translate":
                A = np.eye(4); A[:3, 3] = vals; M = M @ A
            elif tag == "scale":
                M = M @ np.diag([*vals, 1.0])
            elif tag == "rotate":
                ax, ang = vals[:3], np.deg2rad(vals[3])
                ax = ax / np.linalg.norm(ax)
                K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                A = np.eye(4)
                A[:3, :3] = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
                M = M @ A
        return M
 
    pts = []
 
    def walk(node, parent):
        M = parent @ node_matrix(node)
        for ig in node.findall(T("instance_geometry")):
            gid = ig.get("url").lstrip("#")
            if gid in geoms:
                P = np.c_[geoms[gid], np.ones(len(geoms[gid]))] @ M.T
                pts.append(P[:, :3])
        for child in node.findall(T("node")):
            walk(child, M)
 
    scene = root.find(f".//{T('visual_scene')}")
    if scene is not None:
        for n in scene.findall(T("node")):
            walk(n, np.eye(4))
    if not pts:
        pts = list(geoms.values())
 
    P = np.vstack(pts) * unit
    if up == "Y_UP":
        P = np.c_[P[:, 0], -P[:, 2], P[:, 1]]
    return P.min(axis=0), P.max(axis=0)
 
 
if len(sys.argv) > 1:
    tray_file = Path(sys.argv[1])
else:
    project = Path(__file__).resolve().parent.parent
    found = sorted(project.rglob("tray_yellow.dae"),
                   key=lambda p: ("food" not in str(p).lower(), len(str(p))))
    if not found:
        sys.exit("Couldn't find tray_yellow.dae - run: python read_tray_size.py <path>")
    tray_file = found[0]
 
lo, hi = tray_bounds_from_file(str(tray_file))
lo = [round(float(v), 4) for v in lo]
hi = [round(float(v), 4) for v in hi]
size = [round(b - a, 4) for a, b in zip(lo, hi)]
 
print("Tray file:", tray_file)
print("Size x, y, z (m, before the Mesh's scale):", size)
print()
print("Copy these two lines into Feed_Iso.py:")
print("TRAY_LO =", lo)
print("TRAY_HI =", hi)
 