#!/usr/bin/env python3
"""Write meshes/landing_pad.{obj,mtl,png}: a unit box (scaled in the URDF) whose top face
carries a feature-rich, non-repeating texture with the ArUco marker from frames.yaml
(`ugv.aruco`) in a white quiet zone at the centre. The top face uses the whole texture
(square pixels on the square pad); the sides map to a corner of the border.

Usage: python3 couv_sim_robots/gen_pad_mesh.py   (from the package root)
"""
import os

import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(PKG, "meshes")
PX = 1024
SIDE_UV = 0.02         # the side faces use this corner of the texture (border stripe)


def texture(seed=7):
    rng = np.random.default_rng(seed)
    img = Image.new("RGB", (PX, PX), (40, 40, 44))
    d = ImageDraw.Draw(img)
    # random coloured blocks in a jittered grid (non repeating), like a calibration board
    cell = PX // 16
    for j in range(16):
        for i in range(16):
            if rng.random() < 0.55:
                c = tuple(int(v) for v in rng.integers(30, 250, 3))
                x0, y0 = i * cell + rng.integers(0, 8), j * cell + rng.integers(0, 8)
                d.rectangle([x0, y0, x0 + cell - rng.integers(4, 16), y0 + cell - rng.integers(4, 16)], fill=c)
    for _ in range(120):
        x, y, r = rng.random() * PX, rng.random() * PX, rng.uniform(4, 22)
        d.ellipse([x - r, y - r, x + r, y + r], outline=tuple(int(v) for v in rng.integers(0, 255, 3)), width=3)
    # yellow/black border
    b = PX // 20
    for k in range(0, PX, b):
        col = (240, 200, 20) if (k // b) % 2 == 0 else (20, 20, 20)
        for box in ([k, 0, k + b, b], [k, PX - b, k + b, PX], [0, k, b, k + b], [PX - b, k, PX, k + b]):
            d.rectangle(box, fill=col)
    # ArUco marker in a white quiet zone; image top = +x of the UGV, image right = -y (see UVs)
    cfg = yaml.safe_load(open(os.path.join(PKG, "config", "frames.yaml")))["ugv"]
    a, pad = cfg["aruco"], cfg["landing_pad"]
    px_per_m = PX / pad["size_x"]
    q = int(round(a["quiet_zone"] * px_per_m))
    d.rectangle([PX // 2 - q // 2, PX // 2 - q // 2, PX // 2 + q // 2, PX // 2 + q // 2], fill=(250, 250, 250))
    arr = np.asarray(img, np.float32) + rng.normal(0, 6, (PX, PX, 1))
    m = int(round(a["size"] * px_per_m))
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, a["dictionary"])),
                                           a["id"], m)
    o = PX // 2 - m // 2
    arr[o:o + m, o:o + m] = np.where(marker[..., None] > 127, 245.0, 15.0)
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))


def obj():
    # unit box centred at the origin. Top face: u along -y, v along +x (image top = +x = forward)
    v = [(x, y, z) for z in (-0.5, 0.5) for y in (-0.5, 0.5) for x in (-0.5, 0.5)]
    # vertex index (1-based): z-, (y-,x-)=1 (y-,x+)=2 (y+,x-)=3 (y+,x+)=4 ; z+ : 5..8
    top_uv = {5: (1, 0), 6: (1, 1), 7: (0, 0), 8: (0, 1)}
    lines = ["mtllib landing_pad.mtl", "o landing_pad"]
    lines += [f"v {x} {y} {z}" for x, y, z in v]
    uvs = [top_uv[5], top_uv[6], top_uv[7], top_uv[8], (0, 0), (SIDE_UV, 0), (SIDE_UV, SIDE_UV), (0, SIDE_UV)]
    lines += [f"vt {a} {b}" for a, b in uvs]
    norms = [(0, 0, 1), (0, 0, -1), (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)]
    lines += [f"vn {a} {b} {c}" for a, b, c in norms]
    lines.append("usemtl pad")
    # top (+z), ccw seen from above
    lines += ["f 5/1/1 6/2/1 8/4/1", "f 5/1/1 8/4/1 7/3/1"]
    side = "{0}/5/{n} {1}/6/{n} {2}/7/{n}", "{0}/5/{n} {2}/7/{n} {3}/8/{n}"
    quads = [((1, 3, 4, 2), 2), ((2, 4, 8, 6), 3), ((3, 1, 5, 7), 4), ((4, 3, 7, 8), 5), ((1, 2, 6, 5), 6)]
    for (a, b, c, e), n in quads:
        lines += [side[0].format(a, b, c, n=n), side[1].format(a, b, c, e, n=n)]
    return "\n".join(lines) + "\n"


def main():
    os.makedirs(OUT, exist_ok=True)
    texture().save(os.path.join(OUT, "landing_pad.png"))
    open(os.path.join(OUT, "landing_pad.obj"), "w").write(obj())
    open(os.path.join(OUT, "landing_pad.mtl"), "w").write(
        "newmtl pad\nKa 1 1 1\nKd 1 1 1\nKs 0.05 0.05 0.05\nNs 10\nmap_Kd landing_pad.png\n")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
