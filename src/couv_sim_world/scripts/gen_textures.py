#!/usr/bin/env python3
"""Generate feature-rich, non-repeating PNG textures for every model in layout.py.

Each texture mixes multi-scale value noise, random shapes, lines/cracks and
stains on top of a style-specific base pattern, seeded per texture so no two
surfaces look alike (avoids false loop closures in RGB-D SLAM).

Usage:
    python3 scripts/gen_textures.py                 # all textures
    python3 scripts/gen_textures.py --models drum canopy
    python3 scripts/gen_textures.py --scale 0.5     # faster, lower resolution
Output: models/<model>/materials/textures/<texture>.png
"""
import argparse
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout  # noqa: E402

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FONT_PATHS = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
]


# ---------------------------------------------------------------- primitives
def fbm(rng, w, h, octaves=6, base=4, persistence=0.55):
    """Multi-scale value noise in [0, 1], shape (h, w)."""
    out = np.zeros((h, w), np.float32)
    amp, total = 1.0, 0.0
    for o in range(octaves):
        cells = base * (2 ** o)
        gw, gh = max(2, int(cells * w / max(w, h))), max(2, int(cells * h / max(w, h)))
        grid = rng.random((gh, gw)).astype(np.float32)
        layer = np.asarray(Image.fromarray(grid, "F").resize((w, h), Image.BICUBIC))
        out += amp * layer
        total += amp
        amp *= persistence
    out /= total
    out -= out.min()
    return out / max(out.max(), 1e-6)


def lerp(c0, c1, t):
    c0, c1 = np.asarray(c0, np.float32), np.asarray(c1, np.float32)
    return c0 + (c1 - c0) * t[..., None]


def jitter(rng, c, amount=25):
    return tuple(int(np.clip(v + rng.integers(-amount, amount + 1), 0, 255)) for v in c)


def font(rng, size):
    for p in FONT_PATHS[int(rng.integers(len(FONT_PATHS))):] + FONT_PATHS:
        try:
            return ImageFont.truetype(p, size)
        except OSError:
            continue
    return ImageFont.load_default()


def rand_text(rng, n):
    alphabet = "ABCDEFGHJKLMNPQRSTUVWXYZ0123456789#-/"
    return "".join(alphabet[i] for i in rng.integers(len(alphabet), size=n))


def catmull(points, steps=12):
    pts = [points[0]] + list(points) + [points[-1]]
    out = []
    for i in range(1, len(pts) - 2):
        p0, p1, p2, p3 = (np.asarray(p, float) for p in pts[i - 1:i + 3])
        for t in np.linspace(0, 1, steps, endpoint=False):
            t2, t3 = t * t, t * t * t
            out.append(tuple(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                                    + (-p0 + 3 * p1 - 3 * p2 + p3) * t3)))
    out.append(tuple(points[-1]))
    return out


def to_img(arr):
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8), "RGB")


def to_arr(img):
    return np.asarray(img.convert("RGB"), np.float32)


def overlay(img, rng, draw_fn, blur=0.0, opacity=1.0):
    """Draw on a transparent layer and alpha-composite it onto img."""
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    draw_fn(ImageDraw.Draw(layer), rng)
    if blur:
        layer = layer.filter(ImageFilter.GaussianBlur(blur))
    if opacity < 1.0:
        a = np.asarray(layer, np.float32)
        a[..., 3] *= opacity
        layer = Image.fromarray(a.astype(np.uint8), "RGBA")
    return Image.alpha_composite(img.convert("RGBA"), layer).convert("RGB")


# ---------------------------------------------------------------- common detail layers
def add_stains(img, rng, n, colors, size=(0.03, 0.18), alpha=(40, 110), blur=12):
    w, h = img.size

    def draw(d, r):
        for _ in range(n):
            cx, cy = r.random() * w, r.random() * h
            rad = r.uniform(*size) * w
            pts = []
            k = int(r.integers(7, 14))
            for i in range(k):
                a = 2 * math.pi * i / k
                rr = rad * r.uniform(0.5, 1.2)
                pts.append((cx + rr * math.cos(a), cy + rr * 0.8 * math.sin(a)))
            c = colors[int(r.integers(len(colors)))]
            d.polygon(pts, fill=(*c, int(r.integers(*alpha))))
    return overlay(img, rng, draw, blur=blur * w / 1024)


def add_cracks(img, rng, n, color=(20, 20, 20), width=(1, 4)):
    w, h = img.size

    def draw(d, r):
        for _ in range(n):
            x, y = r.random() * w, r.random() * h
            ang = r.uniform(0, 2 * math.pi)
            pts = [(x, y)]
            for _ in range(int(r.integers(8, 40))):
                ang += r.normal(0, 0.5)
                step = r.uniform(5, 25) * w / 1024
                x, y = x + step * math.cos(ang), y + step * math.sin(ang)
                pts.append((x, y))
                if r.random() < 0.08:  # branch
                    b = r.uniform(-1.2, 1.2)
                    d.line([(x, y), (x + 40 * math.cos(ang + b), y + 40 * math.sin(ang + b))],
                           fill=(*color, 200), width=1)
            d.line(pts, fill=(*color, int(r.integers(140, 230))),
                   width=int(r.integers(*width) * max(1, w // 1024)), joint="curve")
    return overlay(img, rng, draw)


def add_speckle(img, rng, n, colors, rad=(1, 4), alpha=(120, 255)):
    w, h = img.size

    def draw(d, r):
        xs, ys = r.random(n) * w, r.random(n) * h
        rs = r.uniform(*rad, n) * w / 1024
        idx = r.integers(len(colors), size=n)
        al = r.integers(*alpha, size=n)
        for x, y, rr, i, a in zip(xs, ys, rs, idx, al):
            c = jitter(r, colors[i], 20)
            d.ellipse([x - rr, y - rr * r.uniform(0.6, 1.0), x + rr, y + rr], fill=(*c, int(a)))
    return overlay(img, rng, draw)


def add_shapes(img, rng, n, colors, size=(0.01, 0.06), alpha=(80, 200), outline=False):
    w, h = img.size

    def draw(d, r):
        for _ in range(n):
            cx, cy = r.random() * w, r.random() * h
            s = r.uniform(*size) * w
            c = (*jitter(r, colors[int(r.integers(len(colors)))]), int(r.integers(*alpha)))
            kind = r.integers(4)
            if kind == 0:
                d.ellipse([cx - s, cy - s * r.uniform(0.4, 1), cx + s, cy + s], fill=None if outline else c,
                          outline=c, width=max(1, int(s / 8)))
            elif kind == 1:
                d.rectangle([cx - s, cy - s * r.uniform(0.3, 1), cx + s * r.uniform(0.3, 1), cy + s],
                            fill=None if outline else c, outline=c, width=max(1, int(s / 8)))
            elif kind == 2:
                k = int(r.integers(3, 7))
                pts = [(cx + s * r.uniform(0.4, 1.2) * math.cos(a), cy + s * r.uniform(0.4, 1.2) * math.sin(a))
                       for a in np.sort(r.uniform(0, 2 * math.pi, k))]
                d.polygon(pts, fill=None if outline else c, outline=c)
            else:
                a = r.uniform(0, math.pi)
                d.line([(cx - s * math.cos(a), cy - s * math.sin(a)), (cx + s * math.cos(a), cy + s * math.sin(a))],
                       fill=c, width=max(1, int(s / 6)))
    return overlay(img, rng, draw)


def add_text(img, rng, n, colors, size=(0.02, 0.08), alpha=(120, 230), length=(2, 7)):
    w, h = img.size

    def draw(d, r):
        for _ in range(n):
            f = font(r, max(8, int(r.uniform(*size) * h)))
            c = (*jitter(r, colors[int(r.integers(len(colors)))]), int(r.integers(*alpha)))
            d.text((r.random() * w * 0.95, r.random() * h * 0.95), rand_text(r, int(r.integers(*length))),
                   fill=c, font=f)
    return overlay(img, rng, draw)


def finish(img, rng, grain=10):
    arr = to_arr(img)
    w, h = img.size
    arr *= (0.85 + 0.3 * fbm(rng, w, h, octaves=5, base=3))[..., None]
    arr += rng.normal(0, grain, arr.shape[:2])[..., None]
    return to_img(arr)


# ---------------------------------------------------------------- styles
def style_asphalt(rng, w, h, c):
    n = fbm(rng, w, h, 7, 3)
    img = to_img(lerp(c[0], c[1], n ** 1.3))
    img = add_speckle(img, rng, int(w * h / 90), [(25, 25, 25), (150, 150, 145), (200, 195, 185), c[1]], rad=(0.8, 3))
    img = add_stains(img, rng, 18, [(15, 15, 18), (30, 28, 25)], alpha=(50, 120))

    def paint(d, r):  # worn road-paint fragments
        for _ in range(int(r.integers(3, 7))):
            x0, y0 = r.random() * w, r.random() * h
            a = r.choice([0, math.pi / 2, r.uniform(0, math.pi)])
            L, W = r.uniform(0.2, 0.6) * w, r.uniform(0.02, 0.05) * w
            dx, dy = math.cos(a), math.sin(a)
            nx, ny = -dy * W / 2, dx * W / 2
            d.polygon([(x0 + nx, y0 + ny), (x0 + dx * L + nx, y0 + dy * L + ny),
                       (x0 + dx * L - nx, y0 + dy * L - ny), (x0 - nx, y0 - ny)], fill=(*c[2], 210))
    img = overlay(img, rng, paint)
    # wear the paint with noise
    wear = fbm(rng, w, h, 6, 8) > 0.55
    base = to_arr(add_speckle(to_img(lerp(c[0], c[1], n)), rng, int(w * h / 200), [(30, 30, 30)]))
    arr = to_arr(img)
    arr[wear] = 0.5 * arr[wear] + 0.5 * base[wear]
    img = add_cracks(to_img(arr), rng, 14, color=(10, 10, 10))
    img = add_text(img, rng, 3, [c[2]], size=(0.04, 0.1), alpha=(90, 160))
    return finish(img, rng, 8)


def _bricks(rng, w, h, c, bw=(0.08, 0.16), bh=(0.035, 0.06), mortar_col=None, gap=0.006):
    mortar_col = mortar_col or c[1]
    img = to_img(lerp(mortar_col, np.array(mortar_col) * 0.7, fbm(rng, w, h, 5, 6)))
    d = ImageDraw.Draw(img)
    y = 0.0
    g = gap * w
    while y < h:
        rh = rng.uniform(*bh) * h
        x = -rng.random() * bw[1] * w
        while x < w:
            rw = rng.uniform(*bw) * w
            col = jitter(rng, c[int(rng.integers(len(c)))] if rng.random() < 0.25 else c[0], 30)
            d.rectangle([x + g, y + g, x + rw - g, y + rh - g], fill=col)
            x += rw
        y += rh
    arr = to_arr(img)
    arr *= (0.75 + 0.5 * fbm(rng, w, h, 7, 16))[..., None]
    return to_img(arr)


def style_paver(rng, w, h, c):
    img = _bricks(rng, w, h, c, bw=(0.06, 0.14), bh=(0.05, 0.09))
    img = add_stains(img, rng, 25, [(40, 35, 30), (90, 110, 60), (200, 200, 190)])
    img = add_cracks(img, rng, 10, color=(25, 20, 18))
    img = add_shapes(img, rng, 60, [(30, 30, 30), (230, 230, 220), c[2]], size=(0.004, 0.02))
    img = add_speckle(img, rng, int(w * h / 600), [(30, 30, 30), (220, 220, 210)], rad=(0.5, 2))
    return finish(img, rng, 9)


def style_cobble(rng, w, h, c):
    img = to_img(lerp(np.array(c[1]) * 0.6, c[1], fbm(rng, w, h, 6, 5)))

    def stones(d, r):
        for _ in range(int(w * h / 1800)):
            cx, cy = r.random() * w, r.random() * h
            s = r.uniform(0.015, 0.035) * w
            k = int(r.integers(6, 10))
            pts = [(cx + s * r.uniform(0.7, 1.1) * math.cos(a), cy + s * r.uniform(0.7, 1.1) * math.sin(a))
                   for a in np.linspace(0, 2 * math.pi, k, endpoint=False) + r.uniform(0, 1)]
            d.polygon(pts, fill=(*jitter(r, c[int(r.integers(len(c)))], 35), 255), outline=(30, 28, 25, 255))
    img = overlay(img, rng, stones)
    arr = to_arr(img) * (0.7 + 0.6 * fbm(rng, w, h, 7, 20))[..., None]
    img = add_stains(to_img(arr), rng, 20, [(30, 30, 25), (70, 90, 50)])
    img = add_speckle(img, rng, int(w * h / 500), [(20, 20, 20), (230, 225, 215)], rad=(0.5, 2.5))
    return finish(img, rng, 8)


def _wall_base(rng, w, h, c):
    img = to_img(lerp(c[0], np.array(c[0]) * 0.65, fbm(rng, w, h, 7, 4)))
    img = add_speckle(img, rng, int(w * h / 300), [(50, 50, 50), (230, 230, 225)], rad=(0.5, 2), alpha=(60, 160))
    return add_cracks(img, rng, 8, color=(40, 40, 40))


def style_graffiti(rng, w, h, c):
    img = _wall_base(rng, w, h, c)
    pal = list(c[1:]) + [tuple(int(v) for v in rng.integers(0, 256, 3)) for _ in range(5)] + [(15, 15, 15), (245, 245, 245)]

    def strokes(d, r):
        for _ in range(int(r.integers(70, 110))):
            k = int(r.integers(3, 8))
            x0, y0 = r.random() * w, r.random() * h
            span = r.uniform(0.03, 0.15) * w
            pts = [(x0 + r.normal(0, span), y0 + r.normal(0, span * 0.6)) for _ in range(k)]
            col = (*jitter(r, pal[int(r.integers(len(pal)))], 20), int(r.integers(160, 250)))
            wd = int(r.uniform(3, 22) * w / 2048) + 2
            line = catmull(pts)
            d.line(line, fill=col, width=wd, joint="curve")
            if r.random() < 0.4:  # outline pass
                d.line(line, fill=(15, 15, 15, 200), width=max(1, wd // 4), joint="curve")
            if r.random() < 0.35:  # drips
                for _ in range(int(r.integers(1, 5))):
                    px, py = line[int(r.integers(len(line)))]
                    d.line([(px, py), (px, py + r.uniform(10, 80) * h / 1024)], fill=col, width=max(1, wd // 3))
    img = overlay(img, rng, strokes)
    img = add_text(img, rng, 40, pal, size=(0.05, 0.2), length=(2, 6))
    img = add_shapes(img, rng, 120, pal, size=(0.004, 0.03), outline=True)
    img = add_stains(img, rng, 20, [(30, 30, 30), (90, 70, 50)], alpha=(30, 80))
    return finish(img, rng, 9)


def style_poster(rng, w, h, c):
    img = _wall_base(rng, w, h, c)
    pal = list(c[1:]) + [tuple(int(v) for v in rng.integers(0, 256, 3)) for _ in range(6)]
    for _ in range(int(rng.integers(30, 45))):
        pw, ph = int(rng.uniform(0.04, 0.12) * w), int(rng.uniform(0.2, 0.55) * h)
        bg = jitter(rng, pal[int(rng.integers(len(pal)))], 30)
        p = Image.new("RGB", (pw, ph), bg)
        p = add_shapes(p, rng, int(rng.integers(5, 25)), pal + [(20, 20, 20), (240, 240, 240)], size=(0.05, 0.4))
        p = add_text(p, rng, int(rng.integers(3, 9)), [(15, 15, 15), (250, 250, 250)] + pal, size=(0.04, 0.14), length=(3, 9))
        # torn / faded
        mask = Image.fromarray((fbm(rng, pw, ph, 5, 3) > rng.uniform(0.05, 0.3)).astype(np.uint8) * 255, "L")
        img.paste(p.rotate(rng.uniform(-6, 6), expand=False, fillcolor=bg),
                  (int(rng.random() * (w - pw)), int(rng.random() * (h - ph))), mask)
    img = add_stains(img, rng, 25, [(40, 40, 40), (120, 100, 70)], alpha=(30, 90))
    img = add_cracks(img, rng, 6, color=(30, 30, 30))
    return finish(img, rng, 8)


def style_concrete(rng, w, h, c):
    n = fbm(rng, w, h, 8, 3)
    img = to_img(lerp(c[1], c[0], n))
    # faint formwork joints
    d = ImageDraw.Draw(img)
    for _ in range(int(rng.integers(2, 6))):
        y = rng.random() * h
        d.line([(0, y), (w, y + rng.normal(0, 5))], fill=jitter(rng, c[1], 10), width=int(rng.integers(1, 4)))
    img = add_speckle(img, rng, int(w * h / 250), [(40, 40, 40), (90, 90, 90)], rad=(0.6, 3))  # pits
    img = add_stains(img, rng, 30, [(60, 55, 50), (120, 110, 90), (40, 50, 40)], alpha=(30, 90))
    img = add_cracks(img, rng, 10, color=(30, 30, 30))
    img = add_shapes(img, rng, 25, [c[2], (240, 240, 235)], size=(0.01, 0.05), outline=True, alpha=(120, 220))  # chalk
    img = add_text(img, rng, 8, [c[2], (30, 30, 30)], size=(0.02, 0.06))
    return finish(img, rng, 10)


def style_gravel(rng, w, h, c):
    img = to_img(lerp((40, 38, 35), c[1], fbm(rng, w, h, 5, 6)))
    img = add_speckle(img, rng, int(w * h / 45), [c[0], c[1], c[2], (200, 195, 185), (70, 65, 60)], rad=(1.5, 6), alpha=(200, 255))
    img = add_speckle(img, rng, int(w * h / 120), [(230, 225, 215), (30, 30, 30)], rad=(0.5, 1.5))
    img = add_stains(img, rng, 30, [(20, 20, 20), (60, 70, 40), (150, 140, 120)], alpha=(40, 110))
    img = add_shapes(img, rng, 40, [(170, 170, 160), (100, 60, 40)], size=(0.01, 0.04), alpha=(150, 230))  # debris
    return finish(img, rng, 10)


def style_crate(rng, w, h, c):
    arr = np.zeros((h, w, 3), np.float32)
    y = 0
    while y < h:
        ph = int(rng.uniform(0.08, 0.16) * h)
        grain = fbm(rng, w // 8 + 2, max(2, ph // 2), 5, 4)
        grain = np.asarray(Image.fromarray(grain, "F").resize((w, ph), Image.BICUBIC))
        stripes = 0.5 + 0.5 * np.sin(np.linspace(0, rng.uniform(20, 60), ph)[:, None] + 8 * grain)
        arr[y:y + ph] = lerp(jitter(rng, c[0], 25), c[1], 0.6 * stripes * grain)[: h - y]
        arr[y:y + 3] = c[2]
        y += ph
    img = to_img(arr)
    img = add_speckle(img, rng, 80, [(30, 30, 30), (90, 90, 95)], rad=(3, 6))  # nails / knots
    img = add_text(img, rng, 10, [(20, 20, 20), (160, 30, 30)], size=(0.05, 0.15), length=(3, 8))
    img = add_shapes(img, rng, 20, [(20, 20, 20), (160, 30, 30)], size=(0.02, 0.08), outline=True)
    img = add_stains(img, rng, 15, [(50, 35, 20), (20, 20, 20)])
    img = add_cracks(img, rng, 6, color=(40, 25, 15))
    return finish(img, rng, 8)


def style_rusty_paint(rng, w, h, c):
    img = to_img(lerp(c[0], np.array(c[0]) * 0.7, fbm(rng, w, h, 6, 4)))
    d = ImageDraw.Draw(img)
    for _ in range(int(rng.integers(2, 5))):  # bands
        y, bh = rng.random() * h, rng.uniform(0.02, 0.07) * h
        d.rectangle([0, y, w, y + bh], fill=jitter(rng, c[2], 20))
    for _ in range(int(rng.integers(40, 90))):  # rivets
        x, y, r = rng.random() * w, rng.random() * h, rng.uniform(3, 7) * w / 1024
        d.ellipse([x - r, y - r, x + r, y + r], fill=jitter(rng, c[2], 40))
    img = add_text(img, rng, 12, [c[2], (20, 20, 20), (240, 240, 240)], size=(0.04, 0.12), length=(2, 8))
    arr = to_arr(img)
    rust_n = fbm(rng, w, h, 7, 5)
    rust = rust_n > rng.uniform(0.55, 0.65)
    rust_col = lerp(c[1], np.array(c[1]) * 0.55, fbm(rng, w, h, 6, 20))
    arr[rust] = rust_col[rust]
    img = add_cracks(to_img(arr), rng, 25, color=(200, 200, 195), width=(1, 3))  # scratches
    img = add_stains(img, rng, 25, [c[1], (30, 25, 20)], alpha=(50, 130))
    img = add_speckle(img, rng, int(w * h / 400), [c[1], (40, 30, 20)], rad=(0.6, 3))
    return finish(img, rng, 9)


def style_grille(rng, w, h, c):
    img = to_img(lerp(c[0], c[1], 0.35 * fbm(rng, w, h, 6, 4)))
    d = ImageDraw.Draw(img)
    for _ in range(int(rng.integers(2, 4))):  # fan grilles
        cx, cy, R = rng.uniform(0.2, 0.8) * w, rng.uniform(0.2, 0.8) * h, rng.uniform(0.12, 0.25) * w
        d.ellipse([cx - R, cy - R, cx + R, cy + R], fill=jitter(rng, c[2], 15))
        r = R
        while r > 8:
            d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=c[1], width=2)
            r -= rng.uniform(6, 14) * w / 1024
        for a in np.linspace(0, math.pi, int(rng.integers(3, 6)), endpoint=False) + rng.uniform(0, 1):
            d.line([(cx - R * math.cos(a), cy - R * math.sin(a)), (cx + R * math.cos(a), cy + R * math.sin(a))],
                   fill=c[1], width=3)
    x0 = rng.uniform(0.0, 0.5) * w  # louvre slats
    for y in np.arange(rng.uniform(0, 20), h, rng.uniform(10, 18) * h / 1024):
        if rng.random() < 0.85:
            d.line([(x0, y), (x0 + rng.uniform(0.2, 0.45) * w, y)], fill=jitter(rng, c[2], 20), width=3)
    img = add_shapes(img, rng, 30, [(250, 200, 20), (200, 40, 40), (40, 90, 200), (250, 250, 250)], size=(0.015, 0.05))  # stickers
    img = add_text(img, rng, 12, [(30, 30, 30), (200, 40, 40)], size=(0.02, 0.06), length=(3, 10))
    img = add_speckle(img, rng, 150, [(60, 60, 60)], rad=(2, 5))  # screws
    img = add_stains(img, rng, 30, [(90, 70, 50), (40, 40, 40)], alpha=(40, 110))
    img = add_cracks(img, rng, 10, color=(80, 80, 80), width=(1, 2))
    return finish(img, rng, 9)


def style_brick(rng, w, h, c):
    img = _bricks(rng, w, h, [c[0], c[2], jitter(rng, c[0], 40)], bw=(0.1, 0.18), bh=(0.04, 0.06), mortar_col=c[1])
    img = add_stains(img, rng, 25, [(30, 30, 30), (220, 220, 210), (60, 80, 40)])
    img = add_text(img, rng, 10, [(20, 20, 20), (250, 250, 250), (30, 90, 200)], size=(0.04, 0.1))
    img = add_shapes(img, rng, 60, [(250, 250, 250), (20, 20, 20), (240, 200, 30)], size=(0.01, 0.05), outline=True)
    img = add_cracks(img, rng, 8, color=(30, 25, 20))
    return finish(img, rng, 9)


def style_mosaic(rng, w, h, c):
    img = to_img(lerp((60, 60, 60), (110, 110, 110), fbm(rng, w, h, 5, 5)))

    def tiles(d, r):
        for _ in range(int(w * h / 900)):
            cx, cy, s = r.random() * w, r.random() * h, r.uniform(0.008, 0.03) * w
            a = r.uniform(0, math.pi / 2)
            pts = [(cx + s * math.cos(a + k * math.pi / 2 + r.normal(0, 0.15)),
                    cy + s * math.sin(a + k * math.pi / 2 + r.normal(0, 0.15))) for k in range(4)]
            d.polygon(pts, fill=(*jitter(r, c[int(r.integers(len(c)))], 35), 255))
    img = overlay(img, rng, tiles)
    img = add_stains(img, rng, 20, [(30, 30, 30), (200, 200, 190)], alpha=(30, 90))
    img = add_cracks(img, rng, 8, color=(20, 20, 20))
    return finish(img, rng, 8)


def style_camo(rng, w, h, c):
    arr = np.zeros((h, w, 3), np.float32) + np.array(c[0], np.float32)
    for col in c[1:]:
        m = fbm(rng, w, h, 6, 4) > rng.uniform(0.5, 0.62)
        arr[m] = col
    arr *= (0.8 + 0.4 * fbm(rng, w, h, 7, 24))[..., None]
    img = add_cracks(to_img(arr), rng, 25, color=(210, 205, 190), width=(1, 3))
    img = add_text(img, rng, 10, [(230, 230, 220), (20, 20, 20)], size=(0.04, 0.12))
    img = add_speckle(img, rng, int(w * h / 500), [(20, 20, 20), (200, 190, 160)], rad=(0.6, 3))
    img = add_stains(img, rng, 15, [(60, 45, 30)], alpha=(50, 120))
    return finish(img, rng, 8)


def style_hazard(rng, w, h, c):
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    period = rng.uniform(0.1, 0.16) * w
    warp = 25 * fbm(rng, w, h, 4, 3)
    stripes = (((xx + yy + warp) / period) % 1.0) < 0.5
    arr = np.where(stripes[..., None], np.array(c[0], np.float32), np.array(c[1], np.float32))
    worn = fbm(rng, w, h, 7, 6) > rng.uniform(0.62, 0.7)
    under = lerp((60, 60, 60), (110, 105, 100), fbm(rng, w, h, 6, 12))
    arr[worn] = under[worn]
    img = to_img(arr)
    img = add_text(img, rng, 14, [c[2], (20, 20, 20), (250, 250, 250)], size=(0.05, 0.18), length=(1, 5))
    img = add_shapes(img, rng, 80, [c[2], (20, 20, 20), (250, 250, 250)], size=(0.005, 0.04))
    img = add_cracks(img, rng, 12, color=(30, 30, 30))
    img = add_stains(img, rng, 20, [(30, 30, 30), (120, 100, 70)])
    return finish(img, rng, 9)


STYLES = {name[6:]: fn for name, fn in globals().items() if name.startswith("style_")}


def texture_path(model_name, tex_name):
    return os.path.join(PKG_DIR, "models", model_name, "materials", "textures", f"{tex_name}.png")


def generate(spec, scale=1.0):
    rng = np.random.default_rng(spec["seed"])
    w, h = (max(64, int(v * scale)) for v in spec["px"])
    return STYLES[spec["style"]](rng, w, h, [tuple(c) for c in spec["colors"]])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--models", nargs="*", help="only these model names")
    ap.add_argument("--scale", type=float, default=1.0, help="resolution multiplier")
    args = ap.parse_args()
    seeds = {}
    for m in layout.MODELS:
        for tex, spec in m["textures"].items():
            if spec["seed"] in seeds:
                raise SystemExit(f"duplicate texture seed {spec['seed']}: {tex} / {seeds[spec['seed']]}")
            seeds[spec["seed"]] = tex
    for m in layout.MODELS:
        if args.models and m["name"] not in args.models:
            continue
        for tex, spec in m["textures"].items():
            path = texture_path(m["name"], tex)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            generate(spec, args.scale).save(path, optimize=False, compress_level=6)
            print(f"  {os.path.relpath(path, PKG_DIR)}  ({spec['style']}, {spec['px'][0]}x{spec['px'][1]})")


if __name__ == "__main__":
    main()
