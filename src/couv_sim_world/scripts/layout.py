"""Single source of truth for the canopy_world layout.

Every coordinate, size and texture assignment lives here. gen_textures.py,
gen_world.py (SDF + docs/layout.png + README table) and capture_views.py all
import this module, so changing the layout means editing only this file and
re-running the generators.

Units: metres. Origin: map centre. z: up.
Each model is a dict:
    name   : model directory name (model://<name>)
    zone   : 'structure' | 'under' | 'roof' | 'shared' | 'marker'
    label  : human readable name (README / layout.png)
    pose   : (x, y, z, yaw) of the model origin in the world
    parts  : list of primitives in the model frame
             {name, shape: 'box'|'cylinder', size: (sx, sy, sz) | (r, h),
              pos: (x, y, z_centre), texture: <texture name>, collision: bool}
    textures: {texture name: {style, seed, colors, px: (w, h)}}
"""

WORLD_NAME = "canopy_world"

# ---------------------------------------------------------------- parameters
FLOOR_SIZE = 20.0
FLOOR_TILE = 5.0
FLOOR_THICKNESS = 0.02            # visual slab, top face at z = 0

WALL_HALF_EXTENT = 10.0           # walls centred on x, y = +-10
WALL_HEIGHT = 3.0
WALL_THICKNESS = 0.2

ROOF_CENTER = (0.0, 0.0)
ROOF_SIZE = (8.0, 6.0, 0.2)       # x, y, thickness
ROOF_BOTTOM_Z = 3.0
ROOF_GRAVEL_LAYER = 0.05          # top part of the roof slab drawn with gravel
PILLAR_SIZE = (0.3, 0.3, 3.0)
PILLAR_XS = (-3.85, 0.0, 3.85)
PILLAR_YS = (-2.85, 2.85)

ROOF_TOP_Z = ROOF_BOTTOM_Z + ROOF_SIZE[2]

MARKER_SIZE = (2.0, 1.5)
MARKER_THICKNESS = 0.004          # visual only, no collision
START_XY = (-7.0, 0.0)
GOAL_XY = (7.0, 0.0)

DRONE_ALTITUDE = 5.0
# Recommended UGV route: under the canopy, south of the crate stack and north
# of the drum (slalom between the two ground obstacles).
UGV_ROUTE = [(-7.0, 0.0), (-4.5, 0.0), (-1.5, -0.3), (1.8, 0.4), (4.5, 0.0), (7.0, 0.0)]

TEX_PX = 1024                     # default texture resolution (square)


# ---------------------------------------------------------------- helpers
def _tex(style, seed, colors, px=(TEX_PX, TEX_PX)):
    return {"style": style, "seed": seed, "colors": colors, "px": px}


def _box(name, size, pos, texture, collision=True):
    return {"name": name, "shape": "box", "size": tuple(size), "pos": tuple(pos),
            "texture": texture, "collision": collision}


def _cyl(name, radius, height, pos, texture, collision=True):
    return {"name": name, "shape": "cylinder", "size": (radius, height), "pos": tuple(pos),
            "texture": texture, "collision": collision}


# Floor tile styles / colour sets, one entry per tile (row-major from -x,-y).
_FLOOR_STYLES = [
    ("asphalt", [(52, 52, 55), (90, 88, 84), (200, 190, 60)]),
    ("paver",   [(150, 80, 60), (110, 60, 50), (180, 150, 120)]),
    ("asphalt", [(70, 66, 60), (40, 40, 45), (230, 230, 230)]),
    ("paver",   [(120, 120, 128), (80, 85, 95), (170, 160, 140)]),
    ("paver",   [(170, 140, 90), (130, 100, 60), (90, 70, 50)]),
    ("asphalt", [(60, 62, 70), (110, 105, 95), (210, 80, 50)]),
    ("cobble",  [(110, 105, 100), (75, 70, 66), (150, 140, 120)]),
    ("asphalt", [(45, 50, 48), (95, 100, 90), (240, 220, 80)]),
    ("cobble",  [(140, 110, 90), (95, 75, 60), (180, 160, 140)]),
    ("paver",   [(90, 110, 100), (60, 80, 70), (160, 170, 150)]),
    ("asphalt", [(80, 75, 70), (50, 46, 44), (220, 220, 210)]),
    ("cobble",  [(100, 100, 115), (65, 65, 80), (150, 150, 170)]),
    ("paver",   [(160, 90, 90), (120, 60, 65), (200, 170, 160)]),
    ("asphalt", [(58, 55, 52), (100, 92, 80), (80, 160, 210)]),
    ("cobble",  [(125, 120, 95), (85, 80, 60), (170, 165, 130)]),
    ("paver",   [(100, 100, 100), (140, 135, 125), (60, 60, 60)]),
]


def _floor():
    n = int(round(FLOOR_SIZE / FLOOR_TILE))
    parts, textures = [], {}
    start = -FLOOR_SIZE / 2 + FLOOR_TILE / 2
    for j in range(n):
        for i in range(n):
            k = j * n + i
            tex = f"floor_tile_{k:02d}"
            style, colors = _FLOOR_STYLES[k % len(_FLOOR_STYLES)]
            textures[tex] = _tex(style, 100 + k, colors)
            parts.append(_box(f"tile_{k:02d}", (FLOOR_TILE, FLOOR_TILE, FLOOR_THICKNESS),
                              (start + i * FLOOR_TILE, start + j * FLOOR_TILE, -FLOOR_THICKNESS / 2),
                              tex, collision=False))
    return {"name": "floor", "zone": "structure", "label": "Floor (16 tiles)",
            "pose": (0, 0, 0, 0), "parts": parts, "textures": textures,
            "ground_plane": (FLOOR_SIZE, FLOOR_SIZE)}


def _walls():
    h, t, e = WALL_HEIGHT, WALL_THICKNESS, WALL_HALF_EXTENT
    length = 2 * e + t
    # style, colours, aspect-matched texture size
    specs = [
        ("wall_north", (0, e, h / 2), (length, t, h), "graffiti", [(200, 200, 195), (230, 40, 90), (30, 140, 220), (250, 200, 30)]),
        ("wall_south", (0, -e, h / 2), (length, t, h), "poster", [(180, 170, 150), (200, 60, 40), (40, 90, 160), (240, 230, 210)]),
        ("wall_east", (e, 0, h / 2), (t, length, h), "graffiti", [(120, 130, 140), (120, 220, 60), (240, 120, 20), (150, 50, 200)]),
        ("wall_west", (-e, 0, h / 2), (t, length, h), "poster", [(150, 160, 150), (30, 160, 120), (230, 180, 40), (220, 70, 140)]),
    ]
    parts, textures = [], {}
    for k, (name, pos, size, style, colors) in enumerate(specs):
        textures[name] = _tex(style, 200 + k, colors, px=(2048, 1024))
        parts.append(_box(name, size, pos, name))
    return {"name": "outer_walls", "zone": "structure", "label": "Outer walls",
            "pose": (0, 0, 0, 0), "parts": parts, "textures": textures}


def _canopy():
    cx, cy = ROOF_CENTER
    sx, sy, sz = ROOF_SIZE
    body = sz - ROOF_GRAVEL_LAYER
    parts = [
        _box("roof_body", (sx, sy, body), (0, 0, ROOF_BOTTOM_Z + body / 2), "roof_underside"),
        _box("roof_gravel", (sx, sy, ROOF_GRAVEL_LAYER), (0, 0, ROOF_TOP_Z - ROOF_GRAVEL_LAYER / 2), "roof_gravel"),
    ]
    textures = {
        "roof_underside": _tex("concrete", 300, [(150, 148, 140), (110, 108, 100), (60, 90, 120)]),
        "roof_gravel": _tex("gravel", 301, [(130, 125, 115), (90, 85, 80), (175, 165, 150)], px=(2048, 1536)),
    }
    px, py, ph = PILLAR_SIZE
    k = 0
    for y in PILLAR_YS:
        for x in PILLAR_XS:
            tex = f"pillar_{k}"
            base = 120 + 12 * k
            textures[tex] = _tex("concrete", 310 + k, [(base, base - 4, base - 10), (base - 40, base - 42, base - 45), (180, 60 + 20 * k, 40)],
                                 px=(1024, 2048))
            parts.append(_box(tex, (px, py, ph), (x - cx, y - cy, ph / 2), tex))
            k += 1
    return {"name": "canopy", "zone": "structure", "label": "Canopy (roof + 6 pillars)",
            "pose": (cx, cy, 0, 0), "parts": parts, "textures": textures}


def _single(name, zone, label, xy, base_z, parts, textures):
    return {"name": name, "zone": zone, "label": label, "pose": (xy[0], xy[1], base_z, 0),
            "parts": parts, "textures": textures}


def _objects():
    m = []
    # ---- under the roof (UGV only)
    m.append(_single("crate_stack", "under", "Crate stack", (-1.5, 1.2), 0.0,
                     [_box("body", (0.8, 0.8, 1.0), (0, 0, 0.5), "crate")],
                     {"crate": _tex("crate", 400, [(170, 120, 70), (110, 70, 35), (40, 30, 20)])}))
    m.append(_single("drum", "under", "Drum", (1.8, -1.0), 0.0,
                     [_cyl("body", 0.35, 1.0, (0, 0, 0.5), "drum")],
                     {"drum": _tex("rusty_paint", 401, [(30, 90, 170), (140, 70, 30), (220, 220, 220)])}))
    # ---- on the roof (UAV only)
    m.append(_single("ac_unit", "roof", "AC outdoor unit", (-2.0, -1.2), ROOF_TOP_Z,
                     [_box("body", (0.8, 1.2, 0.8), (0, 0, 0.4), "ac_unit")],
                     {"ac_unit": _tex("grille", 410, [(215, 215, 205), (120, 125, 130), (60, 60, 70)])}))
    m.append(_single("water_tank", "roof", "Water tank", (2.0, 1.2), ROOF_TOP_Z,
                     [_cyl("body", 0.5, 1.0, (0, 0, 0.5), "water_tank")],
                     {"water_tank": _tex("rusty_paint", 411, [(60, 150, 80), (200, 190, 120), (30, 60, 40)])}))
    # ---- shared landmarks outside the canopy (both robots)
    m.append(_single("lm_box", "shared", "Landmark A: tall box", (-6.0, 3.0), 0.0,
                     [_box("body", (1.0, 1.0, 1.5), (0, 0, 0.75), "lm_box")],
                     {"lm_box": _tex("brick", 500, [(170, 70, 50), (210, 200, 185), (90, 40, 30)])}))
    m.append(_single("lm_cylinder", "shared", "Landmark B: column", (-6.0, -3.5), 0.0,
                     [_cyl("body", 0.4, 2.0, (0, 0, 1.0), "lm_cylinder")],
                     {"lm_cylinder": _tex("mosaic", 501, [(40, 120, 200), (240, 240, 230), (230, 160, 30), (200, 40, 60)])}))
    # L-shape: footprint 1.6 x 1.6, arm width 0.6, height 1.2 (corner at -x,-y)
    m.append(_single("lm_lshape", "shared", "Landmark C: L-block", (6.0, 3.5), 0.0,
                     [_box("arm_x", (1.6, 0.6, 1.2), (0, -0.5, 0.6), "lm_lshape"),
                      _box("arm_y", (0.6, 1.0, 1.2), (-0.5, 0.3, 0.6), "lm_lshape")],
                     {"lm_lshape": _tex("camo", 502, [(90, 110, 60), (60, 70, 40), (150, 140, 90), (40, 40, 30)])}))
    # Stairs: 3 steps along +x, each 0.5 deep, 1.0 wide, heights 0.6 / 1.2 / 1.8
    m.append(_single("lm_stairs", "shared", "Landmark D: stairs", (6.0, -3.0), 0.0,
                     [_box(f"step_{i}", (0.5, 1.0, 0.6 * (i + 1)), (-0.5 + 0.5 * i, 0, 0.3 * (i + 1)), "lm_stairs")
                      for i in range(3)],
                     {"lm_stairs": _tex("rusty_paint", 503, [(200, 200, 60), (120, 60, 40), (40, 40, 40)])}))
    # ---- start / goal markers (visual only)
    for name, label, xy, colors, seed in [
        ("start_zone", "Start zone", START_XY, [(40, 170, 80), (230, 240, 230), (20, 80, 40)], 600),
        ("goal_zone", "Goal zone", GOAL_XY, [(210, 50, 50), (240, 230, 220), (90, 20, 20)], 601),
    ]:
        m.append(_single(name, "marker", label, xy, 0.0,
                         [_box("plate", (MARKER_SIZE[0], MARKER_SIZE[1], MARKER_THICKNESS),
                               (0, 0, MARKER_THICKNESS / 2 + 0.001), name, collision=False)],
                         {name: _tex("hazard", seed, colors, px=(1024, 768))}))
    return m


MODELS = [_floor(), _walls(), _canopy()] + _objects()


def model_by_name(name):
    return next(m for m in MODELS if m["name"] == name)
