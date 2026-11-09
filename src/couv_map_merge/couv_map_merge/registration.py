"""UGV / drone map registration helpers (numpy + scipy, no ROS spin).

Conventions: 4x4 homogeneous matrices; T_a_b maps points from frame b into frame a.
"""
import math
import os

import numpy as np
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


# ---------------------------------------------------------------- I/O
def load_ply(path):
    """Binary little-endian PLY from rtabmap-export: returns (xyz float32 Nx3, rgb uint8 Nx3)."""
    with open(path, "rb") as f:
        props, n = [], 0
        while True:
            line = f.readline().decode("ascii").strip()
            if line.startswith("element vertex"):
                n, in_vertex = int(line.split()[-1]), True
            elif line.startswith("element"):
                in_vertex = False
            elif line.startswith("property") and in_vertex:
                _, typ, name = line.split()
                props.append((name, {"float": "<f4", "uchar": "u1", "double": "<f8", "int": "<i4"}[typ]))
            elif line == "end_header":
                break
        data = np.frombuffer(f.read(np.dtype(props).itemsize * n), dtype=np.dtype(props), count=n)
    xyz = np.stack([data["x"], data["y"], data["z"]], 1).astype(np.float32)
    rgb = np.stack([data["red"], data["green"], data["blue"]], 1) if "red" in data.dtype.names else \
        np.full((n, 3), 200, np.uint8)
    ok = np.isfinite(xyz).all(1)
    return xyz[ok], rgb[ok]


def load_poses(path):
    """rtabmap-export --poses_format 11: stamp x y z qx qy qz qw id -> (stamps, 4x4 array, ids)."""
    rows = np.array([[float(v) for v in l.split()] for l in open(path) if l.strip() and not l.startswith("#")])
    return rows[:, 0], np.array([to_matrix(r[1:4], r[4:8]) for r in rows]), rows[:, 8].astype(int)


def read_bag(bag, topics):
    """{topic: [(stamp_s, msg), ...]} using header stamps when present."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    storage = "mcap" if any(f.endswith(".mcap") for f in os.listdir(bag)) else "sqlite3"
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, ts = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        h = getattr(msg, "header", None)
        out[topic].append((h.stamp.sec + h.stamp.nanosec * 1e-9 if h else ts * 1e-9, msg))
    return out


# ---------------------------------------------------------------- transforms
def to_matrix(t, q_xyzw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q_xyzw).as_matrix()
    T[:3, 3] = t
    return T


def xyzyaw(x, y, z, yaw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    T[:3, 3] = (x, y, z)
    return T


def inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def apply(T, pts):
    return pts @ T[:3, :3].T + T[:3, 3]


def error(T_est, T_true):
    """(translation error m, rotation error deg) of T_est relative to T_true."""
    E = inv(T_true) @ T_est
    return float(np.linalg.norm(E[:3, 3])), float(np.degrees(Rotation.from_matrix(E[:3, :3]).magnitude()))


class Trajectory:
    """Ground-truth odometry from the bag, interpolated in time."""

    def __init__(self, samples):
        self.t = np.array([s for s, _ in samples])
        p = [m.pose.pose for _, m in samples]
        self.pos = np.array([[q.position.x, q.position.y, q.position.z] for q in p])
        self.rot = Rotation.from_quat([[q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w] for q in p])
        self.slerp = Slerp(self.t, self.rot)

    def at(self, t):
        t = float(np.clip(t, self.t[0], self.t[-1]))
        T = np.eye(4)
        T[:3, :3] = self.slerp([t]).as_matrix()[0]
        T[:3, 3] = [np.interp(t, self.t, self.pos[:, i]) for i in range(3)]
        return T


# ---------------------------------------------------------------- point cloud ops
def voxel_down(pts, voxel, *extra):
    key = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return (pts[idx],) + tuple(e[idx] for e in extra)


def normals(pts, k):
    """PCA normals and planarity (1 - smallest/middle eigen ratio) for every point."""
    tree = cKDTree(pts)
    _, nn = tree.query(pts, k=k)
    nb = pts[nn] - pts[nn].mean(1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nb, nb) / k
    w, v = np.linalg.eigh(cov)
    planarity = 1.0 - w[:, 0] / np.maximum(w[:, 1], 1e-12)
    return v[:, :, 0], planarity


def icp_point_to_plane(src, dst, T_init, cfg, log=None):
    """Point-to-plane ICP. Returns the list of transforms (initial + one per iteration) and per-iteration stats."""
    nrm, planar = normals(dst, cfg["normal_k"])
    keep = planar > cfg["min_planarity"]
    dst, nrm = dst[keep], nrm[keep]
    tree = cKDTree(dst)
    T = T_init.copy()
    steps, stats = [T.copy()], []
    n_it = cfg["max_iterations"]
    sched = cfg["max_corr_dist"]
    for it in range(n_it):
        max_d = sched[min(len(sched) - 1, it * len(sched) // n_it)]
        p = apply(T, src)
        d, j = tree.query(p, distance_upper_bound=max_d)
        ok = np.isfinite(d)
        if ok.sum() < 50:
            stats.append({"iteration": it + 1, "pairs": int(ok.sum()), "rmse": float("nan"), "max_corr": max_d})
            break
        p, q, n = p[ok], dst[j[ok]], nrm[j[ok]]
        r = np.einsum("ij,ij->i", p - q, n)
        cut = np.quantile(np.abs(r), cfg["trim_ratio"])
        m = np.abs(r) <= cut
        p, n, r = p[m], n[m], r[m]
        w = np.ones(len(r))
        if cfg.get("balance_normals", False):  # floor (vertical normals) vs walls/sides get equal total weight
            vert = np.abs(n[:, 2]) > 0.8
            for msk in (vert, ~vert):
                if msk.any():
                    w[msk] = 0.5 * len(r) / msk.sum()
        sw = np.sqrt(w) / cfg.get("sigma_point", 0.03)
        A = np.hstack([np.cross(p, n), n]) * sw[:, None]
        b = -r * sw
        H, g = A.T @ A, A.T @ b
        if cfg.get("prior_sigma_t", 0) > 0:  # T0 prior: keeps unobservable directions (flat floor) at the guess
            D = T @ inv(T_init)
            dev = np.r_[Rotation.from_matrix(D[:3, :3]).as_rotvec(), D[:3, 3]]
            L = np.diag([1 / math.radians(cfg["prior_sigma_rot_deg"]) ** 2] * 3 + [1 / cfg["prior_sigma_t"] ** 2] * 3)
            H, g = H + L, g - L @ dev
        free = [2, 3, 4, 5] if cfg.get("dof", 6) == 4 else list(range(6))  # 4: gravity known (IMU)
        delta = np.zeros(6)
        delta[free] = np.linalg.solve(H[np.ix_(free, free)], g[free])
        dT = np.eye(4)
        dT[:3, :3] = Rotation.from_rotvec(delta[:3]).as_matrix()
        dT[:3, 3] = delta[3:]
        T = dT @ T
        steps.append(T.copy())
        rmse = float(np.sqrt(np.mean(r ** 2)))
        stats.append({"iteration": it + 1, "pairs": int(m.sum()), "rmse": rmse, "max_corr": max_d})
        if log:
            log(f"ICP it {it + 1:2d}: pairs {m.sum():6d}  rmse {rmse * 100:5.2f} cm  max_corr {max_d:.2f} m  "
                f"|dt| {np.linalg.norm(delta[3:]) * 1000:6.2f} mm  |dr| {math.degrees(np.linalg.norm(delta[:3])):.3f} deg")
        if (np.linalg.norm(delta[3:]) < cfg["converge_translation"]
                and math.degrees(np.linalg.norm(delta[:3])) < cfg["converge_rotation_deg"]
                and max_d == sched[-1]):
            break
    return steps, stats


# ---------------------------------------------------------------- the whole problem
def frames_cfg():
    from ament_index_python.packages import get_package_share_directory
    return yaml.safe_load(open(os.path.join(get_package_share_directory("couv_sim_robots"), "config", "frames.yaml")))


def map_origin_in_world(gt, stamps, poses):
    """World pose of a SLAM map origin: GT(base at node 0) * inv(map pose of node 0)."""
    return gt.at(stamps[0]) @ inv(poses[0])


def build_problem(map_dir, bag, merge_cfg, log=print):
    fr = frames_cfg()
    t0 = fr["landing_pad_offset"]
    clouds = {ns: load_ply(os.path.join(map_dir, f"{ns}_cloud.ply")) for ns in ("ugv", "drone")}
    poses = {ns: load_poses(os.path.join(map_dir, f"{ns}_poses.txt")) for ns in ("ugv", "drone")}
    data = read_bag(bag, ["/ugv/ground_truth/odom", "/drone/ground_truth/odom", "/drone/rangefinder"])
    gt = {ns: Trajectory(data[f"/{ns}/ground_truth/odom"]) for ns in ("ugv", "drone")}

    # ground truth of the map-to-map transform (evaluation only)
    W_u = map_origin_in_world(gt["ugv"], poses["ugv"][0], poses["ugv"][1])
    W_d = map_origin_in_world(gt["drone"], poses["drone"][0], poses["drone"][1])
    T_true = inv(W_u) @ W_d

    # initial guesses from onboard information only
    T_T0 = xyzyaw(t0["x"], t0["y"], t0["z"], t0["yaw"])
    steps = [("maps in their own frames (identity)", np.eye(4)), ("T0 (landing pad offset)", T_T0)]
    T_init = T_T0
    if merge_cfg["climb_correction"] == "rangefinder":
        rf = np.array([(s, m.ranges[0]) for s, m in data["/drone/rangefinder"] if np.isfinite(m.ranges[0])])
        t_first = poses["drone"][0][0]
        r_pad = np.median(rf[rf[:, 0] < rf[0, 0] + 1.0, 1])               # on the pad, before take-off
        r_first = np.interp(t_first, rf[:, 0], rf[:, 1])
        climb = r_first - r_pad - poses["drone"][1][0][2, 3]                # VO origin height above the pad seat
        T_init = T_T0 @ xyzyaw(0, 0, climb, 0)
        steps.append((f"T0 + rangefinder climb {climb:+.3f} m", T_init))
        log(f"drone VO started at t={t_first:.2f}s: rangefinder {r_pad:.3f} -> {r_first:.3f} m, climb correction {climb:+.3f} m")
    p = merge_cfg["perturbation"]
    if any(abs(v) > 0 for v in p.values()):
        T_init = T_init @ xyzyaw(p["x"], p["y"], p["z"], math.radians(p["yaw_deg"]))
        steps.append((f"perturbed initial guess {p}", T_init))
    return {"clouds": clouds, "poses": poses, "gt": gt, "T_true": T_true, "steps": steps, "T_init": T_init,
            "W_u": W_u}


def register(problem, icp_cfg, log=print):
    src = voxel_down(problem["clouds"]["drone"][0], icp_cfg["voxel"])[0]
    if icp_cfg.get("source_radius", 0) > 0:  # drone-map points near its origin (least SLAM drift)
        src = src[np.linalg.norm(src[:, :2], axis=1) < icp_cfg["source_radius"]]
    if icp_cfg.get("source_max_z", 0) > 0:  # drop points the UGV cannot see (roof top, rooftop objects)
        src = src[apply(problem["T_init"], src)[:, 2] < icp_cfg["source_max_z"]]
    dst = voxel_down(problem["clouds"]["ugv"][0], icp_cfg["voxel"])[0]
    log(f"ICP: drone {len(src)} pts -> ugv {len(dst)} pts (voxel {icp_cfg['voxel']} m)")
    T_steps, stats = icp_point_to_plane(src, dst, problem["T_init"], icp_cfg, log)
    steps = list(problem["steps"])
    steps += [(f"ICP iteration {s['iteration']} (max corr {s['max_corr']:.2f} m, rmse {s['rmse'] * 100:.1f} cm)", T)
              for s, T in zip(stats, T_steps[1:])]
    return steps, stats
