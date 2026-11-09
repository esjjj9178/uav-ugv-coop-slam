#!/usr/bin/env python3
"""Register the drone map to the UGV map with the ArUco measurements of a serpentine run and
compare against the earlier straight-line run (T0 + rangefinder, no ICP). Writes figures and
results.yaml to --docs.

    ros2 run couv_map_merge compare_aruco.py --run ~/ros2_ws/maps/serpentine_aruco \
        --baseline ~/ros2_ws/maps/couv_20261027_200102_pitch90 --docs <dir>

Methods (drone map -> UGV map), all from onboard data; ground truth only for scoring:
  A  T0 only                     landing-pad offset at the start
  B  ArUco start                 marker measured from the hovering drone before departure
  C  ArUco start + end           + marker at the goal; drift between the two spread along the
                                 drone trajectory by path length (two-anchor pose-graph)
  D  C + ICP                     point-to-plane ICP with T0 prior, accepted only within 10 cm / 2 deg
Score: position / heading error of every drone SLAM node expressed in the UGV map.
"""
import argparse
import glob
import math
import os

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp

from couv_map_merge import feature_anchor as F
from couv_map_merge import registration as R

DRONE_COLOUR, UGV_COLOUR = "#2a78d6", "#eb6834"
METHOD_COLOURS = {"A": "#8c8b85", "B": "#1baf7a", "C": "#2a78d6", "D": "#d55181", "old": "#eda100"}


# ---------------------------------------------------------------- helpers
def odom_traj(samples):
    return R.Trajectory(samples) if samples else None


def pose_at(poses, odom, t):
    """SLAM pose of base_link in its map at time t: nearest node corrected by the odometry motion since it."""
    st, P, _ = poses
    k = int(np.argmin(np.abs(st - t)))
    return P[k] @ R.inv(odom.at(st[k])) @ odom.at(t)


def camera_extrinsic(fr, pitch_deg):
    """drone/base_link -> drone/camera_color_optical_frame."""
    c = fr["drone"]["camera"]
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", [0, math.radians(pitch_deg), 0]).as_matrix()
    T[:3, 3] = (c["x"], c["y"], c["z"])
    opt = np.eye(4)
    opt[:3, :3] = Rotation.from_euler("xyz", [-math.pi / 2, 0, -math.pi / 2]).as_matrix()
    return T @ opt


def marker_in_ugv(fr):
    u = fr["ugv"]
    T = np.eye(4)
    T[:3, :3] = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float)   # marker x = -y_ugv, y = +x_ugv
    T[2, 3] = u["landing_pad"]["top_height"] - u["wheel_radius"]
    return T


def mean_transform(Ts):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_matrix([t[:3, :3] for t in Ts]).mean().as_matrix()
    T[:3, 3] = np.mean([t[:3, 3] for t in Ts], 0)
    return T


def interp_transform(A, B, w):
    """A * (A^-1 B)^w."""
    D = R.inv(A) @ B
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(w * Rotation.from_matrix(D[:3, :3]).as_rotvec()).as_matrix()
    T[:3, 3] = w * D[:3, 3]
    return A @ T


def node_errors(per_node_T, poses_d, gt_d, W_u):
    """Position [m] / rotation [deg] error of each drone node in the UGV map."""
    st, P, _ = poses_d
    pe, re = [], []
    for T, ti, Pi in zip(per_node_T, st, P):
        est, truth = T @ Pi, R.inv(W_u) @ gt_d.at(ti)
        pe.append(np.linalg.norm(est[:3, 3] - truth[:3, 3]))
        re.append(np.degrees(Rotation.from_matrix(est[:3, :3].T @ truth[:3, :3]).magnitude()))
    return np.array(pe), np.array(re)


def summary(pe, re):
    return {"median_cm": round(float(np.median(pe)) * 100, 1), "max_cm": round(float(pe.max()) * 100, 1),
            "start_cm": round(float(pe[0]) * 100, 1), "end_cm": round(float(pe[-1]) * 100, 1),
            "median_deg": round(float(np.median(re)), 2), "max_deg": round(float(re.max()), 2)}


def node_clouds(run, ns, step=6, max_range=8.0):
    """Per-node coloured clouds in the camera optical frame from the exported images."""
    out = {}
    for f in sorted(glob.glob(os.path.join(run, f"{ns}_depth", "*.png"))):
        nid = int(os.path.basename(f)[:-4])
        depth = cv2.imread(f, cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000.0
        rgb = cv2.imread(os.path.join(run, f"{ns}_rgb", f"{nid}.jpg"))[:, :, ::-1]
        # ROS camera_info style (rows/cols/data, not opencv-matrix): skip the "%YAML:1.0" directive
        calib = yaml.safe_load("".join(open(os.path.join(run, f"{ns}_calib", f"{nid}.yaml")).readlines()[1:]))
        K = np.array(calib["camera_matrix"]["data"]).reshape(3, 3)
        v, u = np.mgrid[0:depth.shape[0]:step, 0:depth.shape[1]:step]
        z = depth[v, u]
        ok = (z > 0.2) & (z < max_range)
        x = (u[ok] - K[0, 2]) * z[ok] / K[0, 0]
        y = (v[ok] - K[1, 2]) * z[ok] / K[1, 1]
        out[nid] = (np.stack([x, y, z[ok]], 1), rgb[v[ok], u[ok]])
    return out


def load_camera_poses(run, ns):
    st, P, ids = R.load_poses(os.path.join(run, f"{ns}_camera_poses.txt"))
    return {int(i): T for i, T in zip(ids, P)}


def assemble(clouds, cam_poses, node_T=None):
    pts = []
    for nid, (p, _) in clouds.items():
        if nid in cam_poses:
            T = cam_poses[nid] if node_T is None else node_T[nid] @ cam_poses[nid]
            pts.append(R.apply(T, p))
    return np.vstack(pts)


SEGMENTS = ["start (take-off, ArUco)", "row 1 south y=-6.5", "turn 1 x=+8", "row 2 middle y=0 (over canopy)",
            "turn 2 x=-8", "row 3 north y=+6.5", "goal (wait, ArUco, landing)"]


def interp_anchors(s_nodes, anchors):
    """Per-node transforms interpolated along the path length between anchors [(s, T)] (held outside)."""
    anchors = sorted(anchors, key=lambda a: a[0])
    out = []
    for sv in s_nodes:
        if sv <= anchors[0][0]:
            out.append(anchors[0][1])
        elif sv >= anchors[-1][0]:
            out.append(anchors[-1][1])
        else:
            k = next(n for n in range(len(anchors) - 1) if anchors[n + 1][0] >= sv)
            (s0, T0_), (s1, T1_) = anchors[k], anchors[k + 1]
            out.append(interp_transform(T0_, T1_, (sv - s0) / max(s1 - s0, 1e-9)))
    return out


def overlap_error(a, b, near=0.5, thr=0.10, b_normals=None):
    """Consistency of two clouds where they overlap. For every point of `a` the nearest point of `b` within
    `near` defines the overlap; the gap is the distance to b's local surface (point-to-plane where b is planar,
    so that different samples of the same floor/wall are not counted, point-to-point elsewhere).
    Returns (gap, overlap mask, mismatch mask = overlap & gap > thr)."""
    from scipy.spatial import cKDTree
    if b_normals is None:
        b_normals = R.normals(b, 12)
    n, planar = b_normals
    d, idx = cKDTree(b).query(a, distance_upper_bound=near)
    ov = np.isfinite(d)
    gap = np.full(len(a), np.inf)
    q, k = a[ov] - b[idx[ov]], idx[ov]
    gap[ov] = np.where(planar[k] > 0.6, np.abs(np.einsum("ij,ij->i", q, n[k])), d[ov])
    return gap, ov, ov & (gap > thr)


def local_node_errors(per_node_T, poses_d, poses_u, gt_d, gt_u):
    """Error of every drone node against where it belongs relative to the UGV map *nearby*: the UGV node closest in
    space (ground truth) defines the local frame P_u,j * GT_u(t_j)^-1. Independent of the UGV map's own drift, i.e.
    how well the merged map overlaps (ground truth only for scoring)."""
    st_d, P_d, _ = poses_d
    st_u, P_u, _ = poses_u
    gu = np.array([gt_u.at(t)[:2, 3] for t in st_u])
    out = []
    for T, t, P in zip(per_node_T, st_d, P_d):
        g = gt_d.at(t)
        j = int(np.argmin(np.linalg.norm(gu - g[:2, 3], axis=1)))
        truth = P_u[j] @ R.inv(gt_u.at(st_u[j])) @ g
        out.append(np.linalg.norm((T @ P)[:3, 3] - truth[:3, 3]))
    return np.array(out)


def segment_of_nodes(st_d, gt_d, states, waypoints, reach=0.3):
    """Serpentine segment index of every drone node: before cruise -> 0, legs between the cruise waypoints
    wp0..wp5 -> 1..5 (advanced when the true position comes within `reach` of the next waypoint), after
    cruise -> 6."""
    t_cruise = next(t for t, s in states if s == "cruise")
    t_after = next((t for t, s in states if s == "wait_ugv"), 1e12)
    seg, leg = [], -1
    for t in st_d:
        p = gt_d.at(t)[:2, 3]
        while t >= t_cruise and leg + 1 < len(waypoints) and np.linalg.norm(p - waypoints[leg + 1]) < reach:
            leg += 1
        seg.append(0 if t < t_cruise or leg < 0 else 6 if t >= t_after else min(leg + 1, 5))
    return np.array(seg)


# ---------------------------------------------------------------- registration methods
NAMES = {"A": "T0 only", "B": "ArUco start", "C": "ArUco start+end (drift spread)", "D": "C + ICP",
         "F": "T0 + ArUco end + RGB-D feature anchors"}


def solve(run, pitch=45.0):
    """ArUco relations and per-node drone-map -> UGV-map transforms of methods A-D (also used by aruco_viz.py)."""
    fr = R.frames_cfg()
    from ament_index_python.packages import get_package_share_directory
    mcfg = yaml.safe_load(open(os.path.join(get_package_share_directory("couv_map_merge"), "config", "merge.yaml")))

    # ---- data
    poses = {ns: R.load_poses(os.path.join(run, f"{ns}_poses.txt")) for ns in ("ugv", "drone")}
    bag = R.read_bag(os.path.join(run, "aux_bag"), ["/ugv/ground_truth/odom", "/drone/ground_truth/odom",
                                                   "/mission/state", "/drone/aruco/marker_pose",
                                                   "/ugv/vo_odom", "/drone/vo_odom"])
    gt = {ns: R.Trajectory(bag[f"/{ns}/ground_truth/odom"]) for ns in ("ugv", "drone")}
    vo = {ns: odom_traj(bag[f"/{ns}/vo_odom"]) for ns in ("ugv", "drone")}
    states = [(t, m.data) for t, m in bag["/mission/state"]]
    W_u = R.map_origin_in_world(gt["ugv"], *poses["ugv"][:2])
    T_db_cam, T_ub_marker = camera_extrinsic(fr, pitch), marker_in_ugv(fr)

    def window(name):
        t0 = next(t for t, s in states if s == name)
        t1 = min([t for t, s in states if t > t0] + [1e12])
        return t0, t1

    # ---- ArUco relation of the two maps at each measurement
    aruco = {}
    for phase in ("aruco_start", "aruco_end"):
        t0, t1 = window(phase)
        Ts, raw = [], []
        for t, m in bag["/drone/aruco/marker_pose"]:
            if t0 <= t <= t1:
                p, q = m.pose.position, m.pose.orientation
                T_cm = R.to_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
                T_ub_db = T_ub_marker @ R.inv(T_cm) @ R.inv(T_db_cam)
                Ts.append(pose_at(poses["ugv"], vo["ugv"], t) @ T_ub_db @ R.inv(pose_at(poses["drone"], vo["drone"], t)))
                raw.append((t, T_ub_db))
        aruco[phase] = {"T": mean_transform(Ts), "n": len(Ts), "t": 0.5 * (t0 + t1), "raw": raw}
        # accuracy of the relative pose itself (ground truth only for scoring)
        errs = [np.linalg.norm((T_ub_db - R.inv(gt["ugv"].at(t)) @ gt["drone"].at(t))[:3, 3]) for t, T_ub_db in raw]
        aruco[phase]["relpose_err_cm"] = round(float(np.median(errs)) * 100, 2) if errs else None
        print(f"{phase}: {len(Ts)} detections, UGV->drone relative pose error (median) {aruco[phase]['relpose_err_cm']} cm")

    # ---- methods -> per-node transforms
    st_d, P_d, ids_d = poses["drone"]
    t_first = st_d[0]
    T0 = R.xyzyaw(*(fr["landing_pad_offset"][k] for k in ("x", "y", "z", "yaw")))
    T_A = pose_at(poses["ugv"], vo["ugv"], t_first) @ T0 @ R.inv(pose_at(poses["drone"], vo["drone"], t_first))
    T_B, T_E = aruco["aruco_start"]["T"], aruco["aruco_end"]["T"]
    # path-length fraction of every drone node between the two ArUco captures
    pos = P_d[:, :3, 3]
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))]
    s_a = np.interp(aruco["aruco_start"]["t"], st_d, s)
    s_b = np.interp(aruco["aruco_end"]["t"], st_d, s)
    w = np.clip((s - s_a) / max(s_b - s_a, 1e-6), 0, 1)
    per_node = {"A": [T_A] * len(st_d), "B": [T_B] * len(st_d), "C": [interp_transform(T_B, T_E, wi) for wi in w]}

    # 4 (F): T0 at the start, ArUco at the goal, RGB-D feature anchors in between (prior/gating: C)
    cam = {ns: load_camera_poses(run, ns) for ns in ("ugv", "drone")}
    calib = yaml.safe_load("".join(open(os.path.join(run, "drone_calib", f"{int(ids_d[0])}.yaml")).readlines()[1:]))
    K = np.array(calib["camera_matrix"]["data"]).reshape(3, 3)
    id2s = {int(i): sv for i, sv in zip(ids_d, s)}
    prior = {int(i): T for i, T in zip(ids_d, per_node["C"])}
    feat, feat_stats = F.match_anchors(run, cam["drone"], cam["ugv"], K, prior, gate_t=0.5, gate_deg=8.0)
    track = F.robust_anchor_track(s, [id2s[a[0]] for a in feat], [a[2] for a in feat]) if feat else []
    per_node["F"] = interp_anchors(s, [(0.0, T_A), (s_b, T_E)] + [(sv, T) for sv, T in track if sv < s_b])
    feat_info = {"stats": feat_stats, "anchors_used": len(track),
                 "anchor_nodes": [a[0] for a in feat], "anchor_s": [round(sv, 1) for sv, _ in track]}
    print(f"method 4: {feat_stats['accepted']} feature matches -> {len(track)} anchors along the path")

    # ICP refinement (rigid, with prior, acceptance rule) of A and C; D = C + ICP as adopted.
    # "<k>+ICP" is the forced result (correction applied even when the acceptance rule rejects it).
    clouds = {ns: node_clouds(run, ns) for ns in ("ugv", "drone")}
    ugv_pts = R.voxel_down(assemble(clouds["ugv"], cam["ugv"]), 0.05)[0]
    icp_cfg = dict(mcfg["icp"])
    icp_cfg["source_max_z"] = 0  # roof filter handled below (UGV-map z)
    icp = {}
    for k in "AC":
        id2T = {int(i): T for i, T in zip(ids_d, per_node[k])}
        src = R.voxel_down(assemble(clouds["drone"], cam["drone"], id2T), 0.05)[0]
        src = src[src[:, 2] < 2.5 - 0.0625]
        dT = R.icp_point_to_plane(src, ugv_pts, np.eye(4), icp_cfg)[0][-1]
        ct, cr = R.error(dT, np.eye(4))
        acc = ct <= mcfg["acceptance"]["max_correction_t"] and cr <= mcfg["acceptance"]["max_correction_rot_deg"]
        icp[k] = {"dT": dT, "ct": ct, "cr": cr, "accepted": bool(acc)}
        per_node[k + "+ICP"] = [dT @ T for T in per_node[k]]
        print(f"ICP on {k}: correction {ct * 100:.1f} cm / {cr:.2f} deg -> {'accepted' if acc else 'rejected'}")
    per_node["D"] = per_node["C+ICP"] if icp["C"]["accepted"] else per_node["C"]
    return {"poses": poses, "gt": gt, "W_u": W_u, "aruco": aruco, "per_node": per_node, "clouds": clouds,
            "cam": cam, "icp": icp["C"], "icp_all": icp, "merge_cfg": mcfg, "states": states,
            "features": feat_info}


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--baseline", default="")
    ap.add_argument("--docs", required=True)
    ap.add_argument("--pitch", type=float, default=45.0)
    args = ap.parse_args()
    run, docs = os.path.expanduser(args.run), os.path.expanduser(args.docs)
    os.makedirs(docs, exist_ok=True)
    S = solve(run, args.pitch)
    poses, gt, W_u, aruco, per_node, clouds, cam = (S[k] for k in ("poses", "gt", "W_u", "aruco", "per_node", "clouds", "cam"))
    ct, cr, acc = S["icp"]["ct"], S["icp"]["cr"], S["icp"]["accepted"]
    st_d, ids_d = poses["drone"][0], poses["drone"][2]
    names, mcfg = NAMES, S["merge_cfg"]

    # ---- scores
    res = {"aruco": {k: {"detections": v["n"], "relative_pose_error_cm": v["relpose_err_cm"]} for k, v in aruco.items()},
           "icp_on_C": {"correction_cm": round(ct * 100, 1), "correction_deg": round(cr, 2), "accepted": bool(acc)},
           "methods": {}}
    errs = {}
    for k in "ABCD":
        errs[k] = node_errors(per_node[k], poses["drone"], gt["drone"], W_u)
        res["methods"][f"{k}: {names[k]}"] = summary(*errs[k])
    # SLAM quality of both maps (ATE, own map, anchored at the first node)
    for ns in ("ugv", "drone"):
        stn, Pn, _ = poses[ns]
        W = R.map_origin_in_world(gt[ns], stn, Pn)
        e = np.array([np.linalg.norm((W @ Pn[i])[:3, 3] - gt[ns].at(stn[i])[:3, 3]) for i in range(len(stn))])
        res[f"slam_ate_{ns}"] = {"rmse_cm": round(float(np.sqrt((e ** 2).mean())) * 100, 1),
                                 "max_cm": round(float(e.max()) * 100, 1), "nodes": int(len(stn))}

    old = None
    if args.baseline:
        b = os.path.expanduser(args.baseline)
        pr = R.build_problem(b, os.path.join(b, "aux_bag"), mcfg, log=lambda *a: None)
        Wb = R.map_origin_in_world(pr["gt"]["ugv"], *pr["poses"]["ugv"][:2])
        old = node_errors([pr["T_init"]] * len(pr["poses"]["drone"][0]), pr["poses"]["drone"], pr["gt"]["drone"], Wb)
        res["baseline_straight_run_T0_plus_climb_no_icp"] = summary(*old)
        res["baseline_slam_ate_drone_cm"] = 13.6

    # ---- numbered methods per stage: 1 relative pose only, 2 + ICP, 3 ArUco start+end + ICP,
    # 4 T0 + ArUco end + RGB-D feature anchors (no ICP).
    # ICP rows: adopted (acceptance rule applied) and forced. Error rate = end error / drone path length.
    def drone_path_len(g, t0, t1):
        p = np.array([g.at(t)[:3, 3] for t in np.arange(t0, t1, 0.1)])
        return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())

    def row(pe, length, extra=None):
        r = {"median_cm": round(float(np.median(pe)) * 100, 1), "max_cm": round(float(pe.max()) * 100, 1),
             "end_cm": round(float(pe[-1]) * 100, 1), "end_rate_pct": round(float(pe[-1]) / length * 100, 2),
             "median_rate_pct": round(float(np.median(pe)) / length * 100, 2)}
        return {**r, **(extra or {})}

    def icp_note(i):
        return {"icp_correction": f"{i['ct'] * 100:.1f} cm / {i['cr']:.2f} deg",
                "icp_accepted": i["accepted"]}

    numbered = {}
    L4 = drone_path_len(gt["drone"], st_d[0], st_d[-1])
    ia = S["icp_all"]
    e4 = {k: node_errors(per_node[k], poses["drone"], gt["drone"], W_u)[0] for k in ("A", "A+ICP", "C", "C+ICP", "F")}
    adopted = {k: e4[k + "+ICP"] if ia[k]["accepted"] else e4[k] for k in "AC"}
    loc4 = {k: local_node_errors(per_node[k], poses["drone"], poses["ugv"], gt["drone"], gt["ugv"])
            for k in ("A", "A+ICP", "C+ICP", "F")}

    def local(k):
        e = loc4[k]
        return {"local_median_cm": round(float(np.median(e)) * 100, 1), "local_max_cm": round(float(e.max()) * 100, 1),
                "local_end_cm": round(float(e[-1]) * 100, 1)}
    numbered["stage4_serpentine"] = {
        "drone_path_m": round(L4, 1),
        "1 relative pose only (T0)": row(e4["A"], L4, local("A")),
        "2 relative pose + ICP (adopted)": row(adopted["A"], L4, icp_note(ia["A"])),
        "2 relative pose + ICP (forced)": row(e4["A+ICP"], L4, local("A+ICP")),
        "3 ArUco start+end + ICP (adopted)": row(adopted["C"], L4, icp_note(ia["C"])),
        "3 ArUco start+end + ICP (forced)": row(e4["C+ICP"], L4, local("C+ICP")),
        "4 T0 + ArUco end + RGB-D features": row(e4["F"], L4, {**local("F"), "feature_anchors": S["features"]["anchors_used"],
                                                               "feature_matching": S["features"]["stats"]}),
    }
    if args.baseline:
        nb = len(pr["poses"]["drone"][0])
        L3 = drone_path_len(pr["gt"]["drone"], pr["poses"]["drone"][0][0], pr["poses"]["drone"][0][-1])
        T_icp3 = R.register(pr, mcfg["icp"], log=lambda *a: None)[0][-1][1]
        ct3, cr3 = R.error(T_icp3, pr["T_init"])
        acc3 = ct3 <= mcfg["acceptance"]["max_correction_t"] and cr3 <= mcfg["acceptance"]["max_correction_rot_deg"]
        e3 = {k: node_errors([T] * nb, pr["poses"]["drone"], pr["gt"]["drone"], Wb)[0]
              for k, T in (("T0", pr["steps"][1][1]), ("T0+climb", pr["T_init"]), ("ICP", T_icp3))}
        note3 = icp_note({"ct": ct3, "cr": cr3, "accepted": bool(acc3)})
        numbered["stage3_straight"] = {
            "drone_path_m": round(L3, 1),
            "1 relative pose only (T0)": row(e3["T0"], L3),
            "1 relative pose only (T0 + rangefinder climb)": row(e3["T0+climb"], L3),
            "2 relative pose + ICP (adopted)": row(e3["ICP"] if acc3 else e3["T0+climb"], L3, note3),
            "2 relative pose + ICP (forced)": row(e3["ICP"], L3),
            "3 ArUco": "not available (no marker in stage 3)",
        }
    res["numbered_methods"] = numbered

    # ---- error per serpentine segment for methods 1-3 (ICP forced), with the drift of both maps themselves
    from ament_index_python.packages import get_package_share_directory
    wps = np.array(yaml.safe_load(open(os.path.join(get_package_share_directory("couv_sim_robots"), "config",
                                                    "mission_serpentine.yaml")))["drone"]["waypoints"], float)
    seg = segment_of_nodes(st_d, gt["drone"], S["states"], wps)
    P_d = poses["drone"][1]
    pos_gt = np.array([gt["drone"].at(t)[:3, 3] for t in st_d])
    s_cum = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pos_gt, axis=0), axis=1))]
    Wd = R.map_origin_in_world(gt["drone"], st_d, P_d)
    drift_d = np.linalg.norm(np.array([(Wd @ P)[:3, 3] for P in P_d]) - pos_gt, axis=1)
    st_u, P_u, _ = poses["ugv"]
    drift_u_nodes = np.linalg.norm(np.array([(W_u @ P)[:3, 3] - gt["ugv"].at(t)[:3, 3] for t, P in zip(st_u, P_u)]), axis=1)
    drift_u = drift_u_nodes[[int(np.argmin(np.abs(st_u - t))) for t in st_d]]
    meth = {"1 T0 only": "A", "2 T0 + ICP (forced)": "A+ICP", "3 ArUco start+end + ICP (forced)": "C+ICP",
            "4 T0 + ArUco + RGB-D features": "F"}
    segs = {}
    for i, name in enumerate(SEGMENTS):
        m = seg == i
        if not m.any():
            continue
        last = np.where(m)[0][-1]
        segs[name] = {"nodes": int(m.sum()), "flown_m_at_end": round(float(s_cum[last]), 1),
                      **{k: {"median_cm": round(float(np.median(e4[c][m])) * 100, 1),
                             "max_cm": round(float(e4[c][m].max()) * 100, 1),
                             "end_cm": round(float(e4[c][last]) * 100, 1),
                             "local_median_cm": round(float(np.median(loc4[c][m])) * 100, 1),
                             "local_end_cm": round(float(loc4[c][last]) * 100, 1)} for k, c in meth.items()},
                      "drone_map_drift_end_cm": round(float(drift_d[last]) * 100, 1),
                      "ugv_map_drift_end_cm": round(float(drift_u[last]) * 100, 1)}
    res["segments_serpentine"] = segs

    # ---- overlap consistency of the merged map (independent of the UGV map's own drift): drone points that
    # have a UGV point within 0.5 m are in the overlap; > 10 cm to the nearest UGV point is a mismatch
    ugv_c = R.voxel_down(assemble(clouds["ugv"], cam["ugv"]), 0.04)[0]
    cons = {}
    for name, k in (("1 T0 only", "A"), ("2 T0 + ICP (forced)", "A+ICP"), ("3 ArUco start+end + ICP (forced)", "C+ICP"),
                    ("4 T0 + ArUco + RGB-D features", "F")):
        id2T = {int(i): T for i, T in zip(ids_d, per_node[k])}
        dr = R.voxel_down(assemble(clouds["drone"], cam["drone"], id2T), 0.04)[0]
        d, ov, bad = overlap_error(dr, ugv_c)
        cons[name] = {"overlap_points": int(ov.sum()), "median_gap_cm": round(float(np.median(d[ov])) * 100, 1),
                      "mismatch_pct": round(float(bad.sum() / ov.sum()) * 100, 1)}
    res["overlap_consistency"] = cons
    yaml.safe_dump(res, open(os.path.join(docs, "results.yaml"), "w"), sort_keys=False, allow_unicode=True)
    print(yaml.safe_dump(res, sort_keys=False, allow_unicode=True))

    # ---- figures
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})

    # 1. trajectories
    fig, ax = plt.subplots(figsize=(8, 8), dpi=120)
    for ns, col in (("ugv", UGV_COLOUR), ("drone", DRONE_COLOUR)):
        g = gt[ns]
        ax.plot(g.pos[:, 0], g.pos[:, 1], color=col, lw=2, label=f"{ns.upper()} (ground truth)")
    for phase, mk in (("aruco_start", "o"), ("aruco_end", "s")):
        t = aruco[phase]["t"]
        p = gt["drone"].at(t)[:3, 3]
        ax.plot(p[0], p[1], mk, color="#2b2b2a", ms=9, label=f"{phase} (drone hover)")
    ax.add_patch(plt.Rectangle((-4, -3), 8, 6, fill=False, ls="--", ec="#2b2b2a", label="canopy roof"))
    for x, y in ((-6, 3), (-6, -3.5), (6, 3.5), (6, -3)):
        ax.plot(x, y, "^", color="#1baf7a", ms=10)
    ax.plot([], [], "^", color="#1baf7a", label="shared landmarks")
    ax.set_xlim(-10, 10), ax.set_ylim(-10, 10), ax.set_aspect("equal"), ax.grid(alpha=0.3)
    ax.set_xlabel("x [m]"), ax.set_ylabel("y [m]")
    ax.set_title("Serpentine mission: UGV and drone paths, ArUco captures", loc="left")
    ax.legend(loc="upper left", fontsize=8, frameon=False)
    fig.tight_layout(), fig.savefig(os.path.join(docs, "01_paths.png")), plt.close(fig)

    # 2. error along the flight
    fig, ax = plt.subplots(figsize=(10, 5), dpi=120)
    tt = st_d - st_d[0]
    for k in "ABCD":
        rejected = k == "D" and not acc   # D == C then; dashed so C stays visible
        ax.plot(tt, errs[k][0] * 100, color=METHOD_COLOURS[k], lw=1.5 if rejected else 2, ls="--" if rejected else "-",
                label=f"{k}: {names[k]}" + (" (ICP rejected = C)" if rejected else ""))
    if old is not None:
        ax.plot(pr["poses"]["drone"][0] - pr["poses"]["drone"][0][0], old[0] * 100, color=METHOD_COLOURS["old"],
                lw=2, ls="--", label="earlier straight run: T0 + rangefinder (no ICP)")
    for phase in ("aruco_start", "aruco_end"):
        ax.axvline(aruco[phase]["t"] - st_d[0], color="#6b6a64", ls=":", lw=1)
        ax.text(aruco[phase]["t"] - st_d[0], ax.get_ylim()[1] * 0.95, phase, fontsize=8, color="#6b6a64")
    ax.set_xlabel("time since drone map start [s]"), ax.set_ylabel("drone pose error in UGV map [cm]")
    ax.set_title("Map registration error along the drone trajectory (vs ground truth)", loc="left")
    ax.grid(alpha=0.3), ax.legend(fontsize=8, frameon=False)
    fig.tight_layout(), fig.savefig(os.path.join(docs, "02_error_along_flight.png")), plt.close(fig)

    # 3. summary bars
    fig, ax = plt.subplots(figsize=(9, 4.5), dpi=120)
    labels, med, mx, cols = [], [], [], []
    if old is not None:
        labels.append("earlier straight run\nT0 + rangefinder"), med.append(np.median(old[0]) * 100)
        mx.append(old[0].max() * 100), cols.append(METHOD_COLOURS["old"])
    for k in "ABCD":
        labels.append(f"{k}\n{names[k]}"), med.append(np.median(errs[k][0]) * 100)
        mx.append(errs[k][0].max() * 100), cols.append(METHOD_COLOURS[k])
    x = np.arange(len(labels))
    ax.bar(x - 0.2, med, 0.38, color=cols, label="median")
    ax.bar(x + 0.2, mx, 0.38, color=cols, alpha=0.45, label="max")
    for xi, m1, m2 in zip(x, med, mx):
        ax.text(xi - 0.2, m1 + 0.5, f"{m1:.1f}", ha="center", fontsize=8)
        ax.text(xi + 0.2, m2 + 0.5, f"{m2:.1f}", ha="center", fontsize=8)
    ax.set_xticks(x), ax.set_xticklabels(labels, fontsize=8), ax.set_ylabel("drone pose error [cm]")
    ax.set_title("Registration error: median (solid) / max (light)", loc="left")
    fig.tight_layout(), fig.savefig(os.path.join(docs, "03_error_summary.png")), plt.close(fig)

    # 4. merged maps, top and side views, A vs C
    ugv_all = assemble(clouds["ugv"], cam["ugv"])
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), dpi=110)
    for col, k in enumerate(("A", "C")):
        id2T = {int(i): T for i, T in zip(ids_d, per_node[k])}
        dr = assemble(clouds["drone"], cam["drone"], id2T)
        for row, (i, j, lab) in enumerate(((0, 1, "y"), (0, 2, "z"))):
            a = axes[row, col]
            a.scatter(ugv_all[::3, i], ugv_all[::3, j], s=0.2, c=UGV_COLOUR, alpha=0.5, label="UGV map")
            a.scatter(dr[::3, i], dr[::3, j], s=0.2, c=DRONE_COLOUR, alpha=0.5, label="drone map")
            a.set_aspect("equal"), a.set_xlabel("x [m] (UGV map)"), a.set_ylabel(f"{lab} [m]")
            a.set_title(f"{k}: {names[k]}  ({'top' if row == 0 else 'side'} view)", loc="left", fontsize=10)
    axes[0, 0].legend(markerscale=20, fontsize=8, frameon=False, loc="upper right")
    fig.tight_layout(), fig.savefig(os.path.join(docs, "04_merged_maps_A_vs_C.png")), plt.close(fig)

    # 5. ArUco detection images
    for f in glob.glob(os.path.join(run, "images", "*.png")):
        cv2.imwrite(os.path.join(docs, "05_" + os.path.basename(f)), cv2.imread(f))
    print("figures written to", docs)


if __name__ == "__main__":
    main()
