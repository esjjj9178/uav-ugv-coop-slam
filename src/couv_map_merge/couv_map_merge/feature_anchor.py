"""Inter-map anchors from RGB-D feature matches (method 4 of compare_aruco.py).

For drone node i and UGV node j that look at the same place (checked by projecting the drone node's depth
points, placed with a prior map transform, into the UGV camera with a visibility test), SIFT features of the two
colour images are matched, lifted to 3D with the depth images, and a rigid transform between the two cameras is
estimated with RANSAC. A match is accepted only with enough inliers and close to the prior (gating against the
repeated pillars / wrong matches). Each accepted match gives the drone-map -> UGV-map transform at node i:
    T_i = C_ugv,j * T(drone cam -> ugv cam) * C_drone,i^-1        (C = optical-frame camera pose in its map)
Only onboard data is used; ground truth is not involved.
"""
import os

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from couv_map_merge import registration as R


def load_node(run, ns, nid):
    rgb = cv2.imread(os.path.join(run, f"{ns}_rgb", f"{nid}.jpg"), cv2.IMREAD_GRAYSCALE)
    depth = cv2.imread(os.path.join(run, f"{ns}_depth", f"{nid}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000.0
    return rgb, depth


def backproject(uv, depth, K, near=0.3, far=8.0):
    """Pixel coordinates -> 3D points in the optical frame (nan where the depth is invalid)."""
    u = np.clip(np.round(uv[:, 0]).astype(int), 0, depth.shape[1] - 1)
    v = np.clip(np.round(uv[:, 1]).astype(int), 0, depth.shape[0] - 1)
    z = depth[v, u]
    ok = (z > near) & (z < far)
    p = np.stack([(uv[:, 0] - K[0, 2]) * z / K[0, 0], (uv[:, 1] - K[1, 2]) * z / K[1, 1], z], 1)
    p[~ok] = np.nan
    return p


def kabsch(a, b):
    """Rigid T with b = T a (least squares)."""
    ca, cb = a.mean(0), b.mean(0)
    U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    T = np.eye(4)
    T[:3, :3] = Vt.T @ D @ U.T
    T[:3, 3] = cb - T[:3, :3] @ ca
    return T


def ransac_rigid(a, b, thr, iters, rng):
    best = np.zeros(len(a), bool)
    for _ in range(iters):
        idx = rng.choice(len(a), 3, replace=False)
        T = kabsch(a[idx], b[idx])
        inl = np.linalg.norm(R.apply(T, a) - b, axis=1) < thr
        if inl.sum() > best.sum():
            best = inl
    if best.sum() < 3:
        return None, best
    T = kabsch(a[best], b[best])
    best = np.linalg.norm(R.apply(T, a) - b, axis=1) < thr
    return (kabsch(a[best], b[best]) if best.sum() >= 3 else None), best


def overlap_pairs(run, cam_d, cam_u, K, T_prior, min_frac=0.15, per_node=3, step=16):
    """(drone id, ugv id, overlap fraction) of node pairs that see the same surfaces under T_prior."""
    ids_u = sorted(cam_u)
    depth_u = {j: load_node(run, "ugv", j)[1] for j in ids_u}
    inv_u = np.array([R.inv(cam_u[j]) for j in ids_u])
    pairs = []
    for i in sorted(cam_d):
        _, d = load_node(run, "drone", i)
        v, u = np.mgrid[0:d.shape[0]:step, 0:d.shape[1]:step]
        p = backproject(np.stack([u.ravel(), v.ravel()], 1).astype(float), d, K)
        p = p[np.isfinite(p).all(1)]
        if len(p) < 50:
            continue
        pw = R.apply(T_prior[i] @ cam_d[i], p)                     # in the UGV map
        cand = []
        for j, Ti in zip(ids_u, inv_u):
            q = R.apply(Ti, pw)
            z = q[:, 2]
            front = z > 0.3
            if front.sum() < 0.05 * len(q):
                continue
            uu = K[0, 0] * q[front, 0] / z[front] + K[0, 2]
            vv = K[1, 1] * q[front, 1] / z[front] + K[1, 2]
            inside = (uu >= 0) & (uu < 640) & (vv >= 0) & (vv < 480)
            if not inside.any():
                continue
            dz = depth_u[j][vv[inside].astype(int), uu[inside].astype(int)]
            visible = (dz > 0.3) & (np.abs(dz - z[front][inside]) < 0.3)  # not occluded in the UGV view
            frac = visible.sum() / len(p)
            if frac >= min_frac:
                cand.append((frac, j))
        for frac, j in sorted(cand, reverse=True)[:per_node]:
            pairs.append((i, j, frac))
    return pairs


def match_anchors(run, cam_d, cam_u, K, T_prior, gate_t=0.75, gate_deg=10.0, min_inliers=12, thr=0.05, log=print):
    """Accepted anchors [(drone id, ugv id, T_i, inliers)] and statistics."""
    rng = np.random.default_rng(0)
    sift = cv2.SIFT_create(nfeatures=2000)
    feats = {}

    def features(ns, nid):
        if (ns, nid) not in feats:
            g, d = load_node(run, ns, nid)
            kp, des = sift.detectAndCompute(g, None)
            uv = np.array([k.pt for k in kp]) if kp else np.zeros((0, 2))
            feats[(ns, nid)] = (backproject(uv, d, K) if len(uv) else np.zeros((0, 3)), des)
        return feats[(ns, nid)]

    pairs = overlap_pairs(run, cam_d, cam_u, K, T_prior)
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    anchors, stats = [], {"pairs": len(pairs), "few_matches": 0, "ransac_failed": 0, "gated": 0, "accepted": 0}
    for i, j, _ in pairs:
        pd_, dd = features("drone", i)
        pu, du = features("ugv", j)
        if dd is None or du is None or len(dd) < 10 or len(du) < 10:
            stats["few_matches"] += 1
            continue
        good = [m for m, n in matcher.knnMatch(dd, du, k=2) if m.distance < 0.8 * n.distance]
        a = np.array([pd_[m.queryIdx] for m in good]).reshape(-1, 3)
        b = np.array([pu[m.trainIdx] for m in good]).reshape(-1, 3)
        ok = np.isfinite(a).all(1) & np.isfinite(b).all(1)
        a, b = a[ok], b[ok]
        if len(a) < min_inliers:
            stats["few_matches"] += 1
            continue
        T_cu_cd, inl = ransac_rigid(a, b, thr, 300, rng)
        if T_cu_cd is None or inl.sum() < min_inliers:
            stats["ransac_failed"] += 1
            continue
        T_i = cam_u[j] @ T_cu_cd @ R.inv(cam_d[i])
        dt, dr = R.error(T_i, T_prior[i])
        if dt > gate_t or dr > gate_deg:
            stats["gated"] += 1
            continue
        anchors.append((i, j, T_i, int(inl.sum())))
        stats["accepted"] += 1
    log(f"feature anchors: {stats}")
    return anchors, stats


def robust_anchor_track(s_nodes, anchors_s, anchors_T, window=3.0, max_dev=0.15):
    """Drop anchors deviating from the median translation of their neighbours (within `window` m of path),
    then merge anchors closer than 1 m of path into one mean transform. Returns [(s, T)] sorted by s."""
    s_a, T_a = np.array(anchors_s), list(anchors_T)
    keep = []
    for k in range(len(s_a)):
        nb = np.abs(s_a - s_a[k]) <= window
        med = np.median(np.array([T_a[n][:3, 3] for n in np.where(nb)[0]]), 0)
        if np.linalg.norm(T_a[k][:3, 3] - med) <= max_dev:
            keep.append(k)
    out, k = [], 0
    keep = sorted(keep, key=lambda n: s_a[n])
    while k < len(keep):
        grp = [keep[k]]
        while k + 1 < len(keep) and s_a[keep[k + 1]] - s_a[grp[0]] < 1.0:
            k += 1
            grp.append(keep[k])
        Ts = [T_a[n] for n in grp]
        T = np.eye(4)
        T[:3, :3] = Rotation.from_matrix([t[:3, :3] for t in Ts]).mean().as_matrix()
        T[:3, 3] = np.mean([t[:3, 3] for t in Ts], 0)
        out.append((float(np.mean(s_a[grp])), T))
        k += 1
    return out
