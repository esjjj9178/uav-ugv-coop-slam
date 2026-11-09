#!/usr/bin/env python3
"""Check a D435 depth image against the known canopy_world geometry.

Takes one depth image + camera_info from the UGV camera and the UGV ground-truth pose, and
ray-casts the centre image row (horizontal rays at camera height; the UGV camera is level)
against the boxes/cylinders defined in couv_sim_world/scripts/layout.py. Prints expected vs
measured depth per object hit and overall error statistics.

    ros2 run couv_sim_robots check_depth.py
"""
import math
import os
import sys

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, os.path.join(get_package_prefix("couv_sim_world"), "lib", "couv_sim_world"))
import layout  # noqa: E402  (couv_sim_world single layout definition)


def obstacles(cam_z):
    """2-D footprints of collision parts spanning height cam_z: ('box', x0, x1, y0, y1, name) / ('cyl', cx, cy, r, name)."""
    out = []
    for m in layout.MODELS:
        mx, my, mz, _ = m["pose"]
        for p in m["parts"]:
            if not p["collision"]:
                continue
            px, py, pz = p["pos"]
            if p["shape"] == "box":
                sx, sy, sz = p["size"]
                if mz + pz - sz / 2 <= cam_z <= mz + pz + sz / 2:
                    out.append(("box", mx + px - sx / 2, mx + px + sx / 2, my + py - sy / 2, my + py + sy / 2,
                                f"{m['name']}/{p['name']}"))
            else:
                r, h = p["size"]
                if mz + pz - h / 2 <= cam_z <= mz + pz + h / 2:
                    out.append(("cyl", mx + px, my + py, r, f"{m['name']}/{p['name']}"))
    return out


def ray_hit(o, d, obs):
    best, name = math.inf, None
    for ob in obs:
        if ob[0] == "box":
            _, x0, x1, y0, y1, n = ob
            tmin, tmax = -math.inf, math.inf
            for oo, dd, lo, hi in ((o[0], d[0], x0, x1), (o[1], d[1], y0, y1)):
                if abs(dd) < 1e-12:
                    if not lo <= oo <= hi:
                        tmin, tmax = 1.0, 0.0
                    continue
                a, b = (lo - oo) / dd, (hi - oo) / dd
                tmin, tmax = max(tmin, min(a, b)), min(tmax, max(a, b))
            if tmin <= tmax and 0 < tmin < best:
                best, name = tmin, n
        else:
            _, cx, cy, r, n = ob
            fx, fy = o[0] - cx, o[1] - cy
            b = fx * d[0] + fy * d[1]
            disc = b * b - (fx * fx + fy * fy - r * r)
            if disc >= 0 and 0 < -b - math.sqrt(disc) < best:
                best, name = -b - math.sqrt(disc), n
    return best, name


class Check(Node):
    def __init__(self):
        super().__init__("check_depth")
        self.depth = self.info = self.odom = None
        self.create_subscription(Image, "/ugv/camera/depth/image_rect_raw", lambda m: setattr(self, "depth", m),
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/ugv/camera/color/camera_info", lambda m: setattr(self, "info", m),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "/ugv/ground_truth/odom", lambda m: setattr(self, "odom", m), 10)


def main():
    rclpy.init()
    node = Check()
    while rclpy.ok() and None in (node.depth, node.info, node.odom):
        rclpy.spin_once(node, timeout_sec=0.1)
    cfg = yaml.safe_load(open(os.path.join(get_package_share_directory("couv_sim_robots"), "config", "frames.yaml")))
    cam, wr = cfg["ugv"]["camera"], cfg["ugv"]["wheel_radius"]
    img = np.frombuffer(node.depth.data, np.float32).reshape(node.depth.height, node.depth.width)
    fx, cx, cy = node.info.k[0], node.info.k[2], node.info.k[5]
    p, q = node.odom.pose.pose.position, node.odom.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    cam_xy = (p.x + cam["x"] * c - cam["y"] * s, p.y + cam["x"] * s + cam["y"] * c)
    cam_z = p.z + cam["height"] - wr
    obs = obstacles(cam_z)
    row = int(round(cy))
    print(f"GT ugv/base_link ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) yaw {math.degrees(yaw):.2f} deg -> camera "
          f"({cam_xy[0]:.3f}, {cam_xy[1]:.3f}, {cam_z:.3f}); depth {node.depth.encoding} "
          f"{node.depth.width}x{node.depth.height}, fx {fx:.1f}, centre row v={row}")
    errs, per_obj = [], {}
    for u in range(node.depth.width):
        xr = (u - cx) / fx                       # optical x (right) per metre of depth
        n = math.hypot(1.0, xr)
        dw = ((c + xr * s) / n, (s - xr * c) / n)  # body (forward 1, left -xr) rotated by yaw
        t, name = ray_hit(cam_xy, dw, obs)
        expected = t / n                          # depth = distance along the optical axis
        meas = float(np.nanmedian(img[row - 1:row + 2, u]))
        if name is None or not 0.2 < expected < 10.0 or not np.isfinite(meas):
            continue
        errs.append(meas - expected)
        per_obj.setdefault(name, []).append((u, expected, meas))
    print(f"{'object on the centre row':30s} {'cols':>5s} {'u':>9s} {'expected[m]':>12s} {'measured[m]':>12s} {'error[m]':>9s}")
    for name, v in sorted(per_obj.items(), key=lambda kv: np.median([x[1] for x in kv[1]])):
        e, m = np.array([x[1] for x in v]), np.array([x[2] for x in v])
        print(f"{name:30s} {len(v):5d} {v[0][0]:4d}-{v[-1][0]:<4d} {np.median(e):12.3f} {np.median(m):12.3f} {np.median(m - e):+9.3f}")
    errs = np.array(errs)
    if len(errs):
        inl = np.abs(errs) < 0.2
        print(f"all hit columns: n={len(errs)}, median error {np.median(errs):+.4f} m, std of inliers {errs[inl].std():.4f} m, "
              f"{100 * inl.mean():.1f}% within 0.2 m (the rest are object edges)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
