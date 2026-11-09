#!/usr/bin/env python3
"""Show the registration methods of a serpentine run (compare_aruco.py) in RViz.

mode:=compare (default): methods 1 T0 only | 2 T0 + ICP (forced) | 3 ArUco start+end + ICP (forced) |
4 T0 + ArUco end + RGB-D feature anchors side by side along x (`panel_spacing`), each with the UGV map (orange), the
drone map placed by the method (blue), the SLAM paths (orange / blue), the ground-truth paths in the UGV map (drone
white, UGV grey) and the error numbers. RED = mismatch where the two maps overlap: a point whose nearest point of
the other map is within 0.5 m but whose gap to that map's surface exceeds `mismatch` (default 0.10 m).
Yellow spheres in panel 4 = drone nodes with an accepted RGB-D feature match.

mode:=cycle: three panels (fixed frame `merge_view` = ugv/map):
  +y : UGV map (own colours)            frame panel_ugv
   0 : merged view, UGV orange / drone blue, the drone map placed by each method in turn
  -y : drone map (own colours)          frame panel_drone

Steps: A T0 only -> B ArUco start -> C ArUco start+end -> D C + ICP -> ground-truth placement (reference).
C and D move every drone node by its own transform, so the drone cloud is assembled per step from the
exported node images instead of moving one TF. Lines in the merged view:
  orange / blue      UGV and drone SLAM paths (drone path as placed by the method)
  white / grey thin  drone / UGV ground truth expressed in the UGV map (anchored at its first node);
                     the gap between the orange and grey lines is the UGV map's own drift.

    ros2 launch couv_map_merge aruco_viz.launch.py run:=~/ros2_ws/maps/serpentine_aruco
"""
import os

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from couv_map_merge import registration as R
from couv_map_merge.compare_aruco import (NAMES, assemble, local_node_errors, load_camera_poses, node_errors,
                                          overlap_error, solve)
from couv_map_merge.merge_viz import cloud_msg, tf_msg

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
UGV_COLOUR, DRONE_COLOUR, RED = (235, 104, 52), (42, 120, 214), (235, 30, 30)


def assemble_rgb(clouds, cam_poses, node_T):
    pts, cols = [], []
    for nid, (p, c) in clouds.items():
        if nid in cam_poses and nid in node_T:
            pts.append(R.apply(node_T[nid] @ cam_poses[nid], p))
            cols.append(c)
    pts, cols = np.vstack(pts), np.vstack(cols)
    ok = np.isfinite(pts).all(1)
    return pts[ok], cols[ok]


def tint(rgb, colour):
    return (0.35 * rgb + 0.65 * np.array(colour)).astype(np.uint8)


class ArucoViz(Node):
    def __init__(self):
        super().__init__("aruco_viz")
        self.run = os.path.expanduser(self.declare_parameter("run", "").value)
        pitch = self.declare_parameter("pitch", 45.0).value
        off = self.declare_parameter("panel_offset", 26.0).value
        self.period = self.declare_parameter("step_period", 4.0).value
        voxel = self.declare_parameter("cloud_voxel", 0.05).value
        mode = self.declare_parameter("mode", "compare").value
        spacing = self.declare_parameter("panel_spacing", 25.0).value
        self.mismatch = self.declare_parameter("mismatch", 0.10).value
        log = self.get_logger().info

        log(f"solving registration methods for {self.run} (ICP takes a few seconds)")
        S = solve(self.run, pitch)
        poses, gt, W_u = S["poses"], S["gt"], S["W_u"]
        st_d, P_d, ids_d = poses["drone"]
        cam_d = load_camera_poses(self.run, "drone")
        self.markers = self.create_publisher(MarkerArray, "/merge/markers", LATCHED)
        if mode == "compare":
            self.setup_compare(S, cam_d, spacing, max(voxel, 0.07))
            return

        # ---- steps: per-node transforms of every method + the ground-truth placement
        per_node = dict(S["per_node"])
        per_node["GT"] = [R.inv(W_u) @ gt["drone"].at(t) @ R.inv(P) for t, P in zip(st_d, P_d)]
        icp = S["icp"]
        labels = dict(NAMES)
        labels["D"] += (f" (ICP {icp['ct'] * 100:.1f} cm / {icp['cr']:.2f} deg: "
                        + ("accepted)" if icp["accepted"] else "rejected -> same as C)"))
        labels["GT"] = "ground-truth placement (reference, not a method)"
        self.steps = []
        for k in ("A", "B", "C", "D", "GT"):
            id2T = {int(i): T for i, T in zip(ids_d, per_node[k])}
            xyz, rgb = assemble_rgb(S["clouds"]["drone"], cam_d, id2T)
            xyz, rgb = R.voxel_down(xyz, voxel, rgb)
            pe, _ = node_errors(per_node[k], poses["drone"], gt["drone"], W_u)
            info = (f"{k}: {labels[k]}" if k != "GT" else labels[k]) + "\n" + (
                f"drone node error vs ground truth: median {np.median(pe) * 100:.1f} cm, "
                f"max {pe.max() * 100:.1f} cm, end {pe[-1] * 100:.1f} cm" if k != "GT" else
                "drone nodes at their true poses relative to the UGV map origin")
            path = [T @ P for T, P in zip(per_node[k], P_d)]
            self.steps.append((k, cloud_msg("ugv/map", xyz, tint(rgb, DRONE_COLOUR), Time().to_msg()), path, info))
            log(f"step {k}: {len(xyz)} drone points | {info.splitlines()[-1]}")

        # ---- static frames and clouds
        now = self.get_clock().now().to_msg()
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform([
            tf_msg("merge_view", "ugv/map", np.eye(4), now),
            tf_msg("merge_view", "panel_ugv", R.xyzyaw(0, off, 0, 0), now),
            tf_msg("merge_view", "panel_drone", R.xyzyaw(0, -off, 0, 0), now)])
        self.pubs = {k: self.create_publisher(PointCloud2, f"/merge/{k}", LATCHED)
                     for k in ("ugv_rgb", "drone_rgb", "ugv_merged", "drone_merged")}
        ply = {}
        for ns in ("ugv", "drone"):
            xyz, rgb = R.load_ply(os.path.join(self.run, f"{ns}_cloud.ply"))
            ply[ns] = R.voxel_down(xyz, voxel, rgb)
        self.pubs["ugv_rgb"].publish(cloud_msg("panel_ugv", *ply["ugv"], Time().to_msg()))
        self.pubs["drone_rgb"].publish(cloud_msg("panel_drone", *ply["drone"], Time().to_msg()))
        self.pubs["ugv_merged"].publish(cloud_msg("ugv/map", ply["ugv"][0], tint(ply["ugv"][1], UGV_COLOUR),
                                                  Time().to_msg()))

        # ---- static markers: labels, own-map paths, ground-truth paths in the UGV map, ArUco captures
        stamp = Time().to_msg()
        st_u, P_u, _ = poses["ugv"]
        t_gt = np.arange(st_u[0], st_d[-1], 0.5)
        centre = P_u[:, :3, 3].mean(0)
        self.static_markers = [
            self.text(0, "panel_ugv", (centre[0], centre[1], 8), "UGV map", 1.4, stamp),
            self.text(1, "panel_drone", (centre[0], centre[1], 10), "Drone map", 1.4, stamp),
            self.line(10, "panel_ugv", [P[:3, 3] for P in P_u], UGV_COLOUR, 0.06, stamp),
            self.line(11, "panel_drone", [P[:3, 3] for P in P_d], DRONE_COLOUR, 0.06, stamp),
            self.line(12, "ugv/map", [P[:3, 3] for P in P_u], UGV_COLOUR, 0.08, stamp),
            self.line(13, "ugv/map", [(R.inv(W_u) @ gt["drone"].at(t))[:3, 3] for t in t_gt], (255, 255, 255), 0.03, stamp),
            self.line(14, "ugv/map", [(R.inv(W_u) @ gt["ugv"].at(t))[:3, 3] for t in t_gt], (150, 150, 150), 0.03, stamp),
        ]
        for i, (phase, a) in enumerate(S["aruco"].items()):
            p = (R.inv(W_u) @ gt["ugv"].at(a["t"]))[:3, 3]
            self.static_markers.append(self.text(20 + i, "ugv/map", p + (0, 0, 1.2), phase, 0.5, stamp, (1.0, 0.9, 0.3)))
        self.info_pos = (centre[0], centre[1], 12)
        self.i = 0
        self.tick()
        self.create_timer(self.period, self.tick)
        log(f"replaying {len(self.steps)} steps every {self.period:.0f} s in RViz (fixed frame merge_view)")

    def setup_compare(self, S, cam_d, spacing, voxel):
        """Methods 1-4 side by side in one cloud (frame merge_view = ugv/map, panel k shifted by k * spacing in x)."""
        poses, gt, W_u = S["poses"], S["gt"], S["W_u"]
        st_d, P_d, ids_d = poses["drone"]
        st_u, P_u, _ = poses["ugv"]
        ia = S["icp_all"]
        methods = [
            ("1  T0 only", S["per_node"]["A"]),
            (f"2  T0 + ICP (forced; rule: {'accept' if ia['A']['accepted'] else 'reject'}, "
             f"moved {ia['A']['ct'] * 100:.1f} cm / {ia['A']['cr']:.1f} deg)", S["per_node"]["A+ICP"]),
            (f"3  ArUco start+end + ICP (forced; rule: {'accept' if ia['C']['accepted'] else 'reject'}, "
             f"moved {ia['C']['ct'] * 100:.1f} cm / {ia['C']['cr']:.1f} deg)", S["per_node"]["C+ICP"]),
            (f"4  T0 + ArUco end + RGB-D features ({S['features']['stats']['accepted']} matches, "
             f"{S['features']['anchors_used']} anchors, no ICP)", S["per_node"]["F"]),
        ]
        # dense references for the mismatch test (4 cm voxels), display clouds at `voxel`
        ugv_ref = R.voxel_down(assemble(S["clouds"]["ugv"], S["cam"]["ugv"]), 0.04)[0]
        ugv_ref_n = R.normals(ugv_ref, 12)
        ugv_xyz, ugv_rgb = R.load_ply(os.path.join(self.run, "ugv_cloud.ply"))
        ugv_xyz, ugv_rgb = R.voxel_down(ugv_xyz, voxel, ugv_rgb)
        ugv_rgb = tint(ugv_rgb, UGV_COLOUR)
        stamp = Time().to_msg()
        t_gt = np.arange(st_u[0], st_d[-1], 0.5)
        gt_d = np.array([(R.inv(W_u) @ gt["drone"].at(t))[:3, 3] for t in t_gt])
        gt_u = np.array([(R.inv(W_u) @ gt["ugv"].at(t))[:3, 3] for t in t_gt])
        centre = P_u[:, :3, 3].mean(0)
        top = ugv_xyz[:, 1].max()
        pts, cols, markers = [], [], []
        for k, (name, per_node) in enumerate(methods):
            off = np.array([k * spacing, 0.0, 0.0])
            id2T = {int(i): T for i, T in zip(ids_d, per_node)}
            xyz, rgb = assemble_rgb(S["clouds"]["drone"], cam_d, id2T)
            drone_ref = R.voxel_down(xyz, 0.04)[0]
            xyz, rgb = R.voxel_down(xyz, voxel, rgb)
            gap, ov, bad_d = overlap_error(xyz, ugv_ref, thr=self.mismatch, b_normals=ugv_ref_n)
            _, ov_u, bad_u = overlap_error(ugv_xyz, drone_ref, thr=self.mismatch)
            col_d, col_u = tint(rgb, DRONE_COLOUR), ugv_rgb.copy()
            col_d[bad_d], col_u[bad_u] = RED, RED
            pts += [ugv_xyz + off, xyz + off]
            cols += [col_u, col_d]
            pe, re = node_errors(per_node, poses["drone"], gt["drone"], W_u)
            le = local_node_errors(per_node, poses["drone"], poses["ugv"], gt["drone"], gt["ugv"])
            info = (f"{name}\nglobal (vs ground truth): median {np.median(pe) * 100:.1f} cm  max {pe.max() * 100:.1f}  "
                    f"end {pe[-1] * 100:.1f}\nlocal (vs UGV map nearby): median {np.median(le) * 100:.1f} cm  "
                    f"max {le.max() * 100:.1f}  end {le[-1] * 100:.1f}\n"
                    f"red mismatch in overlap: {bad_d.sum() / max(ov.sum(), 1) * 100:.0f} % of drone overlap points, "
                    f"median gap {np.median(gap[ov]) * 100:.1f} cm")
            self.get_logger().info(info.replace("\n", " | "))
            m0 = 100 * k
            markers += [
                self.text(m0, "merge_view", (centre[0] + off[0], top + 3.0, 6.0), info, 0.9, stamp),
                self.line(m0 + 1, "merge_view", P_u[:, :3, 3] + off, UGV_COLOUR, 0.10, stamp),
                self.line(m0 + 2, "merge_view", [(T @ P)[:3, 3] + off for T, P in zip(per_node, P_d)],
                          DRONE_COLOUR, 0.10, stamp),
                self.line(m0 + 3, "merge_view", gt_d + off, (255, 255, 255), 0.04, stamp),
                self.line(m0 + 4, "merge_view", gt_u + off, (150, 150, 150), 0.04, stamp),
            ]
            if k == 3:   # drone nodes with an accepted feature match
                id2P = {int(i): (T @ P)[:3, 3] for i, T, P in zip(ids_d, per_node, P_d)}
                sp = self.line(m0 + 5, "merge_view", [id2P[i] + off for i in S["features"]["anchor_nodes"]
                                                      if i in id2P], (255, 220, 0), 0.35, stamp)
                sp.type = Marker.SPHERE_LIST
                sp.scale.y = sp.scale.z = 0.35
                markers.append(sp)
        markers.append(self.text(900, "merge_view", (centre[0] + 1.5 * spacing, top + 9.0, 6.0),
                                 "orange: UGV map / path    blue: drone map / path as placed by the method    "
                                 "white / grey: drone / UGV ground truth    RED: overlap mismatch > "
                                 f"{self.mismatch * 100:.0f} cm    yellow: feature-matched drone nodes",
                                 0.7, stamp, (0.8, 0.8, 0.8)))
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform([tf_msg("merge_view", "ugv/map", np.eye(4), self.get_clock().now().to_msg())])
        self.cloud_pub = self.create_publisher(PointCloud2, "/merge/compare_cloud", LATCHED)
        self.cloud_pub.publish(cloud_msg("merge_view", np.vstack(pts), np.vstack(cols), stamp))
        self.markers.publish(MarkerArray(markers=markers))
        self.get_logger().info(f"methods 1-4 published side by side ({sum(len(p) for p in pts)} points, "
                               f"panel spacing {spacing:.0f} m, fixed frame merge_view)")

    # ------------------------------------------------------------------ markers
    def text(self, mid, frame, xyz, txt, size, stamp, colour=(1.0, 1.0, 1.0)):
        m = Marker()
        m.header.frame_id, m.header.stamp, m.ns, m.id = frame, stamp, "text", mid
        m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(c) for c in xyz)
        m.pose.orientation.w = 1.0
        m.scale.z = size
        m.color = ColorRGBA(r=colour[0], g=colour[1], b=colour[2], a=1.0)
        m.text = txt
        return m

    def line(self, mid, frame, pts, colour, width, stamp):
        m = Marker()
        m.header.frame_id, m.header.stamp, m.ns, m.id = frame, stamp, "path", mid
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = width
        m.color = ColorRGBA(r=colour[0] / 255, g=colour[1] / 255, b=colour[2] / 255, a=1.0)
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]
        return m

    def tick(self):
        k, cloud, path, info = self.steps[self.i]
        stamp = Time().to_msg()
        self.pubs["drone_merged"].publish(cloud)
        colour = (0.6, 0.6, 1.0) if k == "GT" else (1.0, 1.0, 1.0)
        self.markers.publish(MarkerArray(markers=self.static_markers + [
            self.line(15, "ugv/map", [T[:3, 3] for T in path], DRONE_COLOUR, 0.08, stamp),
            self.text(2, "merge_view", self.info_pos, f"step {self.i + 1}/{len(self.steps)}  " + info, 0.8, stamp,
                      colour)]))
        self.i = (self.i + 1) % len(self.steps)


def main():
    rclpy.init()
    node = ArucoViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
