#!/usr/bin/env python3
"""Register the drone map to the UGV map and replay every step in RViz.

Three panels (fixed frame `merge_view`):
  +y : UGV map (RTAB-Map, own colours)        frame panel_ugv
   0 : merged view, UGV orange / drone blue   frames ugv/map -> drone/map (animated TF)
  -y : drone map (RTAB-Map, own colours)      frame panel_drone

Steps: identity -> T0 -> T0 + rangefinder climb -> ICP iterations -> adopted result.
ICP is accepted only if it stays within the T0 uncertainty (merge.yaml `acceptance`);
the ground truth from the bag is used only to print/display the error of each step.

    ros2 launch couv_map_merge merge_viz.launch.py map_dir:=<dir> bag:=<bag>
"""
import math
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from couv_map_merge import registration as R

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def cloud_msg(frame, xyz, rgb, stamp):
    n = len(xyz)
    arr = np.zeros(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    rgb = rgb.astype(np.uint32)
    arr["rgb"] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
    m = PointCloud2()
    m.header.frame_id, m.header.stamp = frame, stamp
    m.height, m.width = 1, n
    m.fields = [PointField(name=k, offset=4 * i, datatype=PointField.FLOAT32 if k != "rgb" else PointField.UINT32,
                           count=1) for i, k in enumerate(("x", "y", "z", "rgb"))]
    m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 16, 16 * n, True
    m.data = arr.tobytes()
    return m


def tf_msg(parent, child, T, stamp):
    t = TransformStamped()
    t.header.frame_id, t.header.stamp, t.child_frame_id = parent, stamp, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in T[:3, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = (float(v) for v in q)
    return t


class MergeViz(Node):
    def __init__(self):
        super().__init__("merge_viz")
        share = get_package_share_directory("couv_map_merge")
        self.map_dir = os.path.expanduser(self.declare_parameter("map_dir", "").value)
        self.bag = os.path.expanduser(self.declare_parameter("bag", "").value)
        cfg_path = self.declare_parameter("merge_yaml", os.path.join(share, "config", "merge.yaml")).value
        self.cfg = yaml.safe_load(open(cfg_path))
        log = self.get_logger().info

        pr = R.build_problem(self.map_dir, self.bag, self.cfg, log=log)
        steps, stats = R.register(pr, self.cfg["icp"], log=log)
        T_init, T_icp, T_true = pr["T_init"], steps[-1][1], pr["T_true"]
        corr_t, corr_r = R.error(T_icp, T_init)
        acc = self.cfg["acceptance"]
        accepted = corr_t <= acc["max_correction_t"] and corr_r <= acc["max_correction_rot_deg"]
        T_final = T_icp if accepted else T_init
        verdict = (f"ICP accepted (correction {corr_t * 100:.1f} cm, {corr_r:.2f} deg)" if accepted else
                   f"ICP rejected: correction {corr_t * 100:.1f} cm / {corr_r:.2f} deg exceeds T0 uncertainty "
                   f"({acc['max_correction_t'] * 100:.0f} cm / {acc['max_correction_rot_deg']:.0f} deg) -> keep T0 + climb")
        steps.append(("RESULT: " + verdict, T_final))
        self.steps = [(name, T, *R.error(T, T_true)) for name, T in steps]
        for name, T, et, er in self.steps:
            if not name.startswith("ICP iteration") or name == steps[-2][0]:
                log(f"{name:70s} error vs GT {et * 100:6.1f} cm {er:5.2f} deg")
        self.save_result(pr, T_icp, T_final, accepted, stats)

        # ---- publishers
        now = self.get_clock().now().to_msg()
        v = self.cfg["viz"]
        off = v["panel_offset"]
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform([
            tf_msg("merge_view", "ugv/map", np.eye(4), now),
            tf_msg("merge_view", "panel_ugv", R.xyzyaw(0, off, 0, 0), now),
            tf_msg("merge_view", "panel_drone", R.xyzyaw(0, -off, 0, 0), now)])
        self.tf = TransformBroadcaster(self)
        self.pubs = {}
        for key, frame, ns, colour in (("ugv_rgb", "panel_ugv", "ugv", None), ("drone_rgb", "panel_drone", "drone", None),
                                       ("ugv_merged", "ugv/map", "ugv", v["ugv_color"]),
                                       ("drone_merged", "drone/map", "drone", v["drone_color"])):
            xyz, rgb = pr["clouds"][ns]
            # the animated drone cloud is re-sent every step, so it is kept lighter
            xyz, rgb = R.voxel_down(xyz, v["cloud_voxel"] if key != "drone_merged" else v["moving_cloud_voxel"], rgb)
            if colour is not None:  # tint, keep some texture for depth perception
                rgb = (0.35 * rgb + 0.65 * np.array(colour)).astype(np.uint8)
            self.pubs[key] = self.create_publisher(PointCloud2, f"/merge/{key}", LATCHED)
            # stamp 0 = latest TF. RViz transforms a cloud only when it arrives, so the moving one
            # (drone/map) is re-published after every TF update in tick()
            msg = cloud_msg(frame, xyz, rgb, Time().to_msg())
            self.pubs[key].publish(msg)
            if key == "drone_merged":
                self.moving_cloud = msg
        self.markers = self.create_publisher(MarkerArray, "/merge/markers", LATCHED)
        self.static_markers = self.make_static_markers(pr, Time().to_msg())
        self.i, self.hold = 0, 0.0
        self.create_timer(v["step_period"], self.tick)
        log(f"replaying {len(self.steps)} steps in RViz (fixed frame merge_view)")

    # ------------------------------------------------------------------ helpers
    def save_result(self, pr, T_icp, T_final, accepted, stats):
        def pose(T):
            r = Rotation.from_matrix(T[:3, :3])
            return {"xyz": [round(float(v), 4) for v in T[:3, 3]],
                    "rpy_deg": [round(float(v), 3) for v in np.degrees(r.as_euler("xyz"))]}
        out = {
            "map_dir": self.map_dir, "bag": self.bag,
            "T_ugvmap_dronemap": {"initial_T0_plus_climb": pose(pr["T_init"]), "icp": pose(T_icp),
                                  "adopted": pose(T_final), "icp_accepted": bool(accepted),
                                  "ground_truth": pose(pr["T_true"])},
            "error_vs_ground_truth": {name: {"t_cm": round(et * 100, 2), "rot_deg": round(er, 3)}
                                      for name, _, et, er in self.steps if not name.startswith("ICP iteration")},
            "icp_final_rmse_cm": round(stats[-1]["rmse"] * 100, 2), "icp_iterations": len(stats),
        }
        path = os.path.join(self.map_dir, "merge_result.yaml")
        yaml.safe_dump(out, open(path, "w"), sort_keys=False, allow_unicode=True)
        self.get_logger().info(f"result written to {path}")

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

    def path(self, mid, frame, poses, colour, stamp):
        m = Marker()
        m.header.frame_id, m.header.stamp, m.ns, m.id = frame, stamp, "path", mid
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.06
        m.color = ColorRGBA(r=colour[0] / 255, g=colour[1] / 255, b=colour[2] / 255, a=1.0)
        m.points = [Point(x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3])) for T in poses]
        return m

    def make_static_markers(self, pr, stamp):
        v = self.cfg["viz"]
        uc, dc = v["ugv_color"], v["drone_color"]
        mk = [
            self.text(0, "panel_ugv", (7, 0, 9), "UGV map", 1.4, stamp),
            self.text(1, "panel_drone", (7, 0, 11), "Drone map", 1.4, stamp),
            self.path(10, "panel_ugv", pr["poses"]["ugv"][1], uc, stamp),
            self.path(11, "panel_drone", pr["poses"]["drone"][1], dc, stamp),
            self.path(12, "ugv/map", pr["poses"]["ugv"][1], uc, stamp),
            self.path(13, "drone/map", pr["poses"]["drone"][1], dc, stamp),
        ]
        return mk

    def tick(self):
        now = self.get_clock().now().to_msg()
        v = self.cfg["viz"]
        name, T, et, er = self.steps[self.i]
        self.tf.sendTransform(tf_msg("ugv/map", "drone/map", T, now))
        self.pubs["drone_merged"].publish(self.moving_cloud)
        last = self.i == len(self.steps) - 1
        colour = (0.4, 1.0, 0.4) if last else (1.0, 1.0, 1.0)
        info = (f"Merged map  -  step {self.i + 1}/{len(self.steps)}\n{name}\n"
                f"error vs ground truth: {et * 100:.1f} cm, {er:.2f} deg")
        self.markers.publish(MarkerArray(markers=self.static_markers + [
            self.text(2, "merge_view", (7, 0, 13), info, 0.9, Time().to_msg(), colour)]))
        if last:
            self.hold += v["step_period"]
            if self.hold >= v["hold_final"] and v["loop"]:
                self.i, self.hold = 0, 0.0
        else:
            self.i += 1


def main():
    rclpy.init()
    node = MergeViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
