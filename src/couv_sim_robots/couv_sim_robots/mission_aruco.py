#!/usr/bin/env python3
"""Serpentine mission with ArUco captures and landing (simulation only, ground-truth control).

Drone states (published on /mission/state, transient local):
  waiting -> takeoff -> aruco_start (hover behind/above the UGV, pad marker in view)
  -> climb -> cruise (serpentine at cruise altitude; the UGV drives its own serpentine)
  -> wait_ugv (UGV not yet at its goal) -> approach -> aruco_end -> land -> complete
aruco_detector.py records marker poses while the state is aruco_start / aruco_end.
Parameter: mission_yaml (default: share/couv_sim_robots/config/mission_serpentine.yaml).
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from couv_sim_robots.mission_runner import clamp, slew, wrap, yaw_of


class MissionAruco(Node):
    RATE = 20.0

    def __init__(self):
        super().__init__("mission_aruco")
        share = get_package_share_directory("couv_sim_robots")
        default = os.path.join(share, "config", "mission_serpentine.yaml")
        path = self.declare_parameter("mission_yaml", default).value or default
        self.cfg = yaml.safe_load(open(path))
        frames = yaml.safe_load(open(os.path.join(share, "config", "frames.yaml")))
        self.pad_above_base = frames["ugv"]["landing_pad"]["top_height"] - frames["ugv"]["wheel_radius"]
        self.seat_above_base = frames["landing_pad_offset"]["z"] - 0.005  # drone base_link above ugv base_link when landed
        self.odom = {"ugv": None, "drone": None}
        for r in self.odom:
            self.create_subscription(Odometry, f"/{r}/ground_truth/odom", lambda m, r=r: self.odom.__setitem__(r, m), 10)
        self.pub_ugv = self.create_publisher(Twist, "/ugv/cmd_vel", 10)
        self.pub_drone = self.create_publisher(Twist, "/drone/cmd_vel", 10)
        self.pub_enable = self.create_publisher(Bool, "/drone/enable", 10)
        self.pub_state = self.create_publisher(String, "/mission/state",
                                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.state = self.t_state = self.t_alive = self.t_start = None
        self.ugv_wp = self.drone_wp = 0
        self.ugv_v, self.drone_v, self.integ = 0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        self.ugv_running = self.ugv_arrived = False
        self.target = None
        self.set_state("waiting")
        self.create_timer(1.0 / self.RATE, self.step)
        self.get_logger().info(f"mission: {path}")

    # ------------------------------------------------------------------ helpers
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, s):
        self.state, self.t_state = s, self.now()
        self.pub_state.publish(String(data=s))
        self.get_logger().info(f"[t={self.t_state:7.2f}] state -> {s}")

    def pose(self, r):
        p = self.odom[r].pose.pose
        return p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)

    def relative_to_ugv(self, fwd, above_base):
        """World point at `fwd` m along the UGV heading and `above_base` m above ugv/base_link."""
        x, y, z, yaw = self.pose("ugv")
        return (x + fwd * math.cos(yaw), y + fwd * math.sin(yaw), z + above_base)

    def drone_cmd(self, target, speed):
        c = self.cfg["drone"]
        x, y, z, yaw = self.pose("drone")
        err = (target[0] - x, target[1] - y, target[2] - z)
        # integral term near the target removes the velocity controller's steady-state offset
        near = math.sqrt(sum(e * e for e in err)) < c["integral_zone"]
        self.integ = [clamp(i + e / self.RATE, c["integral_limit"]) if near else 0.0 for i, e in zip(self.integ, err)]
        v = [c["kp_xy"] * err[0], c["kp_xy"] * err[1], c["kp_z"] * err[2]]
        v = [a + c["ki"] * i for a, i in zip(v, self.integ)]
        n = math.sqrt(sum(a * a for a in v))
        if n > speed:
            v = [a * speed / n for a in v]
        self.drone_v = [slew(p, t, c["max_accel"] / self.RATE) for p, t in zip(self.drone_v, v)]
        cy, sy = math.cos(yaw), math.sin(yaw)
        t = Twist()
        t.linear.x = cy * self.drone_v[0] + sy * self.drone_v[1]
        t.linear.y = -sy * self.drone_v[0] + cy * self.drone_v[1]
        t.linear.z = self.drone_v[2]
        t.angular.z = clamp(c["kp_yaw"] * wrap(c["yaw"] - yaw), c["max_yaw_rate"])
        self.pub_drone.publish(t)
        return math.sqrt(sum(a * a for a in err))

    def ugv_step(self):
        c = self.cfg["ugv"]
        if self.ugv_wp >= len(c["waypoints"]):
            self.pub_ugv.publish(Twist())
            if not self.ugv_arrived:
                self.ugv_arrived = True
                self.get_logger().info(f"[t={self.now():7.2f}] ugv arrived at the goal")
            return
        target = c["waypoints"][self.ugv_wp]
        x, y, _, yaw = self.pose("ugv")
        dist = math.hypot(target[0] - x, target[1] - y)
        if dist < c["tolerance"]:
            self.get_logger().info(f"[t={self.now():7.2f}] ugv reached waypoint {self.ugv_wp}: {target}")
            self.ugv_wp += 1
            return
        herr = wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
        v = 0.0 if abs(herr) > c["slow_down_heading"] else min(c["max_speed"], c["kp_lin"] * dist) * math.cos(herr)
        self.ugv_v = slew(self.ugv_v, v, c["max_accel"] / self.RATE)
        t = Twist()
        t.linear.x = self.ugv_v
        t.angular.z = clamp(c["kp_yaw"] * herr, c["max_yaw_rate"])
        self.pub_ugv.publish(t)

    # ------------------------------------------------------------------ state machine
    def step(self):
        if None in self.odom.values():
            return
        t = self.now()
        dt = t - self.t_state
        c, cd, ah = self.cfg, self.cfg["drone"], self.cfg["aruco_hover"]
        if self.ugv_running:
            self.ugv_step()
        else:
            self.pub_ugv.publish(Twist())
        if self.t_start and t - self.t_start > c["timeout"] and self.state not in ("complete", "aborted"):
            self.ugv_running = False
            self.target = self.pose("drone")[:3]
            self.set_state("aborted")

        s = self.state
        if s == "waiting":
            self.t_alive = self.t_alive or t
            if t - self.t_alive >= c["start_delay"]:
                self.t_start = t
                self.pub_enable.publish(Bool(data=True))
                self.target = self.relative_to_ugv(0.0, self.pad_above_base + ah["above_pad"])
                self.set_state("takeoff")
        elif s == "takeoff":  # straight up first, then back to the photo point
            self.pub_enable.publish(Bool(data=True))
            if self.drone_cmd(self.target, cd["takeoff_speed"]) < cd["tolerance"]:
                self.target = self.relative_to_ugv(-ah["behind"], self.pad_above_base + ah["above_pad"])
                self.set_state("move_to_aruco_start")
        elif s == "move_to_aruco_start":
            if self.drone_cmd(self.target, cd["takeoff_speed"]) < cd["tolerance"]:
                self.set_state("aruco_start")
        elif s == "aruco_start":
            self.drone_cmd(self.target, cd["takeoff_speed"])
            if dt >= ah["duration"]:
                x, y, _, _ = self.pose("drone")
                self.target = (x, y, cd["cruise_altitude"])
                self.set_state("climb")
        elif s == "climb":
            if self.drone_cmd(self.target, cd["max_speed"]) < cd["tolerance"]:
                self.ugv_running = True
                self.set_state("cruise")
        elif s == "cruise":
            wps = cd["waypoints"]
            target = (*wps[self.drone_wp], cd["cruise_altitude"])
            if self.drone_cmd(target, cd["max_speed"]) < cd["tolerance"]:
                self.get_logger().info(f"[t={t:7.2f}] drone reached waypoint {self.drone_wp}: {wps[self.drone_wp]}")
                self.drone_wp += 1
                if self.drone_wp == len(wps):
                    self.target = target
                    self.set_state("wait_ugv")
        elif s == "wait_ugv":
            self.drone_cmd(self.target, cd["max_speed"])
            if self.ugv_arrived:
                hx, hy, _ = self.relative_to_ugv(-ah["behind"], 0.0)
                self.target = (hx, hy, self.pose("drone")[2])  # horizontally first, at cruise altitude
                self.set_state("approach")
        elif s == "approach":
            if self.drone_cmd(self.target, cd["max_speed"]) < cd["tolerance"]:
                goal = self.relative_to_ugv(-ah["behind"], self.pad_above_base + ah["above_pad"])
                if abs(self.target[2] - goal[2]) < 1e-6:
                    self.set_state("aruco_end")
                self.target = goal
        elif s == "aruco_end":
            self.drone_cmd(self.target, cd["takeoff_speed"])
            if dt >= ah["duration"]:
                self.target = self.relative_to_ugv(0.0, self.pad_above_base + ah["above_pad"])
                self.set_state("move_over_pad")
        elif s == "move_over_pad":
            if self.drone_cmd(self.target, cd["takeoff_speed"]) < cd["landing_tolerance"]:
                self.set_state("land")
        elif s == "land":
            seat = self.relative_to_ugv(0.0, self.seat_above_base)
            self.drone_cmd(seat, cd["landing_speed"])
            if self.pose("drone")[2] - seat[2] < cd["touchdown_margin"]:
                self.pub_drone.publish(Twist())
                self.pub_enable.publish(Bool(data=False))
                self.set_state("touchdown")
        elif s == "touchdown":
            self.pub_enable.publish(Bool(data=False))
            if dt >= c["done_hold"]:
                ux, uy, uz, _ = self.pose("ugv")
                dx, dy, dz, _ = self.pose("drone")
                self.set_state("complete")
                self.get_logger().info(f"MISSION COMPLETE in {t - self.t_start:.1f} s sim: ugv ({ux:.2f}, {uy:.2f}), "
                                       f"drone on pad at ({dx - ux:+.3f}, {dy - uy:+.3f}, {dz - uz:+.3f}) rel. ugv")
        elif s in ("complete", "aborted"):
            if s == "aborted":
                self.drone_cmd(self.target, cd["max_speed"])


def main():
    rclpy.init()
    node = MissionAruco()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
