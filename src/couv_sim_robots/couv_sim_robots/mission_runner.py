#!/usr/bin/env python3
"""Repeatable test mission (simulation only, uses ground truth, never a SLAM input).

1. drone: vertical take-off from the UGV pad to `takeoff_altitude`, hover
2. ugv:   follows its waypoints (max 0.4 m/s) to the goal
3. drone: starts with the UGV, flies its waypoints over the roof (max 0.8 m/s)
4. both stop (drone hovers in place); /mission/state = "complete"

Speeds, accelerations and yaw rates are limited to keep camera motion blur low.
Parameters: mission_yaml (default: share/couv_sim_robots/config/mission.yaml).
State is published on /mission/state (std_msgs/String, transient local).
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


def clamp(v, lim):
    return max(-lim, min(lim, v))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def slew(prev, target, max_step):
    return prev + clamp(target - prev, max_step)


class MissionRunner(Node):
    RATE = 20.0

    def __init__(self):
        super().__init__("mission_runner")
        default = os.path.join(get_package_share_directory("couv_sim_robots"), "config", "mission.yaml")
        path = self.declare_parameter("mission_yaml", default).value or default
        self.cfg = yaml.safe_load(open(path))
        self.odom = {"ugv": None, "drone": None}
        for r in self.odom:
            self.create_subscription(Odometry, f"/{r}/ground_truth/odom", lambda m, r=r: self.odom.__setitem__(r, m), 10)
        self.pub_ugv = self.create_publisher(Twist, "/ugv/cmd_vel", 10)
        self.pub_drone = self.create_publisher(Twist, "/drone/cmd_vel", 10)
        self.pub_enable = self.create_publisher(Bool, "/drone/enable", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.create_publisher(String, "/mission/state", latched)
        self.state, self.t_state = None, None
        self.ugv_wp = self.drone_wp = 0
        self.ugv_v = 0.0
        self.drone_v = [0.0, 0.0, 0.0]
        self.hold = None
        self.t_start = self.t_alive = None
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

    def drone_cmd(self, target, speed_limit=None):
        """P control to `target` (x, y, z) in map; returns distance. Velocity sent in the body frame."""
        c = self.cfg["drone"]
        x, y, z, yaw = self.pose("drone")
        err = (target[0] - x, target[1] - y, target[2] - z)
        v = [c["kp_xy"] * err[0], c["kp_xy"] * err[1], c["kp_z"] * err[2]]
        vmax = speed_limit or c["max_speed"]
        n = math.sqrt(sum(a * a for a in v))
        if n > vmax:
            v = [a * vmax / n for a in v]
        step = c["max_accel"] / self.RATE
        self.drone_v = [slew(p, t, step) for p, t in zip(self.drone_v, v)]
        cy, sy = math.cos(yaw), math.sin(yaw)
        t = Twist()
        t.linear.x = cy * self.drone_v[0] + sy * self.drone_v[1]
        t.linear.y = -sy * self.drone_v[0] + cy * self.drone_v[1]
        t.linear.z = self.drone_v[2]
        t.angular.z = clamp(c["kp_yaw"] * wrap(c["yaw"] - yaw), c["max_yaw_rate"])
        self.pub_drone.publish(t)
        return math.sqrt(sum(a * a for a in err))

    def ugv_cmd(self, target):
        c = self.cfg["ugv"]
        x, y, _, yaw = self.pose("ugv")
        dx, dy = target[0] - x, target[1] - y
        dist = math.hypot(dx, dy)
        herr = wrap(math.atan2(dy, dx) - yaw)
        v = 0.0 if abs(herr) > c["slow_down_heading"] else min(c["max_speed"], c["kp_lin"] * dist) * math.cos(herr)
        self.ugv_v = slew(self.ugv_v, v, c["max_accel"] / self.RATE)
        t = Twist()
        t.linear.x = self.ugv_v
        t.angular.z = clamp(c["kp_yaw"] * herr, c["max_yaw_rate"])
        self.pub_ugv.publish(t)
        return dist

    def stop_ugv(self):
        self.ugv_v = 0.0
        self.pub_ugv.publish(Twist())

    # ------------------------------------------------------------------ state machine
    def step(self):
        if None in self.odom.values():
            return
        t = self.now()
        dt = t - self.t_state
        c, cd = self.cfg, self.cfg["drone"]
        if self.state not in ("waiting", "complete", "aborted") and t - self.t_start > c["timeout"]:
            self.stop_ugv()
            self.hold = self.pose("drone")[:3]
            self.set_state("aborted")
        if self.state == "waiting":
            self.t_alive = self.t_alive or t
            if t - self.t_alive >= c["start_delay"]:
                self.t_start = t
                self.pub_enable.publish(Bool(data=True))
                x, y, _, _ = self.pose("drone")
                self.hold = (x, y, cd["takeoff_altitude"])
                self.set_state("takeoff")
        elif self.state == "takeoff":
            self.pub_enable.publish(Bool(data=True))
            if self.drone_cmd(self.hold, cd["takeoff_speed"]) < cd["tolerance"]:
                self.set_state("hover")
        elif self.state == "hover":
            self.drone_cmd(self.hold)
            if dt >= cd["hover_before_ugv"]:
                self.set_state("run")
        elif self.state == "run":
            ugv_wps, drone_wps = c["ugv"]["waypoints"], cd["waypoints"]
            if self.ugv_wp < len(ugv_wps):
                if self.ugv_cmd(ugv_wps[self.ugv_wp]) < c["ugv"]["tolerance"]:
                    self.get_logger().info(f"[t={t:7.2f}] ugv reached waypoint {self.ugv_wp}: {ugv_wps[self.ugv_wp]}")
                    self.ugv_wp += 1
            else:
                self.stop_ugv()
            if self.drone_wp < len(drone_wps):
                if self.drone_cmd(drone_wps[self.drone_wp]) < cd["tolerance"]:
                    self.get_logger().info(f"[t={t:7.2f}] drone reached waypoint {self.drone_wp}: {drone_wps[self.drone_wp]}")
                    self.drone_wp += 1
                    if self.drone_wp == len(drone_wps):
                        self.hold = tuple(drone_wps[-1])
            else:
                self.drone_cmd(self.hold)
            if self.ugv_wp >= len(ugv_wps) and self.drone_wp >= len(drone_wps):
                self.set_state("stopping")
        elif self.state == "stopping":
            self.stop_ugv()
            self.drone_cmd(self.hold)
            if dt >= c["done_hold"]:
                self.set_state("complete")
                ux, uy, _, _ = self.pose("ugv")
                dx, dy, dz, _ = self.pose("drone")
                self.get_logger().info(f"MISSION COMPLETE in {t - self.t_start:.1f} s sim: ugv ({ux:.2f}, {uy:.2f}), "
                                       f"drone ({dx:.2f}, {dy:.2f}, {dz:.2f})")
        else:  # complete / aborted: keep the UGV stopped and the drone hovering in place
            self.stop_ugv()
            self.drone_cmd(self.hold)


def main():
    rclpy.init()
    node = MissionRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
