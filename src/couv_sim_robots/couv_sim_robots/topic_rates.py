#!/usr/bin/env python3
"""Measure topic rates in SIMULATION time (header stamps / clock), plus the real-time factor.

`ros2 topic hz` measures wall-clock rate, which is lower than the sensor rate whenever
Gazebo runs slower than real time. Usage:
    ros2 run couv_sim_robots topic_rates.py [duration_wall_s] [topic ...]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message

DEFAULT = [
    "/ugv/camera/color/image_raw", "/ugv/camera/depth/image_rect_raw", "/ugv/camera/color/camera_info",
    "/ugv/camera/imu", "/ugv/odom", "/ugv/ground_truth/odom", "/ugv/joint_states",
    "/drone/camera/color/image_raw", "/drone/camera/depth/image_rect_raw", "/drone/camera/color/camera_info",
    "/drone/camera/imu", "/drone/imu", "/drone/air_pressure", "/drone/rangefinder", "/drone/ground_truth/odom",
    "/drone/joint_states",
]


def stamp(msg):
    h = getattr(msg, "header", None)
    return h.stamp.sec + h.stamp.nanosec * 1e-9 if h else None


def main():
    args = sys.argv[1:]
    dur = float(args.pop(0)) if args and args[0].replace(".", "").isdigit() else 15.0
    topics = args or DEFAULT
    rclpy.init()
    node = Node("topic_rates")
    t0 = time.time()
    while time.time() - t0 < 2.0:  # discovery
        rclpy.spin_once(node, timeout_sec=0.1)
    names = dict(node.get_topic_names_and_types())
    data = {t: [] for t in topics}
    clock = []
    node.create_subscription(Clock, "/clock", lambda m: clock.append((time.time(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
    for t in topics:
        if t not in names:
            continue
        node.create_subscription(get_message(names[t][0]), t, lambda m, t=t: data[t].append(stamp(m)),
                                 qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(clock) > 1:
        (w0, s0), (w1, s1) = clock[0], clock[-1]
        print(f"real-time factor: {(s1 - s0) / (w1 - w0):.2f}  (sim {s1 - s0:.1f} s in wall {w1 - w0:.1f} s)")
    print(f"{'topic':45s} {'msgs':>6s} {'rate (sim time) [Hz]':>22s}")
    for t in topics:
        st = sorted(s for s in data[t] if s is not None)
        if t not in names:
            print(f"{t:45s} {'-':>6s} {'(not advertised)':>22s}")
        elif len(st) > 2 and st[-1] > st[0]:
            print(f"{t:45s} {len(st):6d} {(len(st) - 1) / (st[-1] - st[0]):22.1f}")
        else:
            print(f"{t:45s} {len(data[t]):6d} {'n/a':>22s}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
