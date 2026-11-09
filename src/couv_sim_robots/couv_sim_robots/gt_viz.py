#!/usr/bin/env python3
"""Ground-truth visualisation helper (simulation only).

* /<robot>/ground_truth/odom  ->  TF  map -> gt/<robot>/base_link
  (a separate `gt/` tree, so it never collides with SLAM's map/odom/base_link TF;
  a second robot_state_publisher with frame_prefix gt/<robot>/ fills in the rest)
* /<robot>/camera/depth/color/points  ->  /viz/<robot>/points  (frame re-labelled into the
  gt/ tree, throttled to `cloud_rate`) so both robots' clouds can be shown together in `map`.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster


class GtViz(Node):
    def __init__(self):
        super().__init__("gt_viz")
        self.robots = self.declare_parameter("robots", ["ugv", "drone"]).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.cloud_period = 1.0 / max(self.declare_parameter("cloud_rate", 2.0).value, 1e-3)
        self.tf = TransformBroadcaster(self)
        self.last_cloud = {}
        self.cloud_pubs = {}
        for r in self.robots:
            self.create_subscription(Odometry, f"/{r}/ground_truth/odom", lambda m, r=r: self.on_odom(r, m), 10)
            self.cloud_pubs[r] = self.create_publisher(PointCloud2, f"/viz/{r}/points", 2)
        # subscribe to the (heavy) clouds only while someone watches /viz/*
        self.cloud_subs = {}
        self.create_timer(1.0, self.update_cloud_subs)

    def update_cloud_subs(self):
        for r in self.robots:
            wanted = self.cloud_pubs[r].get_subscription_count() > 0
            if wanted and r not in self.cloud_subs:
                self.cloud_subs[r] = self.create_subscription(
                    PointCloud2, f"/{r}/camera/depth/color/points", lambda m, r=r: self.on_cloud(r, m),
                    qos_profile_sensor_data)
            elif not wanted and r in self.cloud_subs:
                self.destroy_subscription(self.cloud_subs.pop(r))

    def on_odom(self, robot, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = f"gt/{robot}/base_link"
        p = msg.pose.pose
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.position.x, p.position.y, p.position.z
        t.transform.rotation = p.orientation
        self.tf.sendTransform(t)

    def on_cloud(self, robot, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_cloud.get(robot, -1e9) < self.cloud_period:
            return
        self.last_cloud[robot] = now
        msg.header.frame_id = "gt/" + msg.header.frame_id
        self.cloud_pubs[robot].publish(msg)


def main():
    rclpy.init()
    node = GtViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
