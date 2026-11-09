#!/usr/bin/env python3
"""Detect the UGV landing-pad ArUco marker in the drone colour image.

Publishes /drone/aruco/marker_pose (geometry_msgs/PoseStamped): pose of the marker frame in
the camera optical frame (OpenCV convention, IPPE_SQUARE), stamped with the image time.
While /mission/state is aruco_start / aruco_end it also saves one annotated image per phase
to `image_dir` and logs the mean / spread of the measurements when the phase ends.
Marker dictionary / id / size come from couv_sim_robots config/frames.yaml (`ugv.aruco`).
"""
import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

CAPTURE_STATES = ("aruco_start", "aruco_end")


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")
        frames = yaml.safe_load(open(os.path.join(get_package_share_directory("couv_sim_robots"), "config",
                                                  "frames.yaml")))
        a = frames["ugv"]["aruco"]
        self.marker_id, s = a["id"], a["size"]
        self.obj = np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0], [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]], np.float32)
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, a["dictionary"])),
                                                cv2.aruco.DetectorParameters())
        self.image_dir = os.path.expanduser(self.declare_parameter("image_dir", "").value)
        self.period = 1.0 / self.declare_parameter("rate", 10.0).value
        self.K = self.D = None
        self.state, self.last, self.saved, self.samples = "", -1e9, set(), []
        self.pub = self.create_publisher(PoseStamped, "/drone/aruco/marker_pose", 10)
        self.create_subscription(CameraInfo, "/drone/camera/color/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/drone/camera/color/image_raw", self.on_image, qos_profile_sensor_data)
        self.create_subscription(String, "/mission/state", self.on_state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_info(self, m):
        self.K, self.D = np.array(m.k).reshape(3, 3), np.array(m.d)

    def on_state(self, m):
        if self.state in CAPTURE_STATES and m.data != self.state:
            self.report(self.state)
        self.state, self.samples = m.data, []

    def report(self, phase):
        if not self.samples:
            self.get_logger().warn(f"{phase}: marker never detected")
            return
        t = np.array([p[0] for p in self.samples])
        self.get_logger().info(f"{phase}: {len(t)} detections, marker in camera {np.round(t.mean(0), 3)} m, "
                               f"spread {np.round(t.std(0) * 1000, 1)} mm, distance {np.linalg.norm(t.mean(0)):.2f} m")

    def on_image(self, m):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.K is None or stamp - self.last < self.period:
            return
        self.last = stamp
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
        if ids is None or self.marker_id not in ids.ravel():
            return
        c = corners[list(ids.ravel()).index(self.marker_id)].reshape(4, 2)
        ok, rvec, tvec = cv2.solvePnP(self.obj, c, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return
        q = Rotation.from_rotvec(rvec.ravel()).as_quat()
        p = PoseStamped()
        p.header = m.header
        p.pose.position.x, p.pose.position.y, p.pose.position.z = (float(v) for v in tvec.ravel())
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = (float(v) for v in q)
        self.pub.publish(p)
        if self.state in CAPTURE_STATES:
            self.samples.append((tvec.ravel(), rvec.ravel()))
            if self.image_dir and self.state not in self.saved:
                self.saved.add(self.state)
                os.makedirs(self.image_dir, exist_ok=True)
                vis = cv2.cvtColor(img, cv2.COLOR_RGB2BGR).copy()
                cv2.aruco.drawDetectedMarkers(vis, [c.reshape(1, 4, 2)], np.array([[self.marker_id]]))
                cv2.drawFrameAxes(vis, self.K, self.D, rvec, tvec, 0.15)
                cv2.putText(vis, f"{self.state}: marker at {np.linalg.norm(tvec):.2f} m", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.imwrite(os.path.join(self.image_dir, f"{self.state}.png"), vis)


def main():
    rclpy.init()
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
