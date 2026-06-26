#!/usr/bin/env python3
"""
summit_pcd_mapper.py
====================
Build a single 3D POINT-CLOUD MAP (.pcd) from the Summit XL's 3D LiDAR, in the SLAM map frame
(robot_map), WHILE you drive with the gamepad during slam_toolbox mapping.

slam_toolbox only saves a 2D occupancy grid; this gives you the dense 3D cloud as a portable .pcd
(+ a downsampled .json in our G1 analysis format).

Robot facts (from the diagnostics):
  cloud topic : /robot/top_laser/point_cloud   (sensor_msgs/PointCloud2, BEST_EFFORT)
  map frame   : robot_map      (published by slam_toolbox while mapping is active)
  ROS 2 Humble, ROS_DOMAIN_ID=39, RMW=rmw_cyclonedds_cpp

RUN (on a PC on the same network, or on the robot), WHILE `bash map_res.sh ...` is active:
  export ROS_DOMAIN_ID=39
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  python3 summit_pcd_mapper.py
  # drive slowly with the gamepad, revisit places to close loops; Ctrl+C to save.

OUTPUTS (current dir):
  summit_map.pcd   - portable point cloud (CloudCompare / Open3D / PCL / RViz)
  summit_map.json  - downsampled [x,y,z] list, same shape as our dataset/map_full.json

DEPS: only rclpy + tf2_ros + numpy (all in ROS 2 Humble). open3d is OPTIONAL (ASCII .pcd fallback).
No tf2_sensor_msgs / sensor_msgs_py needed (we parse + transform with NumPy).
"""
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import tf2_ros

CLOUD_TOPIC = "/robot/top_laser/point_cloud"
MAP_FRAME = "robot_map"          # SLAM frame. If mapping WITHOUT slam_toolbox, use "robot_odom" (drifts).
VOXEL = 0.05                     # m: accumulation/downsample voxel
H_MIN, H_MAX = -0.30, 3.0        # m: keep this height band in map frame (drop floor noise / high ceiling)
MIN_HITS = 2                     # a voxel must be hit >= this many times to be kept (denoise)


def read_xyz(msg):
    """PointCloud2 -> (N,3) float32 array, NaN-free. Assumes x,y,z are FLOAT32 (standard RSLIDAR)."""
    off = {f.name: f.offset for f in msg.fields}
    if not all(k in off for k in ("x", "y", "z")):
        return np.empty((0, 3), np.float32)
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 3), np.float32)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)

    def col(o):
        return raw[:, o:o + 4].copy().view("<f4" if not msg.is_bigendian else ">f4").reshape(-1)
    pts = np.stack([col(off["x"]), col(off["y"]), col(off["z"])], axis=1).astype(np.float32)
    return pts[np.isfinite(pts).all(axis=1)]


def tf_matrix(tr):
    """TransformStamped.transform -> (R 3x3, t 3) in map frame."""
    q = tr.rotation; t = tr.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)]])
    return R, np.array([t.x, t.y, t.z])


class Mapper(Node):
    def __init__(self):
        super().__init__("summit_pcd_mapper")
        self.voxels = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self.cb, qos)
        self.n = 0; self.used = 0
        self.get_logger().info(f"Subscribed {CLOUD_TOPIC}; accumulating in '{MAP_FRAME}'. "
                               f"Drive slowly, Ctrl+C to save.")

    def cb(self, msg):
        self.n += 1
        try:
            tf = self.tf_buffer.lookup_transform(MAP_FRAME, msg.header.frame_id, rclpy.time.Time())
        except Exception as e:
            if self.n % 30 == 0:
                self.get_logger().warn(f"waiting for TF {MAP_FRAME} <- {msg.header.frame_id} ({e})")
            return
        pts = read_xyz(msg)
        if len(pts) == 0:
            return
        R, t = tf_matrix(tf.transform)
        pm = pts @ R.T + t                                   # transform to map frame
        m = (pm[:, 2] >= H_MIN) & (pm[:, 2] <= H_MAX)
        pm = pm[m]
        keys = np.round(pm / VOXEL).astype(np.int64)
        for k in map(tuple, keys):
            self.voxels[k] = self.voxels.get(k, 0) + 1
        self.used += 1
        if self.used % 10 == 0:
            self.get_logger().info(f"clouds={self.used}  voxels={len(self.voxels)}")

    def save(self):
        pts = np.array([[k[0] * VOXEL, k[1] * VOXEL, k[2] * VOXEL]
                        for k, c in self.voxels.items() if c >= MIN_HITS], dtype=np.float64)
        if len(pts) == 0:
            self.get_logger().error("No points accumulated. Is mapping active (robot_map TF) and the "
                                    "cloud topic publishing?")
            return
        try:
            import open3d as o3d
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(pts)
            o3d.io.write_point_cloud("summit_map.pcd", pc)
        except Exception:
            self._write_pcd_ascii("summit_map.pcd", pts)
        json.dump({"frame": MAP_FRAME, "voxel": VOXEL, "src": "summit_xl top_laser/point_cloud",
                   "npts": len(pts), "points": [[round(v, 3) for v in p] for p in pts]},
                  open("summit_map.json", "w"))
        self.get_logger().info(f"SAVED summit_map.pcd + summit_map.json  ({len(pts)} points)")

    @staticmethod
    def _write_pcd_ascii(fn, pts):
        with open(fn, "w") as f:
            f.write("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA ascii\n")
            for p in pts:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")


def main():
    rclpy.init()
    node = Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
