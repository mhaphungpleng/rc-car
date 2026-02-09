#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from eva_interfaces.srv import SetLidarTune


@dataclass
class Cluster:
    i0: int
    i1: int
    width_m: float
    dist_m: float
    bearing_rad: float
    score: float


def median(vals: List[float]) -> float:
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return float("nan")
    mid = n // 2
    return s[mid] if (n % 2 == 1) else 0.5 * (s[mid - 1] + s[mid])


class ScanTargetDetector(Node):
    def __init__(self):
        
        super().__init__("scan_target_detector")

        self.srv_apply = self.create_service(
            SetLidarTune,
            "/apply_lidar_tune",
            self.on_apply_lidar_tune
        )

        # ---- parameters ----
        # Target width range (meters): 20–30 cm
        self.declare_parameter("min_target_width_m", 0.10)
        self.declare_parameter("max_target_width_m", 0.40)

        # Distance gate (meters)
        self.declare_parameter("min_dist_m", 0.50)
        self.declare_parameter("max_dist_m", 2.50)

        # Clustering
        self.declare_parameter("cluster_gap_m", 0.18)
        self.declare_parameter("min_cluster_points", 3)

        # Hold last detection (seconds)
        self.declare_parameter("lost_hold_s", 0.45)

        # Limit FOV (deg)
        self.declare_parameter("fov_min_deg", -35.0)
        self.declare_parameter("fov_max_deg", 35.0)

        # smoothing (EMA)
        self.declare_parameter("ema_alpha", 0.25)

        # optional intensity (OFF by default)
        self.declare_parameter("use_intensity", False)
        self.declare_parameter("min_intensity", 800.0)

        # ---- NEW: soft preference (do NOT hard lock center) ----
        # ลงโทษ |y| เพื่อไม่ให้ไปจับก้อนด้านบน แต่ยังตามตอนเลี้ยวได้
        self.declare_parameter("y_penalty", 0.7)  # 0.4–1.2 ปรับตามหน้างาน

        # ---- NEW: tracking (association) เพื่อไม่ให้สลับเป้าหมาย ----
        self.declare_parameter("assoc_wx", 0.8)   # โทษความต่าง x จากเฟรมก่อน
        self.declare_parameter("assoc_wy", 1.2)   # โทษความต่าง y จากเฟรมก่อน (มักสำคัญกว่า)
        self.declare_parameter("assoc_enable", True)

        # ---- get params ----
        self.min_target_width_m = float(self.get_parameter("min_target_width_m").value)
        self.max_target_width_m = float(self.get_parameter("max_target_width_m").value)

        self.min_dist_m = float(self.get_parameter("min_dist_m").value)
        self.max_dist_m = float(self.get_parameter("max_dist_m").value)

        self.cluster_gap_m = float(self.get_parameter("cluster_gap_m").value)
        self.min_cluster_points = int(self.get_parameter("min_cluster_points").value)

        self.lost_hold_s = float(self.get_parameter("lost_hold_s").value)

        self.fov_min = math.radians(float(self.get_parameter("fov_min_deg").value))
        self.fov_max = math.radians(float(self.get_parameter("fov_max_deg").value))

        self.ema_alpha = float(self.get_parameter("ema_alpha").value)

        self.use_intensity = bool(self.get_parameter("use_intensity").value)
        self.min_intensity = float(self.get_parameter("min_intensity").value)

        self.y_penalty = float(self.get_parameter("y_penalty").value)

        self.assoc_wx = float(self.get_parameter("assoc_wx").value)
        self.assoc_wy = float(self.get_parameter("assoc_wy").value)
        self.assoc_enable = bool(self.get_parameter("assoc_enable").value)

        # ---- pubs ----
        self.pub_found = self.create_publisher(Bool, "/target_found", 10)
        self.pub_dist = self.create_publisher(Float32, "/target_distance", 10)
        self.pub_bearing = self.create_publisher(Float32, "/target_bearing", 10)
        self.pub_width = self.create_publisher(Float32, "/target_width", 10)

        # ---- sub ----
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.cb_scan, qos_profile_sensor_data
        )

        # ---- state ----
        self.last_seen_time = 0.0
        self.last_dist = float("nan")
        self.last_bearing = float("nan")
        self.last_width = float("nan")

        # EMA state
        self.filt_dist: Optional[float] = None
        self.filt_bearing: Optional[float] = None

        # tracking state (prev target position in XY)
        self.prev_x: Optional[float] = None
        self.prev_y: Optional[float] = None

        self.get_logger().info(
            f"scan_target_detector started | width=[{self.min_target_width_m:.2f},{self.max_target_width_m:.2f}]m "
            f"dist=[{self.min_dist_m:.2f},{self.max_dist_m:.2f}]m "
            f"fov=[{math.degrees(self.fov_min):.0f},{math.degrees(self.fov_max):.0f}]deg "
            f"y_penalty={self.y_penalty:.2f} assoc={'on' if self.assoc_enable else 'off'}"
        )
    def on_apply_lidar_tune(self, req, res):
        # ----- copy req -> runtime vars -----
        self.min_target_width_m = float(req.min_target_width_m)
        self.max_target_width_m = float(req.max_target_width_m)

        self.min_dist_m = float(req.min_dist_m)
        self.max_dist_m = float(req.max_dist_m)

        self.cluster_gap_m = float(req.cluster_gap_m)
        self.min_cluster_points = int(req.min_cluster_points)

        self.lost_hold_s = float(req.lost_hold_s)

        self.fov_min = math.radians(float(req.fov_min_deg))
        self.fov_max = math.radians(float(req.fov_max_deg))

        self.ema_alpha = float(req.ema_alpha)

        self.use_intensity = bool(req.use_intensity)
        self.min_intensity = float(req.min_intensity)

        self.y_penalty = float(req.y_penalty)
        self.assoc_wx = float(req.assoc_wx)
        self.assoc_wy = float(req.assoc_wy)
        self.assoc_enable = bool(req.assoc_enable)  # ✅ เติม

            # ✅ reset state กันค่าค้างจาก config เก่า
        self.filt_dist = None
        self.filt_bearing = None
        self.prev_x = None
        self.prev_y = None

        res.success = True
        res.message = "Lidar tune applied"
        return res


    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_scan(self, scan: LaserScan):
        now = self.now_s()
        best = self.find_best_cluster(scan)

        if best is not None:
            self.last_seen_time = now

            # EMA smoothing
            a = self.ema_alpha
            if self.filt_dist is None:
                self.filt_dist = best.dist_m
                self.filt_bearing = best.bearing_rad
            else:
                self.filt_dist = a * best.dist_m + (1.0 - a) * self.filt_dist
                self.filt_bearing = a * best.bearing_rad + (1.0 - a) * self.filt_bearing

            self.last_dist = float(self.filt_dist)
            self.last_bearing = float(self.filt_bearing)
            self.last_width = float(best.width_m)

            # update tracking (use RAW best, not filtered)
            self.prev_x = best.dist_m * math.cos(best.bearing_rad)
            self.prev_y = best.dist_m * math.sin(best.bearing_rad)

        found = (now - self.last_seen_time) <= self.lost_hold_s

        self.pub_found.publish(Bool(data=bool(found)))
        if found:
            self.pub_dist.publish(Float32(data=float(self.last_dist)))
            self.pub_bearing.publish(Float32(data=float(self.last_bearing)))
            self.pub_width.publish(Float32(data=float(self.last_width)))
        else:
            nan = float("nan")
            self.pub_dist.publish(Float32(data=nan))
            self.pub_bearing.publish(Float32(data=nan))
            self.pub_width.publish(Float32(data=nan))

    def find_best_cluster(self, scan: LaserScan) -> Optional[Cluster]:
        ranges = scan.ranges

        use_int = getattr(self, "use_intensity", False)
        intens = scan.intensities if (use_int and len(scan.intensities) == len(ranges)) else None

        def angle_of(i: int) -> float:
            return scan.angle_min + i * scan.angle_increment

        n = len(ranges)

        def valid(i: int) -> bool:
            r = ranges[i]
            if r is None or math.isinf(r) or math.isnan(r):
                return False

            ang = angle_of(i)
            if ang < self.fov_min or ang > self.fov_max:
                return False

            rmin = max(scan.range_min, self.min_dist_m)
            rmax = min(scan.range_max, self.max_dist_m)
            if r < rmin or r > rmax:
                return False

            if intens is not None and intens[i] < self.min_intensity:
                return False

            return True

        # cluster by continuity of range
        clusters: List[Tuple[int, int]] = []
        i = 0
        while i < n:
            if not valid(i):
                i += 1
                continue

            i0 = i
            prev_r = ranges[i]
            i += 1

            while i < n and valid(i) and abs(ranges[i] - prev_r) <= self.cluster_gap_m:
                prev_r = ranges[i]
                i += 1

            i1 = i - 1
            if (i1 - i0 + 1) >= self.min_cluster_points:
                clusters.append((i0, i1))

        best: Optional[Cluster] = None

        for (i0, i1) in clusters:
            vals = [ranges[k] for k in range(i0, i1 + 1) if not math.isinf(ranges[k]) and not math.isnan(ranges[k])]
            if len(vals) < self.min_cluster_points:
                continue

            dist_med = median(vals)
            mid = (i0 + i1) // 2
            bearing = angle_of(mid)

            # width estimate using chord
            span = (i1 - i0) * scan.angle_increment
            width_est = 2.0 * dist_med * math.sin(abs(span) / 2.0)

            # width in desired range
            if width_est < self.min_target_width_m or width_est > self.max_target_width_m:
                continue

            center = 0.5 * (self.min_target_width_m + self.max_target_width_m)
            width_err = abs(width_est - center)

            # convert to XY (for y penalty + association)
            x = dist_med * math.cos(bearing)
            y = dist_med * math.sin(bearing)

            # association cost (prefer continuity)
            assoc = 0.0
            if self.assoc_enable and (self.prev_x is not None) and (self.prev_y is not None):
                assoc = abs(x - self.prev_x) * self.assoc_wx + abs(y - self.prev_y) * self.assoc_wy

            # score: width match, prefer nearer, prefer centered-ish, prefer continuity, and penalize high |y|
            score = (
                width_err * 12.0
                + dist_med * 0.4
                + abs(bearing) * 0.2
                + assoc
                + abs(y) * self.y_penalty
            )

            c = Cluster(i0=i0, i1=i1, width_m=width_est, dist_m=dist_med, bearing_rad=bearing, score=score)
            if best is None or c.score < best.score:
                best = c


        return best


def main():
    rclpy.init()
    node = ScanTargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

