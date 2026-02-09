#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Float32, Int16
from eva_interfaces.srv import SetCCTune

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class EscPdFallback(Node):
    """
    Sub:
      /target_found (std_msgs/Bool)
      /target_distance (std_msgs/Float32)

    Pub:
      /cmd_esc_auto (std_msgs/Float32)  <-- ปรับเป็น Int16 ได้ถ้ารถคุณใช้ Int16
    """

    def __init__(self):
        super().__init__("esc_pd_fallback")
        # service
        self.srv_cc = self.create_service(SetCCTune, "/apply_cc_tune", self.on_apply_cc_tune)
        # ---------- Parameters ----------
        # loop rate
        self.declare_parameter("rate_hz", 20.0)

        # PD control (ใช้เมื่อ found=True)
        self.declare_parameter("desired_distance_m", 1.0)
        self.declare_parameter("kp", 5.0)
        self.declare_parameter("kd", 0.2)

        # speed command units (แล้วแต่รถคุณ: อาจเป็น 0..180, 0..255, 1000..2000 ฯลฯ)
        self.declare_parameter("min_cmd", 1000)
        self.declare_parameter("max_cmd", 1100)

        # base feed-forward เมื่อเจอ target
        self.declare_parameter("base_cmd_found", 1055)

        # fallback cruise เมื่อไม่เจอ target
        self.declare_parameter("fallback_cmd", 1070)

        # ramp rate (หน่วย cmd ต่อวินาที)
        self.declare_parameter("ramp_up_per_s", 10.0)
        self.declare_parameter("ramp_down_per_s", 80.0)

        # ถ้า /target_found ไม่มา หรือ stale เกินนี้ ให้ถือว่าไม่เจอ
        self.declare_parameter("found_timeout_s", 0.3)
        self.declare_parameter("dist_timeout_s", 0.3)

        # ---------- Read parameters ----------
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / self.rate_hz

        self.desired_distance_m = float(self.get_parameter("desired_distance_m").value)
        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)

        self.min_cmd = float(self.get_parameter("min_cmd").value)
        self.max_cmd = float(self.get_parameter("max_cmd").value)

        self.base_cmd_found = float(self.get_parameter("base_cmd_found").value)
        self.fallback_cmd = float(self.get_parameter("fallback_cmd").value)

        self.ramp_up_per_s = float(self.get_parameter("ramp_up_per_s").value)
        self.ramp_down_per_s = float(self.get_parameter("ramp_down_per_s").value)

        self.found_timeout_s = float(self.get_parameter("found_timeout_s").value)
        self.dist_timeout_s = float(self.get_parameter("dist_timeout_s").value)

        # ---------- QoS ----------
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ---------- State ----------
        self.last_found = False
        self.last_found_stamp = 0.0

        self.last_dist_m = float("nan")
        self.last_dist_stamp = 0.0

        self.prev_error = 0.0
        self.prev_t = None

        self.cmd_current = float(self.min_cmd)

        # ---------- Pub/Sub ----------
        self.sub_found = self.create_subscription(Bool, "/target_found", self.cb_found, qos)
        self.sub_dist = self.create_subscription(Float32, "/target_distance", self.cb_dist, qos)

        self.pub_cmd = self.create_publisher(Int16, '/cmd_esc_auto', 10)

        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info("esc_pd_fallback started")

    def cb_found(self, msg: Bool):
        self.last_found = bool(msg.data)
        self.last_found_stamp = time.time()

    def cb_dist(self, msg: Float32):
        self.last_dist_m = float(msg.data)
        self.last_dist_stamp = time.time()

    def on_timer(self):
        now = time.time()

        found_fresh = (now - self.last_found_stamp) <= self.found_timeout_s
        dist_fresh = (now - self.last_dist_stamp) <= self.dist_timeout_s

        # เงื่อนไข "ถือว่าเจอ target จริง"
        found = bool(self.last_found) and found_fresh and dist_fresh and (not math.isnan(self.last_dist_m))

        if found:
            # --- PD on distance ---
            error = self.desired_distance_m - self.last_dist_m  # อยากเข้าใกล้/ถอย ให้สลับเครื่องหมายตรงนี้ได้
            if self.prev_t is None:
                derr = 0.0
            else:
                dt = max(1e-3, now - self.prev_t)
                derr = (error - self.prev_error) 

            u = (self.kp * error) + (self.kd * derr)
            cmd_target = self.base_cmd_found - u

            self.prev_error = error
            self.prev_t = now
        else:
            # --- fallback: วิ่งคงที่แบบค่อยๆ เพิ่ม ---
            cmd_target = self.fallback_cmd

            # reset PD memory เพื่อกัน spike ตอนกลับมาเจอใหม่
            self.prev_error = 0.0
            self.prev_t = None

        # --- ramp/acceleration limit ---
        # เลือก ramp_up หรือ ramp_down ตามทิศทาง
        if cmd_target > self.cmd_current:
            step = self.ramp_up_per_s * self.dt
        else:
            step = self.ramp_down_per_s * self.dt

        diff = cmd_target - self.cmd_current
        diff = clamp(diff, -step, step)
        self.cmd_current += diff

        # clamp output
        self.cmd_current = clamp(self.cmd_current, self.min_cmd, self.max_cmd)

        # publish
        out = Int16()
        out.data = int(self.cmd_current)
        self.pub_cmd.publish(out)
    def on_apply_cc_tune(self, req, res):
        # ----- copy req -> runtime vars -----
        self.desired_distance_m = float(req.desired_distance_m)
        self.kp = float(req.kp)
        self.kd = float(req.kd)

        self.min_cmd = float(req.min_cmd)
        self.max_cmd = float(req.max_cmd)

        self.base_cmd_found = float(req.base_cmd_found)
        self.fallback_cmd = float(req.fallback_cmd)

        self.ramp_up_per_s = float(req.ramp_up_per_s)
        self.ramp_down_per_s = float(req.ramp_down_per_s)

        self.found_timeout_s = float(req.found_timeout_s)
        self.dist_timeout_s = float(req.dist_timeout_s)

        # reset PD memory กัน spike
        self.prev_error = 0.0
        self.prev_t = None
        
        res.success = True
        res.message = "CC tune applied"
        return res



def main():
    rclpy.init()
    node = EscPdFallback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

