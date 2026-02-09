#!/usr/bin/env python3
import json
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import String, Int16, Bool, Float32
from eva_interfaces.srv import SetPidTune

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def wrap_deg(a: float) -> float:
    a = a % 360.0
    return a + 360.0 if a < 0 else a


def shortest_error(target: float, cur: float) -> float:
    # error in [-180, +180)
    e = (target - cur) % 360.0
    if e >= 180.0:
        e -= 360.0
    return e


class YawHoldPD(Node):
    def __init__(self):
        super().__init__("yaw_hold_pd")
        #service
        self.srv_pid = self.create_service(SetPidTune, "/apply_pid_tune", self.on_apply_pid_tune)
        # ---------------- Parameters (ปรับได้ runtime) ----------------
        self.declare_parameter("kp", 0.7)
        self.declare_parameter("kd", 0.15)
        self.declare_parameter("deadband_deg", 2.0)

        self.declare_parameter("s1_center", 90)
        self.declare_parameter("s1_min", 72)
        self.declare_parameter("s1_max", 108)

        self.declare_parameter("timeout_s", 0.4)

        # กลับทิศง่าย (เหมือน turntune)
        self.declare_parameter("turn_sign", -1.0)

        # derivative low-pass (0..1)
        self.declare_parameter("d_lpf_alpha", 0.25)

        self.declare_parameter("yaw_lpf_alpha", 0.2)

        # ---------- Target bearing options ----------
        # bearing_mode:
        #   "servo" => /target_bearing เป็นมุม servo เลย (72..108)
        #   "deg"   => /target_bearing เป็นมุมเบี่ยงเบน (deg) เช่น -15..+15 แล้ว map ไป servo
        self.declare_parameter("bearing_mode", "servo")

        # map when bearing_mode="deg"
        self.declare_parameter("bearing_deg_max", 30.0)  # deg ที่ถือว่าเต็มสเกล
        self.declare_parameter("bearing_timeout_s", 0.25)

        self._load_params()

        # ---------------- State ----------------
        self.yaw = None
        self.yaw_ref = None
        self.last_yaw_t = 0.0

        self.target_found = False

        self.bearing = float(self.s1_center)  # default safe
        self.last_bearing_t = 0.0

        self.prev_err = 0.0
        self.prev_t = time.time()
        self.d_filt = 0.0

        # ---------------- Subs ----------------
        self.sub_tel = self.create_subscription(
            String, "/telemetry_json", self.cb_tel, qos_profile_sensor_data
        )
        self.sub_yaw_reset = self.create_subscription(
            Bool, "/yaw_ref_reset", self.cb_reset, 10
        )
        self.sub_target_found = self.create_subscription(
            Bool, "/target_found", self.cb_target, 10
        )
        self.sub_target_bearing = self.create_subscription(
            Float32, "/target_bearing", self.cb_bearing, 10
        )

        # ---------------- Pub ----------------
        self.pub_s1 = self.create_publisher(Int16, "/cmd_s1_auto", 10)

        # ---------------- Timer ----------------
        self.timer = self.create_timer(1.0 / 30.0, self.loop)

        # ---------------- Param callback ----------------
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info("YawHoldPD started (publishing /cmd_s1_auto)")

    # ---------- params ----------
    def _load_params(self):
        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.deadband = float(self.get_parameter("deadband_deg").value)

        self.s1_center = int(self.get_parameter("s1_center").value)
        self.s1_min = int(self.get_parameter("s1_min").value)
        self.s1_max = int(self.get_parameter("s1_max").value)

        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.turn_sign = float(self.get_parameter("turn_sign").value)

        self.d_lpf_alpha = float(self.get_parameter("d_lpf_alpha").value)
        self.d_lpf_alpha = clamp(self.d_lpf_alpha, 0.0, 1.0)

        self.yaw_lpf_alpha = float(self.get_parameter("yaw_lpf_alpha").value)
        self.yaw_lpf_alpha = clamp(self.yaw_lpf_alpha, 0.01, 1.0)

        self.yaw_lpf_alpha = float(self.get_parameter("yaw_lpf_alpha").value)
        self.yaw_lpf_alpha = clamp(self.yaw_lpf_alpha, 0.01, 1.0)

        self.bearing_mode = str(self.get_parameter("bearing_mode").value).strip().lower()
        if self.bearing_mode not in ("servo", "deg"):
            self.bearing_mode = "servo"

        self.bearing_deg_max = float(self.get_parameter("bearing_deg_max").value)
        if self.bearing_deg_max <= 1e-3:
            self.bearing_deg_max = 30.0

        self.bearing_timeout_s = float(self.get_parameter("bearing_timeout_s").value)

    def _on_param_change(self, params):
        self._load_params()
        return SetParametersResult(successful=True)

    # ---------- callbacks ----------
    def cb_tel(self, msg: String):
        try:
            d = json.loads(msg.data)
            if "yaw" not in d:
                return

            # 1. รับค่าดิบมาก่อน
            raw_yaw = wrap_deg(float(d["yaw"]))
            
            # 2. คำนวณ Filter
            if self.yaw is None:
                self.yaw = raw_yaw  # ค่าแรกให้ใช้เลย
            else:
                # หาความต่างระหว่างค่าใหม่(raw) กับค่าเก่า(self.yaw)
                # ใช้ shortest_error เพื่อแก้ปัญหาข้ามจาก 359->0 องศา
                diff = shortest_error(raw_yaw, self.yaw)
                
                # สูตร LPF: ค่าเดิม + (ส่วนต่าง * alpha)
                self.yaw = wrap_deg(self.yaw + (diff * self.yaw_lpf_alpha))

            self.last_yaw_t = time.time()

            # capture yaw_ref at startup
            if self.yaw_ref is None:
                self.yaw_ref = self.yaw
                self.prev_err = 0.0
                self.prev_t = time.time()
                self.d_filt = 0.0
                self.get_logger().info(f"Captured yaw_ref at start: {self.yaw_ref:.1f}")
        except Exception:
            pass

    def cb_reset(self, msg: Bool):
        if msg.data and (self.yaw is not None):
            self.yaw_ref = self.yaw
            self.prev_err = 0.0
            self.prev_t = time.time()
            self.d_filt = 0.0
            self.get_logger().info(f"Yaw ref RESET -> {self.yaw_ref:.1f}")

    def cb_target(self, msg: Bool):
        self.target_found = bool(msg.data)

    def cb_bearing(self, msg: Float32):
        self.bearing = float(msg.data)
        self.last_bearing_t = time.time()
    def on_apply_pid_tune(self, req, res):
        # ----- copy req -> runtime vars -----
        self.kp = float(req.kp)
        self.kd = float(req.kd)
        self.deadband = float(req.deadband_deg)

        self.s1_center = int(req.s1_center)
        self.s1_min = int(req.s1_min)
        self.s1_max = int(req.s1_max)

        self.timeout_s = float(req.timeout_s)
        self.turn_sign = float(req.turn_sign)

        self.d_lpf_alpha = float(req.d_lpf_alpha)
        self.d_lpf_alpha = clamp(self.d_lpf_alpha, 0.0, 1.0)

        self.yaw_lpf_alpha = float(req.yaw_lpf_alpha)
        self.yaw_lpf_alpha = clamp(self.yaw_lpf_alpha, 0.01, 1.0)

        bm = str(req.bearing_mode).strip().lower()
        self.bearing_mode = bm if bm in ("servo", "deg") else "servo"

        self.bearing_deg_max = float(req.bearing_deg_max) if float(req.bearing_deg_max) > 1e-3 else 30.0
        self.bearing_timeout_s = float(req.bearing_timeout_s)

        # reset derivative filter/state กันค่าค้าง
        self.prev_err = 0.0
        self.prev_t = time.time()
        self.d_filt = 0.0

        res.success = True
        res.message = "PID tune applied"
        return res



    # ---------- loop ----------
    def loop(self):
        now = time.time()

        # --- If target tracking is active, steer from bearing ---
        if self.target_found:
            # bearing timeout -> fallback to yaw hold (avoid stuck steering)
            if (now - self.last_bearing_t) > self.bearing_timeout_s:
                # fall back to yaw hold below
                pass
            else:
                if not math.isfinite(self.bearing):
                    self.pub_s1.publish(Int16(data=self.s1_center))
                    return
                bearing_deg = math.degrees(self.bearing)
                bearing_deg = clamp(bearing_deg, -self.bearing_deg_max, self.bearing_deg_max)

                max_offset = max(self.s1_max - self.s1_center, self.s1_center - self.s1_min)
                servo_offset = (bearing_deg / self.bearing_deg_max) * max_offset

                out = float(self.s1_center) + (servo_offset * (self.turn_sign*-1))
                if not math.isfinite(out):
                    self.get_logger().warn("YawHoldPD: out is NaN/inf -> force center")
                    out = float(self.s1_center)
                out = int(round(clamp(out, self.s1_min, self.s1_max)))

                self.pub_s1.publish(Int16(data=out))
                return

        # --- yaw hold mode ---
        if self.yaw is None or (now - self.last_yaw_t) > self.timeout_s:
            self.pub_s1.publish(Int16(data=self.s1_center))
            return

        err = shortest_error(self.yaw_ref, self.yaw)
        if abs(err) < self.deadband:
            err = 0.0

        dt = now - self.prev_t
        dt = clamp(dt, 0.005, 0.100)  # 5ms..100ms
        derr = (err - self.prev_err) / dt

        a = self.d_lpf_alpha
        self.d_filt = (1.0 - a) * self.d_filt + a * derr

        self.prev_err = err
        self.prev_t = now

        u = (self.kp * err) + (self.kd * self.d_filt)
        #out = float(self.s1_center) + (u * self.turn_sign)
        #out = int(round(clamp(out, self.s1_min, self.s1_max)))
        out = float(self.s1_center)

        self.pub_s1.publish(Int16(data=out))


def main():
    rclpy.init()
    node = YawHoldPD()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

