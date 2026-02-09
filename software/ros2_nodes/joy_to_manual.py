#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def apply_deadzone(x: float, dz: float) -> float:
    return 0.0 if abs(x) < dz else x


class JoyToManual(Node):
    """
    Sub:
      /joy (sensor_msgs/Joy)

    Pub:
      /cmd_esc_manual (std_msgs/Int16)
      /cmd_s1_manual  (std_msgs/Int16)

    Mapping:
      - Left stick Y (axes[left_y]) : only forward
          center/neutral => ESC = esc_min (default 1000)
          push up        => esc_min..esc_max (default 1000..2000)
          push down      => esc_min (no reverse)

      - Right stick X (axes[right_x]) : steering
          left/right => s1_min..s1_max (default 0..180, center 90)
    """

    def __init__(self):
        super().__init__("joy_to_manual")

        # -------- parameters --------
        self.declare_parameter("rate_hz", 50.0)

        # Joy axis indices (common for many controllers; adjust if needed)
        self.declare_parameter("axis_left_y", 1)   # L-stick vertical
        self.declare_parameter("axis_right_x", 2)  # R-stick horizontal

        # Deadzones
        self.declare_parameter("deadzone_throttle", 0.08)
        self.declare_parameter("deadzone_steer", 0.08)

        # ESC mapping
        self.declare_parameter("esc_min", 1050)
        self.declare_parameter("esc_max", 1300)

        # Ramp (slew) rate for ESC: units per second
        # เช่น 800 => จาก 1000 ไป 2000 ใช้ ~1.25s
        self.declare_parameter("esc_ramp_up_per_s", 300.0)
        self.declare_parameter("esc_ramp_down_per_s", 1000.0)

        # Servo mapping
        self.declare_parameter("s1_center", 90)
        self.declare_parameter("s1_min", 72)
        self.declare_parameter("s1_max", 108)

        # Invert axes if your controller is opposite
        self.declare_parameter("invert_throttle", False)  # หลายจอย: ดันขึ้น = -1
        self.declare_parameter("invert_steer", True)

        # -------- pubs/subs --------
        self.pub_esc = self.create_publisher(Int16, "/cmd_esc_manual", 10)
        self.pub_s1  = self.create_publisher(Int16, "/cmd_s1_manual", 10)
        self.sub_joy = self.create_subscription(Joy, "/joy", self.cb_joy, 10)

        # -------- state --------
        self.last_joy = None
        self.esc_cmd = int(self.get_parameter("esc_min").value)
        self.s1_cmd  = int(self.get_parameter("s1_center").value)

        self._last_t = time.monotonic()
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)

        self.get_logger().info("joy_to_manual started: /joy -> /cmd_esc_manual, /cmd_s1_manual")

    def cb_joy(self, msg: Joy):
        self.last_joy = msg

    def on_timer(self):
        now = time.monotonic()
        dt = now - self._last_t
        self._last_t = now
        if dt <= 0.0:
            dt = 1e-3

        # defaults
        esc_min = int(self.get_parameter("esc_min").value)
        esc_max = int(self.get_parameter("esc_max").value)
        s1_center = int(self.get_parameter("s1_center").value)
        s1_min = int(self.get_parameter("s1_min").value)
        s1_max = int(self.get_parameter("s1_max").value)

        # If no joy yet -> hold safe
        if self.last_joy is None or len(self.last_joy.axes) == 0:
            self.esc_cmd = esc_min
            self.s1_cmd = s1_center
            self.pub_esc.publish(Int16(data=self.esc_cmd))
            self.pub_s1.publish(Int16(data=self.s1_cmd))
            return

        axL = int(self.get_parameter("axis_left_y").value)
        axR = int(self.get_parameter("axis_right_x").value)

        # Guard index
        ly = self.last_joy.axes[axL] if axL < len(self.last_joy.axes) else 0.0
        rx = self.last_joy.axes[axR] if axR < len(self.last_joy.axes) else 0.0

        # Invert (common: push up = -1 on many pads)
        if bool(self.get_parameter("invert_throttle").value):
            ly = -ly
        if bool(self.get_parameter("invert_steer").value):
            rx = -rx

        # Deadzones
        ly = apply_deadzone(ly, float(self.get_parameter("deadzone_throttle").value))
        rx = apply_deadzone(rx, float(self.get_parameter("deadzone_steer").value))

        # ---------- Throttle target (only forward) ----------
        # center (0) => 1000
        # up (+1)    => 2000
        # down (-1)  => 1000 (no reverse)
        forward = max(0.0, float(ly))  # only positive
        esc_target = esc_min + int(round(forward * (esc_max - esc_min)))
        esc_target = clamp(esc_target, esc_min, esc_max)

        # Smooth ramp to target
        up_rate = float(self.get_parameter("esc_ramp_up_per_s").value)
        down_rate = float(self.get_parameter("esc_ramp_down_per_s").value)

        if esc_target > self.esc_cmd:
            step = up_rate * dt
            self.esc_cmd = int(min(esc_target, self.esc_cmd + step))
        else:
            step = down_rate * dt
            self.esc_cmd = int(max(esc_target, self.esc_cmd - step))

        self.esc_cmd = clamp(self.esc_cmd, esc_min, esc_max)

        # ---------- Steering (servo) ----------
        # rx in [-1..1] -> map to [s1_min..s1_max] around center
        # ใช้ระยะจาก center ไป min/max ให้สมมาตร
        left_span = abs(s1_center - s1_min)
        right_span = abs(s1_max - s1_center)
        span = min(left_span, right_span) if (left_span > 0 and right_span > 0) else max(left_span, right_span)

        s1_target = int(round(s1_center + rx * span))
        s1_target = clamp(s1_target, min(s1_min, s1_max), max(s1_min, s1_max))
        self.s1_cmd = s1_target

        # Publish
        self.pub_esc.publish(Int16(data=int(self.esc_cmd)))
        self.pub_s1.publish(Int16(data=int(self.s1_cmd)))


def main():
    rclpy.init()
    node = JoyToManual()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

