#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Bool, String


def clamp_i16(x: int) -> int:
    return max(-32768, min(32767, int(x)))


class CmdMux(Node):
    """
    เลือกแหล่งคำสั่ง ESC/Servo:
      - AUTO:    /cmd_esc_auto,      /cmd_s1_auto
      - AUTO+:   /cmd_esc_autoplus,  /cmd_s1_autoplus
      - MANUAL:  /cmd_esc_manual,    /cmd_s1_manual

    Emergency แบบ latch:
      - /emergency_stop  True -> latched stop
      - /emergency_reset True -> clear latch

    Output ไป ESP32:
      - /cmd_esc
      - /cmd_s1
    """

    def __init__(self):
        super().__init__('cmd_mux')

        # ---- parameters ----
        self.declare_parameter('esc_stop', 1000)
        self.declare_parameter('s1_center', 90)
        self.declare_parameter('timeout_auto_s', 0.30)
        self.declare_parameter('timeout_autoplus_s', 0.30)
        self.declare_parameter('timeout_manual_s', 0.50)
        self.declare_parameter('publish_hz', 50.0)

        self.esc_stop = int(self.get_parameter('esc_stop').value)
        self.s1_center = int(self.get_parameter('s1_center').value)
        self.timeout_auto_s = float(self.get_parameter('timeout_auto_s').value)
        self.timeout_autoplus_s = float(self.get_parameter('timeout_autoplus_s').value)
        self.timeout_manual_s = float(self.get_parameter('timeout_manual_s').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        # ---- state ----
        self.mode = "MANUAL"  # "AUTO" / "AUTO+" / "MANUAL"
        self.emg_latched = False

        # AUTO
        self.auto_esc = self.esc_stop
        self.auto_s1 = self.s1_center
        self.auto_t_esc = 0.0
        self.auto_t_s1 = 0.0

        # AUTO+
        self.ap_esc = self.esc_stop
        self.ap_s1 = self.s1_center
        self.ap_t_esc = 0.0
        self.ap_t_s1 = 0.0

        # MANUAL
        self.man_esc = self.esc_stop
        self.man_s1 = self.s1_center
        self.man_t_esc = 0.0
        self.man_t_s1 = 0.0

        # ---- subs ----
        self.sub_mode = self.create_subscription(String, '/control_mode', self.cb_mode, 10)

        # AUTO topics
        self.sub_auto_esc = self.create_subscription(Int16, '/cmd_esc_auto', self.cb_auto_esc, 10)
        self.sub_auto_s1 = self.create_subscription(Int16, '/cmd_s1_auto', self.cb_auto_s1, 10)

        # AUTO+ topics (คนละอัน)
        self.sub_ap_esc = self.create_subscription(Int16, '/cmd_esc_autoplus', self.cb_ap_esc, 10)
        self.sub_ap_s1 = self.create_subscription(Int16, '/cmd_s1_autoplus', self.cb_ap_s1, 10)

        # MANUAL topics
        self.sub_man_esc = self.create_subscription(Int16, '/cmd_esc_manual', self.cb_man_esc, 10)
        self.sub_man_s1 = self.create_subscription(Int16, '/cmd_s1_manual', self.cb_man_s1, 10)

        # Emergency
        self.sub_emg = self.create_subscription(Bool, '/emergency_stop', self.cb_emg, 10)
        self.sub_emg_reset = self.create_subscription(Bool, '/emergency_reset', self.cb_emg_reset, 10)

        # ---- pubs ----
        self.pub_esc = self.create_publisher(Int16, '/cmd_esc', 10)
        self.pub_s1 = self.create_publisher(Int16, '/cmd_s1', 10)
        self.pub_emg_state = self.create_publisher(Bool, '/emergency_latched', 10)

        # ---- timer ----
        dt = 1.0 / max(1.0, self.publish_hz)
        self.timer = self.create_timer(dt, self.loop)

        self.get_logger().info("cmd_mux started: AUTO/AUTO+/MANUAL + emergency latch")

    # ---------- callbacks ----------
    def cb_mode(self, msg: String):
        s_raw = str(msg.data).strip()
        s = s_raw.upper()

        # normalize (ตัดช่องว่าง/underscore/dash)
        s2 = s.replace(' ', '').replace('_', '').replace('-', '')

        # รองรับข้อความแบบ: "AUTO+", "AUTO PLUS", "AUTOPLUS", "AUTO_PLUS"
        if s2 in ('AUTO', 'AUTOMODE'):
            self.mode = 'AUTO'
        elif s2 in ('AUTO+', 'AUTOPLUS', 'AUTOPLUSMODE', 'AUTOPLUSPLUS'):
            # ถ้าข้อความเป็น AUTO+ ตรงๆ (มี '+') ก็จะไม่เข้า s2 เพราะ s2 ตัด '-' '_' ' ' แต่ไม่ได้ตัด '+'
            # เลยเช็คเพิ่มจาก raw ด้วย
            self.mode = 'AUTO+'
        elif '+' in s_raw and 'AUTO' in s:
            self.mode = 'AUTO+'
        elif s2 in ('MANUAL', 'MAN', 'MANUALMODE', 'MANMODE'):
            self.mode = 'MANUAL'
        else:
            self.get_logger().warn(f"Unknown control_mode='{s_raw}', keep mode={self.mode}")

    # AUTO
    def cb_auto_esc(self, msg: Int16):
        self.auto_esc = int(msg.data)
        self.auto_t_esc = time.time()

    def cb_auto_s1(self, msg: Int16):
        self.auto_s1 = int(msg.data)
        self.auto_t_s1 = time.time()

    # AUTO+
    def cb_ap_esc(self, msg: Int16):
        self.ap_esc = int(msg.data)
        self.ap_t_esc = time.time()

    def cb_ap_s1(self, msg: Int16):
        self.ap_s1 = int(msg.data)
        self.ap_t_s1 = time.time()

    # MANUAL
    def cb_man_esc(self, msg: Int16):
        self.man_esc = int(msg.data)
        self.man_t_esc = time.time()

    def cb_man_s1(self, msg: Int16):
        self.man_s1 = int(msg.data)
        self.man_t_s1 = time.time()

    # Emergency
    def cb_emg(self, msg: Bool):
        if bool(msg.data):
            self.emg_latched = True

    def cb_emg_reset(self, msg: Bool):
        if bool(msg.data):
            self.emg_latched = False

    # ---------- main loop ----------
    def loop(self):
        now = time.time()

        # publish emg state for GUI
        self.pub_emg_state.publish(Bool(data=bool(self.emg_latched)))

        # emergency overrides everything
        if self.emg_latched:
            self.pub_esc.publish(Int16(data=clamp_i16(self.esc_stop)))
            self.pub_s1.publish(Int16(data=clamp_i16(self.s1_center)))
            return

        # choose source by mode + timeout
        if self.mode == "MANUAL":
            esc_ok = (now - self.man_t_esc) <= self.timeout_manual_s
            s1_ok  = (now - self.man_t_s1)  <= self.timeout_manual_s
            esc = self.man_esc if esc_ok else self.esc_stop
            s1  = self.man_s1  if s1_ok  else self.s1_center

        elif self.mode == "AUTO":
            esc_ok = (now - self.auto_t_esc) <= self.timeout_auto_s
            s1_ok  = (now - self.auto_t_s1)  <= self.timeout_auto_s
            esc = self.auto_esc if esc_ok else self.esc_stop
            s1  = self.auto_s1  if s1_ok  else self.s1_center

        else:  # "AUTO+"
            esc_ok = (now - self.ap_t_esc) <= self.timeout_autoplus_s
            s1_ok  = (now - self.ap_t_s1)  <= self.timeout_autoplus_s
            esc = self.ap_esc if esc_ok else self.esc_stop
            s1  = self.ap_s1  if s1_ok  else self.s1_center

        self.pub_esc.publish(Int16(data=clamp_i16(esc)))
        self.pub_s1.publish(Int16(data=clamp_i16(s1)))


def main():
    rclpy.init()
    node = CmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

