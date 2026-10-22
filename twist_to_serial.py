#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 Twist -> 当前小车串口单字符协议适配器。"""
from __future__ import annotations

import glob
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from front_scan_safety import front_clearance


CMDS = {
    "w": bytes([0x55, 0x55, 0x03, 0x09, 0x80]),
    "x": bytes([0x55, 0x55, 0x03, 0x0A, 0x80]),
    "l": bytes([0x55, 0x55, 0x03, 0x0D, 0x80]),
    "r": bytes([0x55, 0x55, 0x03, 0x0E, 0x80]),
    "s": bytes([0x55, 0x55, 0x03, 0x11, 0x80]),
}


class TwistSerial(Node):
    def __init__(self):
        super().__init__("twist_to_car_serial")
        self.declare_parameter("topic", "/cmd_vel")
        self.declare_parameter("port", os.environ.get("CAR_MOTOR_SERIAL", ""))
        self.declare_parameter("baud", 9600)
        self.declare_parameter("deadband", 0.03)
        self.declare_parameter("turn_command_threshold", 0.20)
        self.declare_parameter("motion_switch_brake_sec", 0.08)
        self.declare_parameter("command_timeout", 0.6)
        self.declare_parameter("linear_speed_mps", float(os.environ.get("CAR_LINEAR_SPEED_MPS", "0.08")))
        self.declare_parameter("rotate_deg_per_sec", float(os.environ.get("CAR_ROT_DEG_PER_SEC", "44.0")))
        self.declare_parameter("min_turn_radius", 0.32)
        self.declare_parameter("allow_backward", False)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_timeout", 0.35)
        # Laser-origin clearance: body extents are front 0.30 m, rear 0.25 m,
        # sides 0.20 m, with a 0.05 m safety margin.
        self.declare_parameter("front_stop_distance", 0.35)
        self.declare_parameter("front_clear_distance", 0.45)
        self.declare_parameter("rear_stop_distance", 0.30)
        self.declare_parameter("rear_clear_distance", 0.40)
        self.declare_parameter("side_stop_distance", 0.25)
        self.declare_parameter("side_clear_distance", 0.35)
        self.declare_parameter("spin_clearance_distance", 0.42)
        self.declare_parameter("front_sector_half_angle_deg", 20.0)
        self.lock = threading.Lock()
        self.last_key = "s"
        self.last_command = 0.0
        self.scan_received_at = 0.0
        self.front_clearance_m = None
        self.rear_clearance_m = None
        self.left_clearance_m = None
        self.right_clearance_m = None
        self.spin_clearance_m = None
        self.scan_usable = False
        self.front_scan_usable = False
        self.front_blocked = False
        self.rear_blocked = False
        self.left_blocked = False
        self.right_blocked = False
        self.spin_blocked = True
        self.safety_stop_active = False
        self.motion_pub = self.create_publisher(Twist, "/base_motion", 10)
        self.create_subscription(Twist, self.get_parameter("topic").value, self.on_twist, 10)
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.1, self.watchdog)
        self.get_logger().warning(
            "Twist 串口桥已启动；线速度 %.3f m/s 为开环初始估值，旋转标定 %.1f 度/秒"
            % (
                float(self.get_parameter("linear_speed_mps").value),
                float(self.get_parameter("rotate_deg_per_sec").value),
            )
        )

    def port_candidates(self):
        configured = str(self.get_parameter("port").value or "").strip()
        return [configured] if configured else [
            "/dev/ttyAMA1", "/dev/ttyAMA4", "/dev/ttyS0", "/dev/serial0",
            *sorted(glob.glob("/dev/ttyUSB*")), *sorted(glob.glob("/dev/ttyACM*")),
        ]

    def send(self, key):
        if key == self.last_key:
            return True
        try:
            import serial
            with self.lock:
                port = next((p for p in self.port_candidates() if os.path.exists(p)), None)
                if not port:
                    raise RuntimeError("未找到小车串口")
                # Open only for the short write so the GUI can share this UART.
                with serial.Serial(
                    port, baudrate=int(self.get_parameter("baud").value), timeout=0.2
                ) as connection:
                    if key != "s" and self.last_key not in ("s", key):
                        connection.write(CMDS["s"])
                        connection.flush()
                        self.last_key = "s"
                        time.sleep(float(self.get_parameter("motion_switch_brake_sec").value))
                    connection.write(CMDS[key])
                    connection.flush()
                self.last_key = key
            return True
        except Exception as exc:
            self.get_logger().error("串口发送 %s 失败: %s" % (key, exc))
            return False

    def _key_for_twist(self, msg):
        deadband = float(self.get_parameter("deadband").value)
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)
        turn_threshold = float(self.get_parameter("turn_command_threshold").value)
        # The chassis protocol cannot command translation and yaw together.
        # Give meaningful yaw requests priority and pivot in place; this also
        # keeps the published open-loop odometry consistent with the command.
        if abs(wz) >= turn_threshold:
            return "l" if wz > 0 else "r"
        if vx > deadband:
            return "w"
        if vx < -deadband and self.get_parameter("allow_backward").value:
            return "x"
        return "s"

    def _publish_base_motion(self, key):
        motion = Twist()
        if key == "w":
            motion.linear.x = float(self.get_parameter("linear_speed_mps").value)
        elif key == "x":
            motion.linear.x = -float(self.get_parameter("linear_speed_mps").value)
        elif key in ("l", "r"):
            rate = math.radians(float(self.get_parameter("rotate_deg_per_sec").value))
            motion.angular.z = rate if key == "l" else -rate
        self.motion_pub.publish(motion)

    def on_scan(self, scan):
        usable, nearest = front_clearance(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            math.radians(float(self.get_parameter("front_sector_half_angle_deg").value)),
        )
        self.scan_received_at = time.monotonic()
        self.scan_usable = any(
            math.isfinite(distance) and scan.range_min <= distance <= scan.range_max
            for distance in scan.ranges
        )
        self.front_scan_usable = usable
        self.front_clearance_m = nearest
        self.rear_clearance_m = self._sector_clearance(scan, math.pi)
        self.left_clearance_m = self._sector_clearance(scan, math.pi / 2.0)
        self.right_clearance_m = self._sector_clearance(scan, -math.pi / 2.0)
        valid_ranges = [
            float(distance)
            for distance in scan.ranges
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max
        ]
        self.spin_clearance_m = min(valid_ranges) if valid_ranges else None

        stop_distance = float(self.get_parameter("front_stop_distance").value)
        clear_distance = float(self.get_parameter("front_clear_distance").value)
        was_blocked = self.front_blocked
        if not usable:
            self.front_blocked = True
        elif nearest is not None and nearest < stop_distance:
            self.front_blocked = True
        elif nearest is None or nearest >= clear_distance:
            self.front_blocked = False

        if self.front_blocked and not was_blocked:
            distance = "无有效前向距离" if nearest is None else "%.2f m" % nearest
            self.get_logger().warning("前向激光安全停: %s" % distance)
            self._force_stop()
        elif was_blocked and not self.front_blocked:
            self.get_logger().info("前向障碍解除，距离已超过 %.2f m" % clear_distance)

        self.rear_blocked = self._update_blocked(
            self.rear_blocked,
            self.rear_clearance_m,
            float(self.get_parameter("rear_stop_distance").value),
            float(self.get_parameter("rear_clear_distance").value),
        )
        self.left_blocked = self._update_blocked(
            self.left_blocked,
            self.left_clearance_m,
            float(self.get_parameter("side_stop_distance").value),
            float(self.get_parameter("side_clear_distance").value),
        )
        self.right_blocked = self._update_blocked(
            self.right_blocked,
            self.right_clearance_m,
            float(self.get_parameter("side_stop_distance").value),
            float(self.get_parameter("side_clear_distance").value),
        )
        spin_limit = float(self.get_parameter("spin_clearance_distance").value)
        self.spin_blocked = self.spin_clearance_m is None or self.spin_clearance_m < spin_limit

        active_blocked = {
            "w": self.front_blocked,
            "x": self.rear_blocked,
            "l": self.spin_blocked,
            "r": self.spin_blocked,
        }.get(self.last_key, False)
        if active_blocked:
            self.get_logger().warning(
                "運動中安全區受阻（方向=%s，最近=%s），立即停車"
                % (self.last_key, self._distance_text(self._active_clearance(self.last_key)))
            )
            self._force_stop()

    def _active_clearance(self, key):
        if key == "w":
            return self.front_clearance_m
        if key == "x":
            return self.rear_clearance_m
        return self.spin_clearance_m

    @staticmethod
    def _sector_clearance(scan, center_angle, half_angle=math.radians(25.0)):
        nearest = None
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            delta = math.atan2(math.sin(angle - center_angle), math.cos(angle - center_angle))
            if abs(delta) <= half_angle and (nearest is None or distance < nearest):
                nearest = float(distance)
        return nearest

    @staticmethod
    def _update_blocked(blocked, distance, stop_distance, clear_distance):
        if distance is None:
            return True
        if distance < stop_distance:
            return True
        if distance >= clear_distance:
            return False
        return blocked

    @staticmethod
    def _distance_text(distance):
        return "无有效距离" if distance is None else "%.2f m" % distance

    def _force_stop(self):
        if self.send("s"):
            self._publish_base_motion("s")
            self.last_command = time.time()
            self.safety_stop_active = True

    def on_twist(self, msg):
        key = self._key_for_twist(msg)
        if key in ("w", "x", "l", "r"):
            timeout = float(self.get_parameter("scan_timeout").value)
            scan_age = time.monotonic() - self.scan_received_at
            blocked = {
                "w": self.front_blocked,
                "x": self.rear_blocked,
                "l": self.spin_blocked,
                "r": self.spin_blocked,
            }[key]
            distance = self._active_clearance(key)
            if (
                not self.scan_usable
                or scan_age > timeout
                or blocked
            ):
                if not self.safety_stop_active:
                    if scan_age > timeout:
                        reason = "激光数据过期 %.2f s" % scan_age
                    elif not self.scan_usable:
                        reason = "激光扫描数据无效"
                    else:
                        direction = {"w": "前方", "x": "后方", "l": "自旋扫掠区", "r": "自旋扫掠区"}[key]
                        reason = "%s障碍 %s" % (direction, self._distance_text(distance))
                    action = {"w": "前进", "x": "后退", "l": "左自旋", "r": "右自旋"}[key]
                    self.get_logger().warning("禁止%s: %s" % (action, reason))
                self._force_stop()
                return
        self.safety_stop_active = False
        sent = self.send(key)
        self._publish_base_motion(key if sent else "s")
        self.last_command = time.time()

    def watchdog(self):
        timeout = float(self.get_parameter("command_timeout").value)
        if self.last_key != "s" and time.time() - self.last_command > timeout:
            if self.send("s"):
                self._publish_base_motion("s")

    def destroy_node(self):
        self.send("s")
        self._publish_base_motion("s")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TwistSerial()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
