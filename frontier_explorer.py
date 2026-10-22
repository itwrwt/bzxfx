#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轻量 ROS2 前沿探索器。

它借鉴 explore_lite 的前沿点聚类/评分，但不依赖 Nav2。节点从 /map
找未知区与已知空闲区的边界，使用 map->base_footprint 位姿做一个保守的
目标跟踪器，并把速度发到 /cmd_vel。真正的底盘协议由 twist_to_serial.py
负责转换，方便替换成当前小车的串口桥。
"""
from __future__ import annotations

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("car_frontier_explorer")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("min_frontier_cells", 8)
        self.declare_parameter("free_threshold", 50)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("max_goal_distance", 5.0)
        self.declare_parameter("linear_speed", 0.12)
        self.declare_parameter("angular_speed", 0.55)
        self.declare_parameter("obstacle_distance", 0.42)

        self.map_msg = None
        self.scan_msg = None
        self.goal = None
        self.last_goal_time = 0.0
        self.failed_goals = []
        self.last_log = 0.0
        self.last_frontier_count = 0
        self.turn_pulse_until = 0.0
        self.turn_cooldown_until = 0.0
        self.last_turn_direction = 1.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value, self.on_map, 3
        )
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value, self.on_scan, 5
        )
        self.timer = self.create_timer(0.2, self.control_loop)
        self.get_logger().info("前沿探索器已启动，等待 /map 和位姿")

    def on_map(self, msg):
        self.map_msg = msg

    def on_scan(self, msg):
        self.scan_msg = msg

    def pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_msg.header.frame_id or "map",
                self.get_parameter("base_frame").value,
                rclpy.time.Time(),
            )
            p = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return float(p.x), float(p.y), yaw
        except Exception:
            return None

    def free(self, data, width, row, col):
        if row < 0 or col < 0 or row >= len(data) // width or col >= width:
            return False
        threshold = int(self.get_parameter("free_threshold").value)
        value = data[row * width + col]
        return 0 <= value <= threshold

    def frontier_goals(self):
        msg = self.map_msg
        if msg is None or msg.info.width < 3 or msg.info.height < 3:
            self.last_frontier_count = 0
            return []
        width, height = msg.info.width, msg.info.height
        data = list(msg.data)
        threshold = int(self.get_parameter("free_threshold").value)
        frontiers = set()
        for row in range(1, height - 1):
            for col in range(1, width - 1):
                idx = row * width + col
                if data[idx] < 0:
                    neighbors = (
                        data[idx - 1], data[idx + 1],
                        data[idx - width], data[idx + width],
                    )
                    if any(0 <= value <= threshold for value in neighbors):
                        frontiers.add((row, col))
        clusters = []
        while frontiers:
            seed = frontiers.pop()
            cluster = [seed]
            queue = deque([seed])
            while queue:
                row, col = queue.popleft()
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        point = (row + dr, col + dc)
                        if point in frontiers:
                            frontiers.remove(point)
                            queue.append(point)
                            cluster.append(point)
            minimum = int(self.get_parameter("min_frontier_cells").value)
            if len(cluster) >= minimum:
                row = sum(p[0] for p in cluster) / len(cluster)
                col = sum(p[1] for p in cluster) / len(cluster)
                x = msg.info.origin.position.x + (col + 0.5) * msg.info.resolution
                # OccupancyGrid rows increase from the map origin toward +Y.
                y = msg.info.origin.position.y + (row + 0.5) * msg.info.resolution
                clusters.append((x, y, len(cluster)))
        self.last_frontier_count = len(clusters)
        return clusters

    def choose_goal(self, pose):
        px, py, _ = pose
        max_distance = float(self.get_parameter("max_goal_distance").value)
        candidates = []
        for x, y, size in self.frontier_goals():
            distance = math.hypot(x - px, y - py)
            if distance > max_distance or distance < 0.35:
                continue
            if any(math.hypot(x - gx, y - gy) < 0.45 for gx, gy in self.failed_goals):
                continue
            # 信息增益优先，同时轻微偏向较近目标。
            score = float(size) / (distance + 0.5)
            candidates.append((score, x, y))
        if not candidates:
            return None
        _, x, y = max(candidates)
        return x, y

    def front_blocked(self):
        if self.scan_msg is None:
            return False
        values = []
        for i, distance in enumerate(self.scan_msg.ranges):
            angle = self.scan_msg.angle_min + i * self.scan_msg.angle_increment
            if abs(angle) < math.radians(25) and math.isfinite(distance):
                values.append(distance)
        return bool(values) and min(values) < float(self.get_parameter("obstacle_distance").value)

    def publish(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def pulse_turn(self, direction):
        """Pulse the fixed-speed serial turn command, then stop for localization."""
        now = time.monotonic()
        if now < self.turn_pulse_until:
            self.publish(angular=direction)
            return
        if now < self.turn_cooldown_until:
            self.publish()
            return
        self.last_turn_direction = math.copysign(1.0, direction)
        self.turn_pulse_until = now + 0.16
        self.turn_cooldown_until = now + 0.65
        self.publish(angular=self.last_turn_direction * float(
            self.get_parameter("angular_speed").value
        ))

    def control_loop(self):
        if self.map_msg is None:
            self.publish()
            if time.time() - self.last_log > 3:
                self.get_logger().warning("等待 /map 栅格，检查 Cartographer 和 /scan")
                self.last_log = time.time()
            return
        current = self.pose()
        if current is None:
            self.publish()
            if time.time() - self.last_log > 3:
                self.get_logger().warning("地图已收到，但 map 到 base_footprint TF 尚不可用")
                self.last_log = time.time()
            return
        px, py, yaw = current
        if self.goal is None:
            self.goal = self.choose_goal(current)
            if self.goal is None:
                self.publish()
                if time.time() - self.last_log > 3:
                    self.get_logger().warning(
                        "没有可达前沿: clusters=%d pose=(%.2f, %.2f) map=%dx%d; "
                        "确认地图周围有已知空闲格和未知区边界" % (
                            self.last_frontier_count, px, py,
                            self.map_msg.info.width, self.map_msg.info.height,
                        )
                    )
                    self.last_log = time.time()
                return
            self.last_goal_time = time.time()
            self.get_logger().info("选择前沿目标: %.2f, %.2f" % self.goal)

        gx, gy = self.goal
        distance = math.hypot(gx - px, gy - py)
        if distance < float(self.get_parameter("goal_tolerance").value):
            self.get_logger().info("到达前沿目标，重新选择")
            self.goal = None
            self.publish()
            return
        if time.time() - self.last_goal_time > 25.0:
            self.failed_goals.append(self.goal)
            self.failed_goals = self.failed_goals[-12:]
            self.get_logger().warning("目标超时，重新选择前沿点")
            self.goal = None
            self.publish()
            return
        if self.front_blocked():
            self.pulse_turn(-self.last_turn_direction)
            return
        target = math.atan2(gy - py, gx - px)
        error = (target - yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(error) > 0.25:
            angular = math.copysign(float(self.get_parameter("angular_speed").value), error)
            self.pulse_turn(angular)
        else:
            self.turn_pulse_until = 0.0
            self.turn_cooldown_until = 0.0
            self.publish(linear=float(self.get_parameter("linear_speed").value))


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.publish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
