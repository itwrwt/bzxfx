#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Select safe frontier goals and delegate path following to Nav2."""
from __future__ import annotations

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener


class FrontierNavigator(Node):
    def __init__(self):
        super().__init__("car_frontier_navigator")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("min_frontier_cells", 8)
        self.declare_parameter("free_threshold", 50)
        self.declare_parameter("max_goal_distance", 5.0)
        self.declare_parameter("goal_tolerance", 0.3)
        self.declare_parameter("failed_goal_cooldown", 20.0)
        self.map_msg = None
        self.goal = None
        self.goal_handle = None
        self.goal_pending = False
        self.failed_goals = []
        self.last_log = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value, self.on_map, qos
        )
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_timer(1.0, self.explore)
        self.get_logger().info("前沿目标发送器已启动，等待 Nav2 与地图")

    def on_map(self, msg):
        self.map_msg = msg

    def pose(self):
        try:
            frame = self.map_msg.header.frame_id or "map"
            tf = self.tf_buffer.lookup_transform(
                frame, self.get_parameter("base_frame").value, rclpy.time.Time()
            )
            p, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return p.x, p.y, yaw
        except Exception:
            return None

    def grid_to_world(self, col, row):
        info = self.map_msg.info
        ox, oy = info.origin.position.x, info.origin.position.y
        q = info.origin.orientation
        origin_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        lx = (col + 0.5) * info.resolution
        ly = (row + 0.5) * info.resolution
        return (
            ox + lx * math.cos(origin_yaw) - ly * math.sin(origin_yaw),
            oy + lx * math.sin(origin_yaw) + ly * math.cos(origin_yaw),
        )

    def frontier_goals(self, pose):
        msg = self.map_msg
        width, height = msg.info.width, msg.info.height
        if width < 3 or height < 3:
            return []
        data = msg.data
        free_limit = int(self.get_parameter("free_threshold").value)

        def is_free(row, col):
            if row < 0 or col < 0 or row >= height or col >= width:
                return False
            value = data[row * width + col]
            return 0 <= value <= free_limit

        frontier = set()
        for row in range(1, height - 1):
            start = row * width
            for col in range(1, width - 1):
                if data[start + col] < 0 and (
                    is_free(row - 1, col) or is_free(row + 1, col)
                    or is_free(row, col - 1) or is_free(row, col + 1)
                ):
                    frontier.add((row, col))

        clusters = []
        while frontier:
            seed = frontier.pop()
            cluster = [seed]
            queue = deque([seed])
            while queue:
                row, col = queue.popleft()
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        point = row + dr, col + dc
                        if point in frontier:
                            frontier.remove(point)
                            queue.append(point)
                            cluster.append(point)
            if len(cluster) < int(self.get_parameter("min_frontier_cells").value):
                continue

            # Navigate to the known-free side of the frontier, not into unknown cells.
            safe_cells = set()
            for row, col in cluster:
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    if is_free(row + dr, col + dc):
                        safe_cells.add((row + dr, col + dc))
            if not safe_cells:
                continue
            col = sum(cell[1] for cell in safe_cells) / len(safe_cells)
            row = sum(cell[0] for cell in safe_cells) / len(safe_cells)
            x, y = self.grid_to_world(col, row)
            distance = math.hypot(x - pose[0], y - pose[1])
            if distance < 0.35 or distance > float(
                self.get_parameter("max_goal_distance").value
            ):
                continue
            cooldown = float(self.get_parameter("failed_goal_cooldown").value)
            if any(
                time.monotonic() - failed_at < cooldown
                and math.hypot(x - gx, y - gy) < 0.45
                for gx, gy, failed_at in self.failed_goals
            ):
                continue
            clusters.append((len(cluster) / (distance + 0.5), x, y))
        return clusters

    def send_goal(self, x, y, pose):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_msg.header.frame_id or "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        target_yaw = math.atan2(y - pose[1], x - pose[0])
        heading = target_yaw if math.isfinite(target_yaw) else pose[2]
        goal.pose.pose.orientation.z = math.sin(heading / 2.0)
        goal.pose.pose.orientation.w = math.cos(heading / 2.0)
        self.goal = (x, y)
        self.goal_pending = True
        future = self.nav.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)
        self.get_logger().info("向 Nav2 发送前沿目标: %.2f, %.2f" % (x, y))

    def on_feedback(self, _feedback):
        pass

    def on_goal_response(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self.goal_handle = None
            self.goal_pending = False
            self.get_logger().error("Nav2 目标发送失败: %s" % exc)
            return
        if not self.goal_handle.accepted:
            self.failed_goals.append((*self.goal, time.monotonic()))
            self.failed_goals = self.failed_goals[-20:]
            self.goal_pending = False
            self.get_logger().warning("Nav2 拒绝前沿目标")
            return
        self.goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            status = -1
            self.get_logger().warning("Nav2 导航结果异常: %s" % exc)
        if status != GoalStatus.STATUS_SUCCEEDED and self.goal is not None:
            self.failed_goals.append((*self.goal, time.monotonic()))
            self.failed_goals = self.failed_goals[-20:]
        self.get_logger().info("Nav2 前沿目标结束，状态=%s" % status)
        self.goal = None
        self.goal_handle = None
        self.goal_pending = False

    def explore(self):
        if self.map_msg is None or self.goal_pending:
            return
        pose = self.pose()
        if pose is None:
            if time.time() - self.last_log > 4.0:
                self.get_logger().warning("等待 map -> base_footprint 位姿")
                self.last_log = time.time()
            return
        if not self.nav.server_is_ready():
            if time.time() - self.last_log > 4.0:
                self.get_logger().warning("等待 Nav2 navigate_to_pose action")
                self.last_log = time.time()
            return
        goals = self.frontier_goals(pose)
        if not goals:
            if time.time() - self.last_log > 4.0:
                self.get_logger().info("当前没有可导航前沿，等待地图更新")
                self.last_log = time.time()
            return
        _, x, y = max(goals)
        self.send_goal(x, y, pose)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
