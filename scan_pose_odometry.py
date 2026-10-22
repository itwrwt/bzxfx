#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish Nav2 odometry from Cartographer's laser-matched odom TF."""
from __future__ import annotations

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from scan_odometry_math import wrapped_angle_delta


class ScanPoseOdometry(Node):
    def __init__(self):
        super().__init__("scan_pose_odometry")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.previous = None
        self.last_warning = 0.0
        self.create_timer(0.05, self.publish_odom)

    def publish_odom(self):
        odom_frame = str(self.get_parameter("odom_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        try:
            transform = self.tf_buffer.lookup_transform(
                odom_frame, base_frame, rclpy.time.Time()
            ).transform
        except Exception as exc:
            if time.monotonic() - self.last_warning > 3.0:
                self.get_logger().warning("等待激光 SLAM 位姿 TF: %s" % exc)
                self.last_warning = time.monotonic()
            return

        x = float(transform.translation.x)
        y = float(transform.translation.y)
        q = transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        now = time.monotonic()
        vx = vy = wz = 0.0
        if self.previous is not None:
            last_time, last_x, last_y, last_yaw = self.previous
            dt = now - last_time
            if 0.01 <= dt <= 0.25:
                dx = (x - last_x) / dt
                dy = (y - last_y) / dt
                vx = math.cos(yaw) * dx + math.sin(yaw) * dy
                vy = -math.sin(yaw) * dx + math.cos(yaw) * dy
                wz = wrapped_angle_delta(yaw, last_yaw) / dt
        self.previous = (now, x, y, yaw)

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = odom_frame
        msg.child_frame_id = base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = q
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.05
        msg.twist.covariance[0] = 0.05
        msg.twist.covariance[7] = 0.05
        msg.twist.covariance[35] = 0.12
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanPoseOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
