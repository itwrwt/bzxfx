#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Integrate calibrated, actually dispatched base motion into odom and TF."""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class CommandOdometry(Node):
    def __init__(self):
        super().__init__("command_odometry")
        self.declare_parameter("base_motion_topic", "/base_motion")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("command_timeout", 0.65)
        self.declare_parameter("publish_tf", True)
        self.vx = 0.0
        self.wz = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last = time.monotonic()
        self.last_command = self.last
        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_topic").value, 10
        )
        self.create_subscription(
            Twist, self.get_parameter("base_motion_topic").value, self.on_twist, 10
        )
        self.create_timer(0.05, self.update)

    def on_twist(self, msg):
        self.vx = float(msg.linear.x)
        self.wz = float(msg.angular.z)
        self.last_command = time.monotonic()

    def update(self):
        now = time.monotonic()
        dt = min(0.2, max(0.0, now - self.last))
        self.last = now
        if now - self.last_command > float(self.get_parameter("command_timeout").value):
            self.vx = 0.0
            self.wz = 0.0
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt
        self.yaw += self.wz * dt
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter("odom_frame").value
        t.child_frame_id = self.get_parameter("base_frame").value
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        if self.get_parameter("publish_tf").value:
            self.tf.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = t.header.frame_id
        odom.child_frame_id = t.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        odom.pose.covariance[0] = 0.08
        odom.pose.covariance[7] = 0.08
        odom.pose.covariance[35] = 0.12
        odom.twist.covariance[0] = 0.08
        odom.twist.covariance[35] = 0.12
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = CommandOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
