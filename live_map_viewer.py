#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Display the live Cartographer OccupancyGrid without saving snapshots."""
from __future__ import annotations

import math
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

try:
    from PIL import Image, ImageTk
except ImportError:
    Image = ImageTk = None


class MapSubscriber(Node):
    def __init__(self):
        super().__init__("car_live_map_viewer")
        self.latest_map = None
        self.received_at = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)

    def on_map(self, msg):
        self.latest_map = msg
        self.received_at = time.monotonic()

    def robot_pose(self, map_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                map_frame, "base_footprint", rclpy.time.Time()
            ).transform
            p, q = transform.translation, transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return p.x, p.y, yaw
        except Exception:
            return None


class LiveMapWindow:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.last_rendered = None
        self.last_render_time = 0.0
        self.photo = None
        self.root.title("实时 SLAM 地图")
        self.root.geometry("1000x760")
        self.root.minsize(600, 450)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.status = tk.StringVar(value="等待 Cartographer 发布 /map…")
        ttk.Label(root, textvariable=self.status, anchor="w").pack(fill="x", padx=8, pady=6)
        host = ttk.Frame(root)
        host.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(host, bg="#252a2d", highlightthickness=0)
        xbar = ttk.Scrollbar(host, orient="horizontal", command=self.canvas.xview)
        ybar = ttk.Scrollbar(host, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(xscrollcommand=xbar.set, yscrollcommand=ybar.set)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        ybar.grid(row=0, column=1, sticky="ns")
        xbar.grid(row=1, column=0, sticky="ew")
        host.rowconfigure(0, weight=1)
        host.columnconfigure(0, weight=1)
        self.root.after(20, self.pump_ros)
        self.root.after(250, self.refresh)

    def pump_ros(self):
        if not rclpy.ok():
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:
            self.status.set("ROS 地图订阅异常: %s" % exc)
        self.root.after(20, self.pump_ros)

    def refresh(self):
        msg = self.node.latest_map
        if msg is None:
            age = time.monotonic() - self.node.received_at if self.node.received_at else None
            if age is None:
                self.status.set("等待 /map；请先启动激光建图")
            self.root.after(250, self.refresh)
            return
        now = time.monotonic()
        if msg is not self.last_rendered and now - self.last_render_time >= 0.5:
            self.render_map(msg)
            self.last_rendered = msg
            self.last_render_time = now
        age = time.monotonic() - self.node.received_at
        freshness = "实时更新" if age < 1.5 else "地图停更 %.1f 秒" % age
        resolution = msg.info.resolution
        self.status.set(
            "%s | %s | %d x %d 格 | %.0f mm/格"
            % (freshness, msg.header.frame_id or "map", msg.info.width,
               msg.info.height, resolution * 1000.0)
        )
        self.root.after(250, self.refresh)

    def render_map(self, msg):
        width, height = msg.info.width, msg.info.height
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            self.status.set("收到无效地图栅格")
            return
        if Image is None or ImageTk is None:
            self.status.set("缺少 Pillow，无法渲染实时地图")
            return
        pixels = bytes(
            90 if value < 0 else max(20, 245 - int(value) * 2)
            for value in msg.data
        )
        image = Image.frombytes("L", (width, height), pixels)
        image = image.transpose(Image.FLIP_TOP_BOTTOM).convert("RGB")
        self.photo = ImageTk.PhotoImage(image)
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, image=self.photo, anchor="nw")
        self.canvas.configure(scrollregion=(0, 0, width, height))
        self.draw_robot(msg, width, height)

    def draw_robot(self, msg, width, height):
        pose = self.node.robot_pose(msg.header.frame_id or "map")
        if pose is None:
            return
        x, y, yaw = pose
        origin = msg.info.origin
        oq = origin.orientation
        origin_yaw = math.atan2(
            2.0 * (oq.w * oq.z + oq.x * oq.y),
            1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z),
        )
        dx, dy = x - origin.position.x, y - origin.position.y
        local_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
        local_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
        col = local_x / msg.info.resolution
        row = local_y / msg.info.resolution
        cx, cy = col + 0.5, height - row - 0.5
        heading = yaw - origin_yaw
        length = 7.0
        tip = (cx + length * math.cos(heading), cy - length * math.sin(heading))
        left = (cx + 4 * math.cos(heading + 2.5), cy - 4 * math.sin(heading + 2.5))
        right = (cx + 4 * math.cos(heading - 2.5), cy - 4 * math.sin(heading - 2.5))
        self.canvas.create_polygon(*tip, *left, *right, fill="#e53935", outline="white", width=1)

    def close(self):
        try:
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            self.root.destroy()


def main():
    rclpy.init()
    node = MapSubscriber()
    root = tk.Tk()
    LiveMapWindow(root, node)
    root.mainloop()


if __name__ == "__main__":
    main()
