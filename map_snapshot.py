#!/usr/bin/env python3
"""Save the latest transient-local OccupancyGrid without nav2_map_saver_cli."""
from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message

CACHE = Path(os.environ.get("CAR_MAPPING_STATE_DIR", str(Path.home() / ".car_mapping"))) / "latest_map.cdr"


class MapSnapshot(Node):
    def __init__(self, watch=False):
        super().__init__("car_map_snapshot")
        self.message = None
        self.watch = watch
        self.last_cache_write = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)

    def on_map(self, message):
        if message.info.width and message.info.height and message.data:
            self.message = message
            now = time.monotonic()
            if self.watch and now - self.last_cache_write >= 5.0:
                CACHE.parent.mkdir(parents=True, exist_ok=True)
                temporary = CACHE.with_suffix(".tmp")
                try:
                    temporary.write_bytes(serialize_message(message))
                    temporary.replace(CACHE)
                    self.last_cache_write = now
                except OSError as exc:
                    self.get_logger().error("无法更新最近地图快照: %s" % exc)


def save_map(message: OccupancyGrid, prefix: Path) -> None:
    info = message.info
    width, height = int(info.width), int(info.height)
    values = list(message.data)
    pixels = bytearray()
    for row in range(height - 1, -1, -1):
        for col in range(width):
            value = int(values[row * width + col])
            if value < 0:
                pixel = 205
            elif value >= 65:
                pixel = 0
            elif value <= 19:
                pixel = 254
            else:
                pixel = 205
            pixels.append(pixel)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    pgm = prefix.with_suffix(".pgm")
    yaml = prefix.with_suffix(".yaml")
    pgm.write_bytes(b"P5\n%d %d\n255\n" % (width, height) + bytes(pixels))
    q = info.origin.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    yaml.write_text(
        "image: %s\nresolution: %.9f\norigin: [%.9f, %.9f, %.9f]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
        % (pgm.name, info.resolution, info.origin.position.x,
           info.origin.position.y, yaw), encoding="utf-8")


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: map_snapshot.py MAP_PREFIX", file=sys.stderr)
        return 2
    rclpy.init()
    watch = sys.argv[1] == "watch"
    node = MapSnapshot(watch)
    try:
        if watch:
            rclpy.spin(node)
            return 0
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and node.message is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
        if node.message is None:
            if not CACHE.exists():
                print("未收到 /map，且本次会话没有地图快照；不能保存空地图", file=sys.stderr)
                return 1
            node.message = deserialize_message(CACHE.read_bytes(), OccupancyGrid)
            print("实时地图不可用，保存本次会话最后快照（%.1f秒前）；未包含停更后的区域"
                  % max(0, time.time() - CACHE.stat().st_mtime))
        save_map(node.message, Path(sys.argv[1]))
        print("已抓取 /map 当前帧并保存")
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
