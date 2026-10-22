"""Small ROS 2 bridge for the YDLIDAR X3 Pro used by Gui.py.

The ROS driver owns the serial device.  This module only starts/stops that
driver and subscribes to /scan for the Tk UI, keeping ROS callbacks off the
Tk main thread.
"""
import math
import os
import signal
import subprocess
import threading
import time


class RadarBridge:
    def __init__(self, workspace="/home/haha3/ydlidar_ws"):
        self.workspace = workspace
        self.process = None
        self.lock = threading.Lock()
        self.running = False
        self.status = "未启动"
        self.last_scan = []
        self.last_scan_time = 0.0
        self._ros_thread = None
        self._node = None
        self._rclpy = None

    def _ensure_ros_subscriber(self):
        if self._ros_thread and self._ros_thread.is_alive():
            return
        self._ros_thread = threading.Thread(target=self._ros_loop, daemon=True)
        self._ros_thread.start()

    @staticmethod
    def _driver_running():
        """Return true when another UI/mapping session already owns YDLIDAR."""
        try:
            result = subprocess.run(
                ["pgrep", "-f", "[y]dlidar_ros2_driver_node"],
                capture_output=True,
                text=True,
                timeout=1.0,
                check=False,
            )
            return bool(result.stdout.strip())
        except (OSError, subprocess.TimeoutExpired):
            return False

    def _ros_loop(self):
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import LaserScan
            self._rclpy = rclpy
            rclpy.init(args=None)

            bridge = self

            class ScanNode(Node):
                def __init__(self):
                    super().__init__("car_ui_scan_view")
                    self.create_subscription(LaserScan, "/scan", self.on_scan, 10)

                def on_scan(self, msg):
                    points = []
                    for i, distance in enumerate(msg.ranges):
                        if not math.isfinite(distance):
                            continue
                        if distance < max(msg.range_min, 0.03) or distance > msg.range_max:
                            continue
                        angle = -(msg.angle_min + i * msg.angle_increment)
                        points.append((angle, distance))
                    with bridge.lock:
                        bridge.last_scan = points
                        bridge.last_scan_time = time.time()
                        bridge.status = "扫描中 (%d点)" % len(points)

            self._node = ScanNode()
            rclpy.spin(self._node)
        except Exception as exc:
            with self.lock:
                self.status = "ROS不可用: %s" % exc
        finally:
            try:
                if self._node is not None:
                    self._node.destroy_node()
                if self._rclpy is not None and self._rclpy.ok():
                    self._rclpy.shutdown()
            except Exception:
                pass

    def start(self):
        if self.process is not None and self.process.poll() is None:
            return True, "雷达已经运行"
        if not os.path.exists(self.workspace):
            self.status = "ROS工作区不存在: %s" % self.workspace
            return False, self.status
        if self._driver_running():
            # Mapping owns the serial port; this UI only joins its /scan topic.
            self.running = True
            self.status = "复用建图中的雷达数据"
            self._ensure_ros_subscriber()
            return True, self.status
        command = (
            "source /opt/ros/humble/setup.bash && "
            "source %s/install/setup.bash && "
            "ros2 launch ydlidar_ros2_driver x3_ydlidar_launch.py"
        ) % self.workspace
        try:
            self.process = subprocess.Popen(
                ["bash", "-lc", command],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
            )
            self.running = True
            self.status = "启动中，等待雷达数据"
            self._ensure_ros_subscriber()
            threading.Thread(target=self._read_driver_log, daemon=True).start()
            return True, self.status
        except Exception as exc:
            self.status = "雷达启动失败: %s" % exc
            return False, self.status

    def _read_driver_log(self):
        process = self.process
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            line = line.strip()
            if not line:
                continue
            with self.lock:
                if "error" in line.lower() or "failed" in line.lower():
                    self.status = "驱动错误: " + line[-180:]
                elif "scan" in line.lower() or "lidar" in line.lower():
                    self.status = line[-180:]
        with self.lock:
            if self.running:
                self.status = "雷达驱动已退出"

    def stop(self):
        process = self.process
        self.running = False
        if process is not None and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=3)
            except Exception:
                try:
                    process.kill()
                except Exception:
                    pass
        self.process = None
        with self.lock:
            self.status = "已停止"
            self.last_scan = []

    def snapshot(self):
        with self.lock:
            return list(self.last_scan), self.last_scan_time, self.status

    def close(self):
        self.stop()
        try:
            if self._node is not None:
                self._node.destroy_node()
            if self._rclpy is not None and self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass
