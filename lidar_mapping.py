#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""树莓派本地 2D 激光建图会话控制器。

用法（在树莓派上运行）：
    python3 lidar_mapping.py manual
    python3 lidar_mapping.py auto
    python3 lidar_mapping.py status
    python3 lidar_mapping.py status
    python3 lidar_mapping.py save [地图名称]
    python3 lidar_mapping.py stop
    python3 lidar_mapping.py list

建图期间保持 start 进程运行；移动小车完成扫描后执行 save，再执行 stop。
默认使用 Cartographer 的纯激光扫描匹配，不依赖轮速里程计；地图保存为标准 Nav2 文件。
"""
from __future__ import annotations

import argparse
import fcntl
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


BASE = Path(__file__).resolve().parent
STATE_DIR = Path(os.environ.get("CAR_MAPPING_STATE_DIR", str(Path.home() / ".car_mapping")))
PID_FILE = STATE_DIR / "mapping.pids"
MODE_FILE = STATE_DIR / "mapping.mode"
LOG_FILE = STATE_DIR / "mapping.log"
LOCK_FILE = STATE_DIR / "mapping.lock"
MAP_DIR = Path(os.environ.get("CAR_MAP_DIR", str(Path.home() / "car" / "maps")))
VIEWER_PID_FILE = STATE_DIR / "live_map_viewer.pid"
ROS_SETUP = os.environ.get("CAR_ROS_SETUP", "/opt/ros/humble/setup.bash")
LIDAR_SETUP = os.environ.get("CAR_LIDAR_SETUP", "/home/haha3/ydlidar_ws/install/setup.bash")
SCRIPT_DIR = Path(__file__).resolve().parent
LIDAR_LAUNCH = os.environ.get(
    "CAR_LIDAR_LAUNCH", "ydlidar_ros2_driver x3_ydlidar_launch.py"
)
SCAN_TOPIC = os.environ.get("CAR_SCAN_TOPIC", "/scan")
BASE_FRAME = os.environ.get("CAR_BASE_FRAME", "base_footprint")
LASER_FRAME = os.environ.get("CAR_LASER_FRAME", "laser")
ODOM_FRAME = os.environ.get("CAR_ODOM_FRAME", "odom")
LASER_X = os.environ.get("CAR_LASER_X", "0.0")
LASER_Y = os.environ.get("CAR_LASER_Y", "0.0")
LASER_Z = os.environ.get("CAR_LASER_Z", "0.15")
MAP_SAVE_TIMEOUT = float(os.environ.get("CAR_MAP_SAVE_TIMEOUT", "45"))

CONTROL_PROCESS_MARKERS = (
    "cartographer_node",
    "cartographer_occupancy_grid_node",
    "slam_toolbox",
    "navigation_launch.py",
    "nav2_",
    "frontier_navigator.py",
    "twist_to_serial.py",
    "command_odometry.py",
)


def _lidar_driver_pids() -> list[str]:
    result = subprocess.run(
        ["pgrep", "-f", "[y]dlidar_ros2_driver_node"],
        capture_output=True,
        text=True,
        check=False,
    )
    return result.stdout.split()


def _ros_domain_id() -> str:
    domain = os.environ.get("ROS_DOMAIN_ID")
    if domain is not None and domain.isdigit():
        return domain
    for pid in _lidar_driver_pids():
        try:
            entries = Path(f"/proc/{pid}/environ").read_bytes().split(b"\0")
            for entry in entries:
                if entry.startswith(b"ROS_DOMAIN_ID="):
                    value = entry.split(b"=", 1)[1].decode("ascii")
                    if value.isdigit():
                        return value
        except OSError:
            continue
    return "0"


def _env_command(command: str) -> list[str]:
    shell = (
        f"export ROS_DOMAIN_ID={_ros_domain_id()} && "
        f"source {ROS_SETUP} && "
        f"if [ -f {LIDAR_SETUP} ]; then source {LIDAR_SETUP}; fi && "
        f"exec {command}"
    )
    return ["bash", "-lc", shell]


def _read_pids() -> list[int]:
    try:
        return [int(line.strip()) for line in PID_FILE.read_text().splitlines() if line.strip()]
    except (OSError, ValueError):
        return []


def _alive(pid: int) -> bool:
    # A zombie still accepts kill(pid, 0), but its ROS process is already dead.
    try:
        os.kill(pid, 0)
        stat = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
        state = stat.rsplit(")", 1)[-1].strip().split(" ", 1)[0]
        return state != "Z"
    except OSError:
        return False


def _lidar_driver_running() -> bool:
    return bool(_lidar_driver_pids())


def _untracked_control_processes(owned_pids: list[int]) -> list[str]:
    """Find control/SLAM processes not belonging to this mapping session."""
    try:
        result = subprocess.run(
            ["ps", "-eo", "pid=,args="],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return ["无法读取进程列表，拒绝启动以避免重复控制"]

    owned_groups = set()
    for pid in owned_pids:
        try:
            owned_groups.add(os.getpgid(pid))
        except OSError:
            pass

    found = []
    for line in result.stdout.splitlines():
        item = line.strip()
        if not item:
            continue
        try:
            pid_text, command = item.split(None, 1)
            pid = int(pid_text)
        except (ValueError, IndexError):
            continue
        if pid == os.getpid() or not any(marker in command for marker in CONTROL_PROCESS_MARKERS):
            continue
        try:
            if os.getpgid(pid) in owned_groups:
                continue
        except OSError:
            continue
        found.append(f"{pid} {command}")
    return found


def _write_pids(processes: list[subprocess.Popen]) -> None:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    PID_FILE.write_text("\n".join(str(p.pid) for p in processes) + "\n", encoding="utf-8")


def _write_mode(mode: str) -> None:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    MODE_FILE.write_text(mode + "\n", encoding="ascii")


def _stop_pids(pids: list[int]) -> None:
    for pid in reversed(pids):
        if not _alive(pid):
            continue
        try:
            os.killpg(os.getpgid(pid), signal.SIGINT)
        except OSError:
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass
    time.sleep(1.0)
    for pid in reversed(pids):
        if _alive(pid):
            try:
                os.killpg(os.getpgid(pid), signal.SIGTERM)
            except OSError:
                pass


def _stop_processes() -> None:
    _stop_pids(_read_pids())
    PID_FILE.unlink(missing_ok=True)
    MODE_FILE.unlink(missing_ok=True)


def _mapping_params() -> Path:
    path = STATE_DIR / "slam_toolbox_mapping.yaml"
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    content = f"""slam_toolbox:
  ros__parameters:
    scan_topic: {SCAN_TOPIC}
    base_frame: {BASE_FRAME}
    odom_frame: {ODOM_FRAME}
    map_frame: map
    mode: mapping
    use_sim_time: false
    resolution: 0.05
    map_update_interval: 0.2
    minimum_time_interval: 0.2
    minimum_travel_distance: 0.05
    minimum_travel_heading: 0.05
    transform_publish_period: 0.02
    use_scan_matching: true
    use_scan_barycenter: true
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    max_laser_range: 12.0
    tf_buffer_duration: 30.0
    scan_buffer_size: 20
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
"""
    path.write_text(content, encoding="utf-8")
    return path


def _cartographer_config() -> Path:
    """Create a 2D laser-only Cartographer configuration for this robot."""
    path = STATE_DIR / "cartographer_2d.lua"
    content = f'''include "map_builder.lua"
include "trajectory_builder.lua"

options = {{
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "{BASE_FRAME}",
  published_frame = "{BASE_FRAME}",
  odom_frame = "{ODOM_FRAME}",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.01,
  trajectory_publish_period_sec = 0.03,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 0.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 0.0,
  landmarks_sampling_ratio = 0.0,
}}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
POSE_GRAPH.optimize_every_n_nodes = 60

return options
'''
    path.write_text(content, encoding="utf-8")
    return path


def start(mode: str = "manual") -> int:
    pids = _read_pids()
    alive = [pid for pid in pids if _alive(pid)]
    if alive and len(alive) != len(pids):
        print("检测到建图会话不完整，正在清理残留进程并重新启动。")
        _stop_pids(alive)
        PID_FILE.unlink(missing_ok=True)
        MODE_FILE.unlink(missing_ok=True)
        alive = []
    conflicts = _untracked_control_processes(alive)
    if conflicts:
        print(
            "检测到未登记的 SLAM/导航/底盘控制进程；为避免多套算法争夺控制，拒绝启动：",
            file=sys.stderr,
        )
        print("; ".join(conflicts), file=sys.stderr)
        return 1
    if alive:
        try:
            current_mode = MODE_FILE.read_text(encoding="ascii").strip()
        except OSError:
            current_mode = "auto" if len(alive) >= 7 else "manual"
        if current_mode == mode:
            print("建图已经运行中（%s模式）。" % ("自动" if mode == "auto" else "手动"))
            return 0
        if mode == "manual" and current_mode == "auto" and len(alive) >= 7:
            auto_process_count = 4 if len(alive) >= 8 else 2
            _stop_pids(alive[-auto_process_count:])
            alive = alive[:-auto_process_count]
            PID_FILE.write_text("\n".join(str(pid) for pid in alive) + "\n", encoding="utf-8")
            _write_mode("manual")
            print("已切换为手动建图；SLAM 和当前地图继续运行，自动控制已停止。")
            return 0
        if mode == "auto" and current_mode == "manual":
            commands = [
                f"python3 {SCRIPT_DIR / 'twist_to_serial.py'}",
                f"python3 {SCRIPT_DIR / 'command_odometry.py'} --ros-args -p publish_tf:=false",
                (
                    "ros2 launch nav2_bringup navigation_launch.py "
                    f"params_file:={SCRIPT_DIR / 'nav2_params.yaml'} "
                    "use_sim_time:=false autostart:=true"
                ),
                f"python3 {SCRIPT_DIR / 'frontier_navigator.py'}",
            ]
            log = open(LOG_FILE, "a", encoding="utf-8", buffering=1)
            try:
                added = []
                for command in commands:
                    added.append(subprocess.Popen(
                        _env_command(command), stdout=log,
                        stderr=subprocess.STDOUT, start_new_session=True,
                    ))
                    time.sleep(1.0)
            except Exception as exc:
                print("切换自动建图失败: %s" % exc, file=sys.stderr)
                _stop_pids([process.pid for process in locals().get("added", [])])
                return 1
            finally:
                log.close()
            PID_FILE.write_text(
                "\n".join(str(pid) for pid in alive + [p.pid for p in added]) + "\n",
                encoding="utf-8",
            )
            _write_mode("auto")
            print("已切换为 Nav2 自动探索；前沿目标通过 Nav2 规划和避障。")
            return 0
        print("建图进程状态不完整；为避免接管电机，请先停止建图再重新启动。", file=sys.stderr)
        return 1
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    MAP_DIR.mkdir(parents=True, exist_ok=True)
    log = open(LOG_FILE, "a", encoding="utf-8", buffering=1)
    config_dir = STATE_DIR
    _cartographer_config()
    processes = []
    commands = []
    if not _lidar_driver_running():
        commands.append(f"ros2 launch {LIDAR_LAUNCH}")
    else:
        print("检测到现有 YDLidar 驱动，复用当前雷达话题。")
    commands.extend([
        (
            "ros2 run tf2_ros static_transform_publisher "
            f"{LASER_X} {LASER_Y} {LASER_Z} 0 0 0 {BASE_FRAME} {LASER_FRAME}"
        ),
        (
            "ros2 run cartographer_ros cartographer_node "
            f"-configuration_directory {config_dir} "
            "-configuration_basename cartographer_2d.lua "
            "--ros-args -r scan:=/scan"
        ),
        "ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05",
    ])
    if mode == "auto":
        commands.extend([
            f"python3 {SCRIPT_DIR / 'twist_to_serial.py'}",
            f"python3 {SCRIPT_DIR / 'command_odometry.py'} --ros-args -p publish_tf:=false",
            (
                "ros2 launch nav2_bringup navigation_launch.py "
                f"params_file:={SCRIPT_DIR / 'nav2_params.yaml'} "
                "use_sim_time:=false autostart:=true"
            ),
            f"python3 {SCRIPT_DIR / 'frontier_navigator.py'}",
        ])
    try:
        for command in commands:
            process = subprocess.Popen(
                _env_command(command),
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            processes.append(process)
            time.sleep(1.0)
        _write_pids(processes)
        _write_mode(mode)
        if mode == "manual":
            print("手动建图已开始。请用 UI 按键驾驶小车扫描；手动模式不会启动自动控制。")
        else:
            print("Nav2 自动探索已开始；前沿目标由 Nav2 规划，实时雷达用于局部避障。")
        print("停止时会先保存地图；也可随时点击保存地图。")
        print(f"日志: {LOG_FILE}")
        return 0
    except Exception as exc:
        print(f"启动建图失败: {exc}", file=sys.stderr)
        for process in processes:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except OSError:
                pass
        return 1
    finally:
        log.close()


def _append_save_log(text: str) -> None:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    with LOG_FILE.open("a", encoding="utf-8", buffering=1) as log:
        log.write("\n[map_save] %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
        log.write(text.rstrip() + "\n")


def save(name: str) -> int:
    pids = _read_pids()
    if not pids or not any(_alive(pid) for pid in pids):
        print("当前没有运行中的建图会话。请先执行 start。", file=sys.stderr)
        return 1
    MAP_DIR.mkdir(parents=True, exist_ok=True)
    prefix = MAP_DIR / name
    started_at = time.time()
    command = (
        "ros2 run nav2_map_server map_saver_cli "
        f"-f {prefix} --ros-args -p map_subscribe_transient_local:=true"
    )
    try:
        result = subprocess.run(
            _env_command(command),
            check=False,
            capture_output=True,
            text=True,
            timeout=MAP_SAVE_TIMEOUT,
        )
    except subprocess.TimeoutExpired as exc:
        details = "地图保存超时 %.1f 秒" % MAP_SAVE_TIMEOUT
        if exc.stdout:
            details += "\nstdout:\n" + str(exc.stdout)
        if exc.stderr:
            details += "\nstderr:\n" + str(exc.stderr)
        _append_save_log(details)
        print(details + "；地图未保存，建图会话仍保留。", file=sys.stderr)
        return 124

    output = "\n".join(part for part in (result.stdout, result.stderr) if part)
    yaml_path = prefix.with_suffix(".yaml")
    pgm_path = prefix.with_suffix(".pgm")
    if (
        result.returncode == 0
        and yaml_path.is_file()
        and pgm_path.is_file()
        and yaml_path.stat().st_mtime >= started_at
        and pgm_path.stat().st_mtime >= started_at
    ):
        yaml_size = yaml_path.stat().st_size
        pgm_size = pgm_path.stat().st_size
        if yaml_size > 0 and pgm_size > 100:
            _append_save_log("成功: %s.yaml (%d bytes), %s.pgm (%d bytes)\n%s" % (
                prefix, yaml_size, prefix, pgm_size, output,
            ))
            print(f"地图已保存到树莓派: {prefix}.yaml 和 {prefix}.pgm")
            return 0
    details = "map_saver 退出码 %s；输出:\n%s" % (result.returncode, output)
    if not prefix.with_suffix(".yaml").exists() or not prefix.with_suffix(".pgm").exists():
        details += "\n未生成完整的 yaml/pgm 文件对。"
    _append_save_log(details)
    print("地图保存失败，建图会话仍保留；详细输出已写入: " + str(LOG_FILE), file=sys.stderr)
    return result.returncode or 1


def stop(save_name: str = "room_map") -> int:
    """Save the latest map while ROS is alive, then stop mapping nodes."""
    save_result = save(save_name)
    if save_result != 0:
        print("地图未保存，建图仍在运行；重试保存，或执行 force-stop 强制结束。", file=sys.stderr)
        return save_result
    _stop_processes()
    print("Map saved before stopping: %s.yaml / %s.pgm" % (save_name, save_name))
    return 0


def force_stop() -> int:
    _stop_processes()
    print("建图进程已强制停止，未执行地图保存。")
    return 0


def status() -> int:
    pids = _read_pids()
    alive = [pid for pid in pids if _alive(pid)]
    print("建图状态: 运行中" if alive else "建图状态: 未运行")
    print(f"地图目录: {MAP_DIR}")
    print(f"日志文件: {LOG_FILE}")
    if pids and len(alive) != len(pids):
        print(f"进程: {len(alive)}/{len(pids)} 正常")
    return 0


def list_maps() -> int:
    MAP_DIR.mkdir(parents=True, exist_ok=True)
    maps = sorted(MAP_DIR.glob("*.yaml"))
    if not maps:
        print(f"{MAP_DIR} 中暂无已保存地图。")
        return 0
    for item in maps:
        pgm = item.with_suffix(".pgm")
        state = "完整" if pgm.exists() else "缺少 .pgm"
        print(f"{item.stem}: {state} | {item}")
    return 0


def _viewer_is_running(pid: int) -> bool:
    if not _alive(pid):
        return False
    try:
        command = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ")
        return b"live_map_viewer.py" in command
    except OSError:
        return False


def open_live_map() -> int:
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print("无法打开实时地图窗口：当前 UI 进程没有图形显示环境。", file=sys.stderr)
        return 1
    try:
        pid = int(VIEWER_PID_FILE.read_text(encoding="ascii").strip())
    except (OSError, ValueError):
        pid = 0
    if pid and _viewer_is_running(pid):
        print("实时地图窗口已经打开。")
        return 0
    viewer_log = STATE_DIR / "live_map_viewer.log"
    log = open(viewer_log, "a", encoding="utf-8", buffering=1)
    try:
        process = subprocess.Popen(
            _env_command(f"python3 {SCRIPT_DIR / 'live_map_viewer.py'}"),
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    finally:
        log.close()
    VIEWER_PID_FILE.write_text(str(process.pid) + "\n", encoding="ascii")
    print("实时地图窗口已启动，正在订阅 /map。")
    return 0


def close_live_map() -> int:
    try:
        pid = int(VIEWER_PID_FILE.read_text(encoding="ascii").strip())
    except (OSError, ValueError):
        pid = 0
    if pid and _viewer_is_running(pid):
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)
        except OSError:
            pass
    VIEWER_PID_FILE.unlink(missing_ok=True)
    print("实时地图窗口已关闭。")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="树莓派本地 2D 激光 SLAM 建图")
    parser.add_argument("command", choices=("start", "manual", "auto", "save", "stop", "force-stop", "status", "list", "view", "close-viewer"))
    parser.add_argument("name", nargs="?", default="room_map")
    args = parser.parse_args()
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    with LOCK_FILE.open("w", encoding="ascii") as lock_file:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
        if args.command in ("start", "manual", "auto"):
            return start("auto" if args.command == "auto" else "manual")
        if args.command == "save":
            return save(args.name)
        if args.command == "stop":
            return stop(args.name)
        if args.command == "force-stop":
            return force_stop()
        if args.command == "list":
            return list_maps()
        if args.command == "view":
            return open_live_map()
        if args.command == "close-viewer":
            return close_live_map()
        return status()


if __name__ == "__main__":
    raise SystemExit(main())
