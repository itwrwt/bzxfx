# bzxfx Rover Mapping and Navigation

Raspberry Pi rover control, YDLIDAR X3 integration, 2D mapping, and Nav2 frontier exploration.

## Main modules

- `Gui.py`: Tkinter control interface and manual operation entry point.
- `radar_bridge.py`, `ydlidar_x3.yaml`: lidar bridge and configuration.
- `lidar_mapping.py`, `map_snapshot.py`: Cartographer mapping session and map saving.
- `frontier_explorer.py`, `frontier_navigator.py`, `nav2_params.yaml`: frontier exploration and Nav2 parameters.
- `twist_to_serial.py`, `motion_command.py`: motion command, chassis serial adapter, and safety checks.
- `command_odometry.py`, `scan_pose_odometry.py`: open-loop odometry and scan-pose helpers.

## Environment

Target deployment: Raspberry Pi, Ubuntu 22.04, ROS 2 Humble. ROS 2, Nav2, Cartographer, and the YDLIDAR driver must be installed separately; they are not installed by `requirements.txt`.

```bash
python3 -m pip install -r requirements.txt
```

Adjust lidar, navigation, and serial settings for the actual hardware. Validate motion and emergency-stop behavior in a safe area before autonomous navigation.

## Optional API credentials

Legacy optional AI examples read credentials from environment variables. Never commit API keys:

```bash
export LINGYI_API_KEY="..."
export DASHSCOPE_API_KEY="..."
```

Generated maps, logs, caches, local environment files, and historical deployment/rollback backups are excluded from this source backup.
