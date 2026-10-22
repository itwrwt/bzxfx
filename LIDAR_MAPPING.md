# Raspberry Pi 2D SLAM and Nav2

The active Raspberry Pi `lidar_mapping.py` starts **Cartographer 2D** using
laser scan matching. In automatic mode, Nav2 consumes Cartographer's `/map`
and TF; the script does not start a second SLAM system. Maps remain on the Pi
under `~/car/maps`.

The session controller now serializes start/save/stop operations with a lock
and refuses to start if it sees an unregistered SLAM, Nav2, frontier, or motor
bridge process. It manages only its own recorded process groups; it does not
kill an unrelated process automatically.

The script still contains an unused helper that can generate a
`slam_toolbox_mapping.yaml` file. That helper is not called by `start()` and
does not launch `slam_toolbox`; the YAML file under `~/.car_mapping` is
therefore not evidence that slam_toolbox is active. Old rollback directories
may contain historical slam_toolbox versions and are not used by the current
entry point.

## Dependencies

On Ubuntu 22.04 with ROS 2 Humble:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

`cartographer_ros`, `nav2_map_server`, `python3-serial`, and the YDLidar
workspace are used by the Pi project. The driver workspace is sourced from
`/home/haha3/ydlidar_ws/install/setup.bash`.

## Run

```bash
cd /home/haha3/1/bzxfx
python3 lidar_mapping.py manual
```

Manual mode keeps the existing SLAM map-building session and leaves
the motor under UI control. Start or switch to Nav2 exploration with:

```bash
python3 lidar_mapping.py auto
```

The frontier client selects reachable free cells adjacent to unknown map
areas and submits `NavigateToPose` goals. Nav2 plans against the SLAM `/map`
and uses the raw `/scan` topic in the local and global obstacle layers. To
return to manual driving without stopping SLAM, run `manual` again. `stop`
saves the latest map before stopping the mapping session.

## Motion and calibration limits

The motor board accepts discrete forward and in-place left/right commands,
not wheel velocities. The bridge maps Nav2's `Twist` requests to those
commands; path tracking is therefore less smooth than a differential-drive
controller with independently controlled wheel speeds.

There are no verified wheel encoders. SLAM provides scan-matched map
pose/TF; `command_odometry.py` publishes an open-loop `/odom` velocity estimate
from the commands actually dispatched, without publishing a competing TF.
`twist_to_serial.py` defaults to `44 deg/s` for open-loop angular odometry.
This is an estimate used to integrate the commanded turn into `/odom`, not a
motor speed setting; the serial protocol still issues discrete rotate
commands. `CAR_ROT_DEG_PER_SEC` can override the estimate. The UI's short-pulse
timing calibration is separate.

The bridge adds scan-based directional stopping: nominal stop thresholds are
0.35 m front, 0.30 m rear, and 0.25 m on either side (the requested body
clearances plus 0.05 m margin), with 0.10 m release hysteresis. In-place turns
are stopped if the nearest valid return is under 0.42 m, approximating the
rotating rectangular chassis envelope. Nav2's footprint is 0.30 m forward,
0.25 m rearward, and 0.20 m to each side. These thresholds assume the laser
origin is near the chassis reference; verify the configured laser transform.

Map saving has a bounded 45-second wait, logs the `map_saver_cli` output, and
checks that fresh YAML/PGM artifacts exist before reporting success. The UI
runs mapping commands in a worker thread. “Save map” uses a timestamped name
and stops the mapping session only after a successful save; “Force stop” does
not save. A failed save leaves the session available for retry.
The Pi's X3 driver is configured at 5 Hz to match SLAM's 0.2-second minimum
scan interval. Nav2's behavior-tree loop is 50 ms (20 Hz), instead of 10 ms
(100 Hz), to reduce scheduling pressure on the Pi.
The UI's `BASE_SPEED=0.55` is PWM duty,
not meters per second. Nav2 therefore starts with a conservative
`CAR_LINEAR_SPEED_MPS=0.08` estimate. Measure straight-line distance over a
timed run and calibrate this value before using long routes. Slip still causes
error, so initial autonomous tests should be slow, in a clear area, with the
emergency stop reachable.

The lidar scan is used without left-right mirroring. The Pi's
actual scan frame is `laser`, with the static transform defaulting to
`base_footprint -> laser` and a 0.15 m z offset.

Nav2 costmaps use a rectangular footprint relative to `base_footprint`:
0.10 m forward, 0.30 m backward, and 0.10 m to either side. Adjust these extents if the measured chassis,
protruding wheels, or sensor mounts are larger.

## Diagnostics

```bash
python3 lidar_mapping.py status
tail -f ~/.car_mapping/mapping.log
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 action list | grep navigate_to_pose
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser
```

建图不动或地图保存失败时，也可以点击 UI 的“建图诊断”，或运行：

```bash
python3 lidar_mapping.py diagnose
```

该命令只读检查 `/scan`、`/map`、ROS 节点、Nav2 动作、速度话题和
`map -> base_footprint` TF，并将 `map_saver` 的完整输出与生成文件大小
写入 `~/.car_mapping/mapping.log`。
