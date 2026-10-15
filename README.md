# witmotion_imu_ros2

ROS 2 Humble serial driver for WT901BLECL (20-byte packets, 115200 bps).
Package: `witmotion_imu_driver`.

## Build and run

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/MasazumiImai/witmotion_imu_ros2.git
cd ~/ros2_ws
colcon build --symlink-install --packages-select witmotion_imu_driver
source install/setup.bash
ros2 launch witmotion_imu_driver imu.launch.py
```

Edit `config/imu.yaml` for the port, frame, sensor `update_rate` (Hz), `use_6axis` and timeout.
Use `params_file:=/absolute/path/imu.yaml` for another configuration file.
Supported rates: 1, 2, 5, 10, 20, 50, 100 and 200 Hz.
The sensor rate and fusion mode are set and read back at startup.
All driver parameters are read-only while running; restart after editing YAML.
`timeout` must be at least two sample periods. Startup configuration is not explicitly saved to EEPROM.
`use_6axis: true` selects sensor-side 6-axis fusion; yaw is relative and can drift over time.
`false` selects 9-axis fusion using magnetic heading.

## Output

`/data_raw` (`sensor_msgs/Imu`, SensorDataQoS) contains acceleration including gravity in m/s², angular velocity in rad/s, and the sensor's attitude as a quaternion.
Timestamps are host decode times; covariances are unknown (zero).
The node exits on disconnection or an IMU packet timeout.

## Magnetic calibration

With the driver running, move the IMU away from magnetic interference, then call:

```bash
ros2 service call /witmotion_imu/start_magnetic_calibration std_srvs/srv/Trigger '{}'
```

After `success: true`, slowly rotate 360° around each of X, Y and Z (three turns per axis are recommended), then finish:

```bash
ros2 service call /witmotion_imu/stop_magnetic_calibration std_srvs/srv/Trigger '{}'
```

The services confirm the calibration mode by register readback.
IMU publication continues during calibration; keep the robot controller stopped during this procedure.
Stop sends a save command for all current sensor settings, including `update_rate` and `use_6axis`.
`success: true` confirms the mode change and command transmission, not yaw accuracy or persistence after power cycling.
Check yaw again after calibration and power cycling.
See the [manufacturer calibration instructions](https://device.report/m/c52bcbb132583022401d74f93b40a800e4757312232965bf7a3b72256685ce3f.pdf#page=18).
Magnetic calibration does not correct yaw while using 6-axis fusion.

## Test

```bash
colcon test --packages-select witmotion_imu_driver --event-handlers console_direct+
colcon test-result --verbose
```

Tests use a pseudo-terminal; no physical IMU is required.
