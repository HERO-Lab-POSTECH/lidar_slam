# LiDAR SLAM

ROS2 LiDAR SLAM packages for 3D mapping and localization.

## Packages

| Package | Description |
|---------|-------------|
| `fast_lio_slam` | FAST-LIO2 + SC-PGO integrated SLAM |
| `cartographer_slam` | Google Cartographer based SLAM |

## Build

```bash
cd /workspace/ros2_ws
colcon build --packages-select fast_lio_slam cartographer_slam
source install/setup.bash
```

## Usage

### FAST-LIO SLAM

```bash
ros2 launch fast_lio_slam mapping.launch.py
```

### Cartographer SLAM

```bash
ros2 launch cartographer_slam livox_slam.launch.py
```

## TF Tree (fast_lio_slam)

```
camera_init
├── odom_raw   (FAST-LIO raw odometry)
└── body       (SC-PGO optimized, 10Hz)
```

## Topics (fast_lio_slam)

| Topic | Type | Description |
|-------|------|-------------|
| `/fast_lio/odometry` | Odometry | Raw odometry from FAST-LIO |
| `/fast_lio/sc_pgo/odom` | Odometry | Optimized odometry (10Hz) |
| `/fast_lio/sc_pgo/path` | Path | Optimized trajectory |
| `/fast_lio/sc_pgo/map` | PointCloud2 | Optimized map |
