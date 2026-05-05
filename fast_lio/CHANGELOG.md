# Changelog

## [Unreleased] — Phase P4b: Topic naming + QoS (refactor)

### Changed
- `src/slam/laserMapping.cpp`: publisher topic renames + QoS helper adoption
  - `/fast_lio/odometry` → `/localization/fast_lio/odometry` (RELIABLE_QOS)
  - `/fast_lio/cloud_registered_body` → `/localization/fast_lio/points_body` (SENSOR_QOS)
  - `/fast_lio/debug/cloud_registered` → `/fast_lio/debug/points_world` (SENSOR_QOS)
  - `/fast_lio/debug/cloud_effected` → `/fast_lio/debug/points_effected` (SENSOR_QOS)
  - `/fast_lio/debug/map`: name unchanged, QoS → SENSOR_QOS helper
  - `/fast_lio/debug/path`: name unchanged, QoS → SENSOR_QOS helper
- `src/localization/localization_node.cpp`: publisher topic renames + QoS helper adoption
  - `/fast_lio/localization/odometry` → `/localization/fast_lio_loc/odometry` (RELIABLE_QOS)
  - `/fast_lio/localization/confidence` → `/localization/fast_lio_loc/confidence` (RELIABLE_QOS)
- `src/localization/tf_publisher.cpp`: publisher topic renames + QoS helper adoption
  - `/fast_lio/localization/map` → `/localization/fast_lio_loc/map` (LATCHED_QOS)
  - `/fast_lio/localization/occupancy_grid` → `/localization/fast_lio_loc/occupancy_grid` (LATCHED_QOS)
- `scripts/regression_compare.py`: `--topic` default updated to `/localization/fast_lio/odometry`
- `scripts/regression_plot.py`: `read_xyz` default topic updated to `/localization/fast_lio_loc/odometry`

### Notes
- Subscribers in `localization_node.cpp` (`/fast_lio/odometry`, `/fast_lio/cloud_registered_body`) intentionally unchanged — updated in P4c (sonar_3d sync)

### Verification
- colcon build PASS (fast_lio, cartographer_slam, ~61s)
- static grep: 0 legacy publisher topic refs in source

---

## [Unreleased] — Phase P4a: QoS helper module (refactor)

### Added
- `include/fast_lio/qos.hpp` — workspace QoS 3-tier helper (SENSOR/RELIABLE/LATCHED) per spec §2.4

### Verification
- colcon build PASS

---

## [Unreleased] — Phase P3: TF naming standard (refactor)

### Changed
- `fast_lio/src/slam/preprocess.cpp`: hardcoded `frame_id = "livox"` → `"livox_link"`
- `fast_lio/src/localization/localization_node.cpp`: default parameter
  `odom_frame "camera_init"` → `"odom"`, `body_frame "body"` → `"base_link"`
- `fast_lio/config/slam/mid360.yaml`: `odom_frame: "map"` → `"odom"` (REP-105)
- `fast_lio/config/localization/localization.yaml`: `odom_frame: "odom"` already set (no change)

### Verification
- `colcon build --packages-select fast_lio --symlink-install`: PASS

---

## [Unreleased] — Phase P2: Foxglove removal (refactor)

### Removed
- `foxglove` launch arg, `foxglove_bridge` Node, IfCondition 블록 일괄 제거
  - `fast_lio/launch/mapping.launch.py`
  - `fast_lio/launch/localization.launch.py`

### Verification
- colcon build PASS
- `ros2 launch fast_lio {mapping,localization}.launch.py --show-args`에 foxglove 없음

