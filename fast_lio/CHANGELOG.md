# Changelog

## [Unreleased] — Post-Audit Fix PR-N (fix)

### Fixed
- `fast_lio/scripts/regression_test.sh`: `ODOM_TOPIC` default `/fast_lio/odometry` → `/localization/fast_lio/odometry` to track PR-J topic rename (7th audit High N-1).
- `fast_lio/scripts/regression_test_localization.sh`: `ODOM_TOPIC` default `/fast_lio/localization/odometry` → `/localization/fast_lio_loc/odometry`; comment cite of `/fast_lio/odometry` → `/localization/fast_lio/odometry` (7th audit High N-1).

### Changed
- `README.md` FAST-LIO topic catalog reflects 8-phase workspace convention rename: `/fast_lio/odometry` → `/localization/fast_lio/odometry`, `/fast_lio/cloud_registered` → `/localization/fast_lio/points_body`, `/fast_lio/path` → `/fast_lio/debug/path` (7th audit Medium N-3).

### Added
- `.gitignore` (repo root): exclude `build/`, `install/`, `log/`, `fast_lio/PCD/*.pcd`, `*.pyc`, `__pycache__/` to prevent accidental check-in of 164MB PCD outputs and colcon build artefacts (7th audit High N-2).

### Verification
- regression scripts, README, .gitignore are non-build assets; colcon build skipped (no source change).

## [Unreleased] — Post-Audit Fix PR-G (fix)

### Changed
- `rviz/fastlio.rviz`: `Fixed Frame: camera_init` → `odom`; TF Frames + Tree `camera_init` → `odom`. Topics `/fast_lio/odometry` → `/localization/fast_lio/odometry`, `/fast_lio/cloud_registered_body` → `/localization/fast_lio/points_body`, `/fast_lio/debug/cloud_effected` → `/fast_lio/debug/points_effected` (4th audit Critical: post-P3/P15 rename never propagated to rviz config).
- `rviz/fastlio.rviz`: Reliability Policy Reliable → Best Effort for `/fast_lio/debug/path`, `/localization/fast_lio/points_body`, `/fast_lio/debug/points_effected`, `/fast_lio/debug/map` (publishers use `pkrc_qos::sensor_qos()` BE; Reliable sub never received messages).
- `rviz/localization.rviz`: TF Frames + Tree `camera_init` → `odom`. Topics `/fast_lio/cloud_registered_body` → `/localization/fast_lio/points_body`, `/fast_lio/localization/map` → `/localization/fast_lio_loc/map`, `/fast_lio/localization/occupancy_grid` → `/localization/fast_lio_loc/occupancy_grid`, `/fast_lio/odometry` → `/localization/fast_lio/odometry`, `/fast_lio/localization/odometry` → `/localization/fast_lio_loc/odometry`.
- `rviz/localization.rviz`: Path display Reliability → Best Effort; CurrentScan Reliability → Best Effort.
- `launch/mapping.launch.py`: rviz2 Node `parameters=[{'use_sim_time': use_sim_time == 'true'}]` (without this, rviz uses wall clock during bag replay; localization.launch.py already had this).
- `src/slam/laserMapping.cpp:publish_path` — `path.header.stamp = msg_body_pose.header.stamp` per call. Without this, Path msg header carried constructor-time wall clock for the entire run.

### Verification
- colcon build PASS

## [Unreleased] — Post-Audit Fix PR-E (fix)

### Changed
- `launch/localization.launch.py` docstring TOPICS Input — `/livox/lidar` → `/sensor/lidar/livox_mid360/points`, `/livox/imu` → `/sensor/ins/livox_mid360/imu` (H-4)
- `launch/mapping.launch.py` docstring TOPICS Input/Output — livox topic names corrected; `/fast_lio/path` → `/fast_lio/debug/path` (H-5)
- `src/slam/laserMapping.cpp` lidar/IMU subscriber QoS — `qos_reliability` yaml 파라미터 분기 제거, `rclcpp::QoS(10)` / `rclcpp::SensorDataQoS()` 로컬 패턴 → `pkrc_qos::sensor_qos()` (M-2). depth 5로 통일.
- `src/localization/localization_node.cpp:193` `sub_initialpose_` QoS — `10` → `pkrc_qos::reliable_qos()` (L-1, style only).

### Verification
- colcon build PASS (56.2s, fast_lio)

## [Unreleased] — Pre-experiment Fix H-1 (fix)

### Changed
- `launch/mapping.launch.py`, `launch/localization.launch.py` — pass `use_sim_time` via `launch_arguments` when including `boat_description/robot_state_publisher.launch.py`. Required so RSP runs on sim clock during bag replay.

## [Unreleased] — Post-Audit Fix B-1 + B-2 (fix)

### Fixed
- B-1: localization_node.cpp cloud subscription topic `/localization/fast_lio/cloud_body` → `/localization/fast_lio/points_body` (publisher uses `points_body`; PR-A introduced silent ICP data loss).
- B-2: localization_node.cpp subscriber QoS — sub_odom_ now `pkrc_qos::reliable_qos()`, sub_cloud_ now `pkrc_qos::sensor_qos()` (matches publisher QoS profiles).

### Verification
- colcon build PASS
- grep `/localization/fast_lio/cloud_body` runtime = 0 hits

### Notes
- Triggered by 2026-05-06 second-pass audit on PR-A1.

## [Unreleased] — Phase P9: Post-merge fixes (fix)

### Fixed
- `src/localization/localization_node.cpp`: subscriber 토픽명을 P4 컨벤션에 맞춰 수정
  - `/fast_lio/odometry` → `/localization/fast_lio/odometry` (C-1)
  - `/fast_lio/cloud_registered_body` → `/localization/fast_lio/cloud_body` (C-1)
- `src/slam/laserMapping.cpp`: `qos_reliability` 기본값 `best_effort`로 변경 (C-1b)
  - livox driver SensorDataQoS(BEST_EFFORT) publisher와 정합
- `src/slam/laserMapping.cpp`, `include/fast_lio/localization/localization_node.h`, `include/fast_lio/localization/tf_publisher.h`: 레거시 frame 기본값 REP-105/Autoware 기준으로 통일 (M-1)
  - `camera_init` → `odom`, `body` → `base_link`
- `launch/mapping.launch.py`, `launch/localization.launch.py`: TOPICS docstring을 실제 publish 토픽에 맞춰 정렬

### Verification
- colcon build PASS (fast_lio)
- grep 검증: `"camera_init"` / `"body"` src/include 0 hits
- grep 검증: `/fast_lio/odometry` / `/fast_lio/cloud_registered_body` runtime 코드 0 hits

### Notes
- 2026-05-06 post-merge audit에서 발견된 항목 (docs/superpowers/audits/)

## [Unreleased] — Phase P7: Map save UX (refactor)

### Changed
- `launch/mapping.launch.py`: `generate_launch_description()` node creation wrapped in `OpaqueFunction` (`_setup_nodes`) so `output_map_path` can be resolved at launch time (was static LaunchConfiguration substitution)
- `launch/mapping.launch.py`: `output_map_path` description updated: `Empty = auto-timestamp`
- `launch/mapping.launch.py`: `import os.path` → `import os` (needed for `os.environ`/`os.path.expanduser`)
- `launch/mapping.launch.py`: `IfCondition` removed (replaced by plain `if use_rviz:` in OpaqueFunction)
- Header docstring updated: default note for `output_map_path` + auto-timestamp example

### Added
- `launch/mapping.launch.py`: `_resolve_map_path(user_path, pkg, filename)` inline helper (spec §2.9):
  - Empty `output_map_path` → auto-timestamp dir `$PKRC_MAP_DIR/fast_lio/<YYYYMMDD_HHMMSS>/` (fallback `~/data/maps`)
  - Updates relative `latest -> <ts>` symlink
  - Non-empty → use as-is, parent dir auto-created

### Verification
- colcon build PASS (fast_lio 0.27s)
- `ros2 launch fast_lio mapping.launch.py --show-args` PASS
- Empty `output_map_path`: timestamp dir + `latest` symlink created under `$PKRC_MAP_DIR/fast_lio/`

### Notes
- `IfCondition` from `launch.conditions` is no longer imported (was only used for rviz conditional)

## [Unreleased] — Phase P6: Config structure (refactor)

### Changed
- `config/slam/mid360.yaml`: header docstring + category separators (TF/COMMON/PREPROCESS/MAPPING/PUBLISH/PCD_SAVE) + unit annotations ([m]/[s]/[Hz]/[deg]) + [Static]/[Dynamic] tags
- `config/localization/localization.yaml`: header docstring + category separators (MAP/REGISTRATION/INITIAL POSE/GLOBAL LOCALIZATION/KALMAN FILTER/SUBMAP UPDATE/DEBUG/TF FRAMES/OCCUPANCY GRID) + unit annotations + [Static] tags

### Added
- No new parameters — documentation only

### Verification
- colcon build PASS (fast_lio 0.26s)
- yaml.safe_load PASS (both yaml files)
- ros2 launch fast_lio mapping.launch.py startup: no parse errors

### Notes
- scan_publish_en working-tree edit (false→true) excluded from this commit — unrelated user edit
- Values unchanged — cosmetic / documentation only per spec §2.7

## [Unreleased] — Phase P5a: Launch arg standardization (refactor)

### Changed (BREAKING — external launch invocations)
- mapping.launch.py:
  - `rviz` → `use_rviz`
  - `save_map_path` → `output_map_path`
  - `config_path` 단일화 (`config_file` 통합; default: `<pkg>/config/slam/mid360.yaml`)
  - `rviz_cfg` → `rviz_config_path`
- localization.launch.py:
  - `rviz` → `use_rviz`
  - `config_path` 단일화 (`config_file`/`lio_config_path`/`lio_config_file` 통합)
- 헤더 docstring 표준 적용 (spec §2.5.3)

### Removed (BREAKING)
- mapping.launch.py: `qos_reliability` arg (QoS helper 적용 후 미사용 — P4)
- mapping.launch.py: `config_file` arg (config_path 통합)
- localization.launch.py: `lio_config_path`, `lio_config_file` args
- localization.launch.py: `config_file` arg

### Fixed
- mapping.launch.py: `decalre_config_file_cmd` typo (블록 제거로 해결)

### Migration
- `rviz:=true` → `use_rviz:=true`
- `save_map_path:=/path` → `output_map_path:=/path`
- `rviz_cfg:=/path` → `rviz_config_path:=/path`
- `config_file:=foo.yaml` → `config_path:=/full/path/to/foo.yaml`
- `lio_config_path:=foo lio_config_file:=bar.yaml` → `config_path:=/full/path/to/foo/bar.yaml`
- `qos_reliability:=...` → (제거; 영향 없음)

### Verification
- colcon build PASS
- ros2 launch --show-args: 신규 arg 표시

---

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

