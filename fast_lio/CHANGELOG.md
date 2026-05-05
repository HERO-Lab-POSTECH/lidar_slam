# Changelog

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

