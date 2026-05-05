# Changelog

## [Unreleased] — Phase P7: Map save UX (refactor)

### Changed
- `launch/slam.launch.py`: `output_map_path` handling updated — `_resolve_map_path` called before `.pbstream` extension logic
- `launch/slam.launch.py`: old `if output_map_path: os.makedirs(...)` block removed (redundant — helper handles it)
- `launch/slam.launch.py`: `if output_map_path:` guard on `cartographer_args.extend(['-save_state_filename', ...])` removed — save always happens (helper always returns a valid path)
- `launch/slam.launch.py`: header docstring: removed outdated NOTE about `output_map_path` being required; added auto-timestamp example
- `launch/slam.launch.py`: `DeclareLaunchArgument('output_map_path')` description updated: `Empty = auto-timestamp`

### Added
- `launch/slam.launch.py`: `_resolve_map_path(user_path, pkg, filename)` inline helper (spec §2.9):
  - Empty `output_map_path` → auto-timestamp dir `$PKRC_MAP_DIR/cartographer/<YYYYMMDD_HHMMSS>/` (fallback `~/data/maps`)
  - Updates relative `latest -> <ts>` symlink
  - Non-empty → use as-is, parent dir auto-created

### Verification
- colcon build PASS (cartographer_slam 0.54s)
- `ros2 launch cartographer_slam slam.launch.py --show-args` PASS
- Empty `output_map_path`: timestamp dir + `latest` symlink created under `$PKRC_MAP_DIR/cartographer/`

### Notes
- map is now always saved on shutdown even when user omits `output_map_path`

## [Unreleased] — Phase P6: Config structure (refactor)

### Changed
- `config/slam_2d.lua`: standardized header docstring + category separators (FRAMES/SENSORS/TF AND PUBLISHING/SAMPLING RATIOS/MAP BUILDER/TRAJECTORY BUILDER/POSE GRAPH) + unit annotations ([m]/[s]/[rad])

### Added
- No new parameters — documentation only

### Verification
- colcon build PASS (cartographer_slam 0.75s)
- lua not installed; cartographer build is primary gate — PASS

### Notes
- Values unchanged — cosmetic / documentation only per spec §2.7

## [Unreleased] — Phase P5a: Launch arg standardization (refactor)

### Changed (BREAKING — external launch invocations)
- slam.launch.py:
  - `localization` → `use_localization`
  - `load_state_filename` → `map_path`
  - `save_state_filename` → `output_map_path`
- 헤더 docstring 표준 적용 (spec §2.5.3)

### Migration
- `localization:=true` → `use_localization:=true`
- `load_state_filename:=/p/x.pbstream` → `map_path:=/p/x.pbstream`
- `save_state_filename:=/p/y.pbstream` → `output_map_path:=/p/y.pbstream`

### Verification
- colcon build PASS
- ros2 launch --show-args: 신규 arg 표시

---

## [Unreleased] — Phase P4b: Topic naming (refactor)

### Changed
- `include/cartographer_slam/node_constants.h`:
  - `kOdometryOutputTopic`: `"cartographer_2d/odometry"` → `"/localization/cartographer/odometry"` (absolute path)

### Notes
- QoS kept as literal `10` depth for cartographer publishers (no helper adoption in P4b)
- Other `node_constants.h` constants (kOccupancyGridTopic, kScanMatchedPointCloudTopic, etc.) unchanged — not in P4b scope per spec §2.3.2

### Verification
- colcon build PASS (cartographer_slam, ~61s combined)
- static grep: 0 legacy `cartographer_2d/odometry` refs in source

---

## [Unreleased] — Phase P4a: QoS helper module (refactor)

### Added
- `include/cartographer_slam/qos.hpp` — workspace QoS 3-tier helper (SENSOR/RELIABLE/LATCHED) per spec §2.4

### Verification
- colcon build PASS

---

## [Unreleased] — Phase P3: TF naming standard (refactor)

### Changed
- `cartographer_slam/config/slam_2d.lua`:
  - `tracking_frame = "base_link"` → `"livox_link"` (sensor frame for scan input)
  - TF tree comment: `livox_frame` → `livox_link`
  - `published_frame`, `odom_frame`, `map_frame` already correct (no change)

### Verification
- `colcon build --packages-select cartographer_slam --symlink-install`: PASS

---

## [Unreleased] — Phase P2: Foxglove removal (refactor)

### Removed
- `foxglove` launch arg + `foxglove_bridge` Node 일괄 제거
  - `cartographer_slam/launch/slam.launch.py`
- `cartographer_slam/foxglove/` 디렉터리 + 그 안의 layout JSON 삭제

### Verification
- colcon build PASS
- `ros2 launch cartographer_slam slam.launch.py --show-args`에 foxglove 없음

