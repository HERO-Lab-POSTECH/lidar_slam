# Changelog

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

