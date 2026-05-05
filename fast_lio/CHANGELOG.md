# Changelog

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

