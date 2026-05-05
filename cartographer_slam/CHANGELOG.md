# Changelog

## [Unreleased] — Phase P2: Foxglove removal (refactor)

### Removed
- `foxglove` launch arg + `foxglove_bridge` Node 일괄 제거
  - `cartographer_slam/launch/slam.launch.py`
- `cartographer_slam/foxglove/` 디렉터리 + 그 안의 layout JSON 삭제

### Verification
- colcon build PASS
- `ros2 launch cartographer_slam slam.launch.py --show-args`에 foxglove 없음

