# Changelog

## [Unreleased] — Phase P3: TF naming standard (refactor)

### Added
- `livox_link` (alias of `livox_frame`, REP-105 + Autoware 표준)
- `oculus_link`, `ping360_link`, `ping1d_link`, `sonoptix_link` — per-sensor child links under `sonar_link`

### Notes
- Legacy aliases (`body`, `oculus`, `livox_frame`) preserved for old bag file compatibility
- All new sensor links use xyz="0 0 0" rpy="0 0 0" (same physical mount as parent — placeholder for future per-sensor offsets)

### Verification
- XML parse PASS (links 11, joints 10)
- colcon build PASS
- robot_state_publisher: 신규 5 frames 모두 /tf_static에 publish됨
