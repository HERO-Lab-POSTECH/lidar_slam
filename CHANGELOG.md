# CHANGELOG - lidar_slam

## [Unreleased] — Phase B-1: KalmanFilter & pose_utils 추출 (refactor)

### Changed
- `localization_node.cpp` 1,210줄 → 1,107줄 (-103줄)
- 호출부는 `pose_utils::` namespace prefix로 갱신 (`createOdometryMsg`, `createCropBox`, `limitPointCloud`)
- `KalmanFilter` 호출은 `using` alias로 무변경

### Added
- `fast_lio/include/fast_lio/localization/kalman_filter.h` — 1D Kalman Filter (header-only, ROS2/Open3D 의존 0)
- `fast_lio/include/fast_lio/localization/pose_utils.h` — pure helper 4종 선언
- `fast_lio/src/localization/pose_utils.cpp` — `matrixToPose`, `createOdometryMsg`, `createCropBox`, `limitPointCloud` 구현
- `fast_lio/CMakeLists.txt`: `localization_pose_utils` 라이브러리 등록 + `localization_node`에 링크

### Notes
- 알고리즘 영향 0 (단순 cut/paste). 회귀 테스트 의무 약함.
- `localization_node.cpp`에 남은 SRP 위반(performLocalizationStep ~210줄 등)은 Phase B-2에서 처리.

## [2026-05-02] — Phase A: dead code & config cleanup (refactor)

### Removed
- **Vendored dead headers** (~4,656 LOC, 0 references in any source)
  - `fast_lio/include/fast_lio/slam/matplotlibcpp.h` (2,499 LOC)
  - `fast_lio/include/fast_lio/slam/nanoflann.hpp` (2,040 LOC)
  - `fast_lio/include/fast_lio/slam/KDTreeVectorOfVectorsAdaptor.h` (117 LOC)
- **Unused LiDAR config files** (운용 LiDAR는 Livox MID-360 단일)
  - `fast_lio/config/slam/avia.yaml`
  - `fast_lio/config/slam/horizon.yaml`
  - `fast_lio/config/slam/ouster64.yaml`
  - `fast_lio/config/slam/velodyne.yaml`
- **Dead Python/matplotlib 의존성**
  - `fast_lio/src/slam/laserMapping.cpp`: `#include <Python.h>` 제거 (matplotlibcpp 잔재)
  - `fast_lio/CMakeLists.txt`: `find_package(PythonLibs REQUIRED)`, `find_path(MATPLOTLIB_CPP_INCLUDE_DIRS …)`, `${PYTHON_INCLUDE_DIRS}`, `${PYTHON_LIBRARIES}` 모두 제거

### Added
- **Regression test skeleton** (`fast_lio/scripts/`)
  - `regression_test.sh` — baseline/candidate 두 번 replay 후 비교 워크플로우
  - `regression_compare.py` — ATE RMSE + 최종 drift 계산 (임계: ATE ≤ 1cm, drift ≤ 5cm)
  - Phase B/C 같은 알고리즘 영향 가능 PR마다 사용

### Notes
- Baseline 보존: 태그 `archive/lidar-slam-pre-refactor-2026-05-02` (SHA `6d75fcd`)
- 설계 문서: `/workspace/docs/plans/2026-05-02-lidar-slam-refactor-design.md`
- Phase A는 알고리즘 영향 0% (참조되지 않는 헤더·config 제거뿐) → regression replay 의무 없음. Phase B부터는 머지 전 PASS 필수.
