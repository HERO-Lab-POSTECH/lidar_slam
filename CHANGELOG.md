# CHANGELOG - lidar_slam

## [Unreleased] — Phase B-2a: localization_engine split (refactor)

### Changed
- `localization_node.cpp` 1,107줄 → 430줄 (-677, ROS2 plumbing만 남김)
- 알고리즘 메서드 9종 (`localizationLoop`, `initializeLocalization`, `performGlobalLocalizationWithFitness`, `performLocalization`, `performLocalizationStep`, `prepareScanForGlobalLocalization`, `ensureMapFpfhComputed`, `generateTranslationCandidates`, `evaluateSingleHypothesis`) 본체를 `localization_engine.cpp`로 이동
- 클래스 구조 무변경: `LocalizationNode`는 단일 클래스 그대로, 메서드 정의만 두 .cpp로 split

### Added
- `fast_lio/include/fast_lio/localization/localization_node.h` — 클래스 선언 (171줄)
- `fast_lio/src/localization/localization_engine.cpp` — 알고리즘 본체 (567줄)
- `fast_lio/scripts/regression_test_localization.sh` — Localization 회귀 (KIRO map_v1.pcd + bag, use_global_localization=false로 결정적 ICP)
- `fast_lio/scripts/regression_plot.py` — baseline vs candidate trajectory 시각 비교

### Verification
- colcon build PASS (29.5s)
- baseline (post-B-1) vs candidate odometry 60s replay → 마지막 100 포인트 평균 위치 차 **0.02 cm** (부유점 잡음 수준)
- Trajectory plot: XY/Z 궤적 완전 일치 (육안 식별 불가)

### Notes
- `use_global_localization: false` + 고정 `initial_pose`로 ICP만 동작 → 결정적 비교. 실제 운용 시는 `use_global_localization: true`(기본)로 사용.
- Phase B-2b (`tf_publisher` 추출)는 별도 PR.

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
