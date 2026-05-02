# CHANGELOG - lidar_slam

## [Unreleased] — Phase A: dead code & config cleanup (refactor)

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
