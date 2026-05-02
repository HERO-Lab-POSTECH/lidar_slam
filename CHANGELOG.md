# CHANGELOG - lidar_slam

## [Unreleased] — Phase A-2: laserMapping.cpp dead code 제거 (refactor)

### Changed
- `fast_lio/src/slam/laserMapping.cpp` 1,270줄 → 1,097줄 (-173, dead code only — 알고리즘 미수정)
- `fast_lio/config/slam/mid360.yaml`: `runtime_pos_log_enable`, `pcd_save.interval` 라인 제거 (기능 미구현)

### Removed
- 12개 BSS 누적 배열 (`T1`, `s_plot[1..11]`) — 각 `MAXN=720000` × 8 B = 70 MB BSS, **`s_plot11[scan_count]`은 callback에서 무조건 작성되어 ~20시간 후 OOB 시한폭탄이었음**
- `MAXN` 매크로
- `runtime_pos_log_enable` 파라미터 + 본체 (timer_callback의 if block, main()의 csv 출력, debug fout_pre/out/dbg ofstream, fp FILE*) — 활성화 시 위 OOB 폭탄을 깨움
- `pcd_save.interval` 파라미터 — 선언만 되고 구현 없음 (yaml 코멘트가 *"may lead to memory crash"*로 인정). C-1에서 실제 의미와 함께 재도입 예정
- 통계 추적 globals: `kdtree_incremental_time`, `kdtree_search_time`, `kdtree_delete_time`, `match_time`, `solve_time`, `solve_const_H_time`, `kdtree_size_st`, `kdtree_size_end`, `add_point_size`, `kdtree_delete_counter`, `time_log_counter`, `aver_time_*`, `frame_num`
- `dump_lio_state_to_log()` 함수 — runtime_pos_log 전용
- `_featsArray` global + `points_cache_collect()`의 dead 코멘트 — 선언만 되고 사용 처 없음
- `SigHandle()` signal handler — 등록 코드 주석처리 상태로 dead
- 사용 안 하는 includes: `<fstream>`, `<thread>`, `<csignal>`, `<unistd.h>`, `<visualization_msgs/msg/marker.hpp>`, `<geometry_msgs/msg/vector3.hpp>`
- 미사용 LaserMappingNode 멤버: `effect_feat_num`, `frame_num`, `deltaT`, `deltaR`, `aver_time_*`, `flg_EKF_converged`, `EKF_stop_flg`, `FILE *fp`, `ofstream fout_*`, ~LaserMappingNode 빈 destructor

### Notes
- A-2는 알고리즘 변경 0%. 회귀 측정 의무 없음. 모든 제거 객체가 검증 가능하게 dead 였음 — 통계용 globals들은 `runtime_pos_log` 블록 안에서만 read, 그 블록은 yaml/launch 어디서도 enable 안 됨.
- 다음 phase: **C-1** — `pcl_wait_save` interval-based flush + mutex + map_save 서비스 fix (이게 사용자가 보고한 "장시간 fast_lio 운용 시 RAM 먹통" 문제의 fix).

### Verification
- colcon build PASS (61s)
- 남은 BSS 80 MB는 모두 `ikdtree` SLAM 자료구조 (76 MB) + `res_last`/`point_selected_surf` (500 KB) — 전부 SLAM 필수.

## [Unreleased] — Phase B-2b: TfPublisher 추출 (refactor)

### Changed
- `localization_node.cpp` 430줄 → 345줄 (-85, TF/map/occupancy_grid publishing 제거)
- `localization_node.h`: `pub_map_`, `pub_occupancy_grid_`, `tf_broadcaster_`, `tf_timer_`, `og_*` 파라미터, 4개 publish 메서드 제거 → `std::unique_ptr<TfPublisher> tf_publisher_` 추가

### Added
- `fast_lio/include/fast_lio/localization/tf_publisher.h` — `fast_lio::localization::TfPublisher` 클래스 선언 (68줄)
- `fast_lio/src/localization/tf_publisher.cpp` — TF/map/occupancy_grid publishing 구현 (125줄)

### Notes
- TfPublisher는 `mat_odom2map_`을 const ref로 공유 (소유는 `LocalizationNode`). `odom_mutex_`도 ref로 주입 → 엔진 코드(`localization_engine.cpp`) 무수정.
- `tf_buffer_`/`tf_listener_`는 dead member (현재 어디서도 사용 안 함). 다음 dead-code phase에서 정리 예정.

### Verification
- colcon build PASS (22.1s)
- baseline (post-B-2a, main `37903cd`) vs candidate (B-2b) 60s replay → 마지막 100 포인트 평균 위치 차 **11.82 cm**
- B-2a에서 확립한 noise floor (median 4.7cm, p95 150cm) 내. baseline-vs-baseline 분포와 일관 → 회귀 없음.

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
