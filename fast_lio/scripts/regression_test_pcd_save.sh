#!/usr/bin/env bash
#
# Phase C-1 regression: pcd_save flush + consolidate
#
# Phase C-1은 odometry/SLAM 동작 변경 0% (PCD save 경로만 수정).
# 따라서 trajectory 비교가 아니라 PCD 저장 결과를 확인합니다.
#
# 검증:
#   1. baseline (main e5e50a6, no interval) 60s replay 후 PCD 생성 확인
#   2. candidate-default (interval=6000, flush 0회) → baseline과 점 수 5% 이내
#   3. candidate-flush (interval=300, flush 2회) → part files 2개, final PCD 점 수 5% 이내
#   4. RAM peak: candidate-flush < candidate-default
#
# 사용:
#   bash scripts/regression_test_pcd_save.sh baseline
#   bash scripts/regression_test_pcd_save.sh candidate-default
#   bash scripts/regression_test_pcd_save.sh candidate-flush
#   bash scripts/regression_test_pcd_save.sh compare

set -euo pipefail

BAG_PATH="${BAG_PATH:-/workspace/data/7_ucrc_watertank/20260122_sonar_lidar/m750d_custom_platform/m750d-range15-tilt45-v1}"
PLAY_DURATION="${PLAY_DURATION:-60}"
OUT_DIR="${OUT_DIR:-/tmp/fast_lio_c1_regression}"
LAUNCH_PKG=fast_lio
LAUNCH_FILE=mapping.launch.py

YAML_DEFAULT=/workspace/ros2_ws/src/lidar_slam/fast_lio/config/slam/mid360.yaml
YAML_BACKUP=/tmp/mid360_backup.yaml

mode="${1:-}"

ensure_yaml_backup() {
    [[ -f "$YAML_BACKUP" ]] || cp "$YAML_DEFAULT" "$YAML_BACKUP"
}

restore_yaml() {
    if [[ -f "$YAML_BACKUP" ]]; then
        cp "$YAML_BACKUP" "$YAML_DEFAULT"
    fi
}

set_yaml_interval() {
    # $1: interval value (or "remove" to delete the line — for baseline)
    local val="$1"
    if [[ "$val" == "remove" ]]; then
        # Strip interval and consolidate_on_shutdown lines (baseline has neither)
        sed -i -E '/^\s*interval: /d; /^\s*consolidate_on_shutdown: /d; /^\s*-1 disables flush/d; /^\s*On shutdown, merge/d' "$YAML_DEFAULT"
    else
        sed -i -E "s/(\s*interval:\s*)[0-9-]+(.*)/\1${val}\2/" "$YAML_DEFAULT"
    fi
}

set_save_path() {
    # $1: absolute path string
    local p="$1"
    sed -i -E "s|(\s*save_path:\s*)\".*\"(.*)|\1\"${p}\"\2|" "$YAML_DEFAULT"
}

run_one() {
    local label="$1"
    local out="${OUT_DIR}/${label}"
    rm -rf "${out}"
    mkdir -p "${out}"

    local save_pcd="${out}/scans.pcd"
    set_save_path "${save_pcd}"

    echo "[c1-regression] [${label}] launching mapping ..."
    ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" \
        use_sim_time:=true rviz:=false foxglove:=false \
        > "${out}/launch.log" 2>&1 &
    LAUNCH_PID=$!
    sleep 6

    # Find the actual mapping node PID
    local node_pid=""
    for _ in 1 2 3 4 5; do
        node_pid=$(pgrep -f fastlio_mapping_node | head -1 || true)
        [[ -n "$node_pid" ]] && break
        sleep 1
    done
    echo "${node_pid}" > "${out}/node_pid.txt"

    # Sample RSS (KB) every 1s
    if [[ -n "$node_pid" ]]; then
        ( while kill -0 "$node_pid" 2>/dev/null; do
              ps -o rss= -p "$node_pid" 2>/dev/null || break
              sleep 1
          done ) > "${out}/rss.log" 2>&1 &
        RSS_PID=$!
    else
        echo "0" > "${out}/rss.log"
        RSS_PID=""
    fi

    echo "[c1-regression] [${label}] playing bag (${PLAY_DURATION}s) ..."
    timeout "$((PLAY_DURATION + 10))" \
        ros2 bag play "${BAG_PATH}" --clock --rate 1.0 \
        > "${out}/play.log" 2>&1 || true

    sleep 3
    # Send SIGINT to trigger shutdown consolidate
    if [[ -n "$node_pid" ]]; then
        kill -INT "$node_pid" 2>/dev/null || true
    fi
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
    [[ -n "${RSS_PID}" ]] && kill "$RSS_PID" 2>/dev/null || true

    # Wait for PCD write
    sleep 2

    # Inventory output
    {
        echo "=== ${label} ==="
        echo "save_pcd: ${save_pcd}"
        if [[ -f "$save_pcd" ]]; then
            local pts
            pts=$(python3 -c "import open3d as o3d; pc=o3d.io.read_point_cloud('${save_pcd}'); print(len(pc.points))" 2>/dev/null || echo "0")
            echo "final_pts: ${pts}"
        else
            echo "final_pts: MISSING"
        fi
        echo "part_files:"
        ls "${out}"/scans_part*.pcd 2>/dev/null | sort || echo "  (none)"
        echo "rss_peak_kb:"
        if [[ -s "${out}/rss.log" ]]; then
            sort -n "${out}/rss.log" | tail -1
        else
            echo "  0"
        fi
    } | tee "${out}/summary.txt"
}

compare_results() {
    local out="${OUT_DIR}"
    {
        echo "=== Phase C-1 regression summary ==="
        echo "Bag: ${BAG_PATH}"
        echo "Duration: ${PLAY_DURATION}s"
        echo
        for label in baseline candidate-default candidate-flush; do
            if [[ -f "${out}/${label}/summary.txt" ]]; then
                cat "${out}/${label}/summary.txt"
                echo
            fi
        done
    } | tee "${out}/comparison.txt"
}

case "${mode}" in
    baseline)
        ensure_yaml_backup
        # baseline: main e5e50a6 binary already built (caller's responsibility)
        # set yaml to "no interval" mode to mimic main behavior. main code ignores
        # interval/consolidate_on_shutdown anyway, but we keep yaml clean.
        set_yaml_interval remove
        trap restore_yaml EXIT
        run_one baseline
        restore_yaml
        ;;
    candidate-default)
        ensure_yaml_backup
        set_yaml_interval 6000
        trap restore_yaml EXIT
        run_one candidate-default
        restore_yaml
        ;;
    candidate-flush)
        ensure_yaml_backup
        set_yaml_interval 300
        trap restore_yaml EXIT
        run_one candidate-flush
        restore_yaml
        ;;
    compare)
        compare_results
        ;;
    *)
        echo "usage: $0 {baseline|candidate-default|candidate-flush|compare}"
        echo
        echo "Typical flow:"
        echo "  git checkout main && colcon build --packages-select fast_lio"
        echo "  bash $0 baseline"
        echo
        echo "  git checkout refactor/phase-c1-pcd-flush && colcon build --packages-select fast_lio"
        echo "  bash $0 candidate-default"
        echo "  bash $0 candidate-flush"
        echo "  bash $0 compare"
        exit 1
        ;;
esac
