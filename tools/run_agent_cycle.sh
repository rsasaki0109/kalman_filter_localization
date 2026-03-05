#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"

BAG_PATH="${1:-data/istanbul/all-sensors-bag1_compressed}"
OUTPUT_DIR="${2:-/tmp/kfl_cycle01}"
PARAM_GRID_JSON="${3:-src/kalman_filter_localization/tools/param_grid_istanbul_quick.json}"
CYCLE_LABEL="${4:-$(basename "$OUTPUT_DIR")}"
SUMMARY_REGISTRY="${5:-src/kalman_filter_localization/docs/results/agent_loop_summary_paths.txt}"
CYCLE_LOG="$OUTPUT_DIR/run_agent_cycle.log"

cd "$WORKSPACE_ROOT"
set +u
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi
set -u

mkdir -p "$OUTPUT_DIR"
: > "$CYCLE_LOG"
echo "requested_ros_domain_id: ${ROS_DOMAIN_ID:-auto}" | tee -a "$CYCLE_LOG"

ROS_LOG_DIR=/tmp/ros2_logs python3 src/kalman_filter_localization/tools/run_open_data_sweep.py \
  --bag-path "$BAG_PATH" \
  --param-grid-json "$PARAM_GRID_JSON" \
  --output-dir "$OUTPUT_DIR" \
  --imu-topic /ins_imu \
  --gnss-topic /gnss_pose \
  --ground-truth-topic /ins_pose \
  --play-topics /sensing/imu/imu_data /gnss/fix /lvx_client/gsof/ins_solution_49 \
  --enable-navsatfix-to-pose \
  --navsatfix-input-topic /gnss/fix \
  --navsatfix-output-topic /gnss_pose \
  --enable-applanix-to-pose \
  --applanix-input-topic /lvx_client/gsof/ins_solution_49 \
  --applanix-output-topic /ins_pose \
  --applanix-origin-navsatfix-topic /gnss/fix \
  --enable-applanix-to-imu \
  --applanix-imu-output-topic /ins_imu \
  --applanix-imu-output-mode ros \
  --initial-yaw-source-topic /ins_pose \
  --initial-yaw-source-msg-type pose_stamped \
  2>&1 | tee -a "$CYCLE_LOG"

SUMMARY_CSV="$OUTPUT_DIR/summary.csv"
mkdir -p "$(dirname "$SUMMARY_REGISTRY")"
touch "$SUMMARY_REGISTRY"
if ! grep -Fxq "$SUMMARY_CSV" "$SUMMARY_REGISTRY"; then
  echo "$SUMMARY_CSV" >> "$SUMMARY_REGISTRY"
fi

UPDATE_CMD=(python3 src/kalman_filter_localization/tools/update_agent_design_loop_report.py)
while IFS= read -r entry; do
  if [ -n "$entry" ] && [ -f "$entry" ]; then
    UPDATE_CMD+=(--summary-csv "$entry")
  fi
done < "$SUMMARY_REGISTRY"
"${UPDATE_CMD[@]}"

ACTUAL_ROS_DOMAIN_ID="$(sed -n 's/^ros_domain_id: //p' "$CYCLE_LOG" | head -n 1)"

echo "Cycle complete"
echo "  summary_csv: $SUMMARY_CSV"
echo "  cycle_label: $CYCLE_LABEL"
echo "  summary_registry: $SUMMARY_REGISTRY"
echo "  report_html: src/kalman_filter_localization/docs/agent_design_loop_report.html"
echo "  ros_domain_id: ${ACTUAL_ROS_DOMAIN_ID:-unknown}"
echo "  cycle_log: $CYCLE_LOG"
