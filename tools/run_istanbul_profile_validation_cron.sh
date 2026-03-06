#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
OUTPUT_BASE="${KFL_VALIDATION_OUTPUT_BASE:-/tmp/kfl_istanbul_profile_validation_runs}"
THRESHOLDS_JSON="${KFL_VALIDATION_THRESHOLDS_JSON:-src/kalman_filter_localization/tools/istanbul_profile_validation_thresholds.json}"
STAMP="$(date '+%Y%m%d_%H%M%S')"
OUTPUT_DIR="${OUTPUT_BASE}/${STAMP}"
LOG_PATH="${OUTPUT_DIR}/validation.log"

mkdir -p "${OUTPUT_DIR}"

cd "${WORKSPACE_ROOT}"
set +u
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi
set -u

CMD=(
  python3
  src/kalman_filter_localization/tools/run_istanbul_profile_validation.py
  --output-dir
  "${OUTPUT_DIR}"
  --thresholds-json
  "${THRESHOLDS_JSON}"
)
if [ "$#" -gt 0 ]; then
  CMD+=(--bags "$@")
fi

{
  echo "workspace_root: ${WORKSPACE_ROOT}"
  echo "output_dir: ${OUTPUT_DIR}"
  echo "thresholds_json: ${THRESHOLDS_JSON}"
  echo "bags: ${*:-default(all-sensors-bag1..6_compressed)}"
} | tee "${LOG_PATH}"

ROS_LOG_DIR=/tmp/ros2_logs "${CMD[@]}" 2>&1 | tee -a "${LOG_PATH}"

ln -sfn "${OUTPUT_DIR}" "${OUTPUT_BASE}/latest"

{
  echo "latest_symlink: ${OUTPUT_BASE}/latest"
  echo "summary_csv: ${OUTPUT_DIR}/profile_validation_summary.csv"
} | tee -a "${LOG_PATH}"
