#!/usr/bin/env bash
# Source checkout launcher; no installation, rebinding, logging or auto-enabling.
set -Eeuo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python}"
mode="${1:-serve}"
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in
  serve|preview|inspect|follow|monitor|experiment|teleop) default_profile="configs/commissioning.local.json"; default_binding="configs/commissioning.binding.json" ;;
  run) default_profile="configs/feedback.reviewed.local.json"; default_binding="configs/feedback.reviewed.binding.json" ;;
  -h|--help)
    echo 'Usage: bash scripts/run_feedback.sh [serve|preview|inspect|follow|monitor|experiment|run|teleop] [CLI options]'
    echo 'Default: serve on 127.0.0.1:8766, waiting for an explicit mode selection.'
    echo 'follow/monitor/run enable motors and follow. run requires a separately reviewed force profile.'
    echo 'teleop uses the original lumos-lerobot entry with explicit feedback options (default: monitor).'
    echo 'Environment: PYTHON_BIN, FEEDBACK_PROFILE, FEEDBACK_BINDING, FEEDBACK_PORT, TELEOP_CONFIG.'
    exit 0 ;;
  *) echo "Unknown mode: $mode" >&2; exit 2 ;;
esac
cd -- "$PROJECT_DIR"
export PYTHONPATH="$PROJECT_DIR/src${PYTHONPATH:+:$PYTHONPATH}"
if [[ "$mode" == "teleop" ]]; then
  exec "$PYTHON_BIN" -B -m lerobot_robot_lumos_stararm102.cli teleop \
    --config "${TELEOP_CONFIG:-configs/session.local.json}" \
    --feedback-profile "${FEEDBACK_PROFILE:-$default_profile}" \
    --feedback-binding "${FEEDBACK_BINDING:-$default_binding}" "$@"
fi
exec "$PYTHON_BIN" -B -m stararm_lumos_feedback.cli "$mode" \
  --profile "${FEEDBACK_PROFILE:-$default_profile}" \
  --binding "${FEEDBACK_BINDING:-$default_binding}" \
  --port "${FEEDBACK_PORT:-8766}" "$@"
