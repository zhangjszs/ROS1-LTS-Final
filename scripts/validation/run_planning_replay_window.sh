#!/usr/bin/env bash
set -euo pipefail

# Fixed-window planning replay recorder + metrics evaluation.
#
# Usage:
#   OUT_DIR=/tmp/planning_track_tuning ./scripts/run_planning_replay_window.sh <name> [bag] [rate] [record_s]
#
# Notes:
# - Replays mission stack (planning+localization) with control disabled.
# - Records a fixed bag-time window aligned by /clock (default), falling back to wall-time if /clock is unavailable.
# - Writes:
#     <name>_topics.bag
#     <name>_rosparam_dump.yaml
#     <name>_planning_metrics.json
#     <name>_cone_map_metrics.json (best-effort)

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT_DIR="${OUT_DIR:-/tmp/planning_track_tuning}"
NAME="${1:?name required}"
BAG_PATH="${2:-/home/kerwin/rosbag/track.bag}"
PLAY_RATE="${3:-2.0}"
RECORD_S="${4:-70}"
TIMEOUT_S="${TIMEOUT_S:-110}"
# Align by /clock (bag-time window). Override via env if needed.
CLOCK_START_OFFSET_S="${CLOCK_START_OFFSET_S:-20.0}"
CLOCK_WINDOW_S="${CLOCK_WINDOW_S:-70.0}"
PRE_ROLL_S="${PRE_ROLL_S:-2.0}"
POST_ROLL_S="${POST_ROLL_S:-2.0}"

mkdir -p "${OUT_DIR}"
rm -f "${OUT_DIR}/${NAME}_launch.log" \
      "${OUT_DIR}/${NAME}_record.log" \
      "${OUT_DIR}/${NAME}_topics.bag" \
      "${OUT_DIR}/${NAME}_topics.bag.active" \
      "${OUT_DIR}/${NAME}_rosparam_dump.yaml" \
      "${OUT_DIR}/${NAME}_planning_metrics.json" \
      "${OUT_DIR}/${NAME}_cone_map_metrics.json"
rm -rf "${OUT_DIR}/roslog_${NAME}" && mkdir -p "${OUT_DIR}/roslog_${NAME}"

source /opt/ros/noetic/setup.bash
if [[ -f "${ROOT_DIR}/devel/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ROOT_DIR}/devel/setup.bash"
elif [[ -f "${ROOT_DIR}/devel/setup.sh" ]]; then
  # shellcheck source=/dev/null
  source "${ROOT_DIR}/devel/setup.sh"
fi
if [[ -f "${ROOT_DIR}/devel/.private/fsd_launch/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ROOT_DIR}/devel/.private/fsd_launch/setup.bash"
elif [[ -f "${ROOT_DIR}/devel/.private/planning_ros/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ROOT_DIR}/devel/.private/planning_ros/setup.bash"
fi

export ROS_LOG_DIR="${OUT_DIR}/roslog_${NAME}"

MISSION_STACK_LAUNCH="$(rospack find fsd_launch)/launch/subsystems/mission_stack.launch"

clock_ns() {
  local msg
  msg="$(timeout 3s rostopic echo -n 1 /clock 2>/dev/null || true)"
  if [[ -z "${msg}" ]]; then
    return 1
  fi
  python3 -c '
import re,sys
s=sys.stdin.read()
m=re.search(r"clock:\s*([0-9]+(?:\.[0-9]+)?)", s)
if m:
  v=m.group(1)
  if "." in v:
    secs, frac = v.split(".", 1)
    frac = (frac + "0"*9)[:9]
  else:
    secs, frac = v, "0"*9
  print(int(secs)*1_000_000_000 + int(frac))
  raise SystemExit(0)
secs_m=re.search(r"secs:\s*([0-9]+)", s)
nsecs_m=re.search(r"nsecs:\s*([0-9]+)", s)
if secs_m and nsecs_m:
  print(int(secs_m.group(1))*1_000_000_000 + int(nsecs_m.group(1)))
  raise SystemExit(0)
raise SystemExit(1)
' <<<"${msg}"
}

sec_to_ns() {
  python3 -c 'import sys; print(int(float(sys.argv[1]) * 1_000_000_000))' "$1"
}

ns_to_sec() {
  python3 -c 'import sys; print(float(int(sys.argv[1]))/1_000_000_000.0)' "$1"
}

bag_start_ns() {
  local bag_path="$1"
  python3 - <<'PY' "${bag_path}"
import sys
try:
  import rosbag
except Exception as e:
  raise SystemExit(1)
bag_path = sys.argv[1]
with rosbag.Bag(bag_path, "r") as bag:
  t = float(bag.get_start_time())
print(int(t * 1_000_000_000))
PY
}

cleanup() {
  if [[ -n "${REC_PID:-}" ]]; then
    kill "${REC_PID}" 2>/dev/null || true
    wait "${REC_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "[REPLAY] name=${NAME}"
echo "[REPLAY] bag=${BAG_PATH} rate=${PLAY_RATE} record_s=${RECORD_S} out_dir=${OUT_DIR}"

bag_start_ns_val=""
if bag_start_ns_val="$(bag_start_ns "${BAG_PATH}" 2>/dev/null)"; then
  echo "[REPLAY] bag_start_ns=${bag_start_ns_val}"
else
  bag_start_ns_val=""
  echo "[REPLAY] WARN: failed to read bag_start_ns; falling back to relative /clock window."
fi

timeout "${TIMEOUT_S}s" roslaunch "${MISSION_STACK_LAUNCH}" \
  simulation:=true \
  bag:="${BAG_PATH}" \
  rate:="${PLAY_RATE}" \
  loop:=false \
  launch_rviz:=false \
  launch_viz:=false \
  enable_control:=false \
  > "${OUT_DIR}/${NAME}_launch.log" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 30); do
  if rostopic list >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

# Reduce variance: wait until data actually flows.
if ! timeout 30s rostopic echo -n 1 /clock >/dev/null 2>&1; then
  echo "[REPLAY] ERROR: /clock not received (rosbag play may have failed)."
  exit 1
fi
if ! timeout 30s rostopic echo -n 1 /localization/cone_map >/dev/null 2>&1; then
  echo "[REPLAY] ERROR: /localization/cone_map not received."
  exit 1
fi
if ! timeout 30s rostopic echo -n 1 /planning/pathlimits >/dev/null 2>&1; then
  echo "[REPLAY] ERROR: /planning/pathlimits not received."
  exit 1
fi

timeout 20s rosparam dump "${OUT_DIR}/${NAME}_rosparam_dump.yaml" >/dev/null 2>&1 || true

offset_ns="$(sec_to_ns "${CLOCK_START_OFFSET_S}")"
window_ns="$(sec_to_ns "${CLOCK_WINDOW_S}")"
pre_roll_ns="$(sec_to_ns "${PRE_ROLL_S}")"
post_roll_ns="$(sec_to_ns "${POST_ROLL_S}")"
start_ns=""
target_ns=""
window_start_ns=""
window_end_ns=""
record_start_ns=""
record_end_ns=""

start0_ns=""
for _ in $(seq 1 80); do
  if start0_ns="$(clock_ns)"; then
    break
  fi
  sleep 0.05
done

if [[ -n "${start0_ns}" ]]; then
  if [[ -n "${bag_start_ns_val}" ]]; then
    window_start_ns="$((bag_start_ns_val + offset_ns))"
    window_end_ns="$((window_start_ns + window_ns))"
    record_start_ns="$((window_start_ns - pre_roll_ns))"
    record_end_ns="$((window_end_ns + post_roll_ns))"
    if (( record_start_ns < bag_start_ns_val )); then
      record_start_ns="$bag_start_ns_val"
    fi
    echo "[REPLAY] waiting /clock to reach absolute record start (window-start - ${PRE_ROLL_S}s)..."
  else
    start_ns="$start0_ns"
    target_ns="$((start_ns + offset_ns))"
    echo "[REPLAY] waiting /clock to reach offset (+${CLOCK_START_OFFSET_S}s) to define window..."
  fi
  for _ in $(seq 1 400); do
    now_ns="$(clock_ns)" || { sleep 0.05; continue; }
    if [[ -n "${window_start_ns}" ]]; then
      if (( now_ns >= record_start_ns )); then
        break
      fi
    elif (( now_ns >= target_ns )); then
      window_start_ns="$now_ns"
      window_end_ns="$((window_start_ns + window_ns))"
      record_start_ns="$window_start_ns"
      record_end_ns="$((window_end_ns + post_roll_ns))"
      break
    fi
    sleep 0.05
  done
  echo "[REPLAY] window by /clock: ${CLOCK_WINDOW_S}s (from ${window_start_ns} to ${window_end_ns} ns)"
  echo "[REPLAY] recording with pre/post roll: ${PRE_ROLL_S}s/${POST_ROLL_S}s (from ${record_start_ns} to ${record_end_ns} ns)"
  rosbag record -O "${OUT_DIR}/${NAME}_topics.bag" \
    /localization/cone_map \
    /planning/pathlimits \
    > "${OUT_DIR}/${NAME}_record.log" 2>&1 &
  REC_PID=$!
	  for _ in $(seq 1 2000); do
	    now_ns="$(clock_ns)" || { sleep 0.05; continue; }
	    if (( now_ns >= record_end_ns )); then
	      break
	    fi
	    sleep 0.05
	  done

	  # Stop recording as soon as the window (with roll) is covered to reduce artifacts size/variance.
	  kill "${REC_PID}" 2>/dev/null || true
	  wait "${REC_PID}" 2>/dev/null || true
	  unset REC_PID
	else
	  echo "[REPLAY] /clock not available; recording topics for ${RECORD_S}s wall-time..."
	  timeout "${RECORD_S}s" rosbag record -O "${OUT_DIR}/${NAME}_topics.bag" \
	    /localization/cone_map \
    /planning/pathlimits \
    > "${OUT_DIR}/${NAME}_record.log" 2>&1 &
  REC_PID=$!
fi

	wait "${LAUNCH_PID}" || true
	unset LAUNCH_PID

	if [[ -n "${REC_PID:-}" ]]; then
	  kill "${REC_PID}" 2>/dev/null || true
	  wait "${REC_PID}" 2>/dev/null || true
	  unset REC_PID
	fi

	if [[ -n "${window_start_ns}" && -n "${window_end_ns}" ]]; then
	  window_t0_s="$(ns_to_sec "${window_start_ns}")"
	  window_t1_s="$(ns_to_sec "${window_end_ns}")"
	  python3 "${ROOT_DIR}/perf_reports/scripts/evaluate_planning_metrics.py" \
	    "${OUT_DIR}/${NAME}_topics.bag" \
	    --t0 "${window_t0_s}" --t1 "${window_t1_s}" \
	    -o "${OUT_DIR}/${NAME}_planning_metrics.json"

	  python3 "${ROOT_DIR}/perf_reports/scripts/evaluate_cone_map_metrics.py" \
	    "${OUT_DIR}/${NAME}_topics.bag" \
	    --t0 "${window_t0_s}" --t1 "${window_t1_s}" \
	    -o "${OUT_DIR}/${NAME}_cone_map_metrics.json" \
	    >/dev/null 2>&1 || true
	else
	  python3 "${ROOT_DIR}/perf_reports/scripts/evaluate_planning_metrics.py" \
	    "${OUT_DIR}/${NAME}_topics.bag" \
	    -o "${OUT_DIR}/${NAME}_planning_metrics.json"

	  python3 "${ROOT_DIR}/perf_reports/scripts/evaluate_cone_map_metrics.py" \
	    "${OUT_DIR}/${NAME}_topics.bag" \
	    -o "${OUT_DIR}/${NAME}_cone_map_metrics.json" \
	    >/dev/null 2>&1 || true
	fi

echo "[REPLAY] artifacts:"
echo "  - ${OUT_DIR}/${NAME}_launch.log"
echo "  - ${OUT_DIR}/${NAME}_record.log"
echo "  - ${OUT_DIR}/${NAME}_topics.bag"
echo "  - ${OUT_DIR}/${NAME}_rosparam_dump.yaml"
echo "  - ${OUT_DIR}/${NAME}_planning_metrics.json"
echo "  - ${OUT_DIR}/${NAME}_cone_map_metrics.json (best-effort)"
