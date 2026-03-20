#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VALIDATE_SCRIPT="${ROOT_DIR}/scripts/validate_pathlimits_v2_cutover_replay.sh"

OUT_DIR="${OUT_DIR:-${ROOT_DIR}/perf_reports/results/pathlimits_v2_cutover_matrix_$(date +%Y%m%d_%H%M%S)}"
PLAY_RATE="${PLAY_RATE:-2.0}"
BOOT_TIMEOUT_S="${BOOT_TIMEOUT_S:-30}"
SCENARIO_TIMEOUT_S="${SCENARIO_TIMEOUT_S:-120}"
MISSIONS_CSV="${MISSIONS_CSV:-trackdrive,autocross,acceleration,skidpad}"

usage() {
  cat <<'EOF'
Usage:
  scripts/run_pathlimits_v2_cutover_matrix.sh [options]

Options:
  --missions <m1,m2,...>      Missions to run (default: trackdrive,autocross,acceleration,skidpad)
  --out-dir <path>            Output root directory
  --rate <float>              rosbag replay rate (default: 2.0)
  --boot-timeout <sec>        Boot timeout per mission (default: 30)
  --scenario-timeout <sec>    Scenario timeout per mission (default: 120)
  -h, --help                  Show help

Environment:
  PATHLIMITS_WAIT_S           Optional wait override passed through to validate script.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --missions)
      MISSIONS_CSV="${2:-}"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="${2:-}"
      shift 2
      ;;
    --rate)
      PLAY_RATE="${2:-}"
      shift 2
      ;;
    --boot-timeout)
      BOOT_TIMEOUT_S="${2:-}"
      shift 2
      ;;
    --scenario-timeout)
      SCENARIO_TIMEOUT_S="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] unknown argument: $1"
      usage
      exit 2
      ;;
  esac
done

if [[ ! -x "${VALIDATE_SCRIPT}" ]]; then
  echo "[ERROR] validate script not executable: ${VALIDATE_SCRIPT}"
  exit 2
fi

mkdir -p "${OUT_DIR}"
MATRIX_TSV="${OUT_DIR}/matrix.tsv"
MATRIX_SUMMARY_MD="${OUT_DIR}/summary.md"
MATRIX_JSON="${OUT_DIR}/matrix.json"

printf "mission\tstatus\tresults_tsv\tsummary_md\tresults_json\n" > "${MATRIX_TSV}"

IFS=',' read -r -a MISSIONS <<< "${MISSIONS_CSV}"
overall_status=0

for mission in "${MISSIONS[@]}"; do
  mission="$(echo "${mission}" | xargs)"
  if [[ -z "${mission}" ]]; then
    continue
  fi

  mission_dir="${OUT_DIR}/${mission}"
  mkdir -p "${mission_dir}"
  echo "[pathlimits-v2-matrix] mission=${mission} out=${mission_dir}"

  run_ok="FAIL"
  if [[ -n "${PATHLIMITS_WAIT_S:-}" ]]; then
    if PATHLIMITS_WAIT_S="${PATHLIMITS_WAIT_S}" bash "${VALIDATE_SCRIPT}" \
      --mission "${mission}" \
      --rate "${PLAY_RATE}" \
      --boot-timeout "${BOOT_TIMEOUT_S}" \
      --scenario-timeout "${SCENARIO_TIMEOUT_S}" \
      --out-dir "${mission_dir}"; then
      run_ok="PASS"
    fi
  else
    if bash "${VALIDATE_SCRIPT}" \
      --mission "${mission}" \
      --rate "${PLAY_RATE}" \
      --boot-timeout "${BOOT_TIMEOUT_S}" \
      --scenario-timeout "${SCENARIO_TIMEOUT_S}" \
      --out-dir "${mission_dir}"; then
      run_ok="PASS"
    fi
  fi

  if [[ "${run_ok}" != "PASS" ]]; then
    overall_status=1
  fi

  printf "%s\t%s\t%s\t%s\t%s\n" \
    "${mission}" \
    "${run_ok}" \
    "${mission_dir}/results.tsv" \
    "${mission_dir}/summary.md" \
    "${mission_dir}/results.json" >> "${MATRIX_TSV}"
done

python3 - "${MATRIX_TSV}" "${MATRIX_SUMMARY_MD}" "${MATRIX_JSON}" <<'PY'
import csv
import json
import pathlib
import sys

matrix_tsv = pathlib.Path(sys.argv[1])
summary_md = pathlib.Path(sys.argv[2])
matrix_json = pathlib.Path(sys.argv[3])

rows = []
with matrix_tsv.open("r", encoding="utf-8") as f:
    reader = csv.DictReader(f, delimiter="\t")
    for row in reader:
        rows.append(row)

matrix_json.write_text(json.dumps(rows, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

lines = []
lines.append("# PathLimitsV2 Cutover Replay Matrix")
lines.append("")
lines.append("| mission | status | results_tsv | summary_md | results_json |")
lines.append("|---|---|---|---|---|")
for row in rows:
    lines.append(
        f"| {row['mission']} | {row['status']} | {row['results_tsv']} | "
        f"{row['summary_md']} | {row['results_json']} |"
    )

summary_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
PY

echo "[pathlimits-v2-matrix] matrix: ${MATRIX_TSV}"
echo "[pathlimits-v2-matrix] summary: ${MATRIX_SUMMARY_MD}"
echo "[pathlimits-v2-matrix] json: ${MATRIX_JSON}"

if [[ "${overall_status}" -eq 0 ]]; then
  echo "[pathlimits-v2-matrix] ACCEPTANCE PASS"
else
  echo "[pathlimits-v2-matrix] ACCEPTANCE FAIL"
fi

exit "${overall_status}"
