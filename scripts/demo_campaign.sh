#!/usr/bin/env bash
set -euo pipefail

# Demo batch runner for AeroSysSim campaingn mode
# Produces a timestamped artifact bundle under artifacts/

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build}"
SIM_BIN="${SIM_BIN:-${BUILD_DIR}/bin/sim_runner}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

ARTIFACTS_ROOT="${ARTIFACTS_ROOT:-${ROOT_DIR}/artifacts}"
OUT_TAG="${OUT_TAG:-demo_campaign}"
STAMP="$(date +"%Y%m%d_%H%M%S")"
RUN_DIR="${RUN_DIR:-${ARTIFACTS_ROOT}/${OUT_TAG}_${STAMP}}"

usage() {
	cat <<EOF
USAGE: scripts/demo_campaign.sh [--no-build]

Creates a timestamped run directory under:
	artifacts/demo_campaign_YYYMMDD__HHMMSS/

Outputs:
	batch_out/summary.csv
	batch_out/<case>/trace.csv
	batch_out/<case>/plots/*.png
	batch_out/plots/*.png
	meta.txt

Environment overrides:
	BUILD_DIR=build
	SIM_BIN=build/bin/sim_runner
	PYTHON_BIN=python3
	ARTIFACTS_ROOT=artifacts
EOF
}

DO_BUILD=1
if [[ "${1:-}" == "--no-build" ]]; then
	DO_BUILD=0
elif [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
	usage
	exit 0
elif [[ "${1:-}" != "" ]]; then
	echo "demo_campaign.sh: unknown argument: ${1}" >&2
	usage >&2
	exit 2
fi

mkdir -p "${RUN_DIR}"
META_FILE="${RUN_DIR}/meta.txt"
{
	echo "AeroSysSim demo_campaign run"
	echo "timestamp: ${STAMP}"
	echo "root_dir: ${ROOT_DIR}"
	echo "build_dir: ${BUILD_DIR}"
	echo "sim_bin: ${SIM_BIN}"
	echo "python_bin: ${PYTHON_BIN}"
} > "${META_FILE}"

echo "Run directory: ${RUN_DIR}"

if [[ "${DO_BUILD}" == "1" ]]; then
	echo "Building (Release)..."
	cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
	cmake --build "${BUILD_DIR}" -j "${NSLOTS:-1}"
fi

if [[ ! -x "${SIM_BIN}" ]]; then
	echo "ERROR: sim_runner not found/executable at: ${SIM_BIN}" >&2
	exit 1
fi

# Create a small campagin directory with a few cases.
CASES_DIR="${RUN_DIR}/cases"
BATCH_OUT="${RUN_DIR}/batch_out"
mkdir -p "${CASES_DIR}"
mkdir -p "${BATCH_OUT}"

# Case 1: diagonal inertia, torque step
cat > "${CASES_DIR}/case_diag_step.cfg" <<EOF
dt=0.01
t0=0.0
steps=200
scenario=coupled_rates
torque-step=0.5,1,0,0,0,2,0
inertia_diag=2,3,4
EOF

# Case 2: full inertia (row-major), different torque step
cat > "${CASES_DIR}/case_full_step.cfg"<<EOF
dt=0.01
t0=0.0
steps=200
scenario=coupled_rates
torque-step=0.5,0,1,0,0,0,1
inertia=2.0,0.2,0.0, 0.2,3.0,0.1, 0.0,0.1,4.0
EOF

# Case 3: constant torque, principal axis scenario
cat > "${CASES_DIR}/case_diag_const.cfg" <<EOF
dt=0.01
t0=0.0
steps=200
scenario=principal_axis
torque=0.0,0.0,1.0
inertia_diag=2,3,4
EOF

# Case 4: 0 torque diagonal inertia
cat > "${CASES_DIR}/case_free_diag.cfg" <<EOF
dt=0.01
t0=0.0
steps=200
scenario=coupled_rates
torque=0.0,0.0,0.0
inertia_diag=2,3,4
EOF

echo "Running batch: ${CASES_DIR} -> ${BATCH_OUT}"
LC_ALL=C "${SIM_BIN}" --batch "${CASES_DIR}" --batch-outdir "${BATCH_OUT}"

echo "Plotting campaign + per-case plots..."
LC_ALL=C "${PYTHON_BIN}" "${ROOT_DIR}/analysis/plot_campaign.py" "${BATCH_OUT}" --per-case --strict-grid \
	| tee "${RUN_DIR}/analysis_summary.txt"

echo "Done."
echo "Artifacts:"
echo "  ${BATCH_OUT}/summary.csv"
echo "  ${BATCH_OUT}/plots/*.png"
echo "  ${BATCH_OUT}/<case>/trace.csv"
echo "  ${BATCH_OUT}/<case>/plots/*.png"
echo "  ${META_FILE}"
echo "  ${RUN_DIR}/analysis_summary.txt"
