# AeroSysSim
------------

AeroSysSim is a simulation tool written in C++17. The repository is structured as a reusable static library with deterministic unit tests and a thin app layer for CLI and I/O.

## Reposity layout
------------------
- include/            Public library headers
- src/                Library implementation (builds libaerosyssim.a)
- apps/               Executables (CLI, I/O). Current: sim_runner
- tests/              Framework-free unit tests registered with CTest
- analysis/            Python scripts for post-processing CSV artifacts
- scripts/             Convenience scripts (demo pipeline)
- artifacts/           Local run outputs

## Requirements
---------------
- C++17 compiler (tested with GCC toolchains on Linux)
- CMake 3.26+ recommended
- Python 3 for analysis scripts
  - analysis/plot_trace.py uses numpy and matplotlib

## Build and Test
-----------------
Default build is Release into ./build.

  ./dev_build.sh

Optional clean build:

  CLEAN=1 ./dev_build.sh

Build only (skip tests):

  RUN_TESTS=0 ./dev_build.sh

The build generates compile_commands.json in the build directory.

## Run sim_runner (CSV to stdout)
---------------------------------
sim_runner prints a deterministic CSV trace with header:

  t,qw,qx,qy,qz,wx,wy,wz

Example:

  ./build/bin/sim_runner --dt 0.01 --steps 10 --t0 0.0 --w 0.3 -0.2 0.1

Options:
- --dt <dt>
- --steps <N>
- --t0 <t0>
- --w <wx> <wy> <wz>
- --torque <tx> <ty> <tz>
- --torque-step <t_break> <tx0> <ty0> <tz0> <tx1> <ty1> <tz1>
- --scenario <principal_axis|coupled_rates>
- --output <path>
- --help

## Write CSV to a file
----------------------
Use --output to write the CSV trace to a file. This is recommended for analysis.

  mkdir -p artifacts
  ./build/bin/sim_runner --dt 0.01 --steps 200 --scenario coupled_rates \
    --torque-step 0.5 1 0 0 0 2 0 \
    --output artifacts/trace.csv

## Demo pipeline (build, run, plot)
-----------------------------------
The one-command demo script builds (optional), runs a deterministic case, writes artifacts into a timestamped folder,
and generates diagnostic plots.

  scripts/demo_attitude.sh

Skip rebuild:

  scripts/demo_attitude.sh --no-build

Override parameters via environment variables (examples):

  DT=0.02 STEPS=400 SCENARIO=principal_axis scripts/demo_attitude.sh --no-build
  T_BREAK=0.25 TAU0_X=0 TAU0_Y=0 TAU0_Z=1 TAU1_X=0 TAU1_Y=0 TAU1_Z=0 scripts/demo_attitude.sh --no-build

Outputs are written under:

  artifacts/demo_attitude_YYYYMMDD_HHMMSS/

Typical contents:
- trace.csv
- plots/w_components.png
- plots/q_norm.png
- meta.txt
- analysis_summary.txt

## Analysis (Python)
--------------------
To generate plots from a CSV trace:

  python3 analysis/plot_trace.py artifacts/trace.csv --outdir artifacts/plots

The script prints summary stats and writes:
- w_components.png (wx, wy, wz vs time)
- q_norm.png (quaternion norm vs time)

## Core Conventions
-------------------
These conventions are enforced by code and tests and should not drift:

- Quaternion storage order: wxyz
- Packed attitude state: [qw, qx, qy, qz, wx, wy, wz]
- Angular velocity is body-frame (w_body)
- Integrator: RK4 fixed step
- Control evaluation: time-varying control callback is evaluated at the start of each integration step (t_k, x_k)

## Tests
--------
 ctest --test-dir build --output-on-failure

Coverage includes:
- Quaternion ops: norm, normalize, multiply
- Linear algebra helpers: skew3 consistency
- RK4 step sanity
- Rigid-body RHS (diagonal inertia analytic cases)
- Attitude stepper (renormalization and determinism)
- Sim engine deterministic trace generation
- sim_runner deterministic CSV output
- Control schedule (piecewise torque) deterministic behavior




## QuickStart demo

```bash
# Build and run a timestemped demo bundle (CSV + plots + metadata)
chmod +x scripts/demo_attitude.sh
/scripts/demo_attitude.sh
