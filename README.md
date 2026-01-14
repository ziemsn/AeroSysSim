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
  - analysis/plot_trace.py and analysis/plot_campaign.py use numpy and matplotlib

## Build and Test
-----------------
Default build is Release into ./build.

  ./dev_build.sh

Optional clean build:

  CLEAN=1 ./dev_build.sh

Build only (skip tests):

  RUN_TESTS=0 ./dev_build.sh

The build generates compile_commands.json in the build directory.

## Run sim_runner (CSV to stdout or file)
---------------------------------
sim_runner prints a deterministic CSV trace with header:

  t,qw,qx,qy,qz,wx,wy,wz

Example (CSV to stdout):

  ./build/bin/sim_runner --dt 0.01 --steps 10 --t0 0.0 --w 0.3 -0.2 0.1

Example (CSV to a file):
  mkdir -p artifacts
  ./build/bin/sim_runner --dt 0.01 --steps 200 --scenario coupled_rates \
    --torque-step 0.5 1 0 0 0 2 0 \
    --output artifacts/trace.csv

CLI Options:
- --dt <dt>
- --steps <N>
- --t0 <t0>
- --w <wx> <wy> <wz>
- --torque <tx> <ty> <tz>
- --torque-step <t_break> <tx0> <ty0> <tz0> <tx1> <ty1> <tz1>
- --scenario <principal_axis|coupled_rates>
- --output <path>
- --output <path>
- --help

## sim_runner config files 
--------------------------
The --config file format is line:based, with optional comments starting with #
Values may be comma- or space-separated depending on the key

Common keys:
- dt=<float>
- t0=<float>
- steps=<int>
- scenario=principal_axis|coupled_rates
- w=<wx,wy,wz>
- torque=<tx,ty,tz>
- torque-step=<t_break,tx0,ty0,tz0,tx1,ty1,tz1>
- output=<path>            If empty or "stdout", outputs to stdout

Inertia keys:
- inertia_diag=<Ixx,Iyy,Izz>
- inertia=<a11,a12,a13,a21,a22,a23,a31,a32,a33>   (9 numbers, row-major)

Notes:
- CLI flags override config values regardless of argument ordering.
- In batch mode, sim_runner overwrites per-case output paths for hygiene.

## Batch campaign mode
---------------------
Campaign mode runs all *.cfg files in a directory, writes a per-case trace.csv under an output directory,
and emits a summary.csv for cross-case analysis.

Run:

  ./build/bin/sim_runner --batch <cases_dir> --batch-outdir <batch_outdir>

Example:

  ./build/bin/sim_runner --batch artifacts/cases --batch-outdir artifacts/batch_out

Output layout:
- <batch_outdir>/summary.csv
- <batch_outdir>/<case_name>/trace.csv
- <batch_outdir>/<case_name>/config.cfg (copied or referenced, depending on implementation)
- <batch_outdir>/<case_name>/plots/ (optional, if you run plotting with --per-case)

summary.csv columns:
  case,config_path,trace_path,samples,t_final,has_inertia_full,
  qnorm_max_abs_err,energy_rel_change,Lnorm_rel_change,
  wx_final,wy_final,wz_final,w_norm_min,w_norm_max

Interpretation:
- energy_rel_change and Lnorm_rel_change are relative max deviations over the trace
  normalized by the initial value magnitude (with a small epsilon floor).

## Demo pipelines
-----------------

### Single-case demo (build, run, plot)
scripts/demo_attitude.sh builds (optional), runs a deterministic case, writes artifacts into a timestamped folder,
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
- plots/w_mag.png
- plots/w_components.png
- plots/q_norm.png
- plots/q_components.png
- plots/euler_zyx_deg.png (unless disabled)
- plots/*phase.png (unless disabled)
- meta.txt
- analysis_summary.txt

### Campaign demo (batch + campaign plots)
scripts/demo_campaign.sh builds (optional), generates a small cases/ directory, runs batch mode,
and produces cross-case campaign plots. Per-case plots are produced if the campaign plotting is run with --per-case.

  scripts/demo_campaign.sh

Skip rebuild:

  scripts/demo_campaign.sh --no-build

Outputs are written under:

  artifacts/demo_campaign_YYYYMMDD_HHMMSS/

Typical contents:
- batch_out/summary.csv
- batch_out/plots/*.png
- batch_out/<case>/trace.csv
- batch_out/<case>/plots/*.png (if enabled)
- meta.txt
- analysis_summary.txt

## Analysis (Python)
--------------------

### Plot a single trace CSV
  python3 analysis/plot_trace.py artifacts/trace.csv --outdir artifacts/plots

Common options:
- --no_euler          Skip Euler angle plot
- --no_phase          Skip body-rate phase plots
- --inertia IXX IYY IZZ   Plot rotational kinetic energy for diagonal inertia
- --qnorm_tol <tol>   Return nonzero if quaternion norm error exceeds tol

Outputs include:
- w_mag.png
- w_components.png
- q_norm.png
- q_components.png
- euler_zyx_deg.png (unless disabled)
- wxy_phase.png, wxz_phase.png, wyz_phase.png (unless disabled)
- rot_kinetic_energy.png (if --inertia is provided)

### Plot a campaign batch_out directory
  python3 analysis/plot_campaign.py artifacts/batch_out --outdir artifacts/batch_out/plots --per-case

Common options:
- --per-case              Call plot_trace.py for each case trace
- --strict-grid           Require identical time grids across traces (use only if dt/steps fixed)
- --max-overlay N         Limit overlay plots to the first N cases


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
- Config parsing 
- Batch campaign invariants summary generation


## QuickStart
-------------
./dev_build.sh
scripts/demo_attitude.sh
scripts/demo_campaign.sh
