#!/usr/bin/env python3
import argparse
import csv
import os
import subprocess
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt

from plot_utils import save_simple_xy_plot, save_simple_xy_multi_scatter

@dataclass
class CaseRow:
    case: str
    config_path: str
    trace_path: str
    samples: int
    t_final: float
    has_inertia_full: bool
    qnorm_max_abs_err: float
    energy_rel_change: float
    Lnorm_rel_change: float
    wx_final: float
    wy_final: float
    wz_final: float
    w_norm_min: float
    w_norm_max: float


def _parse_bool(s: str) -> bool:
    v = s.strip().lower()
    return v in ("1", "true", "yes", "y", "on")

def read_kv_config(path: str) -> dict:
    out = {}
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            if "=" not in line:
                continue
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip()
    return out

def _parse_vec3(s: str):
    v = s.replace(",", " ").split()
    if len(v) != 3:
        raise ValueError(f"expected 3 values, got: {s}")
    return np.array([float(v[0]), float(v[1]), float(v[2])], dtype=float)

def _parse_torque_step7(s: str):
    v = s.replace(",", " ").split()
    if len(v) != 7:
        raise ValueError(f"expected 7 values, got: {s}")
    tb = float(v[0])
    tau0 = np.array([float(v[1]), float(v[2]), float(v[3])], dtype=float)
    tau1 = np.array([float(v[4]), float(v[5]), float(v[6])], dtype=float)
    return tb, tau0, tau1

def torque_meta_from_config(cfg_path: str):
    kv = read_kv_config(cfg_path)

    tau_const = None
    tau0 = None
    tau1 = None

    if "torque-step" in kv:
        _, tau0, tau1 = _parse_torque_step7(kv["torque-step"])
        tau_max = float(max(np.linalg.norm(tau0), np.linalg.norm(tau1)))
        if tau_max > 0.0:
            return "step", tau_max

    if "torque" in kv:
        tau_const = _parse_vec3(kv["torque"])
        tau_max = float(np.linalg.norm(tau_const))
        if tau_max > 0.0:
            return "const", tau_max

    return "none", 0.0

def _resolve_path(base_dir: str, p: str) -> str:
    if os.path.isabs(p):
        return p
    return os.path.normpath(os.path.join(base_dir, p))

def read_summary_csv(path: str) -> List[CaseRow]:
    base_dir = os.path.dirname(os.path.abspath(path))
    rows: List[CaseRow] = []

    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("summary.csv has no header")

        fields = set([h.strip() for h in reader.fieldnames])

        required = [
                "case", "config_path", "trace_path", "samples", "t_final", "has_inertia_full",
                "qnorm_max_abs_err", "energy_rel_change", "Lnorm_rel_change",
                "wx_final", "wy_final", "wz_final", "w_norm_min", "w_norm_max"
            ]
        for k in required:
            if k not in fields:
                raise ValueError(f"summary.csv missing requried column: {k}")

        for d in reader:
            if not d:
                continue
            case = d["case"].strip()
            cfgp = _resolve_path(base_dir, d["config_path"].strip())
            trcp = _resolve_path(base_dir, d["trace_path"].strip())
            rows.append(
                    CaseRow(
                        case=case,
                        config_path=cfgp,
                        trace_path=trcp,
                        samples=int(d["samples"]),
                        t_final=float(d["t_final"]),
                        has_inertia_full=_parse_bool(d["has_inertia_full"]),
                        qnorm_max_abs_err=float(d["qnorm_max_abs_err"]),
                        energy_rel_change=float(d["energy_rel_change"]),
                        Lnorm_rel_change=float(d["Lnorm_rel_change"]),
                        wx_final=float(d["wx_final"]),
                        wy_final=float(d["wy_final"]),
                        wz_final=float(d["wz_final"]),
                        w_norm_min=float(d["w_norm_min"]),
                        w_norm_max=float(d["w_norm_max"]),
                )
            )

    return rows

def ensure_outdir(p: str) -> None:
    os.makedirs(p, exist_ok=True)

def save_plot(path: str) -> None:
    plt.savefig(path, bbox_inches='tight')
    plt.close()

def read_trace_csv(path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    # returns (t, q_wxyz, w_wyz)
    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise ValueError(f"{path}: empty CSV")
        header = [h.strip() for h in header]
        expected = ["t", "qw", "qx", "qy", "qz", "wx", "wy", "wz"]
        if header != expected:
            raise ValueError(f"{path}: header mismatch. Expected {expected}, got {header}")

        t_list: List[float] = []
        q_list: List[Tuple[float, float, float, float]] = []
        w_list: List[Tuple[float, float, float]] = []
        for row in reader:
            if not row:
                continue
            if len(row) != 8:
                raise ValueError(f"{path}: expected 8 fields, got {len(row)}: {row}")
            vals = [float(x) for x in row]
            t_list.append(vals[0])
            q_list.append((vals[1], vals[2], vals[3], vals[4]))
            w_list.append((vals[5], vals[6], vals[7]))

    t = np.asarray(t_list, dtype=float)
    q = np.asarray(q_list, dtype=float)
    w = np.asarray(w_list, dtype=float)
    return t, q, w

def main() -> int:
    ap = argparse.ArgumentParser(description="Plot AeroSysSim batch campaign outputs (summary + traces).")
    ap.add_argument("batch_outdir", help="Directory that contains summary.csv and per-case outputs")
    ap.add_argument("--summary", default=None, help="Override path to summary.csv (default: <batch_outdir>/summary.csv)")
    ap.add_argument("--outdir", default=None, help="Output directory for campaign plots (default: <batch_outdir>/plots)")
    ap.add_argument("--per-case", action="store_true",
        help="Also generate per-case plots by calling analysis/plot_trace.py for each case")
    ap.add_argument("--per-case-subdir", default="plots", help="subdir under each case dir for per-case plots")
    ap.add_argument("--strict-grid", action="store_true", help="Require identical time grids acrosss traces")
    ap.add_argument("--max-overlay", type=int, default=0, help="if >0, limit overlays to first N cases")
    args = ap.parse_args()

    batch_outdir = os.path.abspath(args.batch_outdir)
    summary_path = os.path.abspath(args.summary) if args.summary else os.path.join(batch_outdir, "summary.csv")
    outdir = os.path.abspath(args.outdir) if args.outdir else os.path.join(batch_outdir, "plots")
    ensure_outdir(outdir)

    cases = read_summary_csv(summary_path)
    if len(cases) == 0:
        raise ValueError("No cases found in summary.csv")

    # -----------------------------
    # Summary arrays and metadata
    # -----------------------------
    n = len(cases)
    x_case = np.arange(n, dtype=float)
    names = [c.case for c in cases]

    # Torque metadata derived from each case config
    torque_mode_list = []
    tau_max_norm_list = []
    for c in cases:
        mode, tau = torque_meta_from_config(c.config_path)
        torque_mode_list.append(mode)           # expected in {"none","const","step"} (or subset)
        tau_max_norm_list.append(float(tau))    # ||tau|| max for the schedule
    torque_mode = np.asarray(torque_mode_list, dtype=str)
    tau_max_norm = np.asarray(tau_max_norm_list, dtype=float)

    # Core metrics from summary.csv
    qerr = np.asarray([c.qnorm_max_abs_err for c in cases], dtype=float)
    echange = np.asarray([c.energy_rel_change for c in cases], dtype=float)
    lchange = np.asarray([c.Lnorm_rel_change for c in cases], dtype=float)

    wx = np.asarray([c.wx_final for c in cases], dtype=float)
    wy = np.asarray([c.wy_final for c in cases], dtype=float)
    wz = np.asarray([c.wz_final for c in cases], dtype=float)
    w_final = np.sqrt(wx * wx + wy * wy + wz * wz)
    w_min = np.asarray([c.w_norm_min for c in cases], dtype=float)
    w_max = np.asarray([c.w_norm_max for c in cases], dtype=float)

    # Persist the case index mapping for human interpretation of plots
    case_index_path = os.path.join(outdir, "case_index.txt")
    with open(case_index_path, "w") as f:
        f.write("index,case\n")
        for i, nm in enumerate(names):
            f.write(f"{i},{nm}\n")

    # -----------------------------
    # Summary plots (single-series)
    # -----------------------------
    save_simple_xy_plot(
        x_case, qerr,
        xlabel="case index",
        ylabel="qnorm_max_abs_err",
        title="Quaternion norm max absolute error",
        outpath=os.path.join(outdir, "summary_qnorm_max_abs_err.png"),
        kind="scatter",
    )

    save_simple_xy_plot(
        x_case, echange,
        xlabel="case index",
        ylabel="energy_rel_change",
        title="Energy relative change",
        outpath=os.path.join(outdir, "summary_energy_rel_change.png"),
        kind="scatter",
    )

    save_simple_xy_plot(
        x_case, lchange,
        xlabel="case index",
        ylabel="Lnorm_rel_change",
        title="Angular momentum magnitude relative change",
        outpath=os.path.join(outdir, "summary_Lnorm_rel_change.png"),
        kind="scatter",
    )

    # w stats as multi-series scatter
    save_simple_xy_multi_scatter(
        [
            ("|w|_min", x_case, w_min),
            ("|w|_max", x_case, w_max),
            ("|w|_final", x_case, w_final),
        ],
        xlabel="case index",
        ylabel="|w| [rad/s]",
        title="Body-rate magnitude summary stats",
        outpath=os.path.join(outdir, "summary_w_stats.png"),
    )

    # Pareto-style scatter: energy change vs quaternion error
    save_simple_xy_plot(
        echange, qerr,
        xlabel="energy_rel_change",
        ylabel="qnorm_max_abs_err",
        title="Pareto: energy change vs quaternion error",
        outpath=os.path.join(outdir, "summary_pareto_energy_vs_qerr.png"),
        kind="scatter",
    )

    # -----------------------------
    # Summary plots (grouped views)
    # -----------------------------
    # Energy change by forcing mode
    series_E = []
    for mode in ("none", "const", "step"):
        m = (torque_mode == mode)
        if np.any(m):
            series_E.append((mode, x_case[m], echange[m]))
    if len(series_E) > 0:
        save_simple_xy_multi_scatter(
            series_E,
            xlabel="case index",
            ylabel="energy_rel_change",
            title="Energy relative change by forcing mode",
            outpath=os.path.join(outdir, "summary_energy_rel_change_by_mode.png"),
        )

    # L change by forcing mode
    series_L = []
    for mode in ("none", "const", "step"):
        m = (torque_mode == mode)
        if np.any(m):
            series_L.append((mode, x_case[m], lchange[m]))
    if len(series_L) > 0:
        save_simple_xy_multi_scatter(
            series_L,
            xlabel="case index",
            ylabel="Lnorm_rel_change",
            title="Angular momentum magnitude relative change by forcing mode",
            outpath=os.path.join(outdir, "summary_Lnorm_rel_change_by_mode.png"),
        )

    # Torque-free only (this is the conservation-style view)
    m_free = (torque_mode == "none")
    if np.any(m_free):
        save_simple_xy_plot(
            x_case[m_free], echange[m_free],
            xlabel="case index",
            ylabel="energy_rel_change",
            title="Energy relative change (torque-free only)",
            outpath=os.path.join(outdir, "summary_energy_rel_change_torque_free.png"),
            kind="scatter",
        )
        save_simple_xy_plot(
            x_case[m_free], lchange[m_free],
            xlabel="case index",
            ylabel="Lnorm_rel_change",
            title="Angular momentum magnitude relative change (torque-free only)",
            outpath=os.path.join(outdir, "summary_Lnorm_rel_change_torque_free.png"),
            kind="scatter",
        )

    # Forcing magnitude correlations (include "none" as a separate label for interpretability)
    series_tau_E = []
    series_tau_L = []
    for mode in ("const", "step", "none"):
        m = (torque_mode == mode)
        if np.any(m):
            series_tau_E.append((mode, tau_max_norm[m], echange[m]))
            series_tau_L.append((mode, tau_max_norm[m], lchange[m]))
    if len(series_tau_E) > 0:
        save_simple_xy_multi_scatter(
            series_tau_E,
            xlabel="max ||tau||",
            ylabel="energy_rel_change",
            title="Energy relative change vs forcing magnitude",
            outpath=os.path.join(outdir, "summary_tau_vs_energy_rel_change.png"),
        )
    if len(series_tau_L) > 0:
        save_simple_xy_multi_scatter(
            series_tau_L,
            xlabel="max ||tau||",
            ylabel="Lnorm_rel_change",
            title="Angular momentum magnitude relative change vs forcing magnitude",
            outpath=os.path.join(outdir, "summary_tau_vs_Lnorm_rel_change.png"),
        )

    # Generate per-case plots
    if args.per_case:
        plot_trace_py = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plot_trace.py")
        for c in cases:
            case_dir = os.path.dirname(os.path.abspath(c.trace_path))
            case_plot_dir = os.path.join(case_dir,args.per_case_subdir)
            ensure_outdir(case_plot_dir)
            cmd = [sys.executable, plot_trace_py, c.trace_path, "--outdir", case_plot_dir]
            subprocess.run(cmd, check=True)

    # -----------------------------
    # Trace-aware plots (overlays + heatmap)
    # -----------------------------
    t_ref: Optional[np.ndarray] = None
    grids_match = True
    wmag_list: List[np.ndarray] = []
    qnorm_list: List[np.ndarray] = []

    overlay_n = n
    if args.max_overlay and args.max_overlay > 0:
        overlay_n = min(overlay_n, args.max_overlay)

    for i, c in enumerate(cases):
        t, q, w = read_trace_csv(c.trace_path)
        if t_ref is None:
            t_ref = t
        else:
            same = (t.shape == t_ref.shape) and np.allclose(t, t_ref, rtol=0.0, atol=0.0)
            if not same:
                grids_match = False
                if args.strict_grid:
                    raise ValueError(
                        f"time grid mismatch for case '{c.case}' "
                        f"(use --strict-grid only if all cases share fixed dt/steps)"
                    )

        wmag_list.append(np.linalg.norm(w, axis=1))
        qnorm_list.append(np.linalg.norm(q, axis=1))

    if t_ref is None:
        raise ValueError("No traces loaded")

    # Overlays only make sense if grids match (otherwise plot(t_ref, y_i) is invalid)
    if grids_match:
        plt.figure()
        for i in range(overlay_n):
            plt.plot(t_ref, wmag_list[i])
        plt.xlabel("t")
        plt.ylabel("|w| [rad/s]")
        save_plot(os.path.join(outdir, "overlay_w_mag.png"))

        plt.figure()
        for i in range(overlay_n):
            plt.plot(t_ref, qnorm_list[i])
        plt.xlabel("t")
        plt.ylabel("||q||")
        save_plot(os.path.join(outdir, "overlay_q_norm.png"))

        # Heatmap requires equal-length series
        W = np.vstack(wmag_list)  # (num_cases, num_samples)
        plt.figure()
        plt.imshow(
            W,
            aspect="auto",
            origin="lower",
            extent=[float(t_ref[0]), float(t_ref[-1]), 0.0, float(n - 1)],
        )
        plt.xlabel("t")
        plt.ylabel("case index")
        plt.title("|w| heatmap")
        plt.colorbar()
        save_plot(os.path.join(outdir, "heatmap_w_mag.png"))
    else:
        # Still informative to write a small note for the operator
        with open(os.path.join(outdir, "trace_grid_note.txt"), "w") as f:
            f.write("Traces do not share an identical time grid.\n")
            f.write("Overlays and heatmap were skipped.\n")
            f.write("Use --strict-grid only for fixed dt/steps campaigns.\n")

    print(f"cases: {len(cases)}")
    print(f"campaign plots outdir: {outdir}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())




