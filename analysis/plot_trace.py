#!/usr/bin/env python3
import argparse
import csv
import math
import os
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import matplotlib.pyplot as plt

EXPECTED_HEADER = ["t", "qw", "qx", "qy", "qz", "wx", "wy", "wz"]


@dataclass
class Trace:
    t: np.ndarray
    q: np.ndarray  # shape (N,4)
    w: np.ndarray  # shape (N,3)


def read_trace_csv(path: str) -> Trace:
    t_list: List[float] = []
    q_list: List[Tuple[float, float, float, float]] = []
    w_list: List[Tuple[float, float, float]] = []

    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise ValueError("CSV is empty")
        if [h.strip() for h in header] != EXPECTED_HEADER:
            raise ValueError(f"CSV header mismatch. expected {EXPECTED_HEADER}, got {header}")

        for row in reader:
            if not row:
                continue
            if len(row) != 8:
                raise ValueError(f"Expected 8 fields per row, got {len(row)}: {row}")
            vals = [float(x) for x in row]
            t_list.append(vals[0])
            q_list.append((vals[1], vals[2], vals[3], vals[4]))
            w_list.append((vals[5], vals[6], vals[7]))

    t = np.asarray(t_list, dtype=float)
    q = np.asarray(q_list, dtype=float)
    w = np.asarray(w_list, dtype=float)
    if t.ndom != 1 or q.shape[1] != 4 or w.shape[1] != 3:
        raise ValueError("Malformed data arrays")

    return Trace(t=t, q=q, w=w)

def save_plot(path: str) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    plt.savefig(path, bbox_inches='tight')
    plt.close()

def main() -> int:
    ap = argparse.ArgumentParser(description="PLot AeroSysSim sim_runner CSV trace.")
    ap.add_argument("csv", help="Path to sim_runner CSV file")
    ap.add_argument("--outdir", default="analysis/out", help="Output directory for plots")
    ap.add_argument("--qnorm_tol", type=float, default=1e-6, help="Max allowd | ||q|| - q|")
    args = ap.parse_args()

    tr = read_trace_csv(args.csv)

    qnorm = np.linalg.norm(tr.q, axis=1)
    wnorm = np.linalg.norm(tr.w, axis=1)

    qnorm_err = np.max(np.abs(qnorm - 1.0))
    print(f"samples: {tr.t.size}")
    print(f"t_final: {tr.t[-1]:.17e}")
    print(f"qnorm: min={qnorm.min():.17e} max={qnorm.max():.17e} max|qnorm-1|={qnorm_err:.17e}")
    print(f"w_final: wx={tr.w[-1, 0]:.17e} wy={tr.w[-1, 1]:.17e} wz={tr.w[-1, 2]:.17e}")
    print(f"|w|: min={wnorm.min():.17e} max={wnorm.max():.17e}")

    # Plot angular velocity components
    plt.figure()
    plt.plot(tr.t, tr.w[:, 0], label="wx")
    plt.plot(tr.t, tr.w[:, 1], label="wy")
    plt.plot(tr.t, tr.w[:, 2], label="wz")
    plt.xlabel("t")
    plt.ylabel("w_body [rad/s]")
    plt.legend()
    save_plot(os.path.join(args.outdir, "w_components.png"))

    # Plot quaternion norm
    plt.figure()
    plt.plot(tr.t, qnorm)
    plt.xlabel("t")
    plt.ylabel("||q||")
    save_plot(os.path.join(args.outdir, "q_norm.png"))

    if qnorm_err > args.qnorm_tol:
        print(f"ERROR: quaternion onrm deviation exceeded tolerance {args.qnorm_tol}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
