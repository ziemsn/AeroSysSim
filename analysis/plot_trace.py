#!/usr/bin/env python3
import argparse
import csv
import os
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np
import matplotlib.pyplot as plt


EXPECTED_HEADER = ["t", "qw", "qx", "qy", "qz", "wx", "wy", "wz"]


@dataclass
class Trace:
    t: np.ndarray
    q: np.ndarray  # (N,4) wxyz
    w: np.ndarray  # (N,3) body rates


def read_trace_csv(path: str) -> Trace:
    t_list: List[float] = []
    q_list: List[Tuple[float, float, float, float]] = []
    w_list: List[Tuple[float, float, float]] = []

    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise ValueError("CSV is empty")
        header = [h.strip() for h in header]
        if header != EXPECTED_HEADER:
            raise ValueError(f"CSV header mismatch. Expected {EXPECTED_HEADER}, got {header}")

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

    if t.size == 0:
        raise ValueError("No data rows in CSV")
    return Trace(t=t, q=q, w=w)


def ensure_outdir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def save_plot(path: str) -> None:
    plt.savefig(path, dpi=160, bbox_inches="tight")
    plt.close()


def quat_to_rotmat_body_to_inertial(q_wxyz: np.ndarray) -> np.ndarray:
    # q = [w, x, y, z], body to inertial
    w, x, y, z = q_wxyz
    ww = w*w
    xx = x*x
    yy = y*y
    zz = z*z

    wx = w*x
    wy = w*y
    wz = w*z
    xy = x*y
    xz = x*z
    yz = y*z

    # Rotation matrix R such that v_I = R * v_B
    R = np.array([
        [ww + xx - yy - zz, 2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),       ww - xx + yy - zz, 2*(yz - wx)],
        [2*(xz - wy),       2*(yz + wx),       ww - xx - yy + zz],
    ], dtype=float)
    return R


def rotmat_to_euler_zyx(R: np.ndarray) -> Tuple[float, float, float]:
    # Returns yaw, pitch, roll in radians for R = Rz(yaw) * Ry(pitch) * Rx(roll)
    # where R maps body to inertial.
    # Handles numerical safety via clipping.
    pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    yaw = np.arctan2(R[1, 0], R[0, 0])
    roll = np.arctan2(R[2, 1], R[2, 2])
    return yaw, pitch, roll


def main() -> int:
    ap = argparse.ArgumentParser(description="Plot AeroSysSim sim_runner CSV trace.")
    ap.add_argument("csv", help="Path to sim_runner CSV file")
    ap.add_argument("--outdir", default="analysis/out", help="Output directory for plots")
    ap.add_argument("--qnorm_tol", type=float, default=1e-6, help="Max allowed | ||q|| - 1 |")
    ap.add_argument("--no_euler", action="store_true", help="Skip Euler angle plot")
    ap.add_argument("--inertia", nargs=3, type=float, metavar=("IXX", "IYY", "IZZ"),
                    help="If provided, plot rotational kinetic energy for diagonal inertia")
    args = ap.parse_args()

    tr = read_trace_csv(args.csv)
    ensure_outdir(args.outdir)

    # Basic diagnostics
    if np.any(np.diff(tr.t) <= 0.0):
        raise ValueError("Time vector is not strictly increasing")

    qnorm = np.linalg.norm(tr.q, axis=1)
    wnorm = np.linalg.norm(tr.w, axis=1)
    qnorm_err = float(np.max(np.abs(qnorm - 1.0)))

    print(f"samples: {tr.t.size}")
    print(f"t_final: {tr.t[-1]:.17e}")
    print(f"qnorm: min={qnorm.min():.17e} max={qnorm.max():.17e} max|qnorm-1|={qnorm_err:.17e}")
    print(f"w_final: wx={tr.w[-1,0]:.17e} wy={tr.w[-1,1]:.17e} wz={tr.w[-1,2]:.17e}")
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

    # Plot quaternion components
    plt.figure()
    plt.plot(tr.t, tr.q[:, 0], label="qw")
    plt.plot(tr.t, tr.q[:, 1], label="qx")
    plt.plot(tr.t, tr.q[:, 2], label="qy")
    plt.plot(tr.t, tr.q[:, 3], label="qz")
    plt.xlabel("t")
    plt.ylabel("q components (wxyz)")
    plt.legend()
    save_plot(os.path.join(args.outdir, "q_components.png"))

    # Optional Euler angle plot (ZYX)
    if not args.no_euler:
        euler = np.zeros((tr.t.size, 3), dtype=float)
        for i in range(tr.t.size):
            R = quat_to_rotmat_body_to_inertial(tr.q[i, :])
            yaw, pitch, roll = rotmat_to_euler_zyx(R)
            euler[i, 0] = yaw
            euler[i, 1] = pitch
            euler[i, 2] = roll

        euler_deg = euler * (180.0 / np.pi)

        plt.figure()
        plt.plot(tr.t, euler_deg[:, 0], label="yaw (deg)")
        plt.plot(tr.t, euler_deg[:, 1], label="pitch (deg)")
        plt.plot(tr.t, euler_deg[:, 2], label="roll (deg)")
        plt.xlabel("t")
        plt.ylabel("Euler angles ZYX [deg]")
        plt.legend()
        save_plot(os.path.join(args.outdir, "euler_zyx_deg.png"))

    # Optional rotational kinetic energy for diagonal inertia
    if args.inertia is not None:
        Ixx, Iyy, Izz = args.inertia
        T = 0.5 * (Ixx * tr.w[:, 0]**2 + Iyy * tr.w[:, 1]**2 + Izz * tr.w[:, 2]**2)
        plt.figure()
        plt.plot(tr.t, T)
        plt.xlabel("t")
        plt.ylabel("T_rot [J] (diag inertia)")
        save_plot(os.path.join(args.outdir, "rot_kinetic_energy.png"))

    if qnorm_err > args.qnorm_tol:
        print(f"ERROR: quaternion norm deviation exceeded tolerance {args.qnorm_tol}")
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

