#!/usr/bin/env python3
"""
compare_waypoints_vs_ppc_log.py
===============================

Compares:
- waypoints.csv (reference path)
- ppc_vehicle_log.csv (achieved vehicle trajectory)

Plots:
1) Square XY plot: waypoints path vs achieved (x,y)
2) Rectangular/step plot: signed cross-track error (CTE) with lane bounds
"""

import sys
import numpy as np
import matplotlib.pyplot as plt

# ------------------------------------------------------------
# HARD-CODED FILEPATHS (EDIT THESE)
# ------------------------------------------------------------
CSV_FILENAME = '/home/za/Documents/VehDyn26/Lab3/waypoints.csv'         # reference path
LOG_FILENAME = '/home/za/Documents/VehDyn26/Lab3/ppc_vehicle_log.csv'  # achieved trajectory

# Student types the speed they used (for labeling only)
SPEED_USED_MPS = 5.0

# Lane bounds: 6 ft left and 6 ft right of centerline
LANE_HALF_WIDTH_FT = 6.0
# ------------------------------------------------------------


def load_named_cols(csv_path, required_cols):
    """Load CSV with header row; return dict of numpy arrays for required columns."""
    try:
        data = np.genfromtxt(
            csv_path,
            delimiter=",",
            names=True,
            dtype=None,
            encoding="utf-8",
            autostrip=True,
        )
    except Exception as e:
        sys.exit(f"ERROR: failed to read {csv_path}: {e}")

    if data.size == 0:
        sys.exit(f"ERROR: {csv_path} appears empty.")

    names = list(data.dtype.names or [])
    missing = [c for c in required_cols if c not in names]
    if missing:
        sys.exit(f"ERROR: {csv_path} missing columns {missing}. Present: {names}")

    out = {c: np.atleast_1d(np.asarray(data[c], dtype=float)) for c in required_cols}
    return out


def signed_distance_point_to_segment(px, py, ax, ay, bx, by):
    """
    Signed distance from point P to segment A->B.
    + means left of segment direction, - means right.
    """
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay

    ab2 = abx * abx + aby * aby
    if ab2 < 1e-12:
        return np.hypot(px - ax, py - ay)

    t = (apx * abx + apy * aby) / ab2
    t = max(0.0, min(1.0, t))

    cx = ax + t * abx
    cy = ay + t * aby

    dist = np.hypot(px - cx, py - cy)

    cross = abx * apy - aby * apx
    return dist if cross >= 0.0 else -dist


def compute_cte_to_polyline(path_x, path_y, veh_x, veh_y):
    """Signed CTE from each vehicle sample to the nearest segment of the waypoint polyline."""
    path_x = np.asarray(path_x, dtype=float)
    path_y = np.asarray(path_y, dtype=float)

    Ax = path_x[:-1]
    Ay = path_y[:-1]
    Bx = path_x[1:]
    By = path_y[1:]

    cte = np.zeros(len(veh_x), dtype=float)

    for k in range(len(veh_x)):
        best_abs = float("inf")
        best_signed = 0.0

        px = float(veh_x[k])
        py = float(veh_y[k])

        for i in range(len(Ax)):
            d = signed_distance_point_to_segment(px, py, Ax[i], Ay[i], Bx[i], By[i])
            if abs(d) < best_abs:
                best_abs = abs(d)
                best_signed = d

        cte[k] = best_signed

    return cte


def style_road_axes(ax):
    """Dark asphalt-style formatting for the CTE plot."""
    ax.set_facecolor("#2b2b2b")
    ax.figure.set_facecolor("#2b2b2b")
    ax.tick_params(colors="white")
    ax.xaxis.label.set_color("white")
    ax.yaxis.label.set_color("white")
    ax.title.set_color("white")
    for spine in ax.spines.values():
        spine.set_color("#aaaaaa")
    ax.grid(True, color="#555555", alpha=0.6)


def main():
    way = load_named_cols(CSV_FILENAME, required_cols=["x", "y"])
    log = load_named_cols(LOG_FILENAME, required_cols=["time", "x", "y"])

    # Time starting at 0
    t = log["time"] - log["time"][0]
    run_time = float(t[-1]) if len(t) else 0.0

    # Signed CTE relative to waypoint polyline
    cte = compute_cte_to_polyline(way["x"], way["y"], log["x"], log["y"])
    mean_abs_cte = float(np.mean(np.abs(cte)))
    max_abs_cte  = float(np.max(np.abs(cte)))

    print(f"Runtime        : {run_time:.2f} s")
    print(f"Speed (manual) : {SPEED_USED_MPS:.2f} m/s")
    print(f"Mean |CTE|     : {mean_abs_cte:.2f} m")
    print(f"Max  |CTE|     : {max_abs_cte:.2f} m")

    # ------------------------------------------------------------
    # 1) Square XY plot (waypoints vs achieved)
    # ------------------------------------------------------------
    plt.figure(figsize=(7.5, 7.5))
    plt.plot(way["x"], way["y"], "o-", linewidth=2, markersize=4, label="Waypoints (path)")
    plt.plot(log["x"], log["y"], "-", linewidth=2, label="Achieved (PPC)")

    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # ------------------------------------------------------------
    # 2) Rectangular/step CTE plot with lane bounds (±6 ft)
    # ------------------------------------------------------------
    lane_half_m = LANE_HALF_WIDTH_FT * 0.3048

    plt.figure(figsize=(10, 4.8))
    ax = plt.gca()
    style_road_axes(ax)

    # Shade "in-lane" region
    ax.fill_between(t, -lane_half_m, +lane_half_m, step="post", color="#3a3a3a", alpha=0.95)

    # Lane boundaries (yellow)
    ax.step(t, +lane_half_m * np.ones_like(t), where="post",
            linewidth=2.5, color="#ffd400", label="Lane bounds (±6 ft)")
    ax.step(t, -lane_half_m * np.ones_like(t), where="post",
            linewidth=2.5, color="#ffd400")

    # CTE (red)
    ax.step(t, cte, where="post", linewidth=2.2, color="red", label="CTE (signed)")
    ax.fill_between(t, 0.0, cte, step="post", color="red", alpha=0.25)

    ax.axhline(0.0, linewidth=1.2, color="#dddddd")

    ax.set_xlabel("time [s]")
    ax.set_ylabel("CTE [m]")

    ax.set_title(
        f"Cross-Track Error (step/area) | speed={SPEED_USED_MPS:.2f} m/s | "
        f"mean|CTE|={mean_abs_cte:.2f} m, max|CTE|={max_abs_cte:.2f} m"
    )

    ax.legend(facecolor="#2b2b2b", edgecolor="#aaaaaa", labelcolor="white")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
