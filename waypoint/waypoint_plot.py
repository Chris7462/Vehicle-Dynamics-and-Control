#!/usr/bin/env python3
"""
What this does:
- Reads waypoints.csv (any column order is OK)
- Looks for columns named: x, y   (case-insensitive)
- Connects to a running CARLA simulator
- Gets road centerlines from the CARLA map
- Plots BOTH:
    (1) CARLA road centerlines (map geometry)
    (2) Your logged waypoints from the CSV (x,y)

Important:
- CARLA must be running for the map overlay to work.
- If your waypoints look mirrored across the x-axis, toggle FLIP_WAYPOINT_Y. This is a weird quirk between ROS <-> CARLA
"""

import csv

import matplotlib.pyplot as plt
import carla

CSV_FILENAME = "/workspace/src/waypoint/waypoints.csv"

FLIP_WAYPOINT_Y = True
MAP_WP_SPACING_M = 2.0


def _find_col(header: list[str], name: str) -> int:
    """Return index of column 'name' in header (case-insensitive). Raises ValueError if not found."""
    header_lc = [h.strip().lower() for h in header]
    name_lc = name.strip().lower()
    if name_lc not in header_lc:
        raise ValueError(f"CSV is missing a '{name}' column. Your header is: {header}")
    return header_lc.index(name_lc)


def load_waypoints_xy(csv_filename: str):
    """Return list of (x, y) from the CSV using header names."""
    pts = []
    with open(csv_filename, "r") as fp:
        reader = csv.reader(fp)

        header = next(reader, None)
        if header is None:
            print("CSV file is empty.")
            return pts

        ix = _find_col(header, "x")
        iy = _find_col(header, "y")

        for row in reader:
            if not row:
                continue
            x = float(row[ix])
            y = float(row[iy])
            if FLIP_WAYPOINT_Y:
                y = -y
            pts.append((x, y))

    return pts


def fetch_map_centerlines(client: carla.Client, spacing_m: float):
    """Sample CARLA map centerline points (x, y)."""
    world = client.get_world()
    carla_map = world.get_map()

    pts = []
    for wp in carla_map.generate_waypoints(spacing_m):
        loc = wp.transform.location
        pts.append((loc.x, loc.y))
    return pts


def main() -> None:
    wp_xy = load_waypoints_xy(CSV_FILENAME)
    if not wp_xy:
        print("No waypoints loaded. Check your CSV file and the column names (need x and y).")
        return

    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)
    map_xy = fetch_map_centerlines(client, MAP_WP_SPACING_M)

    plt.figure()

    mx, my = zip(*map_xy)
    plt.plot(mx, my, ".", markersize=1, label="CARLA road centerlines (map)")

    wx, wy = zip(*wp_xy)
    plt.plot(wx, wy, "-", linewidth=2, label="Logged waypoints.csv")

    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Waypoint verification: CSV overlaid on CARLA map")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
