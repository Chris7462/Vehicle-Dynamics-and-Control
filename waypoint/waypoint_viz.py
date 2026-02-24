#!/usr/bin/env python3
"""
What this does:
- Reads waypoints.csv
- Draws a red "O" marker at each waypoint inside the running CARLA simulator

Requirements:
- CARLA must be running (CarlaUE4.sh)
- In the CARLA spectator window, view the map from top down

Coordinate note:
- CARLA drawing expects y flipped relative to the logged CSV, it is a weird quirk and why map overlay checking is a good idea
"""

import csv
import carla

CSV_FILENAME = "/workspace/src/waypoint/waypoints.csv"
MARKER_LIFE_SEC = 10.0
DEFAULT_Z = 1.0


def _find_col_optional(header: list[str], name: str) -> int | None:
    header_lc = [h.strip().lower() for h in header]
    name_lc = name.strip().lower()
    return header_lc.index(name_lc) if name_lc in header_lc else None


def load_waypoints_xyz(csv_filename: str):
    """Return list of (x, y, z) using header names. Requires x,y; z optional."""
    points = []
    with open(csv_filename, "r") as fp:
        reader = csv.reader(fp)

        header = next(reader, None)
        if header is None:
            print("CSV file is empty.")
            return points

        ix = _find_col_optional(header, "x")
        iy = _find_col_optional(header, "y")
        iz = _find_col_optional(header, "z")

        if ix is None or iy is None:
            raise ValueError(f"CSV must include columns named 'x' and 'y'. Your header is: {header}")

        for row in reader:
            if not row:
                continue

            x = float(row[ix])
            y = float(row[iy])
            z = float(row[iz]) if (iz is not None and row[iz] != "") else DEFAULT_Z

            points.append((x, y, z))

    return points


def draw_waypoints(world: carla.World, points):
    """Draw a red marker at every waypoint."""
    for x, y, z in points:
        # Course setup: CARLA drawing expects y flipped relative to the logged CSV
        loc = carla.Location(x=x, y=-y, z=z)
        world.debug.draw_string(
            loc,
            "O",
            draw_shadow=False,
            color=carla.Color(r=255, g=0, b=0),
            life_time=MARKER_LIFE_SEC,
            persistent_lines=True,
        )


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    points = load_waypoints_xyz(CSV_FILENAME)
    print(f"Loaded {len(points)} waypoints from {CSV_FILENAME}. Drawing markers…")

    draw_waypoints(world, points)
    print(f"Done — markers will disappear in {MARKER_LIFE_SEC} seconds.")


if __name__ == "__main__":
    main()
