#!/usr/bin/env python3
"""Generate QGroundControl .plan files for turn energy experiments."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


EARTH_RADIUS_M = 6378137.0
NAV_WAYPOINT_COMMAND = 16
NAV_TAKEOFF_COMMAND = 22
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

LatLon = Tuple[float, float]
LocalXY = Tuple[float, float]


def parse_angles(raw: str) -> List[float]:
    return [float(part.strip()) for part in raw.split(",") if part.strip()]


def local_xy_to_lat_lon(origin: LatLon, point: LocalXY) -> LatLon:
    lat0, lon0 = origin
    x_east_m, y_north_m = point
    lat = lat0 + math.degrees(y_north_m / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(x_east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


def build_turn_points(
    angle_deg: float,
    pre_length_m: float,
    post_length_m: float,
) -> List[LocalXY]:
    if not 0.0 <= abs(angle_deg) <= 180.0:
        raise ValueError("abs(angle_deg) must be between 0 and 180")
    angle_rad = math.radians(angle_deg)
    turn_rad = math.copysign(math.pi - abs(angle_rad), angle_deg if angle_deg != 0.0 else 1.0)
    return [
        (0.0, 0.0),
        (pre_length_m, 0.0),
        (
            pre_length_m + post_length_m * math.cos(turn_rad),
            post_length_m * math.sin(turn_rad),
        ),
    ]


def simple_item(
    command: int,
    do_jump_id: int,
    lat: float,
    lon: float,
    altitude_m: float,
    hold_or_pitch: float = 0.0,
) -> dict:
    return {
        "AMSLAltAboveTerrain": None,
        "Altitude": altitude_m,
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": command,
        "doJumpId": do_jump_id,
        "frame": MAV_FRAME_GLOBAL_RELATIVE_ALT,
        "params": [
            hold_or_pitch,
            0,
            0,
            None,
            lat,
            lon,
            altitude_m,
        ],
        "type": "SimpleItem",
    }


def build_plan(
    origin: LatLon,
    local_points: List[LocalXY],
    altitude_m: float,
    speed_m_s: float,
) -> dict:
    origin_lat, origin_lon = origin
    items = [
        simple_item(
            command=NAV_TAKEOFF_COMMAND,
            do_jump_id=1,
            lat=origin_lat,
            lon=origin_lon,
            altitude_m=altitude_m,
            hold_or_pitch=0.0,
        )
    ]

    for idx, point in enumerate(local_points[1:], start=2):
        lat, lon = local_xy_to_lat_lon(origin, point)
        items.append(
            simple_item(
                command=NAV_WAYPOINT_COMMAND,
                do_jump_id=idx,
                lat=lat,
                lon=lon,
                altitude_m=altitude_m,
                hold_or_pitch=0.0,
            )
        )

    return {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": speed_m_s,
            "firmwareType": 12,
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": 5,
            "items": items,
            "plannedHomePosition": [origin_lat, origin_lon, altitude_m],
            "vehicleType": 2,
            "version": 2,
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1,
    }


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("logs/turn_energy/plans"))
    parser.add_argument("--angles", default="0,15,30,45,60,90,120,150,180")
    parser.add_argument("--origin-lat", type=float, default=47.3977419)
    parser.add_argument("--origin-lon", type=float, default=8.5455940)
    parser.add_argument("--speed-m-s", type=float, default=2.5)
    parser.add_argument("--altitude-m", type=float, default=13.0)
    parser.add_argument("--pre-length-m", type=float, default=30.0)
    parser.add_argument("--post-length-m", type=float, default=30.0)
    args = parser.parse_args(argv)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    origin = (args.origin_lat, args.origin_lon)
    manifest = []

    for angle_deg in parse_angles(args.angles):
        points = build_turn_points(
            angle_deg=angle_deg,
            pre_length_m=args.pre_length_m,
            post_length_m=args.post_length_m,
        )
        plan = build_plan(
            origin=origin,
            local_points=points,
            altitude_m=args.altitude_m,
            speed_m_s=args.speed_m_s,
        )
        angle_label = f"{angle_deg:g}".replace("-", "minus_").replace(".", "p")
        path = args.output_dir / f"turn_angle_{angle_label}deg.plan"
        path.write_text(json.dumps(plan, indent=4))
        manifest.append(
            {
                "plan_file": str(path),
                "angle_deg": angle_deg,
                "waypoints": len(plan["mission"]["items"]),
                "speed_m_s": args.speed_m_s,
                "altitude_m": args.altitude_m,
                "pre_length_m": args.pre_length_m,
                "post_length_m": args.post_length_m,
            }
        )
        print(f"Generated {path} ({len(plan['mission']['items'])} mission items)")

    (args.output_dir / "manifest.json").write_text(json.dumps(manifest, indent=4))


if __name__ == "__main__":
    main()
