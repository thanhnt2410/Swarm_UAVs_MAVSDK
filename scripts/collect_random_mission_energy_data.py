#!/usr/bin/env python3
"""Collect full-flight energy/time data from controlled random SITL missions."""

from __future__ import annotations

import argparse
import asyncio
import csv
import math
import random
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence

from mavsdk import System

from collect_turn_energy_data import (
    BatteryEnergyResult,
    LatLon,
    LocalXY,
    first_armed_state,
    first_mission_progress,
    first_position,
    integrate_battery_during,
    local_xy_to_lat_lon,
    start_mission_and_wait_mode,
    takeoff_for_run,
    to_mission_plan,
    wait_battery_recharged,
    wait_connected,
    wait_landed,
    wait_mission_finished,
)


# ---------------------------<Experiment defaults>---------------------------

SYSTEM_ADDRESS = "udpin://127.0.0.1:14540"
OUTPUT_PATH = Path("logs/mission_energy/random_missions.csv")
RANDOM_SEED = 42
MISSION_COUNT = 200
REPEATS = 2

SPEED_M_S = 2.5
ALTITUDE_M = 13.0
MIN_TOTAL_DISTANCE_M = 80.0
MAX_TOTAL_DISTANCE_M = 240.0
MIN_WAYPOINT_COUNT = 2
MAX_WAYPOINT_COUNT = 8
MIN_SEGMENT_LENGTH_M = 20.0
MAX_GEOMETRY_ATTEMPTS = 500

BATTERY_RATE_HZ = 10.0
CONNECT_TIMEOUT_S = 30.0
HEALTH_TIMEOUT_S = 30.0
TAKEOFF_TIMEOUT_S = 60.0
TAKEOFF_ATTEMPTS = 2
TAKEOFF_RETRY_DELAY_S = 5.0
MISSION_TIMEOUT_S = 300.0
LANDING_TIMEOUT_S = 90.0
SETTLE_S = 2.0
POST_MISSION_SETTLE_S = 2.0
RECHARGE_WAIT_S = 4.0
RECHARGE_TIMEOUT_S = 15.0
MINIMUM_RECHARGED_PERCENT = 0.95
COOLDOWN_S = 2.0


@dataclass(frozen=True)
class RandomMissionGeometry:
    geometry_id: str
    seed: int
    target_distance_m: float
    total_distance_m: float
    waypoint_count: int
    local_points_xy: List[LocalXY]


def path_distance(points: Sequence[LocalXY]) -> float:
    previous = (0.0, 0.0)
    distance_m = 0.0
    for point in points:
        distance_m += math.hypot(point[0] - previous[0], point[1] - previous[1])
        previous = point
    return distance_m


def minimum_segment_length(points: Sequence[LocalXY]) -> float:
    previous = (0.0, 0.0)
    lengths = []
    for point in points:
        lengths.append(math.hypot(point[0] - previous[0], point[1] - previous[1]))
        previous = point
    return min(lengths)


def build_closed_random_geometry(
    geometry_index: int, rng: random.Random
) -> RandomMissionGeometry:
    waypoint_count = rng.randint(MIN_WAYPOINT_COUNT, MAX_WAYPOINT_COUNT)
    minimum_distance_m = max(MIN_TOTAL_DISTANCE_M, waypoint_count * MIN_SEGMENT_LENGTH_M * 1.2)
    target_distance_m = rng.uniform(minimum_distance_m, MAX_TOTAL_DISTANCE_M)
    geometry_seed = rng.randrange(0, 2**31)
    geometry_rng = random.Random(geometry_seed)

    for _ in range(MAX_GEOMETRY_ATTEMPTS):
        points: List[LocalXY] = []
        x_m = 0.0
        y_m = 0.0
        heading_rad = geometry_rng.uniform(-math.pi, math.pi)
        for _ in range(waypoint_count - 1):
            heading_rad += geometry_rng.uniform(-math.pi * 0.75, math.pi * 0.75)
            segment_m = geometry_rng.uniform(0.7, 1.3)
            x_m += segment_m * math.cos(heading_rad)
            y_m += segment_m * math.sin(heading_rad)
            points.append((x_m, y_m))
        points.append((0.0, 0.0))

        raw_distance_m = path_distance(points)
        if raw_distance_m <= 0.0:
            continue
        scale = target_distance_m / raw_distance_m
        scaled_points = [(x * scale, y * scale) for x, y in points]
        if minimum_segment_length(scaled_points) < MIN_SEGMENT_LENGTH_M:
            continue
        return RandomMissionGeometry(
            geometry_id=f"geometry_{geometry_index:04d}",
            seed=geometry_seed,
            target_distance_m=target_distance_m,
            total_distance_m=path_distance(scaled_points),
            waypoint_count=waypoint_count,
            local_points_xy=scaled_points,
        )

    raise RuntimeError(
        f"Could not generate geometry {geometry_index} with minimum segment "
        f"length {MIN_SEGMENT_LENGTH_M:.1f} m"
    )


def geometry_to_global_points(origin: LatLon, geometry: RandomMissionGeometry) -> List[LatLon]:
    return [local_xy_to_lat_lon(origin, point) for point in geometry.local_points_xy]


async def upload_and_complete_mission(
    drone: System, points: Sequence[LatLon], args: argparse.Namespace
) -> None:
    mission_plan = to_mission_plan(points, SPEED_M_S, args.altitude_m)
    print(f"[mission-collect] Uploading {len(points)} waypoints...")
    await drone.mission.clear_mission()
    await drone.mission.set_return_to_launch_after_mission(False)
    await drone.action.set_current_speed(SPEED_M_S)
    await drone.mission.upload_mission(mission_plan)
    await asyncio.sleep(0.5)

    flight_mode = await start_mission_and_wait_mode(drone, timeout_s=10.0)
    progress = await first_mission_progress(drone, timeout_s=5.0)
    print(
        f"[mission-collect] Flight mode={flight_mode}; "
        f"mission progress={progress.current}/{progress.total}"
    )
    await wait_mission_finished(drone, timeout_s=args.mission_timeout_s)


async def run_full_flight(
    drone: System, points: Sequence[LatLon], args: argparse.Namespace
) -> BatteryEnergyResult:
    async def fly() -> None:
        await takeoff_for_run(drone, args)
        try:
            await upload_and_complete_mission(drone, points, args)
            await asyncio.sleep(args.post_mission_settle_s)
        finally:
            print("[mission-collect] Landing...")
            await drone.action.land()
            await wait_landed(drone, timeout_s=args.landing_timeout_s)
            if await first_armed_state(drone, timeout_s=5.0):
                await drone.action.disarm()

    return await integrate_battery_during(drone, asyncio.create_task(fly()))


def result_row(
    run_id: str,
    repeat_index: int,
    geometry: RandomMissionGeometry,
    result: BatteryEnergyResult,
) -> dict:
    return {
        "run_id": run_id,
        "geometry_id": geometry.geometry_id,
        "repeat_index": repeat_index,
        "seed": geometry.seed,
        "target_distance_m": geometry.target_distance_m,
        "total_distance_m": geometry.total_distance_m,
        "waypoint_count": geometry.waypoint_count,
        "stop_count": geometry.waypoint_count,
        "speed_m_s": SPEED_M_S,
        "altitude_m": ALTITUDE_M,
        "total_flight_time_s": result.measured_time_s,
        "total_flight_energy_wh": result.measured_energy_wh,
        "mean_power_w": result.mean_power_w,
        "battery_voltage_mean_v": result.battery_voltage_mean_v,
        "battery_current_mean_a": result.battery_current_mean_a,
        "valid_energy_intervals": result.valid_energy_intervals,
        "sample_count": result.sample_count,
        "valid": result.valid,
    }


def write_rows(csv_path: Path, rows: Sequence[dict]) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "run_id", "geometry_id", "repeat_index", "seed", "target_distance_m",
        "total_distance_m", "waypoint_count", "stop_count", "speed_m_s", "altitude_m",
        "total_flight_time_s", "total_flight_energy_wh", "mean_power_w",
        "battery_voltage_mean_v", "battery_current_mean_a", "valid_energy_intervals",
        "sample_count", "valid",
    ]
    with csv_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


async def recharge_after_run(drone: System, args: argparse.Namespace) -> None:
    print(f"[mission-collect] Waiting {args.recharge_wait_s:.1f}s for battery reset...")
    await asyncio.sleep(args.recharge_wait_s)
    remaining = await wait_battery_recharged(
        drone, args.minimum_recharged_percent, args.recharge_timeout_s
    )
    print(f"[mission-collect] Battery recharged to {remaining * 100.0:.1f}%.")


async def collect(args: argparse.Namespace) -> None:
    rng = random.Random(RANDOM_SEED)
    geometries = [build_closed_random_geometry(index, rng) for index in range(1, MISSION_COUNT + 1)]

    drone = System()
    await drone.connect(system_address=args.system_address)
    print(f"[mission-collect] Waiting for {args.system_address}...")
    await wait_connected(drone, timeout_s=args.connect_timeout_s)
    print("[mission-collect] Connected.")
    await drone.telemetry.set_rate_battery(BATTERY_RATE_HZ)

    rows: List[dict] = []
    run_group = uuid.uuid4().hex[:8]
    for repeat_index in range(1, REPEATS + 1):
        for geometry_index, geometry in enumerate(geometries, start=1):
            run_id = f"{run_group}_{geometry.geometry_id}_{repeat_index:02d}"
            origin = await first_position(drone, timeout_s=10.0)
            points = geometry_to_global_points(origin, geometry)
            print(
                f"[mission-collect] Run {run_id}: mission={geometry_index}/{len(geometries)}, "
                f"repeat={repeat_index}/{REPEATS}, distance={geometry.total_distance_m:.2f} m, "
                f"waypoints={geometry.waypoint_count}"
            )
            result = await run_full_flight(drone, points, args)
            rows.append(result_row(run_id, repeat_index, geometry, result))
            write_rows(args.output, rows)
            print(
                f"[mission-collect] Saved {args.output} "
                f"(energy={result.measured_energy_wh:.4f} Wh, "
                f"time={result.measured_time_s:.2f}s, valid={result.valid})"
            )
            await recharge_after_run(drone, args)
            await asyncio.sleep(args.cooldown_s)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--system-address", default=SYSTEM_ADDRESS)
    parser.add_argument("--output", type=Path, default=OUTPUT_PATH)
    parser.add_argument("--connect-timeout-s", type=float, default=CONNECT_TIMEOUT_S)
    parser.add_argument("--health-timeout-s", type=float, default=HEALTH_TIMEOUT_S)
    parser.add_argument("--takeoff-timeout-s", type=float, default=TAKEOFF_TIMEOUT_S)
    parser.add_argument("--takeoff-attempts", type=int, default=TAKEOFF_ATTEMPTS)
    parser.add_argument("--takeoff-retry-delay-s", type=float, default=TAKEOFF_RETRY_DELAY_S)
    parser.add_argument("--mission-timeout-s", type=float, default=MISSION_TIMEOUT_S)
    parser.add_argument("--landing-timeout-s", type=float, default=LANDING_TIMEOUT_S)
    parser.add_argument("--settle-s", type=float, default=SETTLE_S)
    parser.add_argument("--post-mission-settle-s", type=float, default=POST_MISSION_SETTLE_S)
    parser.add_argument("--recharge-wait-s", type=float, default=RECHARGE_WAIT_S)
    parser.add_argument("--recharge-timeout-s", type=float, default=RECHARGE_TIMEOUT_S)
    parser.add_argument("--minimum-recharged-percent", type=float, default=MINIMUM_RECHARGED_PERCENT)
    parser.add_argument("--cooldown-s", type=float, default=COOLDOWN_S)
    parser.set_defaults(altitude_m=ALTITUDE_M)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = build_parser().parse_args(argv)
    asyncio.run(collect(args))


if __name__ == "__main__":
    main()
