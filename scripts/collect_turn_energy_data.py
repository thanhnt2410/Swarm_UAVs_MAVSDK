#!/usr/bin/env python3
"""Collect PX4 SITL energy/time data for single-turn UAV paths.

The script connects to one PX4/Gazebo UAV through MAVSDK and flies paired
outbound/inbound three-waypoint corner paths. Each direction is measured
independently, followed by landing, disarming, and simulated battery recharge.
Battery telemetry is integrated during the mission only.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import math
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from statistics import mean
from typing import Iterable, List, Optional, Sequence, Tuple

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan


EARTH_RADIUS_M = 6378137.0


LatLon = Tuple[float, float]
LocalXY = Tuple[float, float]


@dataclass(frozen=True)
class TurnMissionGeometry:
    angle_deg: float
    angle_rad: float
    speed_m_s: float
    pre_length_m: float
    post_length_m: float
    total_distance_m: float
    local_points_xy: List[LocalXY]


@dataclass
class BatteryEnergyResult:
    measured_time_s: float
    measured_energy_wh: float
    mean_power_w: float
    battery_voltage_mean_v: float
    battery_current_mean_a: float
    valid_energy_intervals: int
    sample_count: int

    @property
    def valid(self) -> bool:
        return self.valid_energy_intervals > 0 and self.sample_count > 0


def parse_angles(raw: str) -> List[float]:
    return [float(part.strip()) for part in raw.split(",") if part.strip()]


def local_xy_to_lat_lon(origin: LatLon, point: LocalXY) -> LatLon:
    lat0, lon0 = origin
    x_east_m, y_north_m = point
    lat = lat0 + math.degrees(y_north_m / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(x_east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


def build_turn_geometry(
    angle_deg: float,
    speed_m_s: float,
    pre_length_m: float,
    post_length_m: float,
) -> TurnMissionGeometry:
    if speed_m_s <= 0:
        raise ValueError("speed_m_s must be > 0")
    if pre_length_m < 0 or post_length_m < 0:
        raise ValueError("pre_length_m and post_length_m must be >= 0")
    if not 0.0 <= abs(angle_deg) <= 180.0:
        raise ValueError("abs(angle_deg) must be between 0 and 180")

    angle_rad = math.radians(angle_deg)
    turn_rad = math.copysign(math.pi - abs(angle_rad), angle_deg if angle_deg != 0.0 else 1.0)
    corner = (pre_length_m, 0.0)
    end = (
        pre_length_m + post_length_m * math.cos(turn_rad),
        post_length_m * math.sin(turn_rad),
    )
    points: List[LocalXY] = [(0.0, 0.0), corner, end]
    total_distance_m = pre_length_m + post_length_m
    return TurnMissionGeometry(
        angle_deg=angle_deg,
        angle_rad=angle_rad,
        speed_m_s=speed_m_s,
        pre_length_m=pre_length_m,
        post_length_m=post_length_m,
        total_distance_m=total_distance_m,
        local_points_xy=points,
    )


def geometry_to_global_points(origin: LatLon, geometry: TurnMissionGeometry) -> List[LatLon]:
    return [local_xy_to_lat_lon(origin, point) for point in geometry.local_points_xy]


def to_mission_plan(points: Sequence[LatLon], speed_m_s: float, altitude_m: float) -> MissionPlan:
    items = []
    for lat, lon in points:
        items.append(
            MissionItem(
                latitude_deg=lat,
                longitude_deg=lon,
                relative_altitude_m=altitude_m,
                speed_m_s=speed_m_s,
                is_fly_through=False,
                gimbal_pitch_deg=float("nan"),
                gimbal_yaw_deg=float("nan"),
                loiter_time_s=1.0,
                acceptance_radius_m=float("nan"),
                yaw_deg=float("nan"),
                camera_action=MissionItem.CameraAction.NONE,
                camera_photo_distance_m=float("nan"),
                camera_photo_interval_s=float("nan"),
                vehicle_action=MissionItem.VehicleAction.NONE,
            )
        )
    return MissionPlan(items)


async def wait_connected(drone: System, timeout_s: float) -> None:
    async def wait() -> None:
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    await asyncio.wait_for(wait(), timeout=timeout_s)


async def first_position(drone: System, timeout_s: float) -> LatLon:
    async def wait() -> LatLon:
        async for position in drone.telemetry.position():
            if math.isfinite(position.latitude_deg) and math.isfinite(position.longitude_deg):
                return position.latitude_deg, position.longitude_deg
        raise TimeoutError("Position telemetry stream ended")

    return await asyncio.wait_for(wait(), timeout=timeout_s)


async def wait_for_mission_mode(drone: System, timeout_s: float):
    async def wait():
        async for flight_mode in drone.telemetry.flight_mode():
            if "MISSION" in str(flight_mode).upper():
                return flight_mode
        raise RuntimeError("Flight-mode telemetry stream ended")

    return await asyncio.wait_for(wait(), timeout=timeout_s)


async def first_flight_mode_value(drone: System, timeout_s: float):
    async def wait():
        async for flight_mode in drone.telemetry.flight_mode():
            return flight_mode
        raise RuntimeError("Flight-mode telemetry stream ended")

    return await asyncio.wait_for(wait(), timeout=timeout_s)


async def first_mission_progress(drone: System, timeout_s: float):
    async def wait():
        async for progress in drone.mission.mission_progress():
            return progress
        raise RuntimeError("Mission-progress stream ended")

    return await asyncio.wait_for(wait(), timeout=timeout_s)


async def wait_relative_altitude(drone: System, target_altitude_m: float, timeout_s: float) -> None:
    async def wait() -> None:
        last_report_time = 0.0
        async for position in drone.telemetry.position():
            now = time.monotonic()
            if now - last_report_time >= 2.0:
                print(
                    f"[collect] Relative altitude: {position.relative_altitude_m:.1f} m "
                    f"(target >= {target_altitude_m - 1.0:.1f} m)"
                )
                last_report_time = now
            if position.relative_altitude_m >= target_altitude_m - 1.0:
                return

    await asyncio.wait_for(wait(), timeout=timeout_s)


async def wait_ready_to_arm(drone: System, timeout_s: float) -> None:
    async def wait() -> None:
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                return

    await asyncio.wait_for(wait(), timeout=timeout_s)


async def wait_landed(drone: System, timeout_s: float) -> None:
    async def wait() -> None:
        landed_samples = 0
        async for is_in_air in drone.telemetry.in_air():
            landed_samples = landed_samples + 1 if not is_in_air else 0
            if landed_samples >= 3:
                return

    await asyncio.wait_for(wait(), timeout=timeout_s)


async def first_armed_state(drone: System, timeout_s: float) -> bool:
    async def wait() -> bool:
        async for is_armed in drone.telemetry.armed():
            return is_armed
        raise RuntimeError("Armed-state telemetry stream ended")

    return await asyncio.wait_for(wait(), timeout=timeout_s)


async def wait_battery_recharged(drone: System, minimum_percent: float, timeout_s: float) -> float:
    async def wait() -> float:
        async for battery in drone.telemetry.battery():
            remaining_percent = getattr(battery, "remaining_percent", float("nan"))
            if math.isfinite(remaining_percent) and remaining_percent >= minimum_percent:
                return remaining_percent
        raise RuntimeError("Battery telemetry stream ended")

    return await asyncio.wait_for(wait(), timeout=timeout_s)


async def takeoff_for_run(drone: System, args: argparse.Namespace) -> None:
    for attempt in range(1, args.takeoff_attempts + 1):
        print("[collect] Waiting for GPS/home-position health...")
        await wait_ready_to_arm(drone, timeout_s=args.health_timeout_s)
        await drone.action.set_takeoff_altitude(args.altitude_m)
        print(f"[collect] Taking off to {args.altitude_m:.1f} m (attempt {attempt}/{args.takeoff_attempts})...")
        if not await first_armed_state(drone, timeout_s=5.0):
            await drone.action.arm()
        await drone.action.takeoff()

        try:
            await wait_relative_altitude(drone, args.altitude_m, timeout_s=args.takeoff_timeout_s)
            await asyncio.sleep(args.settle_s)
            return
        except asyncio.TimeoutError as exc:
            flight_mode = await first_flight_mode_value(drone, timeout_s=2.0)
            is_armed = await first_armed_state(drone, timeout_s=2.0)
            print(f"[collect] Takeoff timed out: flight_mode={flight_mode}, armed={is_armed}.")
            print("[collect] Landing and disarming before takeoff retry...")
            await drone.action.land()
            await wait_landed(drone, timeout_s=args.landing_timeout_s)
            if await first_armed_state(drone, timeout_s=5.0):
                await drone.action.disarm()
            if attempt == args.takeoff_attempts:
                raise RuntimeError(
                    f"Takeoff failed after {args.takeoff_attempts} attempts "
                    f"(last flight mode={flight_mode}). Check the PX4 console and vehicle pose in Gazebo."
                ) from exc
            await asyncio.sleep(args.takeoff_retry_delay_s)


async def land_disarm_and_recharge(
    drone: System, args: argparse.Namespace, return_to_launch: bool = False
) -> None:
    if return_to_launch:
        print("[collect] Returning to launch before battery reset...")
        await drone.action.return_to_launch()
    else:
        print("[collect] Landing before battery reset...")
        await drone.action.land()

    await wait_landed(drone, timeout_s=args.landing_timeout_s)
    if await first_armed_state(drone, timeout_s=5.0):
        print("[collect] Disarming...")
        await drone.action.disarm()
    else:
        print("[collect] UAV landed and is already disarmed.")

    print(f"[collect] Waiting {args.recharge_wait_s:.1f}s for simulated battery reset...")
    await asyncio.sleep(args.recharge_wait_s)
    remaining_percent = await wait_battery_recharged(
        drone, minimum_percent=args.minimum_recharged_percent, timeout_s=args.recharge_timeout_s
    )
    print(f"[collect] Simulated battery recharged to {remaining_percent * 100.0:.1f}%.")


async def wait_mission_finished(drone: System, timeout_s: float) -> None:
    async def wait() -> None:
        while True:
            if await drone.mission.is_mission_finished():
                return
            await asyncio.sleep(0.25)

    await asyncio.wait_for(wait(), timeout=timeout_s)


async def integrate_battery_during(drone: System, task: asyncio.Task) -> BatteryEnergyResult:
    energy_wh = 0.0
    valid_intervals = 0
    voltage_samples: List[float] = []
    current_samples: List[float] = []
    power_samples: List[float] = []
    last_sample_time: Optional[float] = None
    start_time = time.monotonic()

    async def collect() -> None:
        nonlocal energy_wh, valid_intervals, last_sample_time
        async for battery in drone.telemetry.battery():
            now = time.monotonic()
            voltage_v = getattr(battery, "voltage_v", float("nan"))
            current_a = getattr(battery, "current_battery_a", float("nan"))
            if math.isfinite(voltage_v) and math.isfinite(current_a) and voltage_v > 0.0 and current_a >= 0.0:
                voltage_samples.append(voltage_v)
                current_samples.append(current_a)
                power_samples.append(voltage_v * current_a)
                if last_sample_time is not None:
                    dt_s = min(max(now - last_sample_time, 0.0), 2.0)
                    energy_wh += voltage_v * current_a * dt_s / 3600.0
                    valid_intervals += 1
                last_sample_time = now
            else:
                last_sample_time = None

    battery_task = asyncio.create_task(collect())
    try:
        await task
    finally:
        battery_task.cancel()
        await asyncio.gather(battery_task, return_exceptions=True)

    measured_time_s = time.monotonic() - start_time
    return BatteryEnergyResult(
        measured_time_s=measured_time_s,
        measured_energy_wh=energy_wh,
        mean_power_w=mean(power_samples) if power_samples else float("nan"),
        battery_voltage_mean_v=mean(voltage_samples) if voltage_samples else float("nan"),
        battery_current_mean_a=mean(current_samples) if current_samples else float("nan"),
        valid_energy_intervals=valid_intervals,
        sample_count=len(power_samples),
    )


async def start_mission_and_wait_mode(drone: System, timeout_s: float = 10.0):
    """Start mission with item index reset and retry if PX4 momentarily stays in HOLD."""
    for attempt in range(1, 4):
        try:
            await drone.mission.set_current_mission_item_index(0)
        except Exception:
            pass
        await asyncio.sleep(0.2)
        try:
            await drone.mission.start_mission()
        except Exception as e:
            print(f"[collect] (attempt {attempt}/3) start_mission: {e}")
        
        try:
            flight_mode = await wait_for_mission_mode(drone, timeout_s=2.5)
            return flight_mode
        except asyncio.TimeoutError:
            pass
        await asyncio.sleep(0.5)

    # Final attempt
    try:
        await drone.mission.set_current_mission_item_index(0)
    except Exception:
        pass
    await drone.mission.start_mission()
    return await wait_for_mission_mode(drone, timeout_s=timeout_s)


async def run_single_mission(
    drone: System,
    mission_plan: MissionPlan,
    speed_m_s: float,
    mission_timeout_s: float,
) -> BatteryEnergyResult:
    print(f"[collect] Uploading mission with {len(mission_plan.mission_items)} waypoints...")
    await drone.mission.clear_mission()
    await drone.mission.set_return_to_launch_after_mission(False)
    await drone.action.set_current_speed(speed_m_s)
    await drone.mission.upload_mission(mission_plan)
    await asyncio.sleep(0.5)
    print("[collect] Starting mission...")
    try:
        flight_mode = await start_mission_and_wait_mode(drone, timeout_s=10.0)
    except asyncio.TimeoutError as exc:
        current_mode = await first_flight_mode_value(drone, timeout_s=2.0)
        raise RuntimeError(
            f"PX4 did not enter MISSION mode within 10 seconds (mode={current_mode}). "
            "Check the PX4 console for a mode rejection or failsafe message."
        ) from exc
    progress = await first_mission_progress(drone, timeout_s=5.0)
    print(
        f"[collect] Flight mode={flight_mode}; "
        f"mission progress={progress.current}/{progress.total}"
    )
    print("[collect] Mission active; collecting battery telemetry...")

    async def wait_for_finish() -> None:
        await wait_mission_finished(drone, mission_timeout_s)

    wait_task = asyncio.create_task(wait_for_finish())
    return await integrate_battery_during(drone, wait_task)


def build_rows_with_baseline(rows: List[dict]) -> List[dict]:
    valid_baselines = [
        row for row in rows
        if row["valid"]
        and abs(abs(float(row["angle_deg"])) - 180.0) < 1e-9
        and float(row["total_distance_m"]) > 0.0
    ]
    if not valid_baselines:
        for row in rows:
            row["turn_time_s"] = ""
            row["turn_energy_wh"] = ""
        return rows

    baseline_time_per_m = mean(
        float(row["measured_time_s"]) / float(row["total_distance_m"])
        for row in valid_baselines
    )
    baseline_energy_per_m = mean(
        float(row["measured_energy_wh"]) / float(row["total_distance_m"])
        for row in valid_baselines
    )

    for row in rows:
        total_distance_m = float(row["total_distance_m"])
        row["turn_time_s"] = float(row["measured_time_s"]) - baseline_time_per_m * total_distance_m
        row["turn_energy_wh"] = float(row["measured_energy_wh"]) - baseline_energy_per_m * total_distance_m
    return rows


def write_rows(csv_path: Path, rows: List[dict]) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "run_id",
        "repeat_index",
        "angle_deg",
        "angle_rad",
        "speed_m_s",
        "pre_length_m",
        "post_length_m",
        "total_distance_m",
        "measured_time_s",
        "measured_energy_wh",
        "turn_time_s",
        "turn_energy_wh",
        "mean_power_w",
        "battery_voltage_mean_v",
        "battery_current_mean_a",
        "valid_energy_intervals",
        "sample_count",
        "valid",
    ]
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def result_row(
    run_id: str,
    direction: str,
    repeat_index: int,
    geometry: TurnMissionGeometry,
    result: BatteryEnergyResult,
) -> dict:
    direction_sign = 1.0 if direction == "outbound" else -1.0
    pre_length_m = geometry.pre_length_m if direction == "outbound" else geometry.post_length_m
    post_length_m = geometry.post_length_m if direction == "outbound" else geometry.pre_length_m
    return {
        "run_id": run_id,
        "repeat_index": repeat_index,
        "angle_deg": direction_sign * geometry.angle_deg,
        "angle_rad": direction_sign * geometry.angle_rad,
        "speed_m_s": geometry.speed_m_s,
        "pre_length_m": pre_length_m,
        "post_length_m": post_length_m,
        "total_distance_m": geometry.total_distance_m,
        "measured_time_s": result.measured_time_s,
        "measured_energy_wh": result.measured_energy_wh,
        "turn_time_s": "",
        "turn_energy_wh": "",
        "mean_power_w": result.mean_power_w,
        "battery_voltage_mean_v": result.battery_voltage_mean_v,
        "battery_current_mean_a": result.battery_current_mean_a,
        "valid_energy_intervals": result.valid_energy_intervals,
        "sample_count": result.sample_count,
        "valid": result.valid,
    }


async def collect(args: argparse.Namespace) -> None:
    drone = System()
    await drone.connect(system_address=args.system_address)
    print(f"[collect] Waiting for {args.system_address}...")
    await wait_connected(drone, timeout_s=args.connect_timeout_s)
    print("[collect] Connected.")

    if args.rtl_after:
        print("[collect] --rtl-after is ignored: inbound missions now fly the exact reverse path.")

    await drone.telemetry.set_rate_battery(args.battery_rate_hz)
    await drone.action.set_takeoff_altitude(args.altitude_m)
    await drone.action.set_current_speed(args.speed_m_s)

    rows: List[dict] = []
    run_group = uuid.uuid4().hex[:8]
    angles = parse_angles(args.angles)

    for repeat_index in range(1, args.repeats + 1):
        for angle_deg in angles:
            geometry = build_turn_geometry(
                angle_deg=angle_deg,
                speed_m_s=args.speed_m_s,
                pre_length_m=args.pre_length_m,
                post_length_m=args.post_length_m,
            )
            pair_id = f"{run_group}_{repeat_index:02d}_{angle_deg:g}"
            origin = await first_position(drone, timeout_s=10.0)
            outbound_points = geometry_to_global_points(origin, geometry)
            directions = [("outbound", outbound_points), ("inbound", list(reversed(outbound_points)))]

            for direction, points in directions:
                if not args.skip_takeoff:
                    await takeoff_for_run(drone, args)

                run_id = f"{pair_id}_{direction}"
                mission_plan = to_mission_plan(points, geometry.speed_m_s, args.altitude_m)
                print(
                    f"[collect] Run {run_id}: direction={direction}, angle={abs(angle_deg):g} deg, "
                    f"distance={geometry.total_distance_m:.2f} m, waypoints={len(points)}"
                )
                result = await run_single_mission(
                    drone=drone,
                    mission_plan=mission_plan,
                    speed_m_s=geometry.speed_m_s,
                    mission_timeout_s=args.mission_timeout_s,
                )
                rows.append(result_row(run_id, direction, repeat_index, geometry, result))
                rows = build_rows_with_baseline(rows)
                write_rows(args.output, rows)
                print(
                    f"[collect] Saved {args.output} "
                    f"(energy={result.measured_energy_wh:.4f} Wh, time={result.measured_time_s:.2f}s, "
                    f"valid={result.valid})"
                )
                if not args.skip_takeoff:
                    await asyncio.sleep(args.post_mission_settle_s)
                    await land_disarm_and_recharge(drone, args, return_to_launch=False)
                await asyncio.sleep(args.cooldown_s)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--system-address", default="udpin://127.0.0.1:14540")
    parser.add_argument("--output", type=Path, default=Path("logs/turn_energy/turn_energy_dataset.csv"))
    parser.add_argument("--angles", default="0,15,30,45,60,90,120,150,180")
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--speed-m-s", type=float, default=2.5)
    parser.add_argument("--altitude-m", type=float, default=13.0)
    parser.add_argument("--pre-length-m", type=float, default=30.0)
    parser.add_argument("--post-length-m", type=float, default=30.0)
    parser.add_argument("--battery-rate-hz", type=float, default=10.0)
    parser.add_argument("--connect-timeout-s", type=float, default=30.0)
    parser.add_argument("--health-timeout-s", type=float, default=30.0)
    parser.add_argument("--takeoff-timeout-s", type=float, default=60.0)
    parser.add_argument("--takeoff-attempts", type=int, default=2)
    parser.add_argument("--takeoff-retry-delay-s", type=float, default=5.0)
    parser.add_argument("--mission-timeout-s", type=float, default=180.0)
    parser.add_argument("--landing-timeout-s", type=float, default=90.0)
    parser.add_argument("--recharge-wait-s", type=float, default=4.0)
    parser.add_argument("--recharge-timeout-s", type=float, default=15.0)
    parser.add_argument("--minimum-recharged-percent", type=float, default=0.95)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--post-mission-settle-s", type=float, default=2.0)
    parser.add_argument("--cooldown-s", type=float, default=2.0)
    parser.add_argument("--skip-takeoff", action="store_true")
    parser.add_argument(
        "--rtl-after",
        action="store_true",
        help="Deprecated compatibility option; reverse-path mode does not use RTL",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = build_parser().parse_args(argv)
    asyncio.run(collect(args))


if __name__ == "__main__":
    main()
