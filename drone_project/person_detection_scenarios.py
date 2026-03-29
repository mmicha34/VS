from __future__ import annotations

import argparse
import math
import random
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence

from pymavlink import mavutil

DRONE_PORTS = {
    "drone1": "udp:127.0.0.1:14550",
    "drone2": "udp:127.0.0.1:14560",
}

DEFAULT_PROBABILITIES = [0.25, 0.55, 0.85]
DEFAULT_ALTITUDE_M = 12.0
WAYPOINT_REACH_THRESHOLD_M = 3.0
POSITION_TIMEOUT_S = 5.0


@dataclass(frozen=True)
class DetectionAttempt:
    step: int
    location_name: str
    probability: float
    random_value: float
    detected: bool


@dataclass(frozen=True)
class LocationPlan:
    name: str
    north_offset_m: float
    east_offset_m: float
    altitude_m: float
    probability: float


@dataclass(frozen=True)
class Position:
    lat: float
    lon: float
    alt: float


class DroneConnection:
    def __init__(self, name: str, endpoint: str) -> None:
        self.name = name
        self.endpoint = endpoint
        self.link: Optional[mavutil.mavfile] = None

    def connect(self) -> None:
        if self.link is not None:
            return
        print(f"{self.name}: connecting on {self.endpoint} ...")
        link = mavutil.mavlink_connection(self.endpoint, source_system=250)
        heartbeat = link.wait_heartbeat(timeout=20)
        link.target_system = heartbeat.get_srcSystem()
        link.target_component = heartbeat.get_srcComponent()
        self.link = link
        print(
            f"{self.name}: connected with sysid={link.target_system} "
            f"compid={link.target_component}"
        )

    def require_link(self) -> mavutil.mavfile:
        if self.link is None:
            raise RuntimeError(f"{self.name}: drone is not connected")
        return self.link

    def wait_for_position(self, timeout_s: float = POSITION_TIMEOUT_S) -> Position:
        link = self.require_link()
        message = link.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout_s)
        if not message:
            raise RuntimeError(f"{self.name}: timed out waiting for GLOBAL_POSITION_INT")
        return Position(
            lat=message.lat / 1e7,
            lon=message.lon / 1e7,
            alt=message.relative_alt / 1000.0,
        )

    def set_guided(self) -> None:
        link = self.require_link()
        mapping = link.mode_mapping()
        if not mapping or "GUIDED" not in mapping:
            raise RuntimeError(f"{self.name}: GUIDED mode is not available")
        mode_id = mapping["GUIDED"]
        link.mav.set_mode_send(
            link.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        print(f"{self.name}: switching to GUIDED")
        time.sleep(1)

    def arm(self) -> None:
        link = self.require_link()
        print(f"{self.name}: arming")
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        time.sleep(2)

    def takeoff(self, altitude_m: float) -> None:
        link = self.require_link()
        print(f"{self.name}: taking off to {altitude_m:.1f} m")
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude_m,
        )
        self.wait_until_altitude(altitude_m * 0.9)

    def wait_until_altitude(self, minimum_altitude_m: float, timeout_s: float = 45.0) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            position = self.wait_for_position()
            print(f"{self.name}: altitude={position.alt:.1f} m")
            if position.alt >= minimum_altitude_m:
                return
            time.sleep(1)
        raise RuntimeError(f"{self.name}: takeoff timed out")

    def goto(self, position: Position) -> None:
        link = self.require_link()
        print(
            f"{self.name}: going to lat={position.lat:.6f}, "
            f"lon={position.lon:.6f}, alt={position.alt:.1f} m"
        )
        link.mav.set_position_target_global_int_send(
            0,
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            int(0b110111111000),
            int(position.lat * 1e7),
            int(position.lon * 1e7),
            position.alt,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def wait_until_close(self, target: Position, threshold_m: float = WAYPOINT_REACH_THRESHOLD_M) -> Position:
        while True:
            current = self.wait_for_position()
            separation = distance_m(current, target)
            print(
                f"{self.name}: now at lat={current.lat:.6f}, lon={current.lon:.6f}, "
                f"alt={current.alt:.1f} m, dist={separation:.1f} m"
            )
            if separation <= threshold_m:
                return current
            time.sleep(1)


def parse_probabilities(raw_values: Optional[Iterable[float]]) -> List[float]:
    if raw_values is None:
        return list(DEFAULT_PROBABILITIES)

    probabilities = [float(value) for value in raw_values]
    if len(probabilities) != 3:
        raise ValueError("Exactly three probabilities are required")

    for probability in probabilities:
        if not 0.0 <= probability <= 1.0:
            raise ValueError("Probabilities must be between 0.0 and 1.0")

    return probabilities


def add_detection_arguments(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    parser.add_argument(
        "--probabilities",
        nargs=3,
        type=float,
        metavar=("P1", "P2", "P3"),
        help="Three detection probabilities, one for each location",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Optional random seed for repeatable simulation results",
    )
    parser.add_argument(
        "--scan-delay",
        type=float,
        default=2.0,
        help="Seconds to wait before evaluating detection at each location",
    )
    parser.add_argument(
        "--altitude",
        type=float,
        default=DEFAULT_ALTITUDE_M,
        help="Mission altitude in meters",
    )
    return parser


def offset_position(origin: Position, north_m: float, east_m: float, altitude_m: float) -> Position:
    lat_delta = north_m / 110540.0
    lon_scale = 111320.0 * math.cos(math.radians(origin.lat))
    if abs(lon_scale) < 1e-6:
        raise ValueError("Longitude scaling became too small for coordinate conversion")
    lon_delta = east_m / lon_scale
    return Position(
        lat=origin.lat + lat_delta,
        lon=origin.lon + lon_delta,
        alt=altitude_m,
    )


def distance_m(a: Position, b: Position) -> float:
    dx = (b.lon - a.lon) * 111320.0 * math.cos(math.radians((a.lat + b.lat) / 2.0))
    dy = (b.lat - a.lat) * 110540.0
    dz = b.alt - a.alt
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def build_location_plan(home: Position, probabilities: Sequence[float], altitude_m: float) -> List[tuple[LocationPlan, Position]]:
    locations = [
        LocationPlan("Location A", north_offset_m=20.0, east_offset_m=0.0, altitude_m=altitude_m, probability=probabilities[0]),
        LocationPlan("Location B", north_offset_m=20.0, east_offset_m=20.0, altitude_m=altitude_m, probability=probabilities[1]),
        LocationPlan("Location C", north_offset_m=0.0, east_offset_m=20.0, altitude_m=altitude_m, probability=probabilities[2]),
    ]
    return [
        (location, offset_position(home, location.north_offset_m, location.east_offset_m, location.altitude_m))
        for location in locations
    ]


def prepare_for_mission(drone: DroneConnection, altitude_m: float) -> Position:
    drone.connect()
    home = drone.wait_for_position()
    print(
        f"{drone.name}: home position lat={home.lat:.6f}, "
        f"lon={home.lon:.6f}, alt={home.alt:.1f} m"
    )
    drone.set_guided()
    drone.arm()
    drone.takeoff(altitude_m)
    return home


def run_detection_mission(
    drone: DroneConnection,
    targets: Sequence[tuple[LocationPlan, Position]],
    *,
    seed: Optional[int],
    scan_delay_s: float,
) -> tuple[List[DetectionAttempt], Optional[tuple[LocationPlan, Position]]]:
    rng = random.Random(seed)
    attempts: List[DetectionAttempt] = []

    prepare_for_mission(drone, targets[0][1].alt)

    for step, (location, target_position) in enumerate(targets, start=1):
        print(f"\n{drone.name}: flying to {location.name}")
        drone.goto(target_position)
        drone.wait_until_close(target_position)

        if scan_delay_s > 0:
            print(f"{drone.name}: scanning at {location.name} for {scan_delay_s:.1f} s")
            time.sleep(scan_delay_s)

        random_value = rng.random()
        detected = random_value <= location.probability
        attempt = DetectionAttempt(
            step=step,
            location_name=location.name,
            probability=location.probability,
            random_value=random_value,
            detected=detected,
        )
        attempts.append(attempt)

        print(
            f"{drone.name}: {location.name} | probability={location.probability:.0%} | "
            f"random={random_value:.3f} | detected={'YES' if detected else 'NO'}"
        )

        if detected:
            print(f"{drone.name}: person detected at {location.name}")
            return attempts, (location, target_position)

    print(f"{drone.name}: mission finished with no detection")
    return attempts, None


def send_drone_directly_to_location(
    drone: DroneConnection,
    target_location: LocationPlan,
    target_position: Position,
) -> Position:
    print(
        f"\n{drone.name}: launching response to {target_location.name} "
        f"at lat={target_position.lat:.6f}, lon={target_position.lon:.6f}"
    )
    prepare_for_mission(drone, target_position.alt)
    drone.goto(target_position)
    final_position = drone.wait_until_close(target_position)
    print(f"{drone.name}: reached {target_location.name}")
    return final_position


def main() -> None:
    parser = add_detection_arguments(
        argparse.ArgumentParser(
            description="Drone 1 searches three waypoints; drone 2 responds to a confirmed detection."
        )
    )
    args = parser.parse_args()

    probabilities = parse_probabilities(args.probabilities)
    scout_seed = args.seed

    scout = DroneConnection("drone1", DRONE_PORTS["drone1"])
    scout.connect()
    home = scout.wait_for_position()
    targets = build_location_plan(home, probabilities, args.altitude)

    print("Mission waypoints:")
    for location, target in targets:
        print(
            f"- {location.name}: lat={target.lat:.6f}, lon={target.lon:.6f}, "
            f"alt={target.alt:.1f} m, probability={location.probability:.0%}"
        )

    scout = DroneConnection("drone1", DRONE_PORTS["drone1"])
    observer = DroneConnection("drone2", DRONE_PORTS["drone2"])

    scout_attempts, detected_target = run_detection_mission(
        scout,
        targets,
        seed=scout_seed,
        scan_delay_s=args.scan_delay,
    )

    observer_result: Optional[tuple[LocationPlan, Position]] = None
    if detected_target is not None:
        observer_position = send_drone_directly_to_location(observer, detected_target[0], detected_target[1])
        observer_result = (detected_target[0], observer_position)
    else:
        print("\ndrone2: no launch, because drone1 did not find a person.")

    print("\nMission summary:")
    for attempt in scout_attempts:
        print(
            f"- drone1: {attempt.location_name} | probability={attempt.probability:.0%} | "
            f"random={attempt.random_value:.3f} | detected={'YES' if attempt.detected else 'NO'}"
        )

    if detected_target is not None and observer_result is not None:
        print(
            f"- drone2: dispatched to {detected_target[0].name} "
            f"(lat={observer_result[1].lat:.6f}, lon={observer_result[1].lon:.6f}, alt={observer_result[1].alt:.1f} m)"
        )
    else:
        print("- drone2: stayed at its start position")


if __name__ == "__main__":
    main()