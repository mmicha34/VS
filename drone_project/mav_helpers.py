from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional, Tuple

from pymavlink import mavutil


GLOBAL_POSITION_INT_ID = mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT


class MavlinkError(RuntimeError):
    pass


def connect_vehicle(
    endpoint: str,
    *,
    name: str = "vehicle",
    source_system: int = 250,
    timeout: float = 20.0,
):
    conn = mavutil.mavlink_connection(endpoint, source_system=source_system)
    heartbeat = conn.wait_heartbeat(timeout=timeout)
    if heartbeat is None:
        raise MavlinkError(f"{name}: no heartbeat received from {endpoint} within {timeout}s")

    conn.target_system = heartbeat.get_srcSystem()
    conn.target_component = heartbeat.get_srcComponent()
    return conn, heartbeat


def mode_id_for(conn, mode_name: str) -> int:
    mode_map = conn.mode_mapping()
    if not mode_map:
        raise MavlinkError("Autopilot did not provide a mode map")
    if mode_name not in mode_map:
        available = ", ".join(sorted(mode_map))
        raise MavlinkError(f"Mode {mode_name!r} is not available. Modes: {available}")
    return mode_map[mode_name]


def set_mode(conn, mode_name: str, settle_s: float = 2.0) -> None:
    mode_id = mode_id_for(conn, mode_name)
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    time.sleep(settle_s)


def _armed_from_heartbeat(msg: Any) -> bool:
    if msg is None:
        return False
    return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def wait_armed(conn, timeout: float = 12.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if _armed_from_heartbeat(msg):
            return
    raise MavlinkError(f"Vehicle did not report armed state within {timeout}s")


def arm_vehicle(conn, timeout: float = 12.0) -> None:
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
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
    wait_armed(conn, timeout=timeout)


def request_message_interval(conn, message_id: int, hz: float) -> None:
    interval_us = 0 if hz <= 0 else int(1_000_000 / hz)
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def wait_global_position(
    conn,
    *,
    timeout: float = 5.0,
    request_hz: Optional[float] = None,
):
    if request_hz is not None:
        request_message_interval(conn, GLOBAL_POSITION_INT_ID, request_hz)

    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg is not None:
            return msg
    raise MavlinkError(f"No GLOBAL_POSITION_INT received within {timeout}s")


def position_from_global_int(msg: Any) -> Dict[str, float]:
    return {
        "lat": msg.lat / 1e7,
        "lon": msg.lon / 1e7,
        "alt": msg.relative_alt / 1000.0,
    }


def wait_altitude(
    conn,
    target_alt_m: float,
    *,
    timeout: float = 30.0,
    tolerance_m: float = 0.8,
    request_hz: float = 2.0,
    label: str = "alt",
) -> float:
    deadline = time.time() + timeout
    last_alt = float("nan")
    while time.time() < deadline:
        msg = wait_global_position(conn, timeout=2, request_hz=request_hz)
        last_alt = msg.relative_alt / 1000.0
        print(f"{label}={last_alt:.1f}m")
        if last_alt >= target_alt_m - tolerance_m:
            return last_alt
    raise MavlinkError(
        f"Target altitude {target_alt_m:.1f}m not reached within {timeout}s; last altitude was {last_alt:.1f}m"
    )


def takeoff_vehicle(conn, altitude_m: float, *, monitor_timeout: float = 30.0) -> float:
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
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
    return wait_altitude(conn, altitude_m, timeout=monitor_timeout, request_hz=2.0)


def goto_global_relative_alt(conn, lat: float, lon: float, alt_m: float) -> None:
    type_mask = int(0b110111111000)
    conn.mav.set_position_target_global_int_send(
        0,
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        int(lat * 1e7),
        int(lon * 1e7),
        alt_m,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def horizontal_distance_m(a: Dict[str, float], b: Dict[str, float]) -> float:
    dx = (b["lon"] - a["lon"]) * 111320 * math.cos(math.radians((a["lat"] + b["lat"]) / 2))
    dy = (b["lat"] - a["lat"]) * 110540
    return math.sqrt(dx * dx + dy * dy)


def wait_until_near(
    conn,
    *,
    target: Dict[str, float],
    threshold_m: float = 2.0,
    timeout: float = 45.0,
    request_hz: float = 2.0,
    name: str = "vehicle",
) -> Tuple[Dict[str, float], float]:
    deadline = time.time() + timeout
    last_pos: Optional[Dict[str, float]] = None
    last_distance = float("inf")

    while time.time() < deadline:
        msg = wait_global_position(conn, timeout=2, request_hz=request_hz)
        last_pos = position_from_global_int(msg)
        last_distance = horizontal_distance_m(last_pos, target)
        print(
            f"{name}: lat={last_pos['lat']:.6f} lon={last_pos['lon']:.6f} "
            f"alt={last_pos['alt']:.1f}m dist={last_distance:.1f}m"
        )
        if last_distance <= threshold_m:
            return last_pos, last_distance

    raise MavlinkError(f"{name} did not reach target within {timeout}s; last distance was {last_distance:.1f}m")
