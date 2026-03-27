from mav_helpers import (
    connect_vehicle,
    goto_global_relative_alt,
    horizontal_distance_m,
    position_from_global_int,
    request_message_interval,
    set_mode,
    wait_global_position,
    wait_until_near,
)


DRONE1_PORT = "udp:127.0.0.1:14550"
DRONE2_PORT = "udp:127.0.0.1:14560"
MOVE_NORTH_METERS = 8.0


def north_offset_degrees(meters: float) -> float:
    return meters / 110540.0


def main() -> None:
    drone1, hb1 = connect_vehicle(DRONE1_PORT, name="drone1")
    drone2, hb2 = connect_vehicle(DRONE2_PORT, name="drone2")

    print(f"drone1 connected: sysid={hb1.get_srcSystem()} compid={hb1.get_srcComponent()}")
    print(f"drone2 connected: sysid={hb2.get_srcSystem()} compid={hb2.get_srcComponent()}")

    set_mode(drone1, "GUIDED")
    set_mode(drone2, "GUIDED")
    request_message_interval(drone1, 33, 2)
    request_message_interval(drone2, 33, 2)

    p1 = position_from_global_int(wait_global_position(drone1, timeout=5))
    p2 = position_from_global_int(wait_global_position(drone2, timeout=5))

    print("Start:")
    print("drone1", p1)
    print("drone2", p2)

    target1 = {
        "lat": p1["lat"] + north_offset_degrees(MOVE_NORTH_METERS),
        "lon": p1["lon"],
        "alt": max(p1["alt"], 10.0),
    }
    print(f"Sending drone1 to new point: {target1}")
    goto_global_relative_alt(drone1, target1["lat"], target1["lon"], target1["alt"])
    p1_after, dist1 = wait_until_near(
        drone1,
        target=target1,
        threshold_m=2.0,
        timeout=45.0,
        name="drone1",
    )
    print(f"drone1 reached target, distance={dist1:.1f}m")

    target2 = {
        "lat": p1_after["lat"],
        "lon": p1_after["lon"],
        "alt": max(p1_after["alt"], 10.0),
    }
    start_gap = horizontal_distance_m(p2, target2)
    print(f"Sending drone2 to drone1 position. Initial separation={start_gap:.1f}m")
    goto_global_relative_alt(drone2, target2["lat"], target2["lon"], target2["alt"])
    _, dist2 = wait_until_near(
        drone2,
        target=target2,
        threshold_m=2.0,
        timeout=60.0,
        name="drone2",
    )
    print(f"drone2 reached drone1 area, distance={dist2:.1f}m")


if __name__ == "__main__":
    main()
