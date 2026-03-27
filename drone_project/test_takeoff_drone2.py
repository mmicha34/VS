from mav_helpers import arm_vehicle, connect_vehicle, set_mode, takeoff_vehicle


ENDPOINT = "udp:127.0.0.1:14560"
TARGET_ALT_M = 10.0


def main() -> None:
    conn, heartbeat = connect_vehicle(ENDPOINT, name="drone2")
    print(
        f"Connected to sysid={heartbeat.get_srcSystem()} "
        f"compid={heartbeat.get_srcComponent()}"
    )

    print("Switching to GUIDED...")
    set_mode(conn, "GUIDED")

    print("Arming...")
    arm_vehicle(conn)

    print(f"Takeoff to {TARGET_ALT_M:.0f} meters...")
    takeoff_vehicle(conn, TARGET_ALT_M)


if __name__ == "__main__":
    main()
