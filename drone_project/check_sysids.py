from mav_helpers import connect_vehicle


PORTS = {
    "drone1": "udp:127.0.0.1:14550",
    "drone2": "udp:127.0.0.1:14560",
}


def main() -> None:
    for name, endpoint in PORTS.items():
        print(f"Connecting to {name} on {endpoint} ...")
        conn, heartbeat = connect_vehicle(endpoint, name=name)
        print(
            f"{name}: sysid={heartbeat.get_srcSystem()} "
            f"compid={heartbeat.get_srcComponent()} target={conn.target_system}:{conn.target_component}"
        )


if __name__ == "__main__":
    main()
