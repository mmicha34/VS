import time

from mav_helpers import connect_vehicle, position_from_global_int, request_message_interval


PORTS = {
    "drone1": "udp:127.0.0.1:14550",
    "drone2": "udp:127.0.0.1:14560",
}


def main() -> None:
    connections = {}

    for name, endpoint in PORTS.items():
        print(f"Connecting to {name} on {endpoint} ...")
        conn, heartbeat = connect_vehicle(endpoint, name=name)
        print(f"{name}: sysid={heartbeat.get_srcSystem()} compid={heartbeat.get_srcComponent()}")
        request_message_interval(conn, 33, 2)
        connections[name] = conn

    print("Reading both drones. Press Ctrl+C to stop.")

    while True:
        for name, conn in connections.items():
            msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
            if msg is None:
                continue
            pos = position_from_global_int(msg)
            print(
                f"{name} (sysid={msg.get_srcSystem()}): "
                f"lat={pos['lat']:.6f}, lon={pos['lon']:.6f}, alt={pos['alt']:.1f}m"
            )
        time.sleep(1)


if __name__ == "__main__":
    main()
