import time

from pymavlink import mavutil

from mav_helpers import connect_vehicle


PORTS = {
    "port14550": "udp:127.0.0.1:14550",
    "port14560": "udp:127.0.0.1:14560",
}


def main() -> None:
    for name, endpoint in PORTS.items():
        print(f"\n--- checking {name} / {endpoint} ---")
        conn, _ = connect_vehicle(endpoint, name=name, source_system=250)
        start = time.time()
        count = 0

        while time.time() - start < 8 and count < 10:
            msg = conn.recv_match(blocking=True, timeout=2)
            if msg is None:
                continue
            print(
                f"type={msg.get_type()} sysid={msg.get_srcSystem()} "
                f"compid={msg.get_srcComponent()}"
            )
            count += 1

        if count == 0:
            print("No MAVLink messages observed after heartbeat")


if __name__ == "__main__":
    main()
