from pathlib import Path
from typing import Dict, Tuple

Waypoint = Tuple[float, float, float]

def load_waypoints(file_path: str) -> Dict[str, Waypoint]:
    """
    Load waypoints from a simple text file.

    Expected format per non-comment line:
        NAME,LAT,LON,ALT

    Returns:
        {
            "WP_A": (lat, lon, alt),
            ...
        }
    """
    waypoints: Dict[str, Waypoint] = {}
    path = Path(file_path)

    if not path.exists():
        raise FileNotFoundError(f"Waypoint file not found: {file_path}")

    for line_number, raw_line in enumerate(path.read_text().splitlines(), start=1):
        line = raw_line.strip()

        if not line or line.startswith("#"):
            continue

        parts = [p.strip() for p in line.split(",")]
        if len(parts) != 4:
            raise ValueError(
                f"Invalid format on line {line_number}: {raw_line!r}. "
                "Expected NAME,LAT,LON,ALT"
            )

        name, lat_str, lon_str, alt_str = parts

        try:
            lat = float(lat_str)
            lon = float(lon_str)
            alt = float(alt_str)
        except ValueError as exc:
            raise ValueError(
                f"Invalid numeric value on line {line_number}: {raw_line!r}"
            ) from exc

        waypoints[name] = (lat, lon, alt)

    return waypoints


if __name__ == "__main__":
    # Change this path if needed
    file_path = "waypoints.txt"

    wps = load_waypoints(file_path)

    print("Loaded waypoints:")
    for name, (lat, lon, alt) in wps.items():
        print(f"{name}: lat={lat}, lon={lon}, alt={alt}")
