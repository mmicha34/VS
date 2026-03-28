from __future__ import annotations

import argparse
import random
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence


@dataclass(frozen=True)
class DetectionAttempt:
    step: int
    probability: float
    random_value: float
    detected: bool


def parse_probabilities(raw_values: Optional[Iterable[float]]) -> List[float]:
    if raw_values is None:
        return [0.20, 0.40, 0.65, 0.85]

    probabilities = [float(value) for value in raw_values]
    if not probabilities:
        raise ValueError("At least one probability is required")

    for probability in probabilities:
        if not 0.0 <= probability <= 1.0:
            raise ValueError("Probabilities must be between 0.0 and 1.0")

    return probabilities


def add_detection_arguments(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    parser.add_argument(
        "--probabilities",
        nargs="+",
        type=float,
        metavar="P",
        help="Detection probabilities for each scan step, e.g. --probabilities 0.2 0.5 0.8",
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
        help="Seconds to wait between scan attempts",
    )
    return parser


def run_first_detection_scenario(
    drone_name: str,
    probabilities: Sequence[float],
    *,
    seed: Optional[int] = None,
    scan_delay_s: float = 2.0,
) -> List[DetectionAttempt]:
    rng = random.Random(seed)
    attempts: List[DetectionAttempt] = []

    print(f"\n{drone_name}: Scenario 1 started")
    print(f"{drone_name}: scanning for a person with probabilities {list(probabilities)}")
    if seed is not None:
        print(f"{drone_name}: using random seed {seed}")

    for index, probability in enumerate(probabilities, start=1):
        if scan_delay_s > 0:
            time.sleep(scan_delay_s)

        random_value = rng.random()
        detected = random_value <= probability
        attempt = DetectionAttempt(
            step=index,
            probability=probability,
            random_value=random_value,
            detected=detected,
        )
        attempts.append(attempt)

        print(
            f"{drone_name}: scan {index} | "
            f"probability={probability:.0%} | "
            f"random={random_value:.3f} | "
            f"detected={'YES' if detected else 'NO'}"
        )

        if detected:
            print(f"{drone_name}: person detected on scan {index}")
            break

    if not attempts[-1].detected:
        print(f"{drone_name}: no person detected in Scenario 1")

    return attempts
