#!/usr/bin/env python3
"""Resolve the recommended Istanbul EKF profile for a given bag path/name."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
PROFILE_DIR = SCRIPT_DIR.parent / "kalman_filter_localization_ros2" / "param" / "profiles"
SHARED_PROFILE = PROFILE_DIR / "istanbul_all_sensors_bag.yaml"
BAG46_PROFILE = PROFILE_DIR / "istanbul_all_sensors_bag4_6.yaml"


def choose_profile_name(bag_name: str) -> str:
    if (
        "all-sensors-bag4" in bag_name
        or "all-sensors-bag5" in bag_name
        or "all-sensors-bag6" in bag_name
    ):
        return BAG46_PROFILE.name
    return SHARED_PROFILE.name


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--bag-path",
        required=True,
        help="bag directory path or bag directory name (e.g. data/istanbul/all-sensors-bag5_compressed)",
    )
    p.add_argument(
        "--print-name",
        action="store_true",
        help="print only the profile filename instead of the absolute path",
    )
    return p


def main() -> int:
    args = build_parser().parse_args()
    bag_name = Path(args.bag_path).name
    profile_name = choose_profile_name(bag_name)
    profile_path = PROFILE_DIR / profile_name
    if not profile_path.exists():
        print(f"ERROR: profile not found: {profile_path}", file=sys.stderr)
        return 2
    print(profile_name if args.print_name else str(profile_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
