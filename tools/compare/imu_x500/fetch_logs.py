#!/usr/bin/env python3
"""Fetch the public X500 ULogs listed in flight_ids.csv into an external cache."""
import argparse
import csv
import shutil
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_IDS = HERE.parents[2] / "docs/evaluation/imu-x500/data/flight_ids.csv"


def local_log(root, log_id):
    if not root:
        return None
    root = Path(root)
    for path in (root / f"{log_id}.ulg", root / "logs_vib" / f"{log_id}.ulg", root / "logs" / f"{log_id}.ulg"):
        if path.is_file():
            return path
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ids", type=Path, default=DEFAULT_IDS)
    parser.add_argument("--cache-dir", type=Path, default=Path.home() / ".cache/hakoniwa-imu-x500")
    parser.add_argument("--logs-dir", type=Path, help="use local copies here instead of the network")
    args = parser.parse_args()
    ids = [row["log_id"] for row in csv.DictReader(args.ids.open())]
    args.cache_dir.mkdir(parents=True, exist_ok=True)
    for log_id in ids:
        dst = args.cache_dir / f"{log_id}.ulg"
        if dst.is_file():
            print(f"cached {dst}")
            continue
        src = local_log(args.logs_dir, log_id)
        if src:
            shutil.copyfile(src, dst)
            print(f"copied {src} -> {dst}")
            continue
        url = f"https://cdn.logs.px4.io/{log_id}.ulg"
        partial = dst.with_suffix(".ulg.part")
        print(f"fetching {url}")
        try:
            urllib.request.urlretrieve(url, partial)
            partial.replace(dst)
        finally:
            if partial.exists():
                partial.unlink()
        print(f"wrote {dst}")


if __name__ == "__main__":
    main()
