#!/usr/bin/env python3
"""Read a sensor log written by a sensor host (docs/spec/sensor-log.md).

A CSV log is read as it is; a binary log (HKBLOG version 1) is decoded. Prints the
columns, or with --csv writes the log as CSV.

    python3 tools/read_sensor_log.py logs/bmi055.bin
    python3 tools/read_sensor_log.py logs/bmi055.bin --csv bmi055.csv
"""
import argparse
import csv
import struct
import sys

# HKBLOG column types (Hakoniwa Drone's LogType): code -> struct format, size
TYPES = {0: ("<?", 1), 1: ("<b", 1), 2: ("<B", 1), 3: ("<h", 2), 4: ("<H", 2), 5: ("<i", 4), 6: ("<I", 4),
         7: ("<q", 8), 8: ("<Q", 8), 9: ("<f", 4), 10: ("<d", 8)}
STRING = 11


def read_hkblog(path):
    """Returns (column names, rows) of a binary sensor log."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:8] != b"HKBLOG\0\0":
        raise ValueError(f"{path}: not an HKBLOG file")
    version, flags, count = struct.unpack_from("<HHI", data, 8)
    if version != 1 or flags != 0:
        raise ValueError(f"{path}: unsupported HKBLOG version {version} flags {flags}")
    at = 16
    names, types = [], []
    for _ in range(count):
        kind, length = struct.unpack_from("<BH", data, at)
        at += 3
        names.append(data[at:at + length].decode())
        types.append(kind)
        at += length
    rows = []
    while at + 4 <= len(data):
        (size,) = struct.unpack_from("<I", data, at)
        at += 4
        end = at + size
        if end > len(data):
            break  # a row cut by a run that stopped without its cleanup
        row = []
        for kind in types:
            if kind == STRING:
                (length,) = struct.unpack_from("<H", data, at)
                row.append(data[at + 2:at + 2 + length].decode())
                at += 2 + length
            else:
                fmt, width = TYPES[kind]
                row.append(struct.unpack_from(fmt, data, at)[0])
                at += width
        if at != end:
            raise ValueError(f"{path}: a row does not match the header")
        rows.append(row)
    return names, rows


def read_sensor_log(path):
    """Returns (column names, rows) of a CSV or binary sensor log."""
    with open(path, "rb") as f:
        magic = f.read(8)
    if magic == b"HKBLOG\0\0":
        return read_hkblog(path)
    with open(path, newline="") as f:
        reader = csv.reader(f)
        names = next(reader)
        return names, [[int(r[0])] + [float(v) for v in r[1:]] for r in reader]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log")
    parser.add_argument("--csv", help="write the log as CSV to this file")
    args = parser.parse_args()
    names, rows = read_sensor_log(args.log)
    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(names)
            writer.writerows(rows)
    else:
        print(f"{len(rows)} rows, {len(names)} columns:")
        for name in names:
            print(f"  {name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
