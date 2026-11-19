"""Shared by the card scripts: running the error-characteristics tool, and the Allan deviation."""
import csv
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent


def run(args, sensor, name, config, motion, duration, **extra):
    """Runs the tool for one sensor and returns its CSV columns as arrays."""
    out = Path(getattr(args, "work", None) or args.out) / "csv" / f"{sensor}-{name}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    command = [args.tool, "--implementation", args.implementation, "--sensor", sensor, "--config",
               str(HERE / "config" / sensor / config), "--motion", motion, "--duration", str(duration),
               "--out", str(out)]
    for key, value in extra.items():
        command += [f"--{key.replace('_', '-')}", str(value)]
    subprocess.run(command, check=True, capture_output=True)
    with open(out) as handle:
        rows = list(csv.DictReader(handle))
    return {key: np.array([float(row[key]) for row in rows]) for key in rows[0]}


def sampled(data, flag="updated"):
    """The rows where the sensor measured a new value."""
    mask = data[flag] > 0
    return {key: value[mask] for key, value in data.items()}


def allan(values, dt, taus):
    """Non-overlapping Allan deviation of a series sampled every dt."""
    out = []
    for tau in taus:
        m = int(round(tau / dt))
        n = len(values) // m if m > 0 else 0
        if n < 3:
            out.append(np.nan)
            continue
        means = values[: n * m].reshape(n, m).mean(axis=1)
        out.append(np.sqrt(0.5 * np.mean(np.diff(means) ** 2)))
    return np.array(out)
