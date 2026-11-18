"""Shared by the figure scripts: running the error-characteristics tool and the plot helpers."""
import csv
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
TRUTH = "0.6"
MEAS = "#d1242f"


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


def save(fig, args, names):
    """Writes each panel of the figure as its own PNG, one name per axes (a colorbar goes with the panel before it).

    The pages put the panels under the error they show, so one figure may give panels to two sections."""
    fig.tight_layout()
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    panels = [ax for ax in fig.axes if not hasattr(ax, "_colorbar")]
    colorbars = [ax for ax in fig.axes if hasattr(ax, "_colorbar")]
    names = [names] if isinstance(names, str) else names
    if len(names) != len(panels):
        raise ValueError(f"{len(panels)} panels, {len(names)} names")
    for i, (ax, name) in enumerate(zip(panels, names)):
        parts = [ax] + [cb for cb in colorbars if panels.index(cb._colorbar.mappable.axes) == i]
        box = matplotlib.transforms.Bbox.union([a.get_tightbbox(renderer) for a in parts])
        path = Path(args.out) / f"{name}.png"
        fig.savefig(path, dpi=110, bbox_inches=box.transformed(fig.dpi_scale_trans.inverted()).padded(0.08))
        print(f"wrote {path}")
    plt.close(fig)


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


def gaussian(ax, sigma, span=3.5, label=None):
    x = np.linspace(-span * sigma, span * sigma, 200)
    ax.plot(x, np.exp(-0.5 * (x / sigma) ** 2) / (sigma * np.sqrt(2 * np.pi)), color="k", lw=1,
            label=label or f"configured sigma {sigma:g}")
