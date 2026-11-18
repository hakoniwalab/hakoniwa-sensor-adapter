"""Shared by the measurement cards (cards_*.py): a card is one measurement, a sketch of how the sensor was mounted
and moved on the left and the result on the right, with the axes and the expected values written on the plot.

The data come from runs of the error-characteristics tool (ec_common.run), the same runs as imu_figures.py and
mag_baro_gps_figures.py. The tool's --mount is in body FLU axes: x forward, y left, z up. The cards are in Japanese.
"""
import argparse
import tempfile
from pathlib import Path

import numpy as np
from matplotlib.patches import Arc, FancyArrowPatch, Rectangle

import ec_common
from ec_common import plt

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
INK, MUTED, TRUTH, BODY = "#14213d", "#8a96a8", "0.55", "#c9d6ea"
C0, C1, C2, C3 = "#14213d", "#2f6fd6", "#d1242f", "#1a7f37"
HIST = "#f2b8bd"
EARTH_R = 6378137.0
LAT0, LON0 = 35.6812, 139.7671
DEG = np.pi / 180.0


def arrow(ax, a, b, color=INK, lw=1.6, style="-|>", scale=14, **kw):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle=style, mutation_scale=scale, color=color, lw=lw, **kw))


def airframe(ax, size=0.25, alpha=1.0, centre=(0.0, 0.0)):
    """A quadcopter seen from above, forward up."""
    cx, cy = centre
    for ang in (45, 135, 225, 315):
        a = np.radians(ang)
        tip = (cx + size * np.sin(a), cy + size * np.cos(a))
        ax.plot([cx, tip[0]], [cy, tip[1]], color=BODY, lw=7 * size / 0.25, solid_capstyle="round", alpha=alpha,
                zorder=1)
        ax.add_patch(plt.Circle(tip, size * 0.28, fc="none", ec=BODY, lw=2, alpha=alpha))
    h = 0.24 * size
    ax.add_patch(Rectangle((cx - h, cy - h), 2 * h, 2 * h, fc=BODY, ec="none", alpha=alpha, zorder=1))


def spin_arrow(ax, radius, text, color=INK):
    ax.add_patch(Arc((0, 0), 2 * radius, 2 * radius, theta1=40, theta2=140, color=color, lw=1.8))
    a = np.radians(140)
    arrow(ax, (radius * np.cos(np.radians(132)), radius * np.sin(np.radians(132))),
          (radius * np.cos(a + 0.12), radius * np.sin(a + 0.12)), color=color, scale=16)
    ax.text(0, radius + 0.03, text, ha="center", va="bottom", fontsize=10, color=color)


def chip(ax, xy, color, size=0.05, label=None, **text):
    """A sensor drawn as a small square, with an optional label to its right."""
    ax.add_patch(Rectangle((xy[0] - size / 2, xy[1] - size / 2), size, size, fc=color, ec="none", zorder=6))
    if label:
        ax.text(xy[0] + size, xy[1], label, fontsize=text.pop("fontsize", 9), color=text.pop("color", color),
                va=text.pop("va", "center"), **text)


def sketch_axes(ax, xlim=(-0.45, 0.45), ylim=(-0.42, 0.5), title="測り方"):
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(title, fontsize=11, color=INK, loc="left")


def notes(ax, lines, y=-0.3, x=None, size=8.5, color=MUTED):
    """Lines of text at the foot of a sketch."""
    x = ax.get_xlim()[0] + 0.02 if x is None else x
    ax.text(x, y, "\n".join(lines), fontsize=size, color=color, va="top", linespacing=1.6)


def card(title, right=1):
    """A card: the sketch on the left, one or two result plots on the right."""
    fig = plt.figure(figsize=(12.5, 4.6))
    grid = fig.add_gridspec(1, 1 + right, width_ratios=[0.85] + [1.15 / right] * right)
    fig.suptitle(title, fontsize=13.5, color=INK, x=0.02, ha="left", fontweight="bold")
    axes = [fig.add_subplot(grid[0, i]) for i in range(1 + right)]
    return (fig, *axes)


def result_title(ax, text):
    ax.set_title(text, fontsize=11, color=INK, loc="left")


def hist_against_sigma(ax, values, sigma, bins=50, label="測った値", count_label="その値が出た回数"):
    """A histogram of the measured values against the normal distribution of the configured sigma."""
    values = np.asarray(values)
    _, edges, _ = ax.hist(values, bins=bins, color=HIST, label=f"{label}（標準偏差 {values.std():.3g}）")
    width = edges[1] - edges[0]
    x = np.linspace(-4 * sigma, 4 * sigma, 300)
    ax.plot(x, values.size * width * np.exp(-0.5 * (x / sigma) ** 2) / (sigma * np.sqrt(2 * np.pi)), color=C2, lw=2,
            label=f"設定 σ = {sigma:g} なら、こう出るはず")
    ax.set_ylabel(count_label)
    top = ax.get_ylim()[1]
    ax.set_ylim(0, top * 1.25)
    ax.legend(loc="upper left", fontsize=8.3)


def save(fig, args, name):
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    path = Path(args.out) / f"{name}.png"
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print(f"wrote {path}")


def run(args, sensor, name, config, motion, duration, **extra):
    return ec_common.run(args, sensor, name, config, motion, duration, **extra)


def sampled(d, flag="updated"):
    return ec_common.sampled(d, flag)


def north_east(d, lat="lat", lon="lon"):
    north = np.radians(d[lat] - LAT0) * EARTH_R
    east = np.radians(d[lon] - LON0) * EARTH_R * np.cos(np.radians(LAT0))
    return north, east


def main(doc, makers):
    parser = argparse.ArgumentParser(description=doc.splitlines()[0])
    parser.add_argument("--tool", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--implementation", default="hakoniwa")
    parser.add_argument("--only", nargs="*", help="cards to make, by function name")
    parser.add_argument("--work", help="where the tool's CSV files go (default: a temporary directory), so that --out can be "
                                       "the documentation folder")
    args = parser.parse_args()
    args.work = args.work or tempfile.mkdtemp(prefix="error-cards-")
    Path(args.out).mkdir(parents=True, exist_ok=True)
    for make in makers:
        if not args.only or make.__name__ in args.only:
            make(args)
