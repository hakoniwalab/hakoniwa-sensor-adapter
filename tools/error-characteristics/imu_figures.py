"""IMU error characteristics, one error per figure, from runs of the error-characteristics tool.

    python3 imu_figures.py --tool <build>/error-characteristics --out <dir> [--implementation hakoniwa]

Each figure runs the tool with a configuration of config/imu/ that turns on one error only, on a motion
written as a formula, and lays the measurement over the truth or over the value the configuration asks for.
Needs numpy and matplotlib.
"""
import argparse
import csv
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
CONFIG = HERE / "config" / "imu"
DEG = np.pi / 180.0
G0 = 9.80665
TRUTH = "0.6"
MEAS = "#d1242f"


def run(args, name, config, motion, duration, **extra):
    out = Path(args.out) / "csv" / f"{name}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    command = [args.tool, "--implementation", args.implementation, "--sensor", "imu", "--config",
               str(CONFIG / config), "--motion", motion, "--duration", str(duration), "--out", str(out)]
    for key, value in extra.items():
        command += [f"--{key.replace('_', '-')}", str(value)]
    subprocess.run(command, check=True, capture_output=True)
    with open(out) as handle:
        rows = list(csv.DictReader(handle))
    return {key: np.array([float(row[key]) for row in rows]) for key in rows[0]}


def sampled(data, part):
    mask = data[f"{part}_updated"] > 0
    return {key: value[mask] for key, value in data.items()}


def save(fig, args, name):
    fig.tight_layout()
    path = Path(args.out) / f"{name}.png"
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print(f"wrote {path}")


def mount(args):
    """Lever arm and axes: the same turn, the IMU at the centre, 0.1 m and 0.2 m forward, and turned 90 deg."""
    rate = np.pi  # 180 deg/s about z
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    for r in (0.0, 0.1, 0.2):
        d = run(args, f"mount-r{r}", "truth.json", "turn", 1.0, rate=rate, mount=f"{r},0,0,0,0,0")
        axes[0].plot(d["time"], d["ax"], label=f"IMU {r:.1f} m forward")
        axes[0].axhline(-rate * rate * r, color=TRUTH, lw=0.8, ls="--")
    axes[0].set_title("Lever arm: accel x while turning at 180 deg/s\n(dashed: -omega^2 r)")
    axes[0].set_xlabel("time [s]"); axes[0].set_ylabel("accel x [m/s^2]"); axes[0].legend(fontsize=8)
    d0 = run(args, "mount-yaw0", "truth.json", "step", 2.0, mount="0,0,0,0,0,0")
    d90 = run(args, "mount-yaw90", "truth.json", "step", 2.0, mount="0,0,0,0,0,90")
    axes[1].plot(d0["time"], d0["gx"], label="mounted straight: gyro x")
    axes[1].plot(d90["time"], d90["gy"], "--", label="turned 90 deg (yaw): gyro y")
    axes[1].plot(d90["time"], d90["gx"], ":", label="turned 90 deg (yaw): gyro x")
    axes[1].set_title("Orientation: a turn about body x, seen in sensor axes")
    axes[1].set_xlabel("time [s]"); axes[1].set_ylabel("angular rate [rad/s]"); axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    save(fig, args, "imu-1-mount")


def noise(args):
    """White noise: the spread against the configured sigma, and noise density against the period."""
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    d = sampled(run(args, "noise", "noise.json", "rest", 20.0), "gyro")
    sigma = 0.0008726646
    values = d["gx"] / DEG
    axes[0].hist(values, bins=60, density=True, color=MEAS, alpha=0.6, label=f"measured, sd {values.std():.4f}")
    x = np.linspace(values.min(), values.max(), 200)
    s = sigma / DEG
    axes[0].plot(x, np.exp(-0.5 * (x / s) ** 2) / (s * np.sqrt(2 * np.pi)), color="k", lw=1,
                 label=f"configured sigma {s:.4f}")
    axes[0].set_title(f"Gyro noise at rest ({len(values)} samples)")
    axes[0].set_xlabel("gyro x [deg/s]"); axes[0].legend(fontsize=8)
    density = 0.00024
    periods, measured = [], []
    for ms in (2, 4, 8):
        dd = sampled(run(args, f"noise-density-{ms}ms", f"noise-density-{ms}ms.json", "rest", 20.0), "gyro")
        periods.append(ms)
        measured.append(dd["gx"].std())
    p = np.linspace(1.5, 9, 100)
    axes[1].plot(p, density / np.sqrt(p * 1e-3), color="k", lw=1, label="sigma = density / sqrt(period)")
    axes[1].plot(periods, measured, "o", color=MEAS, label="measured sd")
    axes[1].set_title("Noise density: sigma follows the update period")
    axes[1].set_xlabel("update period [ms]"); axes[1].set_ylabel("gyro x sd [rad/s]"); axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    save(fig, args, "imu-2-noise")


def offset(args):
    """Per-unit offset: one value drawn per sensor at start; 60 units with different seeds."""
    gyro, accel = [], []
    for seed in range(1, 1001):
        d = run(args, f"offset-{seed}", "offset.json", "rest", 0.002, seed=seed)
        gyro.append(d["gx"][-1] / DEG)
        accel.append((d["az"][-1] - G0) / G0 * 1000.0)
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    for ax, values, sigma, label in ((axes[0], gyro, 1.0, "gyro x zero-rate offset [deg/s]"),
                                     (axes[1], accel, 20.0, "accel z zero-g offset [mg]")):
        values = np.array(values)
        ax.hist(values, bins=40, density=True, color=MEAS, alpha=0.6, label=f"1000 units, sd {values.std():.2f}")
        x = np.linspace(-3.5 * sigma, 3.5 * sigma, 200)
        ax.plot(x, np.exp(-0.5 * (x / sigma) ** 2) / (sigma * np.sqrt(2 * np.pi)), color="k", lw=1,
                label=f"configured sigma {sigma:g}")
        ax.set_xlabel(label); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    axes[0].set_title("Every unit has its own offset (seeds 1-1000)")
    axes[1].set_title("The same for the accelerometer")
    save(fig, args, "imu-3-offset")


def allan(values, dt, taus):
    out = []
    for tau in taus:
        m = int(round(tau / dt))
        n = len(values) // m
        if n < 3:
            out.append(np.nan)
            continue
        means = values[: n * m].reshape(n, m).mean(axis=1)
        out.append(np.sqrt(0.5 * np.mean(np.diff(means) ** 2)))
    return np.array(out)


def drift(args):
    """Gauss-Markov drift (sigma 0.005 deg/s, tau 100 s): slow wander, and the Allan deviation with noise."""
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    for seed in (1, 2, 3):
        d = sampled(run(args, f"drift-{seed}", "drift.json", "rest", 1800.0, seed=seed, every=4), "gyro")
        axes[0].plot(d["time"] / 60.0, d["gx"] / DEG, lw=0.8, label=f"unit {seed}")
    axes[0].axhline(0.005, color=TRUTH, ls="--", lw=0.8); axes[0].axhline(-0.005, color=TRUTH, ls="--", lw=0.8)
    axes[0].set_title("Gyro drift at rest, 3 units (dashed: +-sigma)")
    axes[0].set_xlabel("time [min]"); axes[0].set_ylabel("gyro x [deg/s]"); axes[0].legend(fontsize=8)
    d = sampled(run(args, "noise-drift", "noise-drift.json", "rest", 1800.0, seed=1, every=4), "gyro")
    dt = 0.004
    taus = np.logspace(np.log10(dt), np.log10(400), 40)
    adev = allan(d["gx"] / DEG, dt, taus)
    axes[1].loglog(taus, adev, color=MEAS, label="measured (noise + drift)")
    axes[1].loglog(taus, 0.0008726646 / DEG / np.sqrt(taus / dt), color="k", lw=0.8, ls="--",
                   label="white noise: sigma / sqrt(tau / period)")
    axes[1].set_title("Allan deviation: noise falls with averaging, drift stops it")
    axes[1].set_xlabel("averaging time tau [s]"); axes[1].set_ylabel("Allan deviation [deg/s]")
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3, which="both")
    save(fig, args, "imu-4-drift")


def delay(args):
    """Update period 4 ms alone, then with a 10 ms delay, on a step at t = 1.002 s."""
    # the step falls between two samples (they are at multiples of 4 ms)
    a = run(args, "period", "period.json", "step", 1.05, t0=1.002)
    b = run(args, "period-delay", "period-delay.json", "step", 1.05, t0=1.002)
    w = (a["time"] >= 0.990) & (a["time"] <= 1.025)
    t = (a["time"][w] - 1.002) * 1000.0
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    for ax, d, label, title in ((axes[0], a, "period 4 ms", "Update period: the value is held between samples"),
                                (axes[1], b, "period 4 ms + delay 10 ms", "Delay: the same samples, 10 ms late")):
        ax.step(t, d["true_gx"][w], where="post", color=TRUTH, label="truth")
        ax.step(t, d["gx"][w], where="post", color=MEAS, label=label)
        ax.set_title(title); ax.set_xlabel("time from the step [ms]"); ax.set_ylabel("gyro x [rad/s]")
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
    save(fig, args, "imu-5-delay-period")


def resolution_range(args):
    """Resolution 0.061 deg/s on a slow ramp; full scale 2000 deg/s on a fast one."""
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    slow = run(args, "resolution", "resolution-range.json", "ramp", 0.5, rate=0.01)
    axes[0].plot(slow["time"], slow["true_gx"] / DEG, color=TRUTH, label="truth")
    axes[0].step(slow["time"], slow["gx"] / DEG, where="post", color=MEAS, label="resolution 0.061 deg/s")
    axes[0].set_title("Resolution: a slow ramp becomes steps")
    axes[0].set_xlabel("time [s]"); axes[0].set_ylabel("gyro x [deg/s]")
    fast = run(args, "range", "resolution-range.json", "ramp", 5.0, rate=10.0, every=10)
    axes[1].plot(fast["time"], fast["true_gx"] / DEG, color=TRUTH, label="truth")
    axes[1].plot(fast["time"], fast["gx"] / DEG, color=MEAS, label="full scale 2000 deg/s")
    sat = fast["gyro_flags"] > 0
    if sat.any():
        axes[1].axvline(fast["time"][sat][0], color="k", lw=0.8, ls=":", label="saturated from here")
    axes[1].set_title("Range: the value stops at full scale")
    axes[1].set_xlabel("time [s]"); axes[1].set_ylabel("gyro x [deg/s]")
    for ax in axes:
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
    save(fig, args, "imu-6-resolution-range")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tool", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--implementation", default="hakoniwa")
    parser.add_argument("--only", nargs="*", help="figures to make: mount noise offset drift delay resolution_range")
    args = parser.parse_args()
    Path(args.out).mkdir(parents=True, exist_ok=True)
    figures = {"mount": mount, "noise": noise, "offset": offset, "drift": drift, "delay": delay,
               "resolution_range": resolution_range}
    for name in args.only or figures:
        figures[name](args)


if __name__ == "__main__":
    main()
