"""Magnetometer, barometer and GPS error characteristics, one error per figure (the IMU's are in imu_figures.py).

    python3 mag_baro_gps_figures.py --tool <build>/error-characteristics --out <dir> [--only mag baro gps]

The configurations of config/mag, config/baro and config/gps each turn on one error. The world is the tool's
illustrative one: Tokyo, a 46000 nT field (declination -7.5 deg, inclination 49.5 deg), the standard atmosphere.
"""
import argparse

import numpy as np

import ec_common
from ec_common import MEAS, TRUTH, allan, gaussian, plt, save

EARTH_R = 6378137.0
LAT0, LON0 = 35.6812, 139.7671


def run(args, sensor, name, config, motion, duration, **extra):
    return ec_common.run(args, sensor, name, config, motion, duration, **extra)


def sampled(d):
    return ec_common.sampled(d, "updated")


def north_east(d, lat="lat", lon="lon"):
    north = np.radians(d[lat] - LAT0) * EARTH_R
    east = np.radians(d[lon] - LON0) * EARTH_R * np.cos(np.radians(LAT0))
    return north, east


# ---------------------------------------------------------------------------------------------------- magnetometer
def mag(args):
    turn = 2 * np.pi / 4.0  # one turn in 4 s
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    a = run(args, "mag", "mount-yaw0", "truth.json", "turn", 4.0, rate=turn)
    b = run(args, "mag", "mount-yaw90", "truth.json", "turn", 4.0, rate=turn, mount="0,0,0,0,0,90")
    axes[0].plot(a["time"], a["mx"] / 1000, label="mounted straight: x")
    axes[0].plot(b["time"], b["mx"] / 1000, "--", label="turned 90 deg (yaw): x")
    axes[0].set_title("Mounting: one turn; the same field, a quarter turn apart")
    axes[0].set_xlabel("time [s]"); axes[0].set_ylabel("field x [uT]"); axes[0].legend(fontsize=8)
    c = run(args, "mag", "iron", "iron.json", "turn", 4.0, rate=turn)
    axes[1].plot(c["true_mx"] / 1000, c["true_my"] / 1000, color=TRUTH, label="truth: a circle about 0")
    axes[1].plot(c["mx"] / 1000, c["my"] / 1000, color=MEAS, label="hard + soft iron (configured in FRD axes)")
    # The configuration is in the sensor's FRD axes (Drone PRO's), the plot in FLU: the hard iron (3000, -1500) nT
    # is (3000, +1500) here, and the soft iron's off-diagonal terms mix the vertical field (about 35 uT) into x and y.
    centre = ((c["mx"].max() + c["mx"].min()) / 2000, (c["my"].max() + c["my"].min()) / 2000)
    axes[1].plot([0], [0], "+", color=TRUTH)
    axes[1].plot([centre[0]], [centre[1]], "+", color=MEAS, ms=10)
    axes[1].annotate(f"centre ({centre[0]:.1f}, {centre[1]:.1f}) uT", centre, xytext=(6, 6),
                     textcoords="offset points", fontsize=8, color=MEAS)
    axes[1].set_aspect("equal"); axes[1].set_title("Hard and soft iron: the circle shifts and squashes")
    axes[1].set_xlabel("field x [uT]"); axes[1].set_ylabel("field y [uT]"); axes[1].legend(fontsize=8, loc="lower left")
    for ax in axes:
        ax.grid(alpha=0.3)
    save(fig, args, "mag-1-mount-iron")

    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    d = sampled(run(args, "mag", "noise", "noise.json", "rest", 60.0))
    err = d["mx"] - d["true_mx"]
    axes[0].hist(err, bins=50, density=True, color=MEAS, alpha=0.6, label=f"{len(err)} samples, sd {err.std():.1f}")
    gaussian(axes[0], 30.0)
    axes[0].set_title("Noise at rest (period 20 ms)"); axes[0].set_xlabel("field x - truth [nT]")
    axes[0].legend(fontsize=8)
    for seed in (1, 2, 3):
        dd = sampled(run(args, "mag", f"drift-{seed}", "drift.json", "rest", 1800.0, seed=seed, every=20))
        axes[1].plot(dd["time"] / 60.0, dd["mx"] - dd["true_mx"], lw=0.8, label=f"unit {seed}")
    axes[1].axhline(100, color=TRUTH, ls="--", lw=0.8); axes[1].axhline(-100, color=TRUTH, ls="--", lw=0.8)
    axes[1].set_title("Gauss-Markov drift (sigma 100 nT, tau 60 s), 3 units")
    axes[1].set_xlabel("time [min]"); axes[1].set_ylabel("field x - truth [nT]"); axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    save(fig, args, "mag-2-noise-drift")

    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    p = run(args, "mag", "period", "period.json", "yawstep", 1.2, t0=1.005)
    q = run(args, "mag", "period-delay", "period-delay.json", "yawstep", 1.2, t0=1.005)
    w = (p["time"] >= 0.95) & (p["time"] <= 1.15)
    t = (p["time"][w] - 1.005) * 1000
    axes[0].step(t, p["true_mx"][w], where="post", color=TRUTH, label="truth")
    axes[0].step(t, p["mx"][w], where="post", color=MEAS, label="period 20 ms")
    axes[0].step(t, q["mx"][w], where="post", color="#0b2a5b", label="period 20 ms + delay 50 ms")
    axes[0].set_title("A quarter turn at once: held, then late"); axes[0].set_xlabel("time from the turn [ms]")
    axes[0].set_ylabel("field x [nT]"); axes[0].legend(fontsize=8)
    r = run(args, "mag", "resolution", "resolution-range.json", "turn", 6.0, rate=0.002)
    axes[1].plot(r["time"], r["true_my"], color=TRUTH, label="truth")
    axes[1].step(r["time"], r["my"], where="post", color=MEAS, label="resolution 15 nT")
    clipped = r["flags"] > 0
    axes[1].set_title(f"Resolution: 15 nT steps on a slow turn\n(range 0.3 G: z clipped at -30 uT, "
                      f"{clipped.mean() * 100:.0f} % of samples flagged)")
    axes[1].set_xlabel("time [s]"); axes[1].set_ylabel("field y [nT]"); axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    save(fig, args, "mag-3-delay-resolution")


# ---------------------------------------------------------------------------------------------------- barometer
def baro(args):
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    for z in (0.0, 0.5, 1.0):
        d = run(args, "baro", f"mount-{z}", "truth.json", "rest", 0.1, mount=f"0,0,{z},0,0,0")
        axes[0].bar(f"{z:.1f} m", d["pressure"][-1] - 100845.4, color=MEAS if z else TRUTH)
    axes[0].set_title("Mounting height: 0.5 m higher reads about 6 Pa less")
    axes[0].set_ylabel("pressure - at body origin [Pa]"); axes[0].set_xlabel("sensor above the body origin")
    units = []
    for seed in range(1, 1001):
        d = run(args, "baro", f"offset-{seed}", "offset.json", "rest", 0.002, seed=seed)
        units.append(d["pressure_altitude"][-1] - d["true_altitude"][-1])
    units = np.array(units)
    axes[1].hist(units, bins=40, density=True, color=MEAS, alpha=0.6, label=f"1000 units, sd {units.std():.2f}")
    gaussian(axes[1], 4.0)
    axes[1].set_title("Zero offset: every unit reads its own height"); axes[1].set_xlabel("altitude - truth [m]")
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    save(fig, args, "baro-1-mount-offset")

    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    d = sampled(run(args, "baro", "noise", "noise.json", "rest", 60.0))
    err = d["pressure_altitude"] - d["true_altitude"]
    axes[0].hist(err, bins=50, density=True, color=MEAS, alpha=0.6, label=f"{len(err)} samples, sd {err.std():.3f}")
    gaussian(axes[0], 0.1)
    axes[0].set_title("Noise at rest (period 20 ms)"); axes[0].set_xlabel("altitude - truth [m]")
    axes[0].legend(fontsize=8)
    dd = sampled(run(args, "baro", "noise-drift", "noise-drift.json", "rest", 1800.0, every=20))
    dt = 0.02
    taus = np.logspace(np.log10(dt), np.log10(400), 40)
    axes[1].loglog(taus, allan(dd["pressure_altitude"] - dd["true_altitude"], dt, taus), color=MEAS,
                   label="noise 0.1 m + drift (0.3 m, tau 30 s)")
    axes[1].loglog(taus, 0.1 / np.sqrt(taus / dt), color="k", lw=0.8, ls="--", label="white noise alone")
    axes[1].set_title("Allan deviation: averaging helps until the drift")
    axes[1].set_xlabel("averaging time [s]"); axes[1].set_ylabel("Allan deviation [m]"); axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3, which="both")
    save(fig, args, "baro-2-noise-drift")

    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    p = run(args, "baro", "period", "period.json", "hstep", 1.2, t0=1.005)
    q = run(args, "baro", "period-delay", "period-delay.json", "hstep", 1.2, t0=1.005)
    w = (p["time"] >= 0.95) & (p["time"] <= 1.15)
    t = (p["time"][w] - 1.005) * 1000
    axes[0].step(t, p["true_altitude"][w], where="post", color=TRUTH, label="truth")
    axes[0].step(t, p["pressure_altitude"][w], where="post", color=MEAS, label="period 20 ms")
    axes[0].step(t, q["pressure_altitude"][w], where="post", color="#0b2a5b", label="period 20 ms + delay 60 ms")
    axes[0].set_title("A 10 m step: held, then late"); axes[0].set_xlabel("time from the step [ms]")
    axes[0].set_ylabel("altitude [m]"); axes[0].legend(fontsize=8)
    r = run(args, "baro", "resolution", "resolution-range.json", "climb", 4.0, rate=0.05, t0=0.0)
    axes[1].plot(r["time"], r["true_altitude"] - 40.0, color=TRUTH, label="truth (climbing 5 cm/s)")
    axes[1].step(r["time"], r["pressure_altitude"] - 40.0, where="post", color=MEAS, label="resolution 0.5 Pa")
    s = run(args, "baro", "range", "resolution-range.json", "climb", 6.0, rate=2000.0, t0=0.0, every=10)
    sat = s["flags"] > 0
    top = s["pressure_altitude"][sat][0] if sat.any() else float("nan")
    axes[1].set_title(f"Resolution: about 4 cm steps\n(range 300 hPa: clipped above {top:.0f} m)")
    axes[1].set_xlabel("time [s]"); axes[1].set_ylabel("altitude - start [m]"); axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    save(fig, args, "baro-3-delay-resolution")


# ---------------------------------------------------------------------------------------------------- GPS
def gps(args):
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    turn = 2 * np.pi / 8.0
    for r, color in ((0.0, TRUTH), (0.5, MEAS)):
        d = sampled(run(args, "gps", f"mount-{r}", "truth.json", "turn", 8.0, rate=turn, mount=f"{r},0,0,0,0,0"))
        n, e = north_east(d)
        axes[0].plot(e, n, "o-", ms=3, color=color, label=f"antenna {r:.1f} m forward")
    axes[0].set_aspect("equal"); axes[0].set_title("Mounting: turning in place, the antenna draws a circle")
    axes[0].set_xlabel("east [m]"); axes[0].set_ylabel("north [m]"); axes[0].legend(fontsize=8)
    d = sampled(run(args, "gps", "noise", "noise.json", "rest", 300.0))
    n, e = north_east(d)
    axes[1].plot(e, n, ".", ms=2, color=MEAS, alpha=0.5, label=f"{len(n)} fixes, sd N {n.std():.2f} E {e.std():.2f}")
    angle = np.linspace(0, 2 * np.pi, 100)
    axes[1].plot(0.3 * np.cos(angle), 0.3 * np.sin(angle), color="k", lw=1, label="configured sigma 0.3 m")
    axes[1].set_aspect("equal"); axes[1].set_title("Position noise at rest (10 Hz)")
    axes[1].set_xlabel("east [m]"); axes[1].set_ylabel("north [m]"); axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    save(fig, args, "gps-1-mount-noise")

    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    d = sampled(run(args, "gps", "drift", "drift.json", "rest", 1800.0, every=100))
    n, e = north_east(d)
    sc = axes[0].scatter(e, n, c=d["time"] / 60.0, s=3, cmap="viridis")
    fig.colorbar(sc, ax=axes[0], label="time [min]")
    axes[0].set_aspect("equal"); axes[0].set_title("Drift (sigma 0.5 m, tau 60 s): the fix wanders")
    axes[0].set_xlabel("east [m]"); axes[0].set_ylabel("north [m]")
    dd = sampled(run(args, "gps", "noise-drift", "noise-drift.json", "rest", 1800.0, every=100))
    nn, _ = north_east(dd)
    dt = 0.1
    taus = np.logspace(np.log10(dt), np.log10(400), 40)
    axes[1].loglog(taus, allan(nn, dt, taus), color=MEAS, label="noise 0.3 m + drift 0.5 m")
    axes[1].loglog(taus, 0.3 / np.sqrt(taus / dt), color="k", lw=0.8, ls="--", label="white noise alone")
    axes[1].set_title("Allan deviation of north"); axes[1].set_xlabel("averaging time [s]")
    axes[1].set_ylabel("Allan deviation [m]"); axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3, which="both")
    save(fig, args, "gps-2-drift")

    fig, ax = plt.subplots(1, 1, figsize=(6, 3.8))
    p = run(args, "gps", "period", "period.json", "move", 1.6, t0=1.05)
    q = run(args, "gps", "period-delay", "period-delay.json", "move", 1.6, t0=1.05)
    w = (p["time"] >= 0.95) & (p["time"] <= 1.6)
    np_, _ = north_east(p)
    nq, _ = north_east(q)
    ax.plot(p["time"][w], p["true_n"][w], color=TRUTH, label="truth: 5 m/s north from 1.05 s")
    ax.step(p["time"][w], np_[w], where="post", color=MEAS, label="period 100 ms")
    ax.step(q["time"][w], nq[w], where="post", color="#0b2a5b", label="period 100 ms + delay 100 ms")
    ax.set_title("Moving off: 10 fixes a second, each one late")
    ax.set_xlabel("time [s]"); ax.set_ylabel("north [m]"); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    save(fig, args, "gps-3-delay")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tool", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--implementation", default="hakoniwa")
    parser.add_argument("--only", nargs="*", choices=["mag", "baro", "gps"])
    args = parser.parse_args()
    for name in args.only or ["mag", "baro", "gps"]:
        {"mag": mag, "baro": baro, "gps": gps}[name](args)


if __name__ == "__main__":
    main()
