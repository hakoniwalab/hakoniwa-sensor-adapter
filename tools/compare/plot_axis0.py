"""Plots for axis 0 (no-noise comparison): barometer and GPS differences, sampling times.

Usage: python compare/plot_axis0.py RESULT_DIR
"""
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

out = Path(sys.argv[1])


def read(name):
    with open(out / name) as f:
        return list(csv.DictReader(f))


fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))

rows = read("baro.csv")
alt = [float(r["altitude_m"]) for r in rows]
ax[0].plot(alt, [float(r["hk_pa"]) - float(r["us1976_pa"]) for r in rows], label="Hakoniwa - US1976")
ax[0].plot(alt, [float(r["gz_pa"]) - float(r["us1976_pa"]) for r in rows], label="Gazebo - US1976")
ax[0].plot(alt, [float(r["hk_pa"]) - float(r["gz_pa"]) for r in rows], "k--", label="Hakoniwa - Gazebo")
if rows and "airsim_pa" in rows[0] and any(float(r["airsim_pa"]) != 0.0 for r in rows):
    ax[0].plot(alt, [float(r["airsim_pa"]) - float(r["us1976_pa"]) for r in rows], label="Project AirSim - US1976")
ax[0].set_xlabel("altitude above sea level [m]")
ax[0].set_ylabel("pressure difference [Pa]")
ax[0].set_title("Barometer, no noise")
ax[0].grid(alpha=0.3)
ax[0].legend(fontsize=8)

rows = read("gps.csv")
dist = [float(r["distance_m"]) / 1000 for r in rows]
ax[1].plot(dist, [float(r["north_err_m"]) for r in rows], label="north")
ax[1].plot(dist, [float(r["east_err_m"]) for r in rows], label="east")
ax[1].plot(dist, [float(r["up_err_m"]) for r in rows], label="up")
ax[1].set_xlabel("distance from the origin [km]")
ax[1].set_ylabel("Hakoniwa - Gazebo [m]")
ax[1].set_title("GPS position, no noise")
ax[1].grid(alpha=0.3)
ax[1].legend(fontsize=8)

rows = read("timing.csv")
t = [int(r["t_ms"]) for r in rows]
ax[2].stem(t, [int(r["hk_measured"]) * 1.0 for r in rows], linefmt="C0-", markerfmt="C0o", basefmt=" ",
           label="Hakoniwa")
ax[2].stem(t, [int(r["gz_measured"]) * 0.6 for r in rows], linefmt="C1-", markerfmt="C1s", basefmt=" ",
           label="Gazebo")
ax[2].set_xlabel("simulation time [ms] (3 ms physics step)")
ax[2].set_yticks([])
ax[2].set_title("250 Hz sensor on a 3 ms step: when it measures")
ax[2].legend(fontsize=8)

fig.tight_layout()
fig.savefig(out / "axis0-no-noise.png", dpi=120)
print("wrote", out / "axis0-no-noise.png")
