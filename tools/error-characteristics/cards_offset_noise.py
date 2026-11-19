"""Measurement cards of the per-unit offset, iron and noise sections.

    python3 cards_offset_noise.py --tool <build>/error-characteristics --out <dir> [--lang ja|en] [--only offset_imu ...]

See cards_common.py for what a card is.
"""
import subprocess
from pathlib import Path

import numpy as np
from matplotlib.patches import Rectangle

from cards_common import (C0, C1, C2, C3, DEG, INK, MUTED, L, airframe, arrow, card, chip, hist_against_sigma, main,
                          north_east, notes, result_title, run, sampled, save, sketch_axes, spin_arrow)
from ec_common import HERE

G0 = 9.80665
SEEDS = range(1, 1001)
TIME = ("時間 [s]", "time [s]")


def units_sketch(ax, color, label, rows=3, cols=6):
    """Many identical sensors side by side."""
    sketch_axes(ax, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    for i in range(rows):
        for j in range(cols):
            x, y = -0.35 + j * 0.13, 0.3 - i * 0.12
            chip(ax, (x, y), color, size=0.06)
            ax.text(x, y - 0.05, f"#{i * cols + j + 1}", fontsize=6, color=MUTED, ha="center", va="top")
    ax.text(0.33, -0.06, "…", fontsize=14, color=MUTED, va="center")
    ax.text(0, 0.42, label, fontsize=10, color=INK, ha="center")


# ------------------------------------------------------------------------------------------------ 2. offset and iron
def offset_time(args):
    """The offset is one value per start-up: constant in time, the same again with the same seed."""
    fig, left, right = card(L("零点のずれ：時間がたっても変わらない", "Zero offset: it does not change with time"))
    sketch_axes(left, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    colors = (C1, C2, C3)
    seeds = (1, 2, 5)  # three units whose offsets lie apart, so the lines can be told apart
    for k, (seed, color) in enumerate(zip(seeds, colors)):
        x = -0.28 + 0.28 * k
        airframe(left, size=0.1, alpha=0.6, centre=(x, 0.2))
        chip(left, (x, 0.2), color, size=0.04)
        left.text(x, 0.06, L(f"乱数の種 {seed}", f"seed {seed}"), fontsize=9, color=color, ha="center", va="top")
    left.text(0, 0.45, L("静止した IMU を 10 秒測る", "an IMU at rest, measured for 10 s"), fontsize=10, color=INK, ha="center")
    notes(left, L(["設定：ジャイロの零点のずれ σ 1°/s（\"randomStdDegPerSec\"）だけ",
                   "種 1 は 2 回起動して、2 回とも測る",
                   "真値はずっと 0。ノイズは入れない"],
                  ["Configured: only the gyro zero offset, σ 1°/s (\"randomStdDegPerSec\")",
                   "seed 1 started twice and measured both times",
                   "The truth stays 0. No noise"]), y=-0.06)
    right.axhline(0, color=MUTED, lw=2.5, label=L("真値（0）", "truth (0)"))
    for seed, color in zip(seeds, colors):
        d = sampled(run(args, "imu", f"offset-time-{seed}", "offset.json", "rest", 10.0, seed=seed), "gyro_updated")
        g = d["gx"] / DEG
        right.plot(d["time"], g, color=color, lw=2.2, label=L(f"種 {seed}", f"seed {seed}"))
        right.text(10.2, g[-1], L(f"{g[-1]:+.3f}°/s（10 秒で変化 {g.max() - g.min():.3f}）",
                                  f"{g[-1]:+.3f}°/s (change in 10 s: {g.max() - g.min():.3f})"),
                   fontsize=8.5, color=color, va="center")
    again = sampled(run(args, "imu", "offset-time-1-again", "offset.json", "rest", 10.0, seed=1), "gyro_updated")
    right.plot(again["time"], again["gx"] / DEG, color=C0, lw=1.2, ls=(0, (2, 3)),
               label=L("種 1 の 2 回目の起動（重なる）", "seed 1, second start (on top)"))
    right.set_xlim(0, L(15.5, 16.5))
    right.set_ylim(-1.8, 1.9)
    right.set_xlabel(L(*TIME))
    right.set_ylabel(L("ジャイロ x の値 [°/s]（真値は 0）", "gyro x [°/s] (truth 0)"))
    result_title(right, L("結果：個体ごとに違う高さの、まっすぐな線", "Result: flat lines, each unit at its own height"))
    right.legend(loc="upper left", fontsize=8.3, ncol=2)
    right.grid(alpha=0.3)
    save(fig, args, "card-offset-time")


def offset_imu(args):
    gyro, accel = [], []
    for seed in SEEDS:
        d = run(args, "imu", f"offset-{seed}", "offset.json", "rest", 0.002, seed=seed)
        gyro.append(d["gx"][-1] / DEG)
        accel.append((d["az"][-1] - G0) / G0 * 1000.0)
    fig, left, r1, r2 = card(L("零点のずれ：1000 個の IMU を、静止させて読む", "Zero offset: 1000 IMUs read at rest"), right=2)
    units_sketch(left, C1, L("同じ設定の IMU を 1000 個（乱数の種 1〜1000）", "1000 IMUs, one configuration (seeds 1–1000)"))
    notes(left, L(["どれも水平に静止。真値はジャイロ 0、加速度は重力 1 g だけ",
                   "設定：ジャイロ σ 1°/s、加速度 σ 20 mg（個体差）",
                   "ほかの誤差は入れない。電源を入れた直後の値を読む"],
                  ["All level and at rest: truth gyro 0, accelerometer 1 g of gravity",
                   "Configured: gyro σ 1°/s, accelerometer σ 20 mg (unit to unit)",
                   "No other error. Read right after power-on"]), y=-0.13)
    units, count = L("1000 個", "1000 units"), L("その値だった個数", "number of units")
    hist_against_sigma(r1, gyro, 1.0, bins=40, label=units, count_label=count)
    r1.set_xlabel(L("ジャイロ x の値 [°/s]（真値は 0）", "gyro x [°/s] (truth 0)"))
    result_title(r1, L("ジャイロ：個体ごとのずれ", "Gyro: one offset per unit"))
    hist_against_sigma(r2, accel, 20.0, bins=40, label=units, count_label=count)
    r2.set_xlabel(L("加速度 z − 1 g [mg]（真値は 0）", "accelerometer z − 1 g [mg] (truth 0)"))
    result_title(r2, L("加速度：個体ごとのずれ", "Accelerometer: one offset per unit"))
    for ax in (r1, r2):
        ax.grid(alpha=0.3)
    save(fig, args, "card-offset-imu")


def offset_baro(args):
    values = []
    for seed in SEEDS:
        d = run(args, "baro", f"offset-{seed}", "offset.json", "rest", 0.002, seed=seed)
        values.append(d["pressure_altitude"][-1] - d["true_altitude"][-1])
    fig, left, right = card(L("零点のずれ：1000 個の気圧センサーを、同じ高さで読む", "Zero offset: 1000 barometers read at one height"))
    units_sketch(left, C3, L("同じ設定の気圧センサーを 1000 個", "1000 barometers, one configuration"))
    notes(left, L(["どれも同じ高さ（40 m）に静止", "設定：σ 4 m（高度にした個体差）",
                   "測った気圧を高度に直し、真の高度との差を見る"],
                  ["All at rest at one height (40 m)", "Configured: σ 4 m (unit to unit, as altitude)",
                   "The pressure is turned into altitude and compared with the truth"]), y=-0.13)
    hist_against_sigma(right, values, 4.0, bins=40, label=L("1000 個", "1000 units"),
                       count_label=L("その値だった個数", "number of units"))
    right.set_xlabel(L("気圧から求めた高度 − 真の高度 [m]", "altitude from pressure − true altitude [m]"))
    result_title(right, L("結果：個体ごとに、違う高さを読む", "Result: each unit reads its own height"))
    right.grid(alpha=0.3)
    save(fig, args, "card-offset-baro")


def iron_mag(args):
    turn = 2 * np.pi / 4.0
    fig, left, right = card(L("地磁気の鉄の影響：機体の磁石と鉄で、円がずれて潰れる",
                              "Magnetometer iron: the vehicle's magnets and iron shift and squash the circle"))
    sketch_axes(left, title=L("測り方（上から見た図、前が上）", "Setup (seen from above, forward is up)"))
    airframe(left, alpha=0.7)
    spin_arrow(left, 0.4, L("その場で 4 秒に一回転", "turning in place, one turn in 4 s"))
    chip(left, (0, 0), C2, size=0.05)
    left.text(0.04, 0.03, L("地磁気センサー", "magnetometer"), fontsize=9, color=C2)
    left.add_patch(Rectangle((-0.17, -0.12), 0.09, 0.04, fc="#555", ec="none", zorder=5))
    left.text(-0.13, -0.15, L("磁石・モーター\n（ハードアイアン）", "magnets, motors\n(hard iron)"), fontsize=8, color="#555",
              ha="center", va="top")
    left.add_patch(Rectangle((0.08, -0.2), 0.12, 0.03, fc="#9aa3ad", ec="none", zorder=5))
    left.text(0.14, -0.23, L("鉄の部品\n（ソフトアイアン）", "iron parts\n(soft iron)"), fontsize=8, color="#555",
              ha="center", va="top")
    notes(left, L(["設定（FRD 軸）：ハードアイアン (3000, −1500, 800) nT、",
                   "ソフトアイアン 対角 (1.02, 0.98, 1.01)、非対角 (0.01, −0.02, 0.005)"],
                  ["Configured (FRD axes): hard iron (3000, −1500, 800) nT,",
                   "soft iron diagonal (1.02, 0.98, 1.01), off-diagonal (0.01, −0.02, 0.005)"]), y=-0.33)
    c = run(args, "mag", "iron", "iron.json", "turn", 4.0, rate=turn)
    right.plot(c["true_mx"] / 1000, c["true_my"] / 1000, color=MUTED, lw=2.5,
               label=L("真値：0 を中心とする円", "truth: a circle about 0"))
    right.plot(c["mx"] / 1000, c["my"] / 1000, color=C2, lw=2.2,
               label=L("測った値：ずれて、少し潰れる", "measured: shifted and slightly squashed"))
    centre = ((c["mx"].max() + c["mx"].min()) / 2000, (c["my"].max() + c["my"].min()) / 2000)
    right.plot(0, 0, "+", color=MUTED, ms=12, mew=2)
    right.plot(*centre, "+", color=C2, ms=12, mew=2)
    arrow(right, (0, 0), centre, color=C2, lw=1.2, scale=10)
    right.text(centre[0] + 2.5, centre[1] + 3, L(f"中心 ({centre[0]:.1f}, {centre[1]:.1f}) µT",
                                                 f"center ({centre[0]:.1f}, {centre[1]:.1f}) µT"), fontsize=9.5, color=C2)
    right.set_aspect("equal")
    right.set_xlabel(L("センサー x（前）の磁場 [µT]", "field on sensor x (forward) [µT]"))
    right.set_ylabel(L("センサー y（左）の磁場 [µT]", "field on sensor y (left) [µT]"))
    result_title(right, L("結果：一回転したときの、水平の磁場の軌跡", "Result: the horizontal field over one turn"))
    right.legend(loc="lower left", fontsize=8.3)
    right.set_xlim(-38, 42)
    right.set_ylim(-40, 38)
    right.grid(alpha=0.3)
    save(fig, args, "card-iron-mag")


# ------------------------------------------------------------------------------------------------ 3. noise
def rest_sketch(ax, sensors, lines):
    """Sensors standing still."""
    sketch_axes(ax, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    airframe(ax, alpha=0.6, centre=(0, 0.15))
    for k, (label, color) in enumerate(sensors):
        x = -0.06 + 0.12 * k - 0.06 * (len(sensors) - 1)
        chip(ax, (x + 0.06, 0.15), color, size=0.05)
        ax.text(x + 0.06, 0.15 - 0.07 - 0.06 * k, label, fontsize=9, color=color, ha="center", va="top")
    ax.text(0, 0.45, L("静止したまま測り続ける", "measured at rest"), fontsize=10, color=INK, ha="center")
    notes(ax, lines, y=-0.15)


def noise_gyro(args):
    d = sampled(run(args, "imu", "noise", "noise.json", "rest", 20.0), "gyro_updated")
    fig, left, right = card(L("ノイズ：静止したジャイロを 20 秒測る", "Noise: a gyro at rest for 20 s"))
    rest_sketch(left, [("IMU", C1)], L(["周期 4 ms で 5000 回測る（真値はずっと 0）", "設定：ノイズ σ 0.05°/s（\"noise\"）",
                                        "ほかの誤差は入れない"],
                                       ["5000 samples, one every 4 ms (truth 0 all along)",
                                        "Configured: noise σ 0.05°/s (\"noise\")", "No other error"]))
    hist_against_sigma(right, d["gx"] / DEG, 0.05, bins=60, label=L("5000 回", "5000 samples"))
    right.set_xlabel(L("ジャイロ x の値 [°/s]（真値は 0）", "gyro x [°/s] (truth 0)"))
    result_title(right, L("結果：測った値の散らばり方", "Result: how the samples spread"))
    right.grid(alpha=0.3)
    save(fig, args, "card-noise-gyro")


def noise_density(args):
    density = 0.00024
    fig, left, right = card(L("ノイズ密度：同じジャイロを、周期 2 / 4 / 8 ms で測る",
                              "Noise density: one gyro at update periods of 2, 4 and 8 ms"))
    sketch_axes(left, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    for k, (ms, color) in enumerate(((2, C2), (4, C1), (8, C3))):
        y = 0.3 - 0.17 * k
        chip(left, (-0.38, y), color, size=0.05, label=L(f"周期 {ms} ms", f"{ms} ms"))
        ticks = np.arange(-0.12, 0.42, ms * 0.012)
        left.plot(ticks, np.full(ticks.size, y), "|", color=color, ms=9)
    left.text(0.15, 0.42, L("測る瞬間（縮尺は目安）", "samples (roughly to scale)"), fontsize=8.5, color=MUTED, ha="center")
    notes(left, L(["設定：ノイズ密度 0.00024 rad/s/√Hz（\"noiseDensity\"）",
                   "σ は書かない。周期から σ ＝ ノイズ密度 ÷ √周期 で決まる", "どれも静止して 20 秒"],
                  ["Configured: noise density 0.00024 rad/s/√Hz (\"noiseDensity\")",
                   "No σ: the period sets it, σ = density ÷ √period", "Each at rest for 20 s"]), y=-0.15)
    periods, measured = [], []
    for ms in (2, 4, 8):
        dd = sampled(run(args, "imu", f"noise-density-{ms}ms", f"noise-density-{ms}ms.json", "rest", 20.0),
                     "gyro_updated")
        periods.append(ms)
        measured.append(dd["gx"].std() / DEG)
    p = np.linspace(1.5, 9, 100)
    right.plot(p, density / np.sqrt(p * 1e-3) / DEG, color=MUTED, lw=1.5, ls="--",
               label=L("計算：ノイズ密度 ÷ √周期", "computed: density ÷ √period"))
    for ms, s, color in zip(periods, measured, (C2, C1, C3)):
        right.plot(ms, s, "o", color=color, ms=10)
        right.text(ms + 0.25, s, L(f"{s:.3f}°/s（計算 {density / np.sqrt(ms * 1e-3) / DEG:.3f}）",
                                   f"{s:.3f}°/s (computed {density / np.sqrt(ms * 1e-3) / DEG:.3f})"),
                   fontsize=9, color=color, va="center")
    right.set_xlim(1, 12.5)
    right.set_xlabel(L("周期 [ms]", "update period [ms]"))
    right.set_ylabel(L("測った値の標準偏差 σ [°/s]", "standard deviation σ of the samples [°/s]"))
    result_title(right, L("結果：速く測るほど、1回のばらつきが大きい", "Result: the faster, the larger each sample's spread"))
    right.legend(loc="upper right", fontsize=8.5)
    right.grid(alpha=0.3)
    save(fig, args, "card-noise-density")


def integrated_angle(args, config, seeds, duration):
    """Runs a resting gyro once per seed and integrates gyro x every physics step (1 ms), as an attitude estimator
    would. Returns the time and, per time, the standard deviation of the angle over the seeds, in degrees."""
    out = Path(args.work) / "csv" / "angle.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    angles = []
    for seed in seeds:
        subprocess.run([args.tool, "--implementation", args.implementation, "--sensor", "imu", "--config",
                        str(HERE / "config" / "imu" / config), "--motion", "rest", "--duration", str(duration),
                        "--seed", str(seed), "--out", str(out)], check=True, capture_output=True)
        data = np.loadtxt(out, delimiter=",", skiprows=1, usecols=(0, 8))  # time, gx
        angles.append(np.cumsum(data[:, 1]) * 0.001)
    return data[:, 0], np.std(np.array(angles), axis=0) / DEG


def noise_density_angle(args):
    density = 0.00024
    sigma = density / np.sqrt(0.004)
    duration, seeds = 60.0, range(1, 101)
    fig, left, r1, r2 = card(L("ノイズ密度の意味：ジャイロを積分した角度のずれ",
                               "What noise density is for: the angle error of an integrated gyro"), right=2)
    sketch_axes(left, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    airframe(left, alpha=0.6, centre=(0, 0.2))
    chip(left, (0, 0.2), C1, size=0.05)
    left.text(0, 0.45, L("静止したジャイロの値を 60 秒足し合わせる（積分）", "a resting gyro, summed for 60 s (integrated)"),
              fontsize=10, color=INK, ha="center")
    left.text(0, 0.0, L("真の角度はずっと 0。ずれはノイズが積もった分", "The true angle stays 0; the error is noise piled up"),
              fontsize=9, color=INK, ha="center")
    notes(left, L(["周期 2 / 4 / 8 ms の 3 通り × 設定 2 通り。それぞれ乱数の種 100 個",
                   "縦軸：100 回の角度のずれの標準偏差",
                   f"σ 固定：\"noise\" {sigma:.5f} rad/s（4 ms で右と同じになる値）",
                   f"ノイズ密度：\"noiseDensity\" {density} rad/s/√Hz"],
                  ["Periods 2, 4 and 8 ms × two configurations, 100 seeds each",
                   "Vertical axis: standard deviation of the 100 angle errors",
                   f"fixed σ: \"noise\" {sigma:.5f} rad/s (equal to the right one at 4 ms)",
                   f"noise density: \"noiseDensity\" {density} rad/s/√Hz"]), y=-0.1)
    top = 0.0
    for ax, kind, title in ((r1, "sigma", L("σ 固定：周期で変わってしまう", "Fixed σ: changes with the period")),
                            (r2, "density", L("ノイズ密度：周期によらず同じ", "Noise density: the same at any period"))):
        for ms, color in ((2, C2), (4, C1), (8, C3)):
            t, s = integrated_angle(args, f"noise-{kind}-{ms}ms.json", seeds, duration)
            ax.plot(t, s, color=color, lw=2, label=L(f"周期 {ms} ms：60 秒で {s[-1]:.3f}°", f"{ms} ms: {s[-1]:.3f}° at 60 s"))
            expected = (sigma * np.sqrt(ms * 1e-3 * t) if kind == "sigma" else density * np.sqrt(t)) / DEG
            ax.plot(t, expected, color=color, lw=0.9, ls="--")
            top = max(top, s.max())
        ax.set_xlabel(L(*TIME))
        ax.set_ylabel(L("積分した角度のずれ（標準偏差）[°]", "angle error (standard deviation) [°]"))
        result_title(ax, title)
        ax.legend(loc="upper left", fontsize=8.3, title=L("実線：測定　破線：計算", "solid: measured, dashed: computed"),
                  title_fontsize=8)
        ax.grid(alpha=0.3)
    for ax in (r1, r2):
        ax.set_ylim(0, top * 1.35)
    save(fig, args, "card-noise-density-angle")


def noise_mag_baro(args):
    m = sampled(run(args, "mag", "noise", "noise.json", "rest", 60.0))
    b = sampled(run(args, "baro", "noise", "noise.json", "rest", 60.0))
    fig, left, r1, r2 = card(L("ノイズ：静止した地磁気センサーと気圧センサーを 60 秒測る",
                               "Noise: a magnetometer and a barometer at rest for 60 s"), right=2)
    rest_sketch(left, [(L("地磁気", "magnetometer"), C2), (L("気圧", "barometer"), C3)],
                L(["どちらも周期 20 ms で 3000 回", "設定：地磁気 σ 30 nT、気圧 σ 0.1 m", "横軸は「測った値 − 真値」"],
                  ["3000 samples each, one every 20 ms", "Configured: magnetometer σ 30 nT, barometer σ 0.1 m",
                   "Horizontal axis: measured − truth"]))
    samples = L("3000 回", "3000 samples")
    hist_against_sigma(r1, m["mx"] - m["true_mx"], 30.0, label=samples)
    r1.set_xlabel(L("地磁気 x − 真値 [nT]", "field x − truth [nT]"))
    result_title(r1, L("地磁気", "Magnetometer"))
    hist_against_sigma(r2, b["pressure_altitude"] - b["true_altitude"], 0.1, label=samples)
    r2.set_xlabel(L("気圧から求めた高度 − 真値 [m]", "altitude from pressure − truth [m]"))
    result_title(r2, L("気圧", "Barometer"))
    for ax in (r1, r2):
        ax.grid(alpha=0.3)
    save(fig, args, "card-noise-mag-baro")


def noise_gps(args):
    d = sampled(run(args, "gps", "noise", "noise.json", "rest", 300.0))
    n, e = north_east(d)
    fig, left, right = card(L("ノイズ：静止した GPS を 5 分測る", "Noise: a GPS at rest for 5 minutes"))
    rest_sketch(left, [(L("GPS アンテナ", "GPS antenna"), C2)],
                L(["毎秒 10 回、3000 回測位（真の位置はずっと同じ）", "設定：位置 σ 水平 0.3 m・鉛直 0.5 m、",
                   "　　　速度 σ 水平 0.1 m/s・鉛直 0.2 m/s"],
                  ["3000 fixes, 10 a second (the true position never moves)",
                   "Configured: position σ 0.3 m horizontal, 0.5 m vertical;",
                   "velocity σ 0.1 m/s horizontal, 0.2 m/s vertical"]))
    right.plot(e, n, ".", ms=2.5, color=C2, alpha=0.45,
               label=L(f"測位 3000 回（標準偏差 北 {n.std():.2f} m、東 {e.std():.2f} m）",
                       f"3000 fixes (standard deviation N {n.std():.2f} m, E {e.std():.2f} m)"))
    angle = np.linspace(0, 2 * np.pi, 100)
    for k, style in ((1, "-"), (2, "--")):
        right.plot(0.3 * k * np.cos(angle), 0.3 * k * np.sin(angle), color=C0, lw=1.2, ls=style,
                   label=L(f"設定 σ の {k} 倍（{0.3 * k:.1f} m）", f"{k} × the configured σ ({0.3 * k:.1f} m)"))
    right.plot(0, 0, "+", color=C0, ms=12, mew=2)
    right.set_aspect("equal")
    right.set_xlim(-1.3, 1.3)
    right.set_ylim(-1.2, 1.4)
    right.set_xlabel(L("東 [m]（真の位置から）", "east [m] (from the true position)"))
    right.set_ylabel(L("北 [m]（真の位置から）", "north [m] (from the true position)"))
    result_title(right, L("結果：測った位置の散らばり（地図の向き）", "Result: the spread of the fixes (map orientation)"))
    right.legend(loc="upper left", fontsize=8)
    right.grid(alpha=0.3)
    save(fig, args, "card-noise-gps")


if __name__ == "__main__":
    main(__doc__, [offset_time, offset_imu, offset_baro, iron_mag, noise_gyro, noise_density, noise_density_angle,
                   noise_mag_baro, noise_gps])
