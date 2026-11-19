"""Measurement cards of the per-unit offset, iron and noise sections (Japanese).

    python3 cards_offset_noise.py --tool <build>/error-characteristics --out <dir> [--only offset_imu ...]

See cards_common.py for what a card is.
"""
import numpy as np
from matplotlib.patches import Ellipse, Rectangle

from cards_common import (C0, C1, C2, C3, DEG, INK, MUTED, airframe, arrow, card, chip, hist_against_sigma, main,
                          north_east, notes, plt, result_title, run, sampled, save, sketch_axes, spin_arrow)

G0 = 9.80665
SEEDS = range(1, 1001)


def units_sketch(ax, color, label, rows=3, cols=6):
    """Many identical sensors side by side."""
    sketch_axes(ax, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    rng = np.random.default_rng(1)
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
    fig, left, right = card("零点のずれ：時間がたっても変わらない")
    sketch_axes(left, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    colors = (C1, C2, C3)
    seeds = (1, 2, 5)  # three units whose offsets lie apart, so the lines can be told apart
    for k, (seed, color) in enumerate(zip(seeds, colors)):
        x = -0.28 + 0.28 * k
        airframe(left, size=0.1, alpha=0.6, centre=(x, 0.2))
        chip(left, (x, 0.2), color, size=0.04)
        left.text(x, 0.06, f"乱数の種 {seed}", fontsize=9, color=color, ha="center", va="top")
    left.text(0, 0.45, "静止した IMU を 10 秒測る", fontsize=10, color=INK, ha="center")
    notes(left, ["設定：ジャイロの零点のずれ σ 1°/s（\"randomStdDegPerSec\"）だけ",
                 "種 1 は 2 回起動して、2 回とも測る",
                 "真値はずっと 0。ノイズは入れない"], y=-0.06)
    right.axhline(0, color=MUTED, lw=2.5, label="真値（0）")
    for seed, color in zip(seeds, colors):
        d = sampled(run(args, "imu", f"offset-time-{seed}", "offset.json", "rest", 10.0, seed=seed), "gyro_updated")
        g = d["gx"] / DEG
        right.plot(d["time"], g, color=color, lw=2.2, label=f"種 {seed}")
        right.text(10.2, g[-1], f"{g[-1]:+.3f}°/s（10 秒で変化 {g.max() - g.min():.3f}）", fontsize=8.5, color=color,
                   va="center")
    again = sampled(run(args, "imu", "offset-time-1-again", "offset.json", "rest", 10.0, seed=1), "gyro_updated")
    right.plot(again["time"], again["gx"] / DEG, color=C0, lw=1.2, ls=(0, (2, 3)), label="種 1 の 2 回目の起動（重なる）")
    right.set_xlim(0, 15.5)
    right.set_ylim(-1.8, 1.9)
    right.set_xlabel("時間 [s]")
    right.set_ylabel("ジャイロ x の値 [°/s]（真値は 0）")
    result_title(right, "結果：個体ごとに違う高さの、まっすぐな線")
    right.legend(loc="upper left", fontsize=8.3, ncol=2)
    right.grid(alpha=0.3)
    save(fig, args, "card-offset-time")


def offset_imu(args):
    gyro, accel = [], []
    for seed in SEEDS:
        d = run(args, "imu", f"offset-{seed}", "offset.json", "rest", 0.002, seed=seed)
        gyro.append(d["gx"][-1] / DEG)
        accel.append((d["az"][-1] - G0) / G0 * 1000.0)
    fig, left, r1, r2 = card("零点のずれ：1000 個の IMU を、静止させて読む", right=2)
    units_sketch(left, C1, "同じ設定の IMU を 1000 個（乱数の種 1〜1000）")
    notes(left, ["どれも水平に静止。真値はジャイロ 0、加速度は重力 1 g だけ",
                 "設定：ジャイロ σ 1°/s、加速度 σ 20 mg（個体差）",
                 "ほかの誤差は入れない。電源を入れた直後の値を読む"], y=-0.13)
    hist_against_sigma(r1, gyro, 1.0, bins=40, label="1000 個", count_label="その値だった個数")
    r1.set_xlabel("ジャイロ x の値 [°/s]（真値は 0）")
    result_title(r1, "ジャイロ：個体ごとのずれ")
    hist_against_sigma(r2, accel, 20.0, bins=40, label="1000 個", count_label="その値だった個数")
    r2.set_xlabel("加速度 z − 1 g [mg]（真値は 0）")
    result_title(r2, "加速度：個体ごとのずれ")
    for ax in (r1, r2):
        ax.grid(alpha=0.3)
    save(fig, args, "card-offset-imu")


def offset_baro(args):
    units = []
    for seed in SEEDS:
        d = run(args, "baro", f"offset-{seed}", "offset.json", "rest", 0.002, seed=seed)
        units.append(d["pressure_altitude"][-1] - d["true_altitude"][-1])
    fig, left, right = card("零点のずれ：1000 個の気圧センサーを、同じ高さで読む")
    units_sketch(left, C3, "同じ設定の気圧センサーを 1000 個")
    notes(left, ["どれも同じ高さ（40 m）に静止", "設定：σ 4 m（高度にした個体差）",
                 "測った気圧を高度に直し、真の高度との差を見る"], y=-0.13)
    hist_against_sigma(right, units, 4.0, bins=40, label="1000 個", count_label="その値だった個数")
    right.set_xlabel("気圧から求めた高度 − 真の高度 [m]")
    result_title(right, "結果：個体ごとに、違う高さを読む")
    right.grid(alpha=0.3)
    save(fig, args, "card-offset-baro")


def iron_mag(args):
    turn = 2 * np.pi / 4.0
    fig, left, right = card("地磁気の鉄の影響：機体の磁石と鉄で、円がずれて潰れる")
    sketch_axes(left, title="測り方（上から見た図、前が上）")
    airframe(left, alpha=0.7)
    spin_arrow(left, 0.4, "その場で 4 秒に一回転")
    chip(left, (0, 0), C2, size=0.05)
    left.text(0.04, 0.03, "地磁気センサー", fontsize=9, color=C2)
    left.add_patch(Rectangle((-0.17, -0.12), 0.09, 0.04, fc="#555", ec="none", zorder=5))
    left.text(-0.13, -0.15, "磁石・モーター\n（ハードアイアン）", fontsize=8, color="#555", ha="center", va="top")
    left.add_patch(Rectangle((0.08, -0.2), 0.12, 0.03, fc="#9aa3ad", ec="none", zorder=5))
    left.text(0.14, -0.23, "鉄の部品\n（ソフトアイアン）", fontsize=8, color="#555", ha="center", va="top")
    notes(left, ["設定（FRD 軸）：ハードアイアン (3000, −1500, 800) nT、",
                 "ソフトアイアン 対角 (1.02, 0.98, 1.01)、非対角 (0.01, −0.02, 0.005)"], y=-0.33)
    c = run(args, "mag", "iron", "iron.json", "turn", 4.0, rate=turn)
    right.plot(c["true_mx"] / 1000, c["true_my"] / 1000, color=MUTED, lw=2.5, label="真値：0 を中心とする円")
    right.plot(c["mx"] / 1000, c["my"] / 1000, color=C2, lw=2.2, label="測った値：ずれて、少し潰れる")
    centre = ((c["mx"].max() + c["mx"].min()) / 2000, (c["my"].max() + c["my"].min()) / 2000)
    right.plot(0, 0, "+", color=MUTED, ms=12, mew=2)
    right.plot(*centre, "+", color=C2, ms=12, mew=2)
    arrow(right, (0, 0), centre, color=C2, lw=1.2, scale=10)
    right.text(centre[0] + 2.5, centre[1] + 3, f"中心 ({centre[0]:.1f}, {centre[1]:.1f}) µT", fontsize=9.5, color=C2)
    right.set_aspect("equal")
    right.set_xlabel("センサー x（前）の磁場 [µT]")
    right.set_ylabel("センサー y（左）の磁場 [µT]")
    result_title(right, "結果：一回転したときの、水平の磁場の軌跡")
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
    ax.text(0, 0.45, "静止したまま測り続ける", fontsize=10, color=INK, ha="center")
    notes(ax, lines, y=-0.15)


def noise_gyro(args):
    d = sampled(run(args, "imu", "noise", "noise.json", "rest", 20.0), "gyro_updated")
    fig, left, right = card("ノイズ：静止したジャイロを 20 秒測る")
    rest_sketch(left, [("IMU", C1)], ["周期 4 ms で 5000 回測る（真値はずっと 0）", "設定：ノイズ σ 0.05°/s（\"noise\"）",
                                      "ほかの誤差は入れない"])
    hist_against_sigma(right, d["gx"] / DEG, 0.05, bins=60, label="5000 回")
    right.set_xlabel("ジャイロ x の値 [°/s]（真値は 0）")
    result_title(right, "結果：測った値の散らばり方")
    right.grid(alpha=0.3)
    save(fig, args, "card-noise-gyro")


def noise_density(args):
    density = 0.00024
    fig, left, right = card("ノイズ密度：同じジャイロを、周期 2 / 4 / 8 ms で測る")
    sketch_axes(left, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    for k, (ms, color) in enumerate(((2, C2), (4, C1), (8, C3))):
        y = 0.3 - 0.17 * k
        chip(left, (-0.38, y), color, size=0.05, label=f"周期 {ms} ms")
        ticks = np.arange(-0.12, 0.42, ms * 0.012)
        left.plot(ticks, np.full(ticks.size, y), "|", color=color, ms=9)
    left.text(0.15, 0.42, "測る瞬間（縮尺は目安）", fontsize=8.5, color=MUTED, ha="center")
    notes(left, ["設定：ノイズ密度 0.00024 rad/s/√Hz（\"noiseDensity\"）",
                 "σ は書かない。周期から σ ＝ ノイズ密度 ÷ √周期 で決まる", "どれも静止して 20 秒"], y=-0.15)
    periods, measured = [], []
    for ms in (2, 4, 8):
        dd = sampled(run(args, "imu", f"noise-density-{ms}ms", f"noise-density-{ms}ms.json", "rest", 20.0),
                     "gyro_updated")
        periods.append(ms)
        measured.append(dd["gx"].std() / DEG)
    p = np.linspace(1.5, 9, 100)
    right.plot(p, density / np.sqrt(p * 1e-3) / DEG, color=MUTED, lw=1.5, ls="--", label="計算：ノイズ密度 ÷ √周期")
    for ms, s, color in zip(periods, measured, (C2, C1, C3)):
        right.plot(ms, s, "o", color=color, ms=10)
        right.text(ms + 0.25, s, f"{s:.3f}°/s（計算 {density / np.sqrt(ms * 1e-3) / DEG:.3f}）", fontsize=9,
                   color=color, va="center")
    right.set_xlim(1, 12)
    right.set_xlabel("周期 [ms]")
    right.set_ylabel("測った値の標準偏差 σ [°/s]")
    result_title(right, "結果：速く測るほど、1回のばらつきが大きい")
    right.legend(loc="upper right", fontsize=8.5)
    right.grid(alpha=0.3)
    save(fig, args, "card-noise-density")


def noise_mag_baro(args):
    m = sampled(run(args, "mag", "noise", "noise.json", "rest", 60.0))
    b = sampled(run(args, "baro", "noise", "noise.json", "rest", 60.0))
    fig, left, r1, r2 = card("ノイズ：静止した地磁気センサーと気圧センサーを 60 秒測る", right=2)
    rest_sketch(left, [("地磁気", C2), ("気圧", C3)], ["どちらも周期 20 ms で 3000 回", "設定：地磁気 σ 30 nT、気圧 σ 0.1 m",
                                                    "横軸は「測った値 − 真値」"])
    hist_against_sigma(r1, m["mx"] - m["true_mx"], 30.0, label="3000 回")
    r1.set_xlabel("地磁気 x − 真値 [nT]")
    result_title(r1, "地磁気")
    hist_against_sigma(r2, b["pressure_altitude"] - b["true_altitude"], 0.1, label="3000 回")
    r2.set_xlabel("気圧から求めた高度 − 真値 [m]")
    result_title(r2, "気圧")
    for ax in (r1, r2):
        ax.grid(alpha=0.3)
    save(fig, args, "card-noise-mag-baro")


def noise_gps(args):
    d = sampled(run(args, "gps", "noise", "noise.json", "rest", 300.0))
    n, e = north_east(d)
    fig, left, right = card("ノイズ：静止した GPS を 5 分測る")
    rest_sketch(left, [("GPS アンテナ", C2)], ["毎秒 10 回、3000 回測位（真の位置はずっと同じ）",
                                              "設定：位置 σ 水平 0.3 m・鉛直 0.5 m、",
                                              "　　　速度 σ 水平 0.1 m/s・鉛直 0.2 m/s"])
    right.plot(e, n, ".", ms=2.5, color=C2, alpha=0.45, label=f"測位 3000 回（標準偏差 北 {n.std():.2f} m、東 {e.std():.2f} m）")
    angle = np.linspace(0, 2 * np.pi, 100)
    for k, style in ((1, "-"), (2, "--")):
        right.plot(0.3 * k * np.cos(angle), 0.3 * k * np.sin(angle), color=C0, lw=1.2, ls=style,
                   label=f"設定 σ の {k} 倍（{0.3 * k:.1f} m）")
    right.plot(0, 0, "+", color=C0, ms=12, mew=2)
    right.set_aspect("equal")
    right.set_xlim(-1.3, 1.3)
    right.set_ylim(-1.2, 1.4)
    right.set_xlabel("東 [m]（真の位置から）")
    right.set_ylabel("北 [m]（真の位置から）")
    result_title(right, "結果：測った位置の散らばり（地図の向き）")
    right.legend(loc="upper left", fontsize=8)
    right.grid(alpha=0.3)
    save(fig, args, "card-noise-gps")


if __name__ == "__main__":
    main(__doc__, [offset_time, offset_imu, offset_baro, iron_mag, noise_gyro, noise_density, noise_mag_baro, noise_gps])
