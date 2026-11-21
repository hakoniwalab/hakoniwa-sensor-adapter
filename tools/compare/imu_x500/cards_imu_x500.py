#!/usr/bin/env python3
"""Draw the four bilingual X500 IMU measurement cards from measure.py's CSV files."""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np
from matplotlib.colors import LogNorm

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent / "error-characteristics"))
from cards_common import (BODY, C0, C1, C2, C3, INK, LANG, MUTED, L, airframe, card, chip, notes,  # noqa: E402
                          plt, result_title, save, sketch_axes, spin_arrow)

REAL, SIM, OFF, NOTCH = "#333333", "#d0553c", "#9bb7d4", "#3b8a5a"
FMIN, FMAX = 40.0, 1000.0


def rows(data, name):
    with (Path(data) / name).open() as handle:
        return list(csv.DictReader(handle))


def f(row, key):
    return float(row[key])


def drone_sketch(ax, lines, chips=True):
    sketch_axes(ax, xlim=(-0.48, 0.48), ylim=(-0.48, 0.52))
    airframe(ax, size=0.30, centre=(0, 0.15))
    spin_arrow(ax, 0.31, L("ESC の回転数", "ESC rotor speed"), color=C1)
    if chips:
        chip(ax, (0, 0.15), C2, size=0.075, label=L("  生 IMU", "  raw IMU"), fontsize=9)
    notes(ax, lines, y=-0.24, size=8.1)


def orders(args):
    summary = rows(args.data, "order_summary.csv")[0]
    windows = rows(args.data, "order_windows.csv")
    mapped = rows(args.data, "order_map.csv")
    x = np.array(sorted({f(row, "rotor_hz") for row in mapped}))
    y = np.array(sorted({f(row, "vibration_hz") for row in mapped}))
    xi = {value: index for index, value in enumerate(x)}
    yi = {value: index for index, value in enumerate(y)}
    level = np.full((len(y), len(x)), np.nan)
    for row in mapped:
        level[yi[f(row, "vibration_hz")], xi[f(row, "rotor_hz")]] = f(row, "level_deg_s_sqrt_hz")
    positive = level[np.isfinite(level) & (level > 0)]
    fig, left, ax = card(L("B1 線の位置：ジャイロの山は、モーター回転の整数倍に乗るか",
                                "B1 Line positions: do gyro peaks lie at integer rotor orders?"))
    drone_sketch(left, L([
        f"実機 20 飛行、空中の 0.5 秒窓（{int(f(summary, 'real_windows')):,} 窓）",
        "生ジャイロ 40–1000 Hz の最も強い山",
        "各モーターの k × 回転周波数の ±3 %、k = 1…6",
        "橙の点：athrill3 vib-nonotch（3 回）",
    ], [
        f"20 real flights, airborne 0.5 s windows ({int(f(summary, 'real_windows')):,})",
        "strongest raw-gyro peak from 40–1000 Hz",
        "±3% of k × each motor rotor frequency, k = 1…6",
        "orange dots: athrill3 vib-nonotch (three runs)",
    ]))
    mesh = ax.pcolormesh(x, y, level, shading="nearest", cmap="Blues",
                         norm=LogNorm(vmin=np.percentile(positive, 5), vmax=np.percentile(positive, 99.5)))
    for order in range(1, 7):
        xx = np.array([x.min(), min(x.max(), FMAX / order)])
        ax.plot(xx, order * xx, color=INK, lw=0.8, ls="--", alpha=0.65)
        if order * x.min() <= 980:
            ax.text(x.min() + 1, order * x.min() + 6, f"{order}×", color=INK, fontsize=7.5)
    sim = [row for row in windows if row["source"] == "athrill"]
    ax.scatter([f(row, "rotor_hz") for row in sim], [f(row, "peak_hz") for row in sim], s=7, color=SIM,
               alpha=0.42, edgecolors="none", label="athrill3")
    ax.set_xlim(25, 270)
    ax.set_ylim(FMIN, FMAX)
    ax.set_xlabel(L("4 モーターの平均回転周波数 [Hz]", "mean rotor frequency of four motors [Hz]"))
    ax.set_ylabel(L("ジャイロ振動の周波数 [Hz]", "gyro vibration frequency [Hz]"))
    result_title(ax, L("色：実機の生ジャイロのスペクトル密度（3 軸合成）", "colour: real raw-gyro spectral density (three axes)"))
    colorbar = fig.colorbar(mesh, ax=ax, pad=0.01)
    colorbar.set_label(L("揺れ [deg/s/√Hz]", "level [deg/s/√Hz]"), fontsize=8)
    inset = ax.inset_axes([0.57, 0.62, 0.39, 0.32])
    values = [f(summary, "real_hit_rate"), f(summary, "random_hit_rate"), f(summary, "wrong_time_hit_rate")]
    inset.bar(range(3), values, color=[C1, MUTED, "#b08968"])
    threshold = 2 * values[1]
    inset.axhline(threshold, color=C2, ls="--", lw=1)
    inset.text(2.48, threshold + 0.025, L("基準 2×無作為", "criterion 2× random"), fontsize=6.8,
               color=C2, ha="right")
    for index, value in enumerate(values):
        inset.text(index, value + 0.025, f"{value:.2f}", ha="center", fontsize=7.5, color=INK)
    inset.set_xticks(range(3), L(["同じ時刻", "無作為の山", "別の時刻"], ["true time", "random peak", "wrong time"]),
                     fontsize=6.8)
    inset.set_ylim(0, 1.08)
    inset.set_ylabel(L("当たった割合", "hit fraction"), fontsize=7)
    inset.tick_params(axis="y", labelsize=6.5)
    inset.grid(axis="y", alpha=0.2)
    ax.legend(loc="lower right", fontsize=7.5)
    save(fig, args, "card-imu-orders")


def amplitude(args):
    data = rows(args.data, "amplitude.csv")
    order = np.array([int(row["order"]) for row in data])
    real = np.array([f(row, "real_407292d1_deg_s") for row in data])
    lo = np.array([f(row, "real_flight_min_deg_s") for row in data])
    hi = np.array([f(row, "real_flight_max_deg_s") for row in data])
    sim = np.array([f(row, "athrill_median_deg_s") for row in data])
    sim_lo = np.array([f(row, "athrill_min_deg_s") for row in data])
    sim_hi = np.array([f(row, "athrill_max_deg_s") for row in data])
    ratio = np.array([f(row, "athrill_over_real") for row in data])
    fig, left, a1, a2 = card(L("B2 倍数ごとの大きさ：実機に合わせた振動は、閉ループでも同じ大きさか",
                                      "B2 Amplitude by order: does the fitted vibration survive closed loop?"), right=2)
    drone_sketch(left, L([
        "ホバー：水平 < 1 m/s、|上下| < 0.5 m/s",
        "0.5 秒ごと、各 k×回転帯の |xyz| RMS",
        "実機 407292d1：ICM-42688-P、8 kHz",
        "athrill3：BMI055、2 kHz、vib-nonotch 3 回",
    ], [
        "hover: horizontal < 1 m/s, |vertical| < 0.5 m/s",
        "0.5 s; |xyz| RMS in each k× rotor band",
        "real 407292d1: ICM-42688-P at 8 kHz",
        "athrill3: BMI055 at 2 kHz, three vib-nonotch runs",
    ]))
    a1.hlines(order, lo, hi, color=BODY, lw=5, label=L("実機 20 飛行の最小～最大", "min–max over 20 real flights"))
    a1.plot(real, order, "o", color=REAL, ms=6, label=L("実機 407292d1", "real 407292d1"))
    a1.errorbar(sim, order, xerr=[sim - sim_lo, sim_hi - sim], fmt="^", color=SIM, ms=6,
                capsize=2, label=L("athrill3：中央値と 3 回の幅", "athrill3: median and three-run range"))
    a1.set_xscale("log")
    a1.set_xlim(0.005, 70)
    a1.set_yticks(order, [f"{k}×" for k in order])
    a1.invert_yaxis()
    a1.set_xlabel(L("帯域 RMS、3 軸合成 [deg/s]", "band RMS, three-axis magnitude [deg/s]"))
    a1.set_ylabel(L("回転の倍数", "rotor order"))
    result_title(a1, L("測った大きさ（横棒は機体差）", "amplitude (bars: aircraft spread)"))
    a1.grid(axis="x", which="both", alpha=0.25)
    a1.legend(fontsize=7.2, loc="lower left")
    a2.axhspan(0.70, 1.43, color="#e6f3e9")
    a2.axhline(0.70, color=C3, ls="--", lw=1)
    a2.axhline(1.43, color=C3, ls="--", lw=1)
    a2.axhline(1.0, color=MUTED, lw=0.8)
    a2.plot(order, ratio, "o-", color=SIM, lw=1.2)
    for xvalue, value in zip(order, ratio):
        a2.text(xvalue, value * (1.18 if value > 0.02 else 1.5), f"{value:.2f}", ha="center", fontsize=7.5)
    a2.set_yscale("log")
    a2.set_ylim(0.0007, 2.5)
    a2.set_xticks(order, [f"{k}×" for k in order])
    a2.set_xlabel(L("回転の倍数", "rotor order"))
    a2.set_ylabel(L("athrill3 ÷ 実機 407292d1", "athrill3 / real 407292d1"))
    result_title(a2, L("基準：1～5 倍は 0.70～1.43", "criterion: 0.70–1.43"))
    a2.text(6, 0.0011, L("6 倍はモデル外", "6× not fitted"), ha="center", fontsize=7.5, color=MUTED)
    a2.grid(axis="y", which="both", alpha=0.25)
    save(fig, args, "card-imu-amplitude")


def exponent(args):
    data = rows(args.data, "exponent.csv")
    summary = {int(row["order"]): row for row in rows(args.data, "exponent_summary.csv")}
    fig, left, ax = card(L("B3 回転数による増え方：1 飛行の中で、揺れは回転の何乗か",
                                "B3 Speed exponent: within one flight, what power of rotor speed?"))
    drone_sketch(left, L([
        "空中の 0.5 秒窓（離陸・着陸を含む）",
        "p95 ÷ p5 の回転周波数が 1.2 以上の 8 飛行",
        "log(帯域 RMS) を log(平均回転周波数) に回帰",
        "点 1 つ = 1 飛行、縦棒 = IQR、横線 = 中央値",
    ], [
        "airborne 0.5 s windows (take-off/landing included)",
        "eight flights with rotor-frequency p95/p5 ≥ 1.2",
        "regress log(band RMS) on log(mean rotor frequency)",
        "one point per flight; bar = IQR; line = median",
    ]))
    rng = np.random.default_rng(4)
    colors = {1: C1, 2: SIM}
    for order in (1, 2):
        values = np.array([f(row, "slope") for row in data if int(row["order"]) == order])
        xpos = order + rng.uniform(-0.10, 0.10, len(values))
        ax.scatter(xpos, values, s=28, color=colors[order], alpha=0.75, zorder=3)
        q1, med, q3 = (f(summary[order], key) for key in ("q25", "median", "q75"))
        ax.vlines(order, q1, q3, color=INK, lw=5, alpha=0.55)
        ax.hlines(med, order - 0.22, order + 0.22, color=INK, lw=2)
        fitted = f(summary[order], "fitted_model")
        ax.hlines(fitted, order - 0.42, order + 0.42, color=C3, lw=1.4, ls="--")
        ax.text(order, max(values) + 0.28, L(f"中央値 {med:.2f}", f"median {med:.2f}"), ha="center", fontsize=8)
        ax.text(order + 0.43, fitted, L(f"設定 {fitted:.2f}", f"fitted {fitted:.2f}"), va="center", fontsize=7.3, color=C3)
    ax.axhline(2.0, color=MUTED, lw=1.2, ls="--")
    ax.text(0.57, 2.06, L("モデル既定値 2", "model default 2"), fontsize=7.5, color=MUTED)
    ax.set_xlim(0.5, 2.5)
    ax.set_ylim(-0.6, 4.5)
    ax.set_xticks([1, 2], L(["1 倍の帯", "2 倍の帯"], ["1× band", "2× band"]))
    ax.set_ylabel(L("帯域 RMS ∝ 回転周波数 p の指数 p", "exponent p in band RMS ∝ rotor frequencyᵖ"))
    ax.set_xlabel(L("回転の倍数", "rotor order"))
    result_title(ax, L("実機の飛行ごとの指数（n = 8）", "per-flight exponents in real logs (n = 8)"))
    ax.grid(axis="y", alpha=0.25)
    save(fig, args, "card-imu-exponent")


def px4_metric(args):
    data = rows(args.data, "px4_summary.csv")
    labels = {
        "gyro_vibration_metric": L("ジャイロ [rad/s]", "gyro [rad/s]"),
        "accel_vibration_metric": L("加速度 [m/s²]", "accel [m/s²]"),
        "delta_angle_coning_metric": L("円錐誤差 [rad²]", "delta-angle coning [rad²]"),
    }
    y = np.arange(len(data))
    real = np.array([f(row, "real_407292d1") for row in data])
    real_lo = np.array([f(row, "real_min") for row in data])
    real_hi = np.array([f(row, "real_max") for row in data])
    fig, left, a1, a2 = card(L("C1 PX4 自身の振動指標：実機と閉ループの PX4 が見る揺れ",
                                      "C1 PX4 vibration metrics: what real and closed-loop PX4 observe"), right=2)
    drone_sketch(left, L([
        "ホバー中の vehicle_imu_status（各記録の中央値）",
        "実機 407292d1 と実機 20 飛行の最小～最大",
        "athrill3：振動なし／ノッチなし／ノッチあり、各 3 回",
        "差分速度の円錐誤差は 29 記録すべてに項目なし",
    ], [
        "vehicle_imu_status in hover (median per log)",
        "real 407292d1 and min–max across 20 real flights",
        "athrill3: off / no notch / notch, three runs each",
        "delta-velocity coning field absent from all 29 logs",
    ]))
    a1.hlines(y, real_lo, real_hi, color=BODY, lw=5, label=L("実機 20 飛行の幅", "20-real-flight range"))
    a1.plot(real, y, "o", color=REAL, ms=6, label="407292d1")
    scenarios = [("off", OFF, "s", L("振動なし", "off")), ("vib-nonotch", SIM, "^", L("ノッチなし", "no notch")),
                 ("vib-notch", NOTCH, "D", L("ノッチあり", "notch"))]
    for index, (group, color, marker, label) in enumerate(scenarios):
        med = np.array([f(row, f"{group}_median") for row in data])
        lo = np.array([f(row, f"{group}_min") for row in data])
        hi = np.array([f(row, f"{group}_max") for row in data])
        a1.errorbar(med, y + (index - 1) * 0.08, xerr=[med - lo, hi - med], fmt=marker, color=color, ms=5,
                    capsize=2, label=label)
    a1.set_xscale("log")
    a1.set_yticks(y, [labels[row["metric"]] for row in data], fontsize=8)
    a1.invert_yaxis()
    a1.set_xlabel(L("PX4 が記録した値（各項目の固有単位）", "PX4 value (native unit of each metric)"))
    result_title(a1, L("絶対値（横棒は実機の機体差）", "absolute values (bars: real spread)"))
    a1.grid(axis="x", which="both", alpha=0.25)
    a1.legend(fontsize=6.8, loc="lower left")
    a2.axvspan(0.5, 2.0, color="#e6f3e9")
    a2.axvline(0.5, color=C3, ls="--", lw=1)
    a2.axvline(2.0, color=C3, ls="--", lw=1)
    a2.axvline(1.0, color=MUTED, lw=0.8)
    ratio_real_lo, ratio_real_hi = real_lo / real, real_hi / real
    a2.hlines(y, ratio_real_lo, ratio_real_hi, color=BODY, lw=5, label=L("実機 20 飛行", "20 real flights"))
    for index, (group, color, marker, label) in enumerate(scenarios):
        med = np.array([f(row, f"{group}_median") for row in data]) / real
        a2.plot(med, y + (index - 1) * 0.08, marker, color=color, ms=5, label=label)
        if group == "vib-nonotch":
            for yy, value in zip(y, med):
                a2.text(value * 1.2, yy, f"{value:.3g}×", va="center", fontsize=7.2, color=color)
    a2.set_xscale("log")
    a2.set_xlim(1e-4, 4)
    a2.set_yticks(y, [labels[row["metric"]] for row in data], fontsize=8)
    a2.invert_yaxis()
    a2.set_xlabel(L("各値 ÷ 実機 407292d1", "each value / real 407292d1"))
    result_title(a2, L("基準：ノッチなしが 0.5～2 倍", "criterion: no notch 0.5–2×"))
    a2.grid(axis="x", which="both", alpha=0.25)
    save(fig, args, "card-imu-px4-metric")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--lang", choices=("ja", "en"), default="ja")
    parser.add_argument("--only", nargs="*", choices=("orders", "amplitude", "exponent", "px4_metric"))
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    LANG[0] = args.lang
    makers = (orders, amplitude, exponent, px4_metric)
    for maker in makers:
        if not args.only or maker.__name__ in args.only:
            maker(args)


if __name__ == "__main__":
    main()
