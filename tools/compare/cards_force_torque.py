"""Measurement cards of the force-torque page: the Hakoniwa and Gazebo force-torque sensors side by side, one
characteristic per card (a sketch of the measurement on the left, the results on the right).

    python3 cards_force_torque.py --tool <build>/ft-series --out <dir> [--lang ja|en] [--only bias ...]

The data come from ft-series (ft_series.cpp) with the configurations of config/force-torque. The card layout is
shared with the error-characteristics cards (../error-characteristics/cards_common.py).
"""
import csv
import subprocess
import sys
from pathlib import Path

import numpy as np
from matplotlib.patches import Rectangle

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "error-characteristics"))
from cards_common import (BODY, C0, C1, C2, C3, INK, MUTED, L, card, main, notes, result_title, save,  # noqa: E402
                          sketch_axes)

HK, GZ = C1, "#b26a00"


def series(args, implementation, config, name, duration, **extra):
    """Runs ft-series once and returns its columns as arrays."""
    out = Path(args.work) / "ft" / f"{implementation}-{name}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    command = [args.tool, "--implementation", implementation, "--config", str(HERE / "config" / "force-torque" / config),
               "--duration", str(duration), "--out", str(out)]
    for key, value in extra.items():
        command += [f"--{key.replace('_', '-')}", str(value)]
    subprocess.run(command, check=True, capture_output=True)
    with open(out) as handle:
        rows = list(csv.DictReader(handle))
    return {key: np.array([float(row[key]) for row in rows]) for key in rows[0]}


def config(implementation, name):
    return f"{'hk' if implementation == 'hakoniwa' else 'gz'}-{name}.{'json' if implementation == 'hakoniwa' else 'sdf'}"


def wrist(ax, x0, y0, scale=1.0, color=C2):
    """A small wrist seen from the side: robot link, sensor, gripper."""
    s = scale
    ax.add_patch(Rectangle((x0 - 0.05 * s, y0), 0.1 * s, 0.16 * s, fc=BODY, ec="none"))
    ax.add_patch(Rectangle((x0 - 0.07 * s, y0 - 0.035 * s), 0.14 * s, 0.035 * s, fc=color, ec="none"))
    ax.add_patch(Rectangle((x0 - 0.045 * s, y0 - 0.12 * s), 0.09 * s, 0.085 * s, fc="#7c8aa3", ec="none"))


def units_row(ax, y, color, label, count=8):
    for k in range(count):
        wrist(ax, -0.36 + k * 0.1, y, scale=0.55, color=color)
    ax.text(0.42, y + 0.03, "…", fontsize=13, color=MUTED, va="center")
    ax.text(-0.42, y + 0.17, label, fontsize=9.5, color=color, fontweight="bold")


# ------------------------------------------------------------------------------------------------ bias
def bias(args):
    seeds = range(1, 41)
    values = {}
    for impl in ("hakoniwa", "gazebo"):
        values[impl] = np.array([series(args, impl, config(impl, "bias"), f"bias-{s}", 0.002, seed=s)["fx"][0]
                                 for s in seeds])
    fig, left, r1, r2 = card(L("バイアス：40 個のセンサーを、何もかけずに読む", "Bias: 40 sensors read with no load"), right=2)
    sketch_axes(left, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    units_row(left, 0.22, HK, L("箱庭 40 個（乱数の種 1〜40）", "Hakoniwa, 40 units (seeds 1–40)"))
    units_row(left, -0.06, GZ, L("Gazebo 40 個（同じ種）", "Gazebo, 40 units (same seeds)"))
    notes(left, L(["真値はずっと 0（何もかけない）。最初の値の Fx を読む",
                   "設定はどちらも「バイアス 0.5 N」",
                   "箱庭 \"bias\": 0.5、Gazebo <bias_mean>0.5、<bias_stddev>0"],
                  ["The truth stays 0 (no load). Fx of the first sample is read",
                   "Both are configured with a 0.5 N bias",
                   "Hakoniwa \"bias\": 0.5; Gazebo <bias_mean>0.5, <bias_stddev>0"]), y=-0.22)
    for ax, impl, color, title in ((r1, "hakoniwa", HK, L("箱庭：±0.5 N の範囲に散らばる", "Hakoniwa: spread within ±0.5 N")),
                                   (r2, "gazebo", GZ, L("Gazebo：+0.5 か −0.5 の 2 つだけ", "Gazebo: only +0.5 or −0.5"))):
        v = values[impl]
        ax.axhspan(-0.5, 0.5, color="#eef1f6", zorder=0)
        ax.axhline(0.5, color=MUTED, lw=0.8, ls="--")
        ax.axhline(-0.5, color=MUTED, lw=0.8, ls="--")
        ax.plot(list(seeds), v, "o", color=color, ms=6)
        distinct = len(np.unique(np.round(v, 9)))
        ax.text(0.5, 0.02, L(f"違う値の数：{distinct}　マイナス：{int((v < 0).sum())} 個",
                             f"distinct values: {distinct}, negative: {int((v < 0).sum())}"),
                transform=ax.transAxes, fontsize=9, color=color)
        ax.set_ylim(-0.75, 0.75)
        ax.set_xlabel(L("センサーの番号（乱数の種）", "sensor number (seed)"))
        ax.set_ylabel(L("Fx [N]（真値は 0）", "Fx [N] (truth 0)"))
        result_title(ax, title)
        ax.grid(alpha=0.3)
    save(fig, args, "card-ft-bias")


def pair_sketch(ax, title_ja, title_en, lines):
    """One Hakoniwa and one Gazebo sensor fed the same truth."""
    sketch_axes(ax, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    ax.text(0, 0.42, L(title_ja, title_en), fontsize=10, color=INK, ha="center")
    wrist(ax, -0.18, 0.12, scale=1.2, color=HK)
    wrist(ax, 0.18, 0.12, scale=1.2, color=GZ)
    ax.text(-0.18, -0.08, L("箱庭", "Hakoniwa"), fontsize=9.5, color=HK, ha="center", fontweight="bold")
    ax.text(0.18, -0.08, "Gazebo", fontsize=9.5, color=GZ, ha="center", fontweight="bold")
    notes(ax, lines, y=-0.17)


def one_sketch(ax, title_ja, title_en, lines):
    """One Hakoniwa sensor (a characteristic Gazebo does not have)."""
    sketch_axes(ax, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    ax.text(0, 0.42, L(title_ja, title_en), fontsize=10, color=INK, ha="center")
    wrist(ax, 0, 0.12, scale=1.2, color=HK)
    ax.text(0, -0.08, L("箱庭（Gazebo にはない機能）", "Hakoniwa (Gazebo has no such feature)"), fontsize=9.5, color=HK,
            ha="center", fontweight="bold")
    notes(ax, lines, y=-0.17)


def sampled(d):
    m = d["updated"] > 0
    return {k: v[m] for k, v in d.items()}


# ------------------------------------------------------------------------------------------------ agreement
def agreement(args):
    h = sampled(series(args, "hakoniwa", "hk-ideal.json", "ideal", 10.0, motion="sine"))
    g = sampled(series(args, "gazebo", "gz-ideal.sdf", "ideal", 10.0, motion="sine"))
    first = h["time"] <= 2.0
    fig, left, r1, r2 = card(L("ノイズなしでの一致：同じ真値で、出力が重なる", "No-noise agreement: the same truth, the same output"),
                             right=2)
    pair_sketch(left, "同じ揺れる力を、2 つに渡す", "the same moving force into both",
                L(["誤差はすべて切る。6 軸それぞれに、振幅と周波数の違う揺れ",
                   "500 Hz で 10 秒（5000 サンプル）。中央は Fz の最初の 2 秒"],
                  ["Every error off. Each of the 6 axes swings with its own amplitude and frequency",
                   "500 Hz for 10 s (5000 samples). Middle: Fz, the first 2 s"]))
    r1.plot(h["time"][first], h["true_fz"][first], color=MUTED, lw=4, label=L("真値", "truth"))
    r1.plot(h["time"][first], h["fz"][first], color=HK, lw=1.8, label=L("箱庭", "Hakoniwa"))
    r1.plot(g["time"][first], g["fz"][first], color=GZ, lw=1.2, ls="--", label="Gazebo")
    r1.set_xlabel(L("時間 [s]", "time [s]"))
    r1.set_ylabel("Fz [N]")
    result_title(r1, L("3 本が重なる", "The three lines coincide"))
    r1.legend(loc="lower right", fontsize=8.5)
    r1.grid(alpha=0.3)
    names = ["fx", "fy", "fz", "tx", "ty", "tz"]
    worst = [np.abs(h[n] - g[n]).max() for n in names]
    r2.bar(range(6), np.maximum(worst, 1e-19), color=INK)
    for i, w in enumerate(worst):
        r2.text(i, max(w, 1e-19) * 2, f"{w:.1e}" if w > 0 else "0", ha="center", fontsize=8.5, color=INK)
    r2.set_yscale("log")
    r2.axhline(1e-12, color=MUTED, lw=0.8, ls=":")
    r2.text(-0.4, 1.6e-12, L("1e-12（丸め誤差の目安）", "1e-12 (rounding scale)"), fontsize=8.5, color=MUTED)
    r2.set_ylim(1e-19, 1e-10)
    r2.set_xticks(range(6))
    r2.set_xticklabels(["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"])
    r2.set_ylabel(L("|箱庭 − Gazebo| の最大（10 秒）[N, N·m]", "max |Hakoniwa − Gazebo| over 10 s [N, N·m]"))
    result_title(r2, L("差は丸め誤差の大きさだけ", "Differences only of rounding size"))
    r2.grid(alpha=0.3, which="both", axis="y")
    save(fig, args, "card-ft-agreement")


# ------------------------------------------------------------------------------------------------ noise
def noise(args):
    h = sampled(series(args, "hakoniwa", "hk-noise.json", "noise", 40.0, seed=1))
    g = sampled(series(args, "gazebo", "gz-noise.sdf", "noise", 40.0, seed=1))
    fig, left, r1, r2 = card(L("ノイズ：静止したセンサーを 40 秒測る", "Noise: sensors at rest for 40 s"), right=2)
    pair_sketch(left, "何もかけずに、測り続ける", "no load, measured all along",
                L(["500 Hz で 20,000 サンプル。真値はずっと 0", "設定はどちらも Fx の σ 0.1 N",
                   "箱庭：データシートの 1σ を書く（\"definition\": \"1sigma\"）"],
                  ["20,000 samples at 500 Hz. The truth stays 0", "Both are configured with σ 0.1 N on Fx",
                   "Hakoniwa: the datasheet 1σ (\"definition\": \"1sigma\")"]))
    x = np.linspace(-0.45, 0.45, 300)
    pdf = np.exp(-0.5 * (x / 0.1) ** 2) / (0.1 * np.sqrt(2 * np.pi))
    for ax, d, color, name in ((r1, h, HK, L("箱庭", "Hakoniwa")), (r2, g, GZ, "Gazebo")):
        v = d["fx"]
        _, edges, _ = ax.hist(v, bins=60, color=color, alpha=0.45,
                              label=L(f"{len(v)} 回（標準偏差 {v.std():.4f}）", f"{len(v)} samples (sd {v.std():.4f})"))
        ax.plot(x, len(v) * (edges[1] - edges[0]) * pdf, color=C2, lw=2,
                label=L("設定 σ 0.1 なら、こう出るはず", "expected with σ 0.1"))
        ax.set_xlabel(L("Fx [N]（真値は 0）", "Fx [N] (truth 0)"))
        ax.set_ylabel(L("その値が出た回数", "count"))
        ax.set_ylim(0, ax.get_ylim()[1] * 1.3)
        result_title(ax, name)
        ax.legend(loc="upper left", fontsize=8.3)
        ax.grid(alpha=0.3)
    save(fig, args, "card-ft-noise")


# ------------------------------------------------------------------------------------------------ drift
def drift(args):
    duration = 120.0
    fig, left, r1, r2 = card(L("ドリフト：静止したセンサー 3 個ずつを 2 分測る", "Drift: three sensors each, at rest for 2 minutes"),
                             right=2)
    pair_sketch(left, "何もかけずに、測り続ける", "no load, measured all along",
                L(["箱庭：ランダムウォーク、\"drift_per_hour\" 60 N",
                   "Gazebo：ガウス・マルコフ、<dynamic_bias_stddev> 1 N、",
                   "　　　　<dynamic_bias_correlation_time> 10 秒", "乱数の種 1〜3。図は Fx"],
                  ["Hakoniwa: random walk, \"drift_per_hour\" 60 N",
                   "Gazebo: Gauss-Markov, <dynamic_bias_stddev> 1 N,",
                   "<dynamic_bias_correlation_time> 10 s", "seeds 1–3. The plots show Fx"]))
    t = np.linspace(0, duration, 400)
    for ax, impl, color, title in ((r1, "hakoniwa", HK, L("箱庭：広がり続ける", "Hakoniwa: keeps spreading")),
                                   (r2, "gazebo", GZ, L("Gazebo：帯の中に留まる", "Gazebo: stays within a band"))):
        for seed, ls in zip((1, 2, 3), ("-", "--", ":")):
            d = sampled(series(args, impl, config(impl, "drift"), f"drift-{seed}", duration, seed=seed, every=10))
            ax.plot(d["time"], d["fx"], color=color, lw=1.2, ls=ls, label=L(f"種 {seed}", f"seed {seed}"))
        if impl == "hakoniwa":
            env = 60 * np.sqrt(t / 3600)
            ax.fill_between(t, -env, env, color="#eef1f6", zorder=0)
            ax.text(5, 9.5, L("±σ：60 × √(t ÷ 1 時間)\n時間とともに広がる", "±σ: 60 × √(t ÷ 1 h)\nwidens with time"),
                    fontsize=8.5, color=INK)
        else:
            s = 1.0 * np.sqrt(10 / 2)
            ax.axhspan(-s, s, color="#eef1f6", zorder=0)
            ax.axhline(1.0, color=C2, lw=1, ls="--")
            ax.axhline(-1.0, color=C2, lw=1, ls="--")
            ax.text(5, 9.5, L(f"±σ = 1 × √(τ ÷ 2) = {s:.2f} N（帯）\n破線：設定値 1 N",
                              f"±σ = 1 × √(τ ÷ 2) = {s:.2f} N (band)\ndashed: the setting, 1 N"), fontsize=8.5, color=INK)
        ax.set_ylim(-12, 12)
        ax.set_xlabel(L("時間 [s]", "time [s]"))
        ax.set_ylabel(L("Fx [N]（真値は 0）", "Fx [N] (truth 0)"))
        result_title(ax, title)
        ax.legend(loc="lower left", fontsize=8, ncol=3)
        ax.grid(alpha=0.3)
    save(fig, args, "card-ft-drift")


# ------------------------------------------------------------------------------------------------ repeatability
def repeatability(args):
    fig, left, r1, r2 = card(L("再現性：同じ種なら、同じ値の列になるか", "Repeatability: does the same seed give the same sequence?"),
                             right=2)
    pair_sketch(left, "同じ乱数の種 5 で、2 通りに動かす", "seed 5, run in two ways",
                L(["中央：間に別のセンサー（種 99）を 1 刻みずつ動かす",
                   "右：4 秒でリセットして、同じ真値をもう一度",
                   "どちらも、ひとりで動かした列との差を見る（ノイズ σ 0.1 N）"],
                  ["Middle: another sensor (seed 99) stepped in between",
                   "Right: reset at 4 s, then the same truth again",
                   "Both compared with the sequence run alone (noise σ 0.1 N)"]))
    for impl, color in (("hakoniwa", HK), ("gazebo", GZ)):
        cfg = config(impl, "noise")
        alone = sampled(series(args, impl, cfg, "alone", 4.0, seed=5))
        other = sampled(series(args, impl, cfg, "other", 4.0, seed=5, other_seed=99))
        top = dict(lw=2.5, zorder=5) if impl == "hakoniwa" else dict(lw=0.8, alpha=0.6)
        r1.plot(alone["time"], other["fx"] - alone["fx"], color=color, **top,
                label=L("箱庭" if impl == "hakoniwa" else "Gazebo", "Hakoniwa" if impl == "hakoniwa" else "Gazebo"))
        reset = sampled(series(args, impl, cfg, "reset", 8.0, seed=5, reset_at=4.0))
        n = len(alone["fx"])
        r2.plot(alone["time"], reset["fx"][n:2 * n] - alone["fx"], color=color, **top,
                label=L("箱庭" if impl == "hakoniwa" else "Gazebo", "Hakoniwa" if impl == "hakoniwa" else "Gazebo"))
    for ax, title in ((r1, L("別のセンサーが間に動いたとき", "With another sensor in between")),
                      (r2, L("リセットの後", "After a reset"))):
        ax.set_xlabel(L("時間 [s]", "time [s]"))
        ax.set_ylabel(L("Fx の差 [N]（0 なら同じ列）", "Fx difference [N] (0 = same sequence)"))
        result_title(ax, title)
        ax.legend(loc="upper right", fontsize=8.5)
        ax.set_ylim(-0.6, 0.6)
        ax.grid(alpha=0.3)
    r1.text(0.1, -0.56, L("箱庭はずっと 0（青い線）。\nGazebo は乱数の流れを共有するので崩れる",
                          "Hakoniwa stays 0 (blue line).\nGazebo shares one random\nstream, so it breaks"), fontsize=8.3,
            color=INK, va="bottom", bbox=dict(fc="white", ec="none", alpha=0.85))
    r2.text(0.1, -0.56, L("箱庭は作った直後と同じ列に戻る（0）。\nGazebo は戻らない",
                          "Hakoniwa returns to the sequence\nafter creation (0). Gazebo does not"), fontsize=8.3,
            color=INK, va="bottom", bbox=dict(fc="white", ec="none", alpha=0.85))
    save(fig, args, "card-ft-repeatability")


# ------------------------------------------------------------------------------------------------ quantization
def quantization(args):
    h = sampled(series(args, "hakoniwa", "hk-quant.json", "quant", 1.0, motion="ramp", rate=0.4, seed=1))
    g = sampled(series(args, "gazebo", "gz-quant.sdf", "quant", 1.0, motion="ramp", rate=0.4, seed=1))
    fig, left, r1, r2 = card(L("量子化：値は刻みでしか出ない", "Quantization: the value comes in steps"), right=2)
    pair_sketch(left, "ゆっくり大きくなる力を渡す", "a slowly growing force",
                L(["Fz を毎秒 0.4 N ずつ増やす（1 秒）", "設定はどちらも刻み 0.05 N（Fz）とノイズ σ 0.2 N",
                   "図はノイズより小さな範囲を拡大"],
                  ["Fz grows by 0.4 N per second (1 s)", "Both: steps of 0.05 N (Fz) and noise σ 0.2 N",
                   "The plots zoom in below the noise"]))
    for ax, d, color, name in ((r1, h, HK, L("箱庭", "Hakoniwa")), (r2, g, GZ, "Gazebo")):
        ax.plot(d["time"], d["fz"], "o", color=color, ms=3.5, label=L("出力", "output"))
        ax.plot(d["time"], d["true_fz"], color=MUTED, lw=2, label=L("真値", "truth"))
        for k in range(-12, 14):
            ax.axhline(k * 0.05, color="#c9d6ea", lw=0.7, zorder=0)
        off = np.abs(d["fz"] / 0.05 - np.round(d["fz"] / 0.05)) > 1e-6
        ax.set_ylim(-0.12, 0.52)
        ax.set_xlim(0.4, 0.9)
        ax.set_xlabel(L("時間 [s]", "time [s]"))
        ax.set_ylabel("Fz [N]")
        result_title(ax, L(f"{name}：刻みにない値 {int(off.sum())} 個", f"{name}: {int(off.sum())} values off the steps"))
        ax.legend(loc="upper left", fontsize=8.5)
    save(fig, args, "card-ft-quantization")


# ------------------------------------------------------------------------------------------------ Hakoniwa only
def tare_filter(args):
    tare = sampled(series(args, "hakoniwa", "hk-tare.json", "tare", 4.0, motion="step", rate=10, t0=2.5, tare_at=1.0,
                          seed=3))
    raw = sampled(series(args, "hakoniwa", "hk-tare.json", "notare", 4.0, motion="step", rate=10, t0=2.5, seed=3))
    lp = sampled(series(args, "hakoniwa", "hk-lowpass.json", "lowpass", 0.3, motion="step", rate=10, t0=0.1))
    ideal = sampled(series(args, "hakoniwa", "hk-ideal.json", "ideal-step", 0.3, motion="step", rate=10, t0=0.1))
    fig, left, r1, r2 = card(L("箱庭だけの機能：風袋引きとローパスフィルタ", "Hakoniwa only: tare and the low-pass filter"), right=2)
    one_sketch(left, "1 秒で tare()、2.5 秒で 10 N 押す", "tare() at 1 s, a 10 N push at 2.5 s",
               L(["中央：バイアス 0.5 N とノイズ σ 0.2 N（Fz）の設定。tare() は直前 0.1 秒の平均を引く",
                  "右：ローパス 20 Hz（\"lowpass_cutoff_hz\"）と、フィルタなし",
                  "（0.1 秒で 10 N 押す）"],
                 ["Middle: bias 0.5 N and noise σ 0.2 N (Fz); tare() subtracts the mean of the last 0.1 s",
                  "Right: a 20 Hz low-pass (\"lowpass_cutoff_hz\") against none",
                  "(a 10 N push at 0.1 s)"]))
    r1.plot(raw["time"], raw["fz"], color=MUTED, lw=0.7, label=L("tare() なし", "no tare()"))
    r1.plot(tare["time"], tare["fz"], color=HK, lw=0.8, label=L("1 秒で tare()", "tare() at 1 s"))
    r1.plot(tare["time"], tare["true_fz"], color=INK, lw=1.5, ls="--", label=L("真値", "truth"))
    r1.axvline(1.0, color=C3, lw=1, ls=":")
    after = (tare["time"] > 1.1) & (tare["time"] < 2.4)
    r1.text(1.05, -1.6, L(f"tare 後の平均 {tare['fz'][after].mean():+.3f} N", f"mean after tare {tare['fz'][after].mean():+.3f} N"),
            fontsize=8.5, color=HK)
    r1.set_ylim(-2, 12)
    r1.set_xlabel(L("時間 [s]", "time [s]"))
    r1.set_ylabel("Fz [N]")
    result_title(r1, L("風袋引き：ずれが消え、押した力だけ残る", "Tare: the offset goes, the push stays"))
    r1.legend(loc="upper left", fontsize=8.3)
    r1.grid(alpha=0.3)
    r2.step(ideal["time"] * 1000, ideal["fz"], where="post", color=MUTED, lw=2, label=L("フィルタなし", "no filter"))
    r2.plot(lp["time"] * 1000, lp["fz"], color=HK, lw=2, label=L("20 Hz フィルタ", "20 Hz filter"))
    cross = lp["time"][np.argmax(lp["fz"] >= 9.0)] * 1000 - 100
    r2.text(135, 3, L(f"9 N に届くまで\n{cross:.0f} ms", f"{cross:.0f} ms\nto reach 9 N"), fontsize=9, color=HK)
    r2.set_xlabel(L("時間 [ms]", "time [ms]"))
    r2.set_ylabel("Fz [N]")
    result_title(r2, L("フィルタ：なめらかになるが、遅れる", "Filter: smoother, but later"))
    r2.legend(loc="lower right", fontsize=8.3)
    r2.grid(alpha=0.3)
    save(fig, args, "card-ft-tare-filter")


def calibration_limits(args):
    cal = sampled(series(args, "hakoniwa", "hk-calib.json", "calib", 1.0, motion="ramp", rate=200, seed=2))
    lim = sampled(series(args, "hakoniwa", "hk-limits.json", "limits", 1.0, motion="ramp", rate=1300, every=1))
    fig, left, r1, r2 = card(L("箱庭だけの機能：校正のずれと、飽和・過負荷", "Hakoniwa only: calibration errors, saturation and overload"),
                             right=2)
    one_sketch(left, "Fz をだんだん大きくする", "Fz grows steadily",
               L(["中央：0 → 200 N（レンジいっぱい）。非線形 2 %、",
                  "　スケール誤差 3 %、軸の混ざり 5 %（乱数の種 2 の個体）",
                  "右：0 → 1300 N。レンジ 200 N、過負荷 500 %（1000 N）"],
                 ["Middle: 0 → 200 N (the full range). Nonlinearity 2 %,",
                  "scale error 3 %, crosstalk 5 % (the unit of seed 2)",
                  "Right: 0 → 1300 N. Range 200 N, overload 500 % (1000 N)"]))
    r1.plot(cal["true_fz"], cal["fz"] - cal["true_fz"], color=HK, lw=2, label=L("Fz のずれ（出力 − 真値）", "Fz error (output − truth)"))
    r1.plot(cal["true_fz"], cal["fx"], color=C2, lw=2, label=L("Fx に漏れた力（真値は 0）", "Fx leak (truth 0)"))
    r1.axhline(0, color=MUTED, lw=1)
    r1.set_xlabel(L("Fz の真値 [N]", "true Fz [N]"))
    r1.set_ylabel(L("ずれ [N]", "error [N]"))
    result_title(r1, L("Fz はずれ、Fx にも漏れる", "Fz is off, and leaks into Fx"))
    r1.legend(loc="upper left", fontsize=8.3)
    r1.grid(alpha=0.3)
    flags = lim["flags"].astype(int)
    r2.plot(lim["time"], lim["true_fz"], color=MUTED, lw=2, ls="--", label=L("真値", "truth"))
    r2.plot(lim["time"], lim["fz"], color=HK, lw=2.2, label=L("出力", "output"))
    sat = lim["time"][np.argmax(flags & 1)] if (flags & 1).any() else np.nan
    ovl = lim["time"][np.argmax(flags & 256)] if (flags & 256).any() else np.nan
    r2.axvline(sat, color=C2, lw=1, ls=":")
    r2.axvline(ovl, color=INK, lw=1, ls=":")
    r2.text(sat + 0.02, 600, L(f"{sat:.3f} 秒から\n「飽和」", f"saturated\nfrom {sat:.3f} s"), fontsize=8.5, color=C2)
    r2.text(ovl + 0.02, 300, L(f"{ovl:.3f} 秒から\n「過負荷」", f"overloaded\nfrom {ovl:.3f} s"), fontsize=8.5, color=INK)
    r2.set_ylim(0, 1400)
    r2.set_xlabel(L("時間 [s]", "time [s]"))
    r2.set_ylabel("Fz [N]")
    result_title(r2, L("200 N で止まり、印が付く", "Stops at 200 N, with flags"))
    r2.legend(loc="upper left", fontsize=8.3)
    r2.grid(alpha=0.3)
    save(fig, args, "card-ft-calibration-limits")


if __name__ == "__main__":
    main(__doc__, [agreement, noise, bias, drift, repeatability, quantization, tare_filter, calibration_limits])
