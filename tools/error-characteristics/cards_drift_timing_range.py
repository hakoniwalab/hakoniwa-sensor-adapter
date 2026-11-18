"""Measurement cards of the drift, period-and-delay and resolution-and-range sections (Japanese).

    python3 cards_drift_timing_range.py --tool <build>/error-characteristics --out <dir> [--only drift_gyro ...]

See cards_common.py for what a card is.
"""
import numpy as np

from cards_common import (C0, C1, C2, C3, DEG, INK, MUTED, airframe, arrow, card, chip, main, north_east, notes,
                          plt, result_title, run, sampled, save, sketch_axes)
from ec_common import allan

UNIT_COLORS = (C1, C2, C3)


def rest_units(ax, label, color, lines, count=3):
    sketch_axes(ax, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    for k in range(count):
        x = -0.28 + 0.28 * k
        airframe(ax, size=0.1, alpha=0.6, centre=(x, 0.2))
        chip(ax, (x, 0.2), UNIT_COLORS[k], size=0.04)
        ax.text(x, 0.06, f"{label} {k + 1}", fontsize=9, color=UNIT_COLORS[k], ha="center", va="top")
    ax.text(0, 0.45, "静止したまま 30 分", fontsize=10, color=INK, ha="center")
    notes(ax, lines, y=-0.06)


def allan_plot(ax, values, dt, white_sigma, unit, label):
    taus = np.logspace(np.log10(dt), np.log10(180), 36)  # up to a tenth of the 30 min run
    adev = allan(values, dt, taus)
    white = white_sigma / np.sqrt(taus / dt)
    ax.loglog(taus, adev, color=C2, lw=2.2, label=label)
    ax.loglog(taus, white, color=MUTED, lw=1.2, ls="--", label="ノイズだけなら（平均で下がり続ける）")
    i = int(np.argmax(adev > 1.5 * white))
    ax.plot(taus[i], adev[i], "o", color=C2, ms=8)
    ax.annotate("このあたりから\n下がらなくなる\n＝ドリフトが見える", (taus[i], adev[i]), xytext=(10, -22),
                textcoords="offset points", fontsize=8.5, color=C2, ha="left", va="top")
    ax.set_xlabel("平均する時間 [s]")
    ax.set_ylabel(f"平均した値のばらつき [{unit}]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3, which="both")


# ------------------------------------------------------------------------------------------------ 4. drift
def drift_gyro(args):
    fig, left, r1, r2 = card("ドリフト：静止したジャイロ 3 個を 30 分測る", right=2)
    rest_units(left, "IMU", C1, ["設定：ドリフト σ 0.005°/s、τ 100 秒（\"gauss-markov\"）",
                                 "左のグラフ：ドリフトだけを入れた 3 個",
                                 "右のグラフ：ノイズ（σ 0.05°/s）も入れた 1 個"])
    for seed, color in zip((1, 2, 3), UNIT_COLORS):
        d = sampled(run(args, "imu", f"drift-{seed}", "drift.json", "rest", 1800.0, seed=seed, every=4),
                    "gyro_updated")
        r1.plot(d["time"] / 60.0, d["gx"] / DEG, lw=1, color=color, label=f"IMU {seed}")
    r1.axhspan(-0.005, 0.005, color="#fbe3e5", zorder=0)
    r1.text(30.5, 0.005, "+σ", fontsize=9, color=C2, va="center")
    r1.text(30.5, -0.005, "−σ", fontsize=9, color=C2, va="center")
    r1.set_xlim(0, 33)
    r1.set_xlabel("時間 [分]")
    r1.set_ylabel("ジャイロ x の値 [°/s]（真値は 0）")
    result_title(r1, "ゆっくりさまよう（帯：±σ）")
    r1.legend(loc="upper left", fontsize=8, ncol=3)
    r1.grid(alpha=0.3)
    d = sampled(run(args, "imu", "noise-drift", "noise-drift.json", "rest", 1800.0, seed=1, every=4), "gyro_updated")
    allan_plot(r2, d["gx"] / DEG, 0.004, 0.05, "°/s", "測った値（ノイズ＋ドリフト）")
    result_title(r2, "長く平均しても消えない")
    save(fig, args, "card-drift-gyro")


def drift_mag(args):
    fig, left, right = card("ドリフト：静止した地磁気センサー 3 個を 30 分測る")
    rest_units(left, "地磁気", C2, ["設定：ドリフト σ 100 nT、τ 60 秒", "ドリフトだけを入れる（ノイズなし）"])
    for seed, color in zip((1, 2, 3), UNIT_COLORS):
        dd = sampled(run(args, "mag", f"drift-{seed}", "drift.json", "rest", 1800.0, seed=seed, every=20))
        right.plot(dd["time"] / 60.0, dd["mx"] - dd["true_mx"], lw=1, color=color, label=f"センサー {seed}")
    right.axhspan(-100, 100, color="#fbe3e5", zorder=0)
    right.text(30.5, 100, "+σ", fontsize=9, color=C2, va="center")
    right.text(30.5, -100, "−σ", fontsize=9, color=C2, va="center")
    right.set_xlim(0, 33)
    right.set_xlabel("時間 [分]")
    right.set_ylabel("地磁気 x − 真値 [nT]")
    result_title(right, "結果：τ が短いので、速く行き来する（帯：±σ）")
    right.legend(loc="upper left", fontsize=8, ncol=3)
    right.grid(alpha=0.3)
    save(fig, args, "card-drift-mag")


def drift_baro(args):
    fig, left, r1, r2 = card("ドリフト：静止した気圧センサーを 30 分測る", right=2)
    rest_units(left, "気圧", C3, ["設定：ノイズ σ 0.1 m、ドリフト σ 0.3 m・τ 30 秒", "1 個を 30 分（周期 20 ms）"], count=1)
    dd = sampled(run(args, "baro", "noise-drift", "noise-drift.json", "rest", 1800.0, every=20))
    err = dd["pressure_altitude"] - dd["true_altitude"]
    t = dd["time"] / 60.0
    r1.plot(t, err, lw=0.3, color="#f2b8bd", label="測った値")
    win = 50
    r1.plot(t[win // 2:-win // 2 + 1], np.convolve(err, np.ones(win) / win, mode="valid"), color=C2, lw=1.4,
            label="1 秒ごとの平均（ノイズが消え、ドリフトが残る）")
    r1.set_xlim(0, 10)
    r1.set_ylim(-1.3, 1.6)
    r1.set_xlabel("時間 [分]（最初の 10 分）")
    r1.set_ylabel("気圧から求めた高度 − 真値 [m]")
    result_title(r1, "平均しても、ゆっくり動く")
    r1.legend(loc="upper left", fontsize=8)
    r1.grid(alpha=0.3)
    allan_plot(r2, err, 0.02, 0.1, "m", "測った値（ノイズ＋ドリフト）")
    result_title(r2, "長く平均しても消えない")
    save(fig, args, "card-drift-baro")


def drift_gps(args):
    fig, left, r1, r2 = card("ドリフト：静止した GPS を 30 分測る", right=2)
    rest_units(left, "GPS", C2, ["左のグラフ：ドリフトだけ（水平 σ 0.5 m、τ 60 秒）", "右のグラフ：ノイズ（σ 0.3 m）も入れる",
                                 "毎秒 10 回測位"], count=1)
    d = sampled(run(args, "gps", "drift", "drift.json", "rest", 1800.0, every=100))
    n, e = north_east(d)
    sc = r1.scatter(e, n, c=d["time"] / 60.0, s=2, cmap="viridis")
    fig.colorbar(sc, ax=r1, label="時間 [分]", shrink=0.85)
    r1.plot(0, 0, "+", color=C0, ms=14, mew=2.5)
    r1.set_aspect("equal")
    r1.set_xlabel("東 [m]（真の位置から）")
    r1.set_ylabel("北 [m]")
    result_title(r1, "測位が、ゆっくりさまよう（色＝時刻）")
    r1.grid(alpha=0.3)
    dd = sampled(run(args, "gps", "noise-drift", "noise-drift.json", "rest", 1800.0, every=100))
    nn, _ = north_east(dd)
    allan_plot(r2, nn, 0.1, 0.3, "m", "北の位置（ノイズ＋ドリフト）")
    result_title(r2, "長く平均しても消えない")
    save(fig, args, "card-drift-gps")


# ------------------------------------------------------------------------------------------------ 5. period and delay
def timing_sketch(ax, event, period, delay, lines):
    """A time line: the event, the moments of measurement, and the delay."""
    sketch_axes(ax, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    ax.text(0, 0.42, event, fontsize=10, color=INK, ha="center")
    x0, x1 = -0.42, 0.42
    ax.plot([x0, x1], [0.15, 0.15], color=INK, lw=1)
    scale = 0.84 / (4 * period + delay + period)
    for k in range(int((x1 - x0) / (period * scale)) + 1):
        x = x0 + k * period * scale
        ax.plot([x, x], [0.12, 0.18], color=C1, lw=1.5)
    ax.text(x0, 0.22, f"測る瞬間（{period:g} ms ごと）", fontsize=8.5, color=C1)
    te = x0 + 1.5 * period * scale
    ax.annotate("", xy=(te, 0.15), xytext=(te, 0.3), arrowprops=dict(arrowstyle="-|>", color=C2))
    ax.text(te + 0.01, 0.3, "変化", fontsize=9, color=C2)
    if delay:
        ax.annotate("", xy=(te + delay * scale, 0.05), xytext=(te, 0.05),
                    arrowprops=dict(arrowstyle="<->", color=C3))
        ax.text(te + delay * scale / 2, 0.02, f"遅れ {delay:g} ms", fontsize=8.5, color=C3, ha="center", va="top")
    notes(ax, lines, y=-0.12)


def step_card(ax, t, truth, a, b, ylabel, title):
    ax.step(t, truth, where="post", color=MUTED, lw=2.5, label="真値")
    ax.step(t, a, where="post", color=C1, lw=2, label="周期だけ")
    ax.step(t, b, where="post", color=C3, lw=2, ls="--", label="周期＋遅れ")
    ax.axvline(0, color=C2, lw=0.8, ls=":")
    ax.set_xlabel("変化してからの時間 [ms]")
    ax.set_ylabel(ylabel)
    result_title(ax, title)
    ax.legend(loc="lower right", fontsize=8.5)
    ax.grid(alpha=0.3)


def first_change(t, values, before):
    moved = np.abs(values - before) > 1e-9
    return t[np.argmax(moved)] if moved.any() else np.nan


def mark(ax, x, y, text, color):
    ax.annotate(text, (x, y), xytext=(8, 0), textcoords="offset points", fontsize=8.5, color=color, va="center")


def timing_gyro(args):
    a = run(args, "imu", "period", "period.json", "step", 1.05, t0=1.002)
    b = run(args, "imu", "period-delay", "period-delay.json", "step", 1.05, t0=1.002)
    w = (a["time"] >= 0.990) & (a["time"] <= 1.025)
    t = (a["time"][w] - 1.002) * 1000.0
    fig, left, right = card("周期と遅れ：ジャイロ（周期 4 ms、遅れ 10 ms）")
    timing_sketch(left, "1.002 秒に、x まわりの回転が 0 → 1 rad/s", 4, 10,
                  ["測る瞬間は 4 ms の倍数（1.000、1.004 秒…）。変化はその間",
                   "青：周期 4 ms だけ。緑：さらに遅れ 10 ms"])
    step_card(right, t, a["true_gx"][w], a["gx"][w], b["gx"][w], "ジャイロ x [rad/s]",
              "結果：次に測る瞬間まで待ち、遅れの分さらに待つ")
    ta, tb = first_change(t, a["gx"][w], 0.0), first_change(t, b["gx"][w], 0.0)
    mark(right, ta, 0.5, f"+{ta:.0f} ms（1.004 秒）", C1)
    mark(right, tb, 0.3, f"+{tb:.0f} ms（1.012 秒）", C3)
    save(fig, args, "card-timing-gyro")


def timing_mag(args):
    p = run(args, "mag", "period", "period.json", "yawstep", 1.2, t0=1.005)
    q = run(args, "mag", "period-delay", "period-delay.json", "yawstep", 1.2, t0=1.005)
    w = (p["time"] >= 0.95) & (p["time"] <= 1.15)
    t = (p["time"][w] - 1.005) * 1000
    fig, left, right = card("周期と遅れ：地磁気（周期 20 ms、遅れ 50 ms）")
    timing_sketch(left, "1.005 秒に、機首が一瞬で 90° 回る", 20, 50,
                  ["測る瞬間は 20 ms の倍数。変化はその間", "青：周期 20 ms だけ。緑：さらに遅れ 50 ms"])
    step_card(right, t, p["true_mx"][w] / 1000, p["mx"][w] / 1000, q["mx"][w] / 1000, "地磁気 x [µT]",
              "結果：次に測る瞬間まで待ち、遅れの分さらに待つ")
    before = p["mx"][w][0] / 1000
    ta, tb = first_change(t, p["mx"][w] / 1000, before), first_change(t, q["mx"][w] / 1000, before)
    mid = (before + p["mx"][w][-1] / 1000) / 2
    mark(right, ta, mid, f"+{ta:.0f} ms（1.020 秒）", C1)
    mark(right, tb, mid - 4, f"+{tb:.0f} ms（1.060 秒）", C3)
    save(fig, args, "card-timing-mag")


def timing_baro(args):
    p = run(args, "baro", "period", "period.json", "hstep", 1.2, t0=1.005)
    q = run(args, "baro", "period-delay", "period-delay.json", "hstep", 1.2, t0=1.005)
    w = (p["time"] >= 0.95) & (p["time"] <= 1.15)
    t = (p["time"][w] - 1.005) * 1000
    fig, left, right = card("周期と遅れ：気圧（周期 20 ms、遅れ 60 ms）")
    timing_sketch(left, "1.005 秒に、高さが一瞬で 10 m 上がる", 20, 60,
                  ["測る瞬間は 20 ms の倍数。変化はその間", "青：周期 20 ms だけ。緑：さらに遅れ 60 ms"])
    base = p["true_altitude"][w][0]
    step_card(right, t, p["true_altitude"][w] - base, p["pressure_altitude"][w] - base,
              q["pressure_altitude"][w] - base, "気圧から求めた高度 − 最初の高度 [m]",
              "結果：次に測る瞬間まで待ち、遅れの分さらに待つ")
    ta = first_change(t, p["pressure_altitude"][w] - base, p["pressure_altitude"][w][0] - base)
    tb = first_change(t, q["pressure_altitude"][w] - base, q["pressure_altitude"][w][0] - base)
    mark(right, ta, 5, f"+{ta:.0f} ms（1.020 秒）", C1)
    mark(right, tb, 3, f"+{tb:.0f} ms（1.080 秒）", C3)
    save(fig, args, "card-timing-baro")


def timing_gps(args):
    p = run(args, "gps", "period", "period.json", "move", 1.6, t0=1.05)
    q = run(args, "gps", "period-delay", "period-delay.json", "move", 1.6, t0=1.05)
    w = (p["time"] >= 0.95) & (p["time"] <= 1.6)
    np_, _ = north_east(p)
    nq, _ = north_east(q)
    fig, left, right = card("周期と遅れ：GPS（周期 100 ms、遅れ 100 ms）")
    timing_sketch(left, "1.05 秒に、北へ 5 m/s で動き出す", 100, 100,
                  ["測る瞬間は 100 ms の倍数（毎秒 10 回）", "青：周期 100 ms だけ。緑：さらに遅れ 100 ms"])
    t = (p["time"][w] - 1.05) * 1000
    right.plot(t, p["true_n"][w], color=MUTED, lw=2.5, label="真値")
    right.step(t, np_[w], where="post", color=C1, lw=2, label="周期だけ")
    right.step(t, nq[w], where="post", color=C3, lw=2, ls="--", label="周期＋遅れ")
    right.axvline(0, color=C2, lw=0.8, ls=":")
    right.set_xlabel("動き出してからの時間 [ms]")
    right.set_ylabel("北の位置 [m]（出発点から）")
    result_title(right, "結果：階段で追いかけ、遅れの分だけ後ろにいる")
    right.legend(loc="upper left", fontsize=8.5)
    right.grid(alpha=0.3)
    save(fig, args, "card-timing-gps")


# ------------------------------------------------------------------------------------------------ 6. resolution and range
def ramp_sketch(ax, text, lines):
    sketch_axes(ax, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    ax.text(0, 0.42, text, fontsize=10, color=INK, ha="center")
    airframe(ax, size=0.13, alpha=0.6, centre=(0, 0.17))
    notes(ax, lines, y=-0.06)


def resolution_gyro(args):
    slow = run(args, "imu", "resolution", "resolution-range.json", "ramp", 0.5, rate=0.01)
    fig, left, right = card("分解能：ジャイロ（0.061°/s 刻み）")
    ramp_sketch(left, "x まわりの回転を、ゆっくり増やす", ["回転の速さを毎秒 0.57°/s ずつ増やす（0.5 秒）",
                                                    "設定：分解能 0.061°/s（\"resolutionDegPerSec\"）",
                                                    "16 bit・±2000°/s のチップの刻み"])
    right.plot(slow["time"], slow["true_gx"] / DEG, color=MUTED, lw=2.5, label="真値（なめらか）")
    right.step(slow["time"], slow["gx"] / DEG, where="post", color=C2, lw=2, label="測った値（0.061 刻み）")
    for k in range(6):
        right.axhline(k * 0.061, color="#e4e9f1", lw=0.8, zorder=0)
    right.set_xlabel("時間 [s]")
    right.set_ylabel("ジャイロ x [°/s]")
    result_title(right, "結果：なめらかな変化が階段になる")
    right.legend(loc="upper left", fontsize=8.5)
    right.grid(alpha=0.3)
    save(fig, args, "card-resolution-gyro")


def range_gyro(args):
    fast = run(args, "imu", "range", "resolution-range.json", "ramp", 5.0, rate=10.0, every=10)
    fig, left, right = card("レンジ：ジャイロ（±2000°/s）")
    ramp_sketch(left, "x まわりの回転を、どんどん速くする", ["回転の速さを毎秒 573°/s ずつ増やす（5 秒）",
                                                     "設定：レンジ ±2000°/s（\"range.fullScaleDegPerSec\"）",
                                                     "超えると値は止まり、飽和のフラグが立つ"])
    right.plot(fast["time"], fast["true_gx"] / DEG, color=MUTED, lw=2.5, ls="--", label="真値")
    right.plot(fast["time"], fast["gx"] / DEG, color=C2, lw=2.2, label="測った値")
    right.axhline(2000, color=C2, lw=0.8, ls=":")
    sat = fast["gyro_flags"] > 0
    if sat.any():
        ts = fast["time"][sat][0]
        right.axvline(ts, color=C0, lw=0.8, ls=":")
        right.text(ts + 0.05, 600, f"{ts:.3f} 秒から\n飽和のフラグ", fontsize=9, color=C0)
    right.text(0.1, 2080, "上限 2000°/s", fontsize=9, color=C2)
    right.set_ylim(0, 3100)
    right.set_xlabel("時間 [s]")
    right.set_ylabel("ジャイロ x [°/s]")
    result_title(right, "結果：上限で止まり、超えた分は出ない")
    right.legend(loc="upper left", fontsize=8.5)
    right.grid(alpha=0.3)
    save(fig, args, "card-range-gyro")


def resolution_range_mag(args):
    r = run(args, "mag", "resolution", "resolution-range.json", "turn", 6.0, rate=0.002)
    fig, left, r1, r2 = card("分解能とレンジ：地磁気（15 nT 刻み、±0.3 G）", right=2)
    ramp_sketch(left, "とてもゆっくり回す（6 秒で 0.7°）", ["設定：分解能 15 nT、レンジ ±0.3 G（±30 µT）",
                                                     "東京の磁場は、下向きに約 35 µT",
                                                     "→ 下向き（z）はレンジを超えている"])
    r1.plot(r["time"], r["true_my"], color=MUTED, lw=2.5, label="真値")
    r1.step(r["time"], r["my"], where="post", color=C2, lw=2, label="測った値（15 nT 刻み）")
    r1.set_xlabel("時間 [s]")
    r1.set_ylabel("地磁気 y [nT]")
    result_title(r1, "分解能：階段になる")
    r1.legend(loc="upper left", fontsize=8.3)
    r1.grid(alpha=0.3)
    r2.plot(r["time"], r["true_mz"] / 1000, color=MUTED, lw=2.5, ls="--", label="真値（約 −35 µT）")
    r2.plot(r["time"], r["mz"] / 1000, color=C2, lw=2.2, label="測った値")
    r2.axhline(-30, color=C2, lw=0.8, ls=":")
    r2.text(0.1, -29.3, "下限 −30 µT", fontsize=9, color=C2)
    r2.set_ylim(-40, -25)
    r2.set_xlabel("時間 [s]")
    r2.set_ylabel("地磁気 z [µT]")
    result_title(r2, f"レンジ：ずっと切れている（フラグ {(r['flags'] > 0).mean() * 100:.0f} %）")
    r2.legend(loc="lower right", fontsize=8.3)
    r2.grid(alpha=0.3)
    save(fig, args, "card-resolution-range-mag")


def resolution_range_baro(args):
    r = run(args, "baro", "resolution", "resolution-range.json", "climb", 4.0, rate=0.05, t0=0.0)
    s = run(args, "baro", "range", "resolution-range.json", "climb", 6.0, rate=2000.0, t0=0.0, every=10)
    fig, left, r1, r2 = card("分解能とレンジ：気圧（0.5 Pa 刻み、300〜1100 hPa）", right=2)
    ramp_sketch(left, "上昇する", ["左のグラフ：毎秒 5 cm でゆっくり上昇（4 秒）", "右のグラフ：毎秒 2000 m で一気に上昇（6 秒）",
                                 "設定：分解能 0.5 Pa（高さで約 4 cm）、", "　　　レンジ 300〜1100 hPa"])
    r1.plot(r["time"], r["true_altitude"] - 40.0, color=MUTED, lw=2.5, label="真値（毎秒 5 cm）")
    r1.step(r["time"], r["pressure_altitude"] - 40.0, where="post", color=C2, lw=2, label="測った値（0.5 Pa 刻み）")
    r1.set_xlabel("時間 [s]")
    r1.set_ylabel("気圧から求めた高度 − 出発点 [m]")
    result_title(r1, "分解能：約 4 cm の階段")
    r1.legend(loc="upper left", fontsize=8.3)
    r1.grid(alpha=0.3)
    r2.plot(s["time"], s["true_altitude"] / 1000, color=MUTED, lw=2.5, ls="--", label="真値")
    r2.plot(s["time"], s["pressure_altitude"] / 1000, color=C2, lw=2.2, label="測った値")
    sat = s["flags"] > 0
    if sat.any():
        top = s["pressure_altitude"][sat][0]
        r2.axhline(top / 1000, color=C2, lw=0.8, ls=":")
        r2.text(0.1, top / 1000 + 0.3, f"300 hPa ＝ 高度 {top:.0f} m で止まる", fontsize=9, color=C2)
    r2.set_xlabel("時間 [s]")
    r2.set_ylabel("高度 [km]")
    result_title(r2, "レンジ：下限 300 hPa で切れる")
    r2.legend(loc="lower right", fontsize=8.3)
    r2.grid(alpha=0.3)
    save(fig, args, "card-resolution-range-baro")


if __name__ == "__main__":
    main(__doc__, [drift_gyro, drift_mag, drift_baro, drift_gps, timing_gyro, timing_mag, timing_baro, timing_gps,
                   resolution_gyro, range_gyro, resolution_range_mag, resolution_range_baro])
