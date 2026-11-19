"""Concept figure 6: update period and delay (draft). Illustrative data made with numpy."""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
LANG = sys.argv[2] if len(sys.argv) > 2 else "ja"


def L(ja, en):
    return ja if LANG == "ja" else en

INK, MUTED, TRUTH, MEAS, BLUE, GREEN = "#14213d", "#8a96a8", "0.45", "#d1242f", "#2f6fd6", "#1a7f37"

fig = plt.figure(figsize=(16, 6.4))
grid = fig.add_gridspec(1, 3, width_ratios=[1, 1, 1.0], wspace=0.22)


def held(t_ms, values_at, period_ms, delay_ms=0.0):
    """Sample-and-hold of a delayed signal: a new value only when t is a multiple of the period."""
    t_sample = np.floor(t_ms / period_ms) * period_ms
    return values_at(t_sample - delay_ms)


# 1. sampling: a new value each period, held in between
ax = fig.add_subplot(grid[0])
t = np.arange(0, 200, 0.1)
truth = lambda x: np.sin(2 * np.pi * np.clip(x, 0, None) / 160)
ax.plot(t, truth(t), color=TRUTH, lw=2.5, label=L("真値（物理の毎ステップ）", "truth (every physics step)"))
ax.plot(t, held(t, truth, 20), color=MEAS, lw=2, label=L("測った値（周期 20 ms）", "measured (period 20 ms)"))
ts = np.arange(0, 200, 20)
ax.plot(ts, truth(ts), "o", color=MEAS, ms=6)
ax.set_xlim(0, 200)
ax.set_ylim(-1.5, 1.8)
ax.set_xlabel(L("時間 [ms]", "Time [ms]"))
ax.set_ylabel(L("値", "Value"))
ax.set_title(L("① 更新周期：周期ごとに測り、次まで同じ値", "① Update period: sample, then hold until the next"), fontsize=12.5, color=INK, loc="left")
ax.legend(loc="upper right", fontsize=8.5)
ax.annotate("", xy=(100, -1.25), xytext=(120, -1.25), arrowprops=dict(arrowstyle="<->", color=MEAS))
ax.text(110, -1.38, L("周期", "period"), color=MEAS, fontsize=9, ha="center", va="top")

# 2. delay: a step input, delay then the next sample boundary
ax = fig.add_subplot(grid[1])
t = np.arange(0, 60, 0.05)
t0, delay, period = 2.0, 10.0, 10.0
step = lambda x: (x >= t0).astype(float)
ax.plot(t, step(t), color=TRUTH, lw=2.5, label=L("真値（2 ms で急に変わる）", "truth (steps at 2 ms)"))
ax.plot(t, step(t - delay), color=BLUE, lw=1.6, ls="--", label=L("遅れ 10 ms だけ", "delay 10 ms only"))
out = held(t, step, period, delay)
ax.plot(t, out, color=MEAS, lw=2.2, label=L("遅れ 10 ms ＋ 周期 10 ms", "delay 10 ms + period 10 ms"))
for b in np.arange(0, 60, period):
    ax.axvline(b, color="#e4e9f1", lw=0.8, zorder=0)
t_out = np.ceil((t0 + delay) / period) * period
ax.annotate("", xy=(t0, 1.18), xytext=(t0 + delay, 1.18), arrowprops=dict(arrowstyle="<->", color=BLUE))
ax.text(t0 + delay / 2, 1.24, L("遅れ", "delay"), color=BLUE, fontsize=9, ha="center", va="bottom")
ax.annotate("", xy=(t0 + delay, 0.5), xytext=(t_out, 0.5), arrowprops=dict(arrowstyle="<->", color=MEAS))
ax.text(t_out + 1, 0.5, L("次の測る瞬間\nまで待つ", "waits for\nthe next\nsample"), color=MEAS, fontsize=8.5, va="center")
ax.set_xlim(0, 60)
ax.set_ylim(-0.25, 1.6)
ax.set_xlabel(L("時間 [ms]（縦の細線：測る瞬間）", "Time [ms] (thin vertical lines: sampling instants)"))
ax.set_ylabel(L("値", "Value"))
ax.set_title(L("② 遅れ：ずらしてから、周期で測る", "② Delay: shift, then sample each period"), fontsize=12.5, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.3)
ax.text(0.5, -0.2, L("見かけの遅れは「遅れ」〜「遅れ＋周期」の間", "Apparent delay: between delay and delay + period"), fontsize=8.8, color=INK, va="bottom")

# 3. where the delays sit, and who must know them
ax = fig.add_subplot(grid[2])
ax.axis("off")
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.set_title(L("③ 2 つの遅れと、オートパイロットへの伝え方", "③ Two delays, and telling the autopilot"), fontsize=12.5, color=INK, loc="left")
boxes = [(0.1, L("物理\n（真値）", "Physics\n(truth)"), TRUTH), (2.6, L("センサー\nの中", "In the\nsensor"), MEAS),
         (5.1, L("受け取り\n・処理", "Receive,\nprocess"), BLUE), (7.6, L("制御・\n推定で使う", "Control,\nestimation"), GREEN)]
for x, label, color in boxes:
    ax.add_patch(FancyBboxPatch((x, 6.6), 1.9, 1.6, boxstyle="round,pad=0.05,rounding_size=0.15", fc=color,
                                ec="none", alpha=0.9))
    ax.text(x + 0.95, 7.4, label, ha="center", va="center", color="white", fontsize=9.5 if LANG == "ja" else 9,
            fontweight="bold")
for x in (2.08, 4.58, 7.08):
    ax.add_patch(FancyArrowPatch((x, 7.4), (x + 0.4, 7.4), arrowstyle="-|>", mutation_scale=12, color=INK))
ax.text(3.6, 6.3, "delayMsec\nupdatePeriodMsec", ha="center", va="top", fontsize=7.5, color=MEAS,
        family="monospace")
ax.text(3.6, 5.2, L("センサーの遅れ\n（センサーPRO）", "Sensor delay\n(Sensor PRO)"), ha="center", va="top", fontsize=8.3, color=MEAS)
ax.text(6.1, 6.3, "softDelayMsec", ha="center", va="top", fontsize=7.5, color=BLUE, family="monospace")
ax.text(6.1, 5.2, L("受け渡しの遅れ\n（ホスト）", "Transfer delay\n(host)"), ha="center", va="top", fontsize=8.3, color=BLUE)
ax.text(0.1, 3.0,
        L("推定器（EKF）は「何 ms 前の値か」を知って合わせる。\n"
          "ホストは delayMsec と同じ値を、オートパイロットの遅れの設定に書く\n"
          "（例：気圧 EKF2_BARO_DELAY、GPS SENS_GPS0_DELAY）。\n"
          "設定と実際が合っていないと、推定がずれる。",
          "The estimator (EKF) aligns each value by knowing\nhow many ms old it is.\n"
          "The host writes the same value as delayMsec into\nthe autopilot's delay parameters\n"
          "(e.g. baro EKF2_BARO_DELAY, GPS SENS_GPS0_DELAY).\n"
          "If the setting and the real delay differ,\nthe estimate is off."),
        fontsize=8.8, color=INK, va="top", wrap=True)

fig.text(0.04, 0.03,
         L("遅れは物理の刻み（例 1 ms）の整数倍に丸める。測る瞬間は時刻 0 から周期の倍数（t ÷ 周期 が割り切れるとき）。"
           "制御もログも、同じ保持された値を読む。\n"
           "5 センサーとも同じキー：\"delayMsec\"（遅れ）、\"updatePeriodMsec\"（周期）。"
           "例：ジャイロ・加速度 1〜4 ms、地磁気・気圧 10〜50 ms、GPS 100〜200 ms",
           "The delay is rounded to a whole number of physics steps (e.g. 1 ms). Samples are taken at multiples of the "
           "period from time 0 (when t ÷ period divides evenly). Control and logs read the same held value.\n"
           "Same keys in all 5 sensors: \"delayMsec\" (delay), \"updatePeriodMsec\" (period). "
           "E.g. gyro, accel 1–4 ms; mag, baro 10–50 ms; GPS 100–200 ms"),
         fontsize=9, color=INK)
fig.subplots_adjust(left=0.05, right=0.97, top=0.9, bottom=0.2)
fig.savefig(sys.argv[1], dpi=110)
print("wrote", sys.argv[1])
