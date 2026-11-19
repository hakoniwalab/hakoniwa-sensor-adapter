"""Force-torque concept figure 6: bias and drift, saturation and overload, resolution (draft, Japanese/English).
Illustrative data; the rules are docs/spec/force-torque.md FTS-ADP-110 and 240 to 290."""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
LANG = sys.argv[2] if len(sys.argv) > 2 else "ja"


def L(ja, en):
    return ja if LANG == "ja" else en

INK, MUTED, TRUTH, MEAS, BLUE, GREEN, GRAY = "#14213d", "#8a96a8", "0.55", "#d1242f", "#2f6fd6", "#1a7f37", "#5b6475"
fig, axes = plt.subplots(1, 3, figsize=(16, 5.6))

# 1. bias and drift
ax = axes[0]
T = np.arange(0, 3 * 3600, 2.0)                                # three hours, a sample every 2 s for the picture
for seed, (bias, color) in enumerate(((0.6, MEAS), (-0.4, BLUE), (0.1, GREEN))):
    walk = np.cumsum(np.random.default_rng(seed + 10).normal(0, 0.5 * np.sqrt(2.0 / 3600), T.size))
    ax.plot(T / 3600, bias + walk, color=color, lw=1.2, label=L(f"個体 {seed + 1}：バイアス {bias:+.1f} N ＋ ドリフト", f"unit {seed + 1}: bias {bias:+.1f} N + drift"))
    ax.plot(T / 3600, np.full(T.size, bias), color=color, lw=0.8, ls=":")
ax.axhline(0, color=TRUTH, lw=2)
ax.set_xlabel(L("時間 [h]", "Time [h]"))
ax.set_ylabel(L("Fz − 真値 [N]（何も触れていない）", "Fz − truth [N] (no contact)"))
ax.set_title(L("① バイアスとドリフト", "① Bias and drift"), fontsize=12, color=INK, loc="left")
ax.plot([], [], color=INK, lw=0.8, ls=":", label=L("点線：その個体のバイアス（一定）", "dotted: that unit's bias (constant)"))
ax.set_ylim(-2, 2.2)
ax.legend(loc="upper left", fontsize=8)
ax.grid(alpha=0.3)

# 2. saturation and overload
ax = axes[1]
t = np.linspace(0, 1, 500)
R, O = 200.0, 1000.0                                           # range 200 N, overload 500 %
truth = 1300 * t
out = np.clip(truth, -R, R)
ax.plot(t, truth, color=TRUTH, lw=2.5, ls="--", label=L("真値（力がどんどん大きくなる）", "truth (force keeps rising)"))
ax.plot(t, out, color=MEAS, lw=2.5, label=L("出力", "output"))
ax.axhline(R, color=MEAS, lw=0.8, ls=":")
ax.axhline(O, color=GRAY, lw=0.8, ls=":")
ts, to = R / 1300, O / 1300
ax.axvspan(ts, to, color="#fbe3e5", zorder=0)
ax.axvspan(to, 1, color="#e4e7ec", zorder=0)
ax.text(0.3, R + 25, L("レンジ 200 N（\"range\"）", "range 200 N (\"range\")"), fontsize=9, color=MEAS)
ax.text(0.02, O + 25, L("過負荷 1000 N（\"overload\" 500 %）", "overload 1000 N (\"overload\" 500 %)"), fontsize=9, color=GRAY)
ax.text((ts + to) / 2, 1080, L("飽和：\n値は 200 N で止まり\n「飽和」の印", "Saturation:\nvalue stops at 200 N,\nsaturation flag"), fontsize=9, color=MEAS, ha="center")
ax.text((to + 1) / 2, 330, L("過負荷：\n実物なら\n壊れるおそれ\n「過負荷」の印", "Overload:\na real sensor\nmay break,\noverload flag"), fontsize=9, color=GRAY, ha="center")
ax.set_ylim(0, 1400)
ax.set_xlabel(L("時間 [s]", "Time [s]"))
ax.set_ylabel("Fz [N]")
ax.set_title(L("② 飽和と過負荷：測れない力と、壊れる力", "② Saturation and overload"), fontsize=12, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.3)
ax.grid(alpha=0.3)

# 3. resolution
ax = axes[2]
t = np.linspace(0, 1, 600)
truth = 0.3 + 2.0 * t + 0.25 * np.sin(2 * np.pi * 2 * t)
q = 0.2
ax.plot(t, truth, color=TRUTH, lw=2.5, label=L("真値（なめらか）", "truth (smooth)"))
ax.step(t, q * np.round(truth / q), where="post", color=MEAS, lw=2, label=L("出力（0.2 N 刻み）", "output (0.2 N steps)"))
for k in range(0, 14):
    ax.axhline(k * q, color="#eef1f6", lw=0.8, zorder=0)
ax.set_xlabel(L("時間 [s]", "Time [s]"))
ax.set_ylabel("Fz [N]")
ax.set_title(L("③ 分解能：値は刻みでしか出ない", "③ Resolution: values come in steps"), fontsize=12, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.5)
ax.text(0.45, 0.35, L("\"resolution\"：軸ごとの刻み\n（風袋引きの後に丸める）", "\"resolution\": step per axis\n(rounded after tare)"), fontsize=9, color=INK)
ax.grid(alpha=0.3)

fig.text(0.04, 0.02,
         L("バイアスは個体ごとに一定（\"bias\"）。ドリフトは測るたびに少しずつ足されて広がり続ける（\"drift_per_hour\"、1 時間あたりの σ）。"
           "ドローンのドリフト（帯の中をさまよう）とは違い、上限なく広がる。\n"
           "過負荷の印は、真値が \"overload\" を超えた時点で立つ（遅れ・フィルタの前）。飽和は出力をレンジで切る。どちらも値と一緒に印（フラグ）で知らせる。",
           "Bias is constant per unit (\"bias\"). Drift adds a little at each sample and keeps spreading (\"drift_per_hour\", σ per hour). "
           "Unlike the drone drift (which wanders within a band), it has no limit.\n"
           "The overload flag is set when the truth exceeds \"overload\" (before delay and filter). Saturation clips the output to the range. "
           "Both come with the value as a flag."),
         fontsize=9, color=INK)
fig.subplots_adjust(left=0.05, right=0.98, top=0.9, bottom=0.2, wspace=0.27)
fig.savefig(sys.argv[1], dpi=110)
print("wrote", sys.argv[1])
