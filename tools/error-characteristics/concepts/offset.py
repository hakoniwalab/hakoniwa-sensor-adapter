"""Concept figure 4: per-unit zero offset, sensitivity, and the magnetometer's iron (draft).
Illustrative data made with numpy."""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
LANG = sys.argv[2] if len(sys.argv) > 2 else "ja"


def L(ja, en):
    return ja if LANG == "ja" else en

INK, MUTED, TRUTH, MEAS = "#14213d", "#8a96a8", "0.45", "#d1242f"
UNITS = ["#d1242f", "#2f6fd6", "#1a7f37", "#b26a00"]
rng = np.random.default_rng(5)

fig, axes = plt.subplots(1, 3, figsize=(16, 6.0))

# 1. zero offset per unit
ax = axes[0]
t = np.arange(0, 10, 0.05)
offsets = [0.8, -0.5, 0.25, -1.1]
ax.axhline(0, color=TRUTH, lw=2.5, label=L("真値（静止：0）", "truth (at rest: 0)"))
for k, (b, color) in enumerate(zip(offsets, UNITS)):
    ax.plot(t, b + rng.normal(0, 0.08, t.size), color=color, lw=0.9)
    ax.text(10.2, b, L(f"個体 {k + 1}", f"unit {k + 1}"), color=color, fontsize=9, va="center")
ax.set_xlim(0, 11.5)
ax.set_ylim(-2, 2)
ax.set_xlabel(L("時間 [s]", "Time [s]"))
ax.set_ylabel(L("測った値", "Measured"))
ax.set_title(L("① 零点のずれ：個体ごとに一定のずれ", "① Zero offset: a fixed offset per unit"), fontsize=12.5, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.5)
ax.annotate("", xy=(1.0, offsets[3]), xytext=(1.0, 0), arrowprops=dict(arrowstyle="<->", color=UNITS[3]))
ax.text(1.15, -0.78, L("ずれ", "offset"), color=UNITS[3], fontsize=9.5, va="center", fontweight="bold",
        bbox=dict(fc="white", ec="none", pad=1.5), zorder=5)

# 2. sensitivity: scale and cross-axis
ax = axes[1]
x = np.linspace(-1, 1, 50)
ax.plot(x, x, color=TRUTH, lw=2.5, label=L("理想：出力＝入力", "ideal: output = input"))
ax.plot(x, 1.15 * x, color=MEAS, lw=2, label=L("スケール誤差 +15 %：傾きが 1 からずれる", "scale error +15 %: slope is not 1"))
ax.plot(x, 0.12 * x, color="#2f6fd6", lw=2, ls="--", label=L("他軸感度 12 %：x の入力が y に漏れる", "cross-axis 12 %: x input leaks into y"))
ax.axhline(0, color="#e4e9f1", lw=0.8, zorder=0)
ax.axvline(0, color="#e4e9f1", lw=0.8, zorder=0)
ax.set_xlim(-1, 1)
ax.set_ylim(-1.25, 1.25)
ax.set_aspect("equal")
ax.set_xticks([-1, -0.5, 0, 0.5, 1])
ax.set_xlabel(L("入力（x 軸の真値）", "Input (x-axis truth)"))
ax.set_ylabel(L("出力", "Output"))
ax.set_title(L("② 感度：倍率がずれ、軸が混ざる", "② Sensitivity: scale is off, axes mix"), fontsize=12.5, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.3)

# 3. magnetometer: hard iron shifts, soft iron squashes
ax = axes[2]
a = np.linspace(0, 2 * np.pi, 200)
circle = np.vstack([np.cos(a), np.sin(a)])
soft = np.array([[1.25, 0.15], [0.15, 0.8]])
hard = np.array([0.45, -0.3])
ax.plot(*circle, color=TRUTH, lw=2.5, label=L("真値：一回転すると円", "truth: a circle over one turn"))
ax.plot(*(circle + hard[:, None]), color="#2f6fd6", lw=2, ls="--", label=L("ハードアイアン：円がずれる（ずれ）", "hard iron: circle shifts (offset)"))
ax.plot(*(soft @ circle + hard[:, None]), color=MEAS, lw=2, label=L("＋ソフトアイアン：潰れる（感度）", "+ soft iron: squashed (sensitivity)"))
ax.plot(0, 0, "+", color=TRUTH, ms=12, mew=2)
ax.plot(*hard, "+", color=MEAS, ms=12, mew=2)
ax.set_aspect("equal")
ax.set_xlim(-1.7, 2.1)
ax.set_ylim(-1.6, 1.9)
ax.set_xlabel(L("磁場 x", "Field x"))
ax.set_ylabel(L("磁場 y", "Field y"))
ax.set_title(L("③ 地磁気の鉄の影響（ずれ＋感度）", "③ Magnetometer iron (offset + sensitivity)"), fontsize=12.5, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.3)

fig.text(0.04, 0.03,
         L("零点のずれ：電源を入れたときに 1 つ決まり、動いている間は変わらない（ゆっくり変わるのは⑤のドリフト）。"
           "乱数で決める \"randomStd…\"（個体差の σ）か、固定値 \"fixed…\"。\n"
           "キー：ジャイロ zeroRateOffset　加速度 zeroGOffset　気圧 zeroOffset　地磁気 hardIronNT（ずれ）・softIron（感度）　"
           "感度は sensitivity（scaleErrorPct・crossAxisPct）　GPS にはない",
           "Zero offset: set once at power-on and constant while running (the slow change is ⑤ Drift). "
           "Random \"randomStd…\" (σ of the unit-to-unit spread) or fixed \"fixed…\".\n"
           "Keys: gyro zeroRateOffset   accel zeroGOffset   baro zeroOffset   mag hardIronNT (offset), softIron (sensitivity)   "
           "sensitivity: sensitivity (scaleErrorPct, crossAxisPct)   none for GPS"),
         fontsize=9, color=INK)
fig.subplots_adjust(left=0.05, right=0.98, top=0.9, bottom=0.2, wspace=0.28)
fig.savefig(sys.argv[1], dpi=110)
print("wrote", sys.argv[1])
