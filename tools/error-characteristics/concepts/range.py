"""Concept figure 7: resolution and range (draft). Illustrative data made with numpy."""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
LANG = sys.argv[2] if len(sys.argv) > 2 else "ja"


def L(ja, en):
    return ja if LANG == "ja" else en

INK, MUTED, TRUTH, MEAS, BLUE, BAND = "#14213d", "#8a96a8", "0.45", "#d1242f", "#2f6fd6", "#fbe3e5"

fig = plt.figure(figsize=(16, 6.2))
grid = fig.add_gridspec(1, 3, width_ratios=[1, 1, 0.95], wspace=0.27)

# 1. resolution: values come in steps
ax = fig.add_subplot(grid[0])
t = np.linspace(0, 1, 1000)
truth = 0.05 + 0.32 * t + 0.03 * np.sin(2 * np.pi * 3 * t)
step = 0.061
ax.plot(t, truth, color=TRUTH, lw=2.5, label=L("真値（なめらか）", "truth (smooth)"))
ax.plot(t, np.round(truth / step) * step, color=MEAS, lw=2, label=L("出力：0.061 刻みに丸める", "output: rounded to steps of 0.061"))
for k in range(0, 8):
    ax.axhline(k * step, color="#eef1f6", lw=0.8, zorder=0)
ax.annotate("", xy=(1.05, 5 * step), xytext=(1.05, 6 * step), arrowprops=dict(arrowstyle="<->", color=MEAS))
ax.text(1.065, 5.5 * step, L("1 刻み", "1\nstep"), color=MEAS, fontsize=9, va="center")
ax.set_xlim(0, 1.17)
ax.set_ylim(0, 0.45)
ax.set_xlabel(L("時間 [s]", "Time [s]"))
ax.set_ylabel(L("角速度 [deg/s]", "Angular rate [deg/s]"))
ax.set_title(L("① 分解能：値が刻みでしか出ない", "① Resolution: values come only in steps"), fontsize=12.5, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.5)

# 2. range: clipped at the full scale
ax = fig.add_subplot(grid[1])
t = np.linspace(0, 2, 1000)
truth = 2500 * np.sin(2 * np.pi * 0.6 * t)
fs = 2000
ax.axhspan(-fs, fs, color="#f4f7fb", zorder=0)
ax.plot(t, truth, color=TRUTH, lw=2.5, ls="--", label=L("真値", "truth"))
ax.plot(t, np.clip(truth, -fs, fs), color=MEAS, lw=2.2, label=L("出力：±2000 で切れる", "output: clipped at ±2000"))
ax.axhline(fs, color=MEAS, lw=0.8, ls=":")
ax.axhline(-fs, color=MEAS, lw=0.8, ls=":")
ax.text(2.05, fs, "+2000", color=MEAS, fontsize=9, va="center")
ax.text(2.05, -fs, "−2000", color=MEAS, fontsize=9, va="center")
ax.text(1.25, 2350, L("超えた分は失われ、\n超えたことも伝わらない", "The excess is lost, and nothing\ntells that it was exceeded"), color=MEAS, fontsize=8.8, ha="center", va="bottom")
ax.set_xlim(0, 2.4)
ax.set_ylim(-3000, 3300)
ax.set_xlabel(L("時間 [s]", "Time [s]"))
ax.set_ylabel(L("角速度 [deg/s]", "Angular rate [deg/s]"))
ax.set_title(L("② レンジ：上限・下限で切れる", "② Range: clipped at the upper and lower limits"), fontsize=12.5, color=INK, loc="left")
ax.legend(loc="lower right", fontsize=8.5)

# 3. the trade-off of one chip, and the keys
ax = fig.add_subplot(grid[2])
ax.axis("off")
ax.set_title(L("③ 同じチップでも、レンジを広げると刻みが粗い", "③ Same chip: wider range, coarser steps"), fontsize=12.5, color=INK, loc="left")
rows = [[f"±{r}", f"{2 * r / 65536:.4f}"] for r in (250, 500, 1000, 2000)]
table = ax.table(cellText=rows, colLabels=[L("レンジ [deg/s]", "Range [deg/s]"), L("1 刻み [deg/s]", "1 step [deg/s]")], cellLoc="center",
                 bbox=[0.0, 0.62, 0.75, 0.33])
table.auto_set_font_size(False)
table.set_fontsize(9.5)
for (r, c), cell in table.get_celld().items():
    cell.set_edgecolor("#d6deea")
    if r == 0:
        cell.set_facecolor("#eaf0f9")
        cell.set_text_props(fontweight="bold", color=INK)
ax.text(0.0, 0.57, L("16 bit のジャイロ：刻み ＝ レンジの幅 ÷ 65536", "16-bit gyro: step = range width ÷ 65536"), fontsize=8.8, color=INK, va="top")
keys = [(L("センサー", "Sensor"), L("分解能", "Resolution"), L("レンジ", "Range")),
        (L("ジャイロ", "Gyro"), "resolutionDegPerSec", "range.fullScaleDegPerSec"),
        (L("加速度", "Accel"), "resolutionMps2", "range.fullScaleG"),
        (L("地磁気", "Mag"), "resolutionNT", "range.fullScaleGauss"),
        (L("気圧", "Baro"), "resolutionPa", "range.minHPa / maxHPa")]
table = ax.table(cellText=[list(k) for k in keys[1:]], colLabels=list(keys[0]), cellLoc="left",
                 colWidths=[0.16, 0.37, 0.47], bbox=[0.0, 0.05, 1.0, 0.42])
table.auto_set_font_size(False)
table.set_fontsize(8.2)
for (r, c), cell in table.get_celld().items():
    cell.set_edgecolor("#d6deea")
    if r == 0:
        cell.set_facecolor("#eaf0f9")
        cell.set_text_props(fontweight="bold", color=INK)
ax.text(0.0, -0.02, L("GPS にはこの 2 段はない", "GPS has neither of these two stages"), fontsize=8.5, color=MUTED, va="top")

fig.text(0.04, 0.03,
         L("この 2 段は最後に、この順で行う：分解能で丸めてから、レンジで切る。どちらも書かなければ何もしない。"
           "レンジはセンサーの軸ごとに切る（斜めに付けたセンサーは、機体の軸では範囲内でも 1 軸だけ切れることがある）。\n"
           "データシートの値をそのまま書く：ジャイロ ±2000 deg/s、加速度 ±16 g など。"
           "宙返りや激しい振動でレンジを超えると、オートパイロットは誤った値で姿勢を推定する。",
           "These two stages come last, in this order: round to the resolution, then clip at the range. "
           "If neither is set, nothing happens.\n"
           "The range is clipped per sensor axis (a sensor mounted at an angle can clip on one axis even when "
           "the vehicle axes are within range).\n"
           "Write the datasheet values as is: gyro ±2000 deg/s, accel ±16 g, etc. "
           "If a flip or heavy vibration exceeds the range, the autopilot estimates the attitude from wrong values."),
         fontsize=9, color=INK)
fig.subplots_adjust(left=0.05, right=0.97, top=0.9, bottom=0.2)
fig.savefig(sys.argv[1], dpi=110)
print("wrote", sys.argv[1])
